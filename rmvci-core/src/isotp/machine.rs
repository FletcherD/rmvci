//! Sans-IO ISO 15765-2 transmit/receive state machines. Frames and clock
//! readings go in, frames and events come out; no I/O, no sleeping — which
//! is what lets proptest drive a TxMachine against an RxMachine through
//! every BS/STmin combination in microseconds.
//!
//! These exist because the cable firmware's own ISO-TP transmit path is
//! defective (ignores BS and STmin, truncates FF_DL to 8 bits); connecting
//! as raw CAN and running these on the host is the correct-by-construction
//! alternative.

use std::time::{Duration, Instant};

use crate::error::IsoTpError;

/// Largest classic ISO-TP payload (12-bit FF_DL).
pub const MAX_PAYLOAD: usize = 4095;

/// Decode an STmin byte per ISO 15765-2: 0x00–0x7F are milliseconds,
/// 0xF1–0xF9 are 100–900 µs (rounded up to 1 ms here — we cannot pace finer
/// over a serial link anyway), and reserved values mean "use the maximum",
/// 0x7F ms.
pub fn decode_stmin(b: u8) -> Duration {
    match b {
        0x00..=0x7f => Duration::from_millis(b as u64),
        0xf1..=0xf9 => Duration::from_millis(1),
        _ => Duration::from_millis(0x7f),
    }
}

fn pad(mut frame: Vec<u8>, padding: Option<u8>) -> Vec<u8> {
    if let Some(p) = padding {
        frame.resize(8, p);
    }
    frame
}

/// What the transmit driver must do next.
#[derive(Debug, PartialEq, Eq)]
pub enum TxAction {
    /// Put this frame on the bus now.
    Send(Vec<u8>),
    /// Wait for a flow-control frame until the deadline, feeding candidate
    /// frames through [`TxMachine::on_frame`].
    WaitFc { deadline: Instant },
    /// STmin pacing: do nothing until this instant.
    WaitUntil(Instant),
    Done,
}

enum TxState {
    SingleFrame,
    FirstFrame,
    AwaitFc { deadline: Instant },
    Consecutive { earliest: Instant },
    Finished,
}

pub struct TxMachine {
    payload: Vec<u8>,
    offset: usize,
    sn: u8,
    /// Block size granted by the last FC (0 = unlimited).
    bs: u8,
    cfs_since_fc: u8,
    stmin: Duration,
    wait_count: u8,
    wft_max: u8,
    n_bs: Duration,
    padding: Option<u8>,
    state: TxState,
}

impl TxMachine {
    pub fn new(
        payload: &[u8],
        padding: Option<u8>,
        n_bs: Duration,
        wft_max: u8,
    ) -> Result<Self, IsoTpError> {
        if payload.is_empty() {
            return Err(IsoTpError::Malformed("empty payload"));
        }
        if payload.len() > MAX_PAYLOAD {
            return Err(IsoTpError::PayloadTooLong(payload.len()));
        }
        let state = if payload.len() <= 7 { TxState::SingleFrame } else { TxState::FirstFrame };
        Ok(Self {
            payload: payload.to_vec(),
            offset: 0,
            sn: 0,
            bs: 0,
            cfs_since_fc: 0,
            stmin: Duration::ZERO,
            wait_count: 0,
            wft_max,
            n_bs,
            padding,
            state,
        })
    }

    pub fn next(&mut self, now: Instant) -> Result<TxAction, IsoTpError> {
        match self.state {
            TxState::SingleFrame => {
                let mut f = Vec::with_capacity(8);
                f.push(self.payload.len() as u8);
                f.extend_from_slice(&self.payload);
                self.state = TxState::Finished;
                Ok(TxAction::Send(pad(f, self.padding)))
            }
            TxState::FirstFrame => {
                let len = self.payload.len();
                let mut f = Vec::with_capacity(8);
                f.push(0x10 | (len >> 8) as u8);
                f.push(len as u8);
                f.extend_from_slice(&self.payload[..6]);
                self.offset = 6;
                self.sn = 1;
                self.state = TxState::AwaitFc { deadline: now + self.n_bs };
                Ok(TxAction::Send(f)) // FF is always full-length
            }
            TxState::AwaitFc { deadline } => {
                if now >= deadline {
                    Err(IsoTpError::FcTimeout)
                } else {
                    Ok(TxAction::WaitFc { deadline })
                }
            }
            TxState::Consecutive { earliest } => {
                if now < earliest {
                    return Ok(TxAction::WaitUntil(earliest));
                }
                let chunk = (self.payload.len() - self.offset).min(7);
                let mut f = Vec::with_capacity(8);
                f.push(0x20 | self.sn);
                f.extend_from_slice(&self.payload[self.offset..self.offset + chunk]);
                self.offset += chunk;
                self.sn = (self.sn + 1) & 0x0f;

                if self.offset >= self.payload.len() {
                    self.state = TxState::Finished;
                } else {
                    self.cfs_since_fc += 1;
                    if self.bs != 0 && self.cfs_since_fc >= self.bs {
                        // Block exhausted: the ECU owes us another FC. (The
                        // firmware's ISO-TP never re-waits here — that is its
                        // "~48-byte limit" bug.)
                        self.state = TxState::AwaitFc { deadline: now + self.n_bs };
                    } else {
                        self.state = TxState::Consecutive { earliest: now + self.stmin };
                    }
                }
                Ok(TxAction::Send(pad(f, self.padding)))
            }
            TxState::Finished => Ok(TxAction::Done),
        }
    }

    /// Feed a frame received on the response identifier. Non-FC frames and
    /// FCs arriving outside an FC wait are ignored.
    pub fn on_frame(&mut self, data: &[u8], now: Instant) -> Result<(), IsoTpError> {
        if !matches!(self.state, TxState::AwaitFc { .. }) {
            return Ok(());
        }
        if data.is_empty() || data[0] >> 4 != 0x3 {
            return Ok(());
        }
        if data.len() < 3 {
            return Err(IsoTpError::Malformed("flow control shorter than 3 bytes"));
        }
        match data[0] & 0x0f {
            0 => {
                // CTS
                self.bs = data[1];
                self.stmin = decode_stmin(data[2]);
                self.cfs_since_fc = 0;
                self.wait_count = 0;
                // STmin paces the gap *between* CFs; the first one of a
                // block may go immediately.
                self.state = TxState::Consecutive { earliest: now };
                Ok(())
            }
            1 => {
                // WAIT — honored up to wft_max times (the firmware treats
                // this as CTS, another of its defects).
                self.wait_count += 1;
                if self.wait_count > self.wft_max {
                    return Err(IsoTpError::WaitLimit(self.wft_max));
                }
                self.state = TxState::AwaitFc { deadline: now + self.n_bs };
                Ok(())
            }
            2 => Err(IsoTpError::Overflow),
            _ => Err(IsoTpError::Malformed("reserved flow status")),
        }
    }
}

/// What the receive driver must do after feeding a frame.
#[derive(Debug, PartialEq, Eq)]
pub enum RxEvent {
    /// Put this flow-control frame on the bus now.
    SendFc(Vec<u8>),
    /// Reassembly complete.
    Done(Vec<u8>),
    /// Keep feeding frames.
    Continue,
}

enum RxState {
    Idle,
    Receiving { expected_sn: u8, cfs_since_fc: u8, deadline: Instant },
}

pub struct RxMachine {
    buf: Vec<u8>,
    ff_dl: usize,
    /// BS we advertise in our FC (0 = send everything).
    rx_bs: u8,
    /// STmin byte we advertise.
    rx_stmin: u8,
    padding: Option<u8>,
    n_cr: Duration,
    state: RxState,
}

impl RxMachine {
    pub fn new(rx_bs: u8, rx_stmin: u8, padding: Option<u8>, n_cr: Duration) -> Self {
        Self { buf: Vec::new(), ff_dl: 0, rx_bs, rx_stmin, padding, n_cr, state: RxState::Idle }
    }

    fn fc(&self) -> Vec<u8> {
        pad(vec![0x30, self.rx_bs, self.rx_stmin], self.padding)
    }

    /// N_Cr enforcement: call periodically while waiting for CFs.
    pub fn check_timeout(&self, now: Instant) -> Result<(), IsoTpError> {
        if let RxState::Receiving { deadline, .. } = self.state
            && now > deadline
        {
            return Err(IsoTpError::CfTimeout);
        }
        Ok(())
    }

    pub fn on_frame(&mut self, data: &[u8], now: Instant) -> Result<RxEvent, IsoTpError> {
        if data.is_empty() {
            return Err(IsoTpError::Malformed("empty frame"));
        }
        match data[0] >> 4 {
            0x0 => {
                let len = (data[0] & 0x0f) as usize;
                if len == 0 || 1 + len > data.len() {
                    return Err(IsoTpError::Malformed("bad single-frame length"));
                }
                self.state = RxState::Idle;
                Ok(RxEvent::Done(data[1..1 + len].to_vec()))
            }
            0x1 => {
                if data.len() < 8 {
                    return Err(IsoTpError::Malformed("short first frame"));
                }
                let ff_dl = ((data[0] & 0x0f) as usize) << 8 | data[1] as usize;
                if ff_dl <= 7 {
                    return Err(IsoTpError::Malformed("FF_DL fits in a single frame"));
                }
                self.ff_dl = ff_dl;
                self.buf.clear();
                self.buf.extend_from_slice(&data[2..8]);
                self.state = RxState::Receiving {
                    expected_sn: 1,
                    cfs_since_fc: 0,
                    deadline: now + self.n_cr,
                };
                Ok(RxEvent::SendFc(self.fc()))
            }
            0x2 => {
                let RxState::Receiving { expected_sn, cfs_since_fc, .. } = self.state else {
                    return Err(IsoTpError::Malformed("consecutive frame without first frame"));
                };
                let sn = data[0] & 0x0f;
                if sn != expected_sn {
                    self.state = RxState::Idle;
                    return Err(IsoTpError::SequenceError { expected: expected_sn, got: sn });
                }
                let need = self.ff_dl - self.buf.len();
                let take = need.min(7).min(data.len() - 1);
                self.buf.extend_from_slice(&data[1..1 + take]);

                if self.buf.len() >= self.ff_dl {
                    self.state = RxState::Idle;
                    let mut out = std::mem::take(&mut self.buf);
                    out.truncate(self.ff_dl);
                    return Ok(RxEvent::Done(out));
                }
                let cfs = cfs_since_fc + 1;
                if self.rx_bs != 0 && cfs >= self.rx_bs {
                    self.state = RxState::Receiving {
                        expected_sn: (sn + 1) & 0x0f,
                        cfs_since_fc: 0,
                        deadline: now + self.n_cr,
                    };
                    Ok(RxEvent::SendFc(self.fc()))
                } else {
                    self.state = RxState::Receiving {
                        expected_sn: (sn + 1) & 0x0f,
                        cfs_since_fc: cfs,
                        deadline: now + self.n_cr,
                    };
                    Ok(RxEvent::Continue)
                }
            }
            // A flow-control frame is the sender's business, not ours.
            0x3 => Ok(RxEvent::Continue),
            _ => Err(IsoTpError::Malformed("unknown PCI type")),
        }
    }
}
