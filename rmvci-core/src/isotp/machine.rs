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

/// Start a frame, emitting the extended-addressing byte first when mixed/
/// extended addressing is in use. Every ISO-TP frame then carries one fewer
/// data byte (SF ≤ 6, FF 5, CF 6).
fn start_frame(addr: Option<u8>) -> Vec<u8> {
    let mut f = Vec::with_capacity(8);
    if let Some(a) = addr {
        f.push(a);
    }
    f
}

/// What the transmit driver must do next.
#[derive(Debug, PartialEq, Eq)]
pub enum TxAction {
    /// Put this frame on the bus now.
    Send(Vec<u8>),
    /// Wait for a flow-control frame until the deadline, feeding candidate
    /// frames through [`TxMachine::on_frame`].
    WaitFc {
        deadline: Instant,
    },
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
    /// Extended/mixed-addressing byte, prepended to every frame when set.
    addr: Option<u8>,
    state: TxState,
}

impl TxMachine {
    pub fn new(
        payload: &[u8],
        padding: Option<u8>,
        n_bs: Duration,
        wft_max: u8,
    ) -> Result<Self, IsoTpError> {
        Self::with_addr(payload, padding, n_bs, wft_max, None)
    }

    pub fn with_addr(
        payload: &[u8],
        padding: Option<u8>,
        n_bs: Duration,
        wft_max: u8,
        addr: Option<u8>,
    ) -> Result<Self, IsoTpError> {
        if payload.is_empty() {
            return Err(IsoTpError::Malformed("empty payload"));
        }
        if payload.len() > MAX_PAYLOAD {
            return Err(IsoTpError::PayloadTooLong(payload.len()));
        }
        // The address byte costs one data byte, so a single frame holds ≤ 6.
        let sf_max = 7 - addr.is_some() as usize;
        let state =
            if payload.len() <= sf_max { TxState::SingleFrame } else { TxState::FirstFrame };
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
            addr,
            state,
        })
    }

    pub fn next(&mut self, now: Instant) -> Result<TxAction, IsoTpError> {
        match self.state {
            TxState::SingleFrame => {
                let mut f = start_frame(self.addr);
                f.push(self.payload.len() as u8);
                f.extend_from_slice(&self.payload);
                self.state = TxState::Finished;
                Ok(TxAction::Send(pad(f, self.padding)))
            }
            TxState::FirstFrame => {
                let len = self.payload.len();
                let first = 6 - self.addr.is_some() as usize; // FF data bytes
                let mut f = start_frame(self.addr);
                f.push(0x10 | (len >> 8) as u8);
                f.push(len as u8);
                f.extend_from_slice(&self.payload[..first]);
                self.offset = first;
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
                let cf_max = 7 - self.addr.is_some() as usize; // CF data bytes
                let chunk = (self.payload.len() - self.offset).min(cf_max);
                let mut f = start_frame(self.addr);
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
        // In extended addressing the FC frame also leads with the address byte;
        // ignore frames for a different address, then read the PCI past it.
        let h = self.addr.is_some() as usize;
        if let Some(a) = self.addr
            && data.first() != Some(&a)
        {
            return Ok(());
        }
        if data.len() < h + 1 || data[h] >> 4 != 0x3 {
            return Ok(());
        }
        if data.len() < h + 3 {
            return Err(IsoTpError::Malformed("flow control shorter than 3 bytes"));
        }
        let data = &data[h..];
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
    /// Extended/mixed-addressing byte, expected on and prepended to every frame.
    addr: Option<u8>,
    state: RxState,
}

impl RxMachine {
    pub fn new(rx_bs: u8, rx_stmin: u8, padding: Option<u8>, n_cr: Duration) -> Self {
        Self::with_addr(rx_bs, rx_stmin, padding, n_cr, None)
    }

    pub fn with_addr(
        rx_bs: u8,
        rx_stmin: u8,
        padding: Option<u8>,
        n_cr: Duration,
        addr: Option<u8>,
    ) -> Self {
        Self {
            buf: Vec::new(),
            ff_dl: 0,
            rx_bs,
            rx_stmin,
            padding,
            n_cr,
            addr,
            state: RxState::Idle,
        }
    }

    fn fc(&self) -> Vec<u8> {
        let mut f = start_frame(self.addr);
        f.extend_from_slice(&[0x30, self.rx_bs, self.rx_stmin]);
        pad(f, self.padding)
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
        // Validate and strip the extended-addressing byte when in use, so the
        // rest of the parser sees a plain frame. `h` still adjusts the
        // full-frame length checks (a full ext frame carries one fewer byte).
        let h = self.addr.is_some() as usize;
        if let Some(a) = self.addr
            && data[0] != a
        {
            return Err(IsoTpError::Malformed("extended-address mismatch"));
        }
        if data.len() < h + 1 {
            return Err(IsoTpError::Malformed("frame too short for extended address"));
        }
        let data = &data[h..];
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
                if data.len() < 8 - h {
                    return Err(IsoTpError::Malformed("short first frame"));
                }
                let ff_dl = ((data[0] & 0x0f) as usize) << 8 | data[1] as usize;
                // A payload that would fit in a single frame must not use a
                // first frame. The single-frame capacity is one byte smaller
                // under extended addressing (6 vs 7).
                if ff_dl <= 7 - h {
                    return Err(IsoTpError::Malformed("FF_DL fits in a single frame"));
                }
                self.ff_dl = ff_dl;
                self.buf.clear();
                self.buf.extend_from_slice(&data[2..8 - h]);
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
                let take = need.min(7 - h).min(data.len() - 1);
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
