//! ISO 15765-2 transport, both ways the cable can do it:
//!
//! - [`IsoTp`] — **host-side**, over a raw CAN channel (protocol 5). The
//!   machines in [`machine`] do segmentation and flow control correctly:
//!   full 12-bit FF_DL, BS honored, STmin honored, FS=WAIT/OVFLW handled.
//!   Correct but slow: every frame is a serial round trip (~20–35 ms at the
//!   FTDI default 16 ms latency timer; fix the timer for ~2–4× throughput).
//! - [`FirmwareIsoTp`] — the cable firmware's ISO-TP (protocol 6),
//!   bench-validated for single-frame requests and multi-frame *receive*.
//!   Its transmit path ignores BS/STmin and breaks above 255 bytes, so
//!   [`FirmwareIsoTp::send`] refuses larger payloads with
//!   [`IsoTpError::FirmwareFfDlLimit`] rather than emit a malformed frame.
//!
//! Both implement [`UdsTransport`], so application code (and the bench
//! comparison) flips between them with a flag.

pub mod machine;

use std::time::{Duration, Instant};

use crate::error::{Error, IsoTpError};
use crate::session::protocol::{Can, CanConfig, CanFilter, CanId, FlowControlFilter, Iso15765};
use crate::session::{Channel, Device};
use crate::types::RxMsg;

use machine::{RxEvent, RxMachine, TxAction, TxMachine};

/// One request/response exchange — the shape UDS/KWP diagnostics need.
pub trait UdsTransport {
    fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error>;
}

#[derive(Debug, Clone, Copy)]
pub struct IsoTpConfig {
    pub tx_id: CanId,
    pub rx_id: CanId,
    /// Block size we advertise when receiving (0 = unlimited). The adapter's
    /// RX ring holds ~1500 bytes (~100 frames); advertise 16 for very long
    /// responses if polling can't keep up.
    pub rx_bs: u8,
    /// STmin byte we advertise when receiving.
    pub rx_stmin: u8,
    /// Frame padding byte; Toyota expects padded frames. `None` sends
    /// unpadded (note the firmware pads its own transmissions regardless on
    /// protocol 6 — this only affects the host path).
    pub padding: Option<u8>,
    /// How long to wait for the ECU's flow control after a First Frame.
    pub n_bs: Duration,
    /// Max gap between the ECU's consecutive frames.
    pub n_cr: Duration,
    /// How many FS=WAIT frames to tolerate before giving up.
    pub wft_max: u8,
}

impl IsoTpConfig {
    pub fn new(tx_id: CanId, rx_id: CanId) -> Self {
        Self {
            tx_id,
            rx_id,
            rx_bs: 0,
            rx_stmin: 0,
            padding: Some(0x00),
            n_bs: Duration::from_secs(1),
            n_cr: Duration::from_secs(1),
            wft_max: 8,
        }
    }
}

/// Host-side ISO-TP over a raw CAN channel.
pub struct IsoTp {
    chan: Channel<Can>,
    cfg: IsoTpConfig,
}

impl IsoTp {
    /// Connect protocol 5 at 500 kbit/s and install an exact PASS filter on
    /// `rx_id` — flow control arrives on the same identifier, so one filter
    /// covers data and FC.
    pub fn new(dev: &Device, cfg: IsoTpConfig) -> Result<Self, Error> {
        Self::with_bitrate(dev, cfg, CanConfig::default())
    }

    pub fn with_bitrate(dev: &Device, cfg: IsoTpConfig, can: CanConfig) -> Result<Self, Error> {
        let mut chan = dev.connect::<Can>(can)?;
        chan.set_filter(CanFilter::exact(cfg.rx_id))?;
        Ok(Self { chan, cfg })
    }

    /// Frames from the response identifier, indications skipped.
    fn poll_frame(&mut self, budget: Duration) -> Result<Option<RxMsg>, Error> {
        match self.chan.poll(budget)? {
            Some(m)
                if !m.is_indication()
                    && m.data.len() > 4
                    && m.data[..4] == self.cfg.rx_id.to_wire() =>
            {
                Ok(Some(m))
            }
            _ => Ok(None),
        }
    }

    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        let mut tx = TxMachine::new(payload, self.cfg.padding, self.cfg.n_bs, self.cfg.wft_max)?;
        loop {
            match tx.next(Instant::now())? {
                TxAction::Send(frame) => self.chan.send(self.cfg.tx_id, &frame)?,
                TxAction::WaitFc { deadline } => {
                    let now = Instant::now();
                    if now >= deadline {
                        continue; // next() reports the FcTimeout
                    }
                    let budget = (deadline - now).min(Duration::from_millis(100));
                    if let Some(m) = self.poll_frame(budget)? {
                        tx.on_frame(&m.data[4..], Instant::now())?;
                    }
                }
                TxAction::WaitUntil(t) => {
                    let now = Instant::now();
                    if t > now {
                        std::thread::sleep(t - now);
                    }
                }
                TxAction::Done => return Ok(()),
            }
        }
    }

    pub fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>, Error> {
        let mut rx =
            RxMachine::new(self.cfg.rx_bs, self.cfg.rx_stmin, self.cfg.padding, self.cfg.n_cr);
        let deadline = Instant::now() + timeout;
        loop {
            let now = Instant::now();
            rx.check_timeout(now)?;
            if now >= deadline {
                return Err(Error::Timeout(timeout));
            }
            let budget = (deadline - now).min(Duration::from_millis(200));
            if let Some(m) = self.poll_frame(budget)? {
                match rx.on_frame(&m.data[4..], Instant::now())? {
                    RxEvent::SendFc(fc) => self.chan.send(self.cfg.tx_id, &fc)?,
                    RxEvent::Done(payload) => return Ok(payload),
                    RxEvent::Continue => {}
                }
            }
        }
    }

    pub fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        self.send(req)?;
        self.recv(timeout)
    }
}

impl UdsTransport for IsoTp {
    fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        IsoTp::request(self, req, timeout)
    }
}

/// The firmware's own ISO-TP (protocol 6) behind the same interface.
pub struct FirmwareIsoTp {
    chan: Channel<Iso15765>,
    tx_id: CanId,
    rx_id: CanId,
}

impl FirmwareIsoTp {
    pub fn new(dev: &Device, tx_id: CanId, rx_id: CanId) -> Result<Self, Error> {
        Self::with_bitrate(dev, tx_id, rx_id, CanConfig::default())
    }

    pub fn with_bitrate(
        dev: &Device,
        tx_id: CanId,
        rx_id: CanId,
        can: CanConfig,
    ) -> Result<Self, Error> {
        let mut chan = dev.connect::<Iso15765>(can)?;
        chan.set_filter(FlowControlFilter::exact(rx_id, tx_id))?;
        Ok(Self { chan, tx_id, rx_id })
    }

    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        // The firmware writes the First Frame length as `len & 0xFF` with a
        // literal 0x10 high nibble, so anything above 255 bytes goes out
        // malformed. Refuse it here; the host path handles large sends.
        if payload.len() > 255 {
            return Err(IsoTpError::FirmwareFfDlLimit(payload.len()).into());
        }
        self.chan.send(self.tx_id, payload)
    }

    pub fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>, Error> {
        loop {
            let m = self.chan.read(timeout)?;
            if m.data.len() > 4 && m.data[..4] == self.rx_id.to_wire() {
                return Ok(m.data[4..].to_vec());
            }
            // A frame from some other identifier slipping through means the
            // filter isn't what we think it is; keep waiting rather than
            // hand back someone else's payload.
        }
    }

    pub fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        self.send(req)?;
        self.recv(timeout)
    }
}

impl UdsTransport for FirmwareIsoTp {
    fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        FirmwareIsoTp::request(self, req, timeout)
    }
}
