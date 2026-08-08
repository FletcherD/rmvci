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
use crate::types::{RxMsg, RxStatus, TxFlags};

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
    /// Extended/mixed addressing byte. When set, every frame carries it ahead
    /// of the PCI (usable data per frame drops by one) and the receive filter
    /// additionally matches on it.
    pub ext_addr: Option<u8>,
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
            ext_addr: None,
        }
    }

    /// Enable extended/mixed addressing with the given address byte.
    pub fn with_ext_addr(mut self, addr: u8) -> Self {
        self.ext_addr = Some(addr);
        self
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

    /// Connect the raw-CAN channel **without** installing a filter yet, for
    /// callers whose endpoints arrive after the connect (the J2534 shim, which
    /// learns `rx`/`tx` from a later flow-control filter). Call
    /// [`IsoTp::set_endpoints`] before `send`/`recv`.
    pub fn connect_deferred(dev: &Device, cfg: IsoTpConfig, can: CanConfig) -> Result<Self, Error> {
        let chan = dev.connect::<Can>(can)?;
        Ok(Self { chan, cfg })
    }

    /// Set (or change) the endpoints and reinstall the exact rx filter. This is
    /// how the deferred-connect path becomes usable, and how a channel is
    /// repointed at a different ECU.
    pub fn set_endpoints(
        &mut self,
        tx_id: CanId,
        rx_id: CanId,
        ext_addr: Option<u8>,
    ) -> Result<(), Error> {
        self.cfg.tx_id = tx_id;
        self.cfg.rx_id = rx_id;
        self.cfg.ext_addr = ext_addr;
        self.chan.set_filter(CanFilter::exact(rx_id))
    }

    /// The response identifier this client reassembles from (for the shim to
    /// prefix onto a J2534 read message).
    pub fn rx_id(&self) -> CanId {
        self.cfg.rx_id
    }

    /// Frames from the response identifier, indications skipped.
    ///
    /// One poll yields at most one message — READ_MSG returns a single
    /// queued message per call — so a burst of consecutive frames is drained
    /// one at a time out of the adapter's ~1500-byte ring. That is why
    /// `rx_bs` exists: on a very long response, advertise a block size
    /// rather than let the ECU outrun the drain.
    fn poll_frame(&mut self, budget: Duration) -> Result<Option<RxMsg>, Error> {
        match self.chan.poll(budget)? {
            Some(m)
                if !m.is_indication()
                    && m.data.len() > 4
                    && m.data[..4] == self.cfg.rx_id.to_wire()
                    // Raw CAN cannot hardware-filter on the address byte, so
                    // discriminate it here when extended addressing is on.
                    && self.cfg.ext_addr.is_none_or(|a| m.data.get(4) == Some(&a)) =>
            {
                Ok(Some(m))
            }
            _ => Ok(None),
        }
    }

    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        let mut tx = TxMachine::with_addr(
            payload,
            self.cfg.padding,
            self.cfg.n_bs,
            self.cfg.wft_max,
            self.cfg.ext_addr,
        )?;
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
        let mut rx = RxMachine::with_addr(
            self.cfg.rx_bs,
            self.cfg.rx_stmin,
            self.cfg.padding,
            self.cfg.n_cr,
            self.cfg.ext_addr,
        );
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

    /// Send then receive.
    ///
    /// Note `send` may consume frames from the response identifier while
    /// waiting for flow control, and discards any that are not flow control.
    /// That only matters if an ECU starts answering before it has received
    /// the whole multi-frame request, which ISO 15765-2 does not allow.
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
    /// Extended/mixed addressing byte. The firmware inserts it per frame
    /// itself; the host prepends it once to the whole payload and sets
    /// `ISO15765_ADDR_TYPE`.
    ext_addr: Option<u8>,
}

impl FirmwareIsoTp {
    pub fn new(dev: &Device, tx_id: CanId, rx_id: CanId) -> Result<Self, Error> {
        Self::configure(dev, tx_id, rx_id, CanConfig::default(), None)
    }

    pub fn with_bitrate(
        dev: &Device,
        tx_id: CanId,
        rx_id: CanId,
        can: CanConfig,
    ) -> Result<Self, Error> {
        Self::configure(dev, tx_id, rx_id, can, None)
    }

    /// Connect with extended/mixed addressing: the acceptance filter matches on
    /// the 5th (address) byte, and every exchange carries `addr`.
    pub fn with_ext_addr(
        dev: &Device,
        tx_id: CanId,
        rx_id: CanId,
        addr: u8,
    ) -> Result<Self, Error> {
        Self::configure(dev, tx_id, rx_id, CanConfig::default(), Some(addr))
    }

    fn configure(
        dev: &Device,
        tx_id: CanId,
        rx_id: CanId,
        can: CanConfig,
        ext_addr: Option<u8>,
    ) -> Result<Self, Error> {
        let mut chan = dev.connect::<Iso15765>(can)?;
        let filter = FlowControlFilter::exact(rx_id, tx_id);
        let filter = match ext_addr {
            Some(a) => filter.with_ext_addr(a),
            None => filter,
        };
        chan.set_filter(filter)?;
        Ok(Self { chan, tx_id, rx_id, ext_addr })
    }

    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        // With extended addressing the firmware expects the address byte at the
        // head of the payload (it lands at msg[4] on the wire, before the PCI)
        // and segments from there; without it the payload is sent as-is.
        let (buf, flags) = match self.ext_addr {
            Some(a) => {
                let mut b = Vec::with_capacity(1 + payload.len());
                b.push(a);
                b.extend_from_slice(payload);
                (b, TxFlags::ISO15765_ADDR_TYPE)
            }
            None => (payload.to_vec(), TxFlags::default()),
        };
        // The firmware writes the First Frame length as `len & 0xFF` with a
        // literal 0x10 high nibble, so anything above 255 bytes goes out
        // malformed. Refuse it here; the host path handles large sends.
        if buf.len() > 255 {
            return Err(IsoTpError::FirmwareFfDlLimit(buf.len()).into());
        }
        self.chan.send_flags(self.tx_id, &buf, flags)
    }

    pub fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>, Error> {
        loop {
            let m = self.chan.read(timeout)?;
            if m.data.len() > 4 && m.data[..4] == self.rx_id.to_wire() {
                let body = &m.data[4..];
                // With extended addressing the firmware flags the reply with
                // RxStatus 0x80 and (RE-derived — confirm on the bench) leaves
                // the address byte at the head of the reassembled payload;
                // strip it. If the flag is absent, hand back the body as-is.
                let start = usize::from(
                    self.ext_addr.is_some()
                        && m.rx_status.contains(RxStatus::ADDR_TYPE)
                        && !body.is_empty(),
                );
                return Ok(body[start..].to_vec());
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
