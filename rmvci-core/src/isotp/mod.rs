//! ISO 15765-2 transport: one [`IsoTp`] type, run on either [`IsoTpPath`].
//!
//! - [`IsoTpPath::Firmware`] (protocol 6) — the cable's own ISO-TP, one
//!   exchange per request. Bench-validated for single-frame requests and
//!   multi-frame *receive*; its transmit path ignores BS/STmin and breaks above
//!   255 bytes, so [`IsoTp::send`] refuses larger payloads with
//!   [`IsoTpError::FirmwareFfDlLimit`] rather than emit a malformed frame.
//! - [`IsoTpPath::Host`] (protocol 5) — the machines in [`machine`] over raw
//!   CAN: full 12-bit FF_DL, BS and STmin honoured, FS=WAIT/OVFLW handled.
//!   Correct but slow, one serial round trip per CAN frame.
//!
//! The cable has a single global receive owner, so a device runs one path at
//! a time. [`IsoTp`] implements [`UdsTransport`], so application code flips
//! between the paths with one argument.

pub mod machine;

use std::time::{Duration, Instant};

use crate::error::{Error, IsoTpError};
use crate::session::protocol::{Can, CanConfig, CanFilter, CanId, FlowControlFilter, Iso15765};
use crate::session::{Channel, Device};
use crate::types::{RxMsg, RxStatus, TxFlags};

use machine::{RxEvent, RxMachine, TxAction, TxMachine};

/// A KWP2000/UDS negative response of the form `7F <sid> 78`
/// (requestCorrectlyReceived-**ResponsePending**): the ECU acknowledges the
/// request immediately and then sends the real response a beat later. A
/// [`UdsTransport::request`] must never hand this frame back as the answer —
/// it keeps reading until the true response arrives.
pub(crate) fn is_response_pending(resp: &[u8]) -> bool {
    matches!(resp, [0x7f, _, 0x78, ..])
}

/// Upper bound on consecutive `7F .. 78` pending frames tolerated before a
/// `request` gives up and returns the pending frame (so the caller still sees
/// the NRC rather than blocking forever on a stuck ECU).
pub(crate) const MAX_RESPONSE_PENDING: u32 = 30;

/// Drive one request/response to completion: `recv` is called repeatedly,
/// transparently swallowing up to [`MAX_RESPONSE_PENDING`] consecutive
/// `7F .. 78` responsePending frames, and the first non-pending response (or
/// the last pending one, if the ECU never finishes) is returned. The caller
/// performs the send before invoking this. `recv` must yield the bare service
/// bytes — K-line strips its header first.
pub(crate) fn drain_pending(
    mut recv: impl FnMut() -> Result<Vec<u8>, Error>,
) -> Result<Vec<u8>, Error> {
    let mut pending = 0;
    loop {
        let resp = recv()?;
        if is_response_pending(&resp) && pending < MAX_RESPONSE_PENDING {
            pending += 1;
            continue;
        }
        return Ok(resp);
    }
}

/// One request/response exchange — the shape UDS/KWP diagnostics need.
///
/// Implementations transparently consume `7F .. 78` responsePending frames
/// (`7F <sid> 78`, requestCorrectlyReceived-ResponsePending) and return the final
/// application response.
pub trait UdsTransport {
    fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error>;
}

/// Which side of the cable runs ISO 15765-2.
///
/// The cable has a single global receive owner, so one [`IsoTp`] exists per
/// device at a time, on one path or the other — never both.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IsoTpPath {
    /// The cable firmware segments and reassembles (protocol 6). One serial
    /// exchange per request — the fast path, and the default. Two firmware
    /// limits apply: transmit above 255 bytes is malformed (so
    /// [`IsoTp::send`] refuses it with [`IsoTpError::FirmwareFfDlLimit`]) and
    /// the ECU's flow-control BS/STmin are ignored.
    #[default]
    Firmware,
    /// This crate segments and reassembles over a raw CAN channel (protocol 5)
    /// using [`machine`]: full 12-bit FF_DL to 4095 bytes, BS and STmin
    /// honoured, FS=WAIT/OVFLW handled. Correct but slow: every CAN frame is a
    /// serial round trip (~20–35 ms at the FTDI default 16 ms latency timer).
    Host,
}

/// Endpoints and timing for an [`IsoTp`] channel.
#[derive(Debug, Clone, Copy)]
pub struct IsoTpConfig {
    /// Identifier we transmit on.
    pub tx_id: CanId,
    /// Identifier the ECU answers (and sends flow control) on.
    pub rx_id: CanId,
    /// Bus bitrate and connect flags (default 500 kbit/s).
    pub can: CanConfig,
    /// Block size we advertise when receiving (0 = unlimited). The adapter's
    /// RX ring holds ~1500 bytes (~100 frames); advertise 16 for very long
    /// responses if polling can't keep up. Host path only.
    pub rx_bs: u8,
    /// STmin byte we advertise when receiving. Host path only.
    pub rx_stmin: u8,
    /// Frame padding byte; Toyota expects padded frames. `None` sends
    /// unpadded. Host path only — the firmware pads its own transmissions
    /// regardless.
    pub padding: Option<u8>,
    /// How long to wait for the ECU's flow control after a First Frame.
    /// Host path only.
    pub n_bs: Duration,
    /// Max gap between the ECU's consecutive frames. Host path only.
    pub n_cr: Duration,
    /// How many FS=WAIT frames to tolerate before giving up. Host path only.
    pub wft_max: u8,
    /// Extended/mixed addressing byte. When set, every frame carries it ahead
    /// of the PCI (usable data per frame drops by one) and the receive filter
    /// additionally matches on it.
    pub ext_addr: Option<u8>,
}

impl IsoTpConfig {
    /// Defaults: 500 kbit/s, `0x00` padding, unlimited receive block size,
    /// 1 s N_Bs / N_Cr, normal addressing.
    pub fn new(tx_id: CanId, rx_id: CanId) -> Self {
        Self {
            tx_id,
            rx_id,
            can: CanConfig::default(),
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

    /// Bus bitrate / connect flags other than the 500 kbit/s default.
    pub fn with_can(mut self, can: CanConfig) -> Self {
        self.can = can;
        self
    }
}

/// ISO 15765-2 over the cable, on either [`IsoTpPath`].
///
/// ```no_run
/// use rmvci_core::{CanId, Device, IsoTp, IsoTpConfig, IsoTpPath};
/// use std::time::Duration;
///
/// # fn main() -> Result<(), rmvci_core::Error> {
/// let dev = Device::open("/dev/ttyUSB0")?;
/// let cfg = IsoTpConfig::new(CanId::Std(0x7e0), CanId::Std(0x7e8));
/// let mut tp = IsoTp::new(&dev, cfg, IsoTpPath::Firmware)?;
/// let vin = tp.request(&[0x22, 0xf1, 0x90], Duration::from_secs(2))?;
/// # Ok(()) }
/// ```
pub struct IsoTp {
    chan: PathChannel,
    cfg: IsoTpConfig,
}

enum PathChannel {
    Firmware(Channel<Iso15765>),
    Host(Channel<Can>),
}

impl IsoTp {
    /// Connect the bus and install the exact receive filter for `cfg.rx_id`.
    ///
    /// On the host path one PASS filter on `rx_id` covers both data and flow
    /// control (they arrive on the same identifier); on the firmware path it
    /// is the firmware's flow-control filter, with the address byte when
    /// `ext_addr` is set.
    pub fn new(dev: &Device, cfg: IsoTpConfig, path: IsoTpPath) -> Result<Self, Error> {
        let mut tp = match path {
            IsoTpPath::Firmware => Self { chan: PathChannel::Firmware(dev.connect(cfg.can)?), cfg },
            IsoTpPath::Host => Self::connect_deferred(dev, cfg)?,
        };
        tp.install_filter()?;
        Ok(tp)
    }

    /// Connect a **host-path** channel without installing a filter yet, for
    /// callers whose endpoints arrive after the connect (the J2534 shim, which
    /// learns `rx`/`tx` from a later flow-control filter). Call
    /// [`IsoTp::set_endpoints`] before `send`/`recv`.
    pub fn connect_deferred(dev: &Device, cfg: IsoTpConfig) -> Result<Self, Error> {
        Ok(Self { chan: PathChannel::Host(dev.connect(cfg.can)?), cfg })
    }

    /// Set (or change) the endpoints and reinstall the receive filter. This is
    /// how a deferred-connect channel becomes usable, and how a channel is
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
        self.install_filter()
    }

    fn install_filter(&mut self) -> Result<(), Error> {
        match &mut self.chan {
            PathChannel::Host(chan) => chan.set_filter(CanFilter::exact(self.cfg.rx_id)),
            PathChannel::Firmware(chan) => {
                let filter = FlowControlFilter::exact(self.cfg.rx_id, self.cfg.tx_id);
                let filter = match self.cfg.ext_addr {
                    Some(a) => filter.with_ext_addr(a),
                    None => filter,
                };
                chan.set_filter(filter)
            }
        }
    }

    /// Which side is running ISO-TP.
    pub fn path(&self) -> IsoTpPath {
        match self.chan {
            PathChannel::Firmware(_) => IsoTpPath::Firmware,
            PathChannel::Host(_) => IsoTpPath::Host,
        }
    }

    /// The response identifier this client reassembles from (for the shim to
    /// prefix onto a J2534 read message).
    pub fn rx_id(&self) -> CanId {
        self.cfg.rx_id
    }

    /// Send one ISO-TP message, segmented as the path allows.
    ///
    /// On the firmware path payloads above 255 bytes are refused with
    /// [`IsoTpError::FirmwareFfDlLimit`]: the firmware writes the First Frame
    /// length as `len & 0xFF` with a literal `0x10` high nibble, so they would
    /// go out malformed. Use [`IsoTpPath::Host`] for those.
    pub fn send(&mut self, payload: &[u8]) -> Result<(), Error> {
        match &mut self.chan {
            PathChannel::Host(_) => self.host_send(payload),
            PathChannel::Firmware(chan) => {
                // With extended addressing the firmware expects the address
                // byte at the head of the payload (it lands at msg[4] on the
                // wire, before the PCI) and segments from there.
                let (buf, flags) = match self.cfg.ext_addr {
                    Some(a) => {
                        let mut b = Vec::with_capacity(1 + payload.len());
                        b.push(a);
                        b.extend_from_slice(payload);
                        (b, TxFlags::ISO15765_ADDR_TYPE)
                    }
                    None => (payload.to_vec(), TxFlags::default()),
                };
                if buf.len() > 255 {
                    return Err(IsoTpError::FirmwareFfDlLimit(buf.len()).into());
                }
                chan.send_flags(self.cfg.tx_id, &buf, flags)
            }
        }
    }

    /// Receive one reassembled ISO-TP message from `rx_id`.
    pub fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>, Error> {
        match &mut self.chan {
            PathChannel::Host(_) => self.host_recv(timeout),
            PathChannel::Firmware(chan) => loop {
                let m = chan.read(timeout)?;
                if m.data.len() > 4 && m.data[..4] == self.cfg.rx_id.to_wire() {
                    let body = &m.data[4..];
                    // With extended addressing the firmware flags the reply
                    // with RxStatus 0x80 and (RE-derived — confirm on the
                    // bench) leaves the address byte at the head of the
                    // reassembled payload; strip it. If the flag is absent,
                    // hand back the body as-is.
                    let start = usize::from(
                        self.cfg.ext_addr.is_some()
                            && m.rx_status.contains(RxStatus::ADDR_TYPE)
                            && !body.is_empty(),
                    );
                    return Ok(body[start..].to_vec());
                }
                // A frame from some other identifier slipping through means
                // the filter isn't what we think it is; keep waiting rather
                // than hand back someone else's payload.
            },
        }
    }

    /// Send then receive, swallowing `7F .. 78` responsePending frames.
    ///
    /// On the host path `send` may consume frames from the response identifier
    /// while waiting for flow control, and discards any that are not flow
    /// control. That only matters if an ECU starts answering before it has
    /// received the whole multi-frame request, which ISO 15765-2 does not
    /// allow.
    pub fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        self.send(req)?;
        drain_pending(|| self.recv(timeout))
    }

    // ---- host path: the machines in `machine` driven over raw CAN ----

    fn host_chan(&mut self) -> &mut Channel<Can> {
        match &mut self.chan {
            PathChannel::Host(chan) => chan,
            PathChannel::Firmware(_) => unreachable!("host_* called on the firmware path"),
        }
    }

    /// Frames from the response identifier, indications skipped.
    ///
    /// One poll yields at most one message — READ_MSG returns a single
    /// queued message per call — so a burst of consecutive frames is drained
    /// one at a time out of the adapter's ~1500-byte ring. That is why
    /// `rx_bs` exists: on a very long response, advertise a block size
    /// rather than let the ECU outrun the drain.
    fn poll_frame(&mut self, budget: Duration) -> Result<Option<RxMsg>, Error> {
        let cfg = self.cfg;
        match self.host_chan().poll(budget)? {
            Some(m)
                if !m.is_indication()
                    && m.data.len() > 4
                    && m.data[..4] == cfg.rx_id.to_wire()
                    // Raw CAN cannot hardware-filter on the address byte, so
                    // discriminate it here when extended addressing is on.
                    && cfg.ext_addr.is_none_or(|a| m.data.get(4) == Some(&a)) =>
            {
                Ok(Some(m))
            }
            _ => Ok(None),
        }
    }

    fn host_send(&mut self, payload: &[u8]) -> Result<(), Error> {
        let cfg = self.cfg;
        let mut tx =
            TxMachine::with_addr(payload, cfg.padding, cfg.n_bs, cfg.wft_max, cfg.ext_addr)?;
        loop {
            match tx.next(Instant::now())? {
                TxAction::Send(frame) => self.host_chan().send(cfg.tx_id, &frame)?,
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

    fn host_recv(&mut self, timeout: Duration) -> Result<Vec<u8>, Error> {
        let cfg = self.cfg;
        let mut rx =
            RxMachine::with_addr(cfg.rx_bs, cfg.rx_stmin, cfg.padding, cfg.n_cr, cfg.ext_addr);
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
                    RxEvent::SendFc(fc) => self.host_chan().send(cfg.tx_id, &fc)?,
                    RxEvent::Done(payload) => return Ok(payload),
                    RxEvent::Continue => {}
                }
            }
        }
    }
}

impl UdsTransport for IsoTp {
    fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        IsoTp::request(self, req, timeout)
    }
}

#[cfg(test)]
mod pending_tests {
    use super::*;

    #[test]
    fn is_response_pending_matches_only_7f_xx_78() {
        assert!(is_response_pending(&[0x7f, 0x21, 0x78]));
        assert!(is_response_pending(&[0x7f, 0x30, 0x78, 0x00]));
        assert!(!is_response_pending(&[0x61, 0x0d, 0x09]));
        assert!(!is_response_pending(&[0x7f, 0x21, 0x31])); // a real NRC
        assert!(!is_response_pending(&[0x7f, 0x78])); // too short
    }

    #[test]
    fn drain_swallows_pending_then_returns_real() {
        let mut frames = std::collections::VecDeque::from(vec![
            vec![0x7f, 0x21, 0x78],
            vec![0x7f, 0x21, 0x78],
            vec![0x61, 0x0d, 0x09],
        ]);
        let got = drain_pending(|| Ok(frames.pop_front().unwrap())).unwrap();
        assert_eq!(got, vec![0x61, 0x0d, 0x09]);
        assert!(frames.is_empty());
    }

    #[test]
    fn drain_gives_up_after_the_cap_and_returns_the_pending() {
        let mut calls = 0u32;
        let got = drain_pending(|| {
            calls += 1;
            Ok(vec![0x7f, 0x21, 0x78])
        })
        .unwrap();
        assert!(is_response_pending(&got));
        assert_eq!(calls, MAX_RESPONSE_PENDING + 1); // capped, not infinite
    }

    #[test]
    fn drain_propagates_recv_error() {
        let err = drain_pending(|| Err(Error::BufferEmpty));
        assert!(err.is_err());
    }
}
