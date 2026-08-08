//! `RawChannel` — the dynamically-typed channel the J2534 shim consumes.
//! `Channel<P>` is a zero-cost typed veneer over this; there is exactly one
//! implementation of every operation.

use std::time::{Duration, Instant};

use crate::codec::inner;
use crate::error::{CodecError, Error, IsoTpError};
use crate::session::device::Device;
use crate::types::{Cmd, MAX_MSG, ProtocolId, RxMsg};

pub struct RawChannel {
    dev: Device,
    proto: ProtocolId,
    nfilters: usize,
}

impl RawChannel {
    pub(crate) fn connect(
        dev: Device,
        proto: ProtocolId,
        flags: u32,
        baud: u32,
    ) -> Result<Self, Error> {
        dev.connect_proto(proto, flags, baud)?;
        Ok(Self { dev, proto, nfilters: 0 })
    }

    pub fn proto(&self) -> ProtocolId {
        self.proto
    }

    /// Filters currently installed via this channel. Zero means the channel
    /// receives *nothing* — the adapter's acceptance filter starts enabled
    /// and empty.
    pub fn filters_installed(&self) -> usize {
        self.nfilters
    }

    pub fn start_filter(
        &mut self,
        filter_id: u32,
        ftype: u32,
        mask: &[u8],
        pattern: &[u8],
        flow: Option<&[u8]>,
        idlen: usize,
    ) -> Result<(), Error> {
        let cmd = inner::start_filter(self.proto, filter_id, ftype, mask, pattern, flow, idlen)?;
        self.transact_status(cmd, Cmd::StartMsgFilter, Duration::from_millis(2000))?;
        self.nfilters += 1;
        Ok(())
    }

    pub fn stop_filter(&mut self, filter_id: u32) -> Result<(), Error> {
        self.transact_status(
            inner::stop_filter(self.proto, filter_id),
            Cmd::StopMsgFilter,
            Duration::from_millis(2000),
        )?;
        self.nfilters = self.nfilters.saturating_sub(1);
        Ok(())
    }

    /// Send one message. For CAN/ISO15765 the first 4 bytes of `msg` are the
    /// big-endian CAN identifier.
    pub fn write(&mut self, txflags: u32, msg: &[u8]) -> Result<(), Error> {
        // Size bounds follow the adapter: a raw CAN frame is a 4-byte id plus
        // at most 8 data bytes; ISO15765 segments up to MAX_MSG; K-line keeps
        // the original 64-byte bound. J1850 VPW/PWM use the SAE frame ceilings
        // — reachable only via this raw path (there is no typed
        // `Channel<J1850*>`, and the firmware's VPW/PWM objects are
        // experimental, never hardware-verified).
        let (lo, hi) = match self.proto {
            ProtocolId::Can => (4, 12),
            ProtocolId::Iso15765 => (4, MAX_MSG),
            ProtocolId::J1850Vpw => (1, 12),
            ProtocolId::J1850Pwm => (1, 11),
            ProtocolId::Iso9141 | ProtocolId::Iso14230 => (1, 64),
        };
        if msg.len() < lo || msg.len() > hi {
            return Err(CodecError::MsgTooLong(msg.len()).into());
        }
        // The firmware writes the ISO-TP First Frame length as `len & 0xFF`
        // under a literal 0x10 high nibble, so a payload above 255 bytes goes
        // out as a malformed First Frame (FF_DL wrong, tail lost). Refuse it
        // here rather than corrupt silently — the host-side `IsoTp` path over
        // raw CAN (protocol 5) handles transfers this large. `msg` is the
        // 4-byte identifier followed by the ISO-TP payload.
        if self.proto == ProtocolId::Iso15765 && msg.len() - 4 > 255 {
            return Err(IsoTpError::FirmwareFfDlLimit(msg.len() - 4).into());
        }
        let cmd = inner::write_msg(self.proto, txflags, msg)?;
        self.transact_status(cmd, Cmd::WriteMsg, Duration::from_millis(1000))
    }

    /// One READ_MSG poll: `Ok(None)` when nothing is queued.
    pub fn poll(&mut self, timeout: Duration) -> Result<Option<RxMsg>, Error> {
        self.dev.poll_proto(self.proto, timeout)
    }

    /// Poll until a *data* message arrives (transmit echoes and ISO15765
    /// start-of-message indications are skipped), or the deadline passes.
    ///
    /// On timeout with no filter installed this returns
    /// [`Error::NoFilterInstalled`] — the adapter's most confusing failure
    /// mode, silence that looks exactly like a dead bus.
    pub fn read(&mut self, timeout: Duration) -> Result<RxMsg, Error> {
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(if self.nfilters == 0 {
                    Error::NoFilterInstalled { proto: self.proto }
                } else {
                    Error::Timeout(timeout)
                });
            }
            if let Some(m) = self.poll(remaining.min(Duration::from_millis(500)))?
                && !m.is_indication()
            {
                return Ok(m);
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    /// SET_CONFIG. The firmware silently discards unknown parameters while
    /// returning success, so `Ok` proves delivery, not effect.
    pub fn set_config(&mut self, param: u32, value: u32) -> Result<(), Error> {
        self.transact_status(
            inner::set_config(self.proto, param, value),
            Cmd::Ioctl,
            Duration::from_millis(2000),
        )
    }

    /// START_PERIODIC_MSG. `msg_id` is chosen by the caller (host-assigned,
    /// like a filter handle) and echoed by the firmware; pass the same value
    /// to [`RawChannel::stop_periodic`]. `msg` is capped at 12 bytes by the
    /// firmware (a 4-byte CAN id plus up to 8 data bytes).
    pub fn start_periodic(
        &mut self,
        msg_id: u32,
        txflags: u32,
        interval_ms: u32,
        msg: &[u8],
    ) -> Result<(), Error> {
        let cmd = inner::start_periodic(self.proto, msg_id, txflags, interval_ms, msg)?;
        self.transact_status(cmd, Cmd::StartPeriodicMsg, Duration::from_millis(2000))
    }

    /// STOP_PERIODIC_MSG. `msg_id` is the handle returned by
    /// [`RawChannel::start_periodic`]; an unknown id is rejected
    /// ([`Error::Rejected`] with [`crate::types::DeviceStatus::InvalidMsgId`]).
    pub fn stop_periodic(&mut self, msg_id: u32) -> Result<(), Error> {
        self.transact_status(
            inner::stop_periodic(self.proto, msg_id),
            Cmd::StopPeriodicMsg,
            Duration::from_millis(2000),
        )
    }

    pub fn clear_periodic(&mut self) -> Result<(), Error> {
        self.transact_status(
            inner::clear_periodic(self.proto),
            Cmd::Ioctl,
            Duration::from_millis(1000),
        )
    }

    /// K-line FAST_INIT; returns the ECU key bytes.
    pub fn fast_init(&mut self, init: &[u8]) -> Result<Vec<u8>, Error> {
        let resp =
            self.dev.transact(inner::fast_init(self.proto, init), Duration::from_millis(3000))?;
        Self::parse_init_reply(&resp)
    }

    /// K-line FIVE_BAUD_INIT (slow init); returns the ECU key bytes, like
    /// [`RawChannel::fast_init`]. Used by ISO9141 and older KWP sessions.
    pub fn five_baud_init(&mut self, init: &[u8]) -> Result<Vec<u8>, Error> {
        let resp =
            self.dev.transact(inner::five_baud_init(self.proto, init), Duration::from_millis(3000))?;
        Self::parse_init_reply(&resp)
    }

    /// IOCTL GET_CONFIG — read one config parameter back from the adapter.
    /// The reply layout is RE-derived and not yet hardware-confirmed; see
    /// [`inner::parse_get_config`].
    pub fn get_config(&mut self, param: u32) -> Result<u32, Error> {
        let resp = self
            .dev
            .transact(inner::get_config(self.proto, param), Duration::from_millis(2000))?;
        Ok(inner::parse_get_config(&resp)?)
    }

    /// IOCTL READ_VBATT — battery voltage in millivolts, as the adapter
    /// reports it. On the Mini-VCI this is a hardcoded 12000 (no ADC); other
    /// cables return a real reading. The reply layout is RE-derived and not yet
    /// hardware-confirmed; see [`inner::parse_vbatt`].
    pub fn read_vbatt(&mut self) -> Result<u32, Error> {
        let resp =
            self.dev.transact(inner::read_vbatt(self.proto), Duration::from_millis(1000))?;
        Ok(inner::parse_vbatt(&resp)?)
    }

    /// The K-line init IOCTLs (FAST_INIT / FIVE_BAUD_INIT) reply with the IOCTL
    /// command echo followed by the ECU key bytes (length-driven tail).
    fn parse_init_reply(resp: &[u8]) -> Result<Vec<u8>, Error> {
        if resp.len() < 3 || resp[2] != Cmd::Ioctl as u8 {
            return Err(CodecError::MalformedReply("not a K-line init reply").into());
        }
        let field = resp[0] as usize | (resp[1] as usize) << 8;
        let mlen = field.saturating_sub(1).min(resp.len() - 3);
        Ok(resp[3..3 + mlen].to_vec())
    }

    fn transact_status(
        &self,
        cmd_bytes: Vec<u8>,
        cmd: Cmd,
        timeout: Duration,
    ) -> Result<(), Error> {
        let resp = self.dev.transact(cmd_bytes, timeout)?;
        let status = inner::status_of(&resp, cmd)?;
        if status.is_ok() { Ok(()) } else { Err(Error::Rejected { cmd, status }) }
    }
}

impl Drop for RawChannel {
    /// Best-effort per-channel disconnect (0x08). The session itself stays
    /// up; it closes when the last Device handle drops. `Drop` cannot
    /// report, so a failure is logged — it leaves a channel connected on the
    /// adapter, and the firmware only has three slots.
    fn drop(&mut self) {
        if let Err(e) = self.dev.disconnect_proto(self.proto) {
            tracing::warn!(
                proto = ?self.proto,
                error = %e,
                "channel disconnect failed; the adapter may still hold this channel"
            );
        }
    }
}
