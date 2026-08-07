//! `RawChannel` — the dynamically-typed channel the J2534 shim consumes.
//! `Channel<P>` is a zero-cost typed veneer over this; there is exactly one
//! implementation of every operation.

use std::time::{Duration, Instant};

use crate::codec::inner;
use crate::error::{CodecError, Error};
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
        // the original 64-byte bound.
        let (lo, hi) = match self.proto {
            ProtocolId::Can => (4, 12),
            ProtocolId::Iso15765 => (4, MAX_MSG),
            _ => (1, 64),
        };
        if msg.len() < lo || msg.len() > hi {
            return Err(CodecError::MsgTooLong(msg.len()).into());
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

    pub fn clear_periodic(&mut self) -> Result<(), Error> {
        self.transact_status(
            inner::clear_periodic(self.proto),
            Cmd::Ioctl,
            Duration::from_millis(1000),
        )
    }

    /// K-line FAST_INIT; returns the ECU key bytes.
    pub fn fast_init(&mut self, init: &[u8]) -> Result<Vec<u8>, Error> {
        let resp = self
            .dev
            .transact(inner::fast_init(self.proto, init), Duration::from_millis(3000))?;
        if resp.len() < 3 || resp[2] != Cmd::Ioctl as u8 {
            return Err(CodecError::MalformedReply("not a fast-init reply").into());
        }
        let field = resp[0] as usize | (resp[1] as usize) << 8;
        let mlen = field.saturating_sub(1).min(resp.len() - 3);
        Ok(resp[3..3 + mlen].to_vec())
    }

    fn transact_status(&self, cmd_bytes: Vec<u8>, cmd: Cmd, timeout: Duration) -> Result<(), Error> {
        let resp = self.dev.transact(cmd_bytes, timeout)?;
        let status = inner::status_of(&resp, cmd)?;
        if status.is_ok() { Ok(()) } else { Err(Error::Rejected { cmd, status }) }
    }
}

impl Drop for RawChannel {
    /// Best-effort per-channel disconnect (0x08). The session itself stays
    /// up; it closes when the last Device handle drops.
    fn drop(&mut self) {
        let _ = self.dev.disconnect_proto(self.proto);
    }
}
