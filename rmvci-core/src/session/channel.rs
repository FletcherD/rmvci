//! `Channel<P>` — the typed channel API. Each protocol exposes only the
//! filter shape and message form it actually supports; everything lowers to
//! the single `RawChannel` implementation.

use std::marker::PhantomData;
use std::time::Duration;

use crate::error::Error;
use crate::session::protocol::{CanFramed, CanId, FilterSpec, KLine, Protocol};
use crate::session::raw::RawChannel;
use crate::types::{RxMsg, TxFlags};

pub struct Channel<P: Protocol> {
    raw: RawChannel,
    /// Wire id of the currently-installed filter, if any.
    current_filter: Option<u32>,
    next_filter_id: u32,
    _p: PhantomData<P>,
}

impl<P: Protocol> Channel<P> {
    pub(crate) fn new(raw: RawChannel) -> Self {
        Self { raw, current_filter: None, next_filter_id: 0x000e_7a00, _p: PhantomData }
    }

    /// Install the channel's filter, replacing any previous one — *set*,
    /// singular, because that is what the hardware does: the firmware keeps
    /// one live acceptance-filter range per identifier width, and every
    /// StartMsgFilter rewrites it. An API that pretended to accumulate
    /// filters would lie about what is actually live.
    pub fn set_filter(&mut self, filter: P::Filter) -> Result<(), Error> {
        let enc = filter.encode();
        if let Some(old) = self.current_filter.take() {
            // Retiring the old descriptor is bookkeeping — the new filter
            // overwrites the hardware entry either way — so a failure here
            // must not stop the install. It is still worth seeing.
            if let Err(e) = self.raw.stop_filter(old) {
                tracing::warn!(filter_id = old, error = %e, "could not retire the previous filter");
            }
        }
        let id = self.next_filter_id;
        self.next_filter_id += 1;
        self.raw.start_filter(
            id,
            enc.ftype as u32,
            &enc.mask,
            &enc.pattern,
            enc.flow.as_deref(),
            enc.idlen,
        )?;
        self.current_filter = Some(id);
        Ok(())
    }

    pub fn clear_filter(&mut self) -> Result<(), Error> {
        if let Some(old) = self.current_filter.take() {
            self.raw.stop_filter(old)?;
        }
        Ok(())
    }

    /// One READ_MSG poll; `Ok(None)` when nothing is queued.
    pub fn poll(&mut self, timeout: Duration) -> Result<Option<RxMsg>, Error> {
        self.raw.poll(timeout)
    }

    /// Wait for a data message (echoes/indications skipped).
    pub fn read(&mut self, timeout: Duration) -> Result<RxMsg, Error> {
        self.raw.read(timeout)
    }

    pub fn set_config(&mut self, param: u32, value: u32) -> Result<(), Error> {
        self.raw.set_config(param, value)
    }

    /// Escape hatch to the dynamically-typed layer.
    pub fn raw(&mut self) -> &mut RawChannel {
        &mut self.raw
    }

    /// Disconnect explicitly (also happens on drop).
    pub fn disconnect(self) -> Result<(), Error> {
        drop(self);
        Ok(())
    }
}

impl<P: KLine> Channel<P> {
    /// Send raw K-line message bytes (header + payload). Works for both
    /// ISO14230 and ISO9141 — they share the firmware's K-line path.
    pub fn write(&mut self, msg: &[u8]) -> Result<(), Error> {
        self.raw.write(0, msg)
    }

    /// FAST_INIT wake-up; returns the ECU key bytes.
    pub fn fast_init(&mut self, init: &[u8]) -> Result<Vec<u8>, Error> {
        self.raw.fast_init(init)
    }

    /// FIVE_BAUD_INIT (slow init); returns the ECU key bytes. The usual init
    /// for ISO9141 and older KWP2000 sessions.
    pub fn five_baud_init(&mut self, init: &[u8]) -> Result<Vec<u8>, Error> {
        self.raw.five_baud_init(init)
    }

    pub fn clear_periodic(&mut self) -> Result<(), Error> {
        self.raw.clear_periodic()
    }
}

impl<P: CanFramed> Channel<P> {
    /// Send one message to `id` (a raw CAN frame on protocol 5, a full
    /// ISO-TP payload on protocol 6). The 29-bit TxFlag is derived from the
    /// identifier.
    pub fn send(&mut self, id: CanId, payload: &[u8]) -> Result<(), Error> {
        self.send_flags(id, payload, TxFlags::default())
    }

    pub fn send_flags(&mut self, id: CanId, payload: &[u8], flags: TxFlags) -> Result<(), Error> {
        let mut flags = flags;
        if id.is_ext() {
            flags |= TxFlags::CAN_29BIT_ID;
        }
        let mut msg = Vec::with_capacity(4 + payload.len());
        msg.extend_from_slice(&id.to_wire());
        msg.extend_from_slice(payload);
        self.raw.write(flags.bits(), &msg)
    }
}
