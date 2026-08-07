//! `Device` — a cheap `Clone` handle onto the actor. No lifetimes cross this
//! boundary, which is what lets the J2534 shim store devices and channels in
//! a C-style slot table and lets `Drop` do the wire teardown.

use std::sync::Arc;
use std::sync::mpsc;
use std::time::Duration;

use crate::codec::inner;
use crate::error::{CodecError, Error};
use crate::session::actor::{self, Request};
use crate::transport::{Clock, RealClock, Transport};
use crate::types::{Cmd, ProtocolId, RxMsg};

pub struct DeviceConfig {
    /// Serial port path; `None` falls back to `$MVCI_PORT`, then
    /// `/dev/ttyUSB0`.
    pub port: Option<String>,
    /// Idle interval after which the actor pokes the adapter so it doesn't
    /// reset. Every real transaction postpones it. The C driver hammered
    /// every 15 ms; the true watchdog threshold is measured on the bench in
    /// M4 — until then 100 ms is the conservative default.
    pub keepalive: Duration,
    pub clock: Arc<dyn Clock>,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self { port: None, keepalive: Duration::from_millis(100), clock: Arc::new(RealClock) }
    }
}

/// Resolution order matches the C shim: explicit → `$MVCI_PORT` →
/// `/dev/ttyUSB0`.
pub fn resolve_port(explicit: Option<&str>) -> String {
    explicit
        .map(str::to_string)
        .or_else(|| std::env::var("MVCI_PORT").ok())
        .unwrap_or_else(|| "/dev/ttyUSB0".to_string())
}

#[derive(Clone)]
pub struct Device {
    tx: mpsc::Sender<Request>,
    key: [u8; 8],
}

impl Device {
    /// Open, DTR/RTS-reset and handshake the cable on `port`.
    #[cfg(feature = "serial")]
    pub fn open(port: impl Into<String>) -> Result<Self, Error> {
        Self::open_with(DeviceConfig { port: Some(port.into()), ..DeviceConfig::default() })
    }

    #[cfg(feature = "serial")]
    pub fn open_with(cfg: DeviceConfig) -> Result<Self, Error> {
        let path = resolve_port(cfg.port.as_deref());
        let io = crate::transport::SerialTransport::open(&path)?;
        Self::open_transport(io, cfg)
    }

    /// Open over any transport (mock transports in tests).
    pub fn open_transport<T: Transport>(io: T, cfg: DeviceConfig) -> Result<Self, Error> {
        let (tx, key) = actor::spawn(io, Arc::new(cfg))?;
        Ok(Self { tx, key })
    }

    /// The DES session key from the opening handshake (diagnostics only; a
    /// connect-retry re-handshake may have replaced it since).
    pub fn des_key(&self) -> [u8; 8] {
        self.key
    }

    /// CMD 0x03 — the firmware identity string, `"J2534 MINIV1.03"` on the
    /// stock cable.
    pub fn firmware_version(&self) -> Result<String, Error> {
        let resp = self.transact(inner::read_version(), Duration::from_millis(1000))?;
        if resp.len() < 3 || resp[2] != Cmd::ReadVersion as u8 {
            return Err(CodecError::MalformedReply("not a version reply").into());
        }
        let body = &resp[3..];
        let end = body.iter().position(|&b| b == 0).unwrap_or(body.len());
        Ok(String::from_utf8_lossy(&body[..end]).into_owned())
    }

    pub(crate) fn transact(&self, inner: Vec<u8>, timeout: Duration) -> Result<Vec<u8>, Error> {
        let (rtx, rrx) = mpsc::channel();
        self.tx
            .send(Request::Transact { inner, timeout, reply: rtx })
            .map_err(|_| Error::Closed)?;
        rrx.recv().map_err(|_| Error::Closed)?
    }

    pub(crate) fn connect_proto(
        &self,
        proto: ProtocolId,
        flags: u32,
        baud: u32,
    ) -> Result<(), Error> {
        let (rtx, rrx) = mpsc::channel();
        self.tx
            .send(Request::Connect { proto, flags, baud, reply: rtx })
            .map_err(|_| Error::Closed)?;
        rrx.recv().map_err(|_| Error::Closed)?
    }

    pub(crate) fn disconnect_proto(&self, proto: ProtocolId) -> Result<(), Error> {
        let (rtx, rrx) = mpsc::channel();
        self.tx.send(Request::Disconnect { proto, reply: rtx }).map_err(|_| Error::Closed)?;
        rrx.recv().map_err(|_| Error::Closed)?
    }

    pub(crate) fn poll_proto(
        &self,
        proto: ProtocolId,
        timeout: Duration,
    ) -> Result<Option<RxMsg>, Error> {
        let (rtx, rrx) = mpsc::channel();
        self.tx
            .send(Request::Poll { proto, timeout, reply: rtx })
            .map_err(|_| Error::Closed)?;
        rrx.recv().map_err(|_| Error::Closed)?
    }
}
