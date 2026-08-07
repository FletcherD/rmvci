//! `Device` — a cheap `Clone` handle onto the actor. No lifetimes cross this
//! boundary, which is what lets the J2534 shim store devices and channels in
//! a C-style slot table and lets `Drop` do the wire teardown.

use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex};
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
    /// Optional idle poll: if set, the actor sends a READ_MSG when the wire
    /// has been quiet this long. Every real transaction postpones it.
    ///
    /// **Default: `None` — the adapter does not need it.** libMVCI polls
    /// every 15 ms on the premise that "the adapter resets if left idle".
    /// Measured on the cable (`live_idle_decay_characterisation`), after
    /// **15 minutes** of complete silence the DES session, the connected
    /// channel, its acceptance filter and a full request/response exchange
    /// all still work. The premise is simply not true, so rMVCI sends
    /// nothing when it has nothing to say.
    ///
    /// Set it if you want the RX ring drained in the background during long
    /// quiet periods, or a faster wedge verdict. Note this is **not** an ECU
    /// tester-present: a K-line ISO14230 session has its own P3 timeout at
    /// the *ECU*, which the application must service itself.
    pub keepalive: Option<Duration>,
    pub clock: Arc<dyn Clock>,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self { port: None, keepalive: None, clock: Arc::new(RealClock) }
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

/// Set by the actor thread just before it releases the port, so
/// [`Device::close`] can wait for the port to actually be free.
#[derive(Default)]
pub(crate) struct CloseSignal {
    done: Mutex<bool>,
    cv: Condvar,
}

impl CloseSignal {
    pub(crate) fn signal(&self) {
        *self.done.lock().unwrap() = true;
        self.cv.notify_all();
    }

    fn wait(&self, timeout: Duration) -> bool {
        let done = self.done.lock().unwrap();
        let (done, _) = self.cv.wait_timeout_while(done, timeout, |d| !*d).unwrap();
        *done
    }
}

#[derive(Clone)]
pub struct Device {
    tx: mpsc::Sender<Request>,
    key: [u8; 8],
    closed: Arc<CloseSignal>,
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

    /// Open the cable over raw USB instead of a tty, claiming it away from
    /// `ftdi_sio`. `serial` picks one FT232R among several by its USB serial
    /// string (the Mini-VCI's is on the cable, e.g. `A69QL5OE`); `None` takes
    /// the first.
    ///
    /// Prefer this when you cannot set the FTDI latency timer via sysfs —
    /// here it is a control transfer, so no root and no udev rule. Needs
    /// write access to the `/dev/bus/usb` node.
    #[cfg(feature = "usb")]
    pub fn open_usb(serial: Option<&str>, cfg: DeviceConfig) -> Result<Self, Error> {
        let io = crate::transport::UsbTransport::open(serial)?;
        Self::open_transport(io, cfg)
    }

    /// Open over any transport (mock transports in tests).
    pub fn open_transport<T: Transport>(io: T, cfg: DeviceConfig) -> Result<Self, Error> {
        let (tx, key, closed) = actor::spawn(io, Arc::new(cfg))?;
        Ok(Self { tx, key, closed })
    }

    /// Close the session and **wait for the port to be released**.
    ///
    /// Dropping a `Device` starts the teardown but returns immediately, so
    /// an immediate reopen of the same port fails with "Device or resource
    /// busy". Call this when you intend to reopen. Other clones of this
    /// handle keep the session alive; the wait then times out and returns
    /// `false`.
    pub fn close(self) -> bool {
        let closed = Arc::clone(&self.closed);
        drop(self);
        closed.wait(Duration::from_secs(5))
    }

    /// The DES session key from the opening handshake (diagnostics only; a
    /// connect-retry re-handshake may have replaced it since).
    pub fn des_key(&self) -> [u8; 8] {
        self.key
    }

    /// Open a typed channel. The firmware supports at most 3 concurrent
    /// channels; a fourth Connect is rejected by the adapter.
    pub fn connect<P: crate::session::protocol::Protocol>(
        &self,
        cfg: P::Config,
    ) -> Result<crate::session::Channel<P>, Error> {
        use crate::session::protocol::ChannelConfig;
        let (flags, baud) = cfg.wire();
        let raw = crate::session::RawChannel::connect(self.clone(), P::ID, flags, baud)?;
        Ok(crate::session::Channel::new(raw))
    }

    /// Open a dynamically-typed channel (the J2534 shim's path).
    pub fn connect_raw(
        &self,
        proto: ProtocolId,
        flags: u32,
        baud: u32,
    ) -> Result<crate::session::RawChannel, Error> {
        crate::session::RawChannel::connect(self.clone(), proto, flags, baud)
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
        self.tx.send(Request::Poll { proto, timeout, reply: rtx }).map_err(|_| Error::Closed)?;
        rrx.recv().map_err(|_| Error::Closed)?
    }
}
