//! The port-owning actor: one thread owns the transport, the DES key, the
//! keepalive timer and the per-protocol receive queues; callers send a
//! request and block on a oneshot reply. The C driver used a keepalive
//! thread plus a non-recursive mutex, which deadlocked whenever a lock
//! holder called back into the public API — here that hazard cannot exist.

use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use crate::codec::{frame, inner};
use crate::error::Error;
use crate::session::device::{CloseSignal, DeviceConfig};
use crate::session::link::KeyedLink;
use crate::session::wire;
use crate::transport::{LatencyResult, Transport};
use crate::types::{Cmd, ProtocolId, RxMsg};

/// Consecutive keepalive failures after which the adapter is declared wedged
/// and requests fail fast instead of each eating a full timeout.
const WEDGE_THRESHOLD: u8 = 3;

pub(crate) enum Request {
    /// Encrypt an inner command, send it, return the decrypted reply inner.
    Transact {
        inner: Vec<u8>,
        timeout: Duration,
        reply: mpsc::Sender<Result<Vec<u8>, Error>>,
    },
    /// CONNECT with the C driver's recovery: after a prior Disconnect the
    /// adapter ignores Connect until it is reset, so on rejection re-run the
    /// handshake and retry once (serial.c:427).
    Connect {
        proto: ProtocolId,
        flags: u32,
        baud: u32,
        reply: mpsc::Sender<Result<(), Error>>,
    },
    Disconnect {
        proto: ProtocolId,
        reply: mpsc::Sender<Result<(), Error>>,
    },
    /// One READ_MSG poll, draining the actor-side queue first.
    Poll {
        proto: ProtocolId,
        timeout: Duration,
        reply: mpsc::Sender<Result<Option<RxMsg>, Error>>,
    },
}

/// What a successful open hands back to `Device`.
pub(crate) type Opened = (mpsc::Sender<Request>, [u8; 8], Arc<CloseSignal>);

/// Run the open sequence on a fresh thread and hand back the request sender,
/// the session key, and the close signal. Blocks until the handshake
/// finishes.
pub(crate) fn spawn<T: Transport>(mut io: T, cfg: Arc<DeviceConfig>) -> Result<Opened, Error> {
    let (tx, rx) = mpsc::channel::<Request>();
    let (open_tx, open_rx) = mpsc::channel::<Result<[u8; 8], Error>>();
    let closed = Arc::new(CloseSignal::default());
    let closed_in_thread = Arc::clone(&closed);

    std::thread::Builder::new()
        .name("rmvci-actor".into())
        .spawn(move || {
            let opened = open(&mut io, &cfg);
            let link = match opened {
                Ok(link) => {
                    let _ = open_tx.send(Ok(*link.key()));
                    link
                }
                Err(e) => {
                    let _ = open_tx.send(Err(e));
                    closed_in_thread.signal();
                    return;
                }
            };
            Actor {
                io,
                link,
                cfg,
                rx,
                queues: HashMap::new(),
                connected: Vec::new(),
                silent_exchanges: 0,
                wedged: false,
                last_wire: Instant::now(),
            }
            .run();
            // The transport (and with it the port) is dropped as the Actor
            // goes out of scope; signal only after that.
            closed_in_thread.signal();
        })
        .map_err(|e| Error::Transport(e.into()))?;

    open_rx.recv().map_err(|_| Error::Closed)?.map(|key| (tx, key, closed))
}

/// DTR/RTS reset dance (io.c:187-197) + latency fix + handshake. The dance is
/// what recovers a wedged MCU; do not shorten the waits.
fn open<T: Transport>(io: &mut T, cfg: &DeviceConfig) -> Result<KeyedLink, Error> {
    io.set_modem(true, false)?; // RTS clear, DTR set
    cfg.clock.sleep(Duration::from_millis(15));
    io.set_modem(false, false)?; // DTR clear -> MCU resets
    cfg.clock.sleep(Duration::from_millis(1000)); // boot
    io.purge_rx()?;

    match io.optimize_latency() {
        LatencyResult::Set { millis } => tracing::info!(millis, "FTDI latency timer set"),
        LatencyResult::AlreadyLow { millis } => {
            tracing::debug!(millis, "FTDI latency timer already low")
        }
        LatencyResult::Failed { reason } => tracing::warn!(
            %reason,
            "FTDI latency timer left at its default; every exchange pays up to 16 ms extra"
        ),
        LatencyResult::Unavailable => {}
    }

    KeyedLink::establish(io, &*cfg.clock)
}

struct Actor<T: Transport> {
    io: T,
    link: KeyedLink,
    cfg: Arc<DeviceConfig>,
    rx: mpsc::Receiver<Request>,
    /// Messages the optional idle poll drained, waiting for a `Poll`.
    queues: HashMap<ProtocolId, VecDeque<RxMsg>>,
    /// Connected channels, kept sorted; drives the poll's proto and teardown.
    connected: Vec<ProtocolId>,
    /// Consecutive exchanges that got no reply. Counted from *real* requests
    /// as well as idle polls, so a wedged adapter is detected while it is
    /// being used — which is the only time it matters — without any idle
    /// traffic. Reset by any successful exchange.
    silent_exchanges: u8,
    wedged: bool,
    last_wire: Instant,
}

impl<T: Transport> Actor<T> {
    fn run(mut self) {
        loop {
            // With no idle poll configured, block until a request arrives.
            let outcome = match self.cfg.keepalive {
                Some(interval) => {
                    let deadline = self.last_wire + interval;
                    self.rx.recv_timeout(deadline.saturating_duration_since(Instant::now()))
                }
                None => self.rx.recv().map_err(|_| mpsc::RecvTimeoutError::Disconnected),
            };
            match outcome {
                Ok(req) => self.handle(req),
                Err(mpsc::RecvTimeoutError::Timeout) => self.idle_poll(),
                Err(mpsc::RecvTimeoutError::Disconnected) => {
                    self.teardown();
                    return;
                }
            }
        }
    }

    fn handle(&mut self, req: Request) {
        if self.wedged {
            let err = || Error::Wedged(self.silent_exchanges);
            match req {
                Request::Transact { reply, .. } => drop(reply.send(Err(err()))),
                Request::Connect { reply, .. } => drop(reply.send(Err(err()))),
                Request::Disconnect { reply, .. } => drop(reply.send(Err(err()))),
                Request::Poll { reply, .. } => drop(reply.send(Err(err()))),
            }
            return;
        }
        match req {
            Request::Transact { inner, timeout, reply } => {
                let r = self.transact(&inner, timeout);
                let _ = reply.send(r);
            }
            Request::Connect { proto, flags, baud, reply } => {
                let r = self.connect(proto, flags, baud);
                let _ = reply.send(r);
            }
            Request::Disconnect { proto, reply } => {
                let r = self.disconnect(proto);
                let _ = reply.send(r);
            }
            Request::Poll { proto, timeout, reply } => {
                let r = self.poll(proto, timeout);
                let _ = reply.send(r);
            }
        }
    }

    fn transact(&mut self, inner_bytes: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        let r = self.transact_inner(inner_bytes, timeout);
        // Silence is what a wedged adapter looks like. A rejection or a
        // corrupt frame means it is talking, so those do not count.
        match &r {
            Err(Error::Timeout(_)) => self.note_silence(),
            _ => self.silent_exchanges = 0,
        }
        r
    }

    fn transact_inner(&mut self, inner_bytes: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        let wire_frame = frame::encode_encrypted(self.link.key(), inner_bytes)?;
        tracing::trace!(tx = %hex(inner_bytes), "transact");
        self.io.purge_rx()?;
        self.io.write_all(&wire_frame)?;
        self.last_wire = Instant::now();
        let raw = wire::read_frame(&mut self.io, timeout)?;
        self.last_wire = Instant::now();
        let resp = frame::decode_encrypted(self.link.key(), &raw)?;
        tracing::trace!(rx = %hex(&resp), "reply");
        Ok(resp)
    }

    fn note_silence(&mut self) {
        self.silent_exchanges = self.silent_exchanges.saturating_add(1);
        tracing::warn!(consecutive = self.silent_exchanges, "no reply from the adapter");
        if self.silent_exchanges >= WEDGE_THRESHOLD {
            self.wedged = true;
            tracing::error!(
                "adapter unresponsive; declared wedged — reopen the device to \
                 DTR/RTS-reset it"
            );
        }
    }

    fn transact_status(
        &mut self,
        inner_bytes: &[u8],
        cmd: Cmd,
        timeout: Duration,
    ) -> Result<(), Error> {
        let resp = self.transact(inner_bytes, timeout)?;
        let status = inner::status_of(&resp, cmd)?;
        if status.is_ok() { Ok(()) } else { Err(Error::Rejected { cmd, status }) }
    }

    fn connect(&mut self, proto: ProtocolId, flags: u32, baud: u32) -> Result<(), Error> {
        let inner_bytes = inner::connect(proto, flags, baud);
        let timeout = Duration::from_millis(2000);
        if let Err(first) = self.transact_status(&inner_bytes, Cmd::Connect, timeout) {
            tracing::debug!(error = %first, "connect rejected; re-running the handshake once");
            self.link = KeyedLink::establish(&mut self.io, &*self.cfg.clock)?;
            self.transact_status(&inner_bytes, Cmd::Connect, timeout)?;
        }
        if !self.connected.contains(&proto) {
            self.connected.push(proto);
            self.connected.sort_by_key(|p| p.wire());
        }
        Ok(())
    }

    fn disconnect(&mut self, proto: ProtocolId) -> Result<(), Error> {
        let r = self.transact_status(
            &inner::disconnect(proto),
            Cmd::Disconnect,
            Duration::from_millis(1000),
        );
        self.connected.retain(|p| *p != proto);
        r
    }

    fn poll(&mut self, proto: ProtocolId, timeout: Duration) -> Result<Option<RxMsg>, Error> {
        if let Some(m) = self.queues.get_mut(&proto).and_then(VecDeque::pop_front) {
            return Ok(Some(m));
        }
        let resp = self.transact(&inner::read_poll(proto), timeout)?;
        let parsed = inner::parse_read_reply(&resp)?;
        if parsed.msg.is_empty() {
            Ok(None)
        } else {
            Ok(Some(RxMsg { rx_status: parsed.rx_status, data: parsed.msg.to_vec() }))
        }
    }

    /// Optional idle poll (off by default — the adapter does not need one;
    /// see `DeviceConfig::keepalive`). Its only jobs are draining the RX ring
    /// during long quiet periods and reaching a wedge verdict sooner.
    ///
    /// The reply is parsed, so a real message the poll happened to drain is
    /// queued rather than discarded as the C keepalive did.
    fn idle_poll(&mut self) {
        if self.wedged {
            self.last_wire = Instant::now();
            return;
        }
        let proto = self.connected.first().copied().unwrap_or(ProtocolId::Iso15765);
        // transact() does the wedge accounting for us.
        match self.transact(&inner::read_poll(proto), Duration::from_millis(500)) {
            Ok(resp) => {
                if let Ok(parsed) = inner::parse_read_reply(&resp)
                    && !parsed.msg.is_empty()
                {
                    self.queues.entry(proto).or_default().push_back(RxMsg {
                        rx_status: parsed.rx_status,
                        data: parsed.msg.to_vec(),
                    });
                }
            }
            Err(_) => self.last_wire = Instant::now(),
        }
    }

    /// All request senders dropped: disconnect every channel (0x08 each),
    /// close the encrypted session (0x02), release the port.
    ///
    /// Every step continues past a failure — the port must be released even
    /// if the adapter has stopped answering — but nothing is swallowed
    /// silently, because a failure here is exactly what leaves the adapter
    /// needing a physical replug.
    fn teardown(&mut self) {
        for proto in std::mem::take(&mut self.connected) {
            if let Err(e) = self.transact_status(
                &inner::disconnect(proto),
                Cmd::Disconnect,
                Duration::from_millis(500),
            ) {
                tracing::warn!(?proto, error = %e, "channel disconnect failed during teardown");
            }
        }
        if let Err(e) = self.transact(&inner::session_close(), Duration::from_millis(500)) {
            tracing::warn!(error = %e, "session close failed during teardown");
        }
        tracing::debug!("session closed");
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ")
}
