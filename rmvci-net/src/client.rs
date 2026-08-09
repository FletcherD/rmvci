//! PC-side [`Transport`] that forwards every primitive to a remote
//! `rmvci-bridge` over TCP. Everything above the transport — codec, actor,
//! channels, both ISO-TP paths — runs unchanged on this machine; only the raw
//! byte I/O crosses the network.
//!
//! ```no_run
//! use rmvci_core::{Device, DeviceConfig};
//! use rmvci_net::TcpTransport;
//!
//! let io = TcpTransport::connect("192.168.1.50:6979")?;
//! let dev = Device::open_transport(io, DeviceConfig::default())?;
//! println!("firmware: {}", dev.firmware_version()?);
//! # Ok::<(), rmvci_core::Error>(())
//! ```

use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

use rmvci_core::transport::{LatencyResult, Transport};
use rmvci_core::TransportError;

use crate::wire;

/// Fallback socket-read window for the non-`read` operations (write, purge,
/// set_modem, optimize_latency), and the slack added on top of a `read`'s own
/// timeout. A response that does not arrive inside this means the link or the
/// bridge is dead, which surfaces as a [`TransportError`] rather than hanging
/// the session actor forever.
const OP_TIMEOUT: Duration = Duration::from_secs(10);

fn net_err(reason: impl std::fmt::Display) -> TransportError {
    TransportError::Open { port: "rmvci-bridge".into(), reason: reason.to_string() }
}

pub struct TcpTransport {
    stream: TcpStream,
}

impl TcpTransport {
    /// Connect to a bridge and complete the version handshake.
    pub fn connect(addr: impl ToSocketAddrs) -> Result<Self, TransportError> {
        let mut stream = TcpStream::connect(addr)?;
        // Strict request/response with small frames: Nagle would add up to a
        // round of delay to every exchange, so disable it.
        stream.set_nodelay(true)?;

        stream.write_all(wire::MAGIC)?;
        stream.write_all(&[wire::VERSION])?;
        stream.flush()?;

        stream.set_read_timeout(Some(OP_TIMEOUT))?;
        let mut ack = [0u8; 1];
        stream.read_exact(&mut ack)?;
        if ack[0] != wire::RESP_OK {
            return Err(net_err(
                "bridge rejected the handshake (protocol version mismatch?)",
            ));
        }
        Ok(Self { stream })
    }

    /// Send one request body, wait for its response body. `read_timeout`
    /// bounds the socket read so a dead peer cannot wedge the caller.
    fn transact(&mut self, body: &[u8], read_timeout: Duration) -> Result<Vec<u8>, TransportError> {
        wire::write_frame(&mut self.stream, body)?;
        // Duration of zero disables the timeout on some platforms; floor it.
        self.stream
            .set_read_timeout(Some(read_timeout.max(Duration::from_millis(1))))?;
        wire::read_frame(&mut self.stream).map_err(Into::into)
    }
}

/// Split a response body into the payload after `RESP_OK`, or turn a
/// `RESP_ERR` body into the corresponding [`TransportError`].
fn ok_payload(resp: &[u8]) -> Result<&[u8], TransportError> {
    match resp.first() {
        Some(&wire::RESP_OK) => Ok(&resp[1..]),
        Some(&wire::RESP_ERR) => {
            Err(net_err(String::from_utf8_lossy(&resp[1..]).into_owned()))
        }
        _ => Err(net_err("malformed response frame")),
    }
}

impl Transport for TcpTransport {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        let mut body = Vec::with_capacity(1 + buf.len());
        body.push(wire::REQ_WRITE);
        body.extend_from_slice(buf);
        let resp = self.transact(&body, OP_TIMEOUT)?;
        ok_payload(&resp).map(|_| ())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError> {
        let mut body = Vec::with_capacity(1 + 4 + 8);
        body.push(wire::REQ_READ);
        body.extend_from_slice(&(buf.len() as u32).to_le_bytes());
        let micros = timeout.as_micros().min(u64::MAX as u128) as u64;
        body.extend_from_slice(&micros.to_le_bytes());
        // The bridge blocks its own cable read for up to `timeout`; the socket
        // must outlast that, so add OP_TIMEOUT of slack.
        let resp = self.transact(&body, timeout + OP_TIMEOUT)?;
        let payload = ok_payload(&resp)?;
        let n = payload.len().min(buf.len());
        buf[..n].copy_from_slice(&payload[..n]);
        Ok(n)
    }

    fn purge_rx(&mut self) -> Result<(), TransportError> {
        let resp = self.transact(&[wire::REQ_PURGE], OP_TIMEOUT)?;
        ok_payload(&resp).map(|_| ())
    }

    fn set_modem(&mut self, dtr: bool, rts: bool) -> Result<(), TransportError> {
        let flags = (dtr as u8) | ((rts as u8) << 1);
        let resp = self.transact(&[wire::REQ_SET_MODEM, flags], OP_TIMEOUT)?;
        ok_payload(&resp).map(|_| ())
    }

    /// The latency fix runs on the bridge, against the real cable. A network
    /// failure here is reported as [`LatencyResult::Failed`] rather than an
    /// error, matching the trait's infallible contract.
    fn optimize_latency(&mut self) -> LatencyResult {
        match self.transact(&[wire::REQ_OPT_LATENCY], OP_TIMEOUT) {
            Ok(resp) => match ok_payload(&resp) {
                Ok(payload) => wire::decode_latency(payload),
                Err(e) => LatencyResult::Failed { reason: e.to_string() },
            },
            Err(e) => LatencyResult::Failed { reason: e.to_string() },
        }
    }
}
