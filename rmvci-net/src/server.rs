//! Phone-side server: owns one real cable [`Transport`] and executes the
//! primitives a remote [`TcpTransport`](crate::TcpTransport) forwards to it.
//!
//! The cable has a single global RX owner (see the parent project's
//! `re/FINDINGS.md`), so the bridge serves **one client at a time**: the
//! transport is opened once and reused across sequential connections, and each
//! fresh PC-side [`Device`](rmvci_core::Device) re-runs the DTR/RTS reset
//! handshake (forwarded as `set_modem`) to resynchronise.

use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::Duration;

use rmvci_core::transport::Transport;

use crate::wire;

/// Serve forwarded transport primitives on `listener`, reusing `transport`
/// across sequential client connections. Runs until the listener errors.
pub fn serve<T: Transport>(listener: TcpListener, mut transport: T) -> io::Result<()> {
    for conn in listener.incoming() {
        let stream = match conn {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!(error = %e, "accept failed");
                continue;
            }
        };
        let peer = stream.peer_addr().ok();
        if let Err(e) = stream.set_nodelay(true) {
            tracing::warn!(?peer, error = %e, "set_nodelay failed");
        }
        tracing::info!(?peer, "client connected");
        match serve_connection(stream, &mut transport) {
            Ok(()) => tracing::info!(?peer, "client disconnected"),
            Err(e) => tracing::warn!(?peer, error = %e, "client session ended"),
        }
    }
    Ok(())
}

/// Handle one client from hello to disconnect. Public so a caller that has
/// already accepted a stream (or a test) can drive a single session.
pub fn serve_connection<T: Transport>(
    mut stream: TcpStream,
    transport: &mut T,
) -> io::Result<()> {
    let mut hello = [0u8; 5];
    stream.read_exact(&mut hello)?;
    if &hello[..4] != wire::MAGIC {
        let _ = stream.write_all(&[wire::RESP_ERR]);
        return Err(io::Error::new(io::ErrorKind::InvalidData, "bad magic in hello"));
    }
    if hello[4] != wire::VERSION {
        let _ = stream.write_all(&[wire::RESP_ERR]);
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("protocol version {} != {}", hello[4], wire::VERSION),
        ));
    }
    stream.write_all(&[wire::RESP_OK])?;
    stream.flush()?;
    // Block indefinitely for the next request: a READ carries its own bounded
    // timeout applied to the cable, never to this socket, so a quiet link is
    // just an idle client, not a failure.
    stream.set_read_timeout(None)?;

    loop {
        let req = match wire::read_frame(&mut stream) {
            Ok(r) => r,
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
            Err(e) => return Err(e),
        };
        let resp = dispatch(transport, &req);
        wire::write_frame(&mut stream, &resp)?;
    }
}

fn dispatch<T: Transport>(t: &mut T, req: &[u8]) -> Vec<u8> {
    let Some((&tag, body)) = req.split_first() else {
        return wire::err_resp("empty request");
    };
    match tag {
        wire::REQ_WRITE => match t.write_all(body) {
            Ok(()) => vec![wire::RESP_OK],
            Err(e) => wire::err_resp(&e.to_string()),
        },
        wire::REQ_READ => {
            if body.len() < 12 {
                return wire::err_resp("short READ request");
            }
            let cap = u32::from_le_bytes(body[0..4].try_into().unwrap()) as usize;
            let micros = u64::from_le_bytes(body[4..12].try_into().unwrap());
            let cap = cap.min(wire::MAX_READ);
            let mut buf = vec![0u8; cap];
            match t.read(&mut buf, Duration::from_micros(micros)) {
                Ok(n) => {
                    let mut v = Vec::with_capacity(1 + n);
                    v.push(wire::RESP_OK);
                    v.extend_from_slice(&buf[..n.min(cap)]);
                    v
                }
                Err(e) => wire::err_resp(&e.to_string()),
            }
        }
        wire::REQ_PURGE => match t.purge_rx() {
            Ok(()) => vec![wire::RESP_OK],
            Err(e) => wire::err_resp(&e.to_string()),
        },
        wire::REQ_SET_MODEM => {
            let Some(&flags) = body.first() else {
                return wire::err_resp("short SET_MODEM request");
            };
            match t.set_modem(flags & 1 != 0, flags & 2 != 0) {
                Ok(()) => vec![wire::RESP_OK],
                Err(e) => wire::err_resp(&e.to_string()),
            }
        }
        wire::REQ_OPT_LATENCY => {
            let mut v = vec![wire::RESP_OK];
            v.extend_from_slice(&wire::encode_latency(&t.optimize_latency()));
            v
        }
        other => wire::err_resp(&format!("unknown request tag {other:#04x}")),
    }
}
