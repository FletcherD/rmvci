//! The bridge wire protocol. Rust-to-Rust, deliberately tiny: five request
//! kinds mirroring the five [`Transport`](rmvci_core::transport::Transport)
//! primitives, each answered by one response frame on the same connection.
//!
//! Framing: `u32` little-endian body length, then that many body bytes.
//! Ordering is guaranteed by the single TCP stream and the strict
//! request/response cadence, so no request ids are needed.
//!
//! A fixed 5-byte hello (`MAGIC` + `VERSION`) precedes the framed traffic so a
//! protocol mismatch fails loudly at connect instead of desyncing later.

use std::io::{self, Read, Write};

use rmvci_core::transport::LatencyResult;

pub const MAGIC: &[u8; 4] = b"RMVN";
pub const VERSION: u8 = 1;

/// Upper bound on a framed body, a guard against a corrupt/hostile length
/// prefix allocating unbounded memory.
pub const MAX_FRAME: usize = 1 << 20; // 1 MiB
/// Upper bound on a single `READ` capacity. Real reads here are tens of bytes
/// (one J2534 inner reply); this only caps a malformed request's allocation.
pub const MAX_READ: usize = 64 * 1024;

// Request tags (first body byte, PC -> phone).
pub const REQ_WRITE: u8 = 0x01;
pub const REQ_READ: u8 = 0x02;
pub const REQ_PURGE: u8 = 0x03;
pub const REQ_SET_MODEM: u8 = 0x04;
pub const REQ_OPT_LATENCY: u8 = 0x05;

// Response status (first body byte, phone -> PC).
pub const RESP_OK: u8 = 0x00;
pub const RESP_ERR: u8 = 0x01;

// LatencyResult variant tags (payload of a successful OPT_LATENCY response).
const LAT_SET: u8 = 0x00;
const LAT_ALREADY_LOW: u8 = 0x01;
const LAT_FAILED: u8 = 0x02;
const LAT_UNAVAILABLE: u8 = 0x03;

/// Write one length-prefixed frame and flush it.
pub fn write_frame(w: &mut impl Write, body: &[u8]) -> io::Result<()> {
    let len = u32::try_from(body.len())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "frame body too large"))?;
    w.write_all(&len.to_le_bytes())?;
    w.write_all(body)?;
    w.flush()
}

/// Read one length-prefixed frame. A clean disconnect surfaces as
/// [`io::ErrorKind::UnexpectedEof`] from the length read.
pub fn read_frame(r: &mut impl Read) -> io::Result<Vec<u8>> {
    let mut lenb = [0u8; 4];
    r.read_exact(&mut lenb)?;
    let len = u32::from_le_bytes(lenb) as usize;
    if len > MAX_FRAME {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("frame length {len} exceeds MAX_FRAME {MAX_FRAME}"),
        ));
    }
    let mut body = vec![0u8; len];
    r.read_exact(&mut body)?;
    Ok(body)
}

/// Build an error response body from a message.
pub fn err_resp(msg: &str) -> Vec<u8> {
    let mut v = Vec::with_capacity(1 + msg.len());
    v.push(RESP_ERR);
    v.extend_from_slice(msg.as_bytes());
    v
}

/// Encode a [`LatencyResult`] as the payload after a `RESP_OK` byte.
pub fn encode_latency(l: &LatencyResult) -> Vec<u8> {
    match l {
        LatencyResult::Set { millis } => vec![LAT_SET, *millis],
        LatencyResult::AlreadyLow { millis } => vec![LAT_ALREADY_LOW, *millis],
        LatencyResult::Failed { reason } => {
            let mut v = vec![LAT_FAILED];
            v.extend_from_slice(reason.as_bytes());
            v
        }
        LatencyResult::Unavailable => vec![LAT_UNAVAILABLE],
    }
}

/// Decode the payload produced by [`encode_latency`].
pub fn decode_latency(b: &[u8]) -> LatencyResult {
    match b.first() {
        Some(&LAT_SET) => LatencyResult::Set { millis: b.get(1).copied().unwrap_or(0) },
        Some(&LAT_ALREADY_LOW) => LatencyResult::AlreadyLow { millis: b.get(1).copied().unwrap_or(0) },
        Some(&LAT_FAILED) => {
            LatencyResult::Failed { reason: String::from_utf8_lossy(&b[1..]).into_owned() }
        }
        _ => LatencyResult::Unavailable,
    }
}
