//! Framed reads over a `Transport`.

use std::time::{Duration, Instant};

use crate::error::{CodecError, Error};
use crate::transport::Transport;
use crate::types::MAX_FRAME;

/// Read one framed message: the 16-bit LEN header within `timeout`, then the
/// body.
///
/// The body deadline scales with the frame: at 115200 8N1 a byte takes
/// ~87 µs, so a full 4 KB ISO15765 reply spends ~360 ms on the wire and would
/// blow a caller's 200 ms poll budget mid-frame. A truncated read leaves the
/// tail in the tty buffer for the next purge to swallow, which desynchronises
/// the session in a way that only shows up on long replies.
pub(crate) fn read_frame<T: Transport>(io: &mut T, timeout: Duration) -> Result<Vec<u8>, Error> {
    let mut hdr = [0u8; 2];
    read_exact(io, &mut hdr, timeout)?;
    let len = u16::from_le_bytes(hdr) as usize;
    if len < 3 {
        return Err(CodecError::TooShort(len).into());
    }
    if len > MAX_FRAME {
        return Err(CodecError::TooLong(len).into());
    }

    let mut frame = vec![0u8; len];
    frame[..2].copy_from_slice(&hdr);
    // ~11.5 bytes/ms on the wire, doubled, plus slack (ported from serial.c:291).
    let body_timeout = timeout + Duration::from_millis((len / 5) as u64 + 50);
    read_exact(io, &mut frame[2..], body_timeout)?;
    Ok(frame)
}

fn read_exact<T: Transport>(io: &mut T, buf: &mut [u8], timeout: Duration) -> Result<(), Error> {
    let deadline = Instant::now() + timeout;
    let mut off = 0;
    while off < buf.len() {
        let remaining = deadline.saturating_duration_since(Instant::now());
        let n = io.read(&mut buf[off..], remaining)?;
        if n == 0 {
            // The transport's wait elapsed with nothing available.
            return Err(Error::Timeout(timeout));
        }
        off += n;
    }
    Ok(())
}
