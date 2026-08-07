//! Incremental deframer: push raw serial bytes in, take complete frames out.
//!
//! The stream has no sync marker, so a corrupt length field is unrecoverable
//! by scanning — the session layer recovers by purging the port instead
//! (`clear()` here). Validation of checksums belongs to `frame::decode_*`;
//! this only cuts the stream at frame boundaries.

use crate::error::CodecError;
use crate::types::MAX_FRAME;

#[derive(Default)]
pub struct Deframer {
    buf: Vec<u8>,
}

impl Deframer {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// Bytes buffered but not yet returned as a frame.
    pub fn pending(&self) -> usize {
        self.buf.len()
    }

    /// Drop all buffered bytes (after a purge or a length-field error).
    pub fn clear(&mut self) {
        self.buf.clear();
    }

    /// Pop the next complete frame, `None` if more bytes are needed.
    ///
    /// A length field below 3 or above `MAX_FRAME` is an error; the buffer is
    /// left untouched so the caller can decide to `clear()` and resync.
    pub fn next_frame(&mut self) -> Result<Option<Vec<u8>>, CodecError> {
        if self.buf.len() < 2 {
            return Ok(None);
        }
        let len = self.buf[0] as usize | (self.buf[1] as usize) << 8;
        if len < 3 {
            return Err(CodecError::TooShort(len));
        }
        if len > MAX_FRAME {
            return Err(CodecError::TooLong(len));
        }
        if self.buf.len() < len {
            return Ok(None);
        }
        let frame = self.buf.drain(..len).collect();
        Ok(Some(frame))
    }
}
