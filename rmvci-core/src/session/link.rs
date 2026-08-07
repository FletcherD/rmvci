//! Encrypted-link establishment.
//!
//! Command 0x01 (OPEN / identify) TOGGLES the adapter's encryption rather
//! than enabling it — sent twice in one session it silently turns the cipher
//! back off. The invariant that keeps this safe: 0x01 is emitted in exactly
//! one place, [`KeyedLink::establish`], and always immediately after the
//! plaintext reset frame that returns the adapter to its un-keyed state. No
//! other code path can produce the opcode.

use std::time::Duration;

use crate::codec::{frame, inner};
use crate::error::Error;
use crate::session::wire;
use crate::transport::{Clock, Transport};

/// Proof that the handshake ran: holds the session's DES key. Replacing a
/// `KeyedLink` (re-handshake after the adapter de-initialises) goes through
/// `establish` again, which starts with the reset frame.
pub(crate) struct KeyedLink {
    key: [u8; 8],
}

impl KeyedLink {
    /// Reset + identify + read challenge → install key (serial.c:298).
    pub fn establish<T: Transport>(io: &mut T, clock: &dyn Clock) -> Result<Self, Error> {
        let reset = frame::encode_plain(&[])?;
        io.purge_rx()?;
        io.write_all(&reset)?;
        clock.sleep(Duration::from_millis(110));
        io.purge_rx()?;

        let ident = frame::encode_plain(&inner::identify())?;
        io.write_all(&ident)?;

        let challenge = wire::read_frame(io, Duration::from_millis(2000))?;
        let payload = frame::decode_plain(&challenge)?;
        let key = inner::parse_challenge(payload)?;
        tracing::debug!(key = ?key, "handshake complete");
        Ok(Self { key })
    }

    pub fn key(&self) -> &[u8; 8] {
        &self.key
    }
}
