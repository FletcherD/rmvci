//! Outer wire frame: `[LEN u16 LE][PAYLOAD][XORSUM]`.
//!
//! `LEN` is the total frame length including itself and the checksum, and is
//! genuinely 16-bit — short messages just leave the high byte zero, which is
//! why it long looked like a reserved 0x00. ISO15765 replies exceed 255.
//! `XORSUM` is the XOR of every byte before it.
//!
//! Payload is plaintext for the pre-key handshake frames; afterwards it is
//! `DES-ECB(key, inner)` with the inner zero-padded to a multiple of 8.

use super::crypto;
use crate::error::CodecError;

pub fn xorsum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |c, b| c ^ b)
}

pub(crate) fn rd_u16(p: &[u8]) -> usize {
    p[0] as usize | (p[1] as usize) << 8
}

/// Build a plaintext frame (handshake only).
pub fn encode_plain(payload: &[u8]) -> Result<Vec<u8>, CodecError> {
    let total = payload.len() + 3;
    if total > 0xffff {
        return Err(CodecError::TooLong(total));
    }
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(&(total as u16).to_le_bytes());
    out.extend_from_slice(payload);
    out.push(xorsum(&out));
    Ok(out)
}

/// Encrypt an inner command into a wire frame.
pub fn encode_encrypted(key: &[u8; 8], inner: &[u8]) -> Result<Vec<u8>, CodecError> {
    let clen = inner.len().div_ceil(8) * 8;
    let total = clen + 3;
    if total > 0xffff {
        return Err(CodecError::TooLong(total));
    }
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(&(total as u16).to_le_bytes());
    out.extend_from_slice(inner);
    out.resize(2 + clen, 0);
    crypto::encrypt_in_place(key, &mut out[2..]);
    out.push(xorsum(&out));
    Ok(out)
}

/// Validate a plaintext frame and return its payload.
pub fn decode_plain(frame: &[u8]) -> Result<&[u8], CodecError> {
    if frame.len() < 3 {
        return Err(CodecError::TooShort(frame.len()));
    }
    if rd_u16(frame) != frame.len() {
        return Err(CodecError::LengthMismatch { field: rd_u16(frame), actual: frame.len() });
    }
    let (body, &[wire]) = frame.split_at(frame.len() - 1) else {
        unreachable!()
    };
    let computed = xorsum(body);
    if computed != wire {
        return Err(CodecError::BadChecksum { wire, computed });
    }
    Ok(&frame[2..frame.len() - 1])
}

/// Validate and decrypt an encrypted frame, returning the padded inner.
///
/// The device uses two checksum conventions: status replies checksum over the
/// ciphertext, message replies over the plaintext. Both are accepted.
pub fn decode_encrypted(key: &[u8; 8], frame: &[u8]) -> Result<Vec<u8>, CodecError> {
    if frame.len() < 3 {
        return Err(CodecError::TooShort(frame.len()));
    }
    if rd_u16(frame) != frame.len() {
        return Err(CodecError::LengthMismatch { field: rd_u16(frame), actual: frame.len() });
    }
    let plen = frame.len() - 3;
    if plen == 0 || plen % 8 != 0 {
        return Err(CodecError::BadCipherLength(plen));
    }

    let wire = frame[frame.len() - 1];
    let cipher_sum = xorsum(&frame[..frame.len() - 1]);

    let mut inner = frame[2..frame.len() - 1].to_vec();
    crypto::decrypt_in_place(key, &mut inner);

    let plain_sum = frame[0] ^ frame[1] ^ xorsum(&inner);
    if wire != cipher_sum && wire != plain_sum {
        return Err(CodecError::BadEncryptedChecksum { wire, cipher: cipher_sum, plain: plain_sum });
    }
    Ok(inner)
}
