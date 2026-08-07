//! Single DES-ECB, as the cable speaks it. The key is the first 8 bytes of
//! the handshake challenge, used as-is — parity bits are not corrected on
//! either end.

use des::Des;
use des::cipher::generic_array::GenericArray;
use des::cipher::{BlockDecrypt, BlockEncrypt, KeyInit};

/// Encrypt `buf` in place. `buf.len()` must be a multiple of 8.
pub fn encrypt_in_place(key: &[u8; 8], buf: &mut [u8]) {
    debug_assert_eq!(buf.len() % 8, 0);
    let des = Des::new(GenericArray::from_slice(key));
    for block in buf.chunks_exact_mut(8) {
        des.encrypt_block(GenericArray::from_mut_slice(block));
    }
}

/// Decrypt `buf` in place. `buf.len()` must be a multiple of 8.
pub fn decrypt_in_place(key: &[u8; 8], buf: &mut [u8]) {
    debug_assert_eq!(buf.len() % 8, 0);
    let des = Des::new(GenericArray::from_slice(key));
    for block in buf.chunks_exact_mut(8) {
        des.decrypt_block(GenericArray::from_mut_slice(block));
    }
}
