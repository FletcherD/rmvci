//! Frame decryption runs on every reply from the adapter, including from a
//! confused or wedged one. Arbitrary bytes under an arbitrary key must
//! produce an error, never a panic.
#![no_main]

use libfuzzer_sys::fuzz_target;
use rmvci_core::codec::frame;

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let key: [u8; 8] = data[..8].try_into().unwrap();
    let rest = &data[8..];
    let _ = frame::decode_encrypted(&key, rest);
    let _ = frame::decode_plain(rest);
});
