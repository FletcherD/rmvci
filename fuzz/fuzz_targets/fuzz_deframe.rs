//! The deframer sees raw bytes off a serial port: line noise, truncated
//! frames, a wedged adapter's stack garbage. It must never panic or allocate
//! unboundedly, whatever arrives.
#![no_main]

use libfuzzer_sys::fuzz_target;
use rmvci_core::codec::Deframer;

fuzz_target!(|data: &[u8]| {
    let mut d = Deframer::new();
    // Feed in small chunks so partial-frame paths get exercised.
    for chunk in data.chunks(7.max(1)) {
        d.push(chunk);
        loop {
            match d.next_frame() {
                Ok(Some(_)) => continue,
                Ok(None) => break,
                Err(_) => {
                    d.clear(); // the session layer's resync path
                    break;
                }
            }
        }
    }
});
