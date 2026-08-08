//! The extended/mixed-addressing receive path adds an address byte ahead of
//! every PCI and shifts the full-frame length checks by one. Arbitrary frames
//! — a leading byte that does or doesn't match the configured address, frames
//! too short to hold the address byte, CFs without an FF — must error out,
//! never panic or overflow the reassembly buffer.
#![no_main]

use std::time::{Duration, Instant};

use libfuzzer_sys::fuzz_target;
use rmvci_core::isotp::machine::RxMachine;

fuzz_target!(|data: &[u8]| {
    // First byte seeds the configured address; the rest are 8-byte CAN frames.
    let (addr, frames) = data.split_first().map_or((0u8, &[][..]), |(a, r)| (*a, r));
    let now = Instant::now();
    let mk = || RxMachine::with_addr(0, 0, Some(0), Duration::from_secs(1), Some(addr));
    let mut rx = mk();
    for frame in frames.chunks(8) {
        if rx.on_frame(frame, now).is_err() {
            rx = mk(); // a real driver aborts and starts over
        }
    }
});
