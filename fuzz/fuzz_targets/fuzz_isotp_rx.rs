//! The ISO-TP receiver consumes frames straight off the bus. Reordered,
//! truncated and contradictory sequences (an FF_DL that never arrives, CFs
//! without an FF, absurd sequence numbers) must error out, never panic or
//! grow the reassembly buffer without bound.
#![no_main]

use std::time::Instant;

use libfuzzer_sys::fuzz_target;
use rmvci_core::isotp::machine::RxMachine;

fuzz_target!(|data: &[u8]| {
    let now = Instant::now();
    let mut rx = RxMachine::new(0, 0, Some(0), std::time::Duration::from_secs(1));
    // Treat the input as a sequence of 8-byte CAN frames.
    for frame in data.chunks(8) {
        if rx.on_frame(frame, now).is_err() {
            // A real driver aborts the transfer and starts over.
            rx = RxMachine::new(0, 0, Some(0), std::time::Duration::from_secs(1));
        }
    }
});
