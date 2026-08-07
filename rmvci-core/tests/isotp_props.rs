//! Property tests for the sans-IO ISO-TP machines: a TxMachine piped into an
//! RxMachine must reproduce the payload for every payload size / BS / STmin
//! combination, honoring BS exactly; malformed input must error, never panic.

use std::time::{Duration, Instant};

use proptest::prelude::*;
use rmvci_core::error::IsoTpError;
use rmvci_core::isotp::machine::{RxEvent, RxMachine, TxAction, TxMachine, decode_stmin};

const N_BS: Duration = Duration::from_secs(1);
const N_CR: Duration = Duration::from_secs(1);

/// Drive tx against rx in lockstep with a synthetic clock. Returns the
/// reassembled payload and the CF counts between flow controls.
fn pipe(
    payload: &[u8],
    rx_bs: u8,
    rx_stmin: u8,
    padding: Option<u8>,
) -> Result<(Vec<u8>, Vec<usize>), IsoTpError> {
    let mut now = Instant::now();
    let mut tx = TxMachine::new(payload, padding, N_BS, 8)?;
    let mut rx = RxMachine::new(rx_bs, rx_stmin, padding, N_CR);

    let mut result = None;
    let mut cf_counts = Vec::new();
    let mut cfs_in_block = 0usize;
    let mut sent_frames = 0usize;

    loop {
        assert!(sent_frames < 2 * 4096, "machines are looping");
        match tx.next(now)? {
            TxAction::Send(frame) => {
                sent_frames += 1;
                if frame[0] >> 4 == 0x2 {
                    cfs_in_block += 1;
                }
                match rx.on_frame(&frame, now)? {
                    RxEvent::SendFc(fc) => {
                        if cfs_in_block > 0 || frame[0] >> 4 == 0x2 {
                            cf_counts.push(cfs_in_block);
                            cfs_in_block = 0;
                        }
                        tx.on_frame(&fc, now)?;
                    }
                    RxEvent::Done(p) => {
                        result = Some(p);
                    }
                    RxEvent::Continue => {}
                }
            }
            TxAction::WaitUntil(t) => now = t,
            TxAction::WaitFc { .. } => {
                // In lockstep the FC always arrives with the triggering
                // frame; reaching here means the rx machine failed to grant
                // a block. Let the deadline fire so the error surfaces.
                now += N_BS;
            }
            TxAction::Done => break,
        }
    }
    Ok((result.expect("transfer finished without Done"), cf_counts))
}

proptest! {
    /// Round trip for every size class, BS and STmin (reserved values
    /// included — decode_stmin maps them to the 127 ms maximum).
    #[test]
    fn round_trip(
        payload in proptest::collection::vec(any::<u8>(), 1..=1200),
        rx_bs in 0u8..=16,
        rx_stmin in any::<u8>(),
        pad in proptest::option::of(Just(0x00u8)),
    ) {
        let (got, _) = pipe(&payload, rx_bs, rx_stmin, pad).unwrap();
        prop_assert_eq!(got, payload);
    }

    /// The transmitter honors BS exactly: every completed block between two
    /// flow controls contains exactly BS consecutive frames. (The cable
    /// firmware fails this — it waits for FC once and never again.)
    #[test]
    fn block_size_honored(
        payload in proptest::collection::vec(any::<u8>(), 8..=600),
        rx_bs in 1u8..=8,
    ) {
        let (_, cf_counts) = pipe(&payload, rx_bs, 0, Some(0)).unwrap();
        // All blocks except the last must be exactly BS.
        if cf_counts.len() > 1 {
            for c in &cf_counts[..cf_counts.len() - 1] {
                prop_assert_eq!(*c, rx_bs as usize);
            }
        }
        for c in &cf_counts {
            prop_assert!(*c <= rx_bs as usize);
        }
    }

    /// 4095 is the ceiling; larger payloads are refused up front.
    #[test]
    fn payload_ceiling(extra in 1usize..100) {
        let too_big = vec![0u8; 4095 + extra];
        prop_assert!(matches!(
            TxMachine::new(&too_big, Some(0), N_BS, 8),
            Err(IsoTpError::PayloadTooLong(_))
        ));
    }

    /// Arbitrary garbage frames never panic the receiver.
    #[test]
    fn rx_never_panics(
        frames in proptest::collection::vec(
            proptest::collection::vec(any::<u8>(), 0..16), 0..64),
    ) {
        let now = Instant::now();
        let mut rx = RxMachine::new(0, 0, Some(0), N_CR);
        for f in &frames {
            let _ = rx.on_frame(f, now); // Ok or Err, never panic
        }
    }
}

#[test]
fn sequence_error_is_detected() {
    let now = Instant::now();
    let mut rx = RxMachine::new(0, 0, Some(0), N_CR);
    // FF announcing 20 bytes, then a CF with the wrong SN.
    let ff = [0x10, 20, 1, 2, 3, 4, 5, 6];
    assert!(matches!(rx.on_frame(&ff, now), Ok(RxEvent::SendFc(_))));
    let bad_cf = [0x23, 7, 8, 9, 10, 11, 12, 13]; // SN 3, expected 1
    assert!(matches!(
        rx.on_frame(&bad_cf, now),
        Err(IsoTpError::SequenceError { expected: 1, got: 3 })
    ));
}

#[test]
fn wait_and_overflow_flow_statuses() {
    let now = Instant::now();
    let payload = vec![0u8; 64];

    // FS=WAIT is tolerated up to wft_max, then errors.
    let mut tx = TxMachine::new(&payload, Some(0), N_BS, 2).unwrap();
    assert!(matches!(tx.next(now), Ok(TxAction::Send(_)))); // FF
    let wait = [0x31, 0, 0, 0, 0, 0, 0, 0];
    assert!(tx.on_frame(&wait, now).is_ok());
    assert!(tx.on_frame(&wait, now).is_ok());
    assert!(matches!(tx.on_frame(&wait, now), Err(IsoTpError::WaitLimit(2))));

    // FS=OVFLW aborts immediately.
    let mut tx = TxMachine::new(&payload, Some(0), N_BS, 8).unwrap();
    assert!(matches!(tx.next(now), Ok(TxAction::Send(_))));
    let ovflw = [0x32, 0, 0, 0, 0, 0, 0, 0];
    assert!(matches!(tx.on_frame(&ovflw, now), Err(IsoTpError::Overflow)));
}

#[test]
fn stmin_decode_per_spec() {
    assert_eq!(decode_stmin(0x00), Duration::from_millis(0));
    assert_eq!(decode_stmin(0x7f), Duration::from_millis(127));
    assert_eq!(decode_stmin(0xf1), Duration::from_millis(1)); // 100 µs, rounded up
    assert_eq!(decode_stmin(0xf9), Duration::from_millis(1)); // 900 µs
    assert_eq!(decode_stmin(0x80), Duration::from_millis(127)); // reserved
    assert_eq!(decode_stmin(0xfa), Duration::from_millis(127)); // reserved
}
