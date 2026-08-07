//! In-crate session tests exercising the pub(crate) request surface
//! (connect / poll / disconnect) that `RawChannel` builds on in M3.

use std::sync::Arc;
use std::time::Duration;

use crate::codec::{frame, inner};
use crate::session::{Device, DeviceConfig};
use crate::transport::mock::{MockClock, MockTransport, Step};
use crate::types::ProtocolId;

const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];

fn challenge_frame() -> Vec<u8> {
    vec![0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9]
}

fn handshake_steps() -> Vec<Step> {
    vec![
        Step::exchange(frame::encode_plain(&[]).unwrap(), Vec::new()),
        Step::exchange(frame::encode_plain(&inner::identify()).unwrap(), challenge_frame()),
    ]
}

fn enc(inner_bytes: &[u8]) -> Vec<u8> {
    frame::encode_encrypted(&KEY_OLD, inner_bytes).unwrap()
}

/// Status reply `[02 00 CMD status]` as the wire frame.
fn status_reply(cmd: u8, status: u8) -> Vec<u8> {
    enc(&[0x02, 0x00, cmd, status])
}

fn cfg(keepalive_ms: u64) -> DeviceConfig {
    DeviceConfig {
        port: None,
        keepalive: Some(Duration::from_millis(keepalive_ms)),
        clock: Arc::new(MockClock::default()),
    }
}

#[test]
fn connect_success_and_teardown_order() {
    let connect_frame = enc(&inner::connect(ProtocolId::Iso15765, 0, 500_000));
    let disconnect_frame = enc(&inner::disconnect(ProtocolId::Iso15765));
    let close_frame = enc(&inner::session_close());

    let mut steps = handshake_steps();
    steps.push(Step::exchange(connect_frame, status_reply(0x07, 0x00)));
    steps.push(Step::exchange(disconnect_frame.clone(), status_reply(0x08, 0x00)));
    steps.push(Step::exchange(close_frame.clone(), Vec::new()));
    let mock = MockTransport::new(steps);
    let events = mock.events();

    let dev = Device::open_transport(mock, cfg(60_000)).expect("open");
    dev.connect_proto(ProtocolId::Iso15765, 0, 500_000).expect("connect");
    drop(dev);

    // Teardown must disconnect the channel (0x08) before closing the
    // encrypted session (0x02).
    for _ in 0..100 {
        let writes = MockTransport::writes(&events);
        let di = writes.iter().position(|w| *w == disconnect_frame);
        let ci = writes.iter().position(|w| *w == close_frame);
        if let (Some(di), Some(ci)) = (di, ci) {
            assert!(di < ci, "disconnect must precede session close");
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("teardown frames never sent: {:02x?}", MockTransport::writes(&events));
}

/// After a prior Disconnect the adapter ignores Connect until it is reset;
/// the actor must re-run the handshake and retry once (serial.c:427).
#[test]
fn connect_retries_after_rehandshake() {
    let connect_frame = enc(&inner::connect(ProtocolId::Iso14230, 4096, 10400));

    let mut steps = handshake_steps();
    // First attempt: rejected with ERR_FAILED.
    steps.push(Step::exchange(connect_frame.clone(), status_reply(0x07, 0x07)));
    // The actor re-handshakes (reset + identify -> same key here)...
    steps.extend(handshake_steps());
    // ...and the retry succeeds.
    steps.push(Step::exchange(connect_frame, status_reply(0x07, 0x00)));

    let dev = Device::open_transport(MockTransport::new(steps), cfg(60_000)).expect("open");
    dev.connect_proto(ProtocolId::Iso14230, 4096, 10400)
        .expect("connect should succeed after rehandshake");
}

/// A message the keepalive happened to drain must reach the next poll, not
/// the floor (the C driver discarded keepalive replies wholesale).
#[test]
fn keepalive_preserves_drained_data() {
    let msg = [0xde, 0xad, 0xbe, 0xef];
    let read_reply: Vec<u8> = {
        let mut v = Vec::new();
        v.extend_from_slice(&((9 + msg.len()) as u16).to_le_bytes());
        v.push(0x09);
        v.extend_from_slice(&[0; 8]); // RxStatus 0 + reserved
        v.extend_from_slice(&msg);
        v
    };

    let mut steps = handshake_steps();
    steps.push(Step::reply_any(enc(&read_reply)));
    for _ in 0..16 {
        steps.push(Step::reply_any(enc(&[0x02, 0x00, 0x09, 0x10])));
    }

    let dev = Device::open_transport(MockTransport::new(steps), cfg(15)).expect("open");
    std::thread::sleep(Duration::from_millis(80));

    let got = dev
        .poll_proto(ProtocolId::Iso15765, Duration::from_millis(100))
        .expect("poll")
        .expect("the drained message must be queued");
    assert_eq!(got.data, msg);
}

#[test]
fn poll_empty_returns_none() {
    let mut steps = handshake_steps();
    steps.push(Step::exchange(
        enc(&inner::read_poll(ProtocolId::Iso15765)),
        enc(&[0x02, 0x00, 0x09, 0x10]), // ERR_BUFFER_EMPTY status
    ));
    let dev = Device::open_transport(MockTransport::new(steps), cfg(60_000)).expect("open");
    let got = dev.poll_proto(ProtocolId::Iso15765, Duration::from_millis(100)).expect("poll");
    assert_eq!(got, None);
}
