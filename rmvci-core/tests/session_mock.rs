//! Session-layer tests against the scripted MockTransport: the open/handshake
//! sequence, firmware version readout, keepalive cadence and the wedge
//! detector. All expected wire bytes are computed with the codec (which is
//! itself pinned by the captured vectors), so these tests assert byte-exact
//! traffic without hardware.

use std::sync::Arc;
use std::time::Duration;

use rmvci_core::codec::{frame, inner};
use rmvci_core::transport::mock::{Event, MockClock, MockTransport, Step};
use rmvci_core::types::ProtocolId;
use rmvci_core::{Device, DeviceConfig, Error};

const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];

fn challenge_frame() -> Vec<u8> {
    vec![0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9]
}

fn reset_frame() -> Vec<u8> {
    frame::encode_plain(&[]).unwrap()
}

fn identify_frame() -> Vec<u8> {
    frame::encode_plain(&inner::identify()).unwrap()
}

fn handshake_steps() -> Vec<Step> {
    vec![
        Step::exchange(reset_frame(), Vec::new()),
        Step::exchange(identify_frame(), challenge_frame()),
    ]
}

/// A status reply to a READ_MSG poll: nothing queued (ERR_BUFFER_EMPTY).
fn ka_ack() -> Vec<u8> {
    frame::encode_encrypted(&KEY_OLD, &[0x02, 0x00, 0x09, 0x10]).unwrap()
}

fn cfg(keepalive_ms: u64) -> DeviceConfig {
    DeviceConfig {
        port: None,
        keepalive: Duration::from_millis(keepalive_ms),
        clock: Arc::new(MockClock::default()),
    }
}

#[test]
fn open_runs_reset_dance_then_handshake() {
    let mock = MockTransport::new(handshake_steps());
    let events = mock.events();

    let dev = Device::open_transport(mock, cfg(60_000)).expect("open");
    assert_eq!(dev.des_key(), KEY_OLD);

    let ev = events.lock().unwrap().clone();
    // DTR/RTS dance: RTS clear + DTR set, then DTR clear, then purge.
    assert_eq!(ev[0], Event::Modem { dtr: true, rts: false });
    assert_eq!(ev[1], Event::Modem { dtr: false, rts: false });
    assert_eq!(ev[2], Event::PurgeRx);
    // Handshake: purge, reset frame, purge, identify.
    assert_eq!(ev[3], Event::PurgeRx);
    assert_eq!(ev[4], Event::Write(reset_frame()));
    assert_eq!(ev[5], Event::PurgeRx);
    assert_eq!(ev[6], Event::Write(identify_frame()));
    assert_eq!(ev.len(), 7, "no extra traffic before any request: {ev:?}");
}

#[test]
fn firmware_version_round_trip() {
    let version_inner: Vec<u8> = {
        let s = b"J2534 MINIV1.03";
        let mut v = Vec::new();
        v.extend_from_slice(&((1 + s.len()) as u16).to_le_bytes());
        v.push(0x03);
        v.extend_from_slice(s);
        v
    };
    let mut steps = handshake_steps();
    steps.push(Step::exchange(
        frame::encode_encrypted(&KEY_OLD, &inner::read_version()).unwrap(),
        frame::encode_encrypted(&KEY_OLD, &version_inner).unwrap(),
    ));

    let dev = Device::open_transport(MockTransport::new(steps), cfg(60_000)).expect("open");
    assert_eq!(dev.firmware_version().unwrap(), "J2534 MINIV1.03");
}

#[test]
fn keepalive_fires_when_idle_with_exact_bytes() {
    // 20 ms idle threshold; the scripted replies keep the link healthy.
    let mut steps = handshake_steps();
    for _ in 0..32 {
        steps.push(Step::reply_any(ka_ack()));
    }
    let mock = MockTransport::new(steps);
    let events = mock.events();

    let _dev = Device::open_transport(mock, cfg(20)).expect("open");
    std::thread::sleep(Duration::from_millis(150));

    let writes = MockTransport::writes(&events);
    // Writes 0 and 1 are the handshake; everything after must be keepalives.
    assert!(writes.len() >= 5, "expected several keepalives, saw {} writes", writes.len());
    // No channel connected -> the keepalive polls ISO15765, and under KEY_OLD
    // that is exactly the captured keepalive vector from mvci_test.c:117.
    let expected =
        frame::encode_encrypted(&KEY_OLD, &inner::read_poll(ProtocolId::Iso15765)).unwrap();
    assert_eq!(expected, [0x0b, 0x00, 0x31, 0x18, 0x19, 0x2b, 0x97, 0x53, 0x24, 0xce, 0x3e]);
    for w in &writes[2..] {
        assert_eq!(w, &expected);
    }
}

#[test]
fn silent_adapter_is_declared_wedged_and_fails_fast() {
    // Handshake succeeds, then the adapter never answers again.
    let dev =
        Device::open_transport(MockTransport::new(handshake_steps()), cfg(15)).expect("open");

    // Three failed keepalives are needed; give them time to happen.
    std::thread::sleep(Duration::from_millis(200));

    match dev.firmware_version() {
        Err(Error::Wedged(n)) => assert!(n >= 3, "wedge counter should be >= 3, got {n}"),
        other => panic!("expected Error::Wedged, got {other:?}"),
    }
}

#[test]
fn drop_closes_the_session() {
    let mut steps = handshake_steps();
    // Teardown with no connected channels: session close (01 00 02 ...).
    steps.push(Step::reply_any(Vec::new()));
    let mock = MockTransport::new(steps);
    let events = mock.events();

    let dev = Device::open_transport(mock, cfg(60_000)).expect("open");
    drop(dev);

    // The actor tears down asynchronously; wait for the write to appear.
    let close_frame = frame::encode_encrypted(&KEY_OLD, &inner::session_close()).unwrap();
    for _ in 0..100 {
        if MockTransport::writes(&events).contains(&close_frame) {
            return;
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    panic!("session close frame never sent; writes: {:02x?}", MockTransport::writes(&events));
}
