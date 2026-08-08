//! START/STOP_PERIODIC_MSG (0x0F/0x10). There is no captured vendor vector for
//! periodic messages, so these tests pin the exact ARGS layout from the
//! firmware RE (`re/FINDINGS.md` §6: ARGS 16+N = proto, MsgID, TxFlags,
//! interval, data) and drive a full round trip over MockTransport. The MsgID
//! is host-assigned, so the driver sends the id it was given verbatim.

use std::sync::Arc;
use std::time::Duration;

use rmvci_core::codec::{frame, inner};
use rmvci_core::transport::mock::{MockClock, MockTransport, Step};
use rmvci_core::types::ProtocolId;
use rmvci_core::{Device, DeviceConfig};

const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];

fn enc(inner_bytes: &[u8]) -> Vec<u8> {
    frame::encode_encrypted(&KEY_OLD, inner_bytes).unwrap()
}

fn status_reply(cmd: u8) -> Vec<u8> {
    enc(&[0x02, 0x00, cmd, 0x00])
}

fn handshake_steps() -> Vec<Step> {
    vec![
        Step::exchange(frame::encode_plain(&[]).unwrap(), Vec::new()),
        Step::exchange(
            frame::encode_plain(&inner::identify()).unwrap(),
            vec![0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9],
        ),
    ]
}

fn dev(steps: Vec<Step>) -> Device {
    Device::open_transport(
        MockTransport::new(steps),
        DeviceConfig {
            port: None,
            keepalive: Some(Duration::from_secs(600)),
            clock: Arc::new(MockClock::default()),
        },
    )
    .expect("open")
}

#[test]
fn start_periodic_wire_layout() {
    // CAN periodic: id 7DF + payload `02 01 00` (OBD mode 01 PID 00), 500 ms.
    let msg = [0x00, 0x00, 0x07, 0xdf, 0x02, 0x01, 0x00];
    let got = inner::start_periodic(ProtocolId::Can, 0x000e_7b00, 0, 500, &msg).unwrap();
    #[rustfmt::skip]
    let exp = [
        0x18, 0x00,             // ILEN = 1 + 16 + 7 = 24
        0x0f,                   // CMD START_PERIODIC_MSG
        0x05, 0x00, 0x00, 0x00, // proto = CAN
        0x00, 0x7b, 0x0e, 0x00, // MsgID (host-assigned) = 0x000e7b00
        0x00, 0x00, 0x00, 0x00, // TxFlags
        0xf4, 0x01, 0x00, 0x00, // interval = 500 ms
        0x00, 0x00, 0x07, 0xdf, 0x02, 0x01, 0x00, // message
    ];
    assert_eq!(got, exp);
}

#[test]
fn stop_periodic_wire_layout() {
    let got = inner::stop_periodic(ProtocolId::Can, 0x000e_7b00);
    #[rustfmt::skip]
    let exp = [
        0x0d, 0x00,             // ILEN = 13
        0x10,                   // CMD STOP_PERIODIC_MSG
        0x05, 0x00, 0x00, 0x00, // proto = CAN
        0x00, 0x7b, 0x0e, 0x00, // MsgID
        0x00, 0x00, 0x00, 0x00, // unused
    ];
    assert_eq!(got, exp);
}

#[test]
fn periodic_round_trip_over_mock() {
    let msg = [0x00, 0x00, 0x07, 0xdf, 0x02, 0x01, 0x00];
    let mut steps = handshake_steps();
    steps.push(Step::exchange(enc(&inner::connect(ProtocolId::Can, 0, 500_000)), status_reply(0x07)));
    steps.push(Step::exchange(
        enc(&inner::start_periodic(ProtocolId::Can, 0x000e_7b00, 0, 500, &msg).unwrap()),
        status_reply(0x0f),
    ));
    steps.push(Step::exchange(
        enc(&inner::stop_periodic(ProtocolId::Can, 0x000e_7b00)),
        status_reply(0x10),
    ));

    let device = dev(steps);
    let mut chan = device.connect_raw(ProtocolId::Can, 0, 500_000).expect("connect");
    chan.start_periodic(0x000e_7b00, 0, 500, &msg).expect("start periodic");
    chan.stop_periodic(0x000e_7b00).expect("stop periodic");
}
