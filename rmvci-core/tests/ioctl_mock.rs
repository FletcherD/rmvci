//! GET_CONFIG (ioctl sub 1) and FIVE_BAUD_INIT (ioctl sub 4). No captured
//! vendor vector exists for either, so these pin the ARGS layout from the
//! firmware RE and drive a round trip over MockTransport. The GET_CONFIG
//! *reply* layout is RE-derived and flagged unverified in the code; the test
//! locks in the layout the driver currently assumes so a bench correction is a
//! deliberate, visible change.

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
fn get_config_wire_layout() {
    let got = inner::get_config(ProtocolId::Iso14230, 0x01); // DATA_RATE
    #[rustfmt::skip]
    let exp = [
        0x0a, 0x00,             // ILEN = 1 + 9 = 10
        0x0e,                   // CMD IOCTL
        0x01,                   // sub GET_CONFIG
        0x04, 0x00, 0x00, 0x00, // proto = ISO14230
        0x01, 0x00, 0x00, 0x00, // param = DATA_RATE
    ];
    assert_eq!(got, exp);
}

#[test]
fn five_baud_init_wire_layout() {
    let got = inner::five_baud_init(ProtocolId::Iso9141, &[0x33]);
    #[rustfmt::skip]
    let exp = [
        0x07, 0x00,             // ILEN = 1 + 5 + 1 = 7
        0x0e,                   // CMD IOCTL
        0x04,                   // sub FIVE_BAUD_INIT
        0x03, 0x00, 0x00, 0x00, // proto = ISO9141
        0x33,                   // init address
    ];
    assert_eq!(got, exp);
}

#[test]
fn get_config_round_trip_over_mock() {
    // Assumed reply layout: [ILEN][0x0e][value u32 LE]. value = 500000.
    let value = 500_000u32;
    let mut reply = vec![0x05, 0x00, 0x0e];
    reply.extend_from_slice(&value.to_le_bytes());

    let mut steps = handshake_steps();
    steps.push(Step::exchange(enc(&inner::connect(ProtocolId::Can, 0, 500_000)), status_reply(0x07)));
    steps.push(Step::exchange(enc(&inner::get_config(ProtocolId::Can, 0x01)), enc(&reply)));

    let device = dev(steps);
    let mut chan = device.connect_raw(ProtocolId::Can, 0, 500_000).expect("connect");
    assert_eq!(chan.get_config(0x01).expect("get_config"), value);
}

#[test]
fn read_vbatt_wire_layout() {
    let got = inner::read_vbatt(ProtocolId::Can);
    #[rustfmt::skip]
    let exp = [
        0x06, 0x00,             // ILEN = 1 + 5 = 6
        0x0e,                   // CMD IOCTL
        0x03,                   // sub READ_VBATT
        0x05, 0x00, 0x00, 0x00, // proto = CAN
    ];
    assert_eq!(got, exp);
}

#[test]
fn read_vbatt_round_trip_over_mock() {
    // Assumed reply layout: [ILEN][0x0e][value u32 LE]. Mini-VCI answers 12000 mV.
    let mut reply = vec![0x05, 0x00, 0x0e];
    reply.extend_from_slice(&12_000u32.to_le_bytes());

    let mut steps = handshake_steps();
    steps.push(Step::exchange(enc(&inner::connect(ProtocolId::Can, 0, 500_000)), status_reply(0x07)));
    steps.push(Step::exchange(enc(&inner::read_vbatt(ProtocolId::Can)), enc(&reply)));

    let device = dev(steps);
    let mut chan = device.connect_raw(ProtocolId::Can, 0, 500_000).expect("connect");
    assert_eq!(chan.read_vbatt().expect("read_vbatt"), 12_000);
}

#[test]
fn set_programming_voltage_wire_layout() {
    let got = inner::set_programming_voltage(1, 5000);
    #[rustfmt::skip]
    let exp = [
        0x09, 0x00,             // ILEN = 1 + 8 = 9
        0x0d,                   // CMD SET_PROGRAMMING_VOLTAGE (device-scoped, no proto)
        0x01, 0x00, 0x00, 0x00, // pin number
        0x88, 0x13, 0x00, 0x00, // 5000 mV
    ];
    assert_eq!(got, exp);
}

#[test]
fn set_programming_voltage_round_trip_over_mock() {
    // Device-scoped: no channel connect needed.
    let mut steps = handshake_steps();
    steps.push(Step::exchange(enc(&inner::set_programming_voltage(1, 5000)), status_reply(0x0d)));

    let device = dev(steps);
    device.set_programming_voltage(1, 5000).expect("set programming voltage");
}

#[test]
fn five_baud_init_round_trip_over_mock() {
    // K-line init reply: IOCTL echo + two key bytes.
    let reply = vec![0x03, 0x00, 0x0e, 0xef, 0x8f];

    let mut steps = handshake_steps();
    steps.push(Step::exchange(
        enc(&inner::connect(ProtocolId::Iso9141, 0x1000, 10_400)),
        status_reply(0x07),
    ));
    steps.push(Step::exchange(enc(&inner::five_baud_init(ProtocolId::Iso9141, &[0x33])), enc(&reply)));

    let device = dev(steps);
    let mut chan = device.connect_raw(ProtocolId::Iso9141, 0x1000, 10_400).expect("connect");
    assert_eq!(chan.five_baud_init(&[0x33]).expect("five baud"), vec![0xef, 0x8f]);
}
