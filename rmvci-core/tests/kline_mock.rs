//! ISO9141 completeness: `Channel<Iso9141>` now has the same message/init
//! surface as `Channel<Iso14230>` (they share the firmware K-line object).
//! Before this, ISO9141 could connect and filter but not send or init. These
//! byte-exact exchanges prove the typed surface reaches the wire; ISO9141 stays
//! unverified against a real ECU (bench has no K-line peer).

use std::sync::Arc;
use std::time::Duration;

use rmvci_core::codec::{frame, inner};
use rmvci_core::transport::mock::{MockClock, MockTransport, Step};
use rmvci_core::types::ProtocolId;
use rmvci_core::{Device, DeviceConfig, Iso9141, KLineConfig};

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
fn iso9141_typed_channel_writes_and_inits() {
    let msg = [0x68, 0x6a, 0xf1, 0x01, 0x00]; // ISO9141 header + OBD mode 01 PID 00
    // K-line init reply: IOCTL echo + two key bytes (KWP2000 slow-init).
    let init_reply = vec![0x03, 0x00, 0x0e, 0xef, 0x8f];

    let (flags, baud) = (0x1000u32, 10_400u32); // KLineConfig::default()
    let mut steps = handshake_steps();
    steps.push(Step::exchange(enc(&inner::connect(ProtocolId::Iso9141, flags, baud)), status_reply(0x07)));
    steps.push(Step::exchange(enc(&inner::write_msg(ProtocolId::Iso9141, 0, &msg).unwrap()), status_reply(0x0a)));
    steps.push(Step::exchange(enc(&inner::five_baud_init(ProtocolId::Iso9141, &[0x33])), enc(&init_reply)));

    let device = dev(steps);
    let mut chan = device.connect::<Iso9141>(KLineConfig::default()).expect("connect ISO9141");
    chan.write(&msg).expect("write");
    assert_eq!(chan.five_baud_init(&[0x33]).expect("five baud"), vec![0xef, 0x8f]);
}
