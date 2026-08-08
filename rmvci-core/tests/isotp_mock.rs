//! End-to-end ISO-TP client tests over MockTransport — the offline version
//! of the bench scenario (`re/bench/isotp_responder.py`): `21 43` single
//! frame and `21 44` 40-byte multi-frame, on both the host path (raw CAN +
//! host machines) and the firmware path (protocol 6).

use std::sync::Arc;
use std::time::Duration;

use rmvci_core::codec::{frame, inner};
use rmvci_core::transport::mock::{MockClock, MockTransport, Step};
use rmvci_core::types::ProtocolId;
use rmvci_core::{
    CanId, Device, DeviceConfig, Error, FirmwareIsoTp, IsoTp, IsoTpConfig, IsoTpError,
};

const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];
const TX_ID: [u8; 4] = [0x00, 0x00, 0x07, 0xc4];
const RX_ID: [u8; 4] = [0x00, 0x00, 0x07, 0xcc];

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
            vec![
                0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9,
            ],
        ),
    ]
}

/// A read-poll exchange delivering one message (id + frame bytes),
/// optionally flagged with an RxStatus.
fn poll_delivers(proto: ProtocolId, rx_status: u32, data: &[u8]) -> Step {
    let mut reply = Vec::new();
    reply.extend_from_slice(&((9 + data.len()) as u16).to_le_bytes());
    reply.push(0x09);
    reply.extend_from_slice(&rx_status.to_le_bytes());
    reply.extend_from_slice(&[0; 4]);
    reply.extend_from_slice(data);
    Step::exchange(enc(&inner::read_poll(proto)), enc(&reply))
}

fn write_ok(proto: ProtocolId, txflags: u32, msg: &[u8]) -> Step {
    Step::exchange(enc(&inner::write_msg(proto, txflags, msg).unwrap()), status_reply(0x0a))
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

/// The bench responder's 40-byte answer to `21 44`.
fn multiframe_payload() -> Vec<u8> {
    let mut p = vec![0x61, 0x44];
    p.extend((0..38).map(|i| i as u8));
    p
}

#[test]
fn host_path_single_and_multi_frame() {
    let payload = multiframe_payload();

    let mut steps = handshake_steps();
    // connect raw CAN @ 500k + exact PASS filter on 7CC.
    steps.push(Step::exchange(
        enc(&inner::connect(ProtocolId::Can, 0, 500_000)),
        status_reply(0x07),
    ));
    steps.push(Step::exchange(
        enc(&inner::start_filter(ProtocolId::Can, 0x000e_7a00, 1, &[0xff; 4], &RX_ID, None, 4)
            .unwrap()),
        status_reply(0x0b),
    ));

    // --- request 1: 21 43, single-frame both ways ---
    steps.push(write_ok(ProtocolId::Can, 0, &{
        let mut m = TX_ID.to_vec();
        m.extend_from_slice(&[0x02, 0x21, 0x43, 0, 0, 0, 0, 0]);
        m
    }));
    steps.push(poll_delivers(ProtocolId::Can, 0, &{
        let mut m = RX_ID.to_vec();
        m.extend_from_slice(&[0x04, 0x61, 0x43, 0x7b, 0x79, 0, 0, 0]);
        m
    }));

    // --- request 2: 21 44 -> FF + FC + 5 CFs (40 bytes) ---
    steps.push(write_ok(ProtocolId::Can, 0, &{
        let mut m = TX_ID.to_vec();
        m.extend_from_slice(&[0x02, 0x21, 0x44, 0, 0, 0, 0, 0]);
        m
    }));
    // First frame: FF_DL 40, first 6 payload bytes.
    steps.push(poll_delivers(ProtocolId::Can, 0, &{
        let mut m = RX_ID.to_vec();
        m.push(0x10);
        m.push(40);
        m.extend_from_slice(&payload[..6]);
        m
    }));
    // Our flow control: CTS, BS 0, STmin 0.
    steps.push(write_ok(ProtocolId::Can, 0, &{
        let mut m = TX_ID.to_vec();
        m.extend_from_slice(&[0x30, 0, 0, 0, 0, 0, 0, 0]);
        m
    }));
    // Five consecutive frames.
    let mut off = 6;
    for sn in 1..=5u8 {
        let take = (payload.len() - off).min(7);
        steps.push(poll_delivers(ProtocolId::Can, 0, &{
            let mut m = RX_ID.to_vec();
            m.push(0x20 | sn);
            m.extend_from_slice(&payload[off..off + take]);
            m.resize(4 + 8, 0);
            m
        }));
        off += take;
    }

    let device = dev(steps);
    let mut tp = IsoTp::new(&device, IsoTpConfig::new(CanId::Std(0x7c4), CanId::Std(0x7cc)))
        .expect("isotp channel");

    let r1 = tp.request(&[0x21, 0x43], Duration::from_secs(2)).expect("21 43");
    assert_eq!(r1, [0x61, 0x43, 0x7b, 0x79]);

    let r2 = tp.request(&[0x21, 0x44], Duration::from_secs(2)).expect("21 44");
    assert_eq!(r2, payload);
}

#[test]
fn firmware_path_reply_and_ffdl_guard() {
    let mut steps = handshake_steps();
    steps.push(Step::exchange(
        enc(&inner::connect(ProtocolId::Iso15765, 0, 500_000)),
        status_reply(0x07),
    ));
    steps.push(Step::exchange(
        enc(&inner::start_filter(
            ProtocolId::Iso15765,
            0x000e_7a00,
            3,
            &[0xff; 4],
            &RX_ID,
            Some(&TX_ID),
            4,
        )
        .unwrap()),
        status_reply(0x0b),
    ));
    steps.push(write_ok(ProtocolId::Iso15765, 0, &{
        let mut m = TX_ID.to_vec();
        m.extend_from_slice(&[0x21, 0x43]);
        m
    }));
    // The adapter first reports the start-of-message indication (RxStatus
    // 0x02, identifier only), then the reassembled reply.
    steps.push(poll_delivers(ProtocolId::Iso15765, 0x0002, &RX_ID));
    steps.push(poll_delivers(ProtocolId::Iso15765, 0, &{
        let mut m = RX_ID.to_vec();
        m.extend_from_slice(&[0x61, 0x43, 0x7b, 0x79]);
        m
    }));

    let device = dev(steps);
    let mut tp = FirmwareIsoTp::new(&device, CanId::Std(0x7c4), CanId::Std(0x7cc))
        .expect("firmware isotp channel");

    let r = tp.request(&[0x21, 0x43], Duration::from_secs(2)).expect("21 43");
    assert_eq!(r, [0x61, 0x43, 0x7b, 0x79]);

    // >255 bytes would leave the firmware as a malformed First Frame; the
    // driver must refuse it client-side without touching the wire.
    let big = vec![0u8; 256];
    match tp.send(&big) {
        Err(Error::IsoTp(IsoTpError::FirmwareFfDlLimit(256))) => {}
        other => panic!("expected FirmwareFfDlLimit, got {other:?}"),
    }
}

/// Extended/mixed addressing on the firmware path: the acceptance filter is
/// 5 bytes wide (idlen=5, address in the 5th), the transmitted message carries
/// the address byte at msg[4] with TxFlags ISO15765_ADDR_TYPE (0x80), and the
/// reply — flagged RxStatus 0x80 — has its leading address byte stripped.
#[test]
fn firmware_path_extended_addressing() {
    const ADDR: u8 = 0xf1;
    let mask5 = [0xff, 0xff, 0xff, 0xff, 0xff];
    let pattern5 = [0x00, 0x00, 0x07, 0xcc, ADDR]; // rx id + address
    let flow5 = [0x00, 0x00, 0x07, 0xc4, ADDR]; // tx id + address

    let mut steps = handshake_steps();
    steps.push(Step::exchange(
        enc(&inner::connect(ProtocolId::Iso15765, 0, 500_000)),
        status_reply(0x07),
    ));
    // 5-byte flow-control filter (idlen=5).
    steps.push(Step::exchange(
        enc(&inner::start_filter(
            ProtocolId::Iso15765,
            0x000e_7a00,
            3,
            &mask5,
            &pattern5,
            Some(&flow5),
            5,
        )
        .unwrap()),
        status_reply(0x0b),
    ));
    // TX: address byte at msg[4], TxFlags = ISO15765_ADDR_TYPE (0x80).
    steps.push(write_ok(ProtocolId::Iso15765, 0x80, &{
        let mut m = TX_ID.to_vec();
        m.extend_from_slice(&[ADDR, 0x21, 0x43]);
        m
    }));
    // Start-of-message indication, then the addr-prefixed reply flagged 0x80.
    steps.push(poll_delivers(ProtocolId::Iso15765, 0x0002, &RX_ID));
    steps.push(poll_delivers(ProtocolId::Iso15765, 0x0080, &{
        let mut m = RX_ID.to_vec();
        m.extend_from_slice(&[ADDR, 0x61, 0x43, 0x7b, 0x79]);
        m
    }));

    let device = dev(steps);
    let mut tp = FirmwareIsoTp::with_ext_addr(&device, CanId::Std(0x7c4), CanId::Std(0x7cc), ADDR)
        .expect("firmware ext-addr channel");
    let r = tp.request(&[0x21, 0x43], Duration::from_secs(2)).expect("21 43");
    assert_eq!(r, [0x61, 0x43, 0x7b, 0x79]);
}

/// The same FF_DL>255 guard must also protect the raw `PassThruWriteMsgs` path
/// (a J2534 app writing an ISO15765 message directly, bypassing FirmwareIsoTp).
/// The guard fires before any wire traffic, so no write Step is scripted.
#[test]
fn raw_write_guards_ffdl_over_255() {
    let mut steps = handshake_steps();
    steps.push(Step::exchange(
        enc(&inner::connect(ProtocolId::Iso15765, 0, 500_000)),
        status_reply(0x07),
    ));

    let device = dev(steps);
    let mut chan = device.connect_raw(ProtocolId::Iso15765, 0, 500_000).expect("connect");

    // 4-byte id + 256-byte payload -> payload 256 > 255.
    let mut msg = TX_ID.to_vec();
    msg.extend(std::iter::repeat_n(0u8, 256));
    match chan.write(0, &msg) {
        Err(Error::IsoTp(IsoTpError::FirmwareFfDlLimit(256))) => {}
        other => panic!("expected FirmwareFfDlLimit, got {other:?}"),
    }
}
