//! The 24 self-test vectors from libMVCI's `test/mvci_test.c` `selftest()`,
//! ported byte-exact. The first 16 are real bytes captured from the vendor
//! `MVCI32.dll` (serial capture + a Frida session), so a pass means this
//! codec is byte-for-byte identical to the vendor DLL on the wire. The last 8
//! are CAN-era vectors written against the firmware RE and bench-validated.
//!
//! Each test cites its source line in mvci_test.c. Do not "fix" expected
//! bytes: they are the backward-compatibility guarantee.

// The leading "N." in each doc comment is the C suite's vector number, not a
// markdown list.
#![allow(clippy::doc_lazy_continuation)]

use rmvci_core::codec::{Deframer, crypto, frame, inner};
use rmvci_core::types::{ProtocolId, RxStatus};

/// old-session key (challenge b0 cb 49 68 07 45 c8 7f a9) — mvci_test.c:46
const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];
/// frida-session key (challenge 13 c4 6c 2b ba 61 65 a1 89) — mvci_test.c:48
const KEY_NEW: [u8; 8] = [0x13, 0xc4, 0x6c, 0x2b, 0xba, 0x61, 0x65, 0xa1];

/// 1. reset frame — mvci_test.c:58
#[test]
fn reset_frame() {
    assert_eq!(frame::encode_plain(&[]).unwrap(), [0x03, 0x00, 0x03]);
}

/// 2. identify frame ('b' == xorsum) — mvci_test.c:64
#[test]
fn identify_frame() {
    let exp = [0x0c, 0x00, 0x07, 0x00, 0x01, 0x4d, 0x56, 0x43, 0x49, 0x2d, 0x54, 0x62];
    assert_eq!(frame::encode_plain(&inner::identify()).unwrap(), exp);
}

/// 3. parse challenge -> recover key (and validate checksum) — mvci_test.c:73
#[test]
fn parse_challenge() {
    let chal = [
        0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9,
    ];
    let payload = frame::decode_plain(&chal).unwrap();
    assert_eq!(payload.len(), 11);
    assert_eq!(inner::parse_challenge(payload).unwrap(), KEY_OLD);
}

/// 4. SET_CONFIG(param=7,value=0), KEY_OLD -> real wire frame — mvci_test.c:84
#[test]
fn set_config_7_0_wire_old_key() {
    let exp = [
        0x13, 0x00, 0x1a, 0x7c, 0xef, 0xa7, 0x56, 0x16, 0x8c, 0xbc, 0x3f, 0x7d, 0x9a, 0x06,
        0x8e, 0x58, 0x87, 0xe6, 0x24,
    ];
    let i = inner::set_config(ProtocolId::Iso14230, 7, 0);
    assert_eq!(frame::encode_encrypted(&KEY_OLD, &i).unwrap(), exp);
}

/// 5. SET_CONFIG(param=7,value=1), KEY_OLD — mvci_test.c:94
#[test]
fn set_config_7_1_wire_old_key() {
    let exp = [
        0x13, 0x00, 0x1a, 0x7c, 0xef, 0xa7, 0x56, 0x16, 0x8c, 0xbc, 0x0a, 0xbd, 0x17, 0x8f,
        0xd9, 0x48, 0xcf, 0x57, 0x6b,
    ];
    let i = inner::set_config(ProtocolId::Iso14230, 7, 1);
    assert_eq!(frame::encode_encrypted(&KEY_OLD, &i).unwrap(), exp);
}

/// 6. SET_CONFIG(param=7,value=0), KEY_NEW (frida capture) — mvci_test.c:104
#[test]
fn set_config_7_0_wire_new_key() {
    // DES(0e000e0204000000)=6fa4af0b4b42d695 ; DES(0700000000000000)=c86c26f57b63d494
    let body = [
        0x6f, 0xa4, 0xaf, 0x0b, 0x4b, 0x42, 0xd6, 0x95, 0xc8, 0x6c, 0x26, 0xf5, 0x7b, 0x63,
        0xd4, 0x94,
    ];
    let exp = frame::encode_plain(&body).unwrap();
    let i = inner::set_config(ProtocolId::Iso14230, 7, 0);
    assert_eq!(frame::encode_encrypted(&KEY_NEW, &i).unwrap(), exp);
}

/// 7. keepalive build (inner 05 00 09 06 00 00 00 00, KEY_OLD) — mvci_test.c:117
#[test]
fn keepalive_wire_old_key() {
    let ka = inner::read_poll(ProtocolId::Iso15765);
    assert_eq!(ka, [0x05, 0x00, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00]);
    let exp = [0x0b, 0x00, 0x31, 0x18, 0x19, 0x2b, 0x97, 0x53, 0x24, 0xce, 0x3e];
    assert_eq!(frame::encode_encrypted(&KEY_OLD, &ka).unwrap(), exp);
}

/// 8. decrypt a real response frame -> 02 00 0e 00 00 00 00 28 — mvci_test.c:126
#[test]
fn decrypt_response_new_key() {
    let body = [0x81, 0x07, 0x0e, 0xb3, 0xd5, 0xf4, 0xee, 0xa6];
    let wire = frame::encode_plain(&body).unwrap();
    let inner_bytes = frame::decode_encrypted(&KEY_NEW, &wire).unwrap();
    assert_eq!(inner_bytes, [0x02, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x28]);
}

/// 9. DES round-trip identity — mvci_test.c:138
#[test]
fn des_round_trip() {
    let orig: Vec<u8> = (0..16u8).map(|i| i.wrapping_mul(7).wrapping_add(1)).collect();
    let mut buf = orig.clone();
    crypto::encrypt_in_place(&KEY_OLD, &mut buf);
    crypto::decrypt_in_place(&KEY_OLD, &mut buf);
    assert_eq!(buf, orig);
}

/// 10. reject corrupted checksum — mvci_test.c:147
#[test]
fn reject_bad_checksum() {
    let bad = [0x0b, 0x00, 0x31, 0x18, 0x19, 0x2b, 0x97, 0x53, 0x24, 0xce, 0x00];
    assert!(frame::decode_plain(&bad).is_err());
}

/// 11. supported ProtocolIDs: 1..6 and the vendor alias, nothing else —
/// mvci_test.c:156. 7..11 index past the adapter's six-entry handler table
/// and wedge it. In Rust the enum is closed, so this is a TryFrom test.
#[test]
fn proto_supported_1_to_6_only() {
    for p in 1u32..=6 {
        assert!(ProtocolId::try_from(p).is_ok(), "proto {p} must be accepted");
    }
    assert_eq!(ProtocolId::try_from(0x1000_0001).unwrap(), ProtocolId::Iso15765);
    for p in 7u32..=12 {
        assert!(ProtocolId::try_from(p).is_err(), "proto {p} must be refused");
    }
    assert!(ProtocolId::try_from(0).is_err());
}

/// 12. frames longer than 255 bytes: LEN is 16-bit little-endian — mvci_test.c:168
#[test]
fn frame_300_bytes_round_trips() {
    let payload: Vec<u8> = (0..300u32).map(|i| i as u8).collect();
    let wire = frame::encode_plain(&payload).unwrap();
    assert_eq!(wire.len(), 303);
    assert_eq!(wire[0], 0x2f);
    assert_eq!(wire[1], 0x01);
    assert_eq!(frame::decode_plain(&wire).unwrap(), &payload[..]);
}

/// 13. a full-size ISO15765 inner still encodes — mvci_test.c:182
#[test]
fn frame_4096_byte_inner() {
    let big = vec![0u8; 4096];
    let wire = frame::encode_encrypted(&KEY_OLD, &big).unwrap();
    assert_eq!(wire.len(), 4099);
    assert_eq!(wire[0], 0x03);
    assert_eq!(wire[1], 0x10);
}

/// 14. a read reply whose ILEN exceeds 255 — mvci_test.c:191
#[test]
fn read_reply_ilen_265() {
    let mut reply = vec![0u8; 11 + 256];
    reply[0] = 0x09;
    reply[1] = 0x01; // ILEN = 265 -> 256 msg bytes
    reply[2] = 0x09;
    for i in 0..256usize {
        reply[11 + i] = (i as u8) ^ 0x5a;
    }
    let parsed = inner::parse_read_reply(&reply).unwrap();
    assert_eq!(parsed.rx_status, RxStatus::empty());
    assert_eq!(parsed.msg.len(), 256);
    assert_eq!(parsed.msg[0], 0x5a);
    assert_eq!(parsed.msg[255], 255 ^ 0x5a);
}

/// 15. ISO15765 flow-control filter: three 4-byte identifiers, MSB-first,
/// ARGS = 12 + 3*4 = 24, ILEN = 25 — mvci_test.c:204. The adapter rejects any
/// other length with ERR_NULL_PARAMETER because it infers the field width.
#[test]
fn start_filter_iso15765_flow_control() {
    let exp = [
        0x19, 0x00, 0x0b, 0x06, 0, 0, 0, 0x8b, 0x7a, 0x0e, 0x00, 0x03, 0, 0, 0, 0xff, 0xff,
        0xff, 0xff, 0x00, 0x00, 0x07, 0xcc, 0x00, 0x00, 0x07, 0xc4,
    ];
    let got = inner::start_filter(
        ProtocolId::Iso15765,
        0x000e_7a8b,
        3,
        &[0xff, 0xff, 0xff, 0xff],
        &[0x00, 0x00, 0x07, 0xcc],
        Some(&[0x00, 0x00, 0x07, 0xc4]),
        4,
    )
    .unwrap();
    assert_eq!(got, exp);
}

/// 16. a CAN filter with a K-line-width identifier must be refused locally —
/// mvci_test.c:223
#[test]
fn reject_bad_filter_id_width() {
    let one = [0xc0u8];
    assert!(inner::start_filter(ProtocolId::Iso15765, 1, 1, &one, &one, None, 1).is_err());
    assert!(inner::start_filter(ProtocolId::Iso14230, 1, 1, &one, &one, None, 2).is_err());
}

/// 17. TxFlags reaches the wire — without it 29-bit CAN is unreachable —
/// mvci_test.c:233
#[test]
fn write_msg_29bit_txflags() {
    let msg = [0x00, 0x00, 0x07, 0xc4, 0x21, 0x43];
    let exp = [
        0x0f, 0, 0x0a, 0x06, 0, 0, 0, 0x00, 0x01, 0, 0, 0x00, 0x00, 0x07, 0xc4, 0x21, 0x43,
    ];
    let got = inner::write_msg(ProtocolId::Iso15765, 0x0100, &msg).unwrap();
    assert_eq!(got, exp);
}

/// 18. set_config carries the channel's protocol, not a baked-in ISO14230 —
/// mvci_test.c:244
#[test]
fn set_config_targets_can_channel() {
    let got = inner::set_config(ProtocolId::Iso15765, 1 /* DATA_RATE */, 500_000);
    assert_eq!(got.len(), 16);
    assert_eq!(got[4], 0x06);
    assert_eq!(got[5], 0);
    assert_eq!(got[8], 0x01);
    assert_eq!(got[12], 0x20);
    assert_eq!(got[13], 0xa1);
}

// ---- operational inner builders vs captured live plaintext ----

/// 19. connect(ISO14230, 4096, 10400) — mvci_test.c:255
#[test]
fn inner_connect_iso14230() {
    let exp = [
        0x0d, 0x00, 0x07, 0x04, 0, 0, 0, 0, 0x10, 0, 0, 0xa0, 0x28, 0, 0,
    ];
    assert_eq!(inner::connect(ProtocolId::Iso14230, 4096, 10400), exp);
}

/// 20. K-line filter c0/c0 — mvci_test.c:262
#[test]
fn inner_start_filter_kline() {
    let exp = [
        0x10, 0, 0x0b, 0x04, 0, 0, 0, 0x8b, 0x7a, 0x0e, 0, 0x01, 0, 0, 0, 0xc0, 0xc0, 0,
    ];
    let got =
        inner::start_filter(ProtocolId::Iso14230, 0x000e_7a8b, 1, &[0xc0], &[0xc0], None, 1)
            .unwrap();
    assert_eq!(got, exp);
}

/// 21. fast_init(81 19 f0 81) — mvci_test.c:271
#[test]
fn inner_fast_init() {
    let exp = [0x0a, 0, 0x0e, 0x05, 0x04, 0, 0, 0, 0x81, 0x19, 0xf0, 0x81];
    assert_eq!(inner::fast_init(ProtocolId::Iso14230, &[0x81, 0x19, 0xf0, 0x81]), exp);
}

/// 22. write_msg(82 19 f0 01 05) — mvci_test.c:279
#[test]
fn inner_write_msg_kline() {
    let exp = [
        0x0e, 0, 0x0a, 0x04, 0, 0, 0, 0, 0, 0, 0, 0x82, 0x19, 0xf0, 0x01, 0x05,
    ];
    let got = inner::write_msg(ProtocolId::Iso14230, 0, &[0x82, 0x19, 0xf0, 0x01, 0x05]).unwrap();
    assert_eq!(got, exp);
}

/// 23. parse read reply -> msg bytes — mvci_test.c:287
#[test]
fn parse_read_reply_kline() {
    let reply = [
        0x0f, 0, 0x09, 0, 0, 0, 0, 0, 0, 0, 0, 0x83, 0xf0, 0x19, 0x41, 0x0d, 0, 0, 0, 0, 0, 0,
        0, 0,
    ];
    let parsed = inner::parse_read_reply(&reply).unwrap();
    assert_eq!(parsed.msg, [0x83, 0xf0, 0x19, 0x41, 0x0d, 0x00]);
}

/// 24. full connect frame, KEY_NEW -> captured ciphertext + checksum —
/// mvci_test.c:296
#[test]
fn connect_full_frame_new_key() {
    let cipher = [
        0x5b, 0x81, 0x15, 0x2c, 0xcb, 0xe7, 0xd9, 0xc2, 0x47, 0x5c, 0xeb, 0x5c, 0x21, 0x70,
        0xf0, 0xdd,
    ];
    let exp = frame::encode_plain(&cipher).unwrap();
    let i = inner::connect(ProtocolId::Iso14230, 4096, 10400);
    assert_eq!(frame::encode_encrypted(&KEY_NEW, &i).unwrap(), exp);
}

/// Bonus (not in the C suite): the deframer cuts the captured frames out of a
/// single contiguous stream, including a >255-byte one.
#[test]
fn deframer_on_captured_stream() {
    let f1 = frame::encode_plain(&inner::identify()).unwrap();
    let f2 = frame::encode_plain(&(0..300u32).map(|i| i as u8).collect::<Vec<_>>()).unwrap();
    let f3 = frame::encode_encrypted(&KEY_OLD, &inner::read_poll(ProtocolId::Iso15765)).unwrap();

    let mut d = Deframer::new();
    d.push(&f1);
    d.push(&f2);
    d.push(&f3);
    assert_eq!(d.next_frame().unwrap().unwrap(), f1);
    assert_eq!(d.next_frame().unwrap().unwrap(), f2);
    assert_eq!(d.next_frame().unwrap().unwrap(), f3);
    assert_eq!(d.next_frame().unwrap(), None);
    assert_eq!(d.pending(), 0);
}
