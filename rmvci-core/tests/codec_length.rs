//! Regression: the inner-command length field is a genuine 16-bit little-endian
//! value with no 8-bit cliff.
//!
//! libMVCI validated frame/inner lengths as a byte plus a reserved zero, so it
//! rejected anything ≥ 256 bytes — which silently broke every multi-frame
//! ISO-TP transfer. rMVCI uses full `u16` lengths (`inner::ilen`, `rd_u16`).
//! These vectors lock that in: a >255-byte WriteMsgs encodes and round-trips,
//! and a >255-byte ReadMsgs reply parses without truncation.

use rmvci_core::codec::{frame, inner};
use rmvci_core::types::{ProtocolId, RxStatus};

const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];
const TX_ID: [u8; 4] = [0x00, 0x00, 0x07, 0xc4];
const RX_ID: [u8; 4] = [0x00, 0x00, 0x07, 0xcc];

/// A 260-byte ISO15765 WriteMsgs (4-byte id + 256-byte payload) encodes with a
/// correct 16-bit ILEN and survives an encrypt/decrypt round trip.
#[test]
fn large_write_has_no_length_cliff() {
    let mut msg = TX_ID.to_vec();
    msg.extend(std::iter::repeat_n(0xab_u8, 256)); // payload crosses the 255 boundary
    let inner = inner::write_msg(ProtocolId::Iso15765, 0, &msg).unwrap();

    // ILEN = 1 (CMD) + 8 (proto + txflags) + 260 (msg) = 269 = 0x010d, LE.
    assert_eq!(&inner[0..2], &[0x0d, 0x01], "ILEN must be a real u16, not truncated");
    assert_eq!(inner.len(), 3 + 8 + 260);

    // Full wire round trip — proves the framer/deframer carry the length too.
    let wire = frame::encode_encrypted(&KEY_OLD, &inner).unwrap();
    let back = frame::decode_encrypted(&KEY_OLD, &wire).unwrap();
    assert_eq!(&back[..inner.len()], &inner[..]);
}

/// A 300-byte ReadMsgs reply parses back to exactly 300 message bytes — the
/// case libMVCI's byte-length check dropped.
#[test]
fn large_read_reply_is_not_truncated() {
    let payload: Vec<u8> = (0..300).map(|i| i as u8).collect();
    let mut data = RX_ID.to_vec();
    data.extend_from_slice(&payload); // 4-byte id + 300-byte payload = 304 bytes

    let mut inner = Vec::new();
    inner.extend_from_slice(&((9 + data.len()) as u16).to_le_bytes()); // ILEN = 313 = 0x0139
    inner.push(0x09); // READ_MSG echo
    inner.extend_from_slice(&0u32.to_le_bytes()); // RxStatus
    inner.extend_from_slice(&0u32.to_le_bytes()); // reserved
    inner.extend_from_slice(&data);
    assert_eq!(&inner[0..2], &[0x39, 0x01]);

    let reply = inner::parse_read_reply(&inner).unwrap();
    assert_eq!(reply.rx_status, RxStatus::empty());
    assert_eq!(reply.msg.len(), RX_ID.len() + 300);
    assert_eq!(&reply.msg[4..], &payload[..]);
}
