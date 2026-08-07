//! Property tests for the sans-IO codec. These target invariants the captured
//! vectors can't cover: arbitrary lengths (crossing the 255-byte boundary the
//! original C spec got wrong), arbitrary keys, the dual checksum convention,
//! and the deframer under arbitrary chunking.

use proptest::prelude::*;
use rmvci_core::codec::{Deframer, frame, inner};
use rmvci_core::types::{Cmd, DeviceStatus, ProtocolId};

proptest! {
    /// decode_plain(encode_plain(p)) == p for any payload up to 4096 bytes.
    #[test]
    fn plain_round_trip(payload in proptest::collection::vec(any::<u8>(), 0..4096)) {
        let wire = frame::encode_plain(&payload).unwrap();
        prop_assert_eq!(frame::decode_plain(&wire).unwrap(), &payload[..]);
    }

    /// Encrypted round trip: the decoded inner is the original plus zero
    /// padding to the DES block size.
    #[test]
    fn encrypted_round_trip(
        key in any::<[u8; 8]>(),
        inner_bytes in proptest::collection::vec(any::<u8>(), 1..4096),
    ) {
        let wire = frame::encode_encrypted(&key, &inner_bytes).unwrap();
        let back = frame::decode_encrypted(&key, &wire).unwrap();
        let padded = inner_bytes.len().div_ceil(8) * 8;
        prop_assert_eq!(back.len(), padded);
        prop_assert_eq!(&back[..inner_bytes.len()], &inner_bytes[..]);
        prop_assert!(back[inner_bytes.len()..].iter().all(|&b| b == 0));
    }

    /// The device checksums status replies over the ciphertext but message
    /// replies over the plaintext; the decoder must accept both conventions
    /// and reject a checksum matching neither.
    #[test]
    fn dual_checksum_conventions(
        key in any::<[u8; 8]>(),
        inner_bytes in proptest::collection::vec(any::<u8>(), 1..256),
    ) {
        let mut wire = frame::encode_encrypted(&key, &inner_bytes).unwrap();

        // As encoded: checksum over the ciphertext (status-reply convention).
        prop_assert!(frame::decode_encrypted(&key, &wire).is_ok());

        // Re-checksum over the plaintext (message-reply convention).
        let mut padded = inner_bytes.clone();
        padded.resize(inner_bytes.len().div_ceil(8) * 8, 0);
        let plain_sum = wire[0] ^ wire[1] ^ frame::xorsum(&padded);
        let cipher_sum = frame::xorsum(&wire[..wire.len() - 1]);
        let last = wire.len() - 1;
        wire[last] = plain_sum;
        prop_assert!(frame::decode_encrypted(&key, &wire).is_ok());

        // A third value must be rejected.
        let bogus = (0..=255u8).find(|&c| c != plain_sum && c != cipher_sum).unwrap();
        wire[last] = bogus;
        prop_assert!(frame::decode_encrypted(&key, &wire).is_err());
    }

    /// The deframer yields the identical frame sequence no matter how the
    /// byte stream is chunked (1-byte drips included).
    #[test]
    fn deframer_survives_chunking(
        payloads in proptest::collection::vec(
            proptest::collection::vec(any::<u8>(), 0..600), 1..5),
        chunk in 1usize..17,
    ) {
        let frames: Vec<Vec<u8>> =
            payloads.iter().map(|p| frame::encode_plain(p).unwrap()).collect();
        let stream: Vec<u8> = frames.concat();

        let mut d = Deframer::new();
        let mut got = Vec::new();
        for piece in stream.chunks(chunk) {
            d.push(piece);
            while let Some(f) = d.next_frame().unwrap() {
                got.push(f);
            }
        }
        prop_assert_eq!(got, frames);
        prop_assert_eq!(d.pending(), 0);
    }

    /// Read-reply build/parse round trip, including >255-byte ILENs.
    #[test]
    fn read_reply_round_trip(
        rx in any::<u32>(),
        msg in proptest::collection::vec(any::<u8>(), 1..1024),
    ) {
        let mut reply = Vec::with_capacity(11 + msg.len());
        reply.extend_from_slice(&((9 + msg.len()) as u16).to_le_bytes());
        reply.push(0x09);
        reply.extend_from_slice(&rx.to_le_bytes());
        reply.extend_from_slice(&[0; 4]);
        reply.extend_from_slice(&msg);

        let parsed = inner::parse_read_reply(&reply).unwrap();
        prop_assert_eq!(parsed.rx_status.bits(), rx);
        prop_assert_eq!(parsed.msg, &msg[..]);
    }

    /// status_of accepts exactly the [02 00 CMD status] shape and maps every
    /// status byte.
    #[test]
    fn status_reply_round_trip(status in any::<u8>(), garbage in any::<[u8; 4]>()) {
        // Trailing garbage models the adapter's uninitialised-stack bytes.
        let mut reply = vec![0x02, 0x00, Cmd::Connect as u8, status];
        reply.extend_from_slice(&garbage);
        let got = inner::status_of(&reply, Cmd::Connect).unwrap();
        prop_assert_eq!(got, DeviceStatus::from(status));
        // Echo of a different command is malformed.
        prop_assert!(inner::status_of(&reply, Cmd::WriteMsg).is_err());
    }

    /// Inner write_msg build/parse consistency for arbitrary payload sizes.
    #[test]
    fn write_msg_layout(
        txflags in any::<u32>(),
        msg in proptest::collection::vec(any::<u8>(), 1..4100),
    ) {
        let built = inner::write_msg(ProtocolId::Can, txflags, &msg).unwrap();
        prop_assert_eq!(built.len(), 11 + msg.len());
        prop_assert_eq!(u16::from_le_bytes([built[0], built[1]]) as usize, 9 + msg.len());
        prop_assert_eq!(built[2], 0x0a);
        prop_assert_eq!(u32::from_le_bytes(built[7..11].try_into().unwrap()), txflags);
        prop_assert_eq!(&built[11..], &msg[..]);
    }
}
