//! Typed errors. The C driver returned 0/-1 and (originally) never read the
//! device status byte at all, so rejections looked like success; every
//! failure mode here is distinct and matchable.

use std::time::Duration;

use crate::types::{Cmd, DeviceStatus, ProtocolId};

/// A ProtocolID outside the set the adapter implements.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
#[error(
    "unsupported ProtocolID {0:#x}: the adapter implements only 1-6 (and the \
     0x10000001 alias); any other value indexes past its handler table and \
     wedges the MCU until the port is reopened"
)]
pub struct InvalidProtocolId(pub u32);

/// Byte-level transport failure (serial port / USB).
#[derive(Debug, thiserror::Error)]
pub enum TransportError {
    #[error(transparent)]
    Io(#[from] std::io::Error),
    #[error("failed to open port {port}: {reason}")]
    Open { port: String, reason: String },
}

/// Frame/inner codec failure — the bytes are structurally wrong.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum CodecError {
    #[error("frame too short ({0} bytes; minimum is 3)")]
    TooShort(usize),
    #[error("frame too long ({0} bytes)")]
    TooLong(usize),
    #[error("length field says {field} but frame is {actual} bytes")]
    LengthMismatch { field: usize, actual: usize },
    #[error("bad checksum: wire {wire:#04x}, computed {computed:#04x}")]
    BadChecksum { wire: u8, computed: u8 },
    #[error(
        "bad checksum on encrypted frame: wire {wire:#04x} matches neither the \
         ciphertext sum {cipher:#04x} nor the plaintext sum {plain:#04x}"
    )]
    BadEncryptedChecksum { wire: u8, cipher: u8, plain: u8 },
    #[error("encrypted payload length {0} is not a positive multiple of 8")]
    BadCipherLength(usize),
    #[error(
        "invalid filter identifier width {idlen} for {proto:?} (1 for K-line, \
         4 for CAN/ISO15765, 5 for extended addressing)"
    )]
    BadIdLen { proto: ProtocolId, idlen: usize },
    #[error("filter field is {have} bytes but the identifier width is {need}")]
    FilterFieldTooShort { have: usize, need: usize },
    #[error("message of {0} bytes exceeds the adapter limit")]
    MsgTooLong(usize),
    #[error("malformed reply: {0}")]
    MalformedReply(&'static str),
}

/// ISO-TP (ISO 15765-2) failure, host-side path.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum IsoTpError {
    #[error("payload of {0} bytes exceeds the 4095-byte ISO-TP limit")]
    PayloadTooLong(usize),
    #[error(
        "payload of {0} bytes exceeds 255: the cable firmware truncates FF_DL \
         to 8 bits and would emit a malformed First Frame — use the host-side \
         ISO-TP path for transfers this large"
    )]
    FirmwareFfDlLimit(usize),
    #[error("no flow control from the ECU within the N_Bs timeout")]
    FcTimeout,
    #[error("consecutive frame gap exceeded the N_Cr timeout")]
    CfTimeout,
    #[error("ECU sent FS=WAIT more than {0} times")]
    WaitLimit(u8),
    #[error("ECU reported flow-control overflow (FS=OVFLW)")]
    Overflow,
    #[error("consecutive frame sequence error: expected SN {expected}, got {got}")]
    SequenceError { expected: u8, got: u8 },
    #[error("malformed ISO-TP frame: {0}")]
    Malformed(&'static str),
}

/// Top-level driver error.
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Transport(#[from] TransportError),
    #[error(transparent)]
    Codec(#[from] CodecError),
    #[error(transparent)]
    Protocol(#[from] InvalidProtocolId),
    #[error(transparent)]
    IsoTp(#[from] IsoTpError),
    #[error("no reply from the adapter within {0:?} (wedged? reopening the port pulses DTR/RTS and resets the MCU)")]
    Timeout(Duration),
    #[error("adapter rejected {cmd:?}: {status:?}")]
    Rejected { cmd: Cmd, status: DeviceStatus },
    #[error(
        "read timed out and no PASS/FLOW_CONTROL filter is installed on \
         {proto:?}: this adapter's acceptance filter starts enabled and \
         empty, so a filterless channel receives nothing at all — install a \
         filter first"
    )]
    NoFilterInstalled { proto: ProtocolId },
    #[error("no message available")]
    BufferEmpty,
    #[error("adapter unresponsive after {0} consecutive keepalive failures; reopen the device")]
    Wedged(u8),
    #[error("device handle closed")]
    Closed,
}
