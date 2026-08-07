//! Wire-level constants and value types.
//!
//! Everything here mirrors behaviour verified on the cable's firmware — see
//! `re/FINDINGS.md` in the parent project for the evidence.

use crate::error::InvalidProtocolId;

/// Serial link speed of the FT232R side of the cable.
pub const BAUD: u32 = 115_200;

/// Largest message payload the adapter accepts in a WriteMsgs
/// (4-byte CAN id + 4096 data bytes for ISO15765).
pub const MAX_MSG: usize = 4100;
/// Largest inner command: ILEN(2) + CMD(1) + RxStatus(4) + reserved(4) + MSG,
/// rounded to the DES block size.
pub const MAX_INNER: usize = 4112;
/// Largest wire frame: LEN(2) + payload + XORSUM(1).
pub const MAX_FRAME: usize = 3 + MAX_INNER;

/// Vendor alias for ISO15765 accepted by `PassThruConnect`; the firmware
/// dispatches it to the same handler as protocol 6. Inbound spelling only —
/// the wire always carries 6.
pub const ISO15765_ALIAS: u32 = 0x1000_0001;

/// ProtocolIDs the adapter implements. `ChannelID == ProtocolID` on this
/// device — there is no separate channel handle.
///
/// This enum is deliberately closed: the firmware indexes a six-entry handler
/// table with `ProtocolID - 1` and no bounds check, so IDs 7–11 send it
/// through a wild pointer and it stops answering until the port is reopened
/// (the DTR/RTS pulse on open resets the MCU). No raw `u32` reaches the wire.
#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum ProtocolId {
    J1850Vpw = 1,
    J1850Pwm = 2,
    Iso9141 = 3,
    Iso14230 = 4,
    Can = 5,
    Iso15765 = 6,
}

impl TryFrom<u32> for ProtocolId {
    type Error = InvalidProtocolId;

    fn try_from(v: u32) -> Result<Self, InvalidProtocolId> {
        Ok(match v {
            1 => Self::J1850Vpw,
            2 => Self::J1850Pwm,
            3 => Self::Iso9141,
            4 => Self::Iso14230,
            5 => Self::Can,
            6 | ISO15765_ALIAS => Self::Iso15765,
            other => return Err(InvalidProtocolId(other)),
        })
    }
}

impl ProtocolId {
    pub fn wire(self) -> u32 {
        self as u32
    }

    pub fn is_can(self) -> bool {
        matches!(self, Self::Can | Self::Iso15765)
    }
}

/// Inner command opcodes (firmware jump table at 0xb64c).
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Cmd {
    /// Replies 8 DES key bytes in the clear, then TOGGLES encryption —
    /// sending it twice silently turns the cipher back off.
    Open = 0x01,
    /// Tears down the encrypted session and every channel with it.
    CloseSession = 0x02,
    ReadVersion = 0x03,
    Connect = 0x07,
    /// Per-channel disconnect. Distinct from CloseSession.
    Disconnect = 0x08,
    ReadMsg = 0x09,
    WriteMsg = 0x0a,
    StartMsgFilter = 0x0b,
    StopMsgFilter = 0x0c,
    SetProgrammingVoltage = 0x0d,
    Ioctl = 0x0e,
    StartPeriodicMsg = 0x0f,
    StopPeriodicMsg = 0x10,
}

/// Status byte of a device reply (inner offset +3). The values are standard
/// J2534 error codes; everything after this byte in a status reply is the
/// adapter's uninitialised stack and must never be parsed.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DeviceStatus {
    NoError,
    NotSupported,
    InvalidChannelId,
    NullParameter,
    Failed,
    DeviceNotConnected,
    Timeout,
    InvalidMsg,
    InvalidMsgId,
    BufferEmpty,
    BufferFull,
    ChannelInUse,
    InvalidFilterId,
    InvalidDeviceId,
    Unknown(u8),
}

impl From<u8> for DeviceStatus {
    fn from(v: u8) -> Self {
        match v {
            0x00 => Self::NoError,
            0x01 => Self::NotSupported,
            0x02 => Self::InvalidChannelId,
            0x04 => Self::NullParameter,
            0x07 => Self::Failed,
            0x08 => Self::DeviceNotConnected,
            0x09 => Self::Timeout,
            0x0a => Self::InvalidMsg,
            0x0d => Self::InvalidMsgId,
            0x10 => Self::BufferEmpty,
            0x11 => Self::BufferFull,
            0x14 => Self::ChannelInUse,
            0x16 => Self::InvalidFilterId,
            0x1a => Self::InvalidDeviceId,
            other => Self::Unknown(other),
        }
    }
}

impl DeviceStatus {
    pub fn is_ok(self) -> bool {
        self == Self::NoError
    }

    /// The raw wire byte — also a valid J2534 error code, which is what the
    /// PassThru shim reports it as.
    pub fn code(self) -> u8 {
        match self {
            Self::NoError => 0x00,
            Self::NotSupported => 0x01,
            Self::InvalidChannelId => 0x02,
            Self::NullParameter => 0x04,
            Self::Failed => 0x07,
            Self::DeviceNotConnected => 0x08,
            Self::Timeout => 0x09,
            Self::InvalidMsg => 0x0a,
            Self::InvalidMsgId => 0x0d,
            Self::BufferEmpty => 0x10,
            Self::BufferFull => 0x11,
            Self::ChannelInUse => 0x14,
            Self::InvalidFilterId => 0x16,
            Self::InvalidDeviceId => 0x1a,
            Self::Unknown(v) => v,
        }
    }
}

bitflags::bitflags! {
    /// RxStatus word of a message reply (32 bits — reading only the low byte
    /// would drop `CAN_29BIT` and silently discard every 29-bit message).
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub struct RxStatus: u32 {
        /// Overloaded bit: marks both the adapter's transmit echo and the
        /// ISO15765 start-of-message indication. Neither is payload.
        const NOT_DATA = 0x0002;
        /// ISO15765 extended addressing.
        const ADDR_TYPE = 0x0080;
        /// 29-bit CAN identifier.
        const CAN_29BIT = 0x0100;
        const _ = !0;
    }
}

bitflags::bitflags! {
    /// TxFlags for WriteMsgs. The firmware maps these onto internal config
    /// IDs per message; `CAN_ID_BOTH` (0x800) is not handled by the firmware
    /// at all and is deliberately absent.
    #[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
    pub struct TxFlags: u32 {
        const ISO15765_FRAME_PAD = 0x0040;
        const ISO15765_ADDR_TYPE = 0x0080;
        const CAN_29BIT_ID = 0x0100;
    }
}

/// Filter types for StartMsgFilter. `Block` is accepted by the firmware but
/// is a complete no-op — it programs nothing.
#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum FilterType {
    Pass = 1,
    Block = 2,
    FlowControl = 3,
}

/// One received message, as the adapter reported it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RxMsg {
    pub rx_status: RxStatus,
    /// For CAN/ISO15765: 4-byte big-endian CAN identifier, then the payload.
    pub data: Vec<u8>,
}

impl RxMsg {
    /// True for the adapter's transmit echo and the ISO15765 start-of-message
    /// indication — frames that carry no payload data.
    pub fn is_indication(&self) -> bool {
        self.rx_status.contains(RxStatus::NOT_DATA)
    }
}
