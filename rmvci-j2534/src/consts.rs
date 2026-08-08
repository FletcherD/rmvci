//! SAE J2534-1 constants (subset used by this DLL), matching
//! libMVCI's `include/mvci/j2534.h`.

use core::ffi::c_long;

pub const PASSTHRU_MSG_DATA_SIZE: usize = 4128;

// Protocol IDs
pub const J1850VPW: u32 = 0x01;
pub const J1850PWM: u32 = 0x02;
pub const ISO9141: u32 = 0x03;
pub const ISO14230: u32 = 0x04;
pub const CAN: u32 = 0x05;
pub const ISO15765: u32 = 0x06;

// Filter types
pub const PASS_FILTER: u32 = 0x01;
pub const BLOCK_FILTER: u32 = 0x02;
pub const FLOW_CONTROL_FILTER: u32 = 0x03;

// Ioctl IDs
pub const GET_CONFIG: u32 = 0x01;
pub const SET_CONFIG: u32 = 0x02;
pub const READ_VBATT: u32 = 0x03;
pub const FIVE_BAUD_INIT: u32 = 0x04;
pub const FAST_INIT: u32 = 0x05;
pub const CLEAR_TX_BUFFER: u32 = 0x07;
pub const CLEAR_RX_BUFFER: u32 = 0x08;
pub const CLEAR_PERIODIC_MSGS: u32 = 0x09;
pub const CLEAR_MSG_FILTERS: u32 = 0x0a;

// TxFlags / RxStatus bits referenced by the shim
pub const ISO15765_FRAME_PAD: u32 = 0x0040;
pub const ISO15765_ADDR_TYPE: u32 = 0x0080;
pub const CAN_29BIT_ID: u32 = 0x0100;

/// Vendor ConnectFlag (rMVCI-specific, top bit — outside the SAE-defined
/// range): open an ISO15765 channel that runs ISO-TP **host-side** over raw CAN
/// (protocol 5) instead of the cable's firmware ISO-TP. Slower per frame, but
/// segments payloads beyond 255 bytes correctly and honors the ECU's flow
/// control (BS/STmin) — which the firmware path cannot. Stripped before the
/// flags reach the wire.
pub const RMVCI_HOST_ISOTP: u32 = 0x8000_0000;

// Error codes
pub const STATUS_NOERROR: c_long = 0x00;
pub const ERR_NOT_SUPPORTED: c_long = 0x01;
pub const ERR_INVALID_CHANNEL_ID: c_long = 0x02;
pub const ERR_INVALID_PROTOCOL_ID: c_long = 0x03;
pub const ERR_NULL_PARAMETER: c_long = 0x04;
pub const ERR_FAILED: c_long = 0x07;
pub const ERR_DEVICE_NOT_CONNECTED: c_long = 0x08;
pub const ERR_TIMEOUT: c_long = 0x09;
pub const ERR_INVALID_MSG: c_long = 0x0a;
pub const ERR_INVALID_MSG_ID: c_long = 0x0d;
pub const ERR_BUFFER_EMPTY: c_long = 0x10;
pub const ERR_BUFFER_FULL: c_long = 0x11;
pub const ERR_CHANNEL_IN_USE: c_long = 0x14;
pub const ERR_INVALID_FILTER_ID: c_long = 0x16;
pub const ERR_INVALID_DEVICE_ID: c_long = 0x1a;
