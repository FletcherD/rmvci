//! rMVCI — Rust driver for the Toyota Mini-VCI J2534 cable
//! (FTDI FT232R + LPC2119 running the "J2534 MINI V1.03" firmware).
//!
//! Layering, bottom up:
//! - [`codec`] — sans-IO wire protocol: framing, DES-ECB, inner commands.
//! - [`transport`] — byte transports (serial port, direct USB, JNI; mock for tests).
//! - [`session`] — port-owning actor, handshake, typed channels.
//! - [`isotp`] — ISO 15765-2 over raw CAN (host-side) and via the firmware.
//! - [`kline`] — cooked ISO 14230 (K-line) ECU bring-up and request/response.
//!
//! Protocol behaviour is encoded from a reverse-engineering of the cable's
//! firmware, and all of it is bench-verified against the real cable; the
//! captured vendor-DLL wire vectors in `tests/captured_vectors.rs` pin it.

pub mod codec;
pub mod error;
pub mod isotp;
pub mod kline;
pub mod session;
pub mod transport;
pub mod types;

pub use error::{CodecError, Error, InvalidProtocolId, IsoTpError, TransportError};
pub use isotp::{FirmwareIsoTp, IsoTp, IsoTpConfig, UdsTransport};
pub use kline::{KLINE_TIMING, KLineEcu};
pub use session::protocol::{
    Can, CanConfig, CanFilter, CanId, FlowControlFilter, Iso9141, Iso14230, Iso15765, KLine,
    KLineConfig, KLineFilter, Protocol,
};
pub use session::{Channel, Device, DeviceConfig, RawChannel, resolve_port};
pub use types::{Cmd, DeviceStatus, FilterType, ProtocolId, RxMsg, RxStatus, TxFlags};
