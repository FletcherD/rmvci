//! rMVCI — Rust driver for the Toyota Mini-VCI J2534 cable
//! (FTDI FT232R + LPC2119 running the "J2534 MINI V1.03" firmware).
//!
//! Layering, bottom up:
//! - wire codec — sans-IO framing, DES-ECB, inner commands (internal).
//! - [`transport`] — byte transports (serial port, direct USB, JNI; mock for tests).
//! - [`Device`] — port-owning actor and handshake; [`Channel`] — typed bus channels.
//! - [`IsoTp`] — ISO 15765-2 over raw CAN (host-side) and via the firmware.
//! - [`KLineEcu`] — cooked ISO 14230 (K-line) ECU bring-up and request/response.
//!
//! Protocol behaviour is encoded from a reverse-engineering of the cable's
//! firmware, and all of it is bench-verified against the real cable; the
//! captured vendor-DLL wire vectors in `tests/captured_vectors.rs` pin it.

/// Not part of the stable API: the wire codec is exposed for the captured-vector
/// tests, fuzzing and wire-level tools, and may change in any release.
#[doc(hidden)]
pub mod codec;
mod error;
pub mod isotp;
mod kline;
mod session;
pub mod transport;
pub mod types;

pub use error::{CodecError, Error, InvalidProtocolId, IsoTpError, TransportError};
pub use isotp::{IsoTp, IsoTpConfig, IsoTpPath, UdsTransport};
pub use kline::{KLINE_TIMING, KLineEcu};
pub use session::protocol::{
    Can, CanConfig, CanFilter, CanFramed, CanId, ChannelConfig, EncodedFilter, FilterSpec,
    FlowControlFilter, Iso9141, Iso14230, Iso15765, KLine, KLineConfig, KLineFilter, Protocol,
};
pub use session::{Channel, Device, DeviceConfig, RawChannel, resolve_port};
pub use types::{Cmd, DeviceStatus, FilterType, ProtocolId, RxMsg, RxStatus, TxFlags};
