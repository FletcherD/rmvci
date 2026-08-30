//! # rmvci-diag
//!
//! Transport-agnostic **KWP2000 (ISO 14230-3)** and **UDS (ISO 14229-1)**
//! diagnostic clients with the Toyota extensions recovered from Techstream,
//! built on the [`rmvci-core`] [`UdsTransport`] seam. The same clients drive:
//!
//! - **K-line / ISO 14230** — via [`rmvci_core::KLineEcu`] (the Gen-2 Toyota
//!   body/HVAC bus, e.g. the NHW20 Prius A/C amp at address `0x98`), and
//! - **ISO-TP over CAN** — via [`rmvci_core::IsoTp`] on either
//!   [`rmvci_core::IsoTpPath`] (powertrain/HV, and newer CAN A/C amps).
//!
//! Pick the client by ECU generation:
//!
//! - [`Kwp2000`] — the KWP2000 service set (`21`/`30`/`3B`/`18`/`14`/`12` …),
//!   used by the older Toyota P3/P4 ECUs (K-line and early CAN).
//! - [`Uds`] — the UDS service set (`22`/`2E`/`2F`/`31`/`19` …), used by the
//!   newer P5 (UDS) ECUs.
//!
//! Standard service ids and negative response codes come from the
//! permissively-licensed [`automotive_diag`] crate; this crate adds request
//! building, response validation, and the Toyota services (`0xA5` customize
//! write, the `0x30` active test, `0x12` freeze-frame read).
//!
//! ```no_run
//! use rmvci_core::{Device, KLineEcu};
//! use rmvci_diag::Kwp2000;
//!
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! let dev = Device::open("/dev/ttyUSB0")?;
//! let mut ecu = KLineEcu::connect(&dev, 0x98)?;
//! ecu.fast_init()?;
//! let mut kwp = Kwp2000::new(ecu);
//!
//! let ambient = kwp.read_data_by_local_id(0x02)?; // 21 02
//! let servo   = kwp.read_data_by_local_id(0x0d)?; // 21 0D air-mix damper (D)
//! println!("ambient {ambient:02x?}, servo {servo:02x?}");
//! # Ok(()) }
//! ```
//!
//! [`rmvci-core`]: rmvci_core
//! [`UdsTransport`]: rmvci_core::UdsTransport

mod common;
mod error;
mod kwp2000;
mod toyota;
mod toyota_can;
mod uds;

pub use error::{Error, Result};
pub use kwp2000::Kwp2000;
pub use toyota::SID_TOYOTA_WRITE;
pub use toyota_can::{EnumeratedPid, IsoTpChannel, ToyotaCanLive};
pub use uds::Uds;

/// Re-exported standard protocol definitions from [`automotive_diag`].
pub use automotive_diag::kwp2000::{KwpCommand, KwpError, KwpErrorByte};
pub use automotive_diag::uds::{UdsCommand, UdsError, UdsErrorByte};
