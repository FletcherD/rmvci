//! Byte transports. The trait carries only the primitives; the DTR/RTS reset
//! dance and all protocol timing live in the session layer, so a mock can
//! assert the exact sequence.

use std::time::Duration;

use crate::error::TransportError;

#[cfg(feature = "jni-transport")]
pub mod jni;
pub mod mock;
#[cfg(feature = "serial")]
pub mod serial;
#[cfg(feature = "usb")]
pub mod usb;

#[cfg(feature = "jni-transport")]
pub use jni::JniTransport;
pub use mock::MockTransport;
#[cfg(feature = "serial")]
pub use serial::SerialTransport;
#[cfg(feature = "usb")]
pub use usb::UsbTransport;

/// Outcome of the best-effort FTDI latency-timer fix. The FT232R holds
/// replies under 64 bytes for the full latency interval, and the kernel
/// default is 16 ms — on a strict request/response protocol that is a 16 ms
/// tax on every exchange.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LatencyResult {
    /// Timer written successfully.
    Set { millis: u8 },
    /// Timer was already at or below the target.
    AlreadyLow { millis: u8 },
    /// Attempt failed (usually EACCES on the sysfs node; needs a udev rule).
    Failed { reason: String },
    /// This transport has no latency-timer concept.
    Unavailable,
}

pub trait Transport: Send + 'static {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError>;

    /// Read up to `buf.len()` bytes, blocking up to `timeout` for the first
    /// byte. `Ok(0)` means the timeout expired with nothing available — the
    /// caller treats that as the full wait having elapsed.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError>;

    fn purge_rx(&mut self) -> Result<(), TransportError>;

    /// Set both modem lines at once (used only by the open reset dance).
    fn set_modem(&mut self, dtr: bool, rts: bool) -> Result<(), TransportError>;

    fn optimize_latency(&mut self) -> LatencyResult {
        LatencyResult::Unavailable
    }
}

/// Sleep source, injectable so mock tests don't spend real time in the
/// handshake's 15 ms / 110 ms / 1000 ms delays.
pub trait Clock: Send + Sync {
    fn sleep(&self, d: Duration);
}

pub struct RealClock;

impl Clock for RealClock {
    fn sleep(&self, d: Duration) {
        std::thread::sleep(d);
    }
}
