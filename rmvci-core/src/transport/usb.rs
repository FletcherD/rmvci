//! Direct USB backend for the cable's FT232R, via `nusb` (pure Rust, no
//! libusb). Bypasses the kernel's `ftdi_sio` driver entirely.
//!
//! Two reasons to prefer it over [`super::serial`]:
//!
//! - **It can set the FTDI latency timer without root.** The timer is a
//!   vendor control transfer, which this process issues itself; the serial
//!   backend can only ask sysfs, which needs root or a udev rule. At the
//!   16 ms default every request/response pays up to 16 ms of pure latency.
//! - It works where `ftdi_sio` does not exist.
//!
//! The FTDI vendor request numbers and wValue encodings below follow
//! `drivers/usb/serial/ftdi_sio.h` in the Linux kernel.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, ControlOut, ControlType, In, Out, Recipient};

use super::{LatencyResult, Transport};
use crate::error::TransportError;

/// FTDI's default VID/PID. The Mini-VCI is a stock FT232R and enumerates
/// with these; only the serial string distinguishes it from any other FTDI
/// cable on the bus.
pub const FTDI_VID: u16 = 0x0403;
pub const FT232R_PID: u16 = 0x6001;

// Vendor requests (bmRequestType 0x40 = host-to-device, vendor, device).
const SIO_RESET: u8 = 0;
const SIO_MODEM_CTRL: u8 = 1;
const SIO_SET_BAUDRATE: u8 = 3;
const SIO_SET_DATA: u8 = 4;
const SIO_SET_LATENCY_TIMER: u8 = 9;

const SIO_RESET_SIO: u16 = 0;
const SIO_RESET_PURGE_RX: u16 = 1;

/// 8 data bits (B0..7), no parity (B8..10), 1 stop bit (B11..13).
const DATA_8N1: u16 = 8;

/// 3 MHz reference / 115200 = 26.04; the FT232R takes a 14-bit integer
/// divisor with a 3-bit fractional index above it. Integer 26, fraction 0
/// gives 115384 baud — 0.16 % fast, comfortably inside tolerance, and it is
/// the divisor every FTDI driver uses for this rate.
const BAUD_115200_DIVISOR: u16 = 0x001a;

/// FT232R has exactly one interface with one bulk pair.
const EP_IN: u8 = 0x81;
const EP_OUT: u8 = 0x02;

const TARGET_LATENCY_MS: u8 = 2;

fn io_err(e: impl std::fmt::Display, what: &str) -> TransportError {
    TransportError::Open { port: what.to_string(), reason: e.to_string() }
}

pub struct UsbTransport {
    /// Kept so `Drop` can hand the interface back to `ftdi_sio`.
    device: nusb::Device,
    interface: nusb::Interface,
    ep_in: nusb::Endpoint<Bulk, In>,
    ep_out: nusb::Endpoint<Bulk, Out>,
    /// Payload bytes recovered from IN packets, minus the status headers.
    rx: VecDeque<u8>,
    max_packet: usize,
    label: String,
}

impl Drop for UsbTransport {
    /// Give the cable back to `ftdi_sio`, which restores its `/dev/ttyUSBn`
    /// node. Without this the tty stays missing until the cable is replugged,
    /// which is bafflingly hard to diagnose from the outside.
    fn drop(&mut self) {
        if let Err(e) = self.device.attach_kernel_driver(0) {
            tracing::debug!(error = %e, "could not hand the interface back to the kernel driver");
        }
    }
}

impl UsbTransport {
    /// Open the first FT232R, or the one whose USB serial string matches
    /// `serial` (the Mini-VCI's is stamped on the cable, e.g. `A69QL5OE`).
    pub fn open(serial: Option<&str>) -> Result<Self, TransportError> {
        let info = nusb::list_devices()
            .wait()
            .map_err(|e| io_err(e, "list USB devices"))?
            .find(|d| {
                d.vendor_id() == FTDI_VID
                    && d.product_id() == FT232R_PID
                    && serial.is_none_or(|want| d.serial_number() == Some(want))
            })
            .ok_or_else(|| TransportError::Open {
                port: serial.unwrap_or("any FT232R").to_string(),
                reason: format!("no {FTDI_VID:04x}:{FT232R_PID:04x} device found"),
            })?;

        let label = format!(
            "usb {:04x}:{:04x} {}",
            info.vendor_id(),
            info.product_id(),
            info.serial_number().unwrap_or("(no serial)")
        );

        let device = info.open().wait().map_err(|e| io_err(e, &label))?;
        // On Linux `ftdi_sio` has already claimed this interface and is
        // presenting it as /dev/ttyUSBn; take it away for the duration.
        let interface =
            device.detach_and_claim_interface(0).wait().map_err(|e| io_err(e, &label))?;

        let ep_in = interface.endpoint::<Bulk, In>(EP_IN).map_err(|e| io_err(e, &label))?;
        let ep_out = interface.endpoint::<Bulk, Out>(EP_OUT).map_err(|e| io_err(e, &label))?;
        let max_packet = ep_in.max_packet_size();

        let t = Self { device, interface, ep_in, ep_out, rx: VecDeque::new(), max_packet, label };

        // Reset the SIO engine, then 115200 8N1 to match the cable's fixed
        // host-side rate.
        t.control(SIO_RESET, SIO_RESET_SIO)?;
        t.control(SIO_SET_BAUDRATE, BAUD_115200_DIVISOR)?;
        t.control(SIO_SET_DATA, DATA_8N1)?;
        Ok(t)
    }

    fn control(&self, request: u8, value: u16) -> Result<(), TransportError> {
        self.interface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index: 0,
                    data: &[],
                },
                Duration::from_millis(1000),
            )
            .wait()
            .map_err(|e| io_err(e, &self.label))?;
        Ok(())
    }

    /// Every IN packet begins with two modem-status bytes that are *not*
    /// protocol data. A read that returns exactly the header is the FT232R
    /// reporting "nothing to say", which it does once per latency interval.
    fn absorb(&mut self, data: &[u8]) {
        for packet in data.chunks(self.max_packet) {
            if packet.len() > 2 {
                self.rx.extend(&packet[2..]);
            }
        }
    }

    fn take(&mut self, buf: &mut [u8]) -> usize {
        let n = buf.len().min(self.rx.len());
        for slot in buf.iter_mut().take(n) {
            *slot = self.rx.pop_front().unwrap();
        }
        n
    }
}

impl Transport for UsbTransport {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        let completion =
            self.ep_out.transfer_blocking(Buffer::from(buf.to_vec()), Duration::from_secs(2));
        completion.status.map_err(|e| io_err(e, &self.label))?;
        if completion.actual_len != buf.len() {
            return Err(io_err(
                format!("short write: {} of {} bytes", completion.actual_len, buf.len()),
                &self.label,
            ));
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError> {
        if !self.rx.is_empty() {
            return Ok(self.take(buf));
        }
        let deadline = Instant::now() + timeout;
        loop {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Ok(0);
            }
            // Ask for several packets at once; the device returns as soon as
            // it has anything, so this is not a latency penalty.
            let want = self.max_packet * 8;
            let completion = self
                .ep_in
                .transfer_blocking(Buffer::new(want), remaining.min(Duration::from_millis(200)));
            match completion.status {
                Ok(()) => {
                    let n = completion.actual_len;
                    self.absorb(&completion.buffer[..n]);
                    if !self.rx.is_empty() {
                        return Ok(self.take(buf));
                    }
                }
                // A timeout on this transfer just means the device had
                // nothing yet; keep waiting until the caller's deadline.
                Err(nusb::transfer::TransferError::Cancelled) => {}
                Err(e) => return Err(io_err(e, &self.label)),
            }
        }
    }

    fn purge_rx(&mut self) -> Result<(), TransportError> {
        self.rx.clear();
        self.control(SIO_RESET, SIO_RESET_PURGE_RX)
    }

    fn set_modem(&mut self, dtr: bool, rts: bool) -> Result<(), TransportError> {
        // High byte is the write mask, low byte the value.
        self.control(SIO_MODEM_CTRL, if dtr { 0x0101 } else { 0x0100 })?;
        self.control(SIO_MODEM_CTRL, if rts { 0x0202 } else { 0x0200 })
    }

    /// The whole point of this backend: a vendor control transfer, so no
    /// root and no udev rule.
    fn optimize_latency(&mut self) -> LatencyResult {
        match self.control(SIO_SET_LATENCY_TIMER, TARGET_LATENCY_MS as u16) {
            Ok(()) => LatencyResult::Set { millis: TARGET_LATENCY_MS },
            Err(e) => LatencyResult::Failed { reason: e.to_string() },
        }
    }
}
