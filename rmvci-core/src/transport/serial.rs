//! Serial backend over the kernel `ftdi_sio` driver, via the `serialport`
//! crate. 115200 8N1 raw, matching the cable's fixed host-side rate.

use std::io::Read;
use std::time::Duration;

use serialport::{ClearBuffer, SerialPort};

use super::{LatencyResult, Transport};
use crate::error::TransportError;
use crate::types::BAUD;

pub struct SerialTransport {
    port: Box<dyn SerialPort>,
    /// Original path, kept to locate the sysfs latency node.
    path: String,
}

impl SerialTransport {
    pub fn open(path: &str) -> Result<Self, TransportError> {
        let port = serialport::new(path, BAUD)
            .data_bits(serialport::DataBits::Eight)
            .parity(serialport::Parity::None)
            .stop_bits(serialport::StopBits::One)
            .flow_control(serialport::FlowControl::None)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| TransportError::Open { port: path.to_string(), reason: e.to_string() })?;
        Ok(Self { port, path: path.to_string() })
    }
}

impl Transport for SerialTransport {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        std::io::Write::write_all(&mut self.port, buf)?;
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError> {
        // serialport's timeout floor is 1 ms; a zero timeout would poll.
        self.port
            .set_timeout(timeout.max(Duration::from_millis(1)))
            .map_err(|e| TransportError::Open { port: self.path.clone(), reason: e.to_string() })?;
        match self.port.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(0),
            Err(e) => Err(e.into()),
        }
    }

    fn purge_rx(&mut self) -> Result<(), TransportError> {
        self.port
            .clear(ClearBuffer::Input)
            .map_err(|e| TransportError::Open { port: self.path.clone(), reason: e.to_string() })?;
        Ok(())
    }

    fn set_modem(&mut self, dtr: bool, rts: bool) -> Result<(), TransportError> {
        self.port
            .write_data_terminal_ready(dtr)
            .and_then(|_| self.port.write_request_to_send(rts))
            .map_err(|e| TransportError::Open { port: self.path.clone(), reason: e.to_string() })?;
        Ok(())
    }

    /// Try to drop the FTDI latency timer to 2 ms via sysfs. Needs write
    /// access to the node — root or a udev rule; failure is reported, not
    /// fatal (everything still works, just ~16 ms slower per exchange).
    fn optimize_latency(&mut self) -> LatencyResult {
        const TARGET: u8 = 2;

        // Resolve /dev/serial/by-id/... symlinks to the real ttyUSBn.
        let real = match std::fs::canonicalize(&self.path) {
            Ok(p) => p,
            Err(e) => return LatencyResult::Failed { reason: e.to_string() },
        };
        let Some(name) = real.file_name().and_then(|n| n.to_str()) else {
            return LatencyResult::Failed { reason: format!("odd device path {real:?}") };
        };
        let node = format!("/sys/bus/usb-serial/devices/{name}/latency_timer");

        let current = std::fs::read_to_string(&node).ok().and_then(|s| s.trim().parse::<u8>().ok());
        if let Some(ms) = current
            && ms <= TARGET
        {
            return LatencyResult::AlreadyLow { millis: ms };
        }
        match std::fs::write(&node, format!("{TARGET}\n")) {
            Ok(()) => LatencyResult::Set { millis: TARGET },
            Err(e) => LatencyResult::Failed {
                reason: format!(
                    "{node}: {e} (fix permanently with a udev rule: \
                     ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", \
                     ATTR{{latency_timer}}=\"2\")"
                ),
            },
        }
    }
}
