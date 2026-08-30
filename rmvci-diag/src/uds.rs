use std::time::Duration;

use automotive_diag::uds::{DtcSubFunction, UdsCommand};
use rmvci_core::UdsTransport;

use crate::common::{parse_positive, strip_echo};
use crate::error::{Error, Result};

/// A UDS (ISO 14229-1) diagnostic client over any [`UdsTransport`].
///
/// The counterpart to [`crate::Kwp2000`] for the newer Toyota P5 (UDS) ECUs,
/// which speak 2-byte data identifiers (DIDs), `0x2F` I/O control, and `0x31`
/// routines. The transport handles framing and responsePending (`7F .. 78`);
/// this layer builds service requests and validates responses, returning the
/// bytes after the echoed service id / identifier and mapping a `7F <sid> <nrc>`
/// reply to [`Error::Negative`].
///
/// ```no_run
/// use rmvci_core::{CanId, Device, IsoTp, IsoTpConfig, IsoTpPath};
/// use rmvci_diag::Uds;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let dev = Device::open("/dev/ttyUSB0")?;
/// let cfg = IsoTpConfig::new(CanId::Std(0x7e0), CanId::Std(0x7e8));
/// let isotp = IsoTp::new(&dev, cfg, IsoTpPath::Firmware)?;
/// let mut uds = Uds::new(isotp);
/// uds.diagnostic_session_control(0x03)?;          // extended session
/// let vin = uds.read_data_by_id(0xf190)?;         // 22 F1 90
/// println!("VIN bytes: {vin:02x?}");
/// # Ok(()) }
/// ```
pub struct Uds<T: UdsTransport> {
    io: T,
    timeout: Duration,
}

impl<T: UdsTransport> Uds<T> {
    /// Wrap a ready transport (typically an ISO-TP channel to a P5 ECU).
    pub fn new(io: T) -> Self {
        Self { io, timeout: Duration::from_secs(2) }
    }

    /// Set the per-request timeout (default 2 s); responsePending extends it
    /// transparently in the transport.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Change the per-request timeout in place.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    /// Borrow the underlying transport.
    pub fn transport(&mut self) -> &mut T {
        &mut self.io
    }

    /// Recover the underlying transport.
    pub fn into_inner(self) -> T {
        self.io
    }

    /// Core exchange: send `sid` + `data`, require the positive echo
    /// (`sid | 0x40`), return the bytes after it. `7F <sid> <nrc>` becomes
    /// [`Error::Negative`].
    pub fn send_raw(&mut self, sid: u8, data: &[u8]) -> Result<Vec<u8>> {
        let mut req = Vec::with_capacity(1 + data.len());
        req.push(sid);
        req.extend_from_slice(data);
        let resp = self.io.request(&req, self.timeout)?;
        parse_positive(sid, resp)
    }

    /// [`send_raw`](Self::send_raw) with a typed [`UdsCommand`].
    pub fn send(&mut self, service: UdsCommand, data: &[u8]) -> Result<Vec<u8>> {
        self.send_raw(service.into(), data)
    }

    // ---------------------------------------------------------------- session

    /// diagnosticSessionControl (0x10).
    pub fn diagnostic_session_control(&mut self, session: u8) -> Result<Vec<u8>> {
        self.send(UdsCommand::DiagnosticSessionControl, &[session])
    }

    /// ecuReset (0x11).
    pub fn ecu_reset(&mut self, reset_type: u8) -> Result<Vec<u8>> {
        self.send(UdsCommand::ECUReset, &[reset_type])
    }

    /// testerPresent (0x3E). `suppress_response` sets the 0x80 suppress bit; a
    /// suppressed request that draws no reply is treated as success.
    pub fn tester_present(&mut self, suppress_response: bool) -> Result<()> {
        let sub = if suppress_response { 0x80 } else { 0x00 };
        match self.send(UdsCommand::TesterPresent, &[sub]) {
            Ok(_) => Ok(()),
            Err(Error::Transport(rmvci_core::Error::Timeout(_))) if suppress_response => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// securityAccess (0x27): `sub` is the requestSeed/sendKey sub-function,
    /// `data` the key material (empty for a seed request).
    pub fn security_access(&mut self, sub: u8, data: &[u8]) -> Result<Vec<u8>> {
        let mut d = Vec::with_capacity(1 + data.len());
        d.push(sub);
        d.extend_from_slice(data);
        self.send(UdsCommand::SecurityAccess, &d)
    }

    /// communicationControl (0x28).
    pub fn communication_control(&mut self, control_type: u8, comm_type: u8) -> Result<Vec<u8>> {
        self.send(UdsCommand::CommunicationControl, &[control_type, comm_type])
    }

    /// controlDTCSetting (0x85): `0x01` on, `0x02` off.
    pub fn control_dtc_setting(&mut self, setting: u8) -> Result<Vec<u8>> {
        self.send(UdsCommand::ControlDTCSetting, &[setting])
    }

    // ------------------------------------------------------------------- data

    /// readDataByIdentifier (0x22, 2-byte DID). Returns the data after the
    /// echoed DID.
    pub fn read_data_by_id(&mut self, did: u16) -> Result<Vec<u8>> {
        let resp = self.send(UdsCommand::ReadDataByIdentifier, &did.to_be_bytes())?;
        strip_echo(&resp, &did.to_be_bytes())
    }

    /// writeDataByIdentifier (0x2E, 2-byte DID). **Writes real configuration.**
    pub fn write_data_by_id(&mut self, did: u16, value: &[u8]) -> Result<Vec<u8>> {
        let d = did.to_be_bytes();
        let mut req = Vec::with_capacity(2 + value.len());
        req.extend_from_slice(&d);
        req.extend_from_slice(value);
        self.send(UdsCommand::WriteDataByIdentifier, &req)
    }

    // ------------------------------------------------------------------- DTCs

    /// clearDiagnosticInformation (0x14, 3-byte group; `0xFFFFFF` = all).
    pub fn clear_diagnostic_information(&mut self, group: u32) -> Result<Vec<u8>> {
        let g = group.to_be_bytes();
        self.send(UdsCommand::ClearDiagnosticInformation, &[g[1], g[2], g[3]])
    }

    /// readDTCInformation (0x19) with a report-type sub-function and its
    /// parameters, e.g. `read_dtc_information(0x02, &[0xFF])` =
    /// reportDTCByStatusMask, all.
    pub fn read_dtc_information(&mut self, sub_function: u8, params: &[u8]) -> Result<Vec<u8>> {
        let mut d = Vec::with_capacity(1 + params.len());
        d.push(sub_function);
        d.extend_from_slice(params);
        self.send(UdsCommand::ReadDTCInformation, &d)
    }

    /// reportDTCByStatusMask (0x19 0x02 `<mask>`) — the common "read all DTCs
    /// matching this status" call. Returns the response after `59 02`.
    pub fn read_dtc_by_status_mask(&mut self, status_mask: u8) -> Result<Vec<u8>> {
        let sub = DtcSubFunction::ReportDtcByStatusMask as u8;
        let resp = self.read_dtc_information(sub, &[status_mask])?;
        strip_echo(&resp, &[sub])
    }

    // -------------------------------------------------------- actuators / routine

    /// inputOutputControlByIdentifier (0x2F): `2F <DID_hi> <DID_lo> <control>
    /// <params…>`. On Toyota P5 this is the active test (control `0x03` =
    /// shortTermAdjustment). **Commands real hardware.**
    pub fn io_control_by_id(&mut self, did: u16, control: u8, params: &[u8]) -> Result<Vec<u8>> {
        let d = did.to_be_bytes();
        let mut req = Vec::with_capacity(3 + params.len());
        req.extend_from_slice(&d);
        req.push(control);
        req.extend_from_slice(params);
        let resp = self.send(UdsCommand::InputOutputControlByIdentifier, &req)?;
        strip_echo(&resp, &d)
    }

    /// routineControl (0x31): `sub` is `0x01` start / `0x02` stop / `0x03`
    /// requestResults, then the 2-byte RID and optional params.
    pub fn routine_control(&mut self, sub: u8, rid: u16, params: &[u8]) -> Result<Vec<u8>> {
        let r = rid.to_be_bytes();
        let mut req = Vec::with_capacity(3 + params.len());
        req.push(sub);
        req.extend_from_slice(&r);
        req.extend_from_slice(params);
        self.send(UdsCommand::RoutineControl, &req)
    }
}
