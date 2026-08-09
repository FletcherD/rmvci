use std::time::Duration;

use automotive_diag::kwp2000::KwpCommand;
use rmvci_core::UdsTransport;

use crate::common::{parse_positive, strip_echo};
use crate::error::{Error, Result};

/// A KWP2000 (ISO 14230-3) diagnostic client over any [`UdsTransport`].
///
/// The transport handles framing and responsePending (`7F .. 78`); this layer
/// builds service requests and validates responses. Every method maps an ECU
/// negative response to [`Error::Negative`] and returns the response bytes
/// *after* the echoed service id / identifier.
///
/// ```no_run
/// use rmvci_core::{Device, DeviceConfig, KLineEcu};
/// use rmvci_diag::Kwp2000;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let dev = Device::open("/dev/ttyUSB0")?;
/// let mut ecu = KLineEcu::connect(&dev, 0x98)?; // Prius A/C amp
/// ecu.fast_init()?;
/// let mut kwp = Kwp2000::new(ecu);
/// let ambient = kwp.read_data_by_local_id(0x02)?; // 21 02 -> data
/// println!("ambient raw: {:?}", ambient);
/// # Ok(()) }
/// ```
pub struct Kwp2000<T: UdsTransport> {
    io: T,
    timeout: Duration,
}

impl<T: UdsTransport> Kwp2000<T> {
    /// Wrap a ready transport (K-line already FAST_INIT'd, or an ISO-TP channel).
    pub fn new(io: T) -> Self {
        Self { io, timeout: Duration::from_secs(2) }
    }

    /// Set the per-request timeout (default 2 s). This is the wait for one
    /// response; responsePending extends it transparently in the transport.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Change the per-request timeout in place.
    pub fn set_timeout(&mut self, timeout: Duration) {
        self.timeout = timeout;
    }

    /// Borrow the underlying transport (e.g. to re-init a K-line session).
    pub fn transport(&mut self) -> &mut T {
        &mut self.io
    }

    /// Recover the underlying transport.
    pub fn into_inner(self) -> T {
        self.io
    }

    /// Core exchange: send `sid` + `data`, then require the positive echo
    /// (`sid | 0x40`) and return the bytes after it. A `7F <sid> <nrc>` reply
    /// becomes [`Error::Negative`].
    pub fn send_raw(&mut self, sid: u8, data: &[u8]) -> Result<Vec<u8>> {
        let mut req = Vec::with_capacity(1 + data.len());
        req.push(sid);
        req.extend_from_slice(data);
        let resp = self.io.request(&req, self.timeout)?;
        parse_positive(sid, resp)
    }

    /// [`send_raw`](Self::send_raw) with a typed [`KwpCommand`].
    pub fn send(&mut self, service: KwpCommand, data: &[u8]) -> Result<Vec<u8>> {
        self.send_raw(service.into(), data)
    }

    // ---------------------------------------------------------------- session

    /// startDiagnosticSession (0x10) with a session-type byte.
    pub fn start_diagnostic_session(&mut self, session: u8) -> Result<Vec<u8>> {
        self.send(KwpCommand::StartDiagnosticSession, &[session])
    }

    /// stopDiagnosticSession (0x20).
    pub fn stop_diagnostic_session(&mut self) -> Result<Vec<u8>> {
        self.send_raw(0x20, &[])
    }

    /// ecuReset (0x11) with a reset-type byte.
    pub fn ecu_reset(&mut self, reset_type: u8) -> Result<Vec<u8>> {
        self.send(KwpCommand::ECUReset, &[reset_type])
    }

    /// testerPresent (0x3E). With `response_required = false` (sub 0x02), an ECU
    /// that stays silent is treated as success.
    pub fn tester_present(&mut self, response_required: bool) -> Result<()> {
        let sub = if response_required { 0x01 } else { 0x02 };
        match self.send(KwpCommand::TesterPresent, &[sub]) {
            Ok(_) => Ok(()),
            Err(Error::Transport(rmvci_core::Error::Timeout(_))) if !response_required => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// securityAccess (0x27): `sub` is the requestSeed/sendKey sub-function,
    /// `data` the (empty for seed request) key material.
    pub fn security_access(&mut self, sub: u8, data: &[u8]) -> Result<Vec<u8>> {
        let mut d = Vec::with_capacity(1 + data.len());
        d.push(sub);
        d.extend_from_slice(data);
        self.send(KwpCommand::SecurityAccess, &d)
    }

    // ------------------------------------------------------------------- data

    /// readDataByLocalIdentifier (0x21) — the Toyota data-list read. Returns the
    /// data bytes after the echoed LID.
    pub fn read_data_by_local_id(&mut self, lid: u8) -> Result<Vec<u8>> {
        let resp = self.send(KwpCommand::ReadDataByLocalIdentifier, &[lid])?;
        strip_echo(&resp, &[lid])
    }

    /// readDataByIdentifier (0x22, 2-byte identifier). Returns the data bytes
    /// after the echoed identifier.
    pub fn read_data_by_id(&mut self, id: u16) -> Result<Vec<u8>> {
        let resp = self.send(KwpCommand::ReadDataByIdentifier, &id.to_be_bytes())?;
        strip_echo(&resp, &id.to_be_bytes())
    }

    // ------------------------------------------------------------------- DTCs

    /// readDiagnosticTroubleCodesByStatus (0x18): `status_mask` then a 2-byte
    /// group (`0xFF00` = all groups).
    pub fn read_dtc_by_status(&mut self, status_mask: u8, group: u16) -> Result<Vec<u8>> {
        let g = group.to_be_bytes();
        self.send(KwpCommand::ReadDiagnosticTroubleCodesByStatus, &[status_mask, g[0], g[1]])
    }

    /// readStatusOfDiagnosticTroubleCodes (0x17): 2-byte group (`0xFF00` = all).
    pub fn read_status_of_dtc(&mut self, group: u16) -> Result<Vec<u8>> {
        self.send(KwpCommand::ReadStatusOfDiagnosticTroubleCodes, &group.to_be_bytes())
    }

    /// clearDiagnosticInformation (0x14): 2-byte group (`0xFF00` = all).
    pub fn clear_diagnostic_information(&mut self, group: u16) -> Result<Vec<u8>> {
        self.send(KwpCommand::ClearDiagnosticInformation, &group.to_be_bytes())
    }

    /// readFreezeFrameData (`12 <LID> <record>`) — a KWP-enhanced service absent
    /// from the standard [`KwpCommand`] set; sent by raw SID.
    pub fn read_freeze_frame(&mut self, lid: u8, record: u8) -> Result<Vec<u8>> {
        self.send_raw(0x12, &[lid, record])
    }

    // ------------------------------------------------------ actuators / write

    /// inputOutputControlByLocalIdentifier (0x30). On Toyota this is the active
    /// test: `30 <id> <params…>`.
    pub fn io_control_local(&mut self, id: u8, params: &[u8]) -> Result<Vec<u8>> {
        let mut d = Vec::with_capacity(1 + params.len());
        d.push(id);
        d.extend_from_slice(params);
        self.send(KwpCommand::InputOutputControlByLocalIdentifier, &d)
    }

    /// startRoutineByLocalIdentifier (0x31): `31 <id> <params…>`.
    pub fn start_routine_local(&mut self, id: u8, params: &[u8]) -> Result<Vec<u8>> {
        let mut d = Vec::with_capacity(1 + params.len());
        d.push(id);
        d.extend_from_slice(params);
        self.send(KwpCommand::StartRoutineByLocalIdentifier, &d)
    }

    /// writeDataByLocalIdentifier (0x3B): `3B <LID> <value…>`.
    pub fn write_data_by_local_id(&mut self, lid: u8, value: &[u8]) -> Result<Vec<u8>> {
        let mut d = Vec::with_capacity(1 + value.len());
        d.push(lid);
        d.extend_from_slice(value);
        self.send(KwpCommand::WriteDataByLocalIdentifier, &d)
    }
}

