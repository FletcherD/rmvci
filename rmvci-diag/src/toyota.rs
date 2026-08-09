//! Toyota-specific KWP services and conventions that aren't in the standard
//! ISO 14230-3 catalogue, recovered from the Techstream database RE.

use automotive_diag::obd2::Obd2Command;
use rmvci_core::UdsTransport;

use crate::error::Result;
use crate::kwp2000::Kwp2000;

/// Toyota proprietary customize-write service id (`A5 <LID> <value>`), used by
/// ECUs whose customize descriptor is `0x11` (legacy KWP). Not part of ISO
/// 14230; see `re/techstream/ghidra/notes_customize.md`.
pub const SID_TOYOTA_WRITE: u8 = 0xa5;

impl<T: UdsTransport> Kwp2000<T> {
    /// Toyota active test — drive an actuator to a state via
    /// inputOutputControlByLocalIdentifier: `30 <actuator-id> <state>`.
    ///
    /// e.g. the Prius A/C air-mix damper (D) is id `0x03`, so
    /// `active_test(0x03, vv)` sends `30 03 <vv>`. `state` is a relay ON bit
    /// mask or an analog 0–255 drive value depending on the actuator.
    ///
    /// **Active tests command real hardware.** Only issue them with the vehicle
    /// in the state the service manual requires.
    pub fn active_test(&mut self, actuator_id: u8, state: u8) -> Result<Vec<u8>> {
        self.io_control_local(actuator_id, &[state])
    }

    /// Toyota proprietary customize write (`A5 <LID> <value>`), for descriptor
    /// `0x11` ECUs. Descriptor `0x12`/`0x22` ECUs use
    /// [`write_data_by_local_id`](Kwp2000::write_data_by_local_id) (`3B`)
    /// instead. The value is caller-chosen (never stored in the DB).
    pub fn customize_write_toyota(&mut self, lid: u8, value: u8) -> Result<Vec<u8>> {
        self.send_raw(SID_TOYOTA_WRITE, &[lid, value])
    }

    // --------------------------------------------------------------- OBD-II

    /// OBD-II service 01 (current data): `01 <PID>`. Returns the data bytes
    /// after the echoed PID.
    pub fn obd_current_data(&mut self, pid: u8) -> Result<Vec<u8>> {
        let resp = self.send_raw(Obd2Command::Service01 as u8, &[pid])?;
        // Response is `41 <pid> <data>`; send_raw already stripped 0x41.
        match resp.split_first() {
            Some((&echo, data)) if echo == pid => Ok(data.to_vec()),
            _ => Ok(resp),
        }
    }

    /// OBD-II service 03 (stored DTCs). Returns the raw code payload.
    pub fn obd_stored_dtcs(&mut self) -> Result<Vec<u8>> {
        self.send_raw(Obd2Command::Service03 as u8, &[])
    }
}
