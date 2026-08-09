//! End-to-end client tests over a scripted `UdsTransport`: each step asserts
//! the exact request bytes the client emits and feeds back a canned response,
//! so request-building and response-parsing are checked byte-for-byte without
//! hardware.

use std::collections::VecDeque;
use std::time::Duration;

use rmvci_core::{Error, UdsTransport};
use rmvci_diag::Kwp2000;

/// A transport that replays `(expected_request, response)` steps in order.
struct Scripted {
    steps: VecDeque<(Vec<u8>, Vec<u8>)>,
}

impl Scripted {
    fn new(steps: impl IntoIterator<Item = (Vec<u8>, Vec<u8>)>) -> Self {
        Self { steps: steps.into_iter().collect() }
    }
}

impl UdsTransport for Scripted {
    fn request(&mut self, req: &[u8], _timeout: Duration) -> Result<Vec<u8>, Error> {
        let (expected, response) = self.steps.pop_front().expect("no more scripted steps");
        assert_eq!(req, expected.as_slice(), "request bytes mismatch");
        Ok(response)
    }
}

fn client(steps: impl IntoIterator<Item = (Vec<u8>, Vec<u8>)>) -> Kwp2000<Scripted> {
    Kwp2000::new(Scripted::new(steps))
}

#[test]
fn read_data_by_local_id_strips_sid_and_lid() {
    // 21 0D -> 61 0D 09  => data [09]
    let mut kwp = client([(vec![0x21, 0x0d], vec![0x61, 0x0d, 0x09])]);
    assert_eq!(kwp.read_data_by_local_id(0x0d).unwrap(), vec![0x09]);
}

#[test]
fn ambient_read_matches_the_live_prius() {
    // 21 02 -> 61 02 A4  (the value we actually saw on the car)
    let mut kwp = client([(vec![0x21, 0x02], vec![0x61, 0x02, 0xa4])]);
    assert_eq!(kwp.read_data_by_local_id(0x02).unwrap(), vec![0xa4]);
}

#[test]
fn read_data_by_id_uses_two_byte_identifier() {
    // 22 01 43 -> 62 01 43 7B 79
    let mut kwp = client([(vec![0x22, 0x01, 0x43], vec![0x62, 0x01, 0x43, 0x7b, 0x79])]);
    assert_eq!(kwp.read_data_by_id(0x0143).unwrap(), vec![0x7b, 0x79]);
}

#[test]
fn negative_response_out_of_range() {
    let mut kwp = client([(vec![0x21, 0x99], vec![0x7f, 0x21, 0x31])]);
    let err = kwp.read_data_by_local_id(0x99).unwrap_err();
    assert!(err.is_request_out_of_range());
    assert_eq!(format!("{err}").contains("0x31"), true);
}

#[test]
fn active_test_builds_io_control_frame() {
    // Prius air-mix damper (D) drive: 30 03 80 -> 70 03 80
    let mut kwp = client([(vec![0x30, 0x03, 0x80], vec![0x70, 0x03, 0x80])]);
    assert_eq!(kwp.active_test(0x03, 0x80).unwrap(), vec![0x03, 0x80]);
}

#[test]
fn toyota_customize_write_uses_a5() {
    // A5 1A 02 -> E5 1A 02  (positive echo = 0xA5 | 0x40)
    let mut kwp = client([(vec![0xa5, 0x1a, 0x02], vec![0xe5, 0x1a, 0x02])]);
    assert_eq!(kwp.customize_write_toyota(0x1a, 0x02).unwrap(), vec![0x1a, 0x02]);
}

#[test]
fn dtc_read_and_clear_frames() {
    let mut kwp = client([
        (vec![0x18, 0x00, 0xff, 0x00], vec![0x58, 0x02, 0x94, 0x00, 0x94, 0x10]),
        (vec![0x14, 0xff, 0x00], vec![0x54]),
    ]);
    assert_eq!(kwp.read_dtc_by_status(0x00, 0xff00).unwrap(), vec![0x02, 0x94, 0x00, 0x94, 0x10]);
    assert_eq!(kwp.clear_diagnostic_information(0xff00).unwrap(), Vec::<u8>::new());
}

#[test]
fn freeze_frame_uses_raw_sid_12() {
    let mut kwp = client([(vec![0x12, 0x41, 0xff], vec![0x52, 0x41, 0xff, 0xaa])]);
    assert_eq!(kwp.read_freeze_frame(0x41, 0xff).unwrap(), vec![0x41, 0xff, 0xaa]);
}

#[test]
fn session_and_tester_present() {
    let mut kwp = client([
        (vec![0x10, 0x81], vec![0x50, 0x81]),
        (vec![0x3e, 0x01], vec![0x7e, 0x01]),
    ]);
    assert_eq!(kwp.start_diagnostic_session(0x81).unwrap(), vec![0x81]);
    kwp.tester_present(true).unwrap();
}

#[test]
fn obd_current_data_strips_pid() {
    // 01 0C -> 41 0C 1A F8  (engine RPM), data after PID echo
    let mut kwp = client([(vec![0x01, 0x0c], vec![0x41, 0x0c, 0x1a, 0xf8])]);
    assert_eq!(kwp.obd_current_data(0x0c).unwrap(), vec![0x1a, 0xf8]);
}

// ---- UDS client, same scripted transport ----
use rmvci_diag::Uds;

fn uds(steps: impl IntoIterator<Item = (Vec<u8>, Vec<u8>)>) -> Uds<Scripted> {
    Uds::new(Scripted::new(steps))
}

#[test]
fn uds_read_data_by_id_strips_did() {
    // 22 F1 90 -> 62 F1 90 <vin...>
    let mut u = uds([(vec![0x22, 0xf1, 0x90], vec![0x62, 0xf1, 0x90, 0x31, 0x32])]);
    assert_eq!(u.read_data_by_id(0xf190).unwrap(), vec![0x31, 0x32]);
}

#[test]
fn uds_io_control_strips_did_echo() {
    // 2F 01 43 03 80 -> 6F 01 43 03 80  (P5 active test, control 03)
    let mut u = uds([(vec![0x2f, 0x01, 0x43, 0x03, 0x80], vec![0x6f, 0x01, 0x43, 0x03, 0x80])]);
    assert_eq!(u.io_control_by_id(0x0143, 0x03, &[0x80]).unwrap(), vec![0x03, 0x80]);
}

#[test]
fn uds_routine_and_session() {
    let mut u = uds([
        (vec![0x10, 0x03], vec![0x50, 0x03, 0x00, 0x32, 0x01, 0xf4]),
        (vec![0x31, 0x01, 0x02, 0x03], vec![0x71, 0x01, 0x02, 0x03]),
    ]);
    // 50 <session-echo> <P2> <P2*>: everything after the SID is returned.
    assert_eq!(u.diagnostic_session_control(0x03).unwrap(), vec![0x03, 0x00, 0x32, 0x01, 0xf4]);
    // routineControl returns everything after the SID: sub-func + RID + results.
    assert_eq!(u.routine_control(0x01, 0x0203, &[]).unwrap(), vec![0x01, 0x02, 0x03]);
}

#[test]
fn uds_read_dtc_by_status_mask_strips_subfunction() {
    // 19 02 FF -> 59 02 <dtc records>
    let mut u = uds([(vec![0x19, 0x02, 0xff], vec![0x59, 0x02, 0x94, 0x00, 0x2f])]);
    assert_eq!(u.read_dtc_by_status_mask(0xff).unwrap(), vec![0x94, 0x00, 0x2f]);
}

#[test]
fn uds_clear_uses_three_byte_group() {
    let mut u = uds([(vec![0x14, 0xff, 0xff, 0xff], vec![0x54])]);
    assert_eq!(u.clear_diagnostic_information(0xffffff).unwrap(), Vec::<u8>::new());
}

#[test]
fn uds_negative_shares_error_type() {
    let mut u = uds([(vec![0x22, 0xf1, 0x90], vec![0x7f, 0x22, 0x31])]);
    let e = u.read_data_by_id(0xf190).unwrap_err();
    assert!(e.is_request_out_of_range());
    assert_eq!(e.nrc(), Some(0x31));
}
