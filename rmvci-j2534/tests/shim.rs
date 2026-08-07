//! In-process tests of the 14 PassThru exports against MockTransport-backed
//! devices. The shim's slot table and factory hook are process-global, so
//! everything runs in one sequential #[test].

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use rmvci_core::codec::{frame, inner};
use rmvci_core::transport::mock::{MockClock, MockTransport, Step};
use rmvci_core::types::ProtocolId;
use rmvci_core::{Device, DeviceConfig};
use rmvci_j2534::consts::*;
use rmvci_j2534::{PassthruMsg, SConfig, SConfigList, _set_device_factory};

const KEY_OLD: [u8; 8] = [0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f];

fn enc(inner_bytes: &[u8]) -> Vec<u8> {
    frame::encode_encrypted(&KEY_OLD, inner_bytes).unwrap()
}

fn status_reply(cmd: u8, status: u8) -> Vec<u8> {
    enc(&[0x02, 0x00, cmd, status])
}

fn handshake_steps() -> Vec<Step> {
    vec![
        Step::exchange(frame::encode_plain(&[]).unwrap(), Vec::new()),
        Step::exchange(
            frame::encode_plain(&inner::identify()).unwrap(),
            vec![0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9],
        ),
    ]
}

fn boxed_msg(data: &[u8], tx_flags: u32) -> Box<PassthruMsg> {
    let mut m: Box<PassthruMsg> = unsafe { Box::new(std::mem::zeroed()) };
    m.data[..data.len()].copy_from_slice(data);
    m.data_size = data.len() as u32;
    m.tx_flags = tx_flags;
    m
}

/// The full script for the main scenario device, in exact wire order.
fn main_device_steps() -> Vec<Step> {
    let mut steps = handshake_steps();

    // PassThruConnect(alias 0x10000001) -> wire proto must be 6.
    steps.push(Step::exchange(
        enc(&inner::connect(ProtocolId::Iso15765, 0, 500_000)),
        status_reply(0x07, 0x00),
    ));

    // StartMsgFilter FLOW_CONTROL 7CC/7C4, first shim filter id 0x000e7a01.
    steps.push(Step::exchange(
        enc(&inner::start_filter(
            ProtocolId::Iso15765,
            0x000e_7a01,
            3,
            &[0xff, 0xff, 0xff, 0xff],
            &[0x00, 0x00, 0x07, 0xcc],
            Some(&[0x00, 0x00, 0x07, 0xc4]),
            4,
        )
        .unwrap()),
        status_reply(0x0b, 0x00),
    ));

    // SET_CONFIG with a 2-entry SCONFIG_LIST fans out into two wire calls.
    steps.push(Step::exchange(
        enc(&inner::set_config(ProtocolId::Iso15765, 0x1e, 0)),
        status_reply(0x0e, 0x00),
    ));
    steps.push(Step::exchange(
        enc(&inner::set_config(ProtocolId::Iso15765, 0x1f, 10)),
        status_reply(0x0e, 0x00),
    ));

    // WriteMsgs: 7C4 21 43.
    steps.push(Step::exchange(
        enc(&inner::write_msg(ProtocolId::Iso15765, 0, &[0x00, 0x00, 0x07, 0xc4, 0x21, 0x43])
            .unwrap()),
        status_reply(0x0a, 0x00),
    ));

    // ReadMsgs: one poll returning 7CC 61 43 7B 79.
    let read_reply: Vec<u8> = {
        let payload = [0x00, 0x00, 0x07, 0xcc, 0x61, 0x43, 0x7b, 0x79];
        let mut v = Vec::new();
        v.extend_from_slice(&((9 + payload.len()) as u16).to_le_bytes());
        v.push(0x09);
        v.extend_from_slice(&[0; 8]);
        v.extend_from_slice(&payload);
        v
    };
    steps.push(Step::exchange(enc(&inner::read_poll(ProtocolId::Iso15765)), enc(&read_reply)));

    // StopMsgFilter, Disconnect, and the close-time session teardown.
    steps.push(Step::exchange(
        enc(&inner::stop_filter(ProtocolId::Iso15765, 0x000e_7a01)),
        status_reply(0x0c, 0x00),
    ));
    steps.push(Step::exchange(
        enc(&inner::disconnect(ProtocolId::Iso15765)),
        status_reply(0x08, 0x00),
    ));
    steps.push(Step::exchange(enc(&inner::session_close()), Vec::new()));
    steps
}

#[test]
fn shim_end_to_end() {
    let opened = Arc::new(AtomicUsize::new(0));
    let opened_in_factory = Arc::clone(&opened);
    _set_device_factory(Box::new(move |_port| {
        let n = opened_in_factory.fetch_add(1, Ordering::SeqCst);
        // Device #0 gets the fully-scripted scenario; later ones (slot
        // exhaustion test) only need to open and close.
        let steps = if n == 0 {
            main_device_steps()
        } else {
            let mut s = handshake_steps();
            s.push(Step::reply_any(Vec::new())); // session close on drop
            s
        };
        Device::open_transport(
            MockTransport::new(steps),
            DeviceConfig {
                port: None,
                keepalive: Duration::from_secs(600),
                clock: Arc::new(MockClock::default()),
            },
        )
    }));

    // SAFETY: every pointer handed to the exports below is either null (to
    // exercise the null checks) or derives from a live local allocation.
    unsafe {
    // --- invalid ids are rejected before any device exists ---
    assert_eq!(
        rmvci_j2534::PassThruReadVersion(1, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut()),
        ERR_INVALID_DEVICE_ID
    );

    // --- open ---
    let mut dev_id = 0;
    assert_eq!(rmvci_j2534::PassThruOpen(std::ptr::null(), &mut dev_id), STATUS_NOERROR);
    assert_eq!(dev_id, 1);

    // --- ReadVersion reports the Techstream-compatible identity ---
    let mut fw = [0i8; 80];
    let mut dll = [0i8; 80];
    let mut api = [0i8; 80];
    assert_eq!(
        rmvci_j2534::PassThruReadVersion(dev_id, fw.as_mut_ptr(), dll.as_mut_ptr(), api.as_mut_ptr()),
        STATUS_NOERROR
    );
    let cstr =
        |b: &[i8]| core::ffi::CStr::from_ptr(b.as_ptr()).to_str().unwrap().to_owned();
    assert_eq!(cstr(&fw), "J2534 MINIV1.03");
    assert_eq!(cstr(&dll), "MVCI J2534 DLL v1.4.6");
    assert_eq!(cstr(&api), "04.04");

    // --- a wedge-inducing protocol id is refused client-side ---
    let mut ch_id = 0;
    assert_eq!(
        rmvci_j2534::PassThruConnect(dev_id, 7, 0, 500_000, &mut ch_id),
        ERR_INVALID_PROTOCOL_ID
    );
    let mut errbuf = [0i8; 80];
    assert_eq!(rmvci_j2534::PassThruGetLastError(errbuf.as_mut_ptr()), STATUS_NOERROR);
    assert!(cstr(&errbuf).contains("unsupported protocol"), "got: {}", cstr(&errbuf));

    // --- connect via the vendor alias; the wire carries protocol 6 ---
    assert_eq!(
        rmvci_j2534::PassThruConnect(dev_id, 0x1000_0001, 0, 500_000, &mut ch_id),
        STATUS_NOERROR
    );
    assert_eq!(ch_id, 0x100);

    // --- flow-control filter ---
    let mask = boxed_msg(&[0xff, 0xff, 0xff, 0xff], 0);
    let pattern = boxed_msg(&[0x00, 0x00, 0x07, 0xcc], 0);
    let flow = boxed_msg(&[0x00, 0x00, 0x07, 0xc4], 0);
    let mut filter_id = 0;
    assert_eq!(
        rmvci_j2534::PassThruStartMsgFilter(
            ch_id,
            FLOW_CONTROL_FILTER as u64 as _,
            Box::into_raw(mask),
            Box::into_raw(pattern),
            Box::into_raw(flow),
            &mut filter_id,
        ),
        STATUS_NOERROR
    );
    assert_eq!(filter_id, 0x000e_7a01);

    // --- SCONFIG_LIST fan-out ---
    let mut cfgs = [SConfig { parameter: 0x1e, value: 0 }, SConfig { parameter: 0x1f, value: 10 }];
    let mut list = SConfigList { num_of_params: 2, config_ptr: cfgs.as_mut_ptr() };
    assert_eq!(
        rmvci_j2534::PassThruIoctl(
            ch_id,
            SET_CONFIG as _,
            (&raw mut list).cast(),
            std::ptr::null_mut()
        ),
        STATUS_NOERROR
    );

    // --- write + read ---
    let wmsg = boxed_msg(&[0x00, 0x00, 0x07, 0xc4, 0x21, 0x43], 0);
    let mut n = 1u64 as _;
    assert_eq!(
        rmvci_j2534::PassThruWriteMsgs(ch_id, Box::into_raw(wmsg), &mut n, 0),
        STATUS_NOERROR
    );
    assert_eq!(n, 1);

    let mut rmsg: Box<PassthruMsg> = Box::new(std::mem::zeroed());
    let mut n = 1u64 as _;
    assert_eq!(
        rmvci_j2534::PassThruReadMsgs(ch_id, &mut *rmsg, &mut n, 1000),
        STATUS_NOERROR
    );
    assert_eq!(n, 1);
    assert_eq!(rmsg.data_size, 8);
    assert_eq!(&rmsg.data[..8], &[0x00, 0x00, 0x07, 0xcc, 0x61, 0x43, 0x7b, 0x79]);
    assert_eq!(rmsg.protocol_id, 6);

    // --- stop filter, disconnect, close ---
    assert_eq!(rmvci_j2534::PassThruStopMsgFilter(ch_id, filter_id), STATUS_NOERROR);
    assert_eq!(rmvci_j2534::PassThruDisconnect(ch_id), STATUS_NOERROR);
    assert_eq!(rmvci_j2534::PassThruDisconnect(ch_id), ERR_INVALID_CHANNEL_ID);
    assert_eq!(rmvci_j2534::PassThruClose(dev_id), STATUS_NOERROR);
    assert_eq!(rmvci_j2534::PassThruClose(dev_id), ERR_INVALID_DEVICE_ID);

    // --- slot exhaustion: 4 devices fit, the 5th is refused ---
    let mut ids = Vec::new();
    for _ in 0..4 {
        let mut id = 0;
        assert_eq!(rmvci_j2534::PassThruOpen(std::ptr::null(), &mut id), STATUS_NOERROR);
        ids.push(id);
    }
    let mut id5 = 0;
    assert_eq!(rmvci_j2534::PassThruOpen(std::ptr::null(), &mut id5), ERR_FAILED);
    for id in ids {
        assert_eq!(rmvci_j2534::PassThruClose(id), STATUS_NOERROR);
    }

    // --- stubs stay stubs ---
    assert_eq!(rmvci_j2534::PassThruStopPeriodicMsg(0x100, 1), ERR_NOT_SUPPORTED);
    assert_eq!(rmvci_j2534::PassThruSetProgrammingVoltage(1, 15, 0), ERR_NOT_SUPPORTED);
    }
}
