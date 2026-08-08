//! In-process tests of the 14 PassThru exports against MockTransport-backed
//! devices. The shim's slot table and factory hook are process-global, so
//! everything runs in one sequential #[test].

use core::ffi::c_ulong;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use rmvci_core::codec::{frame, inner};
use rmvci_core::transport::mock::{MockClock, MockTransport, Step};
use rmvci_core::types::ProtocolId;
use rmvci_core::{Device, DeviceConfig};
use rmvci_j2534::consts::*;
use rmvci_j2534::{_set_device_factory, PassthruMsg, SConfig, SConfigList};

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
            vec![
                0x0e, 0x00, 0x09, 0x00, 0x01, 0xb0, 0xcb, 0x49, 0x68, 0x07, 0x45, 0xc8, 0x7f, 0xa9,
            ],
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

    // StartPeriodicMsg: host-assigned id 0x000e7b02 (next_id is 2 after the
    // one filter), then StopPeriodicMsg with the same id.
    let periodic_msg = [0x00, 0x00, 0x07, 0xc4, 0x3e, 0x00];
    steps.push(Step::exchange(
        enc(&inner::start_periodic(ProtocolId::Iso15765, 0x000e_7b02, 0, 1000, &periodic_msg)
            .unwrap()),
        status_reply(0x0f, 0x00),
    ));
    steps.push(Step::exchange(
        enc(&inner::stop_periodic(ProtocolId::Iso15765, 0x000e_7b02)),
        status_reply(0x10, 0x00),
    ));

    // GET_CONFIG(0x1e) -> value 8 (assumed reply layout).
    steps.push(Step::exchange(enc(&inner::get_config(ProtocolId::Iso15765, 0x1e)), {
        enc(&[0x05, 0x00, 0x0e, 0x08, 0x00, 0x00, 0x00])
    }));
    // READ_VBATT -> 12000 mV.
    steps.push(Step::exchange(enc(&inner::read_vbatt(ProtocolId::Iso15765)), {
        enc(&[0x05, 0x00, 0x0e, 0xe0, 0x2e, 0x00, 0x00])
    }));
    // SetProgrammingVoltage(pin 1, 5000 mV) — device-scoped passthrough.
    steps.push(Step::exchange(
        enc(&inner::set_programming_voltage(1, 5000)),
        status_reply(0x0d, 0x00),
    ));

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

/// A read-poll exchange delivering one CAN message.
fn poll_delivers(proto: ProtocolId, rx_status: u32, data: &[u8]) -> Step {
    let mut reply = Vec::new();
    reply.extend_from_slice(&((9 + data.len()) as u16).to_le_bytes());
    reply.push(0x09);
    reply.extend_from_slice(&rx_status.to_le_bytes());
    reply.extend_from_slice(&[0; 4]);
    reply.extend_from_slice(data);
    Step::exchange(enc(&inner::read_poll(proto)), enc(&reply))
}

/// Script for a **host-path** ISO15765 channel (protocol 5 + host ISO-TP
/// machines): connect CAN, the endpoint filter, a single-frame `21 43` round
/// trip, and a 260-byte write that the firmware path could never segment
/// (FF + FC + consecutive frames) — the whole point of the host path.
fn host_device_steps() -> Vec<Step> {
    const TX: [u8; 4] = [0x00, 0x00, 0x07, 0xc4];
    const RX: [u8; 4] = [0x00, 0x00, 0x07, 0xcc];
    let mut steps = handshake_steps();

    // Connect proto 5 (the vendor host-isotp flag is stripped -> flags 0).
    steps.push(Step::exchange(enc(&inner::connect(ProtocolId::Can, 0, 500_000)), status_reply(0x07, 0x00)));
    // set_endpoints installs an exact PASS filter on rx (7cc); first id 0x000e7a00.
    steps.push(Step::exchange(
        enc(&inner::start_filter(ProtocolId::Can, 0x000e_7a00, 1, &[0xff; 4], &RX, None, 4).unwrap()),
        status_reply(0x0b, 0x00),
    ));

    // Small write 21 43 (single frame, padded to 8).
    steps.push(Step::exchange(
        enc(&inner::write_msg(ProtocolId::Can, 0, &{
            let mut m = TX.to_vec();
            m.extend_from_slice(&[0x02, 0x21, 0x43, 0, 0, 0, 0, 0]);
            m
        })
        .unwrap()),
        status_reply(0x0a, 0x00),
    ));
    steps.push(poll_delivers(ProtocolId::Can, 0, &{
        let mut m = RX.to_vec();
        m.extend_from_slice(&[0x04, 0x61, 0x43, 0x7b, 0x79, 0, 0, 0]);
        m
    }));

    // Large write: 260 bytes -> First Frame + Flow Control + Consecutive Frames.
    let payload: Vec<u8> = (0..260u16).map(|i| i as u8).collect();
    steps.push(Step::exchange(
        enc(&inner::write_msg(ProtocolId::Can, 0, &{
            let mut m = TX.to_vec();
            m.push(0x10 | (payload.len() >> 8) as u8);
            m.push(payload.len() as u8);
            m.extend_from_slice(&payload[..6]);
            m
        })
        .unwrap()),
        status_reply(0x0a, 0x00),
    ));
    // ECU flow control: CTS, BS 0, STmin 0.
    steps.push(poll_delivers(ProtocolId::Can, 0, &{
        let mut m = RX.to_vec();
        m.extend_from_slice(&[0x30, 0, 0, 0, 0, 0, 0, 0]);
        m
    }));
    let (mut off, mut sn) = (6usize, 1u8);
    while off < payload.len() {
        let take = (payload.len() - off).min(7);
        steps.push(Step::exchange(
            enc(&inner::write_msg(ProtocolId::Can, 0, &{
                let mut m = TX.to_vec();
                m.push(0x20 | (sn & 0x0f));
                m.extend_from_slice(&payload[off..off + take]);
                m.resize(4 + 8, 0); // pad the CAN frame to 8 data bytes
                m
            })
            .unwrap()),
            status_reply(0x0a, 0x00),
        ));
        off += take;
        sn = (sn + 1) & 0x0f;
    }

    // Disconnect (proto 5, on channel drop) + session close on device drop.
    steps.push(Step::exchange(enc(&inner::disconnect(ProtocolId::Can)), status_reply(0x08, 0x00)));
    steps.push(Step::exchange(enc(&inner::session_close()), Vec::new()));
    steps
}

#[test]
fn shim_end_to_end() {
    let opened = Arc::new(AtomicUsize::new(0));
    let opened_in_factory = Arc::clone(&opened);
    _set_device_factory(Box::new(move |_port| {
        let n = opened_in_factory.fetch_add(1, Ordering::SeqCst);
        // Device #0 gets the firmware scenario, device #1 the host-path
        // scenario; later ones (slot exhaustion test) only open and close.
        let steps = if n == 0 {
            main_device_steps()
        } else if n == 1 {
            host_device_steps()
        } else {
            let mut s = handshake_steps();
            s.push(Step::reply_any(Vec::new())); // session close on drop
            s
        };
        Device::open_transport(
            MockTransport::new(steps),
            DeviceConfig {
                port: None,
                keepalive: Some(Duration::from_secs(600)),
                clock: Arc::new(MockClock::default()),
            },
        )
    }));

    // SAFETY: every pointer handed to the exports below is either null (to
    // exercise the null checks) or derives from a live local allocation.
    unsafe {
        // --- invalid ids are rejected before any device exists ---
        assert_eq!(
            rmvci_j2534::PassThruReadVersion(
                1,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                std::ptr::null_mut()
            ),
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
            rmvci_j2534::PassThruReadVersion(
                dev_id,
                fw.as_mut_ptr(),
                dll.as_mut_ptr(),
                api.as_mut_ptr()
            ),
            STATUS_NOERROR
        );
        let cstr = |b: &[i8]| core::ffi::CStr::from_ptr(b.as_ptr()).to_str().unwrap().to_owned();
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
        let mut cfgs =
            [SConfig { parameter: 0x1e, value: 0 }, SConfig { parameter: 0x1f, value: 10 }];
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
        assert_eq!(rmvci_j2534::PassThruReadMsgs(ch_id, &mut *rmsg, &mut n, 1000), STATUS_NOERROR);
        assert_eq!(n, 1);
        assert_eq!(rmsg.data_size, 8);
        assert_eq!(&rmsg.data[..8], &[0x00, 0x00, 0x07, 0xcc, 0x61, 0x43, 0x7b, 0x79]);
        assert_eq!(rmsg.protocol_id, 6);
        // The message carries a host receive timestamp (µs since first open),
        // which by now — after handshake, connect, filter, config and a write —
        // is well past zero.
        assert!(rmsg.timestamp > 0, "expected a non-zero receive timestamp");

        // --- periodic messages: id is written back, stop takes it verbatim ---
        let pmsg = boxed_msg(&[0x00, 0x00, 0x07, 0xc4, 0x3e, 0x00], 0);
        let mut msg_id = 0;
        assert_eq!(
            rmvci_j2534::PassThruStartPeriodicMsg(ch_id, Box::into_raw(pmsg), &mut msg_id, 1000),
            STATUS_NOERROR
        );
        assert_eq!(msg_id, 0x000e_7b02);
        assert_eq!(rmvci_j2534::PassThruStopPeriodicMsg(ch_id, msg_id), STATUS_NOERROR);

        // --- GET_CONFIG writes the value back into the caller's list ---
        let mut gcfgs = [SConfig { parameter: 0x1e, value: 0 }];
        let mut glist = SConfigList { num_of_params: 1, config_ptr: gcfgs.as_mut_ptr() };
        assert_eq!(
            rmvci_j2534::PassThruIoctl(
                ch_id,
                GET_CONFIG as _,
                (&raw mut glist).cast(),
                std::ptr::null_mut()
            ),
            STATUS_NOERROR
        );
        assert_eq!(gcfgs[0].value, 8);

        // --- READ_VBATT writes millivolts to the output ---
        let mut vbatt: c_ulong = 0;
        assert_eq!(
            rmvci_j2534::PassThruIoctl(
                ch_id,
                READ_VBATT as _,
                std::ptr::null_mut(),
                (&raw mut vbatt).cast()
            ),
            STATUS_NOERROR
        );
        assert_eq!(vbatt, 12000);

        // --- SetProgrammingVoltage is a device-scoped passthrough now ---
        assert_eq!(rmvci_j2534::PassThruSetProgrammingVoltage(dev_id, 1, 5000), STATUS_NOERROR);

        // --- stop filter, disconnect, close ---
        assert_eq!(rmvci_j2534::PassThruStopMsgFilter(ch_id, filter_id), STATUS_NOERROR);
        assert_eq!(rmvci_j2534::PassThruDisconnect(ch_id), STATUS_NOERROR);
        assert_eq!(rmvci_j2534::PassThruDisconnect(ch_id), ERR_INVALID_CHANNEL_ID);
        assert_eq!(rmvci_j2534::PassThruClose(dev_id), STATUS_NOERROR);
        assert_eq!(rmvci_j2534::PassThruClose(dev_id), ERR_INVALID_DEVICE_ID);

        // --- host-path ISO15765 (device #1): the vendor RMVCI_HOST_ISOTP flag
        // opens a raw-CAN channel running host-side ISO-TP, so a >255-byte
        // write segments correctly instead of failing FirmwareFfDlLimit ---
        let mut hdev = 0;
        assert_eq!(rmvci_j2534::PassThruOpen(std::ptr::null(), &mut hdev), STATUS_NOERROR);
        let mut hch = 0;
        assert_eq!(
            rmvci_j2534::PassThruConnect(hdev, ISO15765 as _, RMVCI_HOST_ISOTP as _, 500_000, &mut hch),
            STATUS_NOERROR
        );
        // Flow-control filter carries the endpoints: pattern = rx (7cc), flow = tx (7c4).
        let hmask = boxed_msg(&[0xff, 0xff, 0xff, 0xff], 0);
        let hpat = boxed_msg(&[0x00, 0x00, 0x07, 0xcc], 0);
        let hflow = boxed_msg(&[0x00, 0x00, 0x07, 0xc4], 0);
        let mut hfilter = 0;
        assert_eq!(
            rmvci_j2534::PassThruStartMsgFilter(
                hch,
                FLOW_CONTROL_FILTER as u64 as _,
                Box::into_raw(hmask),
                Box::into_raw(hpat),
                Box::into_raw(hflow),
                &mut hfilter,
            ),
            STATUS_NOERROR
        );

        // Small round trip through the host path.
        let hw = boxed_msg(&[0x00, 0x00, 0x07, 0xc4, 0x21, 0x43], 0);
        let mut hn = 1u64 as _;
        assert_eq!(rmvci_j2534::PassThruWriteMsgs(hch, Box::into_raw(hw), &mut hn, 0), STATUS_NOERROR);
        let mut hr: Box<PassthruMsg> = Box::new(std::mem::zeroed());
        let mut hn = 1u64 as _;
        assert_eq!(rmvci_j2534::PassThruReadMsgs(hch, &mut *hr, &mut hn, 1000), STATUS_NOERROR);
        assert_eq!(&hr.data[..8], &[0x00, 0x00, 0x07, 0xcc, 0x61, 0x43, 0x7b, 0x79]);

        // 260-byte write: firmware path would return FirmwareFfDlLimit; the
        // host path segments it and succeeds.
        let mut big = boxed_msg(&[], 0);
        big.data[..4].copy_from_slice(&[0x00, 0x00, 0x07, 0xc4]);
        for (i, b) in big.data[4..264].iter_mut().enumerate() {
            *b = i as u8;
        }
        big.data_size = 264;
        let mut hn = 1u64 as _;
        assert_eq!(rmvci_j2534::PassThruWriteMsgs(hch, Box::into_raw(big), &mut hn, 0), STATUS_NOERROR);

        assert_eq!(rmvci_j2534::PassThruDisconnect(hch), STATUS_NOERROR);
        assert_eq!(rmvci_j2534::PassThruClose(hdev), STATUS_NOERROR);

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

        // --- with no device/channel open, the (now implemented) periodic and
        // voltage exports report the right "nothing there" errors ---
        assert_eq!(rmvci_j2534::PassThruStopPeriodicMsg(0x100, 1), ERR_INVALID_CHANNEL_ID);
        assert_eq!(rmvci_j2534::PassThruSetProgrammingVoltage(1, 15, 0), ERR_INVALID_DEVICE_ID);
    }
}
