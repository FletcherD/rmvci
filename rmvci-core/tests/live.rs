//! Hardware-gated live tests. All `#[ignore]`; run explicitly with the cable
//! plugged in:
//!
//! ```sh
//! RMVCI_PORT=/dev/serial/by-id/usb-FTDI_...-if00-port0 \
//!     cargo test --test live -- --ignored --nocapture
//! ```
//!
//! Use the /dev/serial/by-id path — bare ttyUSB numbers swap with plug order.

#![cfg(feature = "serial")]

use std::time::Duration;

use rmvci_core::Device;

fn port() -> String {
    std::env::var("RMVCI_PORT")
        .expect("set RMVCI_PORT to the Mini-VCI serial device to run live tests")
}

/// Open, retrying while the port is still held by a previous device's
/// asynchronous teardown (dropping a Device returns before its actor thread
/// finishes the wire teardown and releases the port).
fn open_retrying(port: &str) -> Device {
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    loop {
        match Device::open(port) {
            Ok(d) => return d,
            Err(e) if std::time::Instant::now() < deadline => {
                eprintln!("open pending ({e}); retrying");
                std::thread::sleep(Duration::from_millis(200));
            }
            Err(e) => panic!("open {port}: {e}"),
        }
    }
}

/// M3: the C driver's K-line regression (`mvci_test <port>`), through the
/// typed API. Needs the cable wired to a K-line ECU (or at least a cable —
/// filter/config/fast-init acceptance is adapter-side).
#[test]
#[ignore = "needs the Mini-VCI cable (set RMVCI_PORT)"]
fn live_kline_regression() {
    use rmvci_core::{Iso14230, KLineConfig, KLineFilter};

    let dev = Device::open(port()).expect("open + handshake");
    println!("DES key: {:02x?}", dev.des_key());

    let mut ch = dev.connect::<Iso14230>(KLineConfig::default()).expect("connect ISO14230 @ 10400");

    // The vendor tool's three header filters; the typed API keeps one live
    // (matching the single hardware slot), so install them in sequence to
    // prove acceptance, ending on c0/c0.
    for pattern in [0x40u8, 0x80, 0xc0] {
        ch.set_filter(KLineFilter { mask: 0xc0, pattern })
            .unwrap_or_else(|e| panic!("filter c0/{pattern:02x} rejected: {e}"));
    }

    // The 12 SET_CONFIG params the vendor app sends for ISO14230.
    const CFG: [(u32, u32); 12] = [
        (1, 9600),
        (7, 40),
        (10, 10),
        (11, 10),
        (19, 300),
        (20, 35),
        (21, 50),
        (14, 25),
        (15, 20),
        (16, 20),
        (17, 25),
        (18, 10),
    ];
    for (param, value) in CFG {
        ch.set_config(param, value).unwrap_or_else(|e| panic!("SET_CONFIG {param}: {e}"));
    }
    ch.clear_periodic().expect("clear periodic");

    // FAST_INIT + a couple of OBD PIDs from the SMT ECU (0x19) — only
    // meaningful with the car attached; report, don't fail, on silence.
    match ch.fast_init(&[0x81, 0x19, 0xf0, 0x81]) {
        Ok(key) => {
            println!("FAST_INIT -> {key:02x?}");
            for (pid, name) in [(0x05u8, "ECT"), (0x0c, "RPM")] {
                ch.clear_periodic().ok();
                ch.write(&[0x82, 0x19, 0xf0, 0x01, pid]).expect("write");
                match ch.read(Duration::from_secs(2)) {
                    Ok(m) => println!("OBD {name}: {:02x?}", m.data),
                    Err(e) => println!("OBD {name}: no reply ({e})"),
                }
            }
        }
        Err(e) => println!("FAST_INIT: no ECU answered ({e}) — cable-only run"),
    }
}

/// M4: both ISO-TP paths against the bench fake ECU
/// (`re/bench/isotp_responder.py` on the CH340 USB-CAN analyzer):
/// `21 43` -> single-frame `61 43 xx xx`, `21 44` -> 40-byte multi-frame.
///
/// ```sh
/// python3 re/bench/isotp_responder.py -v &
/// RMVCI_PORT=... cargo test --test live live_can -- --ignored --nocapture
/// ```
#[test]
#[ignore = "needs the cable + bench responder (set RMVCI_PORT)"]
fn live_can_firmware_vs_host() {
    use rmvci_core::{CanId, FirmwareIsoTp, IsoTp, IsoTpConfig, UdsTransport};

    let tx = CanId::Std(0x7c4);
    let rx = CanId::Std(0x7cc);
    let timeout = Duration::from_secs(3);

    let run = |tp: &mut dyn UdsTransport, label: &str| -> (Vec<u8>, Vec<u8>) {
        let r1 = tp.request(&[0x21, 0x43], timeout).expect("21 43");
        println!("{label} 21 43 -> {r1:02x?}");
        assert_eq!(&r1[..2], &[0x61, 0x43], "not a positive 21 43 response");
        assert_eq!(r1.len(), 4, "servo reply must be 61 43 <target> <actual>");

        let r2 = tp.request(&[0x21, 0x44], timeout).expect("21 44");
        println!("{label} 21 44 -> {} bytes: {r2:02x?}", r2.len());
        assert_eq!(&r2[..2], &[0x61, 0x44]);
        assert_eq!(r2.len(), 40, "responder sends a 40-byte multi-frame reply");
        (r1, r2)
    };

    // Firmware path (protocol 6) — one channel at a time: the adapter keeps
    // one hardware filter per ID width, so the paths run sequentially on
    // separate devices.
    let (fw1, fw2) = {
        let dev = open_retrying(&port());
        let mut tp = FirmwareIsoTp::new(&dev, tx, rx).expect("firmware channel");
        let r = run(&mut tp, "firmware");
        drop(tp);
        dev.close(); // wait for the port before the host path reopens it
        r
    };

    // Host path (raw CAN, host machines).
    let (h1, h2) = {
        let dev = open_retrying(&port());
        let mut tp = IsoTp::new(&dev, IsoTpConfig::new(tx, rx)).expect("host channel");
        run(&mut tp, "host    ")
    };

    // M4 exit criterion: byte-identical results on both paths.
    assert_eq!(fw1, h1, "single-frame replies differ between paths");
    assert_eq!(fw2, h2, "multi-frame replies differ between paths");
    println!("both paths byte-identical");
}

/// Characterise what actually decays when the link sits idle, layer by
/// layer. This is what decides whether the actor needs a keepalive at all.
///
/// Asking only "is the session alive?" (a device-level READ_VERSION) is not
/// enough: the adapter could drop a channel or forget its acceptance filter
/// while still answering device commands cheerfully. So after each idle gap
/// the probe walks up the stack and reports the *first* layer that fails:
///
/// 1. session — READ_VERSION (CMD 0x03), device-level.
/// 2. channel — a READ_MSG poll on the connected channel; the firmware answers
///    ERR_INVALID_CHANNEL_ID if the channel is gone.
/// 3. exchange — a real 21 43 to the bench ECU, which additionally needs the
///    acceptance filter and the CAN bus timing to be intact.
///
/// If stage 3 fails it then tries to recover, which says what a driver would
/// have to *do* about it: reinstall the filter, or reconnect the channel.
///
/// Needs `re/bench/isotp_responder.py` running (stage 3 talks to it).
#[test]
#[ignore = "needs the cable + bench responder (set RMVCI_PORT); takes ~30 minutes"]
fn live_idle_decay_characterisation() {
    use rmvci_core::{CanConfig, CanId, DeviceConfig, FlowControlFilter, Iso15765};
    use std::sync::Arc;

    let tx = CanId::Std(0x7c4);
    let rx = CanId::Std(0x7cc);
    let probe_timeout = Duration::from_secs(2);

    // Keepalive far beyond every gap so the actor stays silent during them —
    // this test measures the adapter, not our poll.
    let open = || {
        Device::open_with(DeviceConfig {
            port: Some(port()),
            keepalive: Some(Duration::from_secs(7200)),
            clock: Arc::new(rmvci_core::transport::RealClock),
        })
        .expect("open")
    };

    let gaps_s = [5u64, 30, 60, 300, 900];

    println!("\n=== idle decay: which layer dies first, and when ===");
    println!("(each row: session / channel / exchange after the gap)\n");

    for gap_s in gaps_s {
        let dev = open();
        let mut ch = dev.connect::<Iso15765>(CanConfig::default()).expect("connect");
        ch.set_filter(FlowControlFilter::exact(rx, tx)).expect("filter");

        // Baseline: prove the whole stack works *before* the gap, so a
        // failure afterwards is attributable to the idling and not to setup.
        ch.send(tx, &[0x21, 0x43]).expect("baseline send");
        let baseline = ch.read(probe_timeout).expect("baseline exchange must work");
        assert_eq!(&baseline.data[4..6], &[0x61, 0x43], "bench responder not answering");

        std::thread::sleep(Duration::from_secs(gap_s));

        // --- staged probe, cheapest layer first ---
        let session = dev.firmware_version();
        let channel = ch.poll(Duration::from_millis(500));
        let exchange = ch
            .send(tx, &[0x21, 0x43])
            .and_then(|()| ch.read(probe_timeout))
            .map(|m| m.data[4..].to_vec());

        fn verdict<T>(r: &Result<T, rmvci_core::Error>) -> &'static str {
            if r.is_ok() { "ok  " } else { "DEAD" }
        }
        println!(
            "idle {gap_s:>4} s | session {} | channel {} | exchange {}",
            verdict(&session),
            verdict(&channel),
            verdict(&exchange)
        );
        if let Err(e) = &session {
            println!("           session error: {e}");
        }
        if let Err(e) = &channel {
            println!("           channel error: {e}");
        }

        // --- if the exchange died, find out what fixes it ---
        match &exchange {
            Ok(data) => {
                assert_eq!(&data[..2], &[0x61, 0x43], "exchange returned the wrong payload");
            }
            Err(e) => {
                println!("           exchange error: {e}");

                let refiltered = ch
                    .set_filter(FlowControlFilter::exact(rx, tx))
                    .and_then(|()| ch.send(tx, &[0x21, 0x43]))
                    .and_then(|()| ch.read(probe_timeout));
                if refiltered.is_ok() {
                    println!("           --> RECOVERED by reinstalling the filter");
                } else {
                    drop(ch);
                    let mut fresh =
                        dev.connect::<Iso15765>(CanConfig::default()).expect("reconnect");
                    fresh.set_filter(FlowControlFilter::exact(rx, tx)).expect("filter");
                    let reconnected =
                        fresh.send(tx, &[0x21, 0x43]).and_then(|()| fresh.read(probe_timeout));
                    println!(
                        "           --> filter reinstall did NOT help; reconnect {}",
                        if reconnected.is_ok() { "RECOVERED" } else { "also failed" }
                    );
                    dev.close();
                    continue;
                }
            }
        }

        drop(ch);
        dev.close(); // release the port before the next iteration reopens it
    }
    println!("\n(gaps tested up to {} s)", gaps_s.last().unwrap());
}

/// Send a multi-frame message so `re/bench/isotp_fc_probe.py` can measure
/// whether this path honours the flow control it is given.
///
/// This is the experiment that settles the last claim in `re/FINDINGS.md`
/// resting on static analysis alone (§10.3: the firmware's ISO-TP transmit
/// ignores BS and STmin), and at the same time proves the host path fixes it.
///
/// ```sh
/// python3 re/bench/isotp_fc_probe.py --bs 2 --stmin 50 &
/// RMVCI_PORT=... cargo test --test live fc_firmware -- --ignored --nocapture
/// RMVCI_PORT=... cargo test --test live fc_host     -- --ignored --nocapture
/// ```
///
/// The probe prints the verdict; this side only has to transmit.
#[cfg(feature = "serial")]
fn send_multiframe_for_probe(host_path: bool) {
    use rmvci_core::{CanId, FirmwareIsoTp, IsoTp, IsoTpConfig};

    let tx = CanId::Std(0x7c4);
    let rx = CanId::Std(0x7cc);
    // 60 bytes = First Frame + 8 consecutive frames, enough for a BS of 2 to
    // force several blocks.
    let payload: Vec<u8> = (0..60u8).collect();

    let dev = open_retrying(&port());
    if host_path {
        let mut tp = IsoTp::new(&dev, IsoTpConfig::new(tx, rx)).expect("host channel");
        match tp.send(&payload) {
            Ok(()) => println!("host path: sent {} bytes", payload.len()),
            Err(e) => println!("host path: send ended with {e}"),
        }
    } else {
        let mut tp = FirmwareIsoTp::new(&dev, tx, rx).expect("firmware channel");
        match tp.send(&payload) {
            Ok(()) => println!("firmware path: sent {} bytes", payload.len()),
            Err(e) => println!("firmware path: send ended with {e}"),
        }
    }
    // Let the probe finish collecting before the port closes.
    std::thread::sleep(Duration::from_secs(3));
    dev.close();
}

#[test]
#[ignore = "needs the cable + isotp_fc_probe.py (set RMVCI_PORT)"]
fn live_fc_firmware_path() {
    send_multiframe_for_probe(false);
}

#[test]
#[ignore = "needs the cable + isotp_fc_probe.py (set RMVCI_PORT)"]
fn live_fc_host_path() {
    send_multiframe_for_probe(true);
}

/// Smoke test: open + handshake + identity, then hold the link idle for 60 s
/// with the shipped defaults — which send **no keepalive at all** — and query
/// again. A regression that reintroduced an idle-reset assumption, or a
/// device that really did need poking, would fail the second query.
#[test]
#[ignore = "needs the Mini-VCI cable (set RMVCI_PORT)"]
fn smoke_open_version_survives_idle() {
    let dev = Device::open(port()).expect("open + handshake");
    println!("DES key: {:02x?}", dev.des_key());

    let version = dev.firmware_version().expect("firmware version");
    println!("firmware: {version:?}");
    assert_eq!(version, "J2534 MINIV1.03");

    println!("holding idle for 60 s with no keepalive traffic...");
    std::thread::sleep(Duration::from_secs(60));
    let again = dev.firmware_version().expect("firmware version after idle hold");
    assert_eq!(again, version);
    println!("survived 60 s of silence");
}
