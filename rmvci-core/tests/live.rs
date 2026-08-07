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

/// Measure the adapter's real idle tolerance, with and without a connected
/// channel. The C driver polled every 15 ms on the premise that "the adapter
/// resets if left idle"; this is what decides the actor's default.
///
/// Run it as two cases because a bare DES session and a connected CAN
/// channel are different states — a watchdog could plausibly apply to one and
/// not the other.
#[test]
#[ignore = "needs the Mini-VCI cable (set RMVCI_PORT); takes ~20 minutes"]
fn live_keepalive_threshold() {
    use rmvci_core::{CanConfig, DeviceConfig, Iso15765};
    use std::sync::Arc;

    // Keepalive far beyond every gap so the actor stays silent during them.
    let open = || {
        Device::open_with(DeviceConfig {
            port: Some(port()),
            keepalive: Duration::from_secs(7200),
            clock: Arc::new(rmvci_core::transport::RealClock),
        })
        .expect("open")
    };

    let gaps = [1_000u64, 5_000, 20_000, 60_000, 120_000, 300_000];

    for connected in [false, true] {
        println!(
            "\n--- idle tolerance, channel {} ---",
            if connected { "CONNECTED (ISO15765)" } else { "not connected" }
        );
        let mut survived_all = true;
        for gap_ms in gaps {
            let dev = open();
            let chan =
                connected.then(|| dev.connect::<Iso15765>(CanConfig::default()).expect("connect"));
            dev.firmware_version().expect("baseline query");

            std::thread::sleep(Duration::from_millis(gap_ms));
            let outcome = dev.firmware_version();
            drop(chan);
            dev.close(); // release the port before the next iteration reopens it

            match outcome {
                Ok(_) => println!("  idle {gap_ms:>7} ms: survived"),
                Err(e) => {
                    println!("  idle {gap_ms:>7} ms: DEAD ({e})");
                    println!("  --> tolerance is between this gap and the previous one");
                    survived_all = false;
                    break;
                }
            }
        }
        if survived_all {
            println!("  survived every gap up to {} s", gaps.last().unwrap() / 1000);
        }
    }
}

/// M2 smoke: open + handshake, identity, and 60 s of keepalive survival.
#[test]
#[ignore = "needs the Mini-VCI cable (set RMVCI_PORT)"]
fn smoke_open_version_keepalive() {
    let dev = Device::open(port()).expect("open + handshake");
    println!("DES key: {:02x?}", dev.des_key());

    let version = dev.firmware_version().expect("firmware version");
    println!("firmware: {version:?}");
    assert_eq!(version, "J2534 MINIV1.03");

    // The adapter resets if left idle; the actor's keepalive must hold the
    // session. If it doesn't, this second query fails.
    println!("holding idle for 60 s...");
    std::thread::sleep(Duration::from_secs(60));
    let again = dev.firmware_version().expect("firmware version after idle hold");
    assert_eq!(again, version);
    println!("keepalive held");
}
