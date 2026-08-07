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

/// M3: the C driver's K-line regression (`mvci_test <port>`), through the
/// typed API. Needs the cable wired to a K-line ECU (or at least a cable —
/// filter/config/fast-init acceptance is adapter-side).
#[test]
#[ignore = "needs the Mini-VCI cable (set RMVCI_PORT)"]
fn live_kline_regression() {
    use rmvci_core::{Iso14230, KLineConfig, KLineFilter};

    let dev = Device::open(port()).expect("open + handshake");
    println!("DES key: {:02x?}", dev.des_key());

    let mut ch = dev
        .connect::<Iso14230>(KLineConfig::default())
        .expect("connect ISO14230 @ 10400");

    // The vendor tool's three header filters; the typed API keeps one live
    // (matching the single hardware slot), so install them in sequence to
    // prove acceptance, ending on c0/c0.
    for pattern in [0x40u8, 0x80, 0xc0] {
        ch.set_filter(KLineFilter { mask: 0xc0, pattern })
            .unwrap_or_else(|e| panic!("filter c0/{pattern:02x} rejected: {e}"));
    }

    // The 12 SET_CONFIG params the vendor app sends for ISO14230.
    const CFG: [(u32, u32); 12] = [
        (1, 9600), (7, 40), (10, 10), (11, 10), (19, 300), (20, 35),
        (21, 50), (14, 25), (15, 20), (16, 20), (17, 25), (18, 10),
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
