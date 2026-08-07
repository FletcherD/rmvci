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
