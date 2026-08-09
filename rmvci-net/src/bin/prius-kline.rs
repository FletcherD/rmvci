//! First-contact K-line client for the real NHW20 Prius A/C amplifier, over a
//! remote `rmvci-bridge` (cable at the car).
//!
//! The Gen-2 A/C amp is **K-line (ISO14230), address 0x98** — not the CAN 7C4
//! path `rmvci-remote --hvac` uses. All the protocol lives in the library now:
//! [`KLineEcu`] does the vendor bring-up + FAST_INIT + ISO14230 framing, and
//! [`Kwp2000`] does the `21 <LID>` reads and response handling. This binary is
//! just the Prius A/C data-list scaling on top.
//!
//! ```sh
//! prius-kline 192.168.1.207:6979
//! ```
//!
//! Frames/scales come from the validated Techstream-DB extraction
//! (`re/techstream/datalist_by_gen/datalist_P3_KWP.txt`).

use std::process::ExitCode;
use std::thread::sleep;
use std::time::Duration;

use rmvci_core::{Device, DeviceConfig, KLineEcu};
use rmvci_diag::Kwp2000;
use rmvci_net::TcpTransport;

const ECU: u8 = 0x98; // A/C amplifier K-line address

/// One data-list entry: KWP LID + how to render byte 0 of the reply.
struct Pid {
    lid: u8,
    name: &'static str,
    /// `Some((mul, sub))` → phys = raw*mul - sub; `None` → raw with an optional mask.
    scale: Option<(f64, f64)>,
    mask: Option<u8>,
    unit: &'static str,
}

const CLUSTER: &[Pid] = &[
    Pid { lid: 0x0d, name: "Air Mix Damper Position (D)  [actual]   ", scale: Some((0.5, 14.0)), mask: None, unit: "%" },
    Pid { lid: 0x0e, name: "Air Mix Damper Position (P)  [actual]   ", scale: Some((0.5, 14.0)), mask: None, unit: "%" },
    Pid { lid: 0x13, name: "Air Mix Damper Target   (D)  [commanded]", scale: Some((0.5, 14.0)), mask: None, unit: "%" },
    Pid { lid: 0x14, name: "Air Mix Damper Target   (P)  [commanded]", scale: Some((0.5, 14.0)), mask: None, unit: "%" },
    Pid { lid: 0x41, name: "Air Mix Servo Targ Pulse (D)            ", scale: None, mask: None, unit: "pulse" },
    Pid { lid: 0x1c, name: "Blower Motor Speed Level                ", scale: None, mask: Some(0x1f), unit: "level" },
    Pid { lid: 0x02, name: "Ambient Temp Sensor                     ", scale: Some((0.35, 23.3)), mask: None, unit: "\u{b0}C" },
];

fn run(addr: &str) -> Result<(), Box<dyn std::error::Error>> {
    println!("connecting to bridge {addr} …");
    let io = TcpTransport::connect(addr)?;
    let dev = Device::open_transport(io, DeviceConfig::default())?;
    println!("cable firmware: {}", dev.firmware_version()?);

    println!("opening ISO14230 K-line channel + vendor bring-up …");
    let mut ecu = KLineEcu::connect(&dev, ECU)?;
    println!("FAST_INIT → StartCommunication to A/C amp {ECU:#04x} …");
    let keybytes = ecu.fast_init()?;
    println!("  ECU key bytes: {keybytes:02x?}   ← first contact with the car");

    let mut kwp = Kwp2000::new(ecu);
    println!("\n--- air-mix / blend-door servo cluster ---");
    for pid in CLUSTER {
        match kwp.read_data_by_local_id(pid.lid) {
            Ok(data) => {
                let raw = data.first().copied().unwrap_or(0);
                let shown = raw & pid.mask.unwrap_or(0xff);
                match pid.scale {
                    Some((m, s)) => println!(
                        "  21 {:02x}  {}  raw {:>3} (0x{:02x})  = {:>7.1} {}",
                        pid.lid, pid.name, shown, raw, shown as f64 * m - s, pid.unit
                    ),
                    None => println!(
                        "  21 {:02x}  {}  raw {:>3} (0x{:02x})  {}",
                        pid.lid, pid.name, shown, raw, pid.unit
                    ),
                }
            }
            Err(e) => println!("  21 {:02x}  {}  -> {}", pid.lid, pid.name, e),
        }
        sleep(Duration::from_millis(60)); // ease the ECU P3 inter-message timing
    }
    Ok(())
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn")),
        )
        .with_target(false)
        .init();

    let addr = std::env::args().nth(1).unwrap_or_else(|| "192.168.1.207:6979".into());
    match run(&addr) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
