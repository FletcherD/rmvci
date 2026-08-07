//! Read the Gen 2 Prius (NHW20) A/C amplifier air-mix servo position through
//! the Mini-VCI cable.
//!
//! The A/C amplifier answers KWP2000 service `21` (readDataByLocalIdentifier)
//! on CAN id `7C4`, replying on `7CC`. For local identifier `43` the reply is
//! `61 43 <commanded> <actual>` — two 0–255 blend-door servo pulse counts.
//!
//! ```sh
//! prius-hvac                                   # $MVCI_PORT or /dev/ttyUSB0
//! prius-hvac /dev/serial/by-id/usb-XHorse_M-VCI_...-if00-port0
//! prius-hvac --isotp host --watch              # host-side ISO-TP, poll forever
//! RUST_LOG=rmvci_core=trace prius-hvac         # hex dump of every exchange
//! ```
//!
//! Note the `7C4` addressing itself is *unverified on the Prius* — it comes
//! from the OBDb Climate block for newer Camry/RX/4Runner. A `7F 21 <nrc>`
//! reply or silence means the amplifier is not on this bus or uses a
//! different identifier on this model.

use std::process::ExitCode;
use std::time::Duration;

use rmvci_core::{
    CanId, Device, DeviceConfig, Error, FirmwareIsoTp, IsoTp, IsoTpConfig, UdsTransport,
    resolve_port,
};

/// A/C amplifier request / response identifiers.
const ECU_TX: u16 = 0x7c4;
const ECU_RX: u16 = 0x7cc;
/// Air outlet servo (blend / air-mix) local identifier.
const LID_AIR_OUTLET_SERVO: u8 = 0x43;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IsoTpPath {
    Firmware,
    Host,
}

struct Args {
    port: Option<String>,
    isotp: IsoTpPath,
    watch: bool,
}

const USAGE: &str = "usage: prius-hvac [--isotp firmware|host] [--watch] [PORT]";

fn parse_args() -> Result<Args, String> {
    let mut port = None;
    let mut isotp = IsoTpPath::Firmware;
    let mut watch = false;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--isotp" => {
                isotp = match it.next().as_deref() {
                    Some("firmware") => IsoTpPath::Firmware,
                    Some("host") => IsoTpPath::Host,
                    other => return Err(format!("--isotp expects firmware|host, got {other:?}")),
                }
            }
            "--watch" => watch = true,
            "--help" | "-h" => return Err(USAGE.into()),
            p if port.is_none() => port = Some(p.to_string()),
            other => return Err(format!("unexpected argument {other:?}\n{USAGE}")),
        }
    }
    Ok(Args { port, isotp, watch })
}

/// Decode `61 43 <commanded> <actual>`.
fn decode_servo(reply: &[u8]) -> Result<(u8, u8), String> {
    match reply {
        [0x61, LID_AIR_OUTLET_SERVO, commanded, actual, ..] => Ok((*commanded, *actual)),
        [0x7f, 0x21, nrc, ..] => Err(format!(
            "ECU refused service 21: NRC {nrc:#04x} — the amplifier is on this bus but does \
             not serve local identifier {LID_AIR_OUTLET_SERVO:#04x}"
        )),
        other => Err(format!("unexpected reply {other:02x?}")),
    }
}

fn run() -> Result<(), Error> {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };

    let port = resolve_port(args.port.as_deref());
    println!("opening {port} ({:?} ISO-TP)", args.isotp);
    let dev = Device::open_with(DeviceConfig { port: Some(port), ..DeviceConfig::default() })?;
    println!("firmware: {}", dev.firmware_version()?);

    let tx = CanId::Std(ECU_TX);
    let rx = CanId::Std(ECU_RX);
    let mut transport: Box<dyn UdsTransport> = match args.isotp {
        IsoTpPath::Firmware => Box::new(FirmwareIsoTp::new(&dev, tx, rx)?),
        IsoTpPath::Host => Box::new(IsoTp::new(&dev, IsoTpConfig::new(tx, rx))?),
    };

    let request = [0x21, LID_AIR_OUTLET_SERVO];
    loop {
        match transport.request(&request, Duration::from_secs(2)) {
            Ok(reply) => {
                println!("raw: {reply:02x?}");
                match decode_servo(&reply) {
                    Ok((commanded, actual)) => println!(
                        "air-mix servo: commanded {commanded:>3}, actual {actual:>3} \
                         (delta {:+})",
                        actual as i16 - commanded as i16
                    ),
                    Err(e) => println!("{e}"),
                }
            }
            Err(Error::NoFilterInstalled { .. }) | Err(Error::Timeout(_)) => {
                println!(
                    "no reply from {ECU_TX:#05x} — the A/C amplifier may not be on the \
                     pin 6/14 diagnostic bus on this model, or the ignition is off"
                );
            }
            Err(e) => return Err(e),
        }
        if !args.watch {
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(500));
    }
}

fn main() -> ExitCode {
    // `RUST_LOG=rmvci_core=trace` dumps every inner command and reply — the
    // equivalent of the C driver's MVCI_DEBUG=1.
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn")),
        )
        .with_target(false)
        .init();

    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_the_bench_reply() {
        assert_eq!(decode_servo(&[0x61, 0x43, 0x7b, 0x79]).unwrap(), (0x7b, 0x79));
    }

    #[test]
    fn reports_negative_response() {
        let e = decode_servo(&[0x7f, 0x21, 0x11]).unwrap_err();
        assert!(e.contains("NRC 0x11"), "{e}");
    }
}
