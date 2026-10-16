//! Read the Gen 2 Prius (NHW20) A/C amplifier air-mix damper position through
//! the Mini-VCI cable.
//!
//! The A/C amplifier is a K-line (ISO 14230) ECU at address `0x98`. It answers
//! KWP2000 service `21` (readDataByLocalIdentifier) one LID at a time:
//! `21 0D` is the actual driver-side damper position, `21 13` the target.
//! Each reply is `61 <LID> <b0>`, scaled `b0 * 0.5 - 14` %.
//!
//! ```sh
//! prius-hvac                                   # $MVCI_PORT or /dev/ttyUSB0
//! prius-hvac /dev/serial/by-id/usb-XHorse_M-VCI_...-if00-port0
//! prius-hvac --watch                           # poll forever
//! RUST_LOG=rmvci_core=trace prius-hvac         # hex dump of every exchange
//! ```

use std::process::ExitCode;
use std::time::Duration;

use rmvci_core::{Device, DeviceConfig, Error, KLineEcu, UdsTransport, resolve_port};

const AC_AMP: u8 = 0x98;
const LID_DAMPER_ACTUAL: u8 = 0x0d;
const LID_DAMPER_TARGET: u8 = 0x13;

struct Args {
    port: Option<String>,
    watch: bool,
}

const USAGE: &str = "usage: prius-hvac [--watch] [PORT]";

fn parse_args() -> Result<Args, String> {
    let mut port = None;
    let mut watch = false;
    for a in std::env::args().skip(1) {
        match a.as_str() {
            "--watch" => watch = true,
            "--help" | "-h" => return Err(USAGE.into()),
            p if port.is_none() => port = Some(p.to_string()),
            other => return Err(format!("unexpected argument {other:?}\n{USAGE}")),
        }
    }
    Ok(Args { port, watch })
}

/// Decode `61 <lid> <b0>` to percent.
fn decode_damper(lid: u8, reply: &[u8]) -> Result<f32, String> {
    match reply {
        [0x61, l, b0, ..] if *l == lid => Ok(*b0 as f32 * 0.5 - 14.0),
        [0x7f, 0x21, nrc, ..] => Err(format!("ECU refused 21 {lid:02X}: NRC {nrc:#04x}")),
        other => Err(format!("unexpected reply to 21 {lid:02X}: {other:02x?}")),
    }
}

fn read_damper(ecu: &mut KLineEcu, lid: u8) -> Result<Result<f32, String>, Error> {
    let reply = ecu.request(&[0x21, lid], Duration::from_secs(2))?;
    Ok(decode_damper(lid, &reply))
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
    println!("opening {port}");
    let dev = Device::open_with(DeviceConfig { port: Some(port), ..DeviceConfig::default() })?;
    println!("firmware: {}", dev.firmware_version()?);

    let mut ecu = KLineEcu::connect(&dev, AC_AMP)?;
    println!("key bytes: {:02x?}", ecu.fast_init()?);

    loop {
        let actual = read_damper(&mut ecu, LID_DAMPER_ACTUAL)?;
        let target = read_damper(&mut ecu, LID_DAMPER_TARGET)?;
        match (actual, target) {
            (Ok(a), Ok(t)) => println!("air-mix damper: actual {a:>6.1} %, target {t:>6.1} %"),
            (a, t) => println!("actual: {a:?}, target: {t:?}"),
        }
        if !args.watch {
            break;
        }
        std::thread::sleep(Duration::from_millis(500));
    }
    drop(ecu); // the channel holds a device handle; close() waits for all of them
    dev.close();
    Ok(())
}

fn main() -> ExitCode {
    // `RUST_LOG=rmvci_core=trace` dumps every inner command and reply.
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
    fn decodes_damper_position() {
        assert_eq!(decode_damper(0x0d, &[0x61, 0x0d, 0x09]).unwrap(), -9.5);
    }

    #[test]
    fn rejects_mismatched_lid() {
        assert!(decode_damper(0x0d, &[0x61, 0x13, 0x08]).is_err());
    }

    #[test]
    fn reports_negative_response() {
        let e = decode_damper(0x0d, &[0x7f, 0x21, 0x12]).unwrap_err();
        assert!(e.contains("NRC 0x12"), "{e}");
    }
}
