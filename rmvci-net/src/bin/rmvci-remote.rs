//! PC-side demo client: drive the Mini-VCI over a remote `rmvci-bridge`.
//!
//! Proves the whole stack runs across the network — the DES handshake, the
//! firmware-version query, and (with `--hvac`) an ISO-TP read — with the cable
//! plugged into the remote host, not this one.
//!
//! ```sh
//! rmvci-remote 192.168.1.50:6979              # open + print firmware version
//! rmvci-remote 192.168.1.50:6979 --hvac       # then read 7C4 / KWP 21 43
//! rmvci-remote 192.168.1.50:6979 --hvac --watch --isotp host
//! RUST_LOG=rmvci_core=trace rmvci-remote 192.168.1.50:6979
//! ```
//!
//! `--hvac` targets CAN `7C4` (the bench ECU / newer models). The real NHW20
//! A/C amp is on K-line — see the parent project's CLAUDE.md — so this flag is
//! for bench validation of the link, not the Gen-2 car servo.

use std::process::ExitCode;
use std::time::Duration;

use rmvci_core::{CanId, Device, DeviceConfig, Error, IsoTp, IsoTpConfig, IsoTpPath};
use rmvci_net::TcpTransport;

const ECU_TX: u16 = 0x7c4;
const ECU_RX: u16 = 0x7cc;
const LID_AIR_OUTLET_SERVO: u8 = 0x43;

struct Args {
    addr: String,
    hvac: bool,
    isotp: IsoTpPath,
    watch: bool,
}

const USAGE: &str = "usage: rmvci-remote ADDR:PORT [--hvac] [--isotp firmware|host] [--watch]";

fn parse_args() -> Result<Args, String> {
    let mut addr = None;
    let mut hvac = false;
    let mut isotp = IsoTpPath::Firmware;
    let mut watch = false;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--hvac" => hvac = true,
            "--watch" => watch = true,
            "--isotp" => {
                isotp = match it.next().as_deref() {
                    Some("firmware") => IsoTpPath::Firmware,
                    Some("host") => IsoTpPath::Host,
                    other => return Err(format!("--isotp expects firmware|host, got {other:?}")),
                }
            }
            "--help" | "-h" => return Err(USAGE.into()),
            p if addr.is_none() => addr = Some(p.to_string()),
            other => return Err(format!("unexpected argument {other:?}\n{USAGE}")),
        }
    }
    Ok(Args { addr: addr.ok_or(USAGE)?, hvac, isotp, watch })
}

fn decode_servo(reply: &[u8]) -> Result<(u8, u8), String> {
    match reply {
        [0x61, LID_AIR_OUTLET_SERVO, commanded, actual, ..] => Ok((*commanded, *actual)),
        [0x7f, 0x21, nrc, ..] => Err(format!("ECU refused service 21: NRC {nrc:#04x}")),
        other => Err(format!("unexpected reply {other:02x?}")),
    }
}

fn read_hvac(dev: &Device, path: IsoTpPath, watch: bool) -> Result<(), Error> {
    let tx = CanId::Std(ECU_TX);
    let rx = CanId::Std(ECU_RX);
    let mut transport = IsoTp::new(dev, IsoTpConfig::new(tx, rx), path)?;
    let request = [0x21, LID_AIR_OUTLET_SERVO];
    loop {
        match transport.request(&request, Duration::from_secs(2)) {
            Ok(reply) => {
                println!("raw: {reply:02x?}");
                match decode_servo(&reply) {
                    Ok((c, a)) => println!(
                        "air-mix servo: commanded {c:>3}, actual {a:>3} (delta {:+})",
                        a as i16 - c as i16
                    ),
                    Err(e) => println!("{e}"),
                }
            }
            Err(Error::NoFilterInstalled { .. }) | Err(Error::Timeout(_)) => {
                println!("no reply from {ECU_TX:#05x} (wrong bus for the NHW20, or ignition off)");
            }
            Err(e) => return Err(e),
        }
        if !watch {
            return Ok(());
        }
        std::thread::sleep(Duration::from_millis(500));
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

    println!("connecting to bridge {}", args.addr);
    let io = TcpTransport::connect(&args.addr)?;
    let dev = Device::open_transport(io, DeviceConfig::default())?;
    println!("firmware: {}", dev.firmware_version()?);

    if args.hvac {
        read_hvac(&dev, args.isotp, args.watch)?;
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

    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
