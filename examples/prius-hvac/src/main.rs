//! Read the Gen 2 Prius A/C amplifier air-mix servo position through the
//! Mini-VCI cable: KWP2000 `21 43` to ECU 7C4, reply `61 43 <target> <actual>`
//! on 7CC.
//!
//! Usage: prius-hvac [--isotp firmware|host] [PORT]
//!
//! M1 status: argument parsing only — the session layer arrives in M2/M3 and
//! the two ISO-TP paths in M4.

use std::process::ExitCode;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IsoTpPath {
    Firmware,
    Host,
}

struct Args {
    port: String,
    isotp: IsoTpPath,
}

fn parse_args() -> Result<Args, String> {
    let mut port = None;
    let mut isotp = IsoTpPath::Firmware;
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
            "--help" | "-h" => return Err("usage: prius-hvac [--isotp firmware|host] [PORT]".into()),
            p if port.is_none() => port = Some(p.to_string()),
            other => return Err(format!("unexpected argument {other:?}")),
        }
    }
    let port = port
        .or_else(|| std::env::var("MVCI_PORT").ok())
        .unwrap_or_else(|| "/dev/ttyUSB0".to_string());
    Ok(Args { port, isotp })
}

fn main() -> ExitCode {
    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::FAILURE;
        }
    };

    println!("port: {}, isotp path: {:?}", args.port, args.isotp);
    println!("not wired yet: the session layer lands in M2/M3, ISO-TP in M4");
    ExitCode::FAILURE
}
