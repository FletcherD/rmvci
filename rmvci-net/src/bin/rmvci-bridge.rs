//! Phone-side bridge. Owns the Mini-VCI locally and serves its byte transport
//! to a remote `rmvci` over TCP.
//!
//! Run it on the machine the cable is plugged into — typically a rooted
//! Android phone at the car (via Termux or `adb shell`, as root so it can
//! claim the FT232R):
//!
//! ```sh
//! rmvci-bridge                       # USB (nusb), first FT232R, listen on 0.0.0.0:6979
//! rmvci-bridge --usb A69QL5OE        # pick one FT232R by USB serial string
//! rmvci-bridge --serial /dev/ttyUSB0 # ftdi_sio tty instead of raw USB
//! rmvci-bridge --listen 0.0.0.0:7000 # different bind address/port
//! ```
//!
//! The `--usb` backend claims the device away from `ftdi_sio` and can set the
//! FTDI latency timer without a udev rule, so it is the default; `--serial` is
//! the fallback where raw USB access is unavailable.

use std::net::TcpListener;
use std::process::ExitCode;

use rmvci_core::transport::{SerialTransport, UsbTransport};
use rmvci_net::{serve, DEFAULT_PORT};

enum Backend {
    Usb(Option<String>),
    Serial(String),
}

struct Args {
    backend: Backend,
    listen: String,
}

const USAGE: &str =
    "usage: rmvci-bridge [--usb [SERIAL] | --serial PATH] [--listen ADDR:PORT]";

fn parse_args() -> Result<Args, String> {
    let mut backend = Backend::Usb(None);
    let mut listen = format!("0.0.0.0:{DEFAULT_PORT}");
    let mut it = std::env::args().skip(1).peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--usb" => {
                // Optional serial argument; anything starting with '-' is the
                // next flag, not the serial string.
                let take = it.peek().is_some_and(|s| !s.starts_with('-'));
                backend = Backend::Usb(take.then(|| it.next().unwrap()));
            }
            "--serial" => {
                let path = it.next().ok_or("--serial needs a device path")?;
                backend = Backend::Serial(path);
            }
            "--listen" => listen = it.next().ok_or("--listen needs ADDR:PORT")?,
            "--help" | "-h" => return Err(USAGE.into()),
            other => return Err(format!("unexpected argument {other:?}\n{USAGE}")),
        }
    }
    Ok(Args { backend, listen })
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    let listener = TcpListener::bind(&args.listen)
        .map_err(|e| format!("bind {}: {e}", args.listen))?;
    tracing::info!(listen = %args.listen, "rmvci-bridge listening");

    // Open the cable once; `serve` reuses it across sequential clients.
    match args.backend {
        Backend::Usb(serial) => {
            tracing::info!(serial = ?serial, "opening cable over USB (nusb)");
            let io = UsbTransport::open(serial.as_deref())
                .map_err(|e| format!("open USB cable: {e}"))?;
            serve(listener, io).map_err(|e| e.to_string())
        }
        Backend::Serial(path) => {
            tracing::info!(%path, "opening cable over ftdi_sio serial");
            let io = SerialTransport::open(&path)
                .map_err(|e| format!("open serial {path}: {e}"))?;
            serve(listener, io).map_err(|e| e.to_string())
        }
    }
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
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
