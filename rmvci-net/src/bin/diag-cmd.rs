//! Interactive diagnostic REPL over a remote `rmvci-bridge` — a general tool for
//! probing a Toyota's KWP2000 / UDS surface by hand, on **either bus**:
//!
//! - **K-line / ISO 14230** (body & HVAC ECUs, addressed by a 1-byte address)
//! - **CAN / ISO-TP** (powertrain, HV, gateway — addressed by a tx/rx id pair)
//!
//! Both are `rmvci_core::UdsTransport`, so raw-send and sweep are identical; only
//! bring-up differs (K-line needs FAST_INIT, CAN needs a tx/rx id pair).
//!
//! ```text
//! 21 0d            send a raw request, print the response (framing per bus)
//! sweep 21 00 ff   send <sid> <id> for id 00..=ff, list what answers
//! kline 07         switch to a K-line ECU address (FAST_INIT)
//! can 7e0 7e8      switch to a CAN ECU (tx / rx ids)
//! init             re-run FAST_INIT (K-line only)
//! help / quit
//! ```
//!
//! Usage:
//! ```sh
//! diag-cmd 192.168.1.207:6979 [--ecu 98]          # start on K-line (default)
//! diag-cmd 192.168.1.207:6979 --can 7e0 7e8       # start on CAN
//! printf '21 02\nsweep 21 00 3f\nquit\n' | diag-cmd 192.168.1.207:6979
//! ```

use std::io::{BufRead, Write};
use std::process::ExitCode;
use std::time::Duration;

use rmvci_core::{CanId, Device, DeviceConfig, Error, FirmwareIsoTp, KLineEcu, UdsTransport};
use rmvci_net::TcpTransport;

const TIMEOUT: Duration = Duration::from_millis(1500);

/// The active target: one ECU on one bus. Both variants are [`UdsTransport`].
enum Bus {
    KLine(KLineEcu),
    Can { io: FirmwareIsoTp, tx: CanId, rx: CanId },
}

impl Bus {
    fn io(&mut self) -> &mut dyn UdsTransport {
        match self {
            Bus::KLine(e) => e,
            Bus::Can { io, .. } => io,
        }
    }

    fn describe(&self) -> String {
        match self {
            Bus::KLine(e) => format!("K-line ECU {:#04x}", e.ecu()),
            Bus::Can { tx, rx, .. } => format!("CAN tx {} rx {}", can_hex(*tx), can_hex(*rx)),
        }
    }
}

fn can_hex(id: CanId) -> String {
    match id {
        CanId::Std(v) => format!("{v:#05x}"),
        CanId::Ext(v) => format!("{v:#010x}"),
    }
}

fn parse_can_id(s: &str) -> Option<CanId> {
    let v = u32::from_str_radix(s.trim_start_matches("0x"), 16).ok()?;
    Some(if v <= 0x7ff { CanId::Std(v as u16) } else { CanId::Ext(v) })
}

/// Parse a whitespace/comma-separated hex string ("21 0d", "210d") into bytes.
fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let cleaned: String = s.chars().filter(|c| !c.is_whitespace() && *c != ',').collect();
    if cleaned.is_empty() || cleaned.len() % 2 != 0 {
        return Err(format!("need an even number of hex digits, got {s:?}"));
    }
    (0..cleaned.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&cleaned[i..i + 2], 16).map_err(|e| format!("bad hex {s:?}: {e}")))
        .collect()
}

fn ascii(bytes: &[u8]) -> String {
    bytes.iter().map(|&b| if (0x20..0x7f).contains(&b) { b as char } else { '.' }).collect()
}

/// Classify + print a service response (already framing-stripped, 0x78-drained).
fn show_response(req: &[u8], resp: &[u8]) {
    let sid = req.first().copied().unwrap_or(0);
    match resp.split_first() {
        None => println!("  <empty response>"),
        Some((&0x7f, tail)) => {
            let nrc = tail.get(1).copied().unwrap_or(0);
            println!("  NEGATIVE  7f {:02x} {:02x}   (NRC {nrc:#04x})", tail.first().copied().unwrap_or(sid), nrc);
        }
        Some((&echo, data)) if echo == sid.wrapping_add(0x40) => {
            println!("  OK  {resp:02x?}");
            println!("      data: {:02x?}  |{}|", data, ascii(data));
        }
        _ => println!("  ?  unexpected: {resp:02x?}"),
    }
}

/// Send `<sid> <id>` for every id in the range and report positive responders.
fn sweep(io: &mut dyn UdsTransport, sid: u8, start: u8, end: u8) {
    println!("sweeping {sid:02x} {start:02x}..={end:02x} …");
    let mut hits = 0;
    for id in start..=end {
        match io.request(&[sid, id], TIMEOUT) {
            Ok(resp) => match resp.split_first() {
                Some((&0x7f, tail)) => {
                    let nrc = tail.get(1).copied().unwrap_or(0);
                    if nrc != 0x31 && nrc != 0x11 && nrc != 0x12 {
                        println!("  {sid:02x} {id:02x} -> NRC {nrc:#04x}");
                    }
                }
                Some((&echo, data)) if echo == sid.wrapping_add(0x40) => {
                    hits += 1;
                    println!("  {sid:02x} {id:02x} -> {:02x?}  |{}|", data, ascii(data));
                }
                _ => println!("  {sid:02x} {id:02x} -> ? {resp:02x?}"),
            },
            Err(Error::Timeout(_)) => {} // silent — no answer
            Err(e) => println!("  {sid:02x} {id:02x} -> transport error: {e}"),
        }
    }
    println!("done: {hits} positive response(s)");
}

fn help() {
    println!(
        "commands:\n  \
         <hex bytes>          send a raw request (first byte = service), e.g. `21 0d`\n  \
         sweep <sid> [s] [e]  send <sid> <id> for id s..=e (default 21 00 ff), list responders\n  \
         session <mode>       StartDiagnosticSession `10 <mode>` (e.g. `session 81`);\n  \
         \x20                    session-gated ECUs (ABS/Body/Gateway/EMPS/Trans) need it\n  \
         tp                   TesterPresent `3e 01` (keepalive for a session)\n  \
         kline <hex>          switch to a K-line ECU address (runs FAST_INIT)\n  \
         can <tx> <rx>        switch to a CAN ECU (ISO-TP tx / rx ids), e.g. `can 7e0 7e8`\n  \
         init                 re-run FAST_INIT (K-line only)\n  \
         help | quit\n  \
         (flags: --session <mode> sends 10 <mode> after bring-up; --tp-each sends\n  \
         \x20         3e 01 before every request to hold a session open)"
    );
}

/// Send `10 <mode>` and report; used by the `session` command and `--session`.
fn start_session(io: &mut dyn UdsTransport, mode: u8) {
    println!("-> [10, {mode:02x}]  (StartDiagnosticSession)");
    match io.request(&[0x10, mode], TIMEOUT) {
        Ok(resp) => show_response(&[0x10, mode], &resp),
        Err(e) => println!("  transport error: {e}"),
    }
}

fn bring_up_kline(dev: &Device, addr: u8) -> Result<Bus, Error> {
    let mut ecu = KLineEcu::connect(dev, addr)?;
    let keys = ecu.fast_init()?;
    println!("K-line ECU {addr:#04x}: FAST_INIT ok, key bytes {keys:02x?}");
    Ok(Bus::KLine(ecu))
}

fn bring_up_can(dev: &Device, tx: CanId, rx: CanId) -> Result<Bus, Error> {
    let io = FirmwareIsoTp::new(dev, tx, rx)?;
    println!("CAN ISO-TP: tx {} rx {}", can_hex(tx), can_hex(rx));
    Ok(Bus::Can { io, tx, rx })
}

fn run(addr: &str, start: Bus0, session: Option<u8>, tp_each: bool) -> Result<(), Box<dyn std::error::Error>> {
    println!("connecting to bridge {addr} …");
    let dev = Device::open_transport(TcpTransport::connect(addr)?, DeviceConfig::default())?;
    println!("cable firmware: {}", dev.firmware_version()?);

    let mut bus = match start {
        Bus0::KLine(a) => bring_up_kline(&dev, a)?,
        Bus0::Can(tx, rx) => bring_up_can(&dev, tx, rx)?,
    };
    if let Some(mode) = session {
        start_session(bus.io(), mode);
    }
    help();

    let stdin = std::io::stdin();
    loop {
        print!("[{}]> ", bus.describe());
        std::io::stdout().flush().ok();
        let mut line = String::new();
        if stdin.lock().read_line(&mut line)? == 0 {
            break; // EOF
        }
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let mut it = line.split_whitespace();
        let cmd = it.next().unwrap();
        match cmd {
            "quit" | "exit" | "q" => break,
            "help" | "?" => help(),
            "init" => match &mut bus {
                Bus::KLine(e) => match e.fast_init() {
                    Ok(keys) => println!("FAST_INIT ok, key bytes {keys:02x?}"),
                    Err(err) => println!("FAST_INIT failed: {err}"),
                },
                Bus::Can { .. } => println!("init is K-line only; CAN needs no FAST_INIT"),
            },
            "kline" | "ecu" => match it.next().map(parse_hex) {
                Some(Ok(bytes)) if bytes.len() == 1 => match bring_up_kline(&dev, bytes[0]) {
                    Ok(new) => bus = new,
                    Err(e) => println!("bring-up failed: {e} (previous target kept)"),
                },
                _ => println!("usage: kline <hex address>, e.g. kline 07"),
            },
            "can" => match (it.next().and_then(parse_can_id), it.next().and_then(parse_can_id)) {
                (Some(tx), Some(rx)) => match bring_up_can(&dev, tx, rx) {
                    Ok(new) => bus = new,
                    Err(e) => println!("bring-up failed: {e} (previous target kept)"),
                },
                _ => println!("usage: can <tx> <rx>, e.g. can 7e0 7e8"),
            },
            "sweep" => {
                let sid = it.next().and_then(|s| parse_hex(s).ok()).and_then(|v| v.first().copied()).unwrap_or(0x21);
                let s = it.next().and_then(|s| parse_hex(s).ok()).and_then(|v| v.first().copied()).unwrap_or(0x00);
                let e = it.next().and_then(|s| parse_hex(s).ok()).and_then(|v| v.first().copied()).unwrap_or(0xff);
                if tp_each {
                    let _ = bus.io().request(&[0x3e, 0x01], TIMEOUT);
                }
                sweep(bus.io(), sid, s, e);
            }
            "session" => match it.next().and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok()) {
                Some(mode) => start_session(bus.io(), mode),
                None => println!("usage: session <mode hex>, e.g. session 81"),
            },
            "tp" => {
                println!("-> [3e, 01]  (TesterPresent)");
                match bus.io().request(&[0x3e, 0x01], TIMEOUT) {
                    Ok(resp) => show_response(&[0x3e, 0x01], &resp),
                    Err(e) => println!("  transport error: {e}"),
                }
            }
            _ => match parse_hex(line) {
                Ok(req) => {
                    if tp_each {
                        // Hold the session open before the real request; ignore its reply.
                        let _ = bus.io().request(&[0x3e, 0x01], TIMEOUT);
                    }
                    println!("-> {req:02x?}");
                    match bus.io().request(&req, TIMEOUT) {
                        Ok(resp) => show_response(&req, &resp),
                        Err(e) => println!("  transport error: {e}"),
                    }
                }
                Err(e) => println!("  {e} (type `help`)"),
            },
        }
    }
    println!("bye");
    Ok(())
}

/// The bus to start on, from the command line.
enum Bus0 {
    KLine(u8),
    Can(CanId, CanId),
}

fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("warn")),
        )
        .with_target(false)
        .init();

    let mut addr = "192.168.1.207:6979".to_string();
    let mut start = Bus0::KLine(0x98);
    let mut session: Option<u8> = None;
    let mut tp_each = false;
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--ecu" => {
                if let Some(v) = it.next().and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok()) {
                    start = Bus0::KLine(v);
                }
            }
            "--can" => match (it.next().as_deref().and_then(parse_can_id), it.next().as_deref().and_then(parse_can_id)) {
                (Some(tx), Some(rx)) => start = Bus0::Can(tx, rx),
                _ => {
                    eprintln!("--can needs <tx> <rx>, e.g. --can 7e0 7e8");
                    return ExitCode::from(2);
                }
            },
            "--session" => session = it.next().and_then(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).ok()),
            "--tp-each" => tp_each = true,
            other => addr = other.to_string(),
        }
    }

    match run(&addr, start, session, tp_each) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}
