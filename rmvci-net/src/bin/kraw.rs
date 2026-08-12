//! Raw K-line frame observer: FAST_INIT an ECU, send one request, then dump
//! EVERY raw response frame (header-on) with elapsed time until the ECU goes
//! quiet. Unlike `diag-cmd`, it does not drain `7F..78` responsePending frames —
//! it shows them, so you can watch a slow service (e.g. DTC read `18`) stream
//! pending frames and finally answer. Read-only diagnostic tool.
//!
//! Usage: kraw <bridge-addr> <ecu-hex> <req hex bytes…>
//!   kraw 192.168.1.207:6979 98 18 00 ff 00

use std::time::{Duration, Instant};

use rmvci_core::{Device, DeviceConfig, Error, KLineEcu, KLINE_TIMING};
use rmvci_net::TcpTransport;

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // Optional `--rate <baud>` overrides the K-line DATA_RATE (param 0x01) — to
    // test whether the Mini-VCI firmware honours it / to apply a per-ECU baud.
    let mut rate: Option<u32> = None;
    if let Some(i) = args.iter().position(|a| a == "--rate") {
        rate = args.get(i + 1).and_then(|s| s.parse().ok());
        args.drain(i..=(i + 1).min(args.len() - 1));
    }
    if args.len() < 3 {
        eprintln!("usage: kraw [--rate <baud>] <bridge-addr> <ecu-hex> <req hex… [then …]>");
        std::process::exit(2);
    }
    let addr = &args[0];
    let ecu = u8::from_str_radix(args[1].trim_start_matches("0x"), 16).expect("bad ecu hex");
    // A sequence of requests separated by the literal token "then".
    let reqs: Vec<Vec<u8>> = args[2..]
        .split(|s| s == "then")
        .map(|grp| grp.iter().map(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).expect("bad req hex")).collect())
        .filter(|v: &Vec<u8>| !v.is_empty())
        .collect();

    let dev = Device::open_transport(TcpTransport::connect(addr).unwrap(), DeviceConfig::default()).unwrap();
    println!("firmware: {}", dev.firmware_version().unwrap());
    let mut e = match rate {
        Some(baud) => {
            let mut timing = KLINE_TIMING.to_vec();
            for p in timing.iter_mut() {
                if p.0 == 0x01 {
                    p.1 = baud;
                }
            }
            println!("(K-line DATA_RATE overridden to {baud})");
            KLineEcu::connect_with(&dev, ecu, &timing).unwrap()
        }
        None => KLineEcu::connect(&dev, ecu).unwrap(),
    };
    println!("FAST_INIT {ecu:#04x}: {:02x?}", e.fast_init().unwrap());

    for req in &reqs {
        // Build the ISO14230 frame manually: <0x80|len> <ecu> <tester=F0> <payload…>
        let mut frame = vec![0x80 | (req.len() as u8), ecu, 0xf0];
        frame.extend_from_slice(req);
        let chan = e.channel();
        chan.clear_periodic().unwrap(); // purge stale RX + stop periodics before each request
        println!("-> {:02x?}", frame);
        chan.write(&frame).unwrap();

        let t0 = Instant::now();
        let mut n = 0;
        // Read until the ECU stays quiet for one full timeout. Cap at 60 frames
        // so an infinite-pending LID doesn't hang forever.
        loop {
            match chan.read(Duration::from_millis(2000)) {
                Ok(msg) => {
                    n += 1;
                    println!("  [{:>6.0}ms] #{n:<3} {:02x?}", t0.elapsed().as_secs_f64() * 1000.0, msg.data);
                    if n >= 60 {
                        println!("  … capped at 60 frames");
                        break;
                    }
                }
                Err(Error::Timeout(_)) => {
                    println!("  [{:>6.0}ms] quiet ({n} frame(s) total)", t0.elapsed().as_secs_f64() * 1000.0);
                    break;
                }
                Err(err) => {
                    println!("  read error: {err}");
                    break;
                }
            }
        }
    }
}
