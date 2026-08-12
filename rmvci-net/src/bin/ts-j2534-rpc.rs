//! ts-j2534-rpc — host-side J2534 RPC server for the Techstream forwarding shim.
//!
//! The forwarding `mvci_shim.c` (in the WinXP VM) intercepts every PassThru* call
//! Techstream makes, logs it, and forwards it here over TCP. This server replays
//! each call **verbatim** onto the real Mini-VCI cable via rmvci-core's raw
//! firmware path (`RawChannel`) over the rpi4b bridge — NO cooked K-line bring-up,
//! NO responsePending drain, NO auto-reinit. Techstream's exact bytes reach the
//! real Prius and the ECU's raw replies (incl. `7f .. 78` storms) come straight
//! back, so we capture ground-truth wire behavior.
//!
//! Wire protocol: see re/techstream/shim/FORWARD_PROTOCOL.md.
//!
//!   ts-j2534-rpc [PI_ADDR] [LISTEN_ADDR]
//!   ts-j2534-rpc 192.168.1.207:6979 0.0.0.0:9600     (defaults)
//!
//! One shim connects at a time (Techstream = one DLL = one RPC client). Each new
//! connection resets all state (fresh Device/channels).

use std::collections::HashMap;
use std::io::{self, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::Arc;
use std::time::{Duration, Instant};

use rmvci_core::{Device, DeviceConfig, FilterType, ProtocolId, RawChannel};
use rmvci_net::TcpTransport;

// J2534 status codes (subset) — mirror rmvci-j2534/consts + the shim.
const STATUS_NOERROR: i32 = 0x00;
const ERR_FAILED: i32 = 0x07;
const ERR_DEVICE_NOT_CONNECTED: i32 = 0x08;
const ERR_BUFFER_EMPTY: i32 = 0x10;

// Opcodes — must match FORWARD_PROTOCOL.md and the shim.
const OP_OPEN: u8 = 0x01;
const OP_CLOSE: u8 = 0x02;
const OP_CONNECT: u8 = 0x03;
const OP_DISCONNECT: u8 = 0x04;
const OP_WRITE: u8 = 0x05;
const OP_READ: u8 = 0x06;
const OP_STARTFILTER: u8 = 0x07;
const OP_STOPFILTER: u8 = 0x08;
const OP_SETCONFIG: u8 = 0x09;
const OP_GETCONFIG: u8 = 0x0a;
const OP_FASTINIT: u8 = 0x0b;
const OP_IOCTLMISC: u8 = 0x0c;
const OP_STARTPERIODIC: u8 = 0x0d;
const OP_STOPPERIODIC: u8 = 0x0e;

fn main() {
    // Usage:
    //   ts-j2534-rpc [PI_ADDR] [LISTEN_ADDR]          — LIVE: forward to the cable
    //   ts-j2534-rpc --replay LOG [LISTEN_ADDR]        — REPLAY: answer from a capture
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let replay = if args.first().map(|s| s == "--replay").unwrap_or(false) {
        args.remove(0);
        let log = args.remove(0); // required
        let db = Replay::parse(&log);
        eprintln!(
            "ts-j2534-rpc: REPLAY mode from {log} — {} request→response pairs, {} FAST_INITs",
            db.reads.len(),
            db.inits.len()
        );
        Some(Arc::new(db))
    } else {
        None
    };
    // In replay mode the remaining arg is the LISTEN addr (no cable); in live
    // mode it is [PI_ADDR] [LISTEN_ADDR].
    let (pi_addr, listen_addr) = if replay.is_some() {
        (String::new(), args.first().cloned().unwrap_or_else(|| "0.0.0.0:9600".into()))
    } else {
        (
            args.first().cloned().unwrap_or_else(|| "192.168.1.207:6979".into()),
            args.get(1).cloned().unwrap_or_else(|| "0.0.0.0:9600".into()),
        )
    };

    let listener = TcpListener::bind(&listen_addr)
        .unwrap_or_else(|e| panic!("bind {listen_addr}: {e}"));
    if replay.is_none() {
        eprintln!("ts-j2534-rpc: listening on {listen_addr}, cable bridge at {pi_addr}");
    } else {
        eprintln!("ts-j2534-rpc: listening on {listen_addr} (replay — cable not used)");
    }

    for conn in listener.incoming() {
        match conn {
            Ok(stream) => {
                let peer = stream.peer_addr().map(|a| a.to_string()).unwrap_or_default();
                eprintln!("--- shim connected from {peer} ---");
                let mut sess = Session::new(pi_addr.clone(), replay.clone());
                if let Err(e) = sess.serve(stream) {
                    eprintln!("--- shim {peer} disconnected: {e} ---");
                } else {
                    eprintln!("--- shim {peer} disconnected (clean) ---");
                }
            }
            Err(e) => eprintln!("accept error: {e}"),
        }
    }
}

/// Per-connection state. A fresh cable Device is opened on OP_OPEN (live mode);
/// in replay mode the cable is untouched and answers come from `replay`.
struct Session {
    pi_addr: String,
    replay: Option<Arc<Replay>>,
    dev: Option<Device>,
    chans: HashMap<u32, RawChannel>,
    // replay-mode bookkeeping: proto per channel + the last request written on it
    // (with a "consumed" flag so one write yields one read, like the real cable).
    chan_proto: HashMap<u32, u32>,
    last_req: HashMap<u32, (Vec<u8>, bool)>,
    next_chan: u32,
    next_filter: u32,
    next_periodic: u32,
}

impl Session {
    fn new(pi_addr: String, replay: Option<Arc<Replay>>) -> Self {
        Self {
            pi_addr,
            replay,
            dev: None,
            chans: HashMap::new(),
            chan_proto: HashMap::new(),
            last_req: HashMap::new(),
            next_chan: 0x100,
            next_filter: 1,
            next_periodic: 1,
        }
    }

    fn serve(&mut self, mut stream: TcpStream) -> io::Result<()> {
        stream.set_nodelay(true).ok();
        loop {
            let (op, body) = match read_frame(&mut stream) {
                Ok(v) => v,
                Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(()),
                Err(e) => return Err(e),
            };
            let reply = if self.replay.is_some() {
                self.dispatch_replay(op, &body)
            } else {
                self.dispatch(op, &body)
            };
            write_frame(&mut stream, op, &reply)?;
        }
    }

    fn dispatch(&mut self, op: u8, body: &[u8]) -> Vec<u8> {
        let mut r = Cursor::new(body);
        let mut out = Vec::new();
        match op {
            OP_OPEN => {
                let _reserved = r.u32();
                match TcpTransport::connect_timeout(&self.pi_addr, Duration::from_secs(3))
                    .map_err(|e| e.to_string())
                    .and_then(|io| {
                        Device::open_transport(io, DeviceConfig::default())
                            .map_err(|e| e.to_string())
                    }) {
                    Ok(dev) => {
                        self.dev = Some(dev);
                        self.chans.clear();
                        eprintln!("OPEN: cable session up (bridge {})", self.pi_addr);
                        put_i32(&mut out, STATUS_NOERROR);
                        put_u32(&mut out, 1); // deviceId
                    }
                    Err(e) => {
                        eprintln!("OPEN FAILED: {e}");
                        put_i32(&mut out, ERR_DEVICE_NOT_CONNECTED);
                        put_u32(&mut out, 0);
                    }
                }
            }
            OP_CLOSE => {
                let _dev = r.u32();
                self.chans.clear();
                self.dev = None;
                eprintln!("CLOSE: cable session torn down");
                put_i32(&mut out, STATUS_NOERROR);
            }
            OP_CONNECT => {
                let proto = r.u32();
                let flags = r.u32();
                let baud = r.u32();
                let pid = ProtocolId::try_from(proto);
                match (self.dev.as_ref(), pid) {
                    (Some(dev), Ok(pid)) => match dev.connect_raw(pid, flags, baud) {
                        Ok(chan) => {
                            let id = self.next_chan;
                            self.next_chan += 1;
                            self.chans.insert(id, chan);
                            eprintln!(
                                "CONNECT: proto={proto} flags=0x{flags:x} baud={baud} -> ch=0x{id:x}"
                            );
                            put_i32(&mut out, STATUS_NOERROR);
                            put_u32(&mut out, id);
                        }
                        Err(e) => {
                            eprintln!("CONNECT FAILED proto={proto}: {e}");
                            put_i32(&mut out, ERR_FAILED);
                            put_u32(&mut out, 0);
                        }
                    },
                    _ => {
                        eprintln!("CONNECT: no device or bad proto={proto}");
                        put_i32(&mut out, ERR_FAILED);
                        put_u32(&mut out, 0);
                    }
                }
            }
            OP_DISCONNECT => {
                let ch = r.u32();
                self.chans.remove(&ch); // Drop disconnects the proto
                eprintln!("DISCONNECT ch=0x{ch:x}");
                put_i32(&mut out, STATUS_NOERROR);
            }
            OP_WRITE => {
                let ch = r.u32();
                let txflags = r.u32();
                let data = r.bytes();
                match self.chans.get_mut(&ch) {
                    Some(chan) => match chan.write(txflags, &data) {
                        Ok(()) => put_i32(&mut out, STATUS_NOERROR),
                        Err(e) => {
                            eprintln!("WRITE ch=0x{ch:x} FAILED: {e}");
                            put_i32(&mut out, ERR_FAILED);
                        }
                    },
                    None => put_i32(&mut out, ERR_FAILED),
                }
            }
            OP_READ => {
                let ch = r.u32();
                let timeout_ms = r.u32().max(1) as u64;
                match self.chans.get_mut(&ch) {
                    Some(chan) => {
                        let proto = chan.proto().wire();
                        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
                        let mut got: Option<rmvci_core::RxMsg> = None;
                        let mut err: Option<String> = None;
                        loop {
                            match chan.poll(Duration::from_millis(200)) {
                                // Skip transmit echoes and the ISO14230/15765
                                // start-of-message indication (rx_status NOT_DATA,
                                // a header/format frame with no payload) — the real
                                // driver hands the tester assembled data frames, not
                                // these. Matches RawChannel::read and the mock shim.
                                Ok(Some(m)) if m.is_indication() => {}
                                Ok(Some(m)) => {
                                    got = Some(m);
                                    break;
                                }
                                Ok(None) => {}
                                Err(e) => {
                                    err = Some(e.to_string());
                                    break;
                                }
                            }
                            if Instant::now() >= deadline {
                                break;
                            }
                            std::thread::sleep(Duration::from_millis(5));
                        }
                        match (got, err) {
                            (Some(m), _) => {
                                put_i32(&mut out, STATUS_NOERROR);
                                put_u32(&mut out, proto);
                                put_u32(&mut out, m.rx_status.bits());
                                put_bytes(&mut out, &m.data);
                            }
                            (None, Some(e)) => {
                                eprintln!("READ ch=0x{ch:x} error: {e}");
                                put_i32(&mut out, ERR_FAILED);
                                put_u32(&mut out, proto);
                                put_u32(&mut out, 0);
                                put_bytes(&mut out, &[]);
                            }
                            (None, None) => {
                                put_i32(&mut out, ERR_BUFFER_EMPTY);
                                put_u32(&mut out, proto);
                                put_u32(&mut out, 0);
                                put_bytes(&mut out, &[]);
                            }
                        }
                    }
                    None => {
                        put_i32(&mut out, ERR_FAILED);
                        put_u32(&mut out, 0);
                        put_u32(&mut out, 0);
                        put_bytes(&mut out, &[]);
                    }
                }
            }
            OP_STARTFILTER => {
                let ch = r.u32();
                let ftype = r.u32();
                let mask = r.bytes();
                let pattern = r.bytes();
                let has_flow = r.u8() != 0;
                let flow = r.bytes();
                let flow_opt = if has_flow { Some(flow.as_slice()) } else { None };
                match self.chans.get_mut(&ch) {
                    Some(chan) => {
                        let idlen = match chan.proto() {
                            ProtocolId::Can | ProtocolId::Iso15765 => 4,
                            _ => 1,
                        };
                        let fid = self.next_filter;
                        self.next_filter += 1;
                        // Normalize the J2534 filter-type constant to rmvci's enum
                        // for logging clarity; pass the raw ftype through.
                        let ftname = match ftype {
                            x if x == FilterType::Pass as u32 => "pass",
                            x if x == FilterType::Block as u32 => "block",
                            x if x == FilterType::FlowControl as u32 => "flow",
                            _ => "?",
                        };
                        match chan.start_filter(fid, ftype, &mask, &pattern, flow_opt, idlen) {
                            Ok(()) => {
                                eprintln!(
                                    "STARTFILTER ch=0x{ch:x} {ftname} idlen={idlen} -> id={fid}"
                                );
                                put_i32(&mut out, STATUS_NOERROR);
                                put_u32(&mut out, fid);
                            }
                            Err(e) => {
                                eprintln!("STARTFILTER ch=0x{ch:x} FAILED: {e}");
                                put_i32(&mut out, ERR_FAILED);
                                put_u32(&mut out, 0);
                            }
                        }
                    }
                    None => {
                        put_i32(&mut out, ERR_FAILED);
                        put_u32(&mut out, 0);
                    }
                }
            }
            OP_STOPFILTER => {
                let ch = r.u32();
                let fid = r.u32();
                match self.chans.get_mut(&ch) {
                    Some(chan) => match chan.stop_filter(fid) {
                        Ok(()) => put_i32(&mut out, STATUS_NOERROR),
                        Err(e) => {
                            eprintln!("STOPFILTER FAILED: {e}");
                            put_i32(&mut out, ERR_FAILED);
                        }
                    },
                    None => put_i32(&mut out, ERR_FAILED),
                }
            }
            OP_SETCONFIG => {
                let ch = r.u32();
                let n = r.u32();
                let mut status = STATUS_NOERROR;
                match self.chans.get_mut(&ch) {
                    Some(chan) => {
                        for _ in 0..n {
                            let param = r.u32();
                            let value = r.u32();
                            if let Err(e) = chan.set_config(param, value) {
                                eprintln!("SETCONFIG param=0x{param:x} FAILED: {e}");
                                status = ERR_FAILED;
                            }
                        }
                    }
                    None => status = ERR_FAILED,
                }
                put_i32(&mut out, status);
            }
            OP_GETCONFIG => {
                let ch = r.u32();
                let n = r.u32();
                let mut params = Vec::with_capacity(n as usize);
                for _ in 0..n {
                    params.push(r.u32());
                }
                match self.chans.get_mut(&ch) {
                    Some(chan) => {
                        let mut values = Vec::with_capacity(params.len());
                        let mut status = STATUS_NOERROR;
                        for &p in &params {
                            match chan.get_config(p) {
                                Ok(v) => values.push(v),
                                Err(e) => {
                                    eprintln!("GETCONFIG param=0x{p:x} FAILED: {e}");
                                    values.push(0);
                                    status = ERR_FAILED;
                                }
                            }
                        }
                        put_i32(&mut out, status);
                        put_u32(&mut out, values.len() as u32);
                        for v in values {
                            put_u32(&mut out, v);
                        }
                    }
                    None => {
                        put_i32(&mut out, ERR_FAILED);
                        put_u32(&mut out, 0);
                    }
                }
            }
            OP_FASTINIT => {
                let ch = r.u32();
                let mode = r.u32(); // 0 = fast init, 1 = five-baud init
                let init = r.bytes();
                match self.chans.get_mut(&ch) {
                    Some(chan) => {
                        let res = if mode == 1 {
                            chan.five_baud_init(&init)
                        } else {
                            chan.fast_init(&init)
                        };
                        match res {
                            Ok(resp) => {
                                eprintln!(
                                    "FASTINIT ch=0x{ch:x} mode={mode} init={} -> resp={}",
                                    hexs(&init),
                                    hexs(&resp)
                                );
                                put_i32(&mut out, STATUS_NOERROR);
                                put_bytes(&mut out, &resp);
                            }
                            Err(e) => {
                                eprintln!("FASTINIT ch=0x{ch:x} FAILED: {e}");
                                put_i32(&mut out, ERR_FAILED);
                                put_bytes(&mut out, &[]);
                            }
                        }
                    }
                    None => {
                        put_i32(&mut out, ERR_FAILED);
                        put_bytes(&mut out, &[]);
                    }
                }
            }
            OP_IOCTLMISC => {
                let ch = r.u32();
                let ioctl_id = r.u32();
                // 0x09 = CLEAR_PERIODIC_MSGS; others (CLEAR_TX/RX/FILTERS) are
                // firmware no-ops just like rmvci-j2534.
                let status = match self.chans.get_mut(&ch) {
                    Some(chan) if ioctl_id == 0x09 => match chan.clear_periodic() {
                        Ok(()) => STATUS_NOERROR,
                        Err(e) => {
                            eprintln!("CLEAR_PERIODIC FAILED: {e}");
                            ERR_FAILED
                        }
                    },
                    Some(_) => STATUS_NOERROR,
                    None => ERR_FAILED,
                };
                put_i32(&mut out, status);
            }
            OP_STARTPERIODIC => {
                let ch = r.u32();
                let txflags = r.u32();
                let interval_ms = r.u32();
                let msg = r.bytes();
                match self.chans.get_mut(&ch) {
                    Some(chan) => {
                        let mid = self.next_periodic;
                        self.next_periodic += 1;
                        match chan.start_periodic(mid, txflags, interval_ms, &msg) {
                            Ok(()) => {
                                eprintln!(
                                    "STARTPERIODIC ch=0x{ch:x} interval={interval_ms}ms msg={} -> id={mid}",
                                    hexs(&msg)
                                );
                                put_i32(&mut out, STATUS_NOERROR);
                                put_u32(&mut out, mid);
                            }
                            Err(e) => {
                                eprintln!("STARTPERIODIC ch=0x{ch:x} FAILED: {e}");
                                put_i32(&mut out, ERR_FAILED);
                                put_u32(&mut out, 0);
                            }
                        }
                    }
                    None => {
                        put_i32(&mut out, ERR_FAILED);
                        put_u32(&mut out, 0);
                    }
                }
            }
            OP_STOPPERIODIC => {
                let ch = r.u32();
                let mid = r.u32();
                match self.chans.get_mut(&ch) {
                    Some(chan) => match chan.stop_periodic(mid) {
                        Ok(()) => put_i32(&mut out, STATUS_NOERROR),
                        Err(e) => {
                            eprintln!("STOPPERIODIC FAILED: {e}");
                            put_i32(&mut out, ERR_FAILED);
                        }
                    },
                    None => put_i32(&mut out, ERR_FAILED),
                }
            }
            other => {
                eprintln!("unknown opcode 0x{other:02x}");
                put_i32(&mut out, ERR_FAILED);
            }
        }
        out
    }

    /// Replay dispatch: setup ops return synthetic success; READ / FASTINIT are
    /// answered from the parsed capture. The cable is never touched — this lets
    /// the headless harness finalize a vehicle car-off, using the recorded probe
    /// responses as the oracle (see CSHAREDATA_FINALIZE.md).
    fn dispatch_replay(&mut self, op: u8, body: &[u8]) -> Vec<u8> {
        let rp = self.replay.clone().unwrap();
        let mut r = Cursor::new(body);
        let mut out = Vec::new();
        match op {
            OP_OPEN => {
                let _ = r.u32();
                self.chan_proto.clear();
                self.last_req.clear();
                put_i32(&mut out, STATUS_NOERROR);
                put_u32(&mut out, 1);
            }
            OP_CLOSE => {
                let _ = r.u32();
                put_i32(&mut out, STATUS_NOERROR);
            }
            OP_CONNECT => {
                let proto = r.u32();
                let _flags = r.u32();
                let _baud = r.u32();
                let id = self.next_chan;
                self.next_chan += 1;
                self.chan_proto.insert(id, proto);
                put_i32(&mut out, STATUS_NOERROR);
                put_u32(&mut out, id);
            }
            OP_DISCONNECT => {
                let ch = r.u32();
                self.chan_proto.remove(&ch);
                self.last_req.remove(&ch);
                put_i32(&mut out, STATUS_NOERROR);
            }
            OP_WRITE => {
                let ch = r.u32();
                let _tx = r.u32();
                let data = r.bytes();
                self.last_req.insert(ch, (data, false));
                put_i32(&mut out, STATUS_NOERROR);
            }
            OP_READ => {
                let ch = r.u32();
                let _to = r.u32();
                let proto = self.chan_proto.get(&ch).copied().unwrap_or(4);
                // One recorded answer per write (mimics the real cable): consume it.
                let resp = match self.last_req.get_mut(&ch) {
                    Some((req, consumed)) if !*consumed => match rp.reads.get(req) {
                        Some(data) => {
                            *consumed = true;
                            Some(data.clone())
                        }
                        None => {
                            eprintln!("REPLAY: no recorded answer for {}", hexs(req));
                            None
                        }
                    },
                    _ => None,
                };
                match resp {
                    Some(data) => {
                        put_i32(&mut out, STATUS_NOERROR);
                        put_u32(&mut out, proto);
                        put_u32(&mut out, 0);
                        put_bytes(&mut out, &data);
                    }
                    None => {
                        put_i32(&mut out, ERR_BUFFER_EMPTY);
                        put_u32(&mut out, proto);
                        put_u32(&mut out, 0);
                        put_bytes(&mut out, &[]);
                    }
                }
            }
            OP_FASTINIT => {
                let _ch = r.u32();
                let _mode = r.u32();
                let init = r.bytes();
                match rp.inits.get(&init) {
                    Some(resp) => {
                        eprintln!("REPLAY FASTINIT {} -> {}", hexs(&init), hexs(resp));
                        put_i32(&mut out, STATUS_NOERROR);
                        put_bytes(&mut out, resp);
                    }
                    None => {
                        eprintln!("REPLAY: no recorded FAST_INIT for {}", hexs(&init));
                        put_i32(&mut out, ERR_FAILED);
                        put_bytes(&mut out, &[]);
                    }
                }
            }
            OP_STARTFILTER => {
                let fid = self.next_filter;
                self.next_filter += 1;
                put_i32(&mut out, STATUS_NOERROR);
                put_u32(&mut out, fid);
            }
            OP_GETCONFIG => {
                let _ch = r.u32();
                let n = r.u32();
                put_i32(&mut out, STATUS_NOERROR);
                put_u32(&mut out, n);
                for _ in 0..n {
                    put_u32(&mut out, 0);
                }
            }
            OP_STARTPERIODIC => {
                let mid = self.next_periodic;
                self.next_periodic += 1;
                put_i32(&mut out, STATUS_NOERROR);
                put_u32(&mut out, mid);
            }
            // STOPFILTER / SETCONFIG / IOCTLMISC / STOPPERIODIC: just ack.
            _ => {
                put_i32(&mut out, STATUS_NOERROR);
            }
        }
        out
    }
}

/// A capture (`mvci_calls.log`) parsed into request→response maps for replay.
struct Replay {
    /// WriteMsgs data → the ReadMsgs data that answered it (first non-pending).
    reads: HashMap<Vec<u8>, Vec<u8>>,
    /// FAST_INIT input frame → its response frame.
    inits: HashMap<Vec<u8>, Vec<u8>>,
}

impl Replay {
    fn parse(path: &str) -> Replay {
        let text =
            std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        let mut reads: HashMap<Vec<u8>, Vec<u8>> = HashMap::new();
        let mut inits: HashMap<Vec<u8>, Vec<u8>> = HashMap::new();
        let mut last_write: Option<Vec<u8>> = None;
        let mut last_init: Option<Vec<u8>> = None;
        for line in text.lines() {
            if line.contains("PassThruWriteMsgs") {
                if let Some(d) = extract_after(line, "data=") {
                    last_write = Some(d);
                }
            } else if line.contains("PassThruReadMsgs") {
                match extract_after(line, "data=") {
                    Some(d) => {
                        // K-line pending frame <fmt> f0 <ecu> 7f <svc> 78: keep the
                        // pending write pending so the real answer pairs next read.
                        let pending = d.len() >= 6 && d[3] == 0x7f && d[5] == 0x78;
                        if !pending {
                            if let Some(w) = last_write.take() {
                                reads.entry(w).or_insert(d);
                            }
                        }
                    }
                    None => last_write = None, // BUFFER_EMPTY: silent request
                }
            } else if line.contains("FAST_INIT input") {
                last_init = extract_after(line, "data=");
            } else if line.contains("init ->") {
                if let (Some(i), Some(resp)) = (last_init.take(), extract_after(line, "resp=")) {
                    inits.entry(i).or_insert(resp);
                }
            }
        }
        Replay { reads, inits }
    }
}

/// Parse the space-separated hex bytes following `marker` in a log line.
fn extract_after(line: &str, marker: &str) -> Option<Vec<u8>> {
    let i = line.find(marker)? + marker.len();
    let v: Vec<u8> = line[i..]
        .split_whitespace()
        .map_while(|t| u8::from_str_radix(t, 16).ok())
        .collect();
    if v.is_empty() { None } else { Some(v) }
}

// ---- framing ------------------------------------------------------------

/// Read one `[u32 len][u8 op][body]` frame. Returns (op, body).
fn read_frame(stream: &mut TcpStream) -> io::Result<(u8, Vec<u8>)> {
    let mut lenb = [0u8; 4];
    read_exact_eof(stream, &mut lenb)?;
    let len = u32::from_le_bytes(lenb) as usize;
    if len == 0 || len > 8 * 1024 * 1024 {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "bad frame length"));
    }
    let mut buf = vec![0u8; len];
    stream.read_exact(&mut buf)?;
    let op = buf[0];
    Ok((op, buf[1..].to_vec()))
}

fn write_frame(stream: &mut TcpStream, op: u8, body: &[u8]) -> io::Result<()> {
    let len = (body.len() + 1) as u32;
    stream.write_all(&len.to_le_bytes())?;
    stream.write_all(&[op])?;
    stream.write_all(body)?;
    stream.flush()
}

/// Like read_exact but maps a clean initial EOF to UnexpectedEof so serve() can
/// end the loop quietly when the shim closes the socket between frames.
fn read_exact_eof(stream: &mut TcpStream, buf: &mut [u8]) -> io::Result<()> {
    let mut filled = 0;
    while filled < buf.len() {
        match stream.read(&mut buf[filled..]) {
            Ok(0) => return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "eof")),
            Ok(n) => filled += n,
            Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(e),
        }
    }
    Ok(())
}

// ---- little-endian body codec ------------------------------------------

struct Cursor<'a> {
    b: &'a [u8],
    pos: usize,
}
impl<'a> Cursor<'a> {
    fn new(b: &'a [u8]) -> Self {
        Self { b, pos: 0 }
    }
    fn u8(&mut self) -> u8 {
        let v = self.b.get(self.pos).copied().unwrap_or(0);
        self.pos += 1;
        v
    }
    fn u32(&mut self) -> u32 {
        let mut a = [0u8; 4];
        for i in 0..4 {
            a[i] = self.b.get(self.pos + i).copied().unwrap_or(0);
        }
        self.pos += 4;
        u32::from_le_bytes(a)
    }
    fn bytes(&mut self) -> Vec<u8> {
        let n = self.u32() as usize;
        let end = (self.pos + n).min(self.b.len());
        let v = self.b[self.pos..end].to_vec();
        self.pos += n;
        v
    }
}

fn put_i32(out: &mut Vec<u8>, v: i32) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}
fn put_bytes(out: &mut Vec<u8>, b: &[u8]) {
    put_u32(out, b.len() as u32);
    out.extend_from_slice(b);
}

fn hexs(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect::<Vec<_>>().join(" ")
}
