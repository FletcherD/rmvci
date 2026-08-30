# rmvci-net — drive the Mini-VCI over the network

Run the rMVCI driver on one machine while the cable is plugged into another.
The motivating case: the cable at the car in a **rooted Android phone**, driven
from a **PC** on the same Wi-Fi (or a USB tether).

`rmvci-core`'s [`Transport`] is five raw byte-I/O primitives (`write_all`,
`read`, `purge_rx`, `set_modem`, `optimize_latency`). This crate splits that one
boundary across TCP. **Nothing above the transport changes** — the DES
handshake, the session actor, the typed channels, and both ISO-TP paths all run
locally on the PC; only raw cable bytes cross the wire.

```
   PC  ────────────────── Wi-Fi / LAN ──────────────────  phone (rooted)      car
 ┌──────────────────────┐                          ┌────────────────────┐
 │ your app / rmvci-remote                         │ rmvci-bridge       │
 │  └ Device (codec,      │   TcpTransport   TCP    │  └ serve()         │  USB   ┌─────┐
 │      actor, ISO-TP)  ──┼──►  (client)  ═════════►│     (server) ──────┼──────► │cable│──► OBD
 │                        │                         │   UsbTransport     │        └─────┘
 └──────────────────────┘                          └────────────────────┘
```

## Two halves

- **`TcpTransport`** (PC): implements `Transport` by forwarding each primitive.
  Hand it to `Device::open_transport` — that is the only change to PC-side code.
- **`serve` / `rmvci-bridge`** (phone): owns a real `UsbTransport` /
  `SerialTransport` and executes forwarded primitives. Serves **one client at a
  time** (the cable has a single RX owner); the transport is opened once and
  reused, and each fresh PC-side `Device` re-runs the DTR/RTS reset to resync.

## Use it

PC side, in your own binary (or use the bundled `rmvci-remote`):

```rust
use rmvci_core::{Device, DeviceConfig};
use rmvci_net::TcpTransport;

let io  = TcpTransport::connect("192.168.1.50:6979")?;
let dev = Device::open_transport(io, DeviceConfig::default())?;
println!("firmware: {}", dev.firmware_version()?);
// dev now behaves exactly as a locally-attached cable: connect channels,
// open FirmwareIsoTp / IsoTp, read PIDs, …
```

Phone side:

```sh
rmvci-bridge                        # USB (nusb), first FT232R, 0.0.0.0:6979
rmvci-bridge --usb A69QL5OE         # pick one FT232R by USB serial string
rmvci-bridge --serial /dev/ttyUSB0  # ftdi_sio tty instead of raw USB
rmvci-bridge --listen 0.0.0.0:7000  # bind elsewhere
```

Bundled demo client (proves the round trip end to end):

```sh
rmvci-remote 192.168.1.50:6979              # open + print firmware version
rmvci-remote 192.168.1.50:6979 --hvac       # then read CAN 7C4 / KWP 21 43
rmvci-remote 192.168.1.50:6979 --hvac --watch --isotp host
```

The hands-on diagnostic apps `diag-cmd` (interactive REPL) and `diag-scan`
(whole-vehicle live scan with real names + values) moved to the standalone
[`tsdiag`](../../tsdiag/README.md) crate, which drives **any** transport — a
local cable, USB, or this bridge — and decodes against the Techstream
extraction. The A/C servo read (K-line `0x98`, proven live 2026-08-09) is now
`diag-scan --transport <addr> --ecu A_C_P3.ddb --watch`. `diag-cmd` keeps the
`--ecu`/`--can`/`--session`/`--tp-each` flags described below.

Some K-line ECUs (A/C amp, Immobiliser) answer `21 <LID>` reads straight after
FAST_INIT; others (ABS, Body, Gateway, EMPS, Transmission) answer only their
`21 00` ID read until a **StartDiagnosticSession** (`10 81`) succeeds — use
`--session 81 --tp-each`. A few LIDs (e.g. A/C `21 52`, DTC read `18`) make the
ECU stream `7F .. 78` responsePending forever, which wedges it; `KLineEcu`
detects this, surfaces the NRC, and auto re-FAST_INITs so the next request still
works (so a `sweep` survives storm LIDs). See `re/techstream/LIVE_VALIDATION_2026-08-09.md`.

Raw K-line frame observer (no responsePending drain — shows the whole
pending→final sequence + timing; sends a `then`-separated request sequence):

```sh
kraw 192.168.1.207:6979 98 18 00 ff 00              # watch a DTC read stream 78s
kraw 192.168.1.207:6979 29 21 00 then 10 81 then 21 01   # probe a session flow
```

Both `rmvci_core::KLineEcu` (K-line) and `FirmwareIsoTp` (CAN) are `UdsTransport`,
so raw-send and sweep are bus-agnostic; the framing (ISO14230 header vs ISO-TP)
and 0x78 handling happen in the library. No fixed PID list — for exploring the
car's full surface across both networks. Proven live on both buses.

## Two ways to run the phone-side bridge

`serve()` is generic over any `Transport`, so the *server* half runs anywhere a
cable-owning transport exists. On Android the standalone-binary path is a dead
end, so there are two homes for it:

### On Android → app-hosted over `JniTransport` (this is what works)

`rmvci-bridge` the **binary** does **not** work on Android:

- `nusb` (the `--usb` backend) does not compile for `target_os = "android"` — its
  device-enumeration API is gated off (only `Device::from_fd` is exposed).
- The phones tested have **no `ftdi_sio`** in the kernel, so `--serial` has no
  `/dev/ttyUSB0` either.

Instead the bridge is **hosted inside the Android app** over the proven
`JniTransport` (Java `UsbSerialPort` owns the USB permission). `rmvci-android`
exports `startBridge(port, connection, tcpPort)` / `stopBridge(handle)`, which
build a `JniTransport` and run `rmvci_net::serve_connection` on a background
thread. The `prius-hvac-android` app has a **"Start bridge"** button that opens
the cable and shows `bridge listening on <phone-ip>:6979`. The app needs the
`INTERNET` permission (Android gates the listening socket on it, native code
included). See `rmvci-android/src/lib.rs` and that app's `MainActivity`.

### On a Linux SBC / desktop → the standalone binary

Where `nusb` *does* build (a Linux host — Raspberry Pi, laptop), the binary is
the simplest option:

```sh
cargo build -p rmvci-net --features bridge --release   # rmvci-bridge
rmvci-bridge --usb                                     # or --serial /dev/ttyUSB0
```

The PC-side client/library builds for the host normally:

```sh
cargo build -p rmvci-net --features client   # rmvci-remote
cargo build -p rmvci-net                      # library only (TcpTransport)
```

## Latency — why the link matters

Each `read` is one network **round trip**: the PC asks, the bridge blocks its
cable read for up to the requested timeout, then replies. The MVCI protocol is
strict request/response, so a full exchange is a handful of round trips. On a
LAN or USB tether (sub-millisecond to low-single-digit ms RTT) this is
comfortable; over a congested/high-latency link it slows every exchange
proportionally. `TCP_NODELAY` is set on both ends so Nagle never adds a delay.

The FTDI **latency timer** is set on the *bridge* against the real cable
(`optimize_latency` is forwarded), so the `--usb` backend's no-root 16 → 2 ms
win still applies — the timer that matters is at the hardware, not the socket.

## Wire protocol

A 5-byte hello (`"RMVN"` + version `1`) then length-prefixed frames
(`u32` LE body length + body). Request body = one tag byte
(`WRITE`/`READ`/`PURGE`/`SET_MODEM`/`OPT_LATENCY`) + args; response body = a
status byte (`OK`/`ERR`) + payload. One TCP stream, strict request/response, so
ordering needs no request ids. A remote transport error comes back as an `ERR`
string and resurfaces as a `TransportError` on the PC. Details in `src/wire.rs`.

## Tests

`cargo test -p rmvci-net` runs `tests/loopback.rs`: a `MockTransport` behind
`serve_connection` on one thread, a `TcpTransport` client on another, over a
real loopback socket — every primitive round-trips byte-for-byte, and a version
mismatch is rejected at the hello. No hardware needed.

Still unproven: **on a phone against the real cable** (the JNI backend is
hardware-verified, but this bridge uses the native `UsbTransport`/`SerialTransport`
paths, which are bench-verified on the PC but not yet on an Android host).

[`Transport`]: ../rmvci-core/src/transport/mod.rs
