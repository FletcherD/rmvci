# rmvci-core

A native Rust driver for the Toyota **Mini-VCI** J2534 cable (XHorse M-VCI:
FTDI FT232R + NXP LPC2119 running "J2534 MINI V1.03"), on Linux, macOS and
Windows.

Everything this crate does on the wire is derived from reverse-engineering the
cable's own firmware, and every frame is bench-verified against the real cable.
It is the foundation the [`rmvci-diag`](https://crates.io/crates/rmvci-diag)
diagnostic clients, the J2534 `cdylib`, and the Android JNI layer are built on.

## What's in it

- **`Device`** — opens the port, performs the DES handshake, owns one actor
  thread that serialises access to the cable.
- **Typed `Channel<P>`** — a bus channel whose `ProtocolId` and filter-identifier
  width are fixed at compile time, so the two ways to brick the adapter
  (an out-of-range protocol, a wrong-width filter) are *unrepresentable*.
- **`IsoTp`** — ISO 15765-2 on either `IsoTpPath`: through the firmware
  (fast, the default) or host-side over raw CAN (segments beyond 255 bytes and
  honours the ECU's BS/STmin, which the firmware cannot). Extended/mixed
  addressing on both.
- **`KLineEcu`** — cooked ISO 14230 (K-line) bring-up: the full vendor
  `SET_CONFIG` timing sequence, FAST_INIT, response-pending draining, and
  automatic re-init of an ECU wedged by a pending storm or a silent LID.
- **`codec`** — the sans-IO core: framing, DES-ECB, and inner-command builders,
  usable with no I/O backend at all.

## Backends (Cargo features)

The byte transport is chosen at compile time; the protocol core never depends on
which one you pick.

| Feature | Backend | Notes |
|---|---|---|
| `serial` *(default)* | `serialport`, open-by-path | Linux/macOS/Windows. FTDI latency timer sits at 16 ms unless a udev rule lowers it. |
| `usb` | pure-Rust `nusb`, direct FT232R | No `ftdi_sio`; sets the latency timer without root (measured 48 → 30 ms per exchange). |
| `jni-transport` | bytes supplied by a Java `UsbSerialPort` via JNI | For Android, where Java owns the USB permission. |

```toml
[dependencies]
rmvci-core = "0.1"                                    # serial backend
# rmvci-core = { version = "0.1", features = ["usb"] }
# rmvci-core = { version = "0.1", default-features = false, features = ["jni-transport"] }
```

## Use

```rust
use rmvci_core::{CanId, Device, IsoTp, IsoTpConfig, IsoTpPath};
use std::time::Duration;

let dev = Device::open("/dev/ttyUSB0")?;
let cfg = IsoTpConfig::new(CanId::Std(0x7c4), CanId::Std(0x7cc));
let mut tp = IsoTp::new(&dev, cfg, IsoTpPath::Firmware)?;
let reply = tp.request(&[0x21, 0x43], Duration::from_secs(2))?; // 61 43 <cmd> <act>
```

K-line ECU (ISO 14230), with the full vendor bring-up done for you:

```rust
use rmvci_core::{Device, KLineEcu};

let dev = Device::open("/dev/ttyUSB0")?;
let mut ecu = KLineEcu::connect(&dev, 0x98)?; // filters + 12 SET_CONFIG timings
ecu.fast_init()?;                              // StartCommunication, key bytes
let reply = ecu.request(&[0x21, 0x0d], std::time::Duration::from_secs(1))?;
```

For KWP2000/UDS request-building and response validation on top of this, use
[`rmvci-diag`](https://crates.io/crates/rmvci-diag).

## Traps this device sets for you

Each cost real debugging time and is encoded in the API or the error messages:

- **Nothing is received until a filter is installed.** The acceptance filter
  comes up *enabled and empty*, so a filterless channel is silent —
  indistinguishable from a dead bus. `Channel::read` returns
  `Error::NoFilterInstalled`, not a bare timeout.
- **Use an exact `FF FF FF FF` mask for one CAN id.** The adapter matches the
  *range* `[pattern & mask, pattern | !mask]`; the J2534-habitual `0x000007FF`
  opens it wide and matches nothing useful.
- **`ProtocolId` is a closed enum** and never leaves 1–6, so a bad protocol id
  can't reach the firmware.
- **No keepalive.** The adapter survives ≥15 minutes idle (measured); adding a
  keepalive only risks races. `close()` waits for the port to be released.

## Design

Sans-IO at the bottom (`codec`), one actor thread owning the port in the middle,
typed channels at the top. The 24 captured wire vectors in
`tests/captured_vectors.rs` pin every frame to the genuine vendor DLL's output —
they are ground truth, never "fixed" to match a code change.

## License

GPL-3.0-or-later — structure is ported from
[libMVCI](https://github.com/aselafernando/libMVCI) (GPL-3.0) and its captured
test vectors are carried over verbatim.
