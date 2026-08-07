# rMVCI

A Rust driver for the Toyota **Mini-VCI** J2534 cable (XHorse M-VCI: FTDI
FT232R + NXP LPC2119 running "J2534 MINI V1.03"), on Linux, macOS and
Windows.

Two API surfaces over one implementation:

- **`rmvci-core`** — a native Rust API: `Device`, typed `Channel<P>`, and
  ISO 15765-2 both through the firmware and host-side over raw CAN.
- **`rmvci-j2534`** — a `cdylib` exporting the 14 SAE J2534 `PassThru*`
  functions, so Techstream and other J2534 hosts can load it.

Everything the driver does on the wire is derived from reverse engineering
the cable's own firmware (`../re/FINDINGS.md`) and is validated against
hardware.

## Quick start

```sh
cargo build --workspace
cargo test --workspace          # 54 offline tests, no hardware needed

# Read the Prius A/C amplifier air-mix servo (7C4, KWP 21 43):
cargo run -p prius-hvac -- /dev/serial/by-id/usb-XHorse_M-VCI_...-if00-port0
```

```rust
use rmvci_core::{CanId, Device, FirmwareIsoTp};
use std::time::Duration;

let dev = Device::open("/dev/ttyUSB0")?;
let mut tp = FirmwareIsoTp::new(&dev, CanId::Std(0x7c4), CanId::Std(0x7cc))?;
let reply = tp.request(&[0x21, 0x43], Duration::from_secs(2))?;  // 61 43 <cmd> <act>
```

## Traps this device sets for you

Each of these cost real debugging time and is encoded in the API or the
error messages:

- **Nothing is received until a filter is installed.** The acceptance filter
  comes up *enabled and empty*, so a filterless channel is completely silent
  — indistinguishable from a dead bus. `Channel::read` returns
  `Error::NoFilterInstalled` rather than a bare timeout, and the J2534 shim
  says so in `PassThruGetLastError`.
- **Use an exact `FF FF FF FF` mask for one CAN id.** The adapter matches the
  *range* `[pattern & mask, pattern | !mask]`, so the J2534-habitual
  `0x000007FF` opens it to `0xFFFFFFCC` and matches nothing useful.
  `CanFilter::exact` / `FlowControlFilter::exact` bake the right mask in; the
  trap is opt-in via `with_mask`.
- **A filter *replaces* the previous one.** The firmware keeps one live
  acceptance range per identifier width, so the API is `set_filter`
  (singular), not `add_filter`.
- **Unknown `SET_CONFIG` parameters return success and are discarded.** A
  clean status proves delivery, never effect. `ISO15765_BS`, `STMIN`,
  `BS_TX`, `STMIN_TX`, `WFT_MAX` and `FRAME_PAD` are all stored and never
  read by the firmware.
- **ProtocolIDs outside 1–6 wedge the MCU.** The firmware indexes a
  six-entry table with `id - 1` and no bounds check; recovery needs a port
  reopen (which pulses DTR/RTS and resets the chip). `ProtocolId` is a closed
  enum, so no bad value can reach the wire.
- **`READ_VBATT` is hardcoded to 12000 mV.** There is no ADC read behind it.
- **`BLOCK` filters do nothing.** The firmware accepts and ignores them.
- **Reopening a port right after dropping a `Device` fails.** Teardown is
  asynchronous; call `Device::close()` when you intend to reopen.

## Which ISO-TP path?

| | `FirmwareIsoTp` (protocol 6) | `IsoTp` (protocol 5, host-side) |
|---|---|---|
| Transmit > 255 bytes | **broken** (FF_DL truncated to 8 bits) — refused with `FirmwareFfDlLimit` | full 12-bit FF_DL, to 4095 |
| Flow-control block size | ignored (waits for FC once, then floods) | honored |
| STmin | ignored (fixed ~1 ms) | honored, incl. the 0xF1–0xF9 sub-ms range |
| `FS=WAIT` / `FS=OVFLW` | treated as CTS | honored / reported |
| Receive reassembly | correct FF_DL handling, but into a **single global scratch buffer shared by every protocol object, appended with no bounds check** — a response beyond ~1500 bytes corrupts adapter RAM | on the host, bounded by `Vec` |
| CF sequence error on receive | dropped silently, no notification | `SequenceError { expected, got }` |
| Speed | fast (segmentation on the MCU) | ~20–35 ms per frame — every frame is a USB round trip |

Both implement `UdsTransport`, so switching is one line. Use the firmware
path for short requests (the `21 43` case); use the host path when
correctness on long or flow-controlled transfers matters more than speed.

Worth being explicit about the reassembly row, because it is the one failure
the driver cannot defend against: on the firmware path the adapter does the
reassembly, so an ECU that returns a very long response can overrun its
scratch buffer, and nothing on the host side gets a say. On the host path the
firmware only ever sees single CAN frames, so that class of failure does not
exist. (The ~1500-byte figure is from firmware analysis and is not
independently measured — treat it as an order of magnitude, not a threshold.)

## Measured on the real cable

Two things the C driver assumed turned out not to hold, so rMVCI does
something different:

- **The adapter does not need constant poking.** libMVCI polls it every 15 ms
  because "the adapter resets if left idle". A session here survives **at
  least 5 minutes** of complete silence, with a channel connected and without
  (`live_keepalive_threshold`). The idle poll is kept for liveness detection
  and background RX draining, at a 5 s default instead of 15 ms.
- **A keepalive reply can carry real data.** The C keepalive discarded its
  reply wholesale, so any message it happened to drain was lost. Here the
  reply is parsed and queued for the next `read`.

## Debugging

The driver logs through `tracing`; install any subscriber and set `RUST_LOG`.
`rmvci_core=trace` hex-dumps every inner command and reply (what
`MVCI_DEBUG=1` did in the C driver), `rmvci_core=debug` shows handshakes and
connect retries, and `warn` reports keepalive failures and the FTDI latency
timer it could not set.

```sh
RUST_LOG=rmvci_core=trace cargo run -p prius-hvac -- /dev/ttyUSB0
```

## Throughput: fix the FTDI latency timer

The protocol is strict request/response, and the FT232R holds any reply
under 64 bytes for the full latency interval — 16 ms by default. `Device`
tries to set it to 2 ms via
`/sys/bus/usb-serial/devices/ttyUSBn/latency_timer` and logs what it
achieved. That write needs root, so make it permanent with a udev rule:

```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="2"
```

## Testing

```sh
cargo test --workspace                                   # offline: vectors, properties, mocks
RMVCI_PORT=/dev/serial/by-id/... cargo test --test live -- --ignored --nocapture
```

Offline coverage is in three layers: **24 captured vectors** ported
byte-exact from libMVCI's `mvci_test.c` (real bytes from the vendor
`MVCI32.dll`, so a pass means byte-identical wire behaviour), **property
tests** over the codec and the ISO-TP machines, and **scripted-mock session
tests** that assert exact wire traffic without hardware.

Live tests need the cable; the CAN ones also need the bench fake ECU:

```sh
python3 ../re/bench/isotp_responder.py /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 -v &
RMVCI_PORT=... cargo test --test live live_can -- --ignored --nocapture
```

Fuzzing (the sans-IO layers make this cheap — needs `cargo install cargo-fuzz`
and a current nightly):

```sh
cargo +nightly fuzz run fuzz_deframe -- -max_total_time=60
# targets: fuzz_deframe, fuzz_decrypt, fuzz_inner_parse, fuzz_isotp_rx
```

## Layout

```
rmvci-core/src/
  codec/       sans-IO wire protocol: framing, DES-ECB, deframer, inner commands
  transport/   Transport trait; serialport backend, scripted mock
  session/     port-owning actor, handshake, Device, RawChannel, Channel<P>
  isotp/       sans-IO Tx/Rx machines + the two ISO-TP clients
rmvci-j2534/   the 14 PassThru exports
examples/prius-hvac/
```

The codec and the ISO-TP machines are **sans-IO**: pure functions and state
machines with no I/O and no threads, which is what makes them exhaustively
property-testable.

## Licence

GPL-3.0-or-later. rMVCI is a from-scratch Rust implementation but is derived
from [libMVCI](https://github.com/aselafernando/libMVCI) (GPL-3.0): the
protocol layering follows its C sources and its captured test vectors are
carried over verbatim.
