# rMVCI

A Rust driver for the Toyota **Mini-VCI** J2534 cable (XHorse M-VCI: FTDI
FT232R + NXP LPC2119 running "J2534 MINI V1.03"), on Linux, macOS and
Windows.

Three API surfaces over one implementation:

- **`rmvci-core`** — a native Rust API: `Device`, typed `Channel<P>`, and
  ISO 15765-2 both through the firmware and host-side over raw CAN — including
  extended/mixed addressing on both paths.
- **`rmvci-j2534`** — a `cdylib` exporting the 14 SAE J2534 `PassThru*`
  functions, so Techstream and other J2534 hosts can load it. Every export the
  firmware can back is wired through — periodic messages, the config/init/vbatt
  IOCTLs and programming voltage included (see *J2534 surface* below).
- **`rmvci-android`** — JNI entry points so an Android app can drive the
  cable, with Java owning the USB permission. *(Hardware-verified on a phone:
  the `prius-hvac-android` app read `21 43` -> `61 43 7b 79` over the real
  Mini-VCI.)*

Everything the driver does on the wire is derived from reverse engineering
the cable's own firmware (`../re/FINDINGS.md`) and is validated against
hardware.

## Quick start

```sh
cargo build --workspace
cargo test --workspace          # 74 offline tests, no hardware needed

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
- **`READ_VBATT` is hardcoded to 12000 mV, and `SetProgrammingVoltage` drives
  no pin.** Both are wired through faithfully (a real round trip that also
  proves the cable is alive), so cables whose firmware implements them work —
  but on the Mini-VCI the battery voltage is a firmware constant and a
  programming-voltage set is a no-op that reports success. Don't read either as
  a measurement/effect on this cable.
- **`BLOCK` filters do nothing.** The firmware accepts and ignores them.
- **Periodic + extended addressing is inexpressible.** A periodic message is
  capped at 12 bytes (`N < 13`), which a 4-byte id + 8 data bytes already fills,
  leaving no room for the address byte.
- **Reopening a port right after dropping a `Device` fails.** Teardown is
  asynchronous; call `Device::close()` when you intend to reopen.

## J2534 surface

Every `PassThru*` export is implemented as far as the cable's firmware backs
it — this is a general J2534 driver, not just a Prius reader:

- **Periodic messages** — `PassThruStartPeriodicMsg` / `StopPeriodicMsg`
  (firmware 0x0F/0x10) and `CLEAR_PERIODIC_MSGS`. The MsgID is host-assigned
  (like a filter handle), returned in `*pMsgID`.
- **IOCTLs** — `GET_CONFIG`, `SET_CONFIG`, `READ_VBATT`, `FAST_INIT`,
  `FIVE_BAUD_INIT`, `CLEAR_PERIODIC_MSGS`, and the buffer/filter clears.
  (`GET_CONFIG`/`READ_VBATT` reply layouts are RE-derived — see below.)
- **`SetProgrammingVoltage`** — passthrough to firmware 0x0D (a no-op on the
  Mini-VCI; real on cables that implement it).
- **Protocols** — CAN/ISO15765 and K-line (ISO14230 **and** ISO9141, which
  share the firmware's K-line object; `Channel<Iso9141>` now has the same
  `write`/`fast_init`/`five_baud_init` surface). J1850 VPW/PWM are reachable
  only via the raw `RawChannel` path — the firmware objects exist but are
  unexercised, so there is no typed API and no hardware claim.
- **Extended/mixed ISO-TP addressing** — on both the firmware path
  (`FirmwareIsoTp::with_ext_addr`) and the host path
  (`IsoTpConfig::with_ext_addr`), and via a 5-byte J2534 flow-control filter
  with `ISO15765_ADDR_TYPE`.
- **Host-path ISO15765 channel** — connect an ISO15765 channel with the vendor
  `RMVCI_HOST_ISOTP` ConnectFlag (0x8000_0000) to run ISO-TP host-side over raw
  CAN instead of the firmware. Slower per frame, but it segments payloads beyond
  255 bytes correctly and honors the ECU's BS/STmin — the two things the
  firmware path cannot do (see *Which ISO-TP path?*). The default (flag unset)
  stays the fast firmware path. NB: the cable has a single global RX owner, so a
  channel is one or the other, never both at once.

> **Bench-verify pending.** Three wire behaviours are derived from the firmware
> RE and not yet confirmed on hardware: the `GET_CONFIG`/`READ_VBATT` reply
> framing (assumed `[ILEN][0x0e][value u32 LE]`), and whether the firmware
> leaves the extended-addressing byte at the head of the reassembled reply
> (`FirmwareIsoTp::recv` strips it when RxStatus 0x80 is set). Run
> `re/bench/isotp_responder_extaddr.py` with the `live_can_extended_addressing`
> / `live_ioctl_readback` tests to settle them.

## Which ISO-TP path?

| | `FirmwareIsoTp` (protocol 6) | `IsoTp` (protocol 5, host-side) |
|---|---|---|
| Transmit > 255 bytes | **broken** (FF_DL truncated to 8 bits) — refused with `FirmwareFfDlLimit` | full 12-bit FF_DL, to 4095 |
| Flow-control block size | ignored — **measured**: sent 7 when granted 2 | honored — **measured**: paused at exactly 2 |
| STmin | ignored — **measured**: ~0 ms gaps when asked for 50 | honored — **measured**: 50.0 ms; incl. the 0xF1–0xF9 sub-ms range |
| `FS=WAIT` / `FS=OVFLW` | treated as CTS | honored / reported |
| Receive reassembly | correct FF_DL handling, but into a **single global scratch buffer shared by every protocol object, appended with no bounds check** — a response beyond ~1500 bytes corrupts adapter RAM | on the host, bounded by `Vec` |
| CF sequence error on receive | dropped silently, no notification | `SequenceError { expected, got }` |
| Speed | fast (segmentation on the MCU) | ~20–35 ms per frame — every frame is a USB round trip |

Both implement `UdsTransport`, so switching is one line. Use the firmware
path for short requests (the `21 43` case); use the host path when
correctness on long or flow-controlled transfers matters more than speed.
Through the J2534 shim the choice is the `RMVCI_HOST_ISOTP` ConnectFlag; in the
native API it is `FirmwareIsoTp` vs `IsoTp`.

Worth being explicit about the reassembly row, because it is the one failure
the driver cannot defend against: on the firmware path the adapter does the
reassembly, so an ECU that returns a very long response can overrun its
scratch buffer, and nothing on the host side gets a say. On the host path the
firmware only ever sees single CAN frames, so that class of failure does not
exist. (The ~1500-byte figure is from firmware analysis and is not
independently measured — treat it as an order of magnitude, not a threshold.)

## Measured on the real cable

### There is no keepalive, because the adapter does not need one

libMVCI polls the cable every 15 ms on the premise that "the adapter resets if
left idle". That premise does not survive measurement.
`live_idle_decay_characterisation` idles the link and then probes upward —
session, then channel, then a full request/response exchange — so it can tell
"still alive" from "alive but the channel was silently dropped", which a
device-level query alone cannot:

| idle | session | channel | exchange |
|---|---|---|---|
| 5 s | ok | ok | ok |
| 30 s | ok | ok | ok |
| 60 s | ok | ok | ok |
| 300 s | ok | ok | ok |
| **900 s** | **ok** | **ok** | **ok** |

After 15 minutes of complete silence the DES session, the connected channel,
its acceptance filter and a real `21 43` exchange all still work. So
`DeviceConfig::keepalive` defaults to `None` and rMVCI sends nothing when it
has nothing to say.

Wedge detection does not depend on it: the actor counts consecutive
*unanswered exchanges* from real requests, so a dead adapter is caught while
you are actually using it, at zero idle cost. A rejection doesn't count — an
adapter saying "no" is an adapter that is alive.

Set `keepalive: Some(..)` if you want the RX ring drained during long quiet
periods. Note it is **not** an ECU tester-present: a K-line ISO14230 session
has its own P3 timeout at the *ECU*, which your application must service.

### The firmware's ISO-TP transmit really does ignore flow control

`re/FINDINGS.md` §10.3 derived this from firmware disassembly but had never
tested it against a peer that stresses it. `re/bench/isotp_fc_probe.py` does:
it answers a First Frame with `FC BS=2 STmin=50ms` and measures what comes
back. Sending the same 60-byte payload down each path:

| | frames before pausing (asked 2) | inter-frame gap (asked 50 ms) |
|---|---|---|
| `FirmwareIsoTp` (protocol 6) | **7** — ignored | **~0 ms** — ignored |
| `IsoTp` (host, protocol 5) | **2** — honored | **50.0 ms** — honored |

That is the concrete reason the host path exists, now measured rather than
inferred. Against an ECU that means what it says with `BS`, the firmware path
will overrun it.

## Debugging

The driver logs through `tracing`; install any subscriber and set `RUST_LOG`.
`rmvci_core=trace` hex-dumps every inner command and reply (what
`MVCI_DEBUG=1` did in the C driver), `rmvci_core=debug` shows handshakes and
connect retries, and `warn` reports keepalive failures and the FTDI latency
timer it could not set.

```sh
RUST_LOG=rmvci_core=trace cargo run -p prius-hvac -- /dev/ttyUSB0
```

## Transports

The core talks to the cable through a five-method `Transport` trait, so the
backend is swappable and everything above it is untouched.

| backend | feature | use when |
|---|---|---|
| `SerialTransport` | `serial` (default) | normal desktop use via the kernel's `ftdi_sio` |
| `UsbTransport` | `usb` | you want the latency fix without root, or there is no `ftdi_sio` |
| `JniTransport` | `jni-transport` | Android: Java owns the port, Rust calls back up (see [`rmvci-android`](rmvci-android/README.md)) |
| `MockTransport` | always | tests; scripted byte-exact exchanges |

```rust
let dev = Device::open("/dev/ttyUSB0")?;                              // serial
let dev = Device::open_usb(Some("A69QL5OE"), Default::default())?;    // raw USB
```

`UsbTransport` drives the FT232R directly with [nusb](https://github.com/kevinmehall/nusb)
(pure Rust, no libusb). It needs write access to the `/dev/bus/usb` node,
which on most desktops the `plugdev` ACL already grants.

It takes the interface away from `ftdi_sio` while open, so the
`/dev/ttyUSBn` node disappears and comes back around it. Two things to know:

- **Call `Device::close()`.** The rebind happens when the transport is
  dropped, and a process that just exits can race the actor's teardown and
  skip it — leaving the cable with no tty until it is replugged, which is
  very confusing from the outside. `close()` waits.
- Give udev a couple of seconds to recreate the `/dev/serial/by-id` symlink
  before concluding it did not come back.

**Measured on the cable** (`usb_vs_serial_latency`), median per exchange:

| | per `21 43` exchange |
|---|---|
| tty, latency timer at the 16 ms default | 48.0 ms |
| raw USB, timer set to 2 ms by control transfer | **30.0 ms** |

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

# Extended/mixed addressing (settles the RE-only strip-offset question):
python3 ../re/bench/isotp_responder_extaddr.py -v &
RMVCI_PORT=... cargo test --test live live_can_extended -- --ignored --nocapture

# Periodic emission, and the GET_CONFIG/READ_VBATT reply layouts:
RMVCI_PORT=... cargo test --test live live_periodic     -- --ignored --nocapture
RMVCI_PORT=... cargo test --test live live_ioctl_readback -- --ignored --nocapture
```

Fuzzing (the sans-IO layers make this cheap — needs `cargo install cargo-fuzz`
and a current nightly):

```sh
cargo +nightly fuzz run fuzz_deframe -- -max_total_time=60
# targets: fuzz_deframe, fuzz_decrypt, fuzz_inner_parse, fuzz_isotp_rx,
#          fuzz_isotp_rx_extaddr
```

## Layout

```
rmvci-core/src/
  codec/       sans-IO wire protocol: framing, DES-ECB, deframer, inner commands
  transport/   Transport trait; serialport backend, scripted mock
  session/     port-owning actor, handshake, Device, RawChannel, Channel<P>
  isotp/       sans-IO Tx/Rx machines + the two ISO-TP clients
rmvci-j2534/   the 14 PassThru exports (desktop only — nothing loads J2534 on Android)
rmvci-android/ JNI entry points for an Android app (compiles for aarch64, untested on a
               device — see rmvci-android/BRINGUP.md for the on-device test plan)
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
