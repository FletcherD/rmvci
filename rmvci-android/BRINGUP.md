# Android bring-up — handoff

Everything needed to get `rmvci-android` running on a real phone, and an
honest account of what is and is not proven. Read
[`README.md`](README.md) first for the API; this document is only about
getting it onto hardware.

## Status in one line

**Done — verified on a device (2026-08-07).** A rooted moto g fast (Android 11)
ran both tiers: Tier 1 (fake port, no USB) returned `J2534 MINIV1.03`, and
Tier 2 drove the real Mini-VCI through this JNI path to the bench ECU,
`21 43` -> `61 43 7b 79`, with a clean ~17 ms close that is reopen-safe.

The first device run was debugging, exactly as warned: it flushed out **four
boundary bugs**, all now fixed. None were below the JNI boundary — the codec,
session, channels and ISO-TP were already correct.

| bug | where | fix |
|---|---|---|
| USB permission always read back "denied" | app | `PendingIntent` must be **mutable** — the system fills `EXTRA_PERMISSION_GRANTED` into it, which `FLAG_IMMUTABLE` forbids |
| "Read buffer too small" on the first handshake read | `JniTransport::read` | `usb-serial-for-android` rejects a destination below the endpoint packet size; read into a 512 B scratch and buffer the surplus for later reads |
| process crash (SIG 9) on close | `JniTransport` write/read/set_modem | clear the pending JNI exception on the error path, or a failed teardown write left it pending and killed the daemon thread |
| ~5 s stall on close | `rmvci-android` close | `Device` is `Clone` and the cached channel holds its own sender clone; drop the ISO-TP channel **before** `device.close()` so the actor sees the session end at once (5 s -> 17 ms) |

The tiers below remain the record of how it was brought up.

## You must build an APK. There is no shortcut.

Both escape hatches were checked and are closed:

- **Native binary via `adb shell`** — the test phone is not rooted (`su` absent,
  uid 2000 `shell`), so there is no `/dev/bus/usb` access. It would not
  exercise the JNI code anyway.
- **Termux** — same permission wall. Its `termux-usb` helper passes a file
  descriptor to a native program, which would exercise an nusb `from_fd` path
  that this crate does not implement.

JNI calls need a JVM. An APK is the only way in.

## Environment as measured (2026-08-07, this machine + phone)

| | |
|---|---|
| SDK | `~/Android/Sdk`, platforms 30 / 36 / 36.1 / 37.0 |
| NDK | `~/Android/Sdk/ndk/29.0.14206865` |
| build-tools | 36.0.0 and 36.1.0 — `aapt2`, `d8`, `apksigner`, `zipalign` all present |
| JDKs | 21, 25, 26 — **default is 26** |
| adb | 1.0.41, on `PATH` |
| Phone | moto g fast (`ZY22C6BVK8`), Android 11 / API 30, **arm64-v8a** |
| USB host | `android.hardware.usb.host` present → OTG supported |
| Root | no |
| Phone WiFi | `192.168.1.235` |
| Rust target | `aarch64-linux-android` installed, compiles clean |

`cargo-ndk` is **not** installed. `gradle` and `kotlinc` are not on `PATH`
(a Gradle wrapper would fetch both).

## Do this in two tiers

### Tier 1 — prove the JNI boundary, no USB

A small Java class that pretends to be a `UsbSerialPort` and replays scripted
bytes. No USB, no OTG, no third-party dependency, and it runs on an emulator.

This is where the bugs actually are: method signatures, the daemon-thread
attach, `byte[]` marshalling, `GlobalRef` lifetimes. One threading bug of
exactly that class was already found and fixed by inspection during
development (see "Traps already paid for" below) — assume there is another.

Because it is Java-only with no dependencies it can skip Gradle entirely:
`javac` → `d8` → `aapt2` → `apksigner`, all of which are installed. That is
minutes, against a first Gradle sync that downloads AGP and Kotlin.

The fake port needs these five methods, matching the signatures
`JniTransport` calls:

```java
void write(byte[] src, int timeout);
int  read(byte[] dest, int timeout);   // bytes read, 0 on timeout
void purgeHwBuffers(boolean r, boolean w);
void setDTR(boolean v);
void setRTS(boolean v);
```

Script it to answer the opening handshake. These fixtures are generated from
the real codec, so they are byte-exact:

```java
// The driver writes this first (reset). Answer with nothing.
// 03 00 03
// Then it writes the identify frame:
// 0c 00 07 00 01 4d 56 43 49 2d 54 62
// Answer with the challenge carrying the DES key (Java bytes are signed):
static final byte[] CHALLENGE = { 14, 0, 9, 0, 1, -80, -53, 73, 104, 7, 69, -56, 127, -87 };

// Rmvci.firmwareVersion() then writes (encrypted under the key above):
// 0b 00 96 72 51 88 2a aa ba 9b 97
// Answer with:
static final byte[] VERSION = { 27, 0, -46, 34, 38, -19, -19, 95, -35, 118, 38, 83,
                                26, -68, 96, 6, 95, 35, -61, 35, -57, 97, -101, 1,
                                -20, -72, 120 };
```

**Pass condition:** `Rmvci.open(fakePort, null)` returns a non-zero handle and
`Rmvci.firmwareVersion(handle)` returns `"J2534 MINIV1.03"`. That single
assertion exercises the whole JNI surface except `optimize_latency`.

To regenerate the fixtures if the protocol ever changes, print them from the
codec — `frame::encode_plain`, `frame::encode_encrypted`, `inner::identify`,
`inner::read_version`, with the `KEY_OLD` used across the test suite.

Note `Rmvci.open` takes a `UsbDeviceConnection` second argument; pass `null`
in tier 1. That path is already handled — `optimize_latency` returns
`Unavailable` and the driver carries on.

### Tier 2 — the real cable

Add [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android)
(needs Gradle for the AAR), build the `.so` with `cargo-ndk`, and run against
the bench ECU.

**The bench rig still works with the phone as host.** It is independent of
what drives the Mini-VCI: phone → Mini-VCI → OBD pins 6/14 → the CH340
analyzer on the desktop running `re/bench/isotp_responder.py`. So a full
end-to-end `21 43` is testable on the phone without the car.

```sh
cargo install cargo-ndk
cargo ndk -t arm64-v8a -o app/src/main/jniLibs build --release
```

**Pass condition:** `Rmvci.request(h, 0x7C4, 0x7CC, byteArrayOf(0x21, 0x43), 2000)`
returns `61 43 7b 79`, matching what both desktop transports return.

## Four things that will bite

1. **The phone has one USB port and the cable will occupy it.** adb-over-USB
   dies the moment you attach the Mini-VCI. Switch to WiFi *while still
   wired*: `adb tcpip 5555` then `adb connect 192.168.1.235:5555`, then unplug.
2. **The default JDK is 26 and no AGP accepts it.** Point `JAVA_HOME` at
   `/usr/lib/jvm/java-21-openjdk-amd64` for anything Gradle. Tier 1 via
   `javac`/`d8` sidesteps this.
3. **USB permission.** The runtime dialog is interactive and tedious to
   repeat. A `device_filter.xml` with a `USB_DEVICE_ATTACHED` intent-filter
   auto-grants on plug — much better for iteration. The cable is a stock FTDI
   `0403:6001`.
4. **Power.** The Mini-VCI is bus-powered, so the phone supplies it over OTG.
   ~100 mA should be within a moto g fast's budget; a powered OTG hub is the
   fallback if it browns out.

## Open question

Is there a **USB-C male → USB-A female OTG adapter** on hand? That is the only
physical unknown; everything else is confirmed present. Tier 1 does not need
it.

## Traps already paid for — do not re-introduce

- **A JNI environment pointer is per thread.** The transport is constructed on
  the caller's thread and then *moved into* the session actor's thread. An
  early version captured an `AttachGuard` in the constructor, which would have
  bound the wrong thread — the kind of fault that appears to work and then
  corrupts. It now stores only `JavaVM` + `GlobalRef`s (genuinely `Send +
  Sync`) and attaches per call as a daemon. **Do not "optimise" that back into
  a cached env.** The per-call cost is about a microsecond against exchanges of
  tens of milliseconds.
- **Local refs die when the native call returns.** The actor outlives that, so
  the port and connection are held as `GlobalRef`.
- **Symbol names encode the package.** Exports are
  `Java_dev_rmvci_Rmvci_{open,firmwareVersion,request,close,lastError}`, so the
  Kotlin/Java object must be `dev.rmvci.Rmvci`. To use another package, rename
  the `#[unsafe(no_mangle)]` functions (`.` → `_`) or switch to
  `RegisterNatives`.
- **`close()` is mandatory and blocking.** It waits for the session teardown.
  Leaking the handle leaks a thread and leaves a channel connected on the
  adapter, which only has three.
- **The handle is a raw pointer.** Use after `close()` is undefined behaviour;
  guard it on the Kotlin side.
- **Every call blocks for tens of milliseconds.** Drive them from
  `Dispatchers.IO`, never the main thread.
- **Clear pending JNI exceptions.** `purge_rx` already does this — a driver
  that lacks `purgeHwBuffers` throws, and an uncleared exception makes the
  *next*, unrelated JNI call fail confusingly.

## If tier 1 fails, look here first

| symptom | likely cause |
|---|---|
| `NoSuchMethodError` / open returns 0 | signature mismatch — check the JNI descriptor strings against the fake port |
| crash inside a JNI call, no Rust panic | thread not attached, or a local ref used across calls |
| `open` returns 0, `lastError()` mentions timeout | the fake port is not answering the identify frame, or the challenge bytes are wrong |
| garbage key / decrypt failure | `byte[]` sign handling in `read` — Java bytes are signed, the layout is identical but the copy must not sign-extend |
| works once, fails on reopen | `close()` not called, so the actor and its handle leaked |

`Rmvci.lastError()` carries the Rust-side error text for every failed call —
check it before anything else.

## Not done

- No `cargo-ndk` build has been run (it is not installed here).
- `optimize_latency` uses `UsbDeviceConnection.controlTransfer` for the FTDI
  latency-timer vendor request. Written, never executed. If it fails the
  driver logs a warning and continues at the 16 ms default — worth checking,
  since on desktop the same fix took a `21 43` exchange from 48 ms to 30 ms.
- Only `IsoTpPath::Firmware` has a JNI entry point. The host-side path works
  equally well over this transport but is not exposed — add it if you meet an
  ECU that means what it says with flow-control block size (see the main
  README for why that matters).
