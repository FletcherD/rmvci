# rmvci-android

JNI bindings so an Android app can drive the Mini-VCI through `rmvci-core`.

> **Status: hardware-verified on a device.** A rooted moto g fast (Android 11)
> drove the real Mini-VCI through this JNI path to the bench ECU —
> `21 43` -> `61 43 7b 79` — via the [`prius-hvac-android`](../../prius-hvac-android)
> app. [`BRINGUP.md`](BRINGUP.md) has the full account, including the four
> boundary bugs the first device run flushed out (all fixed).

## How it fits together

Java owns the USB permission and the port; Rust owns the protocol.

```
Kotlin UI  ──"read the air-mix servo"──►  rmvci-android (JNI entry points)
                                                  │
                                          rmvci-core: codec, actor,
                                          channels, ISO-TP
                                                  │
                                            JniTransport
                                                  │
   UsbSerialPort  ◄────"write these bytes"────────┘   ← calls back up
   (usb-serial-for-android)
```

Nothing about the protocol is reimplemented for Android — only the bottom
five methods change.

## Why no nusb / libusb

An Android app cannot open a USB device from native code: permission is
granted to the app through `UsbManager`. Rather than pass a file descriptor
down and reimplement the FTDI layer in Rust,
[usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android)
does the byte transport. Its `FtdiSerialDriver` recognises the cable's stock
FT232R (`0403:6001`) with the default prober, and it already handles FTDI
packet framing including the 2-byte status header.

The JNI round trip costs roughly a microsecond against exchanges that take
tens of milliseconds, so the boundary is not a bottleneck.

## Building

```sh
cargo install cargo-ndk
rustup target add aarch64-linux-android armv7-linux-androideabi \
                  x86_64-linux-android i686-linux-android
cargo ndk -t arm64-v8a -t armeabi-v7a -o ../app/src/main/jniLibs build --release
```

Needs the Android NDK; there is one at `~/Android/Sdk/ndk/29.0.14206865` on
the development machine, but `cargo-ndk` is not installed and no NDK build
has been run. What *has* been run is `cargo check --target
aarch64-linux-android`, which validates the code without linking.

## Kotlin side

The exported symbols encode the fully-qualified class name, so the object
**must** be `dev.rmvci.Rmvci`. To use your own package, rename the
`#[unsafe(no_mangle)]` functions to match (`.` → `_`), or switch to
`RegisterNatives` and drop name mangling entirely.

```kotlin
package dev.rmvci

object Rmvci {
    init { System.loadLibrary("rmvci_android") }

    /** Handle, or 0 on failure — then read lastError(). */
    external fun open(port: UsbSerialPort, connection: UsbDeviceConnection): Long
    external fun firmwareVersion(handle: Long): String?
    external fun request(handle: Long, txId: Int, rxId: Int,
                         req: ByteArray, timeoutMs: Int): ByteArray?
    external fun close(handle: Long)
    external fun lastError(): String
}
```

Opening the port, once permission is granted:

```kotlin
val manager = getSystemService(Context.USB_SERVICE) as UsbManager
val driver = UsbSerialProber.getDefaultProber().findAllDrivers(manager).first()
// UsbManager.requestPermission(driver.device, ...) and wait for the grant first
val connection = manager.openDevice(driver.device)
val port = driver.ports[0]
port.open(connection)
port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)

val handle = Rmvci.open(port, connection)
if (handle == 0L) error(Rmvci.lastError())
```

Reading the Prius air-mix servo (`21 43` to the A/C amplifier on `7C4`):

```kotlin
val reply = Rmvci.request(handle, 0x7C4, 0x7CC, byteArrayOf(0x21, 0x43), 2000)
when {
    reply == null -> Log.w("rmvci", Rmvci.lastError())
    reply.size >= 4 && reply[0] == 0x61.toByte() ->
        Log.i("rmvci", "commanded=${reply[2].toInt() and 0xFF} " +
                       "actual=${reply[3].toInt() and 0xFF}")
    else -> Log.w("rmvci", "unexpected: ${reply.joinToString { "%02x".format(it) }}")
}
```

## Things that will bite

- **Every call blocks.** A single exchange is tens of milliseconds. Drive
  them from `Dispatchers.IO`, never the main thread.
- **`close()` must be called**, and it blocks until the session is torn down
  and the port released. Leaking the handle leaks a thread and leaves the
  adapter with a channel still connected — and the firmware only has three.
- **The handle is a raw pointer.** Using it after `close()` is undefined
  behaviour; guard it on the Kotlin side.
- **Detach on USB unplug.** Android delivers `ACTION_USB_DEVICE_DETACHED`;
  call `close()` there or the next request blocks until it times out.
- **A permission grant is not permanent.** It lasts until the device is
  unplugged, so handle the request flow on every attach.

## What is not done

- No `cargo-ndk` build has been run (the NDK is present; `cargo-ndk` is not
  installed). See [`BRINGUP.md`](BRINGUP.md) for the full tooling inventory.
- The latency-timer control transfer in `JniTransport::optimize_latency` is
  written against `UsbDeviceConnection.controlTransfer` but unverified; if it
  fails the driver logs and continues at the 16 ms default.
- Only `FirmwareIsoTp` is exposed. The host-side ISO-TP path works equally
  well over this transport but has no JNI entry point yet — worth adding if
  you hit an ECU that means what it says with flow-control block size.
