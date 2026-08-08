//! JNI entry points for an Android app.
//!
//! Java owns the USB permission and the port; Rust owns the protocol. The
//! app opens a `UsbSerialPort` (see [`rmvci_core::transport::jni`]), passes
//! it here, and gets back an opaque handle it can make diagnostic requests
//! against.
//!
//! Expected Kotlin declarations:
//!
//! ```kotlin
//! package dev.rmvci          // MUST match the symbol names below
//!
//! object Rmvci {
//!     init { System.loadLibrary("rmvci_android") }
//!
//!     /** Returns a handle, or 0 on failure — see lastError(). */
//!     external fun open(port: UsbSerialPort, connection: UsbDeviceConnection): Long
//!     external fun firmwareVersion(handle: Long): String?
//!     /** One ISO-TP request/response, e.g. byteArrayOf(0x21, 0x43).
//!      *  extAddr is the extended/mixed-addressing byte, or -1 for normal
//!      *  addressing. */
//!     external fun request(handle: Long, txId: Int, rxId: Int, extAddr: Int,
//!                          req: ByteArray, timeoutMs: Int): ByteArray?
//!     external fun close(handle: Long)
//!     external fun lastError(): String
//! }
//! ```
//!
//! Every call is blocking, so drive them off the main thread (a coroutine on
//! `Dispatchers.IO`); a `21 43` exchange takes tens of milliseconds.
//!
//! The exported symbols encode the fully-qualified class name, so they are
//! `Java_dev_rmvci_Rmvci_*` and the Kotlin object must live in package
//! `dev.rmvci` and be named `Rmvci`. To use a different package, rename the
//! `#[unsafe(no_mangle)]` functions to match (`.` becomes `_`), or switch to
//! `RegisterNatives`, which sidesteps name mangling entirely.
//!
//! **Status: hardware-verified on a device.** A rooted moto g fast (Android
//! 11) drove the real Mini-VCI through this JNI path to the bench ECU —
//! `21 43` -> `61 43 7b 79` — via the `prius-hvac-android` app. Bring-up found
//! four boundary bugs now fixed here and in [`JniTransport`] (see BRINGUP.md).

use std::sync::Mutex;
use std::time::Duration;

use jni::JNIEnv;
use jni::objects::{JByteArray, JClass, JObject};
use jni::sys::{jbyteArray, jint, jlong, jstring};

use rmvci_core::transport::JniTransport;
use rmvci_core::{CanId, Device, DeviceConfig, FirmwareIsoTp};

static LAST_ERROR: Mutex<String> = Mutex::new(String::new());

fn set_err(msg: impl Into<String>) {
    let msg = msg.into();
    tracing::warn!(error = %msg, "rmvci-android");
    *LAST_ERROR.lock().unwrap() = msg;
}

/// What a handle points at. The `Device` keeps the session (and its actor
/// thread) alive; channels are opened per request so the app does not have
/// to model the adapter's single-filter-per-width rule.
struct Session {
    device: Device,
    /// Cached channel, keyed by the (tx, rx, ext_addr) it was opened for
    /// (ext_addr is -1 for normal addressing). Connecting and installing a
    /// filter costs two round trips, so an app polling one ECU should not pay
    /// them on every request.
    tp: Option<(u16, u16, i32, FirmwareIsoTp)>,
}

/// # Safety
/// `handle` must be a value previously returned by `open` and not yet closed.
unsafe fn session<'a>(handle: jlong) -> Option<&'a mut Session> {
    if handle == 0 {
        set_err("null handle");
        return None;
    }
    Some(unsafe { &mut *(handle as *mut Session) })
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_dev_rmvci_Rmvci_open(
    env: JNIEnv,
    _class: JClass,
    port: JObject,
    connection: JObject,
) -> jlong {
    let vm = match env.get_java_vm() {
        Ok(vm) => vm,
        Err(e) => {
            set_err(format!("get_java_vm: {e}"));
            return 0;
        }
    };
    // Global refs: the actor outlives this native call, so local refs would
    // dangle.
    let port = match env.new_global_ref(port) {
        Ok(r) => r,
        Err(e) => {
            set_err(format!("global ref for port: {e}"));
            return 0;
        }
    };
    let connection = env.new_global_ref(connection).ok();

    let io = JniTransport::new(vm, port, connection);
    match Device::open_transport(io, DeviceConfig::default()) {
        Ok(device) => Box::into_raw(Box::new(Session { device, tp: None })) as jlong,
        Err(e) => {
            set_err(format!("open: {e}"));
            0
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_dev_rmvci_Rmvci_firmwareVersion(
    env: JNIEnv,
    _class: JClass,
    handle: jlong,
) -> jstring {
    let Some(s) = (unsafe { session(handle) }) else {
        return JObject::null().into_raw();
    };
    match s.device.firmware_version() {
        Ok(v) => match env.new_string(v) {
            Ok(js) => js.into_raw(),
            Err(e) => {
                set_err(format!("new_string: {e}"));
                JObject::null().into_raw()
            }
        },
        Err(e) => {
            set_err(format!("firmware_version: {e}"));
            JObject::null().into_raw()
        }
    }
}

/// One ISO-TP request/response through the adapter's own ISO15765, which is
/// the right choice for short diagnostic requests (see the crate README on
/// picking a path).
#[unsafe(no_mangle)]
pub extern "system" fn Java_dev_rmvci_Rmvci_request<'a>(
    env: JNIEnv<'a>,
    _class: JClass<'a>,
    handle: jlong,
    tx_id: jint,
    rx_id: jint,
    ext_addr: jint,
    req: JByteArray<'a>,
    timeout_ms: jint,
) -> jbyteArray {
    let null = || JObject::null().into_raw();
    let Some(s) = (unsafe { session(handle) }) else {
        return null();
    };
    let request = match env.convert_byte_array(&req) {
        Ok(v) => v,
        Err(e) => {
            set_err(format!("convert request bytes: {e}"));
            return null();
        }
    };

    let (tx_raw, rx_raw) = (tx_id as u16, rx_id as u16);
    if !matches!(&s.tp, Some((t, r, e, _)) if *t == tx_raw && *r == rx_raw && *e == ext_addr) {
        // Different ECU/addressing (or first use): drop the old channel before
        // opening the new one — the adapter keeps one filter per identifier
        // width.
        s.tp = None;
        // ext_addr < 0 means normal addressing; 0..=255 selects extended/mixed.
        let opened = if (0..=255).contains(&ext_addr) {
            FirmwareIsoTp::with_ext_addr(&s.device, CanId::Std(tx_raw), CanId::Std(rx_raw), ext_addr as u8)
        } else {
            FirmwareIsoTp::new(&s.device, CanId::Std(tx_raw), CanId::Std(rx_raw))
        };
        match opened {
            Ok(tp) => s.tp = Some((tx_raw, rx_raw, ext_addr, tp)),
            Err(e) => {
                set_err(format!("open ISO15765 channel: {e}"));
                return null();
            }
        }
    }
    let tp = &mut s.tp.as_mut().unwrap().3;
    let reply = match tp.request(&request, Duration::from_millis(timeout_ms.max(1) as u64)) {
        Ok(r) => r,
        Err(e) => {
            set_err(format!("request: {e}"));
            return null();
        }
    };
    match env.byte_array_from_slice(&reply) {
        Ok(a) => a.into_raw(),
        Err(e) => {
            set_err(format!("reply bytes: {e}"));
            null()
        }
    }
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_dev_rmvci_Rmvci_close(_env: JNIEnv, _class: JClass, handle: jlong) {
    if handle == 0 {
        return;
    }
    let session = unsafe { Box::from_raw(handle as *mut Session) };
    let Session { device, tp } = *session;
    // Drop the cached ISO-TP channel *first*. `Device` is `Clone` and a
    // channel owns its own clone of the actor's sender, so as long as `tp`
    // lives the actor never sees the session go away — `device.close()` would
    // then block for its full timeout. Dropping `tp` here also runs its
    // best-effort per-channel disconnect while the actor is still up.
    drop(tp);
    // Now the last handle drops inside close(): the actor tears the session
    // down and releases the port promptly, so the app can immediately reopen.
    device.close();
}

#[unsafe(no_mangle)]
pub extern "system" fn Java_dev_rmvci_Rmvci_lastError(env: JNIEnv, _class: JClass) -> jstring {
    let msg = LAST_ERROR.lock().unwrap().clone();
    match env.new_string(msg) {
        Ok(js) => js.into_raw(),
        Err(_) => JObject::null().into_raw(),
    }
}
