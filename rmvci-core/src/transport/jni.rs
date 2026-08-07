//! Android backend: a [`Transport`] whose byte I/O is performed by Java.
//!
//! On Android an app cannot touch a USB device from native code — permission
//! is granted to the *app*, through `UsbManager`, and the resulting handle
//! lives on the Java side. So instead of doing I/O itself, this transport
//! calls back **up** into a Java object for every operation. Everything above
//! it — codec, actor, channels, both ISO-TP paths — is unchanged.
//!
//! The expected Java object is a `UsbSerialPort` from
//! [usb-serial-for-android](https://github.com/mik3y/usb-serial-for-android),
//! whose `FtdiSerialDriver` recognises the cable's stock FT232R
//! (`0403:6001`) with the default prober and already handles the FTDI packet
//! framing, including the 2-byte status header.
//!
//! Kotlin side, once:
//!
//! ```kotlin
//! val manager = getSystemService(Context.USB_SERVICE) as UsbManager
//! val driver  = UsbSerialProber.getDefaultProber().findAllDrivers(manager).first()
//! // ... UsbManager.requestPermission(driver.device, ...) and wait for the grant
//! val connection = manager.openDevice(driver.device)
//! val port = driver.ports[0]
//! port.open(connection)
//! port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
//! ```
//!
//! then hand `port` and `connection` to the native side (see `rmvci-android`).
//!
//! ## Threading
//!
//! This is the subtle part. A JNI environment pointer is **per thread**, and
//! the session actor runs on a thread *Rust* spawned, which the JVM knows
//! nothing about. The transport is constructed on the caller's thread and
//! then moved into the actor thread, so it must not capture an environment at
//! construction — that environment would belong to the wrong thread, which
//! is undefined behaviour of the hard-to-debug kind.
//!
//! What it stores instead is a [`JavaVM`] and [`GlobalRef`]s, all of which
//! are genuinely `Send + Sync`, and it obtains an environment per call by
//! attaching the current thread *as a daemon* — idempotent, cheap once
//! attached, and daemon so a native thread that never detaches does not keep
//! the JVM alive. The per-call cost is roughly a microsecond against
//! exchanges that take tens of milliseconds.

use std::time::Duration;

use jni::objects::{GlobalRef, JObject, JPrimitiveArray, JValue};
use jni::{JNIEnv, JavaVM};

use super::{LatencyResult, Transport};
use crate::error::TransportError;

/// FTDI vendor request to set the latency timer, issued through
/// `UsbDeviceConnection.controlTransfer`.
const SIO_SET_LATENCY_TIMER: i32 = 9;
const FTDI_VENDOR_OUT: i32 = 0x40;
const TARGET_LATENCY_MS: u8 = 2;

fn jerr(e: impl std::fmt::Display, what: &str) -> TransportError {
    TransportError::Open { port: what.to_string(), reason: e.to_string() }
}

pub struct JniTransport {
    vm: JavaVM,
    /// The Java `UsbSerialPort`. A global ref, not the local one JNI hands
    /// out: local refs die when the originating native call returns, and the
    /// actor outlives that by design.
    port: GlobalRef,
    /// Optional `UsbDeviceConnection`, used only for the latency-timer
    /// control transfer, which `UsbSerialPort` does not expose.
    connection: Option<GlobalRef>,
}

impl JniTransport {
    /// `port` is a `com.hoho.android.usbserial.driver.UsbSerialPort` that is
    /// already open and configured for 115200 8N1. `connection` is the
    /// `android.hardware.usb.UsbDeviceConnection` it was opened with; pass it
    /// to make [`Transport::optimize_latency`] work.
    pub fn new(vm: JavaVM, port: GlobalRef, connection: Option<GlobalRef>) -> Self {
        Self { vm, port, connection }
    }

    /// An environment for *this* thread. See the module note on threading.
    fn env(&self) -> Result<JNIEnv<'_>, TransportError> {
        self.vm.attach_current_thread_as_daemon().map_err(|e| jerr(e, "attach thread to JVM"))
    }
}

impl Transport for JniTransport {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        let mut env = self.env()?;
        let arr = env.byte_array_from_slice(buf).map_err(|e| jerr(e, "write: alloc"))?;
        // void write(byte[] src, int timeout)
        env.call_method(&self.port, "write", "([BI)V", &[JValue::Object(&arr), JValue::Int(2000)])
            .map_err(|e| jerr(e, "write"))?;
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize, TransportError> {
        let mut env = self.env()?;
        let len = buf.len() as i32;
        // Java's byte is signed; the memory layout matches Rust's u8 exactly.
        let arr: JPrimitiveArray<i8> =
            env.new_byte_array(len).map_err(|e| jerr(e, "read: alloc"))?;
        let ms = timeout.as_millis().min(i32::MAX as u128) as i32;
        // int read(byte[] dest, int timeout) — bytes read, 0 on timeout.
        let n = env
            .call_method(&self.port, "read", "([BI)I", &[JValue::Object(&arr), JValue::Int(ms)])
            .and_then(|v| v.i())
            .map_err(|e| jerr(e, "read"))?;
        if n <= 0 {
            return Ok(0);
        }
        let n = (n as usize).min(buf.len());
        let dst = unsafe { std::slice::from_raw_parts_mut(buf.as_mut_ptr().cast::<i8>(), n) };
        env.get_byte_array_region(&arr, 0, dst).map_err(|e| jerr(e, "read: copy"))?;
        Ok(n)
    }

    fn purge_rx(&mut self) -> Result<(), TransportError> {
        let mut env = self.env()?;
        // void purgeHwBuffers(boolean purgeRead, boolean purgeWrite)
        match env.call_method(
            &self.port,
            "purgeHwBuffers",
            "(ZZ)V",
            &[JValue::Bool(1), JValue::Bool(0)],
        ) {
            Ok(_) => Ok(()),
            // Not every driver implements it. The session layer only uses
            // purge to resynchronise, so an unsupported one is not fatal —
            // but the pending exception must be cleared or the next JNI call
            // fails for unrelated reasons.
            Err(e) => {
                let _ = env.exception_clear();
                tracing::debug!(error = %e, "purgeHwBuffers unavailable on this driver");
                Ok(())
            }
        }
    }

    fn set_modem(&mut self, dtr: bool, rts: bool) -> Result<(), TransportError> {
        let mut env = self.env()?;
        for (method, value) in [("setDTR", dtr), ("setRTS", rts)] {
            env.call_method(&self.port, method, "(Z)V", &[JValue::Bool(value as u8)])
                .map_err(|e| jerr(e, method))?;
        }
        Ok(())
    }

    /// `UsbSerialPort` does not expose the FTDI latency timer, but the
    /// underlying `UsbDeviceConnection` does expose raw control transfers, so
    /// the vendor request is still reachable — and on Android there is no
    /// sysfs alternative at all.
    fn optimize_latency(&mut self) -> LatencyResult {
        let Some(conn) = self.connection.clone() else {
            return LatencyResult::Unavailable;
        };
        let mut env = match self.env() {
            Ok(e) => e,
            Err(e) => return LatencyResult::Failed { reason: e.to_string() },
        };
        // int controlTransfer(int requestType, int request, int value,
        //                     int index, byte[] buffer, int length, int timeout)
        let res = env.call_method(
            &conn,
            "controlTransfer",
            "(IIII[BII)I",
            &[
                JValue::Int(FTDI_VENDOR_OUT),
                JValue::Int(SIO_SET_LATENCY_TIMER),
                JValue::Int(TARGET_LATENCY_MS as i32),
                JValue::Int(0),
                JValue::Object(&JObject::null()),
                JValue::Int(0),
                JValue::Int(1000),
            ],
        );
        match res.and_then(|v| v.i()) {
            Ok(n) if n >= 0 => LatencyResult::Set { millis: TARGET_LATENCY_MS },
            Ok(n) => LatencyResult::Failed { reason: format!("controlTransfer returned {n}") },
            Err(e) => {
                let _ = env.exception_clear();
                LatencyResult::Failed { reason: e.to_string() }
            }
        }
    }
}
