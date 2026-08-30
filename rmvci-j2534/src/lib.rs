//! SAE J2534-1 PassThru API over `rmvci-core`, mirroring libMVCI's
//! `src/mvci.def` — the 14 exports Techstream and other J2534 tools load.
//!
//! `extern "system"` is stdcall on i686 Windows (the ABI Techstream actually
//! calls) and plain C everywhere else. `c_ulong` matches the C header's
//! `unsigned long` (4 bytes on Windows, 8 on LP64 Linux).
//!
//! ID encoding mirrors the C shim: DeviceID = slot+1, ChannelID = slot+0x100,
//! filter IDs are the wire ids (0x000e7a00+n), so StopMsgFilter needs no
//! bookkeeping. One channel per device slot.

use core::ffi::{c_char, c_long, c_ulong, c_void};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use rmvci_core::{
    CanConfig, CanId, CodecError, Device, DeviceConfig, Error, IsoTp, IsoTpConfig, ProtocolId,
    RawChannel,
};

pub mod consts;
use consts::*;

/// J2534 `PASSTHRU_MSG`, byte-compatible with the C definition.
#[repr(C)]
pub struct PassthruMsg {
    pub protocol_id: u32,
    pub rx_status: u32,
    pub tx_flags: u32,
    pub timestamp: u32,
    pub data_size: u32,
    pub extra_data_index: u32,
    pub data: [u8; PASSTHRU_MSG_DATA_SIZE],
}

#[repr(C)]
pub struct SConfig {
    pub parameter: u32,
    pub value: u32,
}

#[repr(C)]
pub struct SConfigList {
    pub num_of_params: u32,
    pub config_ptr: *mut SConfig,
}

// ---- state ------------------------------------------------------------

const MAX_SLOTS: usize = 4;

struct Slot {
    device: Device,
    chan: Option<RawChannel>,
    /// Set instead of `chan` when the channel was opened with
    /// `RMVCI_HOST_ISOTP`: ISO15765 done host-side over raw CAN (protocol 5).
    /// The two are mutually exclusive — one channel per slot.
    host: Option<IsoTp>,
    next_id: u32,
}

static SLOTS: Mutex<[Option<Slot>; 4]> = Mutex::new([None, None, None, None]);
static LAST_ERROR: Mutex<String> = Mutex::new(String::new());

/// Test hook: replace the device constructor (e.g. with a MockTransport
/// device). Not part of the API.
#[doc(hidden)]
pub type DeviceFactory = Box<dyn Fn(Option<&str>) -> Result<Device, Error> + Send>;
#[doc(hidden)]
pub static DEVICE_FACTORY: Mutex<Option<DeviceFactory>> = Mutex::new(None);
#[doc(hidden)]
pub fn _set_device_factory(f: DeviceFactory) {
    *DEVICE_FACTORY.lock().unwrap() = Some(f);
}

fn set_err(msg: impl Into<String>) {
    *LAST_ERROR.lock().unwrap() = msg.into();
}

/// Epoch for message timestamps, started at the first `PassThruOpen`.
static TS_EPOCH: OnceLock<Instant> = OnceLock::new();

/// Receive timestamp for `PASSTHRU_MSG.Timestamp`, in microseconds relative to
/// the first device open. The firmware provides no timestamp, so this is a host
/// monotonic clock stamped when the shim hands the message back — the closest
/// available approximation. The 32-bit microsecond field wraps at ~71.6
/// minutes, which is the J2534 convention, so the truncation is expected.
fn timestamp_micros() -> u32 {
    TS_EPOCH.get_or_init(Instant::now).elapsed().as_micros() as u32
}

fn code_of(e: &Error) -> c_long {
    match e {
        // Firmware status bytes are already J2534 codes; pass them through.
        Error::Rejected { status, .. } => status.code() as c_long,
        Error::NoFilterInstalled { .. } | Error::BufferEmpty => ERR_BUFFER_EMPTY,
        Error::Timeout(_) => ERR_TIMEOUT,
        Error::Protocol(_) => ERR_INVALID_PROTOCOL_ID,
        Error::Transport(_) | Error::Wedged(_) | Error::Closed => ERR_DEVICE_NOT_CONNECTED,
        Error::Codec(CodecError::MsgTooLong(_)) => ERR_INVALID_MSG,
        _ => ERR_FAILED,
    }
}

fn fail(e: &Error) -> c_long {
    set_err(e.to_string());
    code_of(e)
}

/// Never unwind across the FFI boundary.
fn guard(f: impl FnOnce() -> c_long) -> c_long {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(code) => code,
        Err(_) => {
            set_err("internal panic");
            ERR_FAILED
        }
    }
}

fn slot_from_dev(id: c_ulong) -> Option<usize> {
    (1..=MAX_SLOTS as c_ulong).contains(&id).then(|| id as usize - 1)
}

fn slot_from_ch(id: c_ulong) -> Option<usize> {
    let base = 0x100 as c_ulong;
    (base..base + MAX_SLOTS as c_ulong).contains(&id).then(|| (id - base) as usize)
}

fn make_device(name: Option<&str>) -> Result<Device, Error> {
    if let Some(f) = DEVICE_FACTORY.lock().unwrap().as_ref() {
        return f(name);
    }
    Device::open_with(DeviceConfig { port: name.map(str::to_string), ..DeviceConfig::default() })
}

/// # Safety
/// `dst` must point to a caller-provided buffer of at least `s.len() + 1`
/// bytes (the J2534 spec sizes these at 80).
unsafe fn put_cstr(dst: *mut c_char, s: &str) {
    unsafe {
        std::ptr::copy_nonoverlapping(s.as_ptr(), dst.cast::<u8>(), s.len());
        *dst.add(s.len()) = 0;
    }
}

// ---- exports ----------------------------------------------------------

/// # Safety
/// `name` must be null or a NUL-terminated string; `device_id` must be null or writable. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruOpen(name: *const c_void, device_id: *mut c_ulong) -> c_long {
    guard(|| {
        if device_id.is_null() {
            set_err("NULL pDeviceID");
            return ERR_NULL_PARAMETER;
        }
        // Start the message-timestamp clock at the first open.
        TS_EPOCH.get_or_init(Instant::now);
        let name_owned = if name.is_null() {
            None
        } else {
            let s = unsafe { core::ffi::CStr::from_ptr(name.cast()) }.to_string_lossy();
            (!s.is_empty()).then(|| s.into_owned())
        };

        let mut slots = SLOTS.lock().unwrap();
        let Some(idx) = slots.iter().position(Option::is_none) else {
            set_err("no free device slot");
            return ERR_FAILED;
        };
        let device = match make_device(name_owned.as_deref()) {
            Ok(d) => d,
            Err(e) => {
                set_err(format!("device open failed: {e}"));
                return ERR_DEVICE_NOT_CONNECTED;
            }
        };
        slots[idx] = Some(Slot { device, chan: None, host: None, next_id: 1 });
        unsafe { *device_id = (idx + 1) as c_ulong };
        STATUS_NOERROR
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruClose(device_id: c_ulong) -> c_long {
    guard(|| {
        let mut slots = SLOTS.lock().unwrap();
        let Some(slot) = slot_from_dev(device_id).and_then(|i| slots[i].take()) else {
            set_err("invalid device id");
            return ERR_INVALID_DEVICE_ID;
        };
        // Drop order does the wire teardown: channel drop sends the
        // per-channel disconnect (0x08), the last Device handle drop closes
        // the encrypted session (0x02).
        drop(slot);
        STATUS_NOERROR
    })
}

/// # Safety
/// `channel_id` must be null or writable. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruConnect(
    device_id: c_ulong,
    protocol_id: c_ulong,
    flags: c_ulong,
    baud_rate: c_ulong,
    channel_id: *mut c_ulong,
) -> c_long {
    guard(|| {
        if channel_id.is_null() {
            set_err("NULL pChannelID");
            return ERR_NULL_PARAMETER;
        }
        let Ok(proto) = ProtocolId::try_from(protocol_id as u32) else {
            set_err(
                "unsupported protocol id (the adapter stops responding until reopened \
                 if one is sent)",
            );
            return ERR_INVALID_PROTOCOL_ID;
        };
        let flags = flags as u32;
        let host_mode = flags & RMVCI_HOST_ISOTP != 0;
        let mut slots = SLOTS.lock().unwrap();
        let Some(slot) = slot_from_dev(device_id).and_then(|i| slots[i].as_mut()) else {
            set_err("invalid device id");
            return ERR_INVALID_DEVICE_ID;
        };
        // One channel per device slot: any existing channel (raw or host) is
        // dropped first (the C shim silently overwrote its bookkeeping and
        // leaked the old channel in the firmware).
        slot.chan = None;
        slot.host = None;

        // Host-path ISO15765: run ISO-TP host-side over raw CAN (protocol 5).
        // Only meaningful for ISO15765; the vendor bit is stripped either way.
        if host_mode && proto == ProtocolId::Iso15765 {
            let can = CanConfig { bitrate: baud_rate as u32, flags: flags & !RMVCI_HOST_ISOTP };
            // Endpoints arrive later via the flow-control filter.
            let cfg = IsoTpConfig::new(CanId::Std(0), CanId::Std(0)).with_can(can);
            return match IsoTp::connect_deferred(&slot.device, cfg) {
                Ok(tp) => {
                    slot.host = Some(tp);
                    unsafe { *channel_id = (slot_from_dev(device_id).unwrap() + 0x100) as c_ulong };
                    STATUS_NOERROR
                }
                Err(e) => fail(&e),
            };
        }

        match slot.device.connect_raw(proto, flags & !RMVCI_HOST_ISOTP, baud_rate as u32) {
            Ok(chan) => {
                slot.chan = Some(chan);
                unsafe { *channel_id = (slot_from_dev(device_id).unwrap() + 0x100) as c_ulong };
                STATUS_NOERROR
            }
            Err(e) => fail(&e),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruDisconnect(channel_id: c_ulong) -> c_long {
    guard(|| {
        let mut slots = SLOTS.lock().unwrap();
        let slot = slot_from_ch(channel_id).and_then(|i| slots[i].as_mut());
        match slot {
            Some(slot) if slot.chan.is_some() || slot.host.is_some() => {
                // 0x08 on the wire for either channel kind; the session stays up.
                drop(slot.chan.take());
                drop(slot.host.take());
                STATUS_NOERROR
            }
            _ => {
                set_err("invalid channel id");
                ERR_INVALID_CHANNEL_ID
            }
        }
    })
}

fn with_chan(channel_id: c_ulong, f: impl FnOnce(&mut Slot) -> c_long) -> c_long {
    let mut slots = SLOTS.lock().unwrap();
    match slot_from_ch(channel_id).and_then(|i| slots[i].as_mut()) {
        Some(slot) if slot.chan.is_some() => f(slot),
        _ => {
            set_err("invalid channel id");
            ERR_INVALID_CHANNEL_ID
        }
    }
}

/// Run `f` against a host-path ISO-TP channel. Returns `Some(code)` when the
/// channel is host-mode (handled), `None` when it is not — so callers fall
/// through to the firmware/raw path.
fn with_host(channel_id: c_ulong, f: impl FnOnce(&mut IsoTp) -> c_long) -> Option<c_long> {
    let mut slots = SLOTS.lock().unwrap();
    slot_from_ch(channel_id)
        .and_then(|i| slots[i].as_mut())
        .and_then(|slot| slot.host.as_mut())
        .map(f)
}

/// Shared body for the K-line init IOCTLs (FAST_INIT / FIVE_BAUD_INIT): read
/// the init message from `input`, run `run`, and write the ECU key bytes back
/// into the `output` PASSTHRU_MSG.
///
/// # Safety
/// `input`/`output` must each be null or a valid `PASSTHRU_MSG`.
unsafe fn kline_init(
    slot: &mut Slot,
    input: *mut c_void,
    output: *mut c_void,
    run: impl FnOnce(&mut RawChannel, &[u8]) -> Result<Vec<u8>, Error>,
) -> c_long {
    let inp = input.cast::<PassthruMsg>();
    if inp.is_null() || unsafe { (*inp).data_size } == 0 {
        set_err("NULL/empty init message");
        return ERR_NULL_PARAMETER;
    }
    let inp = unsafe { &*inp };
    let n = (inp.data_size as usize).min(inp.data.len());
    match run(slot.chan.as_mut().unwrap(), &inp.data[..n]) {
        Ok(reply) => {
            let outp = output.cast::<PassthruMsg>();
            if !outp.is_null() {
                let o = unsafe { &mut *outp };
                o.protocol_id = slot.chan.as_ref().unwrap().proto().wire();
                o.rx_status = 0;
                o.tx_flags = 0;
                o.timestamp = 0;
                let k = reply.len().min(o.data.len());
                o.data[..k].copy_from_slice(&reply[..k]);
                o.data_size = k as u32;
                o.extra_data_index = k as u32;
            }
            STATUS_NOERROR
        }
        Err(e) => {
            set_err(format!("init failed: {e}"));
            ERR_FAILED
        }
    }
}

/// # Safety
/// Message pointers must be null or valid `PASSTHRU_MSG`s; `filter_id` writable. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruStartMsgFilter(
    channel_id: c_ulong,
    filter_type: c_ulong,
    mask_msg: *mut PassthruMsg,
    pattern_msg: *mut PassthruMsg,
    flow_control_msg: *mut PassthruMsg,
    filter_id: *mut c_ulong,
) -> c_long {
    guard(|| {
        if mask_msg.is_null() || pattern_msg.is_null() || filter_id.is_null() {
            set_err("NULL filter msg");
            return ERR_NULL_PARAMETER;
        }
        let ftype = filter_type as u32;
        if !(PASS_FILTER..=FLOW_CONTROL_FILTER).contains(&ftype) {
            set_err("bad filter type");
            return ERR_INVALID_MSG;
        }
        // J2534: a flow-control message accompanies FLOW_CONTROL_FILTER and
        // only it.
        if ftype == FLOW_CONTROL_FILTER && flow_control_msg.is_null() {
            set_err("FLOW_CONTROL_FILTER needs pFlowControlMsg");
            return ERR_NULL_PARAMETER;
        }
        if ftype != FLOW_CONTROL_FILTER && !flow_control_msg.is_null() {
            set_err("pFlowControlMsg only valid for FLOW_CONTROL_FILTER");
            return ERR_INVALID_MSG;
        }

        // Host-path ISO15765: the flow-control filter carries the endpoints
        // (pattern = rx id, flow = tx id), which we hand to the host client.
        if let Some(code) = with_host(channel_id, |tp| {
            if ftype != FLOW_CONTROL_FILTER {
                set_err("host-path ISO15765 needs a FLOW_CONTROL filter");
                return ERR_INVALID_MSG;
            }
            let pattern = unsafe { &*pattern_msg };
            let flow = unsafe { &*flow_control_msg }; // non-null: checked for FC above
            if pattern.data_size < 4 || flow.data_size < 4 {
                set_err("filter identifier shorter than 4 bytes");
                return ERR_INVALID_MSG;
            }
            let ext29 = flow.tx_flags & CAN_29BIT_ID != 0;
            let rx = CanId::from_wire(pattern.data[..4].try_into().unwrap(), ext29);
            let tx = CanId::from_wire(flow.data[..4].try_into().unwrap(), ext29);
            let ext_addr = (flow.tx_flags & ISO15765_ADDR_TYPE != 0
                && flow.data_size >= 5)
                .then_some(flow.data[4]);
            match tp.set_endpoints(tx, rx, ext_addr) {
                Ok(()) => {
                    // One filter per host channel; a fixed handle is enough.
                    unsafe { *filter_id = 0x000e_7c00 };
                    STATUS_NOERROR
                }
                Err(e) => fail(&e),
            }
        }) {
            return code;
        }

        with_chan(channel_id, |slot| {
            let chan = slot.chan.as_mut().unwrap();
            let mask = unsafe { &*mask_msg };
            let pattern = unsafe { &*pattern_msg };
            let flow = (!flow_control_msg.is_null()).then(|| unsafe { &*flow_control_msg });

            // Identifier width: 1 byte on K-line, a 4-byte CAN id otherwise,
            // 5 when the flow-control message asks for extended addressing.
            let idlen: usize = if chan.proto().is_can() {
                match flow {
                    Some(f) if f.tx_flags & ISO15765_ADDR_TYPE != 0 => 5,
                    _ => 4,
                }
            } else {
                1
            };
            if (mask.data_size as usize) < idlen
                || (pattern.data_size as usize) < idlen
                || flow.is_some_and(|f| (f.data_size as usize) < idlen)
            {
                set_err("filter message shorter than the identifier width");
                return ERR_INVALID_MSG;
            }

            // The J2534 handle IS the wire id, so StopMsgFilter needs no map.
            let wire_id = 0x000e_7a00u32 + (slot.next_id & 0xff);
            slot.next_id += 1;

            match chan.start_filter(
                wire_id,
                ftype,
                &mask.data[..idlen],
                &pattern.data[..idlen],
                flow.map(|f| &f.data[..idlen]),
                idlen,
            ) {
                Ok(()) => {
                    unsafe { *filter_id = wire_id as c_ulong };
                    STATUS_NOERROR
                }
                Err(e) => {
                    set_err(format!("start filter rejected: {e}"));
                    code_of(&e)
                }
            }
        })
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruStopMsgFilter(channel_id: c_ulong, filter_id: c_ulong) -> c_long {
    guard(|| {
        // A host-path channel has one implicit filter; stopping it is a no-op
        // (the endpoints persist until the channel is reconfigured or dropped).
        if let Some(code) = with_host(channel_id, |_| STATUS_NOERROR) {
            return code;
        }
        with_chan(channel_id, |slot| {
            match slot.chan.as_mut().unwrap().stop_filter(filter_id as u32) {
                Ok(()) => STATUS_NOERROR,
                Err(e) => {
                    set_err(format!("no such filter: {e}"));
                    ERR_INVALID_FILTER_ID
                }
            }
        })
    })
}

/// # Safety
/// `msg` must point to `*num_msgs` valid `PASSTHRU_MSG`s; `num_msgs` writable. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruWriteMsgs(
    channel_id: c_ulong,
    msg: *mut PassthruMsg,
    num_msgs: *mut c_ulong,
    _time_interval: c_ulong,
) -> c_long {
    guard(|| {
        if msg.is_null() || num_msgs.is_null() || unsafe { *num_msgs } == 0 {
            set_err("NULL/empty msg");
            return ERR_NULL_PARAMETER;
        }
        // Host-path ISO15765: each message is [4-byte id][ISO-TP payload]; the
        // host client segments it (correct FF_DL, honored BS/STmin) to the tx
        // id set by the flow-control filter.
        if let Some(code) = with_host(channel_id, |tp| {
            let want = unsafe { *num_msgs } as usize;
            let msgs = unsafe { std::slice::from_raw_parts(msg, want) };
            for (sent, m) in msgs.iter().enumerate() {
                let size = (m.data_size as usize).min(m.data.len());
                if size < 4 {
                    unsafe { *num_msgs = sent as c_ulong };
                    set_err("ISO15765 message shorter than a 4-byte id");
                    return ERR_INVALID_MSG;
                }
                if let Err(e) = tp.send(&m.data[4..size]) {
                    unsafe { *num_msgs = sent as c_ulong };
                    set_err(format!("host write rejected: {e}"));
                    return match e {
                        Error::Codec(_) | Error::IsoTp(_) => ERR_INVALID_MSG,
                        other => code_of(&other),
                    };
                }
            }
            unsafe { *num_msgs = want as c_ulong };
            STATUS_NOERROR
        }) {
            return code;
        }
        with_chan(channel_id, |slot| {
            let chan = slot.chan.as_mut().unwrap();
            let want = unsafe { *num_msgs } as usize;
            let msgs = unsafe { std::slice::from_raw_parts(msg, want) };
            for (sent, m) in msgs.iter().enumerate() {
                let size = m.data_size as usize;
                if size > m.data.len() {
                    unsafe { *num_msgs = sent as c_ulong };
                    set_err("DataSize exceeds the message buffer");
                    return ERR_INVALID_MSG;
                }
                // TxFlags reaches the adapter: 29-bit ids, frame padding and
                // extended addressing are selected per message.
                if let Err(e) = chan.write(m.tx_flags, &m.data[..size]) {
                    unsafe { *num_msgs = sent as c_ulong };
                    set_err(format!("write rejected: {e}"));
                    return match e {
                        Error::Codec(_) => ERR_INVALID_MSG,
                        other => code_of(&other),
                    };
                }
            }
            unsafe { *num_msgs = want as c_ulong };
            STATUS_NOERROR
        })
    })
}

/// # Safety
/// `msg` must point to `*num_msgs` writable `PASSTHRU_MSG`s; `num_msgs` writable. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruReadMsgs(
    channel_id: c_ulong,
    msg: *mut PassthruMsg,
    num_msgs: *mut c_ulong,
    timeout: c_ulong,
) -> c_long {
    guard(|| {
        if msg.is_null() || num_msgs.is_null() || unsafe { *num_msgs } == 0 {
            set_err("NULL/empty msg");
            return ERR_NULL_PARAMETER;
        }
        // Host-path ISO15765: reassemble whole messages and hand them back as
        // [4-byte rx id][payload], matching the firmware path's framing.
        if let Some(code) = with_host(channel_id, |tp| {
            let want = unsafe { *num_msgs } as usize;
            let out = unsafe { std::slice::from_raw_parts_mut(msg, want) };
            #[allow(clippy::useless_conversion)]
            let timeout_ms: u64 = timeout.max(1).into();
            let deadline = Instant::now() + Duration::from_millis(timeout_ms);
            let rx_wire = tp.rx_id().to_wire();
            let mut got = 0usize;
            while got < want {
                let remaining = deadline.saturating_duration_since(Instant::now());
                match tp.recv(remaining) {
                    Ok(payload) => {
                        let o = &mut out[got];
                        let n = (4 + payload.len()).min(o.data.len());
                        o.data[..4.min(n)].copy_from_slice(&rx_wire[..4.min(n)]);
                        if n > 4 {
                            o.data[4..n].copy_from_slice(&payload[..n - 4]);
                        }
                        o.protocol_id = ProtocolId::Iso15765.wire();
                        o.rx_status = 0;
                        o.tx_flags = 0;
                        o.timestamp = timestamp_micros();
                        o.data_size = n as u32;
                        o.extra_data_index = n as u32;
                        got += 1;
                    }
                    Err(Error::Timeout(_)) => break,
                    Err(e) => {
                        unsafe { *num_msgs = got as c_ulong };
                        return fail(&e);
                    }
                }
            }
            unsafe { *num_msgs = got as c_ulong };
            if got == 0 {
                set_err("no message");
                return ERR_BUFFER_EMPTY;
            }
            STATUS_NOERROR
        }) {
            return code;
        }
        with_chan(channel_id, |slot| {
            let chan = slot.chan.as_mut().unwrap();
            let want = unsafe { *num_msgs } as usize;
            let out = unsafe { std::slice::from_raw_parts_mut(msg, want) };
            // c_ulong is u32 on Windows, u64 on LP64 Linux; identity there.
            #[allow(clippy::useless_conversion)]
            let timeout_ms: u64 = timeout.max(1).into();
            let deadline = Instant::now() + Duration::from_millis(timeout_ms);
            let mut got = 0usize;

            while got < want {
                match chan.poll(Duration::from_millis(200)) {
                    Ok(Some(m)) => {
                        let o = &mut out[got];
                        let n = m.data.len().min(o.data.len());
                        o.data[..n].copy_from_slice(&m.data[..n]);
                        o.protocol_id = chan.proto().wire();
                        o.rx_status = m.rx_status.bits(); // device bits match J2534
                        o.tx_flags = 0;
                        o.timestamp = timestamp_micros();
                        o.data_size = n as u32;
                        o.extra_data_index = n as u32;
                        got += 1;
                        continue;
                    }
                    Ok(None) => {}
                    Err(e) => {
                        unsafe { *num_msgs = got as c_ulong };
                        return fail(&e);
                    }
                }
                if Instant::now() >= deadline {
                    break;
                }
                std::thread::sleep(Duration::from_millis(5));
            }

            unsafe { *num_msgs = got as c_ulong };
            if got == 0 {
                // The adapter's acceptance filter starts empty and enabled:
                // a channel with no filter installed receives nothing at
                // all. Say so — otherwise this looks exactly like an
                // unresponsive vehicle.
                if chan.filters_installed() == 0 {
                    set_err(
                        "no message (no filter installed; this adapter receives nothing \
                         until PassThruStartMsgFilter is called)",
                    );
                } else {
                    set_err("no message");
                }
                return ERR_BUFFER_EMPTY;
            }
            STATUS_NOERROR
        })
    })
}

/// # Safety
/// `input`/`output` must match the shape the J2534 spec defines for `ioctl_id`. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruIoctl(
    channel_id: c_ulong,
    ioctl_id: c_ulong,
    input: *mut c_void,
    output: *mut c_void,
) -> c_long {
    guard(|| match ioctl_id as u32 {
        SET_CONFIG => with_chan(channel_id, |slot| {
            let list = input.cast::<SConfigList>();
            if list.is_null() {
                set_err("NULL SCONFIG_LIST");
                return ERR_NULL_PARAMETER;
            }
            let list = unsafe { &*list };
            if list.num_of_params > 0 && list.config_ptr.is_null() {
                set_err("NULL ConfigPtr");
                return ERR_NULL_PARAMETER;
            }
            let params =
                unsafe { std::slice::from_raw_parts(list.config_ptr, list.num_of_params as usize) };
            // The firmware takes one parameter per SET_CONFIG command, so
            // the list is fanned out into individual wire calls.
            for p in params {
                if let Err(e) = slot.chan.as_mut().unwrap().set_config(p.parameter, p.value) {
                    set_err(format!("set_config({}) failed: {e}", p.parameter));
                    return code_of(&e);
                }
            }
            STATUS_NOERROR
        }),
        GET_CONFIG => with_chan(channel_id, |slot| {
            let list = input.cast::<SConfigList>();
            if list.is_null() {
                set_err("NULL SCONFIG_LIST");
                return ERR_NULL_PARAMETER;
            }
            let list = unsafe { &*list };
            if list.num_of_params > 0 && list.config_ptr.is_null() {
                set_err("NULL ConfigPtr");
                return ERR_NULL_PARAMETER;
            }
            // GET_CONFIG fills the Value fields of the caller's SCONFIG_LIST
            // in place; one wire call per parameter (the firmware has no list
            // form). NOTE: the reply layout is RE-derived, not yet confirmed
            // on hardware — see inner::parse_get_config.
            let params = unsafe {
                std::slice::from_raw_parts_mut(list.config_ptr, list.num_of_params as usize)
            };
            for p in params {
                match slot.chan.as_mut().unwrap().get_config(p.parameter) {
                    Ok(v) => p.value = v,
                    Err(e) => {
                        set_err(format!("get_config({}) failed: {e}", p.parameter));
                        return code_of(&e);
                    }
                }
            }
            STATUS_NOERROR
        }),
        READ_VBATT => with_chan(channel_id, |slot| {
            let out = output.cast::<c_ulong>();
            if out.is_null() {
                set_err("NULL READ_VBATT output");
                return ERR_NULL_PARAMETER;
            }
            match slot.chan.as_mut().unwrap().read_vbatt() {
                Ok(mv) => {
                    unsafe { *out = mv as c_ulong };
                    STATUS_NOERROR
                }
                Err(e) => fail(&e),
            }
        }),
        CLEAR_PERIODIC_MSGS => {
            with_chan(channel_id, |slot| match slot.chan.as_mut().unwrap().clear_periodic() {
                Ok(()) => STATUS_NOERROR,
                Err(e) => fail(&e),
            })
        }
        FAST_INIT => with_chan(channel_id, |slot| unsafe {
            kline_init(slot, input, output, |c, init| c.fast_init(init))
        }),
        FIVE_BAUD_INIT => with_chan(channel_id, |slot| unsafe {
            kline_init(slot, input, output, |c, init| c.five_baud_init(init))
        }),
        // Parity with the C shim (and the firmware, where CLEAR_TX_BUFFER is
        // itself a no-op that reports success).
        CLEAR_TX_BUFFER | CLEAR_RX_BUFFER | CLEAR_MSG_FILTERS => {
            with_chan(channel_id, |_| STATUS_NOERROR)
        }
        _ => {
            set_err("ioctl not supported");
            ERR_NOT_SUPPORTED
        }
    })
}

// ---- stubs / informational ---------------------------------------------

/// # Safety
/// `msg` must be null or a valid `PASSTHRU_MSG`; `msg_id` must be null or
/// writable. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruStartPeriodicMsg(
    channel_id: c_ulong,
    msg: *mut PassthruMsg,
    msg_id: *mut c_ulong,
    time_interval: c_ulong,
) -> c_long {
    guard(|| {
        if msg.is_null() || msg_id.is_null() {
            set_err("NULL periodic message/id");
            return ERR_NULL_PARAMETER;
        }
        with_chan(channel_id, |slot| {
            let m = unsafe { &*msg };
            let n = (m.data_size as usize).min(m.data.len());
            // The MsgID is host-assigned, like a filter handle; a distinct base
            // (0x000e7b00) keeps periodic handles from colliding visually with
            // filter handles (0x000e7a00). StopPeriodicMsg passes it straight
            // back, so no id map is needed.
            let wire_id = 0x000e_7b00u32 + (slot.next_id & 0xff);
            slot.next_id += 1;
            match slot.chan.as_mut().unwrap().start_periodic(
                wire_id,
                m.tx_flags,
                time_interval as u32,
                &m.data[..n],
            ) {
                Ok(()) => {
                    unsafe { *msg_id = wire_id as c_ulong };
                    STATUS_NOERROR
                }
                Err(e) => fail(&e),
            }
        })
    })
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruStopPeriodicMsg(channel_id: c_ulong, msg_id: c_ulong) -> c_long {
    guard(|| {
        with_chan(channel_id, |slot| {
            match slot.chan.as_mut().unwrap().stop_periodic(msg_id as u32) {
                Ok(()) => STATUS_NOERROR,
                Err(e) => {
                    set_err(format!("no such periodic msg: {e}"));
                    code_of(&e)
                }
            }
        })
    })
}

/// Passthrough to the firmware's SET_PROGRAMMING_VOLTAGE (0x0D). On the
/// Mini-VCI the firmware stores the value and drives no pin (a no-op that
/// reports success); cables whose firmware implements the opcode act on it.
#[unsafe(no_mangle)]
pub extern "system" fn PassThruSetProgrammingVoltage(
    device_id: c_ulong,
    pin_number: c_ulong,
    voltage: c_ulong,
) -> c_long {
    guard(|| {
        let mut slots = SLOTS.lock().unwrap();
        match slot_from_dev(device_id).and_then(|i| slots[i].as_mut()) {
            Some(slot) => {
                match slot.device.set_programming_voltage(pin_number as u32, voltage as u32) {
                    Ok(()) => STATUS_NOERROR,
                    Err(e) => fail(&e),
                }
            }
            None => {
                set_err("invalid device id");
                ERR_INVALID_DEVICE_ID
            }
        }
    })
}

/// # Safety
/// Non-null string pointers must have >= 80 writable bytes. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruReadVersion(
    device_id: c_ulong,
    firmware_version: *mut c_char,
    dll_version: *mut c_char,
    api_version: *mut c_char,
) -> c_long {
    guard(|| {
        let slots = SLOTS.lock().unwrap();
        if slot_from_dev(device_id).and_then(|i| slots[i].as_ref()).is_none() {
            return ERR_INVALID_DEVICE_ID;
        }
        // Report the same identity as the original Mini-VCI DLL so the host
        // app (e.g. Toyota Techstream) recognises the adapter. Overridable
        // for tools that should see the real driver name.
        let fw = std::env::var("RMVCI_VERSION_FW").unwrap_or_else(|_| "J2534 MINIV1.03".into());
        let dll =
            std::env::var("RMVCI_VERSION_DLL").unwrap_or_else(|_| "MVCI J2534 DLL v1.4.6".into());
        let api = std::env::var("RMVCI_VERSION_API").unwrap_or_else(|_| "04.04".into());
        unsafe {
            if !firmware_version.is_null() {
                put_cstr(firmware_version, &fw);
            }
            if !dll_version.is_null() {
                put_cstr(dll_version, &dll);
            }
            if !api_version.is_null() {
                put_cstr(api_version, &api);
            }
        }
        STATUS_NOERROR
    })
}

/// # Safety
/// `error_description` must be null or have >= 80 writable bytes. Pointer validity per SAE J2534-1.
#[unsafe(no_mangle)]
pub unsafe extern "system" fn PassThruGetLastError(error_description: *mut c_char) -> c_long {
    guard(|| {
        if error_description.is_null() {
            return ERR_NULL_PARAMETER;
        }
        let msg = LAST_ERROR.lock().unwrap();
        // J2534 sizes the caller's buffer at 80 bytes.
        let truncated: String = msg.chars().take(79).collect();
        unsafe { put_cstr(error_description, &truncated) };
        STATUS_NOERROR
    })
}
