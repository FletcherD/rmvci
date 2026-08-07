//! SAE J2534-1 PassThru API over `rmvci-core`, mirroring libMVCI's
//! `src/mvci.def` — the 14 exports Techstream and other J2534 tools load.
//!
//! `extern "system"` is stdcall on i686 Windows (the ABI Techstream actually
//! calls) and plain C everywhere else. `c_ulong` matches the C header's
//! `unsigned long` (4 bytes on Windows, 8 on LP64 Linux).
//!
//! M1 status: symbol table only — every export returns `ERR_NOT_SUPPORTED`
//! and touches nothing. Wired to the core session layer in M3.

use core::ffi::{c_char, c_long, c_ulong, c_void};

pub mod consts;

use consts::ERR_NOT_SUPPORTED;

/// J2534 `PASSTHRU_MSG`, byte-compatible with the C definition.
#[repr(C)]
pub struct PassthruMsg {
    pub protocol_id: u32,
    pub rx_status: u32,
    pub tx_flags: u32,
    pub timestamp: u32,
    pub data_size: u32,
    pub extra_data_index: u32,
    pub data: [u8; consts::PASSTHRU_MSG_DATA_SIZE],
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

#[unsafe(no_mangle)]
pub extern "system" fn PassThruOpen(_name: *const c_void, _device_id: *mut c_ulong) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruClose(_device_id: c_ulong) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruConnect(
    _device_id: c_ulong,
    _protocol_id: c_ulong,
    _flags: c_ulong,
    _baud_rate: c_ulong,
    _channel_id: *mut c_ulong,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruDisconnect(_channel_id: c_ulong) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruReadMsgs(
    _channel_id: c_ulong,
    _msg: *mut PassthruMsg,
    _num_msgs: *mut c_ulong,
    _timeout: c_ulong,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruWriteMsgs(
    _channel_id: c_ulong,
    _msg: *mut PassthruMsg,
    _num_msgs: *mut c_ulong,
    _time_interval: c_ulong,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruStartPeriodicMsg(
    _channel_id: c_ulong,
    _msg: *mut PassthruMsg,
    _msg_id: *mut c_ulong,
    _time_interval: c_ulong,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruStopPeriodicMsg(_channel_id: c_ulong, _msg_id: c_ulong) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruStartMsgFilter(
    _channel_id: c_ulong,
    _filter_type: c_ulong,
    _mask_msg: *mut PassthruMsg,
    _pattern_msg: *mut PassthruMsg,
    _flow_control_msg: *mut PassthruMsg,
    _filter_id: *mut c_ulong,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruStopMsgFilter(_channel_id: c_ulong, _filter_id: c_ulong) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruSetProgrammingVoltage(
    _device_id: c_ulong,
    _pin_number: c_ulong,
    _voltage: c_ulong,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruReadVersion(
    _device_id: c_ulong,
    _firmware_version: *mut c_char,
    _dll_version: *mut c_char,
    _api_version: *mut c_char,
) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruGetLastError(_error_description: *mut c_char) -> c_long {
    ERR_NOT_SUPPORTED
}

#[unsafe(no_mangle)]
pub extern "system" fn PassThruIoctl(
    _channel_id: c_ulong,
    _ioctl_id: c_ulong,
    _input: *mut c_void,
    _output: *mut c_void,
) -> c_long {
    ERR_NOT_SUPPORTED
}
