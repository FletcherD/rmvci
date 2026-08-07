//! Load the built cdylib the way a J2534 host application would (dlopen +
//! dlsym) and drive the exports that don't need hardware. This is the proof
//! that the symbol table, ABI and calling convention are right — the
//! in-process tests link the Rust lib and would not catch a missing or
//! mangled export.

use core::ffi::{c_char, c_long, c_ulong};

fn cdylib_path() -> std::path::PathBuf {
    // tests run from target/<profile>/deps/<exe>; the cdylib sits one up.
    let exe = std::env::current_exe().unwrap();
    let dir = exe.parent().unwrap().parent().unwrap();
    let p = dir.join("librmvci_j2534.so");
    assert!(p.exists(), "cdylib not found at {p:?} — build the workspace first");
    p
}

#[test]
fn dlopen_resolves_and_calls_all_exports() {
    let lib = unsafe { libloading::Library::new(cdylib_path()) }.expect("dlopen");

    const EXPORTS: [&str; 14] = [
        "PassThruOpen",
        "PassThruClose",
        "PassThruConnect",
        "PassThruDisconnect",
        "PassThruReadMsgs",
        "PassThruWriteMsgs",
        "PassThruStartPeriodicMsg",
        "PassThruStopPeriodicMsg",
        "PassThruStartMsgFilter",
        "PassThruStopMsgFilter",
        "PassThruSetProgrammingVoltage",
        "PassThruReadVersion",
        "PassThruGetLastError",
        "PassThruIoctl",
    ];
    for name in EXPORTS {
        let sym: libloading::Symbol<*const ()> =
            unsafe { lib.get(name.as_bytes()) }.unwrap_or_else(|e| panic!("missing {name}: {e}"));
        assert!(!sym.is_null());
    }

    // Calls that must work with no hardware attached:
    unsafe {
        let read_version: libloading::Symbol<
            unsafe extern "system" fn(c_ulong, *mut c_char, *mut c_char, *mut c_char) -> c_long,
        > = lib.get(b"PassThruReadVersion").unwrap();
        // No device open in this process -> invalid device id.
        assert_eq!(
            read_version(1, std::ptr::null_mut(), std::ptr::null_mut(), std::ptr::null_mut()),
            0x1a // ERR_INVALID_DEVICE_ID
        );

        let get_last_error: libloading::Symbol<
            unsafe extern "system" fn(*mut c_char) -> c_long,
        > = lib.get(b"PassThruGetLastError").unwrap();
        assert_eq!(get_last_error(std::ptr::null_mut()), 0x04); // ERR_NULL_PARAMETER
        let mut buf = [0 as c_char; 80];
        assert_eq!(get_last_error(buf.as_mut_ptr()), 0x00);

        let stop_periodic: libloading::Symbol<
            unsafe extern "system" fn(c_ulong, c_ulong) -> c_long,
        > = lib.get(b"PassThruStopPeriodicMsg").unwrap();
        assert_eq!(stop_periodic(0x100, 1), 0x01); // ERR_NOT_SUPPORTED
    }
}
