//! Reply parsers read length fields the adapter controls. A hostile or
//! corrupt ILEN must not index out of bounds.
#![no_main]

use libfuzzer_sys::fuzz_target;
use rmvci_core::codec::inner;
use rmvci_core::types::Cmd;

fuzz_target!(|data: &[u8]| {
    let _ = inner::parse_read_reply(data);
    let _ = inner::parse_challenge(data);
    // The IOCTL read-back parsers index a length-driven tail the adapter
    // controls; a short or hostile reply must not panic.
    let _ = inner::parse_get_config(data);
    let _ = inner::parse_vbatt(data);
    for cmd in [Cmd::Connect, Cmd::WriteMsg, Cmd::StartMsgFilter, Cmd::Ioctl] {
        let _ = inner::status_of(data, cmd);
    }
});
