//! Reply parsers read length fields the adapter controls. A hostile or
//! corrupt ILEN must not index out of bounds.
#![no_main]

use libfuzzer_sys::fuzz_target;
use rmvci_core::codec::inner;
use rmvci_core::types::Cmd;

fuzz_target!(|data: &[u8]| {
    let _ = inner::parse_read_reply(data);
    let _ = inner::parse_challenge(data);
    for cmd in [Cmd::Connect, Cmd::WriteMsg, Cmd::StartMsgFilter, Cmd::Ioctl] {
        let _ = inner::status_of(data, cmd);
    }
});
