//! Inner command builders and reply parsers.
//!
//! Inner command: `[ILEN u16 LE][CMD][ARGS...]`, `ILEN = 1 + len(ARGS)`.
//! Every channel-scoped command opens its ARGS with the ProtocolID — there is
//! no separate channel handle, the ProtocolID *is* the channel.
//!
//! Some builders emit a trailing pad byte beyond ILEN (the vendor DLL does the
//! same); the byte-exact captured vectors in `tests/captured_vectors.rs` are
//! the authority on each layout.

use crate::error::CodecError;
use crate::types::{Cmd, DeviceStatus, MAX_MSG, ProtocolId, RxStatus};

fn u32le(v: u32) -> [u8; 4] {
    v.to_le_bytes()
}

fn ilen(args: usize) -> [u8; 2] {
    ((1 + args) as u16).to_le_bytes()
}

/// Handshake identify payload — itself an inner command (`[ILEN=7][OPEN]"MVCI-T"`)
/// sent in plaintext. OPEN toggles encryption; it must be sent exactly once
/// per session, which the session layer's link type-state enforces.
pub fn identify() -> Vec<u8> {
    let mut v = Vec::with_capacity(9);
    v.extend_from_slice(&ilen(6));
    v.push(Cmd::Open as u8);
    v.extend_from_slice(b"MVCI-T");
    v
}

pub fn read_version() -> Vec<u8> {
    vec![0x01, 0x00, Cmd::ReadVersion as u8]
}

pub fn connect(proto: ProtocolId, flags: u32, baud: u32) -> Vec<u8> {
    let mut v = Vec::with_capacity(15);
    v.extend_from_slice(&ilen(12));
    v.push(Cmd::Connect as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(flags));
    v.extend_from_slice(&u32le(baud));
    v
}

pub fn disconnect(proto: ProtocolId) -> Vec<u8> {
    let mut v = Vec::with_capacity(8);
    v.extend_from_slice(&ilen(4));
    v.push(Cmd::Disconnect as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.push(0x00);
    v
}

pub fn session_close() -> Vec<u8> {
    vec![0x01, 0x00, Cmd::CloseSession as u8, 0, 0, 0, 0, 0]
}

/// ReadMsgs poll. Byte 3 is not a sub-function — it is the low byte of the
/// ProtocolID, so a poll only returns messages for that protocol.
pub fn read_poll(proto: ProtocolId) -> Vec<u8> {
    let mut v = Vec::with_capacity(8);
    v.extend_from_slice(&ilen(4));
    v.push(Cmd::ReadMsg as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.push(0x00);
    v
}

pub fn write_msg(proto: ProtocolId, txflags: u32, msg: &[u8]) -> Result<Vec<u8>, CodecError> {
    if msg.len() > MAX_MSG {
        return Err(CodecError::MsgTooLong(msg.len()));
    }
    let mut v = Vec::with_capacity(11 + msg.len());
    v.extend_from_slice(&ilen(8 + msg.len()));
    v.push(Cmd::WriteMsg as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(txflags));
    v.extend_from_slice(msg);
    Ok(v)
}

/// StartMsgFilter. `mask`/`pattern`/`flow` are `idlen` bytes each, copied
/// verbatim in wire order — for CAN that is the identifier MSB-first, exactly
/// as J2534 stores it in `PASSTHRU_MSG.Data`. The flow field is always
/// emitted (zero-filled when absent) because the adapter derives the field
/// width from the total length: `N = (len(ARGS) - 12) / 3`.
///
/// `idlen` is 1 for K-line, 4 for CAN/ISO15765, 5 for extended addressing.
pub fn start_filter(
    proto: ProtocolId,
    filter_id: u32,
    ftype: u32,
    mask: &[u8],
    pattern: &[u8],
    flow: Option<&[u8]>,
    idlen: usize,
) -> Result<Vec<u8>, CodecError> {
    if !(idlen == 1 || idlen == 4 || idlen == 5) || (proto.is_can() && idlen == 1) {
        return Err(CodecError::BadIdLen { proto, idlen });
    }
    for field in [Some(mask), Some(pattern), flow].into_iter().flatten() {
        if field.len() < idlen {
            return Err(CodecError::FilterFieldTooShort { have: field.len(), need: idlen });
        }
    }

    let args = 12 + 3 * idlen;
    let mut v = Vec::with_capacity(3 + args);
    v.extend_from_slice(&ilen(args));
    v.push(Cmd::StartMsgFilter as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(filter_id));
    v.extend_from_slice(&u32le(ftype));
    v.extend_from_slice(&mask[..idlen]);
    v.extend_from_slice(&pattern[..idlen]);
    match flow {
        Some(f) => v.extend_from_slice(&f[..idlen]),
        None => v.resize(v.len() + idlen, 0),
    }
    Ok(v)
}

/// SET_PROGRAMMING_VOLTAGE (0x0D). Device-scoped (no ProtocolID) — ARGS are the
/// J2534 pin number then the voltage in mV.
///
/// On the Mini-VCI the firmware stores the value and drives no pin (a no-op
/// that reports success); cables whose firmware implements the opcode act on
/// it. `PIN_OFF`-style sentinels (`0xffffffff` / `0`) are passed through
/// unchanged.
pub fn set_programming_voltage(pin: u32, millivolts: u32) -> Vec<u8> {
    let mut v = Vec::with_capacity(11);
    v.extend_from_slice(&ilen(8));
    v.push(Cmd::SetProgrammingVoltage as u8);
    v.extend_from_slice(&u32le(pin));
    v.extend_from_slice(&u32le(millivolts));
    v
}

pub fn stop_filter(proto: ProtocolId, filter_id: u32) -> Vec<u8> {
    let mut v = Vec::with_capacity(15);
    v.extend_from_slice(&ilen(12));
    v.push(Cmd::StopMsgFilter as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(filter_id));
    v.extend_from_slice(&u32le(0));
    v
}

/// START_PERIODIC_MSG (0x0F). `msg_id` is **host-assigned** (like a filter
/// handle): the caller picks it, the firmware echoes it back and matches it on
/// STOP_PERIODIC_MSG. `msg` must be under 13 bytes — the firmware's `N < 13`
/// limit — which fits a CAN periodic (4-byte id + up to 8 data bytes) but
/// leaves no room for an extended-addressing byte on top of a full frame.
pub fn start_periodic(
    proto: ProtocolId,
    msg_id: u32,
    txflags: u32,
    interval_ms: u32,
    msg: &[u8],
) -> Result<Vec<u8>, CodecError> {
    if msg.len() >= 13 {
        return Err(CodecError::MsgTooLong(msg.len()));
    }
    let mut v = Vec::with_capacity(3 + 16 + msg.len());
    v.extend_from_slice(&ilen(16 + msg.len()));
    v.push(Cmd::StartPeriodicMsg as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(msg_id));
    v.extend_from_slice(&u32le(txflags));
    v.extend_from_slice(&u32le(interval_ms));
    v.extend_from_slice(msg);
    Ok(v)
}

/// STOP_PERIODIC_MSG (0x10). Mirrors `stop_filter`: proto, the host-assigned
/// MsgID, and an unused trailing word.
pub fn stop_periodic(proto: ProtocolId, msg_id: u32) -> Vec<u8> {
    let mut v = Vec::with_capacity(15);
    v.extend_from_slice(&ilen(12));
    v.push(Cmd::StopPeriodicMsg as u8);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(msg_id));
    v.extend_from_slice(&u32le(0));
    v
}

/// IOCTL sub 0x02 = SET_CONFIG. One parameter per call — the firmware does
/// not take an SCONFIG_LIST. Unknown parameters return success and are
/// silently discarded by the firmware, so a clean status proves nothing.
pub fn set_config(proto: ProtocolId, param: u32, value: u32) -> Vec<u8> {
    let mut v = Vec::with_capacity(16);
    v.extend_from_slice(&ilen(13));
    v.push(Cmd::Ioctl as u8);
    v.push(0x02);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(param));
    v.extend_from_slice(&u32le(value));
    v
}

/// IOCTL sub 0x01 = GET_CONFIG. One parameter per call, mirroring SET_CONFIG.
pub fn get_config(proto: ProtocolId, param: u32) -> Vec<u8> {
    let mut v = Vec::with_capacity(12);
    v.extend_from_slice(&ilen(9));
    v.push(Cmd::Ioctl as u8);
    v.push(0x01);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(&u32le(param));
    v
}

/// Parse a GET_CONFIG reply into the config value.
///
/// **Wire layout unverified.** The firmware RE (§9) confirms the getter exists
/// and is one-parameter-per-call, but does not pin the reply framing. This
/// assumes the value trails the IOCTL command echo
/// (`[ILEN][00][0x0e][value u32 LE]`, analogous to the K-line init reply's
/// length-driven tail). Confirm against the live cable — read back DATA_RATE
/// (0x01) and compare — before trusting it.
pub fn parse_get_config(inner: &[u8]) -> Result<u32, CodecError> {
    if inner.len() < 7 || inner[2] != Cmd::Ioctl as u8 {
        return Err(CodecError::MalformedReply("not a get-config reply"));
    }
    Ok(u32::from_le_bytes(inner[3..7].try_into().unwrap()))
}

/// IOCTL sub 0x03 = READ_VBATT. Same shape as CLEAR_PERIODIC_MSGS.
pub fn read_vbatt(proto: ProtocolId) -> Vec<u8> {
    let mut v = Vec::with_capacity(8);
    v.extend_from_slice(&ilen(5));
    v.push(Cmd::Ioctl as u8);
    v.push(0x03);
    v.extend_from_slice(&u32le(proto.wire()));
    v
}

/// Parse a READ_VBATT reply into millivolts.
///
/// **Wire layout unverified**, same assumption as [`parse_get_config`]: the
/// value trails the IOCTL command echo. On the Mini-VCI the firmware answers a
/// hardcoded 12000 mV (no ADC); other cables return a real measurement. Confirm
/// the layout against the live cable before trusting it.
pub fn parse_vbatt(inner: &[u8]) -> Result<u32, CodecError> {
    if inner.len() < 7 || inner[2] != Cmd::Ioctl as u8 {
        return Err(CodecError::MalformedReply("not a read-vbatt reply"));
    }
    Ok(u32::from_le_bytes(inner[3..7].try_into().unwrap()))
}

/// IOCTL sub 0x04 = FIVE_BAUD_INIT (K-line slow init). Same shape as
/// [`fast_init`], different sub-function.
pub fn five_baud_init(proto: ProtocolId, init: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(8 + init.len());
    v.extend_from_slice(&ilen(5 + init.len()));
    v.push(Cmd::Ioctl as u8);
    v.push(0x04);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(init);
    v
}

/// IOCTL sub 0x09 = CLEAR_PERIODIC_MSGS.
pub fn clear_periodic(proto: ProtocolId) -> Vec<u8> {
    let mut v = Vec::with_capacity(8);
    v.extend_from_slice(&ilen(5));
    v.push(Cmd::Ioctl as u8);
    v.push(0x09);
    v.extend_from_slice(&u32le(proto.wire()));
    v
}

/// IOCTL sub 0x05 = FAST_INIT (K-line).
pub fn fast_init(proto: ProtocolId, init: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(8 + init.len());
    v.extend_from_slice(&ilen(5 + init.len()));
    v.push(Cmd::Ioctl as u8);
    v.push(0x05);
    v.extend_from_slice(&u32le(proto.wire()));
    v.extend_from_slice(init);
    v
}

/// Extract the DES key from the plaintext handshake challenge payload
/// (`09 00 01 <K0..K7>`).
pub fn parse_challenge(payload: &[u8]) -> Result<[u8; 8], CodecError> {
    if payload.len() < 11 {
        return Err(CodecError::MalformedReply("challenge payload too short"));
    }
    if payload[0] != 0x09 {
        return Err(CodecError::MalformedReply("challenge is not a key reply"));
    }
    Ok(payload[3..11].try_into().unwrap())
}

/// Validate a status reply (`[ILEN=2][00][CMD][status]`) and return the
/// device status. The bytes after the status byte are the adapter's
/// uninitialised stack — it copies a fixed five bytes out of a one-byte
/// payload — and must never be parsed.
pub fn status_of(inner: &[u8], cmd: Cmd) -> Result<DeviceStatus, CodecError> {
    if inner.len() < 4 {
        return Err(CodecError::MalformedReply("status reply too short"));
    }
    if super::frame::rd_u16(inner) != 2 {
        return Err(CodecError::MalformedReply("status reply has wrong ILEN"));
    }
    if inner[2] != cmd as u8 {
        return Err(CodecError::MalformedReply("status reply echoes a different command"));
    }
    Ok(DeviceStatus::from(inner[3]))
}

/// A parsed ReadMsgs reply:
/// `[ILEN u16 LE][0x09][RxStatus u32 LE][reserved u32][msg @ offset 11]`,
/// message length `ILEN - 9`.
#[derive(Debug, PartialEq, Eq)]
pub struct ReadReply<'a> {
    pub rx_status: RxStatus,
    /// Empty for a status-only reply (nothing queued).
    pub msg: &'a [u8],
}

pub fn parse_read_reply(inner: &[u8]) -> Result<ReadReply<'_>, CodecError> {
    if inner.len() < 4 || inner[2] != Cmd::ReadMsg as u8 {
        return Err(CodecError::MalformedReply("not a read reply"));
    }
    let rx_status = if inner.len() >= 7 {
        RxStatus::from_bits_retain(u32::from_le_bytes(inner[3..7].try_into().unwrap()))
    } else {
        RxStatus::from_bits_retain(inner[3] as u32)
    };
    let field = super::frame::rd_u16(inner);
    let mlen = field.saturating_sub(9);
    if mlen == 0 {
        return Ok(ReadReply { rx_status, msg: &[] });
    }
    if 11 + mlen > inner.len() {
        return Err(CodecError::MalformedReply("read reply message overruns the inner"));
    }
    Ok(ReadReply { rx_status, msg: &inner[11..11 + mlen] })
}
