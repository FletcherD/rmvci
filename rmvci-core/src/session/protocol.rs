//! Protocol markers and per-protocol filter types.
//!
//! The filter identifier width (`idlen`) is protocol-dependent — 1 byte on
//! K-line, a 4-byte CAN id on CAN/ISO15765, 5 with extended addressing — and
//! the adapter infers it from the command length, rejecting anything else.
//! Encoding it in the `Protocol::Filter` associated type makes a mismatched
//! filter unrepresentable instead of a runtime rejection.

use crate::types::{FilterType, ProtocolId};

mod sealed {
    pub trait Sealed {}
}

/// A protocol the adapter implements, as a type. See [`crate::Channel`].
pub trait Protocol: sealed::Sealed + Send + 'static {
    const ID: ProtocolId;
    type Filter: FilterSpec;
    type Config: ChannelConfig;
}

/// Connect parameters, lowered to the wire `(ConnectFlags, BaudRate)` pair.
pub trait ChannelConfig {
    fn wire(&self) -> (u32, u32);
}

/// An 11- or 29-bit CAN identifier. Encoded big-endian on the wire, matching
/// J2534's `PASSTHRU_MSG.Data[0..4]` convention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CanId {
    Std(u16),
    Ext(u32),
}

impl CanId {
    pub fn to_wire(self) -> [u8; 4] {
        match self {
            Self::Std(v) => (v as u32 & 0x7ff).to_be_bytes(),
            Self::Ext(v) => (v & 0x1fff_ffff).to_be_bytes(),
        }
    }

    pub fn is_ext(self) -> bool {
        matches!(self, Self::Ext(_))
    }
}

/// A filter lowered to its wire fields.
pub struct EncodedFilter {
    pub ftype: FilterType,
    pub mask: Vec<u8>,
    pub pattern: Vec<u8>,
    pub flow: Option<Vec<u8>>,
    pub idlen: usize,
}

pub trait FilterSpec {
    fn encode(&self) -> EncodedFilter;
}

/// K-line PASS filter: one mask/pattern byte over the message header.
#[derive(Debug, Clone, Copy)]
pub struct KLineFilter {
    pub mask: u8,
    pub pattern: u8,
}

impl FilterSpec for KLineFilter {
    fn encode(&self) -> EncodedFilter {
        EncodedFilter {
            ftype: FilterType::Pass,
            mask: vec![self.mask],
            pattern: vec![self.pattern],
            flow: None,
            idlen: 1,
        }
    }
}

/// Raw-CAN PASS filter.
///
/// The adapter turns `(mask, pattern)` into the *range*
/// `[pattern & mask, pattern | !mask]` — so the J2534-habitual `0x7FF` mask
/// opens the range to `0xFFFFFFCC` and matches nothing useful. Use
/// [`CanFilter::exact`] unless you know you want a range.
#[derive(Debug, Clone, Copy)]
pub struct CanFilter {
    pub mask: u32,
    pub pattern: CanId,
}

impl CanFilter {
    /// Exact-match filter for one identifier (mask `FF FF FF FF`).
    pub fn exact(id: CanId) -> Self {
        Self { mask: 0xffff_ffff, pattern: id }
    }
}

impl FilterSpec for CanFilter {
    fn encode(&self) -> EncodedFilter {
        EncodedFilter {
            ftype: FilterType::Pass,
            mask: self.mask.to_be_bytes().to_vec(),
            pattern: self.pattern.to_wire().to_vec(),
            flow: None,
            idlen: 4,
        }
    }
}

/// ISO15765 flow-control filter: `rx` is the identifier the ECU answers on
/// (what we receive), `tx` the identifier the adapter sends flow control to.
///
/// Built through [`FlowControlFilter::exact`] the mask is `FF FF FF FF`; the
/// range trap (see [`CanFilter`]) is opt-in via [`FlowControlFilter::with_mask`].
#[derive(Debug, Clone, Copy)]
pub struct FlowControlFilter {
    rx: CanId,
    tx: CanId,
    mask: u32,
    /// Extended-addressing byte (5-byte fields on the wire). **Untested on
    /// hardware** — the 5th byte layout comes from the firmware RE only.
    ext_addr: Option<u8>,
}

impl FlowControlFilter {
    pub fn exact(rx: CanId, tx: CanId) -> Self {
        Self { rx, tx, mask: 0xffff_ffff, ext_addr: None }
    }

    pub fn with_mask(mut self, mask: u32) -> Self {
        self.mask = mask;
        self
    }

    pub fn with_ext_addr(mut self, addr: u8) -> Self {
        self.ext_addr = Some(addr);
        self
    }

    pub fn rx(&self) -> CanId {
        self.rx
    }

    pub fn tx(&self) -> CanId {
        self.tx
    }
}

impl FilterSpec for FlowControlFilter {
    fn encode(&self) -> EncodedFilter {
        let mut mask = self.mask.to_be_bytes().to_vec();
        let mut pattern = self.rx.to_wire().to_vec();
        let mut flow = self.tx.to_wire().to_vec();
        let mut idlen = 4;
        if let Some(addr) = self.ext_addr {
            mask.push(0xff);
            pattern.push(addr);
            flow.push(addr);
            idlen = 5;
        }
        EncodedFilter { ftype: FilterType::FlowControl, mask, pattern, flow: Some(flow), idlen }
    }
}

// ---- protocol markers -------------------------------------------------

pub struct Iso9141;
pub struct Iso14230;
pub struct Can;
pub struct Iso15765;

/// K-line connect parameters. The defaults match the vendor tool's ISO14230
/// session (`connect(4, 4096, 10400)` — flag 0x1000 is ISO9141_K_LINE_ONLY).
#[derive(Debug, Clone, Copy)]
pub struct KLineConfig {
    pub baud: u32,
    pub flags: u32,
}

impl Default for KLineConfig {
    fn default() -> Self {
        Self { baud: 10400, flags: 0x1000 }
    }
}

impl ChannelConfig for KLineConfig {
    fn wire(&self) -> (u32, u32) {
        (self.flags, self.baud)
    }
}

/// CAN / ISO15765 connect parameters.
///
/// The firmware does not validate the bitrate: `PCLK/bitrate` truncates
/// silently, so a rate that doesn't divide 40 MHz cleanly is quietly rounded
/// (500001 → 571 kbit/s). Stick to the standard rates.
#[derive(Debug, Clone, Copy)]
pub struct CanConfig {
    pub bitrate: u32,
    pub flags: u32,
}

impl Default for CanConfig {
    fn default() -> Self {
        Self { bitrate: 500_000, flags: 0 }
    }
}

impl ChannelConfig for CanConfig {
    fn wire(&self) -> (u32, u32) {
        (self.flags, self.bitrate)
    }
}

impl sealed::Sealed for Iso9141 {}
impl sealed::Sealed for Iso14230 {}
impl sealed::Sealed for Can {}
impl sealed::Sealed for Iso15765 {}

impl Protocol for Iso9141 {
    const ID: ProtocolId = ProtocolId::Iso9141;
    type Filter = KLineFilter;
    type Config = KLineConfig;
}

impl Protocol for Iso14230 {
    const ID: ProtocolId = ProtocolId::Iso14230;
    type Filter = KLineFilter;
    type Config = KLineConfig;
}

impl Protocol for Can {
    const ID: ProtocolId = ProtocolId::Can;
    type Filter = CanFilter;
    type Config = CanConfig;
}

impl Protocol for Iso15765 {
    const ID: ProtocolId = ProtocolId::Iso15765;
    type Filter = FlowControlFilter;
    type Config = CanConfig;
}

/// CAN-framed protocols (4-byte identifier prefix on every message).
pub trait CanFramed: Protocol {}
impl CanFramed for Can {}
impl CanFramed for Iso15765 {}

/// K-line protocols (ISO9141 / ISO14230). They share one firmware class
/// object, so message send and both init flavours are identical across the
/// two. Only ISO14230 is hardware-verified — ISO9141's firmware path exists
/// but is unexercised on a real ECU.
pub trait KLine: Protocol {}
impl KLine for Iso9141 {}
impl KLine for Iso14230 {}
