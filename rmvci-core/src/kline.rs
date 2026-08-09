//! ISO 14230 (K-line KWP2000) as a [`UdsTransport`].
//!
//! [`KLineEcu`] wraps a raw [`Channel<Iso14230>`] and one ECU address into the
//! same request/response shape the ISO-TP paths expose, so a diagnostic layer
//! (e.g. the `rmvci-kwp` crate) drives K-line and CAN identically.
//!
//! It carries the two bring-up facts a Toyota K-line ECU needs, learned the
//! hard way on a real car:
//!
//! - The **full vendor timing config** ([`KLINE_TIMING`]) plus a header PASS
//!   filter must be installed *before* FAST_INIT. Skip it and FAST_INIT returns
//!   garbage and subsequent writes wedge the adapter.
//! - Requests are wrapped in the ISO 14230 header `<0x80|len> <ecu> <tester>`
//!   (the adapter appends the checksum); replies come back header-on
//!   (`<0x80|len> <tester> <ecu> <service…>`) with the checksum stripped, and
//!   [`KLineEcu`] strips the header down to the service bytes.
//!
//! Response-pending (`7F .. 78`) frames are consumed transparently, as with the
//! ISO-TP transports.

use std::time::Duration;

use crate::error::{CodecError, Error};
use crate::isotp::{UdsTransport, drain_pending};
use crate::session::protocol::{Iso14230, KLineConfig, KLineFilter};
use crate::session::{Channel, Device};

/// The 12 `SET_CONFIG` (param, value) pairs the vendor tool (Techstream) sends
/// to bring up an ISO 14230 channel, applied by [`KLineEcu::connect`] before
/// FAST_INIT.
///
/// The **parameter ids** are the standard J2534 `SConfig` ids (below); the
/// **values** are Techstream's captured ISO14230 timing profile for these
/// Toyota ECUs — not the J2534 defaults (e.g. TINIL 35 vs the 25 ms default),
/// and *not* independently re-derived here. We know what each one controls; we
/// have not isolated which are actually load-bearing on the Mini-VCI. The
/// firmware silently discards params it doesn't implement, and several
/// (`P3_MAX`, some `W`s) are "not used" in J2534 itself, so the empirically
/// necessary subset is likely just the fast-init pulse widths + `P1_MAX`/`P3_MIN`
/// — an ablation on the car would confirm. We send the whole vendor profile
/// because it is known-good and cheap.
pub const KLINE_TIMING: [(u32, u32); 12] = [
    (cfg::DATA_RATE, 9600), // K-line bit rate hint
    (cfg::P1_MAX, 40),      // max gap between ECU response bytes
    (cfg::P3_MIN, 10),      // min gap after a response before the next request
    (cfg::P3_MAX, 10),
    (cfg::TIDLE, 300), // bus-idle time required before a fast-init pulse
    (cfg::TINIL, 35),  // fast-init low-pulse width
    (cfg::TWUP, 50),   // wake-up pulse width
    (cfg::W1, 25),     // ┐
    (cfg::W2, 20),     // │  ISO9141 / 5-baud slow-init timing windows
    (cfg::W3, 20),     // │  (set for completeness; fast-init doesn't use them)
    (cfg::W4, 25),     // │
    (cfg::W5, 10),     // ┘
];

/// Standard J2534 `SConfig` parameter ids for the ISO9141/14230 timing values
/// above. These are a J2534 PassThru-API concept (not a diagnostic-protocol
/// one), so no diagnostic crate defines them; the authoritative source is the
/// SAE J2534 spec. Named here rather than left as bare integers.
mod cfg {
    pub const DATA_RATE: u32 = 0x01;
    pub const P1_MAX: u32 = 0x07;
    pub const P3_MIN: u32 = 0x0a;
    pub const P3_MAX: u32 = 0x0b;
    pub const W1: u32 = 0x0e;
    pub const W2: u32 = 0x0f;
    pub const W3: u32 = 0x10;
    pub const W4: u32 = 0x11;
    pub const W5: u32 = 0x12;
    pub const TIDLE: u32 = 0x13;
    pub const TINIL: u32 = 0x14;
    pub const TWUP: u32 = 0x15;
}

const TESTER_DEFAULT: u8 = 0xf0;

/// One ISO 14230 ECU, addressable as a [`UdsTransport`].
pub struct KLineEcu {
    chan: Channel<Iso14230>,
    ecu: u8,
    tester: u8,
}

impl KLineEcu {
    /// Wrap an already-initialised channel targeting `ecu` (tester `0xF0`).
    /// Prefer [`KLineEcu::connect`], which also runs the vendor bring-up.
    pub fn new(chan: Channel<Iso14230>, ecu: u8) -> Self {
        Self { chan, ecu, tester: TESTER_DEFAULT }
    }

    /// Override the tester (source) address (default `0xF0`).
    pub fn with_tester(mut self, tester: u8) -> Self {
        self.tester = tester;
        self
    }

    /// Open an ISO 14230 channel to `ecu` and run the vendor bring-up: install
    /// the header PASS filter and the [`KLINE_TIMING`] config. Does **not**
    /// FAST_INIT — call [`KLineEcu::fast_init`] next (kept separate so the
    /// caller sees the key bytes).
    pub fn connect(dev: &Device, ecu: u8) -> Result<Self, Error> {
        let mut chan = dev.connect::<Iso14230>(KLineConfig::default())?;
        // Response headers are 0x8x (0x80|len); keep the top two bits.
        chan.set_filter(KLineFilter { mask: 0xc0, pattern: 0x80 })?;
        for (param, value) in KLINE_TIMING {
            chan.set_config(param, value)?;
        }
        chan.clear_periodic()?;
        Ok(Self::new(chan, ecu))
    }

    /// FAST_INIT + StartCommunication (`81`) to the ECU; returns the raw key-byte
    /// frame the ECU answers with.
    pub fn fast_init(&mut self) -> Result<Vec<u8>, Error> {
        let frame = self.frame(&[0x81]);
        self.chan.fast_init(&frame)
    }

    /// The wrapped channel, for direct access (e.g. a second FAST_INIT).
    pub fn channel(&mut self) -> &mut Channel<Iso14230> {
        &mut self.chan
    }

    /// The ECU (target) address.
    pub fn ecu(&self) -> u8 {
        self.ecu
    }

    fn frame(&self, payload: &[u8]) -> Vec<u8> {
        build_frame(self.ecu, self.tester, payload)
    }
}

/// Build an ISO 14230 request header + payload: `<0x80|len> <ecu> <tester>`
/// for `len < 64`, else `0x80 <ecu> <tester> <len>`. The adapter appends the
/// checksum.
fn build_frame(ecu: u8, tester: u8, payload: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 + payload.len());
    if payload.len() < 0x40 {
        v.push(0x80 | payload.len() as u8);
        v.push(ecu);
        v.push(tester);
    } else {
        v.push(0x80);
        v.push(ecu);
        v.push(tester);
        v.push(payload.len() as u8);
    }
    v.extend_from_slice(payload);
    v
}

/// Strip the ISO 14230 header from a received frame, returning the service
/// bytes. Header is `<0x80|len> <a> <b>` (len inline in the low 6 bits) or
/// `0x80 <a> <b> <len>` (separate length byte). The adapter has already removed
/// the trailing checksum.
fn strip_header(frame: &[u8]) -> Result<&[u8], Error> {
    if frame.len() < 4 {
        return Err(CodecError::MalformedReply("K-line frame shorter than a header").into());
    }
    let fmt = frame[0];
    let (hdr, len) = match (fmt & 0x3f) as usize {
        0 => (4usize, frame[3] as usize),
        n => (3usize, n),
    };
    let end = (hdr + len).min(frame.len());
    Ok(&frame[hdr..end])
}

impl UdsTransport for KLineEcu {
    fn request(&mut self, req: &[u8], timeout: Duration) -> Result<Vec<u8>, Error> {
        self.chan.clear_periodic()?; // vendor sequence clears before every request
        let frame = self.frame(req);
        self.chan.write(&frame)?;
        let chan = &mut self.chan;
        drain_pending(|| {
            let raw = chan.read(timeout)?;
            Ok(strip_header(&raw.data)?.to_vec())
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_wraps_short_payload() {
        assert_eq!(build_frame(0x98, 0xf0, &[0x21, 0x0d]), [0x82, 0x98, 0xf0, 0x21, 0x0d]);
        assert_eq!(build_frame(0x98, 0xf0, &[0x81]), [0x81, 0x98, 0xf0, 0x81]);
    }

    #[test]
    fn frame_uses_length_byte_past_63() {
        let payload = vec![0xaa; 70];
        let f = build_frame(0x33, 0xf0, &payload);
        assert_eq!(&f[..4], [0x80, 0x33, 0xf0, 70]);
        assert_eq!(f.len(), 4 + 70);
    }

    #[test]
    fn strip_recovers_service_bytes() {
        // 0x83 F0 98 61 0D 09  ->  61 0D 09
        assert_eq!(strip_header(&[0x83, 0xf0, 0x98, 0x61, 0x0d, 0x09]).unwrap(), &[0x61, 0x0d, 0x09]);
        // 0x82 F0 98 61 0E     ->  61 0E
        assert_eq!(strip_header(&[0x82, 0xf0, 0x98, 0x61, 0x0e]).unwrap(), &[0x61, 0x0e]);
    }

    #[test]
    fn strip_rejects_runt() {
        assert!(strip_header(&[0x80, 0xf0, 0x98]).is_err());
    }
}
