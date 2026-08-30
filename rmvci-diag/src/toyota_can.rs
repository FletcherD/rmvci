//! Toyota's CAN live-data services `A8` / `A1` / `A2` — the CAN counterpart of
//! the K-line per-LID poll, recovered from the forwarding-shim capture
//! (`re/techstream/shim/mvci_calls.log`) and documented in
//! `re/techstream/DATA_MODEL.md` §8a. Engine (`7E0`), Hybrid (`7E2`) and
//! HV-Battery (`7E3`) serve live data through this three-step sequence over
//! ISO-TP, not through individual `21`/`22` reads:
//!
//! ```text
//! A8 01                  → E8 01 <pid> <width> <width bytes> …   enumerate every
//!                                                                available PID + width
//! A1 <sub> <selection>   → E1 <sub> <selection>   define a composite poll list (echoed)
//! A2 <listId>            → E2 <listId> <packed>   poll that list; values concatenated
//! A3                     → (no response)          tear the list down
//! ```
//!
//! Positive responses follow the `+0x40` KWP convention (`A8→E8`, `A1→E1`,
//! `A2→E2`). The enumerated `A8 01` widths — not the ddb Pid table — are what
//! slice the packed `E2` payload back into per-PID values (§8a).
//!
//! This client needs `send` + repeated `recv` (the `A8 01` enumeration streams
//! across several `E8 01` messages), which the one-shot [`UdsTransport`] seam
//! doesn't expose — hence the small [`IsoTpChannel`] trait, implemented for both
//! CAN transports.
//!
//! [`UdsTransport`]: rmvci_core::UdsTransport

use std::time::Duration;

use rmvci_core::Error as CoreError;

use crate::common::{parse_positive, strip_echo};
use crate::error::{Error, Result};

/// The `send` + `recv` pair the A8/A1/A2 sequence needs. [`rmvci_core::IsoTp`]
/// exposes these; the trait is the seam that lets [`ToyotaCanLive`] be driven
/// by a scripted channel in tests.
pub trait IsoTpChannel {
    /// Send one ISO-TP message (segmentation handled by the implementation).
    fn send(&mut self, payload: &[u8]) -> std::result::Result<(), CoreError>;
    /// Receive one reassembled ISO-TP message, or [`CoreError::Timeout`].
    fn recv(&mut self, timeout: Duration) -> std::result::Result<Vec<u8>, CoreError>;
}

impl IsoTpChannel for rmvci_core::IsoTp {
    fn send(&mut self, payload: &[u8]) -> std::result::Result<(), CoreError> {
        rmvci_core::IsoTp::send(self, payload)
    }
    fn recv(&mut self, timeout: Duration) -> std::result::Result<Vec<u8>, CoreError> {
        rmvci_core::IsoTp::recv(self, timeout)
    }
}

/// One entry of the `A8 01` enumeration: a PID, its on-wire byte width, and the
/// support/validity mask the ECU reported for it (all-`0xFF` = fully supported).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnumeratedPid {
    /// The PID id (OBD mode-`01`-style, matching the datalist `request.id`).
    pub id: u8,
    /// Byte width of this PID's value in the packed `E2` poll response.
    pub width: u8,
    /// The `<width>` support/validity bytes the ECU returned after the width.
    pub support: Vec<u8>,
}

/// Upper bound on consecutive `7F .. 78` responsePending frames tolerated in one
/// exchange before giving up (mirrors the core transport's own limit).
const MAX_PENDING: u32 = 30;

/// A Toyota CAN live-data session over one ECU's ISO-TP channel.
///
/// Construct the channel with the ECU's request id as `tx` and `tx + 8` as `rx`
/// (the Toyota response-id convention), then drive `enumerate_pids` →
/// `define_poll_list` → `poll_list` → `teardown`.
pub struct ToyotaCanLive<T: IsoTpChannel> {
    io: T,
    timeout: Duration,
    /// Quiescence timeout for the streamed `A8` enumeration: how long to wait for
    /// the *next* `E8` chunk before deciding the enumeration is complete.
    enum_gap: Duration,
}

impl<T: IsoTpChannel> ToyotaCanLive<T> {
    /// Wrap a ready ISO-TP channel (defaults: 2 s per exchange, 300 ms enumeration
    /// gap).
    pub fn new(io: T) -> Self {
        Self { io, timeout: Duration::from_secs(2), enum_gap: Duration::from_millis(300) }
    }

    /// Override the per-exchange timeout.
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }

    /// Override the enumeration quiescence gap (see [`enumerate_pids`]).
    ///
    /// [`enumerate_pids`]: Self::enumerate_pids
    pub fn with_enum_gap(mut self, gap: Duration) -> Self {
        self.enum_gap = gap;
        self
    }

    /// Recover the underlying channel.
    pub fn into_inner(self) -> T {
        self.io
    }

    /// One send + response, transparently swallowing `7F .. 78` responsePending
    /// frames. The returned bytes still carry the service echo.
    fn exchange(&mut self, req: &[u8]) -> Result<Vec<u8>> {
        self.io.send(req)?;
        let mut pending = 0u32;
        loop {
            let resp = self.io.recv(self.timeout)?;
            if matches!(resp.as_slice(), [0x7f, _, 0x78, ..]) && pending < MAX_PENDING {
                pending += 1;
                continue;
            }
            return Ok(resp);
        }
    }

    /// `A8 <sub>` — enumerate every available PID with its width. `sub` is `0x01`
    /// for the primary data-list descriptor (`0x02` for the secondary page seen
    /// on the Engine ECU).
    ///
    /// The ECU streams the enumeration as one or more `E8 <sub>` messages; this
    /// reads until a [`enum_gap`](Self::with_enum_gap)-length silence, then parses
    /// the concatenated `<pid> <width> <width bytes>` TLV stream. A negative
    /// response other than responsePending is surfaced as [`Error::Negative`].
    pub fn enumerate_pids(&mut self, sub: u8) -> Result<Vec<EnumeratedPid>> {
        self.io.send(&[0xa8, sub])?;
        let want = 0xa8u8.wrapping_add(0x40); // E8
        let mut pids = Vec::new();
        let mut pending = 0u32;
        loop {
            match self.io.recv(self.enum_gap) {
                Ok(resp) => match resp.as_slice() {
                    [0x7f, _, 0x78, ..] if pending < MAX_PENDING => pending += 1,
                    [0x7f, svc, nrc, ..] => {
                        return Err(Error::Negative { service: *svc, nrc: *nrc });
                    }
                    [first, s, tail @ ..] if *first == want && *s == sub => {
                        parse_pid_tlv(tail, &mut pids);
                    }
                    _ => {} // unknown frame — ignore and keep reading
                },
                Err(CoreError::Timeout(_)) => break, // enumeration quiescent
                Err(e) => return Err(e.into()),
            }
        }
        Ok(pids)
    }

    /// `A1 <sub> <members…>` — define a composite poll list. Two forms are seen
    /// on the wire: flat `A1 01 <pid …>` and TLV `A1 06 <pid> <count> <sub …>`;
    /// pass `sub` and the raw `members` bytes accordingly. Returns the echoed
    /// selection (the bytes after `E1`).
    pub fn define_poll_list(&mut self, sub: u8, members: &[u8]) -> Result<Vec<u8>> {
        let mut req = Vec::with_capacity(2 + members.len());
        req.push(0xa1);
        req.push(sub);
        req.extend_from_slice(members);
        let resp = self.exchange(&req)?;
        parse_positive(0xa1, resp)
    }

    /// `A2 <listId>` — poll the list defined under `listId`. Returns the packed
    /// value bytes (after the echoed `E2 <listId>`), to be sliced per member
    /// using the widths from [`enumerate_pids`](Self::enumerate_pids).
    pub fn poll_list(&mut self, list_id: u8) -> Result<Vec<u8>> {
        let resp = self.exchange(&[0xa2, list_id])?;
        let body = parse_positive(0xa2, resp)?;
        strip_echo(&body, &[list_id])
    }

    /// `A3` — tear the defined list(s) down. The ECU sends no response, so this
    /// only writes and never blocks on a read.
    pub fn teardown(&mut self) -> Result<()> {
        self.io.send(&[0xa3])?;
        Ok(())
    }
}

/// Parse a `<pid> <width> <width bytes>` TLV stream (the `A8` enumeration body,
/// after the `E8 <sub>` header) into [`EnumeratedPid`]s, appending to `out`. A
/// truncated trailing entry is dropped rather than guessed.
fn parse_pid_tlv(mut tlv: &[u8], out: &mut Vec<EnumeratedPid>) {
    while tlv.len() >= 2 {
        let id = tlv[0];
        let width = tlv[1];
        let end = 2 + width as usize;
        if end > tlv.len() {
            break; // incomplete final TLV
        }
        out.push(EnumeratedPid { id, width, support: tlv[2..end].to_vec() });
        tlv = &tlv[end..];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // `E8 01` enumeration body from mvci_calls.log:126 (Hybrid 7E2), i.e. the
    // bytes after `e8 01`.
    #[test]
    fn parses_a8_enumeration_tlv() {
        let body = [
            0x00, 0x04, 0xff, 0xff, 0xff, 0xff, // pid 00 width 4
            0x01, 0x04, 0xff, 0xff, 0xff, 0xff, // pid 01 width 4
            0x02, 0x02, 0xff, 0xff, // pid 02 width 2
            0x04, 0x01, 0xff, // pid 04 width 1
            0x0c, 0x02, 0xff, 0xff, // pid 0c width 2
            0x0d, 0x01, 0xff, // pid 0d width 1
        ];
        let mut pids = Vec::new();
        parse_pid_tlv(&body, &mut pids);
        assert_eq!(pids.len(), 6);
        assert_eq!(pids[0], EnumeratedPid { id: 0x00, width: 4, support: vec![0xff; 4] });
        assert_eq!(pids[3], EnumeratedPid { id: 0x04, width: 1, support: vec![0xff] });
        assert_eq!(pids[5].id, 0x0d);
        assert_eq!(pids[5].width, 1);
    }

    // Real non-`ff` support bytes from the Engine 7E0 enumeration
    // (mvci_calls.log:148): `01 04 ff 07 25 25`.
    #[test]
    fn keeps_partial_support_mask() {
        let body = [0x01, 0x04, 0xff, 0x07, 0x25, 0x25];
        let mut pids = Vec::new();
        parse_pid_tlv(&body, &mut pids);
        assert_eq!(
            pids,
            vec![EnumeratedPid { id: 1, width: 4, support: vec![0xff, 0x07, 0x25, 0x25] }]
        );
    }

    #[test]
    fn drops_truncated_trailing_tlv() {
        let body = [0x00, 0x04, 0xff, 0xff, 0xff, 0xff, 0x20, 0x04, 0xff]; // last claims width 4, only 1 byte
        let mut pids = Vec::new();
        parse_pid_tlv(&body, &mut pids);
        assert_eq!(pids.len(), 1);
        assert_eq!(pids[0].id, 0x00);
    }

    // A fake channel replaying scripted messages, to exercise the state machines
    // without hardware.
    struct FakeChan {
        sent: Vec<Vec<u8>>,
        replies: std::collections::VecDeque<std::result::Result<Vec<u8>, CoreError>>,
    }
    impl FakeChan {
        fn new(replies: Vec<std::result::Result<Vec<u8>, CoreError>>) -> Self {
            Self { sent: Vec::new(), replies: replies.into() }
        }
    }
    impl IsoTpChannel for FakeChan {
        fn send(&mut self, payload: &[u8]) -> std::result::Result<(), CoreError> {
            self.sent.push(payload.to_vec());
            Ok(())
        }
        fn recv(&mut self, timeout: Duration) -> std::result::Result<Vec<u8>, CoreError> {
            self.replies.pop_front().unwrap_or(Err(CoreError::Timeout(timeout)))
        }
    }

    #[test]
    fn enumerate_reads_multiple_chunks_until_quiescent() {
        // Two E8 01 chunks then silence (Timeout) ends it.
        let chan = FakeChan::new(vec![
            Ok(vec![0xe8, 0x01, 0x00, 0x01, 0xff, 0x20, 0x01, 0xff]),
            Ok(vec![0xe8, 0x01, 0x40, 0x02, 0xff, 0xff]),
            Err(CoreError::Timeout(Duration::from_millis(1))),
        ]);
        let mut can = ToyotaCanLive::new(chan);
        let pids = can.enumerate_pids(0x01).unwrap();
        assert_eq!(pids.iter().map(|p| p.id).collect::<Vec<_>>(), vec![0x00, 0x20, 0x40]);
        assert_eq!(can.into_inner().sent, vec![vec![0xa8, 0x01]]);
    }

    #[test]
    fn enumerate_skips_response_pending() {
        let chan = FakeChan::new(vec![
            Ok(vec![0x7f, 0xa8, 0x78]), // pending
            Ok(vec![0xe8, 0x01, 0x05, 0x01, 0xff]),
            Err(CoreError::Timeout(Duration::from_millis(1))),
        ]);
        let mut can = ToyotaCanLive::new(chan);
        let pids = can.enumerate_pids(0x01).unwrap();
        assert_eq!(pids, vec![EnumeratedPid { id: 0x05, width: 1, support: vec![0xff] }]);
    }

    #[test]
    fn poll_list_strips_echo_and_listid() {
        // E2 06 00 00 00 00 00 (parked, mvci_calls.log:76)
        let chan = FakeChan::new(vec![Ok(vec![0xe2, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00])]);
        let mut can = ToyotaCanLive::new(chan);
        let values = can.poll_list(0x06).unwrap();
        assert_eq!(values, vec![0x00, 0x00, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn define_poll_list_returns_echo_selection() {
        // A1 06 d3 05 09 0a 0b 0c 0d  ->  E1 06 d3 05 09 0a 0b 0c 0d (mvci_calls.log:71-72)
        let members = [0xd3, 0x05, 0x09, 0x0a, 0x0b, 0x0c, 0x0d];
        let mut echo = vec![0xe1, 0x06];
        echo.extend_from_slice(&members);
        let chan = FakeChan::new(vec![Ok(echo)]);
        let mut can = ToyotaCanLive::new(chan);
        let sel = can.define_poll_list(0x06, &members).unwrap();
        let mut expect = vec![0x06];
        expect.extend_from_slice(&members);
        assert_eq!(sel, expect);
    }
}
