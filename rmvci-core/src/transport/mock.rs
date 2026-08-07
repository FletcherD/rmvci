//! Script-driven mock transport for session-level tests (and for downstream
//! crates' tests — this module is always compiled).
//!
//! Each write consumes one script step: the step may assert the exact bytes
//! written and queues the scripted reply into the read buffer. Reads never
//! block — an empty buffer reports an immediate timeout (`Ok(0)`), which is
//! exactly the contract the session layer relies on.

use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::{Clock, Transport};
use crate::error::TransportError;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Event {
    Write(Vec<u8>),
    Modem { dtr: bool, rts: bool },
    PurgeRx,
}

pub struct Step {
    /// If set, the write must match these bytes exactly.
    pub expect: Option<Vec<u8>>,
    /// Bytes queued for subsequent reads.
    pub reply: Vec<u8>,
}

impl Step {
    pub fn exchange(expect: impl Into<Vec<u8>>, reply: impl Into<Vec<u8>>) -> Self {
        Self { expect: Some(expect.into()), reply: reply.into() }
    }

    /// Reply regardless of what was written (for nondeterministic traffic
    /// like keepalives).
    pub fn reply_any(reply: impl Into<Vec<u8>>) -> Self {
        Self { expect: None, reply: reply.into() }
    }

    /// Consume a write and answer nothing (models a dead/wedged adapter).
    pub fn silence() -> Self {
        Self { expect: None, reply: Vec::new() }
    }
}

#[derive(Default)]
pub struct MockTransport {
    steps: VecDeque<Step>,
    rx: VecDeque<u8>,
    events: Arc<Mutex<Vec<Event>>>,
}

impl MockTransport {
    pub fn new(steps: impl IntoIterator<Item = Step>) -> Self {
        Self { steps: steps.into_iter().collect(), rx: VecDeque::new(), events: Arc::default() }
    }

    /// Handle to the event log, valid after the transport moves into the
    /// session actor's thread.
    pub fn events(&self) -> Arc<Mutex<Vec<Event>>> {
        Arc::clone(&self.events)
    }

    /// The writes recorded so far (convenience over `events`).
    pub fn writes(events: &Arc<Mutex<Vec<Event>>>) -> Vec<Vec<u8>> {
        events
            .lock()
            .unwrap()
            .iter()
            .filter_map(|e| match e {
                Event::Write(w) => Some(w.clone()),
                _ => None,
            })
            .collect()
    }
}

impl Transport for MockTransport {
    fn write_all(&mut self, buf: &[u8]) -> Result<(), TransportError> {
        self.events.lock().unwrap().push(Event::Write(buf.to_vec()));
        if let Some(step) = self.steps.pop_front() {
            if let Some(exp) = &step.expect {
                assert_eq!(
                    buf,
                    &exp[..],
                    "MockTransport: unexpected write\n  expected {exp:02x?}\n  got      {buf:02x?}"
                );
            }
            self.rx.extend(step.reply);
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], _timeout: Duration) -> Result<usize, TransportError> {
        let n = buf.len().min(self.rx.len());
        for b in buf.iter_mut().take(n) {
            *b = self.rx.pop_front().unwrap();
        }
        Ok(n)
    }

    fn purge_rx(&mut self) -> Result<(), TransportError> {
        self.events.lock().unwrap().push(Event::PurgeRx);
        self.rx.clear();
        Ok(())
    }

    fn set_modem(&mut self, dtr: bool, rts: bool) -> Result<(), TransportError> {
        self.events.lock().unwrap().push(Event::Modem { dtr, rts });
        Ok(())
    }
}

/// No-op clock: handshake delays cost nothing in tests, and the requested
/// durations are recorded for assertions.
#[derive(Default, Clone)]
pub struct MockClock {
    sleeps: Arc<Mutex<Vec<Duration>>>,
}

impl MockClock {
    pub fn sleeps(&self) -> Arc<Mutex<Vec<Duration>>> {
        Arc::clone(&self.sleeps)
    }
}

impl Clock for MockClock {
    fn sleep(&self, d: Duration) {
        self.sleeps.lock().unwrap().push(d);
    }
}
