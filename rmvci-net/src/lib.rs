//! Network bridge for the Toyota Mini-VCI cable.
//!
//! `rmvci-core`'s [`Transport`](rmvci_core::transport::Transport) is five raw
//! byte-I/O primitives. This crate splits that boundary across a TCP link so
//! the cable can live on one machine while the driver runs on another —
//! e.g. the cable plugged into a rooted Android phone at the car, driven from
//! a PC on the same network.
//!
//! - [`TcpTransport`] (PC side) implements `Transport` by forwarding each
//!   primitive to a remote bridge. Hand it to
//!   [`Device::open_transport`](rmvci_core::Device::open_transport) and the
//!   whole stack above it — codec, session actor, channels, ISO-TP — is
//!   oblivious to the network.
//! - [`serve`] (phone side) owns a real cable transport
//!   ([`UsbTransport`](rmvci_core::transport) / `SerialTransport`) and executes
//!   the forwarded primitives. The `rmvci-bridge` binary is a thin wrapper
//!   around it.
//!
//! Only raw bytes cross the wire; all protocol timing (the DES handshake, the
//! ISO-TP state machines, the K-line P3 timing) stays local to whichever side
//! owns it. Reads are one network round trip each, so a low-latency link
//! (LAN / phone-USB-tether) matters — see the crate README.

mod client;
mod server;
mod wire;

pub use client::TcpTransport;
pub use server::{serve, serve_connection};

/// The bridge's default TCP port (mnemonic: the "MVCI" cable).
pub const DEFAULT_PORT: u16 = 6979;
