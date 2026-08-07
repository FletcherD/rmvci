//! Session layer: the port-owning actor, the encrypted-link handshake, and
//! the public `Device` handle.

pub(crate) mod actor;
mod channel;
mod device;
mod link;
pub mod protocol;
mod raw;
#[cfg(test)]
mod tests;
mod wire;

pub use channel::Channel;
pub use device::{Device, DeviceConfig, resolve_port};
pub use raw::RawChannel;
