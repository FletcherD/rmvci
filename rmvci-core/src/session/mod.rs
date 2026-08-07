//! Session layer: the port-owning actor, the encrypted-link handshake, and
//! the public `Device` handle.

pub(crate) mod actor;
mod device;
mod link;
#[cfg(test)]
mod tests;
mod wire;

pub use device::{Device, DeviceConfig, resolve_port};
