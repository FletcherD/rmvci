//! Sans-IO protocol codec: pure functions and state machines, no I/O and no
//! threads. Everything here is exercised by the byte-exact captured vectors
//! (`tests/captured_vectors.rs`) and by property tests.

pub mod crypto;
pub mod deframe;
pub mod frame;
pub mod inner;

pub use deframe::Deframer;
