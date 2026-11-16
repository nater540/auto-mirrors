//! CAN protocol for the mirror alignment bus: identifiers, message structs, and manual fixed-layout encoding.
//!
//! The messages come from `docs/REFERENCE.md` section 5.4 with the field changes in `docs/ADDENDUM_A_28BYJ48.md`
//! section A.8. There is no serde on the wire: every layout is written out by hand, little-endian.
//!
//! - [`frame`]: the raw frame, the 11-bit identifier layout, node addresses, and channels.
//! - [`message`]: the ten messages and their encoding and decoding, through [`Envelope`].
//! - [`dedup`]: the node-side check that runs a resent MOVE_REL only once.
//!
//! The crate is `no_std` without `alloc` and knows no HAL: the firmware converts between [`Frame`] and its TWAI
//! frame type.
#![no_std]

pub mod dedup;
pub mod frame;
pub mod message;

pub use dedup::{MoveDecision, MoveRefused, MoveRelTracker};
pub use frame::{Channel, Class, Frame, FrameError, NodeId};
pub use message::{
  ConfigKey, DecodeError, Envelope, FaultCode, Message, MoveDone, MoveRel, MoveResult, SensorReading, Status,
  StatusFlags,
};
