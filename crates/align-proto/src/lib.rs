//! CAN protocol for the mirror alignment bus: identifiers, message structs, and manual fixed-layout encoding.
//!
//! The messages come from `docs/REFERENCE.md` section 5.4 with the field changes in `docs/ADDENDUM_A_28BYJ48.md`
//! section A.8. There is no serde on the wire: every layout is written out by hand, little-endian.
//!
//! - [`frame`]: the 11-bit identifier layout, node addresses, and channels.
//! - [`message`]: the ten messages and their encoding and decoding, through [`Envelope`].
//! - [`dedup`]: the node-side check that runs a resent MOVE_REL only once.
//! - [`sensor`]: the sensor node's field types (SHOT, SENSOR, SENSOR_CMD) and [`PROTOCOL_VERSION`].
//!
//! The crate is `no_std` without `alloc` and knows no HAL. Frames are the `embedded-can` [`embedded_can::Frame`]
//! trait, which the esp-hal TWAI frame implements, so the firmware passes its frames in and gets them back.
#![no_std]

pub mod dedup;
pub mod frame;
pub mod message;
pub mod sensor;
#[cfg(test)]
mod test_frame;

pub use dedup::{MoveDecision, MoveRefused, MoveRelTracker};
pub use frame::{Channel, Class, NodeId};
pub use message::{
  ConfigKey, DETAIL_NO_COMMAND, DecodeError, EncodeError, Envelope, FaultCode, Message, MoveDone, MoveRel, MoveResult,
  Status, StatusFlags
};
pub use sensor::{
  CmdStatus, PROTOCOL_VERSION, SUM_Q_ONE, SUM_Q_SATURATED, SensorCmd, SensorCmdReply, SensorOp, SensorReading,
  SensorResult, SensorStatus, Shot, ShotPoint
};
