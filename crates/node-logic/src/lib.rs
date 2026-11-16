//! The motor node's decisions, kept apart from its hardware so that they compile and test on the host.
//!
//! - [`node`]: the state machine. Frames, time, permit, and move outcomes go in; it calls [`NodeIo`] to send
//!   frames, start and stop moves, and write position records.
//! - [`record`]: the position record kept in flash for each motor.
//! - [`console`]: the commands of the bench console (addendum A.6 torque margin test, manual jogs).
//!
//! `firmware/node` wires these to TWAI, the stepper tasks, the arm pulse, flash, and USB-Serial-JTAG.
#![no_std]

pub mod console;
pub mod node;
pub mod record;

pub use node::{
  AxisState, BenchRefused, MAX_CHANNELS, MoveOutcome, MoveRequest, Node, NodeIo, NodeSetup, NodeState, Restored
};
pub use record::{AxisRecord, RECORD_LEN, RecordError};
