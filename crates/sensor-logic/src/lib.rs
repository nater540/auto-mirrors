//! The sensor node's decisions, kept apart from its hardware so that they compile and test on the host
//! (`docs/SENSOR_PLAN.md`, revision 4).
//!
//! - [`node`]: the state machine. Frames, time, and button chords go in; frames come out through [`SensorIo`].
//! - [`buttons`]: the five bench buttons that stand in for the detector, their debounce, chords, and positions.
//! - [`console`]: the commands of the bench console.
//!
//! `firmware/sensor` wires these to TWAI, the button inputs, and the USB console. Detector processing is deferred
//! until the detector is known (the plan's milestone S3).
#![no_std]

pub mod buttons;
pub mod console;
pub mod node;

pub use buttons::{ButtonBench, ButtonSet, Chords, Debouncer};
pub use node::{Injection, KEPT, MAX_PENDING, Reading, SensorIo, SensorNode, SensorSetup, SensorState, ShotMethod};
