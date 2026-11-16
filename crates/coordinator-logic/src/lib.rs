//! The coordinator's side of the bus (milestone M5),
//! kept apart from its hardware so that it compiles and tests on the host.
//!
//! - [`nodes`]: the watch over the nodes, from their heartbeats.
//!   A node is present, lost, restarted, or speaking another protocol version.
//! - [`readings`]: the sensor's readings paired with the shots of a job, late ones asked for again.
//! - [`moves`]: one move in flight on a node, its MOVE_REL sent again while its answer is late.
//!
//! The coordinator's state machine, which gates the relay and the motors, is `align_core::coordinator`; this crate
//! turns bus traffic into its events. `firmware/coordinator` will wire both to TWAI and the Ruida I/O.
#![no_std]

pub mod moves;
pub mod nodes;
pub mod readings;
