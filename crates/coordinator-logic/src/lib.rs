//! The coordinator's side of the bus (milestone M5),
//! kept apart from its hardware so that it compiles and tests on the host.
//!
//! - [`nodes`]: the watch over the nodes, from their heartbeats.
//!   A node is present, lost, restarted, or speaking another protocol version.
//! - [`link`]: the coordinator's own heartbeat and inhibit level, and what each node last said.
//! - [`readings`]: the sensor's readings paired with the shots of a job, late ones asked for again.
//! - [`moves`]: one move in flight on a node, its MOVE_REL sent again while its answer is late.
//! - [`console`]: the commands of the coordinator's USB console.
//!
//! The coordinator's state machine, which gates the relay and the motors, is `align_core::coordinator`; this crate
//! turns bus traffic into its events. `firmware/coordinator` wires both to TWAI and, later, the Ruida I/O.
#![no_std]

pub mod console;
pub mod link;
pub mod moves;
pub mod nodes;
pub mod readings;
