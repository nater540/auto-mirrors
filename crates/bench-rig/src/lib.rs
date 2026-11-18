//! The bring-up of one axis on a bench (milestone M6 before the test frame exists): a node's motor turns a Series E
//! screw through the node's USB console, and the operator reads the pointer's dot off a paper target.
//!
//! The procedures are the aligner's own (`Aligner::measure_backlash`, `Aligner::measure_hysteresis`), unchanged, on two
//! bench implementations of its traits: [`motors::BenchMotors`] types `jog` at each node's console and reads the
//! node's count from its fixed lines, and [`eye::ReadByEye`] asks the operator where the dot is.
//!
//! - [`console`]: one node's console, from a command to the fixed line that answers it.
//! - [`motors`]: the motors behind `Actuator`.
//! - [`eye`]: the typed readings behind `BeamSensor`.
//! - [`procedure`]: the bench's configuration, the checks before the first move, and one run.
//!
//! **This is not the machine's interlock.** On the bench the node's firmware checks are the only layer: no coordinator,
//! no relay, no switched rail. The node moves only in bench mode, with the permit present, and refuses bench moves
//! while it hears a coordinator on the bus.

pub mod console;
pub mod eye;
pub mod motors;
pub mod procedure;

#[cfg(test)]
mod fake_node;
