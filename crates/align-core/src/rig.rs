//! The two seams between the alignment logic and the machine: something that turns a screw,
//! and something that reads the beam.
//!
//! The simulator implements both against a model. The coordinator firmware implements them over CAN and the
//! Ruida I/O: a move is a `MOVE_REL` answered by a `MOVE_DONE`, and a measurement is one canned job of three
//! dots. Both may take seconds, so both are `async`; nothing in this crate assumes a fast answer.
//!
//! The actuator seam is also what lets a DC gearmotor with an encoder replace the stepper (hand-off section 2,
//! item 2): the alignment logic knows only "move this axis by this many steps and tell me when it stopped".

use crate::axis::Axis;
use crate::measurement::Shots;

/// Why a move did not complete.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum MoveFault {
  /// The move was stopped before it finished.
  #[error("move was stopped before it finished, at {pos_steps} steps")]
  Stopped {
    /// The commanded position the node reported where it stopped.
    pos_steps: i32
  },
  /// The motor rail was lost during the move, so steps were lost while the node kept counting.
  #[error("motor rail was lost during the move, at {pos_steps} steps")]
  RailLost {
    /// The commanded position the node reported where it stopped.
    pos_steps: i32
  },
  /// The node refused the move at its own soft limit. Nothing moved.
  #[error("node refused the move at its soft limit")]
  SoftLimit,
  /// The node did not answer, so whether and how far the motor moved is unknown.
  #[error("no answer from the node")]
  NoAnswer
}

/// Why a measurement produced no readings.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SensorFault {
  /// No reading arrived: the job did not run, no fire was seen, or the sensor node was silent.
  #[error("no reading arrived from the sensor")]
  NoReading
}

/// Turns one adjuster screw.
#[expect(
  async_fn_in_trait,
  reason = "the firmware and the simulator both run on one thread, so the future needs no Send bound"
)]
pub trait Actuator {
  /// Moves one axis by a signed number of full steps and returns, once the motor has stopped and released, the
  /// commanded position the node reports (the `pos_steps` of its `MOVE_DONE`), in full steps from the commissioning
  /// position. Positive is the plus direction: the screw advancing against the springs.
  ///
  /// The node's count is the one its soft limits and its record follow, so the aligner adopts it rather than
  /// adding up its own commands.
  /// The actuator has no position feedback: the count says what was commanded, not that the screw turned.
  ///
  /// # Errors
  ///
  /// A [`MoveFault`] if the move was cut short or never confirmed. After one, the axis position is not trusted.
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault>;
}

/// Reads the beam at the three gantry positions.
#[expect(
  async_fn_in_trait,
  reason = "the firmware and the simulator both run on one thread, so the future needs no Send bound"
)]
pub trait BeamSensor {
  /// Runs one measurement cycle (a shot at each of A, B, and C) and returns the three readings. The motors are
  /// stopped for the whole cycle: move, fire, and read strictly alternate.
  ///
  /// # Errors
  ///
  /// A [`SensorFault`] if no readings arrived. A reading that arrived but is unusable is not an error here; the
  /// validity checks on [`Shots`] judge it.
  async fn measure(&mut self) -> Result<Shots, SensorFault>;
}
