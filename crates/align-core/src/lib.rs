//! Alignment logic for the two steering mirrors of a flying-optics CO2 laser.
//!
//! Everything here is `no_std`, free of HAL types, and tested on the host. The firmware and the simulator are thin
//! shells around it. This crate holds, so far (milestones M1, M2, and the start of M5):
//!
//! - [`axis`]: the four motorized adjusters and their fixed order;
//! - [`config`]: every tunable value, each with the evidence tag of its default;
//! - [`units`]: conversions from motor steps to screw travel, mirror tilt, and beam displacement;
//! - [`quadrant`]: quadrant sensor normalization and the validity checks on a reading;
//! - [`measurement`]: the three-position difference vector the solver drives to zero;
//! - [`solver`]: conditioning, the damped least squares step, the Broyden update, and finding an unresponsive axis;
//! - [`rig`]: the [`rig::Actuator`] and [`rig::BeamSensor`] traits, the two seams to the machine;
//! - [`approach`]: the standard approach that ends every move in the plus direction, and the soft limits;
//! - [`calibration`]: the empirical Jacobian from pokes, and its acceptance checks;
//! - [`aligner`]: the procedure that calibrates and iterates to zero. The closed-loop tests that hold it to a
//!   model of the machine are in the `align-sim` crate;
//! - [`coordinator`]: the coordinator state machine of reference 7.3, which gates the relay and the motors.
//!
//! The source documents are `docs/REFERENCE.md` ("the reference") and `docs/ADDENDUM_A_28BYJ48.md` ("the
//! addendum"). Their evidence tags are kept in the doc comments: VERIFIED, CALCULATED, UNVERIFIED, AS MODELED,
//! and ASSUMED. A value that is not VERIFIED or CALCULATED lives in [`config`], never in a literal.
#![no_std]

pub mod aligner;
pub mod approach;
pub mod axis;
pub mod calibration;
pub mod config;
pub mod coordinator;
pub mod measurement;
pub mod quadrant;
pub mod rig;
pub mod solver;
pub mod units;

#[cfg(test)]
mod test_support {
  //! Helpers shared by the unit tests.

  /// Asserts that `actual` is within `tolerance` of `expected`.
  #[track_caller]
  pub fn assert_close(actual: f32, expected: f32, tolerance: f32) {
    assert!(
      (actual - expected).abs() <= tolerance,
      "expected {expected} within {tolerance}, got {actual}"
    );
  }
}
