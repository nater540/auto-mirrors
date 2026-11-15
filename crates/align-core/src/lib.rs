//! Alignment logic for the two steering mirrors of a flying-optics CO2 laser.
//!
//! Everything here is `no_std`, free of HAL types, and tested on the host. The firmware and the simulator are thin
//! shells around it. This crate holds, so far (milestone M1):
//!
//! - [`axis`]: the four motorized adjusters and their fixed order;
//! - [`config`]: every tunable value, each with the evidence tag of its default;
//! - [`units`]: conversions from motor steps to screw travel, mirror tilt, and beam displacement;
//! - [`quadrant`]: quadrant sensor normalization and the validity checks on a reading;
//! - [`measurement`]: the three-position difference vector the solver drives to zero;
//! - [`solver`]: conditioning, the damped least squares step, and the Broyden update.
//!
//! The source documents are `docs/REFERENCE.md` ("the reference") and `docs/ADDENDUM_A_28BYJ48.md` ("the
//! addendum"). Their evidence tags are kept in the doc comments: VERIFIED, CALCULATED, UNVERIFIED, AS MODELED,
//! and ASSUMED. A value that is not VERIFIED or CALCULATED lives in [`config`], never in a literal.
#![no_std]

pub mod axis;
pub mod config;
pub mod measurement;
pub mod quadrant;
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
