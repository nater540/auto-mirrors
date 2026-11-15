//! Alignment logic for the two steering mirrors of a flying-optics CO2 laser.
//!
//! Everything here is `no_std`, free of HAL types, and tested on the host. The firmware and the simulator are thin
//! shells around it.
#![no_std]

pub mod axis;
pub mod config;

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
