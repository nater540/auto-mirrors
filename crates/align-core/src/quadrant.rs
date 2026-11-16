//! Quadrant sensor math: normalized beam position and the validity checks on a reading (reference 2.1 and 2.3).
//!
//! ```text
//!   B | A          S = A + B + C + D
//!   --+--          H = ((A + D) - (B + C)) / S
//!   C | D          V = ((A + B) - (C + D)) / S
//! ```
//!
//! The sensor's own X and Y are called `h` (horizontal) and `v` (vertical) in this crate, because X and Y already
//! name the gantry axes. The sensor node reports `h`, `v`, and the sum over CAN; the quadrant form exists for
//! that node and for the simulator. A failed check is a value, never a panic.

use crate::config::SensorLimits;

/// Baseline-subtracted, integrated signals of the four quadrants, in the sensor's own units.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quadrants {
  /// Upper right.
  pub a: f32,
  /// Upper left.
  pub b: f32,
  /// Lower left.
  pub c: f32,
  /// Lower right.
  pub d: f32
}

/// One beam position reading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reading {
  /// Horizontal position, normalized to `[-1, 1]`.
  pub h_norm: f32,
  /// Vertical position, normalized to `[-1, 1]`.
  pub v_norm: f32,
  /// Sum of the four quadrants, in the sensor's own units.
  pub sum: f32
}

/// Why a reading was rejected.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ReadingFault {
  /// A signal is NaN or infinite.
  #[error("sensor signal is not finite")]
  NotFinite,
  /// The sum is zero or negative, so no position can be formed.
  #[error("sensor sum is not positive: no beam on the sensor")]
  NoSignal,
  /// The sum is below the required fraction of the reference: off the sensor, clipped, or no fire.
  #[error("sensor sum is below the minimum: beam off the sensor, clipped, or no fire")]
  SumLow,
  /// A quadrant is at or above the saturation level: gain too high.
  #[error("a quadrant is saturated")]
  Saturated,
  /// A quadrant is further below zero than noise explains: baseline error.
  #[error("a quadrant is negative beyond noise: baseline error")]
  NegativeQuadrant,
  /// The beam is at the edge of the sensor.
  #[error("beam is at the edge of the sensor")]
  AtEdge,
  /// Two shots of the same spot disagree: creep or a mode hop.
  #[error("two shots disagree beyond noise")]
  NotRepeatable,
  /// The sum changed too much from the previous shot: misfire or upstream clipping.
  #[error("sensor sum changed from the previous shot")]
  EnergyChanged
}

impl Quadrants {
  /// Sum of the four quadrants.
  pub fn sum(&self) -> f32 {
    self.a + self.b + self.c + self.d
  }

  /// Normalized position from the four quadrants.
  ///
  /// # Errors
  ///
  /// [`ReadingFault::NotFinite`] if a quadrant is NaN or infinite,
  /// and [`ReadingFault::NoSignal`] if the sum is not positive.
  pub fn normalize(&self) -> Result<Reading, ReadingFault> {
    let sum = self.sum();
    if !sum.is_finite() {
      return Err(ReadingFault::NotFinite);
    }
    if sum <= 0.0 {
      return Err(ReadingFault::NoSignal);
    }
    Ok(Reading {
      h_norm: ((self.a + self.d) - (self.b + self.c)) / sum,
      v_norm: ((self.a + self.b) - (self.c + self.d)) / sum,
      sum
    })
  }

  /// Checks the raw quadrants: none saturated, none negative beyond noise.
  ///
  /// `full_scale` is the largest signal a quadrant can report and `sigma_noise` its noise,
  /// both in the sensor's own units.
  ///
  /// # Errors
  ///
  /// [`ReadingFault::NotFinite`], [`ReadingFault::Saturated`], or [`ReadingFault::NegativeQuadrant`].
  pub fn check(&self, full_scale: f32, sigma_noise: f32, limits: &SensorLimits) -> Result<(), ReadingFault> {
    let quadrants = [self.a, self.b, self.c, self.d];
    if quadrants.iter().any(|q| !q.is_finite()) {
      return Err(ReadingFault::NotFinite);
    }
    if quadrants.iter().any(|&q| q >= limits.saturation_frac * full_scale) {
      return Err(ReadingFault::Saturated);
    }
    if quadrants.iter().any(|&q| q < -limits.negative_sigma * sigma_noise) {
      return Err(ReadingFault::NegativeQuadrant);
    }
    Ok(())
  }
}

impl Reading {
  /// Checks one reading on its own: finite, a positive sum, enough signal, and not at the edge.
  ///
  /// `sum_ref` is the commissioning reference sum for the power in use. The comparison with it is written so that
  /// a NaN threshold rejects every reading instead of passing every one.
  ///
  /// # Errors
  ///
  /// [`ReadingFault::NotFinite`], [`ReadingFault::NoSignal`], [`ReadingFault::SumLow`], or [`ReadingFault::AtEdge`].
  pub fn check(&self, sum_ref: f32, limits: &SensorLimits) -> Result<(), ReadingFault> {
    if !(self.h_norm.is_finite() && self.v_norm.is_finite() && self.sum.is_finite()) {
      return Err(ReadingFault::NotFinite);
    }
    if self.sum <= 0.0 {
      return Err(ReadingFault::NoSignal);
    }
    let enough = matches!(
      self.sum.partial_cmp(&(limits.sum_min_frac * sum_ref)),
      Some(core::cmp::Ordering::Greater | core::cmp::Ordering::Equal)
    );
    if !enough {
      return Err(ReadingFault::SumLow);
    }
    if self.h_norm.abs() > limits.edge_max_norm || self.v_norm.abs() > limits.edge_max_norm {
      return Err(ReadingFault::AtEdge);
    }
    Ok(())
  }

  /// Whether the reading is inside the range the solver may use as a number. Outside it, only the sign of the
  /// position is meaningful (reference 2.2).
  pub fn in_solver_range(&self, limits: &SensorLimits) -> bool {
    self.h_norm.abs() <= limits.solver_max_norm && self.v_norm.abs() <= limits.solver_max_norm
  }

  /// Checks that a second shot of the same spot agrees with this one.
  ///
  /// `sigma_norm` is the repeatability of one normalized coordinate.
  ///
  /// # Errors
  ///
  /// [`ReadingFault::NotRepeatable`] if either coordinate differs by more than the configured number of sigmas.
  pub fn check_repeat(&self, second: &Reading, sigma_norm: f32, limits: &SensorLimits) -> Result<(), ReadingFault> {
    let allowed = limits.repeat_sigma * sigma_norm;
    let agrees = |first: f32, again: f32| (first - again).abs() <= allowed;
    if agrees(self.h_norm, second.h_norm) && agrees(self.v_norm, second.v_norm) {
      Ok(())
    } else {
      Err(ReadingFault::NotRepeatable)
    }
  }

  /// Checks that the sum is close to the previous shot's at the same power.
  ///
  /// # Errors
  ///
  /// [`ReadingFault::EnergyChanged`] if the sum moved by more than the configured fraction of the previous sum.
  pub fn check_energy(&self, previous_sum: f32, limits: &SensorLimits) -> Result<(), ReadingFault> {
    if (self.sum - previous_sum).abs() <= limits.energy_tolerance_frac * previous_sum {
      Ok(())
    } else {
      Err(ReadingFault::EnergyChanged)
    }
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;
  use crate::test_support::assert_close;

  const LIMITS: SensorLimits = SensorLimits::STARTING;

  fn reading(h_norm: f32, v_norm: f32, sum: f32) -> Reading {
    Reading { h_norm, v_norm, sum }
  }

  #[test]
  fn a_centered_beam_reads_zero() {
    let reading = Quadrants {
      a: 1.0,
      b: 1.0,
      c: 1.0,
      d: 1.0
    }
    .normalize()
    .unwrap();
    assert_close(reading.h_norm, 0.0, 0.0);
    assert_close(reading.v_norm, 0.0, 0.0);
    assert_close(reading.sum, 4.0, 0.0);
  }

  #[test]
  fn each_quadrant_alone_reads_its_own_corner() {
    let corner = |a, b, c, d| {
      let reading = Quadrants { a, b, c, d }.normalize().unwrap();
      (reading.h_norm, reading.v_norm)
    };
    assert_eq!(corner(1.0, 0.0, 0.0, 0.0), (1.0, 1.0));
    assert_eq!(corner(0.0, 1.0, 0.0, 0.0), (-1.0, 1.0));
    assert_eq!(corner(0.0, 0.0, 1.0, 0.0), (-1.0, -1.0));
    assert_eq!(corner(0.0, 0.0, 0.0, 1.0), (1.0, -1.0));
  }

  #[test]
  fn the_position_is_the_conventional_quadrant_formula() {
    let reading = Quadrants {
      a: 4.0,
      b: 3.0,
      c: 1.0,
      d: 2.0
    }
    .normalize()
    .unwrap();
    assert_close(reading.h_norm, (4.0 + 2.0 - 3.0 - 1.0) / 10.0, 1e-6);
    assert_close(reading.v_norm, (4.0 + 3.0 - 1.0 - 2.0) / 10.0, 1e-6);
  }

  #[test]
  fn no_signal_is_a_fault_and_not_a_division_by_zero() {
    let dark = Quadrants {
      a: 0.0,
      b: 0.0,
      c: 0.0,
      d: 0.0
    };
    assert_eq!(dark.normalize(), Err(ReadingFault::NoSignal));
    let negative = Quadrants {
      a: -1.0,
      b: 0.0,
      c: 0.0,
      d: 0.5
    };
    assert_eq!(negative.normalize(), Err(ReadingFault::NoSignal));
  }

  #[test]
  fn a_quadrant_that_is_not_a_number_is_a_fault() {
    let broken = Quadrants {
      a: f32::NAN,
      b: 1.0,
      c: 1.0,
      d: 1.0
    };
    assert_eq!(broken.normalize(), Err(ReadingFault::NotFinite));
    assert_eq!(broken.check(10.0, 0.01, &LIMITS), Err(ReadingFault::NotFinite));
    let infinite = Quadrants {
      a: f32::INFINITY,
      b: 1.0,
      c: 1.0,
      d: 1.0
    };
    assert_eq!(infinite.normalize(), Err(ReadingFault::NotFinite));
  }

  #[test]
  fn a_quadrant_at_nine_tenths_of_full_scale_is_saturated() {
    let below = Quadrants {
      a: 8.9,
      b: 1.0,
      c: 1.0,
      d: 1.0
    };
    assert_eq!(below.check(10.0, 0.01, &LIMITS), Ok(()));
    let at = Quadrants {
      a: 9.0,
      b: 1.0,
      c: 1.0,
      d: 1.0
    };
    assert_eq!(at.check(10.0, 0.01, &LIMITS), Err(ReadingFault::Saturated));
  }

  #[test]
  fn a_quadrant_more_than_three_sigma_below_zero_is_a_baseline_error() {
    let inside = Quadrants {
      a: 1.0,
      b: -0.029,
      c: 1.0,
      d: 1.0
    };
    assert_eq!(inside.check(10.0, 0.01, &LIMITS), Ok(()));
    let outside = Quadrants {
      a: 1.0,
      b: -0.031,
      c: 1.0,
      d: 1.0
    };
    assert_eq!(outside.check(10.0, 0.01, &LIMITS), Err(ReadingFault::NegativeQuadrant));
  }

  #[test]
  fn a_sum_below_three_tenths_of_the_reference_is_rejected() {
    assert_eq!(reading(0.0, 0.0, 31.0).check(100.0, &LIMITS), Ok(()));
    assert_eq!(reading(0.0, 0.0, 29.0).check(100.0, &LIMITS), Err(ReadingFault::SumLow));
  }

  #[test]
  fn a_sum_that_is_not_positive_is_no_signal_whatever_the_reference() {
    for sum_ref in [100.0, 0.0, -1.0] {
      assert_eq!(
        reading(0.0, 0.0, 0.0).check(sum_ref, &LIMITS),
        Err(ReadingFault::NoSignal)
      );
      assert_eq!(
        reading(0.0, 0.0, -5.0).check(sum_ref, &LIMITS),
        Err(ReadingFault::NoSignal)
      );
    }
    assert_eq!(reading(0.0, 0.0, f32::MIN_POSITIVE).check(0.0, &LIMITS), Ok(()));
  }

  #[test]
  fn a_reference_sum_that_is_not_a_number_rejects_every_reading() {
    assert_eq!(
      reading(0.0, 0.0, 100.0).check(f32::NAN, &LIMITS),
      Err(ReadingFault::SumLow)
    );
  }

  #[test]
  fn a_position_beyond_nine_tenths_is_at_the_edge() {
    assert_eq!(reading(0.89, -0.89, 100.0).check(100.0, &LIMITS), Ok(()));
    assert_eq!(
      reading(0.9, -0.9, 100.0).check(100.0, &LIMITS),
      Ok(()),
      "the edge itself is inside"
    );
    assert_eq!(
      reading(0.91, 0.0, 100.0).check(100.0, &LIMITS),
      Err(ReadingFault::AtEdge)
    );
    assert_eq!(
      reading(0.0, -0.91, 100.0).check(100.0, &LIMITS),
      Err(ReadingFault::AtEdge)
    );
  }

  #[test]
  fn a_reading_that_is_not_a_number_is_a_fault() {
    assert_eq!(
      reading(f32::NAN, 0.0, 100.0).check(100.0, &LIMITS),
      Err(ReadingFault::NotFinite)
    );
    assert_eq!(
      reading(0.0, 0.0, f32::NAN).check(100.0, &LIMITS),
      Err(ReadingFault::NotFinite)
    );
  }

  #[test]
  fn the_solver_uses_a_reading_only_inside_six_tenths() {
    assert!(reading(0.59, -0.59, 100.0).in_solver_range(&LIMITS));
    assert!(!reading(0.61, 0.0, 100.0).in_solver_range(&LIMITS));
    assert!(!reading(0.0, -0.61, 100.0).in_solver_range(&LIMITS));
    assert!(!reading(f32::NAN, 0.0, 100.0).in_solver_range(&LIMITS));
  }

  #[test]
  fn two_shots_must_agree_within_three_sigma() {
    let first = reading(0.10, 0.10, 100.0);
    assert_eq!(first.check_repeat(&reading(0.129, 0.071, 100.0), 0.01, &LIMITS), Ok(()));
    assert_eq!(
      first.check_repeat(&reading(0.131, 0.10, 100.0), 0.01, &LIMITS),
      Err(ReadingFault::NotRepeatable)
    );
    assert_eq!(
      first.check_repeat(&reading(0.10, 0.069, 100.0), 0.01, &LIMITS),
      Err(ReadingFault::NotRepeatable)
    );
    assert_eq!(
      first.check_repeat(&reading(f32::NAN, 0.10, 100.0), 0.01, &LIMITS),
      Err(ReadingFault::NotRepeatable)
    );
  }

  #[test]
  fn the_sum_must_stay_within_thirty_percent_of_the_previous_shot() {
    assert_eq!(reading(0.0, 0.0, 129.0).check_energy(100.0, &LIMITS), Ok(()));
    assert_eq!(reading(0.0, 0.0, 71.0).check_energy(100.0, &LIMITS), Ok(()));
    assert_eq!(
      reading(0.0, 0.0, 131.0).check_energy(100.0, &LIMITS),
      Err(ReadingFault::EnergyChanged)
    );
    assert_eq!(
      reading(0.0, 0.0, 69.0).check_energy(100.0, &LIMITS),
      Err(ReadingFault::EnergyChanged)
    );
    assert_eq!(
      reading(0.0, 0.0, f32::NAN).check_energy(100.0, &LIMITS),
      Err(ReadingFault::EnergyChanged)
    );
  }

  proptest! {
    #[test]
    fn non_negative_quadrants_with_signal_always_normalize_inside_the_unit_square(
      a in 0.0_f32..1.0e6, b in 0.0_f32..1.0e6, c in 0.0_f32..1.0e6, d in 0.0_f32..1.0e6,
    ) {
      prop_assume!(a + b + c + d > 0.0);
      let reading = Quadrants { a, b, c, d }.normalize().unwrap();
      prop_assert!((-1.0..=1.0).contains(&reading.h_norm));
      prop_assert!((-1.0..=1.0).contains(&reading.v_norm));
    }

    #[test]
    fn the_position_does_not_depend_on_pulse_energy(
      a in 0.01_f32..100.0, b in 0.01_f32..100.0, c in 0.01_f32..100.0, d in 0.01_f32..100.0,
      gain in 0.1_f32..10.0,
    ) {
      let one = Quadrants { a, b, c, d }.normalize().unwrap();
      let scaled = Quadrants { a: a * gain, b: b * gain, c: c * gain, d: d * gain }.normalize().unwrap();
      prop_assert!((one.h_norm - scaled.h_norm).abs() <= 1e-4);
      prop_assert!((one.v_norm - scaled.v_norm).abs() <= 1e-4);
    }
  }
}
