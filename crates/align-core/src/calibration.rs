//! The empirical Jacobian: built from pokes, then checked (reference 3.1).
//!
//! The Jacobian is never derived from geometry. Each axis is poked by a known number of steps and the change in
//! the measurement vector becomes its column, which absorbs mount orientation, signs, cross-coupling, and the
//! sensor slope. This module holds the arithmetic and the acceptance checks; [`crate::aligner`] runs the moves.

use core::f32::consts::SQRT_2;

use nalgebra::Vector4;

use crate::axis::Axis;
use crate::config::{SensorLimits, Solver};
use crate::solver::Jacobian;

/// An accepted Jacobian and where it was measured. This is the record that is persisted after calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Calibration {
  /// Normalized sensor units per full step.
  pub jacobian: Jacobian,
  /// Commanded position of every axis when the baseline was measured, in steps.
  pub q0_steps: [i32; 4],
  /// Condition number of the Jacobian when it was accepted.
  pub cond: f32,
}

/// The Jacobian from a baseline measurement and the measurement after poking each axis by `poke_steps`:
/// column `j` is `(m_j - m_0) / poke_steps`.
pub fn jacobian_from_pokes(baseline: &Vector4<f32>, poked: &[Vector4<f32>; 4], poke_steps: f32) -> Jacobian {
  Jacobian::from_columns(&poked.map(|after| (after - baseline) / poke_steps))
}

/// The smallest change of the measurement vector that counts as signal: `min_column_sigmas` sigmas of one
/// component. A component is a difference of two readings, so its sigma is `sqrt(2)` times the sigma of one
/// normalized coordinate.
///
/// The reference states this floor for calibration columns (3.1). The aligner applies the same floor before it
/// learns from, or passes judgment on, a solver step.
pub fn noise_floor(sensor: &SensorLimits, solver: &Solver) -> f32 {
  solver.min_column_sigmas * SQRT_2 * sensor.noise_sigma_norm
}

/// The first axis whose poke moved the measurement by less than the [`noise_floor`], if any. Such a column is
/// noise: the axis is dead, disconnected, or poked too little.
pub fn weak_column(jacobian: &Jacobian, poke_steps: f32, sensor: &SensorLimits, solver: &Solver) -> Option<Axis> {
  let floor = noise_floor(sensor, solver);
  Axis::ALL
    .into_iter()
    .zip(jacobian.column_iter())
    .find_map(|(axis, column)| {
      let change = column.norm() * poke_steps.abs();
      // Written so that a NaN counts as weak.
      (!matches!(
        change.partial_cmp(&floor),
        Some(core::cmp::Ordering::Greater | core::cmp::Ordering::Equal)
      ))
      .then_some(axis)
    })
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::test_support::assert_close;

  const SENSOR: SensorLimits = SensorLimits::STARTING;
  const SOLVER: Solver = Solver::STARTING;

  fn true_jacobian() -> Jacobian {
    Jacobian::new(
      2.8e-3, 0.0, 0.0, 0.0, //
      1.0e-4, 2.0e-3, 0.0, 0.0, //
      3.4e-3, 0.0, 3.4e-3, 0.0, //
      0.0, 2.4e-3, -1.0e-4, 2.4e-3,
    )
  }

  #[test]
  fn poking_each_axis_of_a_linear_system_recovers_its_jacobian() {
    let jacobian = true_jacobian();
    let baseline = Vector4::new(0.11, -0.07, 0.20, 0.03);
    let poke = 45.0;
    let poked = [0, 1, 2, 3].map(|axis| {
      let mut step = Vector4::zeros();
      step[axis] = poke;
      baseline + jacobian * step
    });
    let measured = jacobian_from_pokes(&baseline, &poked, poke);
    assert!((measured - jacobian).amax() <= 1e-6, "{measured}");
  }

  #[test]
  fn a_baseline_that_is_not_zero_does_not_enter_the_jacobian() {
    let jacobian = true_jacobian();
    let poke = 45.0;
    let build = |baseline: Vector4<f32>| {
      let poked = [0, 1, 2, 3].map(|axis| {
        let mut step = Vector4::zeros();
        step[axis] = poke;
        baseline + jacobian * step
      });
      jacobian_from_pokes(&baseline, &poked, poke)
    };
    let centered = build(Vector4::zeros());
    let offset = build(Vector4::new(0.3, -0.3, 0.2, 0.1));
    assert!((centered - offset).amax() <= 1e-6);
  }

  #[test]
  fn a_healthy_jacobian_has_no_weak_column() {
    assert_eq!(weak_column(&true_jacobian(), 45.0, &SENSOR, &SOLVER), None);
  }

  #[test]
  fn an_axis_that_did_not_move_the_beam_is_a_weak_column() {
    for dead in Axis::ALL {
      let mut jacobian = true_jacobian();
      jacobian.set_column(dead.index(), &Vector4::zeros());
      assert_eq!(weak_column(&jacobian, 45.0, &SENSOR, &SOLVER), Some(dead));
    }
  }

  #[test]
  fn the_weak_column_floor_is_five_sigmas_of_a_measurement_component() {
    // 5 * sqrt(2) * 0.005 = 0.0354 of measurement change over the poke.
    assert_close(noise_floor(&SENSOR, &SOLVER), 0.0354, 0.0001);
    let mut jacobian = true_jacobian();
    jacobian.set_column(3, &Vector4::new(0.0, 0.0, 0.0, 0.036 / 45.0));
    assert_eq!(weak_column(&jacobian, 45.0, &SENSOR, &SOLVER), None);
    jacobian.set_column(3, &Vector4::new(0.0, 0.0, 0.0, 0.035 / 45.0));
    assert_eq!(weak_column(&jacobian, 45.0, &SENSOR, &SOLVER), Some(Axis::M2V));
  }

  #[test]
  fn a_column_that_is_not_a_number_is_weak() {
    let mut jacobian = true_jacobian();
    jacobian[(0, 1)] = f32::NAN;
    assert_eq!(weak_column(&jacobian, 45.0, &SENSOR, &SOLVER), Some(Axis::M1V));
  }
}
