//! The solver's linear algebra: conditioning, the damped least squares step, and the Broyden update (hand-off
//! 3.2, reference 3.2, addendum A.5).
//!
//! ```text
//! dq = alpha * (J^T J + lambda^2 I)^-1 J^T (-m)        lambda = lambda_frac * sigma_max(J)
//! J <- J + ((dm - J dq) dq^T) / (dq^T dq)              after each accepted step
//! ```
//!
//! `J` is the empirical 4x4 Jacobian, in normalized sensor units per full step, with rows in the order of the
//! measurement vector and columns in the axis order. These are the pure pieces; the iteration loop that strings
//! them together belongs to the calibration and state machine code of later milestones. Every failure is a
//! [`SolveFault`] value: nothing here panics on a singular, ill-conditioned, or non-finite input.
//!
//! nalgebra 0.35.0 builds its fixed-size SVD and Cholesky for `riscv32imac-unknown-none-elf` without `std` and
//! without `alloc` (checked for the pinned version; see `docs/VERSIONS.md`), so no hand-written SVD is needed.

use nalgebra::{Matrix4, SVD, Vector4};

use crate::axis::Axis;
use crate::config::Solver;

/// The empirical Jacobian: normalized sensor units per full step.
pub type Jacobian = Matrix4<f32>;

/// Iteration cap handed to the SVD. A 4x4 converges in a handful of sweeps; the cap only bounds the worst case.
const SVD_MAX_ITERATIONS: usize = 64;

/// Why a solver operation produced no result.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum SolveFault {
  /// An input or a result holds a NaN or an infinity.
  #[error("solver input or result is not finite")]
  NotFinite,
  /// The singular value decomposition did not converge within its iteration cap.
  #[error("singular value decomposition did not converge")]
  SvdDidNotConverge,
  /// The Jacobian has a zero singular value: some combination of axes does not move the beam.
  #[error("Jacobian is singular")]
  Singular,
  /// The condition number is above the configured limit.
  #[error("Jacobian condition number {cond} is above the limit {limit}")]
  IllConditioned {
    /// The condition number found.
    cond: f32,
    /// The configured limit.
    limit: f32
  },
  /// The damped normal matrix has no Cholesky factor. With positive damping this means a numerical breakdown.
  #[error("damped normal matrix is not positive definite")]
  NotPositiveDefinite
}

/// The extreme singular values of a Jacobian.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Conditioning {
  /// Largest singular value.
  pub sigma_max: f32,
  /// Smallest singular value.
  pub sigma_min: f32,
  /// Condition number, `sigma_max / sigma_min`.
  pub cond: f32
}

impl Conditioning {
  /// Singular values and condition number of a Jacobian.
  ///
  /// # Errors
  ///
  /// [`SolveFault::NotFinite`] for a non-finite entry, [`SolveFault::SvdDidNotConverge`], or
  /// [`SolveFault::Singular`] when the smallest singular value is zero.
  pub fn of(jacobian: &Jacobian) -> Result<Self, SolveFault> {
    if !all_finite(jacobian.iter()) {
      return Err(SolveFault::NotFinite);
    }
    let singular_values = SVD::try_new(*jacobian, false, false, f32::EPSILON, SVD_MAX_ITERATIONS)
      .ok_or(SolveFault::SvdDidNotConverge)?
      .singular_values;
    let sigma_max = singular_values.max();
    let sigma_min = singular_values.min();
    if !(sigma_max.is_finite() && sigma_min.is_finite()) {
      return Err(SolveFault::NotFinite);
    }
    // A singular value is never negative; a matrix of rank below four has one at zero up to rounding.
    if sigma_min <= sigma_max * f32::EPSILON {
      return Err(SolveFault::Singular);
    }
    Ok(Self {
      sigma_max,
      sigma_min,
      cond: sigma_max / sigma_min
    })
  }

  /// Rejects a Jacobian whose condition number is above the configured limit. A NaN limit rejects every one.
  ///
  /// # Errors
  ///
  /// [`SolveFault::IllConditioned`] with the condition number and the limit.
  pub fn check(&self, config: &Solver) -> Result<(), SolveFault> {
    let inside = matches!(
      self.cond.partial_cmp(&config.cond_max),
      Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)
    );
    if !inside {
      return Err(SolveFault::IllConditioned {
        cond: self.cond,
        limit: config.cond_max
      });
    }
    Ok(())
  }

  /// The starting damping for this Jacobian: `lambda_frac * sigma_max`.
  pub fn damping(&self, config: &Solver) -> f32 {
    config.lambda_frac * self.sigma_max
  }
}

/// One damped least squares step toward `m = 0`, in fractional full steps.
///
/// `dq = alpha * (J^T J + lambda^2 I)^-1 J^T (-m)`, then each component is clamped to `[-dq_max, dq_max]`. An
/// infinite `dq_max` component leaves that axis unclamped.
///
/// # Errors
///
/// [`SolveFault::NotFinite`] if an input or the step is not finite, and [`SolveFault::NotPositiveDefinite`] if
/// the damped normal matrix cannot be factored.
pub fn dls_step(
  jacobian: &Jacobian,
  measurement: &Vector4<f32>,
  lambda: f32,
  alpha: f32,
  dq_max: &Vector4<f32>
) -> Result<Vector4<f32>, SolveFault> {
  let inputs_finite = all_finite(jacobian.iter())
    && all_finite(measurement.iter())
    && !dq_max.iter().any(|limit| limit.is_nan())
    && lambda.is_finite()
    && alpha.is_finite();
  if !inputs_finite {
    return Err(SolveFault::NotFinite);
  }
  let transposed = jacobian.transpose();
  let normal = transposed * jacobian + Matrix4::identity() * (lambda * lambda);
  let factor = normal.cholesky().ok_or(SolveFault::NotPositiveDefinite)?;
  let unclamped = factor.solve(&(transposed * -measurement)) * alpha;
  // `max` then `min` instead of `clamp`, which panics when its bounds are reversed.
  let dq = unclamped.zip_map(dq_max, |step, limit| step.max(-limit.abs()).min(limit.abs()));
  if !all_finite(dq.iter()) {
    return Err(SolveFault::NotFinite);
  }
  Ok(dq)
}

/// The per-axis step clamp: `step_clamp_pokes` poke sizes on every axis (reference 3.2).
pub fn step_limit(poke_steps: f32, config: &Solver) -> Vector4<f32> {
  Vector4::repeat(config.step_clamp_pokes * poke_steps)
}

/// The Broyden rank-one update of the Jacobian from an accepted step `dq` and the measured change `dm`.
///
/// Returns `None` when the step is shorter than `broyden_min_step_norm_steps` (too small to carry information)
/// or when an input is not finite; the caller keeps the old Jacobian. The caller also checks the conditioning of
/// the result and discards an update that fails it.
pub fn broyden_update(jacobian: &Jacobian, dq: &Vector4<f32>, dm: &Vector4<f32>, config: &Solver) -> Option<Jacobian> {
  if !(all_finite(jacobian.iter()) && all_finite(dq.iter()) && all_finite(dm.iter())) {
    return None;
  }
  let step_squared = dq.norm_squared();
  let min_norm = config.broyden_min_step_norm_steps;
  if step_squared <= 0.0 || step_squared < min_norm * min_norm {
    return None;
  }
  let updated = jacobian + (dm - jacobian * dq) * dq.transpose() / step_squared;
  all_finite(updated.iter()).then_some(updated)
}

/// Whether the measured change `dm` disagrees with the prediction `J dq` enough to mark the move suspect:
/// `norm(dm - J dq) > frac * norm(J dq)` (addendum A.5). This is the only stall detection the steppers have.
///
/// A non-finite input counts as a mismatch: a move that cannot be confirmed is not trusted.
pub fn prediction_mismatch(jacobian: &Jacobian, dq: &Vector4<f32>, dm: &Vector4<f32>, config: &Solver) -> bool {
  let predicted = jacobian * dq;
  let error_norm = (dm - predicted).norm();
  let allowed = config.prediction_mismatch_frac * predicted.norm();
  // Written so that a NaN on either side is a mismatch.
  !matches!(
    error_norm.partial_cmp(&allowed),
    Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)
  )
}

/// The axis that did not respond to its command, if there is one, judged from the measured change.
///
/// Addendum A.5 says a mismatch between the measured change and the prediction marks "the axis" suspect. It does
/// not say which axis when four moved together, and a plain mismatch also arises from a Jacobian that is merely
/// wrong: a Jacobian 30% too small predicts 70% of the real change, a mismatch of 43%. So the mismatch is
/// attributed, and only an axis that fell short counts.
///
/// `J^-1 dm` is the step each axis appears to have taken. An axis is unresponsive when
///
/// - its command stands clear of the noise in its apparent step: the noise of `dm` (each component of sigma
///   `change_sigma`) reaches the apparent step of axis `j` amplified by `norm(row_j(J^-1))`, and the command must
///   exceed `judge_noise_sigmas` of that noise by the margin the test below allows in either direction, so that
///   neither a healthy axis is named nor a dead one missed by noise alone; and
/// - it appears to have taken less than `1 - prediction_mismatch_frac` of its command.
///
/// Of several such axes the one with the largest missing contribution is returned. An axis that did not move at
/// all appears to have taken zero steps while every other axis appears to have taken its command.
///
/// Returns `None` when no axis qualifies, when the Jacobian cannot be inverted, or when an input is not finite.
pub fn unresponsive_axis(
  jacobian: &Jacobian,
  dq: &Vector4<f32>,
  dm: &Vector4<f32>,
  change_sigma: f32,
  config: &Solver
) -> Option<Axis> {
  if !(all_finite(jacobian.iter()) && all_finite(dq.iter()) && all_finite(dm.iter())) {
    return None;
  }
  let inverse = jacobian.try_inverse()?;
  let apparent = inverse * dm;
  let margin = config
    .prediction_mismatch_frac
    .min(1.0 - config.prediction_mismatch_frac);
  let mut worst: Option<(Axis, f32)> = None;
  for (((axis, column), row), (commanded, taken)) in Axis::ALL
    .into_iter()
    .zip(jacobian.column_iter())
    .zip(inverse.row_iter())
    .zip(dq.iter().zip(apparent.iter()))
  {
    let gain = column.norm();
    let apparent_sigma = change_sigma * row.norm();
    let visible = margin * commanded.abs() >= config.judge_noise_sigmas * apparent_sigma;
    // The fraction of the command that is missing: 0 for a perfect response, 1 for none, above 1 for a reversed
    // one. Dividing is safe: a visible command is not zero.
    let missing_frac = (commanded - taken) / commanded;
    if visible && missing_frac > config.prediction_mismatch_frac {
      let missing = (commanded - taken).abs() * gain;
      if worst.is_none_or(|(_, largest)| missing > largest) {
        worst = Some((axis, missing));
      }
    }
  }
  worst.map(|(axis, _)| axis)
}

/// A whole-step vector as the `f32` vector the solver works in.
pub fn steps_to_vector(steps: [i32; 4]) -> Vector4<f32> {
  #[expect(
    clippy::cast_precision_loss,
    reason = "a step count is clamped to a few thousand, far inside the exact range of f32"
  )]
  let [m1_h, m1_v, m2_h, m2_v] = steps.map(|count| count as f32);
  Vector4::new(m1_h, m1_v, m2_h, m2_v)
}

/// Whether the measurement vector is inside the convergence tolerance: `norm_inf(m) <= converge_tol_norm`.
/// A vector holding a NaN is not converged.
pub fn converged(measurement: &Vector4<f32>, config: &Solver) -> bool {
  measurement
    .iter()
    .all(|component| component.abs() <= config.converge_tol_norm)
}

/// Rounds a fractional step vector to whole full steps, in the fixed axis order.
/// A value outside the `i32` range saturates.
pub fn round_to_steps(dq: &Vector4<f32>) -> [i32; 4] {
  #[expect(
    clippy::cast_possible_truncation,
    reason = "a float to int `as` cast saturates, and steps are clamped far inside i32 before this"
  )]
  let round = |value: f32| libm::roundf(value) as i32;
  [round(dq.x), round(dq.y), round(dq.z), round(dq.w)]
}

/// Whether every value is neither NaN nor infinite.
fn all_finite<'a>(mut values: impl Iterator<Item = &'a f32>) -> bool {
  values.all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;
  use crate::test_support::assert_close;

  const CONFIG: Solver = Solver::STARTING;

  /// The model Jacobian of reference 1.4 for both transverse axes, with `k1*c1 = k2*c2 = gain`:
  /// mirror 1 moves the beam over both legs, mirror 2 over the second leg only.
  fn model_jacobian(dy: f32, dx: f32, gain: f32) -> Jacobian {
    Jacobian::new(
      dy, 0.0, 0.0, 0.0, //
      0.0, dy, 0.0, 0.0, //
      dx, 0.0, dx, 0.0, //
      0.0, dx, 0.0, dx
    ) * gain
  }

  /// A Jacobian in realistic units: about 0.0028 normalized units per step over a 500 mm leg.
  fn realistic_jacobian() -> Jacobian {
    model_jacobian(500.0, 500.0, 5.6e-6)
  }

  fn no_clamp() -> Vector4<f32> {
    Vector4::repeat(f32::INFINITY)
  }

  #[test]
  fn the_condition_numbers_match_the_reference_table() {
    assert_close(
      Conditioning::of(&model_jacobian(500.0, 500.0, 1.0)).unwrap().cond,
      2.62,
      0.005
    );
    assert_close(
      Conditioning::of(&model_jacobian(500.0, 800.0, 1.0)).unwrap().cond,
      3.54,
      0.005
    );
    assert_close(
      Conditioning::of(&model_jacobian(50.0, 800.0, 1.0)).unwrap().cond,
      32.0,
      0.5
    );
  }

  #[test]
  fn the_condition_number_does_not_depend_on_the_units_of_the_jacobian() {
    let coarse = Conditioning::of(&model_jacobian(500.0, 800.0, 1.0)).unwrap();
    let fine = Conditioning::of(&model_jacobian(500.0, 800.0, 5.6e-6)).unwrap();
    assert_close(coarse.cond, fine.cond, 1e-3);
  }

  #[test]
  fn the_condition_number_grows_without_bound_as_the_y_span_closes() {
    let narrow = Conditioning::of(&model_jacobian(5.0, 800.0, 1.0)).unwrap();
    assert!(narrow.cond > 300.0);
    assert_eq!(
      Conditioning::of(&model_jacobian(0.0, 800.0, 1.0)),
      Err(SolveFault::Singular)
    );
  }

  #[test]
  fn a_limit_that_is_not_a_number_rejects_every_jacobian() {
    let acceptable = Conditioning::of(&model_jacobian(50.0, 800.0, 1.0)).unwrap();
    let config = Solver {
      cond_max: f32::NAN,
      ..CONFIG
    };
    assert!(matches!(
      acceptable.check(&config),
      Err(SolveFault::IllConditioned { .. })
    ));
  }

  #[test]
  fn a_condition_number_above_fifty_is_rejected_with_the_number_and_the_limit() {
    let acceptable = Conditioning::of(&model_jacobian(50.0, 800.0, 1.0)).unwrap();
    assert_eq!(acceptable.check(&CONFIG), Ok(()));
    let poor = Conditioning::of(&model_jacobian(20.0, 800.0, 1.0)).unwrap();
    match poor.check(&CONFIG) {
      Err(SolveFault::IllConditioned { cond, limit }) => {
        assert_close(cond, 80.0, 0.5);
        assert_close(limit, 50.0, 0.0);
      }
      other => panic!("expected IllConditioned, got {other:?}")
    }
  }

  #[test]
  fn a_dead_axis_makes_the_jacobian_singular() {
    let mut jacobian = realistic_jacobian();
    jacobian.set_column(2, &Vector4::zeros());
    assert_eq!(Conditioning::of(&jacobian), Err(SolveFault::Singular));
    assert_eq!(Conditioning::of(&Jacobian::zeros()), Err(SolveFault::Singular));
  }

  #[test]
  fn a_jacobian_that_is_not_a_number_is_a_fault() {
    let mut jacobian = realistic_jacobian();
    jacobian[(1, 1)] = f32::NAN;
    assert_eq!(Conditioning::of(&jacobian), Err(SolveFault::NotFinite));
    jacobian[(1, 1)] = f32::INFINITY;
    assert_eq!(Conditioning::of(&jacobian), Err(SolveFault::NotFinite));
  }

  #[test]
  fn the_starting_damping_is_five_percent_of_the_largest_singular_value() {
    let conditioning = Conditioning::of(&model_jacobian(500.0, 500.0, 1.0)).unwrap();
    // The singular values of [[1, 0], [1, 1]] are the golden ratio and its inverse.
    assert_close(conditioning.sigma_max, 500.0 * 1.618_034, 0.01);
    assert_close(conditioning.sigma_min, 500.0 * 0.618_034, 0.01);
    assert_close(conditioning.damping(&CONFIG), 0.05 * conditioning.sigma_max, 1e-4);
  }

  #[test]
  fn an_undamped_full_step_solves_the_linear_system_exactly() {
    let jacobian = realistic_jacobian();
    let error_steps = Vector4::new(120.0, -80.0, 40.0, 200.0);
    let measurement = jacobian * error_steps;
    let dq = dls_step(&jacobian, &measurement, 0.0, 1.0, &no_clamp()).unwrap();
    for (step, error) in dq.iter().zip(error_steps.iter()) {
      assert_close(*step, -error, 0.05);
    }
  }

  #[test]
  fn the_configured_step_applies_six_tenths_of_the_correction_less_the_damping() {
    let jacobian = realistic_jacobian();
    let error_steps = Vector4::new(120.0, -80.0, 40.0, 200.0);
    let measurement = jacobian * error_steps;
    let conditioning = Conditioning::of(&jacobian).unwrap();
    let dq = dls_step(
      &jacobian,
      &measurement,
      conditioning.damping(&CONFIG),
      CONFIG.alpha,
      &no_clamp()
    )
    .unwrap();
    let remaining = (measurement + jacobian * dq).norm() / measurement.norm();
    // Undamped, 0.4 of the residual would remain. Damping at 5% of sigma_max leaves at most about 0.4 + 0.6 *
    // lambda^2 / (sigma_min^2 + lambda^2) = 0.404 at this conditioning.
    assert!((0.4..=0.405).contains(&remaining), "remaining fraction {remaining}");
  }

  #[test]
  fn four_configured_steps_bring_a_half_scale_error_inside_the_tolerance_on_a_well_conditioned_jacobian() {
    let jacobian = realistic_jacobian();
    let conditioning = Conditioning::of(&jacobian).unwrap();
    let mut measurement = Vector4::new(0.5, -0.4, 0.3, 0.5);
    for _ in 0..CONFIG.max_iterations {
      assert!(!converged(&measurement, &CONFIG));
      let dq = dls_step(
        &jacobian,
        &measurement,
        conditioning.damping(&CONFIG),
        CONFIG.alpha,
        &no_clamp()
      )
      .unwrap();
      measurement += jacobian * dq;
    }
    assert!(converged(&measurement, &CONFIG), "residual {measurement:?}");
  }

  #[test]
  fn a_step_is_clamped_to_four_pokes_on_each_axis() {
    let jacobian = realistic_jacobian();
    let measurement = jacobian * Vector4::new(1_000.0, -1_000.0, 10.0, -10.0);
    let limit = step_limit(45.0, &CONFIG);
    assert_eq!(limit, Vector4::repeat(180.0));
    let dq = dls_step(&jacobian, &measurement, 0.0, 1.0, &limit).unwrap();
    assert_close(dq.x, -180.0, 0.0);
    assert_close(dq.y, 180.0, 0.0);
    assert_close(dq.z, -10.0, 0.05);
    assert_close(dq.w, 10.0, 0.05);
  }

  #[test]
  fn a_negative_clamp_is_read_as_its_magnitude_and_does_not_panic() {
    let jacobian = realistic_jacobian();
    let measurement = jacobian * Vector4::repeat(1_000.0);
    let dq = dls_step(&jacobian, &measurement, 0.0, 1.0, &Vector4::repeat(-50.0)).unwrap();
    assert_eq!(dq, Vector4::repeat(-50.0));
  }

  #[test]
  fn a_zero_measurement_asks_for_no_move() {
    let dq = dls_step(
      &realistic_jacobian(),
      &Vector4::zeros(),
      1e-4,
      CONFIG.alpha,
      &no_clamp()
    )
    .unwrap();
    assert_eq!(round_to_steps(&dq), [0, 0, 0, 0]);
  }

  #[test]
  fn a_singular_jacobian_without_damping_is_a_fault_and_with_damping_moves_only_the_live_axes() {
    let mut jacobian = realistic_jacobian();
    jacobian.set_column(3, &Vector4::zeros());
    let measurement = Vector4::new(0.1, 0.1, 0.1, 0.1);
    assert_eq!(
      dls_step(&jacobian, &measurement, 0.0, 1.0, &no_clamp()),
      Err(SolveFault::NotPositiveDefinite)
    );
    let dq = dls_step(&jacobian, &measurement, 1e-4, 1.0, &no_clamp()).unwrap();
    assert_close(dq.w, 0.0, 1e-3);
    assert!(dq.x.abs() > 1.0);
  }

  #[test]
  fn an_input_that_is_not_a_number_is_a_fault() {
    let jacobian = realistic_jacobian();
    let measurement = Vector4::new(0.1, 0.1, 0.1, 0.1);
    let bad_measurement = Vector4::new(f32::NAN, 0.1, 0.1, 0.1);
    assert_eq!(
      dls_step(&jacobian, &bad_measurement, 1e-4, 0.6, &no_clamp()),
      Err(SolveFault::NotFinite)
    );
    assert_eq!(
      dls_step(&jacobian, &measurement, f32::NAN, 0.6, &no_clamp()),
      Err(SolveFault::NotFinite)
    );
    assert_eq!(
      dls_step(&jacobian, &measurement, 1e-4, f32::INFINITY, &no_clamp()),
      Err(SolveFault::NotFinite)
    );
    let bad_limit = Vector4::repeat(f32::NAN);
    assert_eq!(
      dls_step(&jacobian, &measurement, 1e-4, 0.6, &bad_limit),
      Err(SolveFault::NotFinite)
    );
  }

  #[test]
  fn the_broyden_update_makes_the_jacobian_reproduce_the_step_it_was_given() {
    let believed = realistic_jacobian();
    let actual = model_jacobian(500.0, 500.0, 5.6e-6 * 1.3);
    let dq = Vector4::new(60.0, -45.0, 30.0, 90.0);
    let dm = actual * dq;
    let updated = broyden_update(&believed, &dq, &dm, &CONFIG).unwrap();
    assert!((updated * dq - dm).amax() <= 1e-5);
    // A direction at right angles to the step keeps the old prediction: the update is rank one.
    let orthogonal = Vector4::new(45.0, 60.0, 0.0, 0.0);
    assert_close(dq.dot(&orthogonal), 0.0, 1e-3);
    assert!((updated * orthogonal - believed * orthogonal).amax() <= 1e-5);
  }

  #[test]
  fn a_correct_jacobian_is_left_unchanged_by_the_update() {
    let jacobian = realistic_jacobian();
    let dq = Vector4::new(60.0, -45.0, 30.0, 90.0);
    let updated = broyden_update(&jacobian, &dq, &(jacobian * dq), &CONFIG).unwrap();
    assert!((updated - jacobian).amax() <= 1e-9);
  }

  #[test]
  fn the_update_is_skipped_for_a_step_too_small_to_carry_information() {
    let jacobian = realistic_jacobian();
    let dm = Vector4::new(0.01, 0.0, 0.0, 0.0);
    assert_eq!(broyden_update(&jacobian, &Vector4::zeros(), &dm, &CONFIG), None);
    assert_eq!(
      broyden_update(&jacobian, &Vector4::new(0.5, 0.5, 0.0, 0.0), &dm, &CONFIG),
      None
    );
    assert!(broyden_update(&jacobian, &Vector4::new(1.0, 0.0, 0.0, 0.0), &dm, &CONFIG).is_some());
    let stricter = Solver {
      broyden_min_step_norm_steps: 24.0,
      ..CONFIG
    };
    assert_eq!(
      broyden_update(&jacobian, &Vector4::new(20.0, 0.0, 0.0, 0.0), &dm, &stricter),
      None
    );
  }

  #[test]
  fn the_update_is_skipped_for_an_input_that_is_not_a_number() {
    let jacobian = realistic_jacobian();
    let dq = Vector4::new(60.0, -45.0, 30.0, 90.0);
    assert_eq!(
      broyden_update(&jacobian, &dq, &Vector4::repeat(f32::NAN), &CONFIG),
      None
    );
    assert_eq!(
      broyden_update(&jacobian, &Vector4::repeat(f32::INFINITY), &(jacobian * dq), &CONFIG),
      None
    );
  }

  #[test]
  fn a_move_that_did_what_the_jacobian_predicted_is_not_suspect() {
    let jacobian = realistic_jacobian();
    let dq = Vector4::new(60.0, -45.0, 30.0, 90.0);
    assert!(!prediction_mismatch(&jacobian, &dq, &(jacobian * dq), &CONFIG));
    assert!(!prediction_mismatch(&jacobian, &dq, &(jacobian * dq * 0.7), &CONFIG));
  }

  #[test]
  fn an_axis_that_did_not_move_makes_the_step_suspect() {
    let jacobian = realistic_jacobian();
    let commanded = Vector4::new(0.0, 0.0, 100.0, 0.0);
    assert!(prediction_mismatch(&jacobian, &commanded, &Vector4::zeros(), &CONFIG));
    // The same dead axis inside a four-axis step of similar sizes.
    let commanded = Vector4::new(20.0, -30.0, 100.0, 25.0);
    let moved = Vector4::new(20.0, -30.0, 0.0, 25.0);
    assert!(prediction_mismatch(&jacobian, &commanded, &(jacobian * moved), &CONFIG));
  }

  #[test]
  fn a_response_of_less_than_half_the_prediction_is_suspect() {
    let jacobian = realistic_jacobian();
    let dq = Vector4::new(60.0, -45.0, 30.0, 90.0);
    assert!(!prediction_mismatch(&jacobian, &dq, &(jacobian * dq * 0.51), &CONFIG));
    assert!(prediction_mismatch(&jacobian, &dq, &(jacobian * dq * 0.49), &CONFIG));
  }

  #[test]
  fn a_measured_change_that_is_not_a_number_is_suspect() {
    let jacobian = realistic_jacobian();
    let dq = Vector4::new(60.0, -45.0, 30.0, 90.0);
    assert!(prediction_mismatch(&jacobian, &dq, &Vector4::repeat(f32::NAN), &CONFIG));
  }

  /// The noise of one component of a change with the starting config: 2 * 0.005.
  const FLOOR: f32 = 0.01;

  #[test]
  fn a_dead_axis_is_named_as_unresponsive_whichever_axis_it_is() {
    let jacobian = realistic_jacobian();
    let commanded = Vector4::new(40.0, -30.0, 60.0, 45.0);
    for dead in Axis::ALL {
      let mut moved = commanded;
      moved[dead.index()] = 0.0;
      assert_eq!(
        unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
        Some(dead)
      );
    }
  }

  #[test]
  fn an_axis_that_moved_backward_is_unresponsive() {
    let jacobian = realistic_jacobian();
    let commanded = Vector4::new(40.0, 40.0, 40.0, 40.0);
    let moved = Vector4::new(40.0, -40.0, 40.0, 40.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
      Some(Axis::M1V)
    );
  }

  #[test]
  fn the_axis_missing_the_most_is_named_when_two_fall_short() {
    let jacobian = realistic_jacobian();
    let commanded = Vector4::new(40.0, 40.0, 40.0, 40.0);
    // Mirror 1 has the larger gain (it acts over both legs), so the comparison is in measurement units: 25 steps
    // missing on M1V weigh less than 40 missing on M2V, and more than 30.
    let moved = Vector4::new(40.0, 15.0, 40.0, 0.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
      Some(Axis::M2V)
    );
    let moved = Vector4::new(40.0, 15.0, 40.0, 10.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
      Some(Axis::M1V)
    );
  }

  #[test]
  fn an_axis_that_took_more_than_half_of_its_command_is_not_unresponsive() {
    let jacobian = realistic_jacobian();
    let commanded = Vector4::new(40.0, 40.0, 40.0, 40.0);
    let moved = Vector4::new(40.0, 21.0, 40.0, 40.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
      None
    );
    let moved = Vector4::new(40.0, 19.0, 40.0, 40.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
      Some(Axis::M1V)
    );
  }

  /// A stored Jacobian that is 30% wrong in every column is a wrong Jacobian, not a dead axis, even though the
  /// plain mismatch of a Jacobian 30% too small is 43%.
  #[test]
  fn a_jacobian_that_is_thirty_percent_wrong_names_no_axis() {
    let actual = realistic_jacobian();
    let commanded = Vector4::new(40.0, -30.0, 60.0, 25.0);
    for scale in [0.7, 1.3] {
      let believed = actual * scale;
      assert_eq!(
        unresponsive_axis(&believed, &commanded, &(actual * commanded), FLOOR, &CONFIG),
        None
      );
    }
    let too_small = actual * 0.7;
    let mismatch = (actual * commanded - too_small * commanded).norm() / (too_small * commanded).norm();
    assert_close(mismatch, 0.43, 0.005);
  }

  #[test]
  fn an_axis_whose_command_is_too_small_to_see_is_never_named() {
    let jacobian = realistic_jacobian();
    // Two steps on M2V predict a change of about 0.006, far below the floor.
    let commanded = Vector4::new(40.0, -30.0, 60.0, 2.0);
    let moved = Vector4::new(40.0, -30.0, 60.0, 0.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &(jacobian * moved), FLOOR, &CONFIG),
      None
    );
    assert_eq!(
      unresponsive_axis(&jacobian, &Vector4::zeros(), &Vector4::zeros(), FLOOR, &CONFIG),
      None
    );
  }

  /// The review of 2026-11-15 found a healthy M2H named unresponsive in one run of eight after a command of 11
  /// steps at the configured noise. A dead axis is named only once its command stands clear of the noise its
  /// apparent step carries: for mirror 2 of this Jacobian, `0.5 * dq >= 3.64 * 0.01 * 505`, so 36.8 steps.
  #[test]
  fn a_dead_axis_is_named_only_when_its_command_clears_the_noise_of_its_apparent_step() {
    let jacobian = realistic_jacobian();
    let row_norm = jacobian.try_inverse().unwrap().row(2).norm();
    assert_close(row_norm, 505.0, 1.0);
    for (steps, named) in [(36.0, None), (37.0, Some(Axis::M2H))] {
      let commanded = Vector4::new(0.0, 0.0, steps, 0.0);
      assert_eq!(
        unresponsive_axis(&jacobian, &commanded, &Vector4::zeros(), FLOOR, &CONFIG),
        named,
        "{steps} steps"
      );
    }
    // Mirror 1's apparent step carries less noise: 0.5 * dq >= 3.64 * 0.01 * 357, so 26 steps.
    for (steps, named) in [(25.0, None), (27.0, Some(Axis::M1V))] {
      let commanded = Vector4::new(0.0, steps, 0.0, 0.0);
      assert_eq!(
        unresponsive_axis(&jacobian, &commanded, &Vector4::zeros(), FLOOR, &CONFIG),
        named,
        "{steps} steps"
      );
    }
  }

  /// The command must clear the noise by the margin of the test in both directions: with a mismatch fraction of
  /// 0.75, a dead axis is told from a healthy one only by the remaining 0.25 of its command.
  #[test]
  fn the_margin_is_the_smaller_side_of_the_mismatch_fraction() {
    let jacobian = realistic_jacobian();
    let config = Solver {
      prediction_mismatch_frac: 0.75,
      ..CONFIG
    };
    // 0.25 * dq >= 3.64 * 0.01 * 505 needs 73.5 steps on M2H.
    for (steps, named) in [(73.0, None), (74.0, Some(Axis::M2H))] {
      let commanded = Vector4::new(0.0, 0.0, steps, 0.0);
      assert_eq!(
        unresponsive_axis(&jacobian, &commanded, &Vector4::zeros(), FLOOR, &config),
        named,
        "{steps} steps"
      );
    }
  }

  /// A diagonal Jacobian of 1/256 per step: every product below is exact in `f32`, so the boundaries are too.
  fn exact_jacobian() -> Jacobian {
    Jacobian::identity() / 256.0
  }

  #[test]
  fn an_axis_that_took_exactly_half_of_its_command_is_not_unresponsive() {
    let jacobian = exact_jacobian();
    let commanded = Vector4::new(40.0, 0.0, 0.0, 0.0);
    let half = jacobian * Vector4::new(20.0, 0.0, 0.0, 0.0);
    assert_eq!(unresponsive_axis(&jacobian, &commanded, &half, FLOOR, &CONFIG), None);
    let less = jacobian * Vector4::new(19.0, 0.0, 0.0, 0.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &less, FLOOR, &CONFIG),
      Some(Axis::M1H)
    );
  }

  #[test]
  fn of_two_axes_missing_the_same_the_first_in_axis_order_is_named() {
    let jacobian = exact_jacobian();
    let commanded = Vector4::new(0.0, 40.0, 40.0, 0.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &Vector4::zeros(), FLOOR, &CONFIG),
      Some(Axis::M1V)
    );
  }

  #[test]
  fn no_axis_is_named_from_an_infinite_input() {
    let mut jacobian = exact_jacobian();
    jacobian[(0, 0)] = f32::INFINITY;
    let commanded = Vector4::new(40.0, 40.0, 40.0, 40.0);
    assert_eq!(
      unresponsive_axis(&jacobian, &commanded, &Vector4::zeros(), FLOOR, &CONFIG),
      None
    );
    let exact = exact_jacobian();
    assert_eq!(
      unresponsive_axis(
        &exact,
        &Vector4::repeat(f32::INFINITY),
        &Vector4::zeros(),
        FLOOR,
        &CONFIG
      ),
      None
    );
    assert_eq!(
      unresponsive_axis(
        &exact,
        &commanded,
        &Vector4::new(f32::INFINITY, 0.0, 0.0, 0.0),
        FLOOR,
        &CONFIG
      ),
      None
    );
  }

  #[test]
  fn no_axis_is_named_when_the_jacobian_cannot_be_inverted_or_the_data_is_not_a_number() {
    let commanded = Vector4::new(40.0, -30.0, 60.0, 25.0);
    let dm = Vector4::new(0.1, 0.1, 0.1, 0.1);
    assert_eq!(
      unresponsive_axis(&Jacobian::zeros(), &commanded, &dm, FLOOR, &CONFIG),
      None
    );
    let not_a_number = Vector4::repeat(f32::NAN);
    assert_eq!(
      unresponsive_axis(&realistic_jacobian(), &commanded, &not_a_number, FLOOR, &CONFIG),
      None
    );
  }

  #[test]
  fn whole_steps_become_the_solver_vector_in_axis_order() {
    assert_eq!(steps_to_vector([1, -2, 3, -4]), Vector4::new(1.0, -2.0, 3.0, -4.0));
  }

  #[test]
  fn convergence_is_the_infinity_norm_at_or_below_the_tolerance() {
    assert!(converged(&Vector4::new(0.02, -0.02, 0.0, 0.019), &CONFIG));
    assert!(!converged(&Vector4::new(0.0, 0.0, -0.021, 0.0), &CONFIG));
    assert!(!converged(&Vector4::new(0.0, f32::NAN, 0.0, 0.0), &CONFIG));
  }

  #[test]
  fn fractional_steps_round_to_the_nearest_whole_step_and_saturate() {
    assert_eq!(round_to_steps(&Vector4::new(1.4, 1.6, -1.4, -1.6)), [1, 2, -1, -2]);
    assert_eq!(round_to_steps(&Vector4::new(0.5, -0.5, 0.49, -0.49)), [1, -1, 0, 0]);
    assert_eq!(
      round_to_steps(&Vector4::new(1e12, -1e12, f32::NAN, 0.0)),
      [i32::MAX, i32::MIN, 0, 0]
    );
  }

  /// A Jacobian with the block-triangular shape of the model, unequal gains, and some cross-coupling.
  fn arbitrary_jacobian() -> impl Strategy<Value = Jacobian> {
    (
      100.0_f32..900.0,
      100.0_f32..900.0,
      proptest::array::uniform4(0.5_f32..2.0),
      proptest::array::uniform16(-0.05_f32..0.05)
    )
      .prop_map(|(dy, dx, gains, coupling)| {
        let [g1h, g1v, g2h, g2v] = gains;
        let model = Jacobian::new(
          g1h * dy,
          0.0,
          0.0,
          0.0, //
          0.0,
          g1v * dy,
          0.0,
          0.0, //
          g1h * dx,
          0.0,
          g2h * dx,
          0.0, //
          0.0,
          g1v * dx,
          0.0,
          g2v * dx
        );
        (model + Jacobian::from_row_slice(&coupling) * dy.min(dx)) * 5.6e-6
      })
  }

  proptest! {
    /// Under the linear model a damped step with alpha in (0, 1] cannot make the residual larger.
    #[test]
    fn an_unclamped_step_never_increases_the_predicted_residual(
      jacobian in arbitrary_jacobian(),
      measurement in proptest::array::uniform4(-0.6_f32..0.6),
      alpha in 0.05_f32..1.0
    ) {
      let measurement = Vector4::from(measurement);
      let conditioning = Conditioning::of(&jacobian).unwrap();
      prop_assume!(conditioning.check(&CONFIG).is_ok());
      let dq = dls_step(&jacobian, &measurement, conditioning.damping(&CONFIG), alpha, &no_clamp()).unwrap();
      let after = (measurement + jacobian * dq).norm();
      prop_assert!(after <= measurement.norm() * 1.000_1 + 1e-6, "{} grew to {after}", measurement.norm());
    }

    /// The secant condition: after the update the Jacobian maps the step onto the change that was measured.
    #[test]
    fn the_updated_jacobian_always_reproduces_the_measured_change(
      jacobian in arbitrary_jacobian(),
      dq in proptest::array::uniform4(-180.0_f32..180.0),
      dm in proptest::array::uniform4(-0.6_f32..0.6)
    ) {
      let dq = Vector4::from(dq);
      let dm = Vector4::from(dm);
      prop_assume!(dq.norm() >= CONFIG.broyden_min_step_norm_steps);
      let updated = broyden_update(&jacobian, &dq, &dm, &CONFIG).unwrap();
      prop_assert!((updated * dq - dm).amax() <= 1e-4);
    }

    #[test]
    fn a_clamped_step_never_exceeds_its_limit(
      jacobian in arbitrary_jacobian(),
      measurement in proptest::array::uniform4(-1.0_f32..1.0),
      limit in 1.0_f32..500.0
    ) {
      let dq = dls_step(&jacobian, &Vector4::from(measurement), 1e-4, 1.0, &Vector4::repeat(limit)).unwrap();
      prop_assert!(dq.amax() <= limit);
    }
  }
}
