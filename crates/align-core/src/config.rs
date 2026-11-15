//! Every tunable value of the alignment system, with the evidence behind its default.
//!
//! The defaults are starting values from the hand-off (section 3.2), the addendum (A.4, A.5, A.9, A.12), and the
//! reference (2.2, 2.3, 3.2). Each field's doc comment opens with the tag its source gives it:
//!
//! - VERIFIED: a named primary or vendor source;
//! - CALCULATED: a formula with its inputs shown;
//! - AS MODELED: read from a community CAD model, to be confirmed on the physical part;
//! - ASSUMED: a judgment in the source documents;
//! - UNVERIFIED: must be measured or tested;
//! - FIRM: a settled design choice, not a measurement.
//!
//! Nothing that is not VERIFIED or CALCULATED may become a literal elsewhere in the code: read it from here.

use core::f32::consts::SQRT_2;

use crate::axis::PerAxis;

/// Which motor sequence direction advances a screw against the springs (the plus direction).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DirSign {
  /// Stepping forward through the phase table advances the screw.
  Forward,
  /// Stepping backward through the phase table advances the screw.
  Reversed,
}

impl DirSign {
  /// The factor applied to a commanded step count: `1` or `-1`.
  pub const fn factor(self) -> i32 {
    match self {
      DirSign::Forward => 1,
      DirSign::Reversed => -1,
    }
  }
}

/// Mount and actuator geometry (addendum A.2, A.9, A.12).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mechanics {
  /// VERIFIED for the common 28BYJ-48 variant (gear tooth count: 32 x 63.68395). Confirm on each unit with the
  /// identification test in addendum A.6; other ratios are sold under the same name.
  pub steps_per_rev: f32,
  /// AS MODELED: M6 x 0.5 adjuster, pitch read from the CAD helices (addendum A.12). Confirm with the 10-turn
  /// caliper test.
  pub screw_lead_um: f32,
  /// AS MODELED for mirror 2 (35.0 mm arms, addendum A.12). UNVERIFIED for mirror 1, whose mount was never checked
  /// against a model.
  pub lever_arm_um: PerAxis<f32>,
  /// ASSUMED: beam angle per unit of mirror tilt. `2` for the screw that tilts about a vertical axis (in-plane),
  /// `sqrt(2)` for the other at 45 degree incidence (reference 1.2). Which screw is which is verified by poking.
  pub deflect_k: PerAxis<f32>,
  /// UNVERIFIED: depends on wiring and thread hand. Set per axis at commissioning.
  pub dir_sign: PerAxis<DirSign>,
  /// VERIFIED: datasheet minimum pull-in torque, 34.3 mN m at 5 V and 100 to 120 pulses per second.
  pub pull_in_torque_newton_m: f32,
}

impl Mechanics {
  /// The starting values from addendum A.9 and A.12.
  pub const STARTING: Self = Self {
    steps_per_rev: 2037.8864,
    screw_lead_um: 500.0,
    lever_arm_um: PerAxis::splat(35_000.0),
    deflect_k: PerAxis {
      m1_h: 2.0,
      m1_v: SQRT_2,
      m2_h: 2.0,
      m2_v: SQRT_2,
    },
    dir_sign: PerAxis::splat(DirSign::Forward),
    pull_in_torque_newton_m: 0.0343,
  };
}

/// How an axis is moved (addendum A.3, A.4, A.5, A.9).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
  /// UNVERIFIED: 200 pulses per second, pending the torque margin test (addendum A.6).
  pub step_period_us: u32,
  /// FIRM: time the stored phase is energized before the first step (addendum A.4).
  pub energize_dwell_ms: u32,
  /// FIRM: time the final phase is held before the coils are released (addendum A.4).
  pub hold_before_release_ms: u32,
  /// UNVERIFIED: wait after the coils release before a shot.
  pub settle_ms: u32,
  /// UNVERIFIED guess, about 21 degrees of screw. Becomes `max(2 x backlash_steps, 60)` once backlash is measured
  /// with the beam (addendum A.5).
  pub overshoot_steps: i32,
  /// CALCULATED for a 4 mm beam radius at 1 m with the lever arm above (addendum A.3); see
  /// [`crate::units::poke_steps`].
  pub poke_steps: i32,
  /// FIRM: the sensor signal a poke aims to produce, in normalized units (reference 3.1).
  pub poke_target_norm: f32,
  /// CALCULATED: upper clamp on a poke (addendum A.3).
  pub poke_max_steps: i32,
  /// UNVERIFIED: cap on one move, about two screw turns.
  pub max_move_steps: i32,
  /// UNVERIFIED: cap on the duration of one move.
  pub max_move_ms: u32,
  /// UNVERIFIED: the soft limits are widened by this fraction of travel, the largest plausible step loss. They are
  /// a runaway guard, not a position truth.
  pub soft_limit_margin_frac: f32,
}

impl Motion {
  /// The starting values from addendum A.9.
  pub const STARTING: Self = Self {
    step_period_us: 5_000,
    energize_dwell_ms: 20,
    hold_before_release_ms: 100,
    settle_ms: 200,
    overshoot_steps: 120,
    poke_steps: 45,
    poke_target_norm: 0.25,
    poke_max_steps: 2_000,
    max_move_steps: 4_000,
    max_move_ms: 25_000,
    soft_limit_margin_frac: 0.10,
  };
}

/// Thresholds for accepting a sensor reading (reference 2.2 and 2.3). All are starting points.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorLimits {
  /// UNVERIFIED: a reading needs a sum of at least this fraction of the reference sum.
  pub sum_min_frac: f32,
  /// UNVERIFIED: a quadrant at or above this fraction of full scale is saturated.
  pub saturation_frac: f32,
  /// UNVERIFIED: a quadrant further below zero than this many noise sigmas means a baseline error.
  pub negative_sigma: f32,
  /// UNVERIFIED: beyond this normalized position the beam is at the edge and the reading is rejected.
  pub edge_max_norm: f32,
  /// CALCULATED: the solver uses a reading only inside this normalized range, where a Gaussian beam is linear to
  /// about 10% (reference 2.2). Beyond it, only the sign is used.
  pub solver_max_norm: f32,
  /// UNVERIFIED: two shots must agree within this many sigmas.
  pub repeat_sigma: f32,
  /// UNVERIFIED: the sum may differ from the previous shot at the same power by this fraction.
  pub energy_tolerance_frac: f32,
  /// UNVERIFIED: a head offset above this fraction of the solver range means mechanical centering is required
  /// (reference 1.4).
  pub offset_warn_frac: f32,
}

impl SensorLimits {
  /// The starting values from reference 1.4, 2.2, and 2.3.
  pub const STARTING: Self = Self {
    sum_min_frac: 0.3,
    saturation_frac: 0.9,
    negative_sigma: 3.0,
    edge_max_norm: 0.9,
    solver_max_norm: 0.6,
    repeat_sigma: 3.0,
    energy_tolerance_frac: 0.3,
    offset_warn_frac: 0.3,
  };
}

/// Solver thresholds (hand-off 3.2, reference 3.2, addendum A.5). All are starting values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Solver {
  /// FIRM starting value: fraction of the damped least squares step that is applied.
  pub alpha: f32,
  /// FIRM starting value: damping as a fraction of the largest singular value of the Jacobian.
  pub lambda_frac: f32,
  /// FIRM starting value: converged when the infinity norm of the measurement vector is at or below this.
  pub converge_tol_norm: f32,
  /// FIRM starting value: iterations before one recalibration, then a fault.
  pub max_iterations: u8,
  /// FIRM starting value: a Jacobian with a condition number above this is rejected.
  pub cond_max: f32,
  /// FIRM starting value: a step is clamped to this many poke sizes on each axis (reference 3.2).
  pub step_clamp_pokes: f32,
  /// UNVERIFIED: a move is suspect when `norm(dm - J dq) > frac * norm(J dq)` (addendum A.5).
  pub prediction_mismatch_frac: f32,
  /// UNVERIFIED placeholder: the Broyden update is skipped for a step whose norm is below this many steps. The
  /// reference asks for twice the minimum reliable move, which has not been measured for the stepper; `1.0`
  /// reproduces the reference sketch (any whole-step move updates).
  pub broyden_min_step_norm_steps: f32,
}

impl Solver {
  /// The starting values from hand-off 3.2.
  pub const STARTING: Self = Self {
    alpha: 0.6,
    lambda_frac: 0.05,
    converge_tol_norm: 0.02,
    max_iterations: 4,
    cond_max: 50.0,
    step_clamp_pokes: 4.0,
    prediction_mismatch_frac: 0.5,
    broyden_min_step_norm_steps: 1.0,
  };
}

/// The whole configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
  /// Mount and actuator geometry.
  pub mechanics: Mechanics,
  /// How an axis is moved.
  pub motion: Motion,
  /// Thresholds for accepting a sensor reading.
  pub sensor: SensorLimits,
  /// Solver thresholds.
  pub solver: Solver,
}

impl Config {
  /// Every section at its starting values.
  pub const STARTING: Self = Self {
    mechanics: Mechanics::STARTING,
    motion: Motion::STARTING,
    sensor: SensorLimits::STARTING,
    solver: Solver::STARTING,
  };
}

impl Default for Config {
  fn default() -> Self {
    Self::STARTING
  }
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::axis::Axis;
  use crate::test_support::assert_close;

  #[test]
  fn the_starting_mechanics_are_the_addendum_a9_constants() {
    let mechanics = Config::default().mechanics;
    assert_close(mechanics.steps_per_rev, 2037.8864, 1e-3);
    assert_close(mechanics.steps_per_rev, 32.0 * 63.683_95, 1e-2);
    assert_close(mechanics.screw_lead_um, 500.0, 0.0);
    assert_close(mechanics.pull_in_torque_newton_m, 0.0343, 1e-6);
    for axis in Axis::ALL {
      assert_close(mechanics.lever_arm_um.get(axis), 35_000.0, 0.0);
      assert_eq!(mechanics.dir_sign.get(axis), DirSign::Forward);
    }
    assert_close(mechanics.deflect_k.get(Axis::M1H), 2.0, 0.0);
    assert_close(mechanics.deflect_k.get(Axis::M1V), 1.414, 1e-3);
    assert_close(mechanics.deflect_k.get(Axis::M2H), 2.0, 0.0);
    assert_close(mechanics.deflect_k.get(Axis::M2V), 1.414, 1e-3);
  }

  #[test]
  fn the_starting_motion_values_are_the_addendum_a9_constants() {
    let motion = Config::default().motion;
    assert_eq!(motion.step_period_us, 5_000);
    assert_eq!(motion.energize_dwell_ms, 20);
    assert_eq!(motion.hold_before_release_ms, 100);
    assert_eq!(motion.settle_ms, 200);
    assert_eq!(motion.overshoot_steps, 120);
    assert_eq!(motion.poke_steps, 45);
    assert_eq!(motion.max_move_steps, 4_000);
    assert_eq!(motion.max_move_ms, 25_000);
    assert_close(motion.soft_limit_margin_frac, 0.10, 1e-6);
  }

  #[test]
  fn the_starting_solver_values_are_the_hand_off_section_3_2_values() {
    let solver = Config::default().solver;
    assert_close(solver.alpha, 0.6, 1e-6);
    assert_close(solver.lambda_frac, 0.05, 1e-6);
    assert_close(solver.converge_tol_norm, 0.02, 1e-6);
    assert_eq!(solver.max_iterations, 4);
    assert_close(solver.cond_max, 50.0, 0.0);
    assert_close(solver.prediction_mismatch_frac, 0.5, 0.0);
  }

  #[test]
  fn a_reversed_axis_negates_a_commanded_step_count() {
    assert_eq!(DirSign::Forward.factor(), 1);
    assert_eq!(DirSign::Reversed.factor(), -1);
  }
}
