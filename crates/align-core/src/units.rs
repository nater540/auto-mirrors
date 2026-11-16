//! Conversions from motor steps to screw travel, mirror tilt, and beam displacement (addendum A.3).
//!
//! ```text
//! screw_travel = steps / STEPS_PER_REV * lead
//! mirror_tilt  = screw_travel / lever_arm
//! beam_disp    = k * mirror_tilt * distance
//! ```
//!
//! Steps are `f32` here because the solver works in fractional steps until a move is rounded. Positive steps are
//! the plus direction: the screw advancing against the springs. These figures are for planning and for the
//! simulator; the control loop never uses them as a Jacobian (reference 1.0: the Jacobian is measured).

use core::f32::consts::TAU;

use crate::axis::Axis;
use crate::config::Mechanics;

/// Slope of the normalized quadrant signal at the center for a Gaussian beam, times the beam radius:
/// `dX/dx0 = 2 * sqrt(2 / pi) / w` (CALCULATED, reference 2.2).
/// A real multimode beam differs; the measured Jacobian absorbs that.
pub const GAUSSIAN_CENTER_SLOPE: f32 = 1.595_769;

/// Screw travel for a move, in micrometers.
pub fn screw_travel_um(mechanics: &Mechanics, steps: f32) -> f32 {
  steps / mechanics.steps_per_rev * mechanics.screw_lead_um
}

/// Mirror tilt for a move on one axis, in microradians.
pub fn mirror_tilt_urad(mechanics: &Mechanics, axis: Axis, steps: f32) -> f32 {
  screw_travel_um(mechanics, steps) / mechanics.lever_arm_um.get(axis) * 1.0e6
}

/// Beam displacement at `distance_mm` from the mirror for a move on one axis, in micrometers.
pub fn beam_displacement_um(mechanics: &Mechanics, axis: Axis, steps: f32, distance_mm: f32) -> f32 {
  // Microradians times millimeters is nanometers.
  mechanics.deflect_k.get(axis) * mirror_tilt_urad(mechanics, axis, steps) * distance_mm * 1.0e-3
}

/// Full steps for one degree of screw rotation.
pub fn steps_per_screw_degree(mechanics: &Mechanics) -> f32 {
  mechanics.steps_per_rev / 360.0
}

/// Step rate for a step period, in pulses per second.
pub fn step_rate_pps(step_period_us: u32) -> f32 {
  #[expect(
    clippy::cast_precision_loss,
    reason = "a step period is a few thousand microseconds, far inside the exact range of f32"
  )]
  let period_us = step_period_us as f32;
  1.0e6 / period_us
}

/// Screw travel speed at a step rate, in micrometers per second.
pub fn screw_speed_um_per_s(mechanics: &Mechanics, step_rate_pps: f32) -> f32 {
  screw_travel_um(mechanics, step_rate_pps)
}

/// Time spent stepping for a move, in milliseconds, without the energize dwell and the hold before release.
pub fn stepping_time_ms(steps: u32, step_period_us: u32) -> u64 {
  // Two u32 factors cannot overflow a u64.
  u64::from(steps) * u64::from(step_period_us) / 1_000
}

/// Poke size, in steps, that should move the normalized sensor signal by `target_norm` for a Gaussian beam of
/// radius `beam_radius_um` (reference 3.1): `target / ((1.596 / w) * beam_um_per_step)`.
pub fn poke_steps(target_norm: f32, beam_radius_um: f32, beam_um_per_step: f32) -> f32 {
  target_norm / (GAUSSIAN_CENTER_SLOPE / beam_radius_um * beam_um_per_step)
}

/// Clamps a poke to `[3 x backlash_steps, max_steps]` (addendum A.3). When the two bounds cross, the backlash
/// bound wins: a poke inside the backlash measures nothing.
pub fn clamp_poke_steps(poke_steps: f32, backlash_steps: f32, max_steps: f32) -> f32 {
  poke_steps.min(max_steps).max(3.0 * backlash_steps)
}

/// Upper bound on the axial force a screw can exert, in newtons: `F = 2 pi eta T / lead` (addendum A.3). The
/// stepper cannot exceed its pull-out torque, so this holds with no current limiting. `efficiency` (eta) is
/// UNVERIFIED in the documents; pass it explicitly.
pub fn axial_force_bound_n(mechanics: &Mechanics, torque_newton_m: f32, efficiency: f32) -> f32 {
  TAU * efficiency * torque_newton_m / (mechanics.screw_lead_um * 1.0e-6)
}

#[cfg(test)]
mod tests {
  use super::*;
  use crate::config::Config;
  use crate::test_support::assert_close;

  fn mechanics() -> Mechanics {
    Config::default().mechanics
  }

  #[test]
  fn one_full_step_moves_the_screw_0_245_um() {
    assert_close(screw_travel_um(&mechanics(), 1.0), 0.245, 0.0005);
  }

  #[test]
  fn one_full_step_tilts_the_mirror_7_01_urad() {
    for axis in Axis::ALL {
      assert_close(mirror_tilt_urad(&mechanics(), axis, 1.0), 7.01, 0.005);
    }
  }

  #[test]
  fn one_full_step_moves_the_beam_14_0_um_at_1_m_on_a_horizontal_axis() {
    assert_close(beam_displacement_um(&mechanics(), Axis::M1H, 1.0, 1_000.0), 14.0, 0.05);
    assert_close(beam_displacement_um(&mechanics(), Axis::M2H, 1.0, 1_000.0), 14.0, 0.05);
  }

  #[test]
  fn one_full_step_moves_the_beam_9_9_um_at_1_m_on_a_vertical_axis() {
    assert_close(beam_displacement_um(&mechanics(), Axis::M1V, 1.0, 1_000.0), 9.9, 0.05);
    assert_close(beam_displacement_um(&mechanics(), Axis::M2V, 1.0, 1_000.0), 9.9, 0.05);
  }

  #[test]
  fn a_half_step_is_half_of_every_full_step_figure() {
    assert_close(screw_travel_um(&mechanics(), 0.5), 0.123, 0.0005);
    // The table prints 3.50 for 3.505.
    assert_close(mirror_tilt_urad(&mechanics(), Axis::M1H, 0.5), 3.50, 0.01);
    assert_close(beam_displacement_um(&mechanics(), Axis::M1H, 0.5, 1_000.0), 7.0, 0.05);
    assert_close(beam_displacement_um(&mechanics(), Axis::M1V, 0.5, 1_000.0), 5.0, 0.05);
  }

  #[test]
  fn a_negative_move_retracts_the_screw_and_reverses_the_beam() {
    assert_close(screw_travel_um(&mechanics(), -100.0), -24.53, 0.01);
    assert!(beam_displacement_um(&mechanics(), Axis::M2V, -100.0, 500.0) < 0.0);
  }

  #[test]
  fn there_are_5_66_full_steps_per_degree_and_2038_per_turn_of_the_screw() {
    assert_close(steps_per_screw_degree(&mechanics()), 5.66, 0.005);
    assert_close(mechanics().steps_per_rev, 2038.0, 0.5);
    assert_close(screw_travel_um(&mechanics(), mechanics().steps_per_rev), 500.0, 1e-3);
  }

  #[test]
  fn at_200_steps_per_second_the_screw_turns_0_098_rev_per_s_and_travels_49_um_per_s() {
    let rate_pps = step_rate_pps(Config::default().motion.step_period_us);
    assert_close(rate_pps, 200.0, 1e-3);
    assert_close(rate_pps / mechanics().steps_per_rev, 0.098, 0.0005);
    assert_close(screw_speed_um_per_s(&mechanics(), rate_pps), 49.0, 0.5);
  }

  #[test]
  fn a_500_step_move_takes_2_5_s_of_stepping() {
    assert_eq!(stepping_time_ms(500, Config::default().motion.step_period_us), 2_500);
  }

  #[test]
  fn the_longest_allowed_move_fits_inside_the_move_time_cap() {
    let motion = Config::default().motion;
    let steps = u32::try_from(motion.max_move_steps).unwrap();
    let total_ms = stepping_time_ms(steps, motion.step_period_us)
      + u64::from(motion.energize_dwell_ms)
      + u64::from(motion.hold_before_release_ms);
    assert!(
      total_ms <= u64::from(motion.max_move_ms),
      "{total_ms} ms exceeds the cap"
    );
  }

  #[test]
  fn stepping_time_does_not_overflow_at_the_largest_inputs() {
    assert_eq!(stepping_time_ms(u32::MAX, u32::MAX), 18_446_744_065_119_617);
  }

  #[test]
  fn the_poke_for_a_4_mm_beam_at_1_m_is_about_45_full_steps() {
    let motion = Config::default().motion;
    let beam_um_per_step = beam_displacement_um(&mechanics(), Axis::M1H, 1.0, 1_000.0);
    let poke = poke_steps(motion.poke_target_norm, 4_000.0, beam_um_per_step);
    assert_close(poke, 45.0, 0.5);
    #[expect(clippy::cast_precision_loss, reason = "45 is exact in f32")]
    let configured = motion.poke_steps as f32;
    assert_close(poke, configured, 0.5);
  }

  #[test]
  fn the_gaussian_small_signal_position_factor_is_0_627() {
    assert_close(GAUSSIAN_CENTER_SLOPE, 1.596, 0.0005);
    assert_close(1.0 / GAUSSIAN_CENTER_SLOPE, 0.627, 0.0005);
    assert_close(GAUSSIAN_CENTER_SLOPE, 2.0 * (2.0 / core::f32::consts::PI).sqrt(), 1e-6);
  }

  #[test]
  fn a_poke_is_clamped_between_three_backlashes_and_the_maximum() {
    assert_close(clamp_poke_steps(45.0, 10.0, 2_000.0), 45.0, 0.0);
    assert_close(clamp_poke_steps(45.0, 40.0, 2_000.0), 120.0, 0.0);
    assert_close(clamp_poke_steps(5_000.0, 10.0, 2_000.0), 2_000.0, 0.0);
  }

  #[test]
  fn a_poke_is_never_inside_the_backlash_even_when_that_exceeds_the_maximum() {
    assert_close(clamp_poke_steps(45.0, 1_000.0, 2_000.0), 3_000.0, 0.0);
  }

  #[test]
  fn the_axial_force_bounds_match_the_addendum_table() {
    let datasheet_newton_m = mechanics().pull_in_torque_newton_m;
    let vendor_newton_m = 0.0805;
    assert_close(axial_force_bound_n(&mechanics(), datasheet_newton_m, 0.14), 60.0, 0.5);
    assert_close(axial_force_bound_n(&mechanics(), datasheet_newton_m, 0.5), 216.0, 0.6);
    assert_close(axial_force_bound_n(&mechanics(), vendor_newton_m, 0.14), 142.0, 0.5);
    assert_close(axial_force_bound_n(&mechanics(), vendor_newton_m, 0.5), 506.0, 0.5);
  }
}
