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
  Reversed
}

impl DirSign {
  /// The factor applied to a commanded step count: `1` or `-1`.
  pub const fn factor(self) -> i32 {
    match self {
      DirSign::Forward => 1,
      DirSign::Reversed => -1
    }
  }
}

/// Mount and actuator geometry (addendum A.2, A.9, A.12).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mechanics {
  /// VERIFIED for the common 28BYJ-48 variant (gear tooth count: 32 x 63.68395). Confirm on each unit with the
  /// identification test in addendum A.6; other ratios are sold under the same name.
  pub steps_per_rev: f32,
  /// AS MODELED: M6 x 0.5 adjuster, pitch read from the CAD helices (addendum A.12).
  /// Confirm with the 10-turn caliper test.
  pub screw_lead_um: f32,
  /// AS MODELED for mirror 2 (35.0 mm arms, addendum A.12). UNVERIFIED for mirror 1,
  /// whose mount was never checked against a model.
  pub lever_arm_um: PerAxis<f32>,
  /// ASSUMED: beam angle per unit of mirror tilt. `2` for the screw that tilts about a vertical axis (in-plane),
  /// `sqrt(2)` for the other at 45 degree incidence (reference 1.2). Which screw is which is verified by poking.
  pub deflect_k: PerAxis<f32>,
  /// UNVERIFIED: depends on wiring and thread hand. Set per axis at commissioning.
  pub dir_sign: PerAxis<DirSign>,
  /// VERIFIED: datasheet minimum pull-in torque, 34.3 mN m at 5 V and 100 to 120 pulses per second.
  pub pull_in_torque_newton_m: f32
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
      m2_v: SQRT_2
    },
    dir_sign: PerAxis::splat(DirSign::Forward),
    pull_in_torque_newton_m: 0.0343
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
  /// UNVERIFIED guess, about 21 degrees of screw.
  /// Becomes `max(2 x backlash_steps, 60)` once backlash is measured with the beam (addendum A.5).
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
  /// UNVERIFIED placeholder: lowest commanded position of each axis, in steps from the commissioning position. The
  /// usable screw range has not been measured (reference 9.1, item 5); the placeholder is one capped move.
  pub soft_min_steps: PerAxis<i32>,
  /// UNVERIFIED placeholder: highest commanded position of each axis; see `soft_min_steps`.
  pub soft_max_steps: PerAxis<i32>,
  /// UNVERIFIED: the soft limits are widened by this fraction of travel, the largest plausible step loss. They are
  /// a runaway guard, not a position truth.
  pub soft_limit_margin_frac: f32
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
    soft_min_steps: PerAxis::splat(-4_000),
    soft_max_steps: PerAxis::splat(4_000),
    soft_limit_margin_frac: 0.10
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
  /// UNVERIFIED: a head offset above this fraction of the solver range means mechanical
  /// centering is required (reference 1.4).
  pub offset_warn_frac: f32,
  /// UNVERIFIED placeholder: noise of one normalized coordinate, one sigma. The sensor has not been characterized
  /// (hand-off open item 6). The placeholder is the largest noise at which the convergence tolerance can still be seen:
  /// a component of the measurement vector is a difference of two readings, and `3 * sqrt(2) * 0.005` is about `0.02`.
  pub noise_sigma_norm: f32,
  /// FIRM starting value: invalid shots in a row before a fault (reference 7.3).
  pub max_invalid_shots: u8
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
    noise_sigma_norm: 0.005,
    max_invalid_shots: 3
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
  /// FIRM starting value: the damping is multiplied by this after a poor step and divided
  /// by it after a good one (reference 3.2).
  pub lambda_adjust_factor: f32,
  /// FIRM starting value: a step that reduces the residual norm by less than this
  /// fraction is a poor step (reference 3.2).
  pub min_residual_reduction_frac: f32,
  /// FIRM starting value: poor steps in a row before a recalibration (reference 3.2:
  /// raise the damping, retry once, then recalibrate).
  pub poor_steps_before_recalibration: u8,
  /// FIRM starting value: recalibrations in one alignment before a fault (hand-off 3.2).
  pub max_recalibrations: u8,
  /// FIRM starting value: a calibration is rejected when a poke changed the measurement by less than this many
  /// sigmas of a measurement component (reference 3.1).
  pub min_column_sigmas: f32,
  /// FIRM starting value: suspect steps in a row on one axis before the "axis not responding" fault (addendum A.5).
  pub suspect_fault_streak: u8,
  /// FIRM, after the 2026-11-15 review: a solver step is judged only when its prediction stands this many noise
  /// sigmas clear of what noise alone produces. CALCULATED value: `3.64` is the 99th percentile of the norm of four
  /// independent unit normal components (the square root of 13.28, the 0.99 quantile of chi-square with four
  /// degrees of freedom), so a healthy step is called a mismatch by noise at most once in a hundred.
  pub judge_noise_sigmas: f32
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
    lambda_adjust_factor: 3.0,
    min_residual_reduction_frac: 0.3,
    poor_steps_before_recalibration: 2,
    max_recalibrations: 1,
    min_column_sigmas: 5.0,
    suspect_fault_streak: 2,
    judge_noise_sigmas: 3.64
  };
}

/// How a motor node guards and reports its moves (hand-off 3.3, reference 5.4, addendum A.2 and A.7).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NodeTiming {
  /// FIRM: a node stops and disarms when the last coordinator heartbeat is older than this (hand-off 3.3).
  pub heartbeat_timeout_ms: u32,
  /// FIRM: the coordinator sends a heartbeat this often (reference 5.4); a node sends its own at the same period.
  pub heartbeat_period_ms: u32,
  /// FIRM, chosen in M4: a node reports the STATUS of each motor this often, besides after every move.
  pub status_period_ms: u32,
  /// UNVERIFIED: the time the switched motor rail takes to come up after the arm pulse starts.
  /// The charge pump and the load switch are not designed yet (addendum A.7).
  /// A move whose rail is not up by then ends as a rail loss.
  pub rail_up_timeout_ms: u32,
  /// CALCULATED: the shortest step period a CONFIG may set, 1 / 500 pulses per second.
  /// The input is the VERIFIED no-load starting rate of more than 500 pulses per second (addendum A.2).
  pub min_step_period_us: u32,
  /// UNVERIFIED: the longest step period a CONFIG may set (10 pulses per second), a sanity bound only.
  pub max_step_period_us: u32,
  /// FIRM, chosen in M4: a move still running this long after its computed duration is cut off as a fault.
  pub move_timeout_margin_ms: u32,
  /// FIRM, chosen after the 2026-11-15 review: the hardware watchdog resets the node if it is not fed this long, so
  /// that a panic or a hang cannot leave coils energized. Longer than a flash sector erase (tens of milliseconds,
  /// with interrupts masked), short enough that a stuck coil cools quickly.
  pub watchdog_timeout_ms: u32
}

impl NodeTiming {
  /// The starting values.
  pub const STARTING: Self = Self {
    heartbeat_timeout_ms: 300,
    heartbeat_period_ms: 100,
    status_period_ms: 1_000,
    rail_up_timeout_ms: 50,
    min_step_period_us: 2_000,
    max_step_period_us: 100_000,
    move_timeout_margin_ms: 1_000,
    watchdog_timeout_ms: 1_000
  };
}

/// The coordinator's interlock and measurement timing (reference 6.6, 6.7, 7.3).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CoordinatorTiming {
  /// FIRM (reference 7.3): a job that has not fired this long after its start is a fault.
  pub fire_wait_timeout_ms: u32,
  /// FIRM (reference 7.3): the motor permit is given only after this long without a fire.
  pub quiet_before_move_ms: u32,
  /// UNVERIFIED: the force-guided relay must confirm a change of state within this long. Its operate and release
  /// times are not known until the part is chosen.
  pub relay_feedback_timeout_ms: u32,
  /// UNVERIFIED: the sensor's readings must all arrive this long after the job ends. Sized for the button
  /// sensor and its resends; a detector's integration time (`docs/SENSOR_PLAN.md`) may need more.
  pub reading_timeout_ms: u32,
  /// FIRM (reference 6.6): the start pulse on FootSW, low for at least 100 ms.
  pub start_pulse_ms: u32,
  /// FIRM, after the 2026-11-15 review: a move still running this long after the motor permit was given is a
  /// fault. A backstop only: each node ends its own move after `max_move_ms` (25 s) and the actuator reports a node
  /// that does not answer. Four axes of up to 25 s each, with margin.
  pub move_timeout_ms: u32,
  /// UNVERIFIED: fire edges closer than this belong to one shot (a dot may be several edges with an RF tube or
  /// PWM). Settled by the Mod5 scope check (`docs/SENSOR_PLAN.md` section 14).
  pub burst_gap_ms: u32,
  /// UNVERIFIED: a reading still missing this long after its shot, or after it was last asked for, is asked for
  /// again (SENSOR_RESEND). The button sensor answers 200 ms after a shot; a detector's integration may be longer.
  pub resend_after_ms: u32,
  /// FIRM: a MOVE_REL is sent at most this many times while its MOVE_DONE is missing. The node runs it once (its
  /// duplicate tracker) and answers a repeat of a finished move with the same MOVE_DONE.
  pub move_tries: u8
}

impl CoordinatorTiming {
  /// The starting values.
  pub const STARTING: Self = Self {
    fire_wait_timeout_ms: 30_000,
    quiet_before_move_ms: 500,
    relay_feedback_timeout_ms: 100,
    reading_timeout_ms: 5_000,
    start_pulse_ms: 200,
    move_timeout_ms: 120_000,
    burst_gap_ms: 5,
    resend_after_ms: 1_000,
    move_tries: 3
  };
}

/// How the sensor node treats SHOT frames (`docs/SENSOR_PLAN.md` sections 3.1 and 9).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorTiming {
  /// UNVERIFIED: a SHOT whose `seq` was seen within this long is a duplicate and is ignored.
  pub dedup_window_ms: u32,
  /// UNVERIFIED: without a coordinator heartbeat for this long,
  /// the sensor drops its pending shots and forgets every seen `seq`.
  pub coordinator_loss_ms: u32
}

impl SensorTiming {
  /// The starting values.
  pub const STARTING: Self = Self {
    dedup_window_ms: 2_000,
    coordinator_loss_ms: 1_000
  };
}

/// The torque margin test a node runs in bench mode (addendum A.6).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TorqueTest {
  /// FIRM (addendum A.6): each cycle is this many full steps plus, then the same minus.
  pub amplitude_steps: i32,
  /// FIRM (addendum A.6): cycles per run.
  pub cycles: u8,
  /// FIRM (addendum A.6): the step rates the test is run at, in pulses per second.
  pub rates_pps: [u32; 3]
}

impl TorqueTest {
  /// The procedure of addendum A.6.
  pub const STARTING: Self = Self {
    amplitude_steps: 1_000,
    cycles: 10,
    rates_pps: [100, 200, 300]
  };
}

/// The bring-up measurements of one axis through the beam: its backlash and its hysteresis (reference 3.4,
/// addendum A.5). The procedures are in [`crate::bringup`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BringUp {
  /// FIRM (reference 3.4): measurements at the starting position before anything else moves, for the noise and the
  /// drift. At least 2, at most [`crate::bringup::MAX_REPEAT_SHOTS`].
  pub repeat_shots: u8,
  /// FIRM (addendum A.5): the backlash search. Each trial moves the axis back by this many steps and returns it
  /// without overshoot. Positive and increasing.
  pub backlash_trials_steps: [i32; 5],
  /// FIRM (reference 3.4): a backlash trial moved the beam when its shift is more than this many sigmas of a shift.
  pub detect_sigmas: f32,
  /// FIRM (reference 3.4): the hysteresis cycle spans this many poke sizes either side of the starting position.
  pub hysteresis_amplitude_pokes: i32,
  /// FIRM (reference 3.4): moves in each half of the hysteresis cycle, with a measurement after each. At most
  /// [`crate::bringup::MAX_MOVES_PER_HALF`].
  pub hysteresis_moves_per_half: u8,
  /// FIRM (addendum A.5): the overshoot is twice the measured backlash, and never less than this.
  pub overshoot_floor_steps: i32
}

impl BringUp {
  /// The procedures of reference 3.4 with the backlash search of addendum A.5.
  pub const STARTING: Self = Self {
    repeat_shots: 3,
    backlash_trials_steps: [8, 16, 32, 64, 128],
    detect_sigmas: 3.0,
    hysteresis_amplitude_pokes: 2,
    hysteresis_moves_per_half: 8,
    overshoot_floor_steps: 60
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
  /// Motor node guards and reporting.
  pub node: NodeTiming,
  /// The bench torque margin test.
  pub torque_test: TorqueTest,
  /// The sensor node's SHOT handling.
  pub sensor_timing: SensorTiming,
  /// The coordinator's interlock and measurement timing.
  pub coordinator: CoordinatorTiming,
  /// The bring-up measurements of backlash and hysteresis.
  pub bring_up: BringUp
}

impl Config {
  /// Every section at its starting values.
  pub const STARTING: Self = Self {
    mechanics: Mechanics::STARTING,
    motion: Motion::STARTING,
    sensor: SensorLimits::STARTING,
    solver: Solver::STARTING,
    node: NodeTiming::STARTING,
    torque_test: TorqueTest::STARTING,
    sensor_timing: SensorTiming::STARTING,
    coordinator: CoordinatorTiming::STARTING,
    bring_up: BringUp::STARTING
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
  fn the_node_timing_follows_the_hand_off_and_the_no_load_start_rate() {
    let node = Config::default().node;
    assert_eq!(node.heartbeat_timeout_ms, 300);
    assert_eq!(node.heartbeat_period_ms, 100);
    // 500 pulses per second is a 2000 us period.
    assert_eq!(node.min_step_period_us, 1_000_000 / 500);
    assert!(node.min_step_period_us <= Config::default().motion.step_period_us);
    assert!(Config::default().motion.step_period_us <= node.max_step_period_us);
  }

  #[test]
  fn the_coordinator_timing_follows_reference_6_and_7_3() {
    let timing = Config::default().coordinator;
    assert_eq!(timing.fire_wait_timeout_ms, 30_000);
    assert_eq!(timing.quiet_before_move_ms, 500);
    assert!(timing.start_pulse_ms >= 100, "FootSW must stay low at least 100 ms");
    assert!(
      timing.move_timeout_ms >= 4 * Config::default().motion.max_move_ms,
      "a backstop to the nodes' own"
    );
  }

  #[test]
  fn the_torque_test_is_the_addendum_a6_procedure() {
    let test = Config::default().torque_test;
    assert_eq!(
      (test.amplitude_steps, test.cycles, test.rates_pps),
      (1_000, 10, [100, 200, 300])
    );
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
  fn the_bring_up_is_the_reference_3_4_procedure_with_the_addendum_a5_search() {
    let bring_up = Config::default().bring_up;
    assert_eq!(bring_up.repeat_shots, 3);
    assert_eq!(bring_up.backlash_trials_steps, [8, 16, 32, 64, 128]);
    assert_close(bring_up.detect_sigmas, 3.0, 0.0);
    assert_eq!(
      (bring_up.hysteresis_amplitude_pokes, bring_up.hysteresis_moves_per_half),
      (2, 8)
    );
    assert_eq!(bring_up.overshoot_floor_steps, 60);
  }

  #[test]
  fn a_reversed_axis_negates_a_commanded_step_count() {
    assert_eq!(DirSign::Forward.factor(), 1);
    assert_eq!(DirSign::Reversed.factor(), -1);
  }
}
