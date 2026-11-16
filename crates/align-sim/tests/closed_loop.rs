//! Closed-loop tests: the aligner of `align-core` against the simulated machine (hand-off milestone M2).
//!
//! Each test asserts on the model's truth (the real beam angles and the noise-free residual), not only on what
//! the aligner reports about itself.

use align_core::aligner::{AlignFault, AlignReport, Aligner};
use align_core::approach::SoftLimits;
use align_core::axis::{Axis, PerAxis};
use align_core::config::Config;
use align_core::measurement::Shots;
use align_core::solver::SolveFault;
use align_sim::block_on;
use align_sim::rig::Rig;
use align_sim::world::{AxisModel, Beam, Record, Scenario};
use nalgebra::Vector4;
use proptest::prelude::*;
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

/// A misalignment the sensor can see at all three points without leaving its linear range: mirror 1 within 35
/// steps, mirror 2 within 50, and a head offset within 200 um.
fn misalignment() -> impl Strategy<Value = (PerAxis<f64>, [f64; 2])> {
  (
    -35.0..35.0,
    -35.0..35.0,
    -50.0..50.0,
    -50.0..50.0,
    -200.0..200.0,
    -200.0..200.0,
  )
    .prop_map(|(m1_h, m1_v, m2_h, m2_v, offset_h, offset_v)| (PerAxis { m1_h, m1_v, m2_h, m2_v }, [offset_h, offset_v]))
}

fn realistic(seed: u64, misalignment_steps: PerAxis<f64>, head_offset_um: [f64; 2]) -> Scenario {
  Scenario {
    misalignment_steps,
    head_offset_um,
    ..Scenario::realistic(seed)
  }
}

/// The noise-free residual of the three-position difference vector.
fn true_residual(rig: &Rig) -> f32 {
  rig.world().true_shots().measurement().amax()
}

fn align(rig: &Rig, config: Config) -> (Result<AlignReport, AlignFault>, Aligner<Rig, Rig>) {
  let mut aligner = rig.aligner(config);
  let outcome = block_on(aligner.align());
  (outcome, aligner)
}

// --- the perfect machine ----------------------------------------------------------------------------------------

#[test]
fn an_aligned_ideal_machine_converges_without_a_solver_step() {
  let rig = Rig::new(Scenario::IDEAL);
  let (outcome, _) = align(&rig, Config::default());
  let report = outcome.unwrap();
  assert_eq!(report.iterations, 0);
  assert_eq!(report.recalibrations, 0);
  assert!(!report.needs_mechanical_centering);
}

#[test]
fn calibrating_an_ideal_machine_recovers_the_model_jacobian() {
  let rig = Rig::new(Scenario::IDEAL);
  let mut aligner = rig.aligner(Config::default());
  let calibration = block_on(aligner.calibrate()).unwrap();
  let truth = rig.world().true_jacobian();
  let relative_error = (calibration.jacobian - truth).norm() / truth.norm();
  // The error function bends slightly over a 45 step poke; the model Jacobian is the slope at the center.
  assert!(relative_error < 0.05, "relative error {relative_error}");
  assert_eq!(calibration.q0_steps, [0; 4]);
  // Every poke was undone.
  for axis in Axis::ALL {
    assert_eq!(rig.world().commanded_steps(axis), 0);
  }
}

#[test]
fn an_ideal_machine_is_aligned_from_a_known_misalignment() {
  let scenario = Scenario {
    misalignment_steps: PerAxis {
      m1_h: 30.0,
      m1_v: -25.0,
      m2_h: -40.0,
      m2_v: 35.0,
    },
    ..Scenario::IDEAL
  };
  let rig = Rig::new(scenario);
  assert!(true_residual(&rig) > 0.05);
  let (outcome, aligner) = align(&rig, Config::default());
  let report = outcome.unwrap();
  assert!(report.iterations <= 4, "{report:?}");
  assert_eq!(report.recalibrations, 0);
  assert!(true_residual(&rig) <= 0.02, "true residual {}", true_residual(&rig));
  assert!(aligner.axes().to_array().iter().all(|state| !state.suspect));
}

// --- the five tests the hand-off names --------------------------------------------------------------------------
//
// Tests 1 and 2 are counts over fixed seeds, not properties over every seed, because what they claim is a rate.
// The rates quoted in their comments were measured over 20000 runs with
// `cargo run -p align-sim --release -- sweep --runs 20000 --y-spans 500 --stick N [--jacobian-error 0.3]`.

/// Run `index` of a repeatable series: the realistic machine with a seat that sticks for `stick_steps`, a
/// misalignment and a head offset drawn from the same ranges as [`misalignment`], and a noise seed of its own.
fn seeded(index: u64, stick_steps: u32) -> (Scenario, StdRng) {
  let mut draw = StdRng::seed_from_u64(index);
  let mut scenario = Scenario::realistic(index);
  scenario.axes = PerAxis::splat(AxisModel {
    stick_steps,
    ..scenario.axes.m1_h
  });
  scenario.misalignment_steps = PerAxis {
    m1_h: draw.random_range(-35.0..35.0),
    m1_v: draw.random_range(-35.0..35.0),
    m2_h: draw.random_range(-50.0..50.0),
    m2_v: draw.random_range(-50.0..50.0),
  };
  scenario.head_offset_um = [draw.random_range(-200.0..200.0), draw.random_range(-200.0..200.0)];
  (scenario, draw)
}

/// What a series of runs came to.
#[derive(Debug, Default)]
struct Tally {
  runs: u32,
  within_the_cap: u32,
  recalibrated: u32,
}

impl Tally {
  /// Counts one converged run. It may have used at most the one allowed recalibration, and the truth must agree
  /// with it: the noise-free residual is allowed the sensor noise on top of the tolerance.
  fn count(&mut self, rig: &Rig, report: &AlignReport, config: &Config) {
    assert!(report.recalibrations <= config.solver.max_recalibrations, "{report:?}");
    assert!(
      true_residual(rig) <= 0.03,
      "run {}: true residual {}",
      self.runs,
      true_residual(rig)
    );
    self.runs += 1;
    if report.recalibrations == 0 {
      assert!(report.iterations <= config.solver.max_iterations, "{report:?}");
      self.within_the_cap += 1;
    } else {
      self.recalibrated += 1;
    }
  }
}

/// Hand-off M2, test 1, where it holds without exception: 20000 of 20000 at each of 0, 1, and 2 steps.
#[test]
fn alignment_converges_within_the_iteration_cap_when_the_seat_sticks_for_two_steps_or_less() {
  let config = Config::default();
  for stick_steps in 0..=2 {
    let mut tally = Tally::default();
    for index in 0..300 {
      let rig = Rig::new(seeded(index, stick_steps).0);
      let (outcome, _) = align(&rig, config);
      tally.count(&rig, &outcome.unwrap(), &config);
    }
    assert_eq!(tally.within_the_cap, 300, "stick {stick_steps}: {tally:?}");
  }
}

/// Hand-off M2, test 1, at the edge: with three steps of stick-slip the residual floor reaches the tolerance, and
/// a few runs in ten thousand need the one allowed recalibration (3 of 20000). None fails.
#[test]
fn a_seat_that_sticks_for_three_steps_costs_at_most_the_one_allowed_recalibration() {
  let config = Config::default();
  let mut tally = Tally::default();
  for index in 0..1_000 {
    let rig = Rig::new(seeded(index, 3).0);
    let (outcome, _) = align(&rig, config);
    tally.count(&rig, &outcome.unwrap(), &config);
  }
  assert!(tally.within_the_cap >= 990, "{tally:?}");
}

/// Hand-off M2, test 2: every column of the stored Jacobian is wrong by 30%, up or down. Every run converges.
/// Not every run does so within the cap: starting from a Jacobian that is too large, four damped steps can fall
/// just short, and the recalibration finishes the job (52 of 20000 at three steps of stick-slip, 11 of 20000 at
/// none).
#[test]
fn alignment_converges_with_a_jacobian_that_is_thirty_percent_wrong() {
  let config = Config::default();
  let mut tally = Tally::default();
  for index in 0..1_000 {
    let (scenario, mut draw) = seeded(index, 3);
    let rig = Rig::new(scenario);
    let scale = core::array::from_fn(|_| if draw.random_bool(0.5) { 1.3 } else { 0.7 });
    let mut aligner = rig.aligner(config).with_calibration(rig.stored_calibration(scale));
    let outcome = block_on(aligner.align());
    tally.count(&rig, &outcome.unwrap(), &config);
  }
  assert!(tally.within_the_cap >= 990, "{tally:?}");
}

/// The worst cases of test 2 on their own: every column too large, and every column too small.
#[test]
fn a_jacobian_that_is_uniformly_thirty_percent_too_large_or_too_small_still_converges() {
  let config = Config::default();
  for scale in [[1.3; 4], [0.7; 4]] {
    let mut tally = Tally::default();
    for index in 0..200 {
      let rig = Rig::new(seeded(index, 3).0);
      let mut aligner = rig.aligner(config).with_calibration(rig.stored_calibration(scale));
      let outcome = block_on(aligner.align());
      tally.count(&rig, &outcome.unwrap(), &config);
    }
    assert!(tally.within_the_cap >= 190, "scale {scale:?}: {tally:?}");
  }
}

proptest! {
  #![proptest_config(ProptestConfig::with_cases(64))]

  /// Hand-off M2, test 4. The limits are tight enough that some misalignments cannot be corrected inside them;
  /// those must end in a fault, never in a move past the limit.
  #[test]
  fn no_move_is_ever_commanded_beyond_the_soft_limits(
    seed in any::<u64>(),
    (misalignment_steps, head_offset_um) in misalignment(),
    limit in 125_i32..400,
  ) {
    let rig = Rig::new(realistic(seed, misalignment_steps, head_offset_um));
    let mut config = Config::default();
    config.motion.soft_min_steps = PerAxis::splat(-limit);
    config.motion.soft_max_steps = PerAxis::splat(limit);
    let (outcome, _) = align(&rig, config);
    for axis in Axis::ALL {
      let limits = SoftLimits::for_axis(&config.motion, axis);
      let (lowest, highest) = rig.world().commanded_range(axis);
      prop_assert!(limits.contains(lowest) && limits.contains(highest), "{:?} reached {}..{}", axis, lowest, highest);
    }
    if let Err(fault) = outcome {
      prop_assert!(matches!(fault, AlignFault::SoftLimit { .. }), "{:?}", fault);
    }
  }
}

/// Hand-off M2, test 3, with a stored calibration from when the axis still worked.
#[test]
fn a_dead_axis_is_marked_suspect_and_then_reported_as_not_responding() {
  for dead in Axis::ALL {
    let mut scenario = realistic(
      11,
      PerAxis {
        m1_h: 25.0,
        m1_v: -25.0,
        m2_h: 35.0,
        m2_v: -35.0,
      },
      [50.0, -50.0],
    );
    *scenario.axes.get_mut(dead) = AxisModel::DEAD;
    let rig = Rig::new(scenario);
    let mut aligner = rig
      .aligner(Config::default())
      .with_calibration(rig.stored_calibration([1.0; 4]));
    let outcome = block_on(aligner.align());
    assert_eq!(outcome, Err(AlignFault::AxisNotResponding { axis: dead }));
    for axis in Axis::ALL {
      assert_eq!(
        aligner.axes().get(axis).suspect,
        axis == dead,
        "suspect flag of {axis:?} with {dead:?} dead"
      );
    }
  }
}

/// Hand-off M2, test 3, without a stored calibration: the dead axis is found by its poke.
#[test]
fn a_dead_axis_fails_calibration_as_a_weak_column() {
  for dead in Axis::ALL {
    let mut scenario = realistic(12, PerAxis::splat(10.0), [0.0, 0.0]);
    *scenario.axes.get_mut(dead) = AxisModel::DEAD;
    let rig = Rig::new(scenario);
    let (outcome, aligner) = align(&rig, Config::default());
    assert_eq!(outcome, Err(AlignFault::WeakColumn { axis: dead }));
    assert!(aligner.axes().get(dead).suspect);
  }
}

/// The two-position absolute target the reference rejects: the readings at A and C, driven to zero.
fn absolute_target(shots: &Shots) -> Vector4<f32> {
  Vector4::new(shots.a.h_norm, shots.a.v_norm, shots.c.h_norm, shots.c.v_norm)
}

/// Hand-off M2, test 5: this guards decision 3.2.
///
/// With a constant offset at the head, driving the absolute readings at two points to zero succeeds by its own
/// measure and leaves the beam at an angle: it tilts the mirrors to cancel the offset at those two points. The
/// three-position difference target ignores the offset and removes the angles.
#[test]
fn a_two_position_absolute_target_fails_where_the_three_position_difference_succeeds() {
  let scenario = Scenario {
    misalignment_steps: PerAxis {
      m1_h: 15.0,
      m1_v: -10.0,
      m2_h: -20.0,
      m2_v: 15.0,
    },
    // A small offset on purpose. At this geometry the absolute target turns every micrometer of offset into
    // about 60 urad of beam angle, so an offset of a few hundred micrometers throws the beam off the sensor
    // before the method can even finish.
    head_offset_um: [20.0, -15.0],
    ..Scenario::IDEAL
  };

  let difference_rig = Rig::new(scenario);
  let (outcome, _) = align(&difference_rig, Config::default());
  outcome.unwrap();
  let difference_angles = difference_rig.world().angle_errors().largest_urad();
  let difference_residual = true_residual(&difference_rig);

  // At this geometry the absolute target does not even pass the conditioning gate: the two points see almost
  // the same combination of the two mirrors.
  let rejected = block_on(
    Rig::new(scenario)
      .aligner(Config::default())
      .with_target(absolute_target)
      .align(),
  );
  assert!(
    matches!(rejected, Err(AlignFault::Calibration(SolveFault::IllConditioned { cond, .. })) if cond > 200.0),
    "{rejected:?}"
  );

  // Given every chance (no conditioning gate, almost no damping, three times the iterations, and a tolerance
  // four times tighter so that it settles on its own solution) it reaches its own target.
  let mut permissive = Config::default();
  permissive.solver.cond_max = 1_000.0;
  permissive.solver.lambda_frac = 0.001;
  permissive.solver.max_iterations = 12;
  permissive.solver.converge_tol_norm = 0.005;
  let absolute_rig = Rig::new(scenario);
  let mut aligner = absolute_rig.aligner(permissive).with_target(absolute_target);
  block_on(aligner.align()).unwrap();
  let shots = absolute_rig.world().true_shots();
  let absolute_angles = absolute_rig.world().angle_errors().largest_urad();
  let absolute_residual = true_residual(&absolute_rig);

  // The absolute method did what it was asked: the beam is centered at A and at C.
  assert!(absolute_target(&shots).amax() <= 0.005, "{shots:?}");
  // And the beam is not parallel to the axes: it sweeps across the sensor as the gantry moves from A to B, by
  // more than five times the convergence tolerance.
  assert!(
    absolute_residual > 0.1,
    "absolute target left a difference residual of {absolute_residual}"
  );
  assert!(absolute_angles > 500.0, "absolute target left {absolute_angles} urad");
  // The difference method left the offset alone and removed the angles, to what its tolerance allows: 0.02
  // normalized over the 500 mm Y span of this model is 100 urad.
  assert!(
    difference_residual <= 0.02,
    "difference target left a residual of {difference_residual}"
  );
  assert!(
    difference_angles < 110.0,
    "difference target left {difference_angles} urad"
  );
  assert!(absolute_angles > 5.0 * difference_angles);
}

// --- the imperfections, one at a time ---------------------------------------------------------------------------

#[test]
fn the_head_offset_is_reported_and_large_ones_ask_for_mechanical_centering() {
  let small = Scenario {
    head_offset_um: [100.0, 0.0],
    ..Scenario::IDEAL
  };
  let report = align(&Rig::new(small), Config::default()).0.unwrap();
  assert!(report.head_offset.h_norm > 0.03);
  assert!(!report.needs_mechanical_centering);

  // 0.18 normalized is about 460 um for a 4 mm beam.
  let large = Scenario {
    head_offset_um: [600.0, 0.0],
    ..Scenario::IDEAL
  };
  let report = align(&Rig::new(large), Config::default()).0.unwrap();
  assert!(report.needs_mechanical_centering, "{report:?}");
}

#[test]
fn a_beam_off_the_sensor_is_a_reading_fault_after_three_shots() {
  let scenario = Scenario {
    head_offset_um: [30_000.0, 0.0],
    ..Scenario::IDEAL
  };
  let rig = Rig::new(scenario);
  let (outcome, _) = align(&rig, Config::default());
  assert!(matches!(outcome, Err(AlignFault::Reading(_))), "{outcome:?}");
  let world = rig.world();
  let measurements = world
    .log()
    .iter()
    .filter(|record| matches!(record, Record::Measure { .. }))
    .count();
  assert_eq!(measurements, 3);
}

#[test]
fn alignment_survives_occasional_skipped_steps() {
  let mut scenario = realistic(
    21,
    PerAxis {
      m1_h: 30.0,
      m1_v: -20.0,
      m2_h: -45.0,
      m2_v: 40.0,
    },
    [80.0, 60.0],
  );
  scenario.axes = PerAxis::splat(AxisModel {
    backlash_steps: 20,
    stick_steps: 3,
    skip_probability: 0.02,
  });
  let rig = Rig::new(scenario);
  let (outcome, _) = align(&rig, Config::default());
  outcome.unwrap();
  assert!(true_residual(&rig) <= 0.03, "true residual {}", true_residual(&rig));
}

#[test]
fn every_axis_is_left_having_last_moved_in_the_plus_direction() {
  let scenario = realistic(
    31,
    PerAxis {
      m1_h: -30.0,
      m1_v: 20.0,
      m2_h: 45.0,
      m2_v: -40.0,
    },
    [0.0, 0.0],
  );
  let rig = Rig::new(scenario);
  let (outcome, aligner) = align(&rig, Config::default());
  outcome.unwrap();
  for axis in Axis::ALL {
    assert_eq!(aligner.axes().get(axis).last_dir, align_core::approach::LastDir::Plus);
    let last_move = rig.world().log().iter().rev().find_map(|record| match record {
      Record::Move {
        axis: moved,
        delta_steps,
      } if *moved == axis => Some(*delta_steps),
      _ => None,
    });
    assert!(last_move.unwrap() > 0, "{axis:?} last moved {last_move:?}");
  }
}

#[test]
fn the_coordinator_position_matches_the_commands_the_rig_received() {
  let scenario = realistic(
    41,
    PerAxis {
      m1_h: 20.0,
      m1_v: 20.0,
      m2_h: -30.0,
      m2_v: -30.0,
    },
    [0.0, 0.0],
  );
  let rig = Rig::new(scenario);
  let (outcome, aligner) = align(&rig, Config::default());
  outcome.unwrap();
  for axis in Axis::ALL {
    assert_eq!(
      i64::from(aligner.axes().get(axis).pos_steps),
      rig.world().commanded_steps(axis)
    );
  }
}

#[test]
fn noise_far_above_the_tolerance_ends_in_a_fault_and_not_in_a_false_success() {
  let scenario = Scenario {
    beam: Beam::PLACEHOLDER.with_position_noise(0.05),
    misalignment_steps: PerAxis::splat(20.0),
    seed: 5,
    ..Scenario::IDEAL
  };
  let rig = Rig::new(scenario);
  let (outcome, _) = align(&rig, Config::default());
  assert!(outcome.is_err(), "{outcome:?}");
}
