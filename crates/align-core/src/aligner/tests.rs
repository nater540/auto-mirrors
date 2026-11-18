//! The aligner's own checks, and the procedure on a scripted machine where every number is exact.
//!
//! The procedure as a whole is held to a model of the machine in `align-sim`. That model has noise and
//! stick-slip, so it cannot tell a measurement from the mean of two, or one repeat measurement from none. The
//! scripted machine here can: it is linear, `m = m0 + J q` with `J = I / 256`, and every product is exact in `f32`.

extern crate std;

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};
use std::cell::RefCell;
use std::rc::Rc;
use std::vec;
use std::vec::Vec;

use super::*;
use crate::approach::LastDir;
use crate::config::DirSign;
use crate::quadrant::Reading;

/// A machine that is never reached: these tests stop before anything moves or measures.
struct Unreached;

impl Actuator for Unreached {
  async fn move_rel(&mut self, _axis: Axis, _delta_steps: i32) -> Result<i32, MoveFault> {
    Err(MoveFault::NoAnswer)
  }
}

impl BeamSensor for Unreached {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    Err(SensorFault::NoReading)
  }
}

const SUM_REF: f32 = 1.0;

/// A float field of the configuration, by name.
type Field = (&'static str, fn(&mut Config) -> &mut f32);

/// A change to the configuration, and the field its refusal names (`None`: accepted).
type Case = (fn(&mut Config), Option<&'static str>);

fn rejected(change: impl FnOnce(&mut Config)) -> Option<&'static str> {
  let mut config = Config::default();
  change(&mut config);
  match check_config(&config, SUM_REF) {
    Ok(()) => None,
    Err(AlignFault::Config(what)) => Some(what),
    Err(other) => panic!("not a configuration fault: {other:?}")
  }
}

#[test]
fn the_starting_configuration_is_accepted() {
  assert_eq!(check_config(&Config::default(), SUM_REF), Ok(()));
}

#[test]
fn a_reference_sum_that_is_not_a_positive_number_is_refused() {
  for sum_ref in [f32::NAN, 0.0, -1.0, f32::INFINITY] {
    let made = Aligner::new(Unreached, Unreached, Config::default(), sum_ref);
    assert!(
      matches!(made, Err(AlignFault::Config(what)) if what.contains("reference sum")),
      "{sum_ref}"
    );
  }
}

#[test]
fn every_threshold_that_is_not_a_number_is_refused() {
  let fields: [Field; 17] = [
    ("soft_limit_margin_frac", |c| &mut c.motion.soft_limit_margin_frac),
    ("sum_min_frac", |c| &mut c.sensor.sum_min_frac),
    ("edge_max_norm", |c| &mut c.sensor.edge_max_norm),
    ("solver_max_norm", |c| &mut c.sensor.solver_max_norm),
    ("offset_warn_frac", |c| &mut c.sensor.offset_warn_frac),
    ("noise_sigma_norm", |c| &mut c.sensor.noise_sigma_norm),
    ("alpha", |c| &mut c.solver.alpha),
    ("lambda_frac", |c| &mut c.solver.lambda_frac),
    ("converge_tol_norm", |c| &mut c.solver.converge_tol_norm),
    ("cond_max", |c| &mut c.solver.cond_max),
    ("step_clamp_pokes", |c| &mut c.solver.step_clamp_pokes),
    ("prediction_mismatch_frac", |c| &mut c.solver.prediction_mismatch_frac),
    ("broyden_min_step_norm_steps", |c| {
      &mut c.solver.broyden_min_step_norm_steps
    }),
    ("lambda_adjust_factor", |c| &mut c.solver.lambda_adjust_factor),
    ("min_residual_reduction_frac", |c| {
      &mut c.solver.min_residual_reduction_frac
    }),
    ("min_column_sigmas", |c| &mut c.solver.min_column_sigmas),
    ("judge_noise_sigmas", |c| &mut c.solver.judge_noise_sigmas)
  ];
  for (name, field) in fields {
    let what = rejected(|config| *field(config) = f32::NAN);
    assert!(what.is_some_and(|what| what.starts_with(name)), "{name}: {what:?}");
  }
}

#[test]
fn each_range_is_held_at_its_edges() {
  let cases: [Case; 30] = [
    (|c| c.motion.poke_steps = 0, Some("poke_steps")),
    (|c| c.motion.poke_steps = 1, None),
    (|c| c.motion.overshoot_steps = 0, Some("overshoot_steps")),
    (|c| c.motion.overshoot_steps = 1, None),
    (|c| c.motion.max_move_steps = 0, Some("max_move_steps")),
    (
      |c| c.motion.soft_min_steps.m2_v = c.motion.soft_max_steps.m2_v + 1,
      Some("soft_min_steps")
    ),
    (|c| c.motion.soft_min_steps.m2_v = c.motion.soft_max_steps.m2_v, None),
    (
      |c| c.motion.soft_limit_margin_frac = 1.01,
      Some("soft_limit_margin_frac")
    ),
    (|c| c.motion.soft_limit_margin_frac = 0.0, None),
    (|c| c.sensor.sum_min_frac = 0.0, Some("sum_min_frac")),
    (|c| c.sensor.sum_min_frac = 1.0, None),
    (|c| c.sensor.edge_max_norm = 1.01, Some("edge_max_norm")),
    (
      |c| c.sensor.solver_max_norm = c.sensor.edge_max_norm + 0.01,
      Some("solver_max_norm")
    ),
    (|c| c.sensor.solver_max_norm = c.sensor.edge_max_norm, None),
    (|c| c.sensor.noise_sigma_norm = 0.0, Some("noise_sigma_norm")),
    (|c| c.solver.alpha = 0.0, Some("alpha")),
    (|c| c.solver.alpha = 1.0, None),
    (|c| c.solver.lambda_frac = 0.0, None),
    (|c| c.solver.converge_tol_norm = 0.0, Some("converge_tol_norm")),
    (|c| c.solver.max_iterations = 0, Some("max_iterations")),
    (|c| c.solver.cond_max = 0.99, Some("cond_max")),
    (|c| c.solver.cond_max = 1.0, None),
    (|c| c.solver.step_clamp_pokes = 0.0, Some("step_clamp_pokes")),
    (
      |c| c.solver.prediction_mismatch_frac = 0.0,
      Some("prediction_mismatch_frac")
    ),
    (|c| c.solver.lambda_adjust_factor = 0.99, Some("lambda_adjust_factor")),
    (|c| c.solver.lambda_adjust_factor = 1.0, None),
    (
      |c| c.solver.min_residual_reduction_frac = 1.01,
      Some("min_residual_reduction_frac")
    ),
    (
      |c| c.solver.poor_steps_before_recalibration = 0,
      Some("poor_steps_before_recalibration")
    ),
    (|c| c.solver.min_column_sigmas = 0.0, Some("min_column_sigmas")),
    (|c| c.solver.suspect_fault_streak = 0, Some("suspect_fault_streak"))
  ];
  for (index, (change, expected)) in cases.into_iter().enumerate() {
    let what = rejected(change);
    match expected {
      None => assert_eq!(what, None, "case {index}"),
      Some(name) => assert!(
        what.is_some_and(|what| what.starts_with(name)),
        "case {index}: {what:?}"
      )
    }
  }
}

#[test]
fn the_bring_up_ranges_are_held_at_their_edges() {
  use crate::bringup::{MAX_MOVES_PER_HALF, MAX_REPEAT_SHOTS};
  let cases: [Case; 20] = [
    (|c| c.bring_up.repeat_shots = 1, Some("repeat_shots")),
    (|c| c.bring_up.repeat_shots = 2, None),
    (|c| c.bring_up.repeat_shots = MAX_REPEAT_SHOTS, None),
    (|c| c.bring_up.repeat_shots = MAX_REPEAT_SHOTS + 1, Some("repeat_shots")),
    (
      |c| c.bring_up.backlash_trials_steps = [0, 16, 32, 64, 128],
      Some("backlash_trials_steps")
    ),
    (
      |c| c.bring_up.backlash_trials_steps = [8, 16, 16, 64, 128],
      Some("backlash_trials_steps")
    ),
    (|c| c.bring_up.backlash_trials_steps = [1, 2, 3, 4, 5], None),
    (|c| c.bring_up.detect_sigmas = 0.0, Some("detect_sigmas")),
    (|c| c.bring_up.detect_sigmas = f32::NAN, Some("detect_sigmas")),
    (
      |c| c.bring_up.hysteresis_amplitude_pokes = 0,
      Some("hysteresis_amplitude_pokes")
    ),
    (
      |c| c.bring_up.hysteresis_moves_per_half = 0,
      Some("hysteresis_moves_per_half")
    ),
    (|c| c.bring_up.hysteresis_moves_per_half = 1, None),
    (|c| c.bring_up.hysteresis_moves_per_half = MAX_MOVES_PER_HALF, None),
    (
      |c| c.bring_up.hysteresis_moves_per_half = MAX_MOVES_PER_HALF + 1,
      Some("hysteresis_moves_per_half")
    ),
    (|c| c.bring_up.overshoot_floor_steps = -1, Some("overshoot_floor_steps")),
    (|c| c.bring_up.overshoot_floor_steps = 0, None),
    (|c| c.bring_up.noise_floor_norm = -0.001, Some("noise_floor_norm")),
    (|c| c.bring_up.noise_floor_norm = f32::NAN, Some("noise_floor_norm")),
    (
      |c| c.bring_up.noise_floor_norm = f32::INFINITY,
      Some("noise_floor_norm")
    ),
    (|c| c.bring_up.noise_floor_norm = 0.0, None)
  ];
  for (index, (change, expected)) in cases.into_iter().enumerate() {
    let what = rejected(change);
    match expected {
      None => assert_eq!(what, None, "case {index}"),
      Some(name) => assert!(
        what.is_some_and(|what| what.starts_with(name)),
        "case {index}: {what:?}"
      )
    }
  }
}

fn aligner(config: &Config) -> Aligner<Unreached, Unreached> {
  Aligner::new(Unreached, Unreached, *config, SUM_REF).unwrap()
}

#[test]
fn a_checked_approach_ends_where_its_legs_do_and_going_plus() {
  let aligner = aligner(&Config::default());
  for (from_dir, delta_steps) in [(LastDir::Unknown, 10), (LastDir::Minus, -30), (LastDir::Plus, 10)] {
    let from = Planned {
      pos_steps: 0,
      last_dir: from_dir
    };
    let to = aligner.check_approach(Axis::M2V, from, delta_steps).unwrap();
    assert_eq!(
      (to.pos_steps, to.last_dir),
      (i64::from(delta_steps), LastDir::Plus),
      "{from_dir:?} {delta_steps}"
    );
  }
  let still = Planned {
    pos_steps: 7,
    last_dir: LastDir::Plus
  };
  let to = aligner.check_approach(Axis::M2V, still, 0).unwrap();
  assert_eq!(
    (to.pos_steps, to.last_dir),
    (7, LastDir::Plus),
    "no legs: nothing changes"
  );
}

#[test]
fn checked_legs_leave_the_axis_going_the_way_its_last_moving_leg_went() {
  let aligner = aligner(&Config::default());
  let cases: [(LastDir, &[i32], i64, LastDir); 6] = [
    (LastDir::Plus, &[-5], -5, LastDir::Minus),
    (LastDir::Minus, &[5], 5, LastDir::Plus),
    (LastDir::Unknown, &[5, -8], -3, LastDir::Minus),
    (LastDir::Unknown, &[-8, 5], -3, LastDir::Plus),
    (LastDir::Minus, &[0], 0, LastDir::Minus),
    (LastDir::Unknown, &[-5, 0], -5, LastDir::Minus)
  ];
  for (from_dir, legs, pos_steps, last_dir) in cases {
    let from = Planned {
      pos_steps: 0,
      last_dir: from_dir
    };
    let to = aligner.check_legs(Axis::M1H, from, legs.iter().copied()).unwrap();
    assert_eq!(
      (to.pos_steps, to.last_dir),
      (pos_steps, last_dir),
      "{from_dir:?} {legs:?}"
    );
  }
}

#[test]
fn a_procedure_s_starting_approach_is_checked_in_full_even_after_a_plus_move() {
  let mut config = Config::default();
  config.motion.soft_min_steps = PerAxis::splat(0);
  config.motion.soft_max_steps = PerAxis::splat(1000);
  // [-100, 1100] with the margin. The approach overshoots by 120 even from plus, which only the bottom axis leaves.
  let plus_at = |pos_steps| AxisState {
    pos_steps,
    last_dir: LastDir::Plus,
    ..AxisState::default()
  };
  let mut axes = PerAxis::splat(plus_at(200));
  axes.m2_h = plus_at(0);
  let aligner = aligner(&config).with_axes(axes);
  assert_eq!(
    aligner.check_start(Axis::M2H).map(|to| to.pos_steps),
    Err(AlignFault::SoftLimit {
      axis: Axis::M2H,
      target_steps: -120
    })
  );
  let to = aligner.check_start(Axis::M1V).unwrap();
  assert_eq!((to.pos_steps, to.last_dir), (200, LastDir::Plus));
}

#[test]
fn a_checked_approach_is_refused_at_the_first_leg_that_leaves_the_limits() {
  let mut config = Config::default();
  config.motion.soft_min_steps = PerAxis::splat(0);
  config.motion.soft_max_steps = PerAxis::splat(100);
  let aligner = aligner(&config);
  let from = Planned {
    pos_steps: 0,
    last_dir: LastDir::Unknown
  };
  // [-10, 110] with the margin. From an unknown direction, 10 steps first go to 10 - 120.
  assert_eq!(
    aligner.check_approach(Axis::M1V, from, 10).map(|to| to.pos_steps),
    Err(AlignFault::SoftLimit {
      axis: Axis::M1V,
      target_steps: -110
    })
  );
  let plus = Planned {
    last_dir: LastDir::Plus,
    ..from
  };
  assert_eq!(
    aligner.check_approach(Axis::M1V, plus, 110).map(|to| to.pos_steps),
    Ok(110)
  );
  assert!(matches!(
    aligner.check_approach(Axis::M1V, plus, 111),
    Err(AlignFault::SoftLimit { target_steps: 111, .. })
  ));
}

#[test]
fn a_leg_of_exactly_the_per_move_cap_is_allowed() {
  let config = Config::default();
  let aligner = aligner(&config);
  let cap = config.motion.max_move_steps;
  let from = Planned {
    pos_steps: 0,
    last_dir: LastDir::Plus
  };
  assert!(aligner.check_approach(Axis::M1H, from, cap).is_ok());
  assert_eq!(
    aligner.check_approach(Axis::M1H, from, cap + 1).map(|to| to.pos_steps),
    Err(AlignFault::MoveTooLong {
      axis: Axis::M1H,
      steps: cap + 1
    })
  );
}

// --- the scripted machine --------------------------------------------------------------------------------------

/// Runs a future that never waits: the scripted machine answers at once.
fn run<F: Future>(future: F) -> F::Output {
  let mut future = pin!(future);
  match future.as_mut().poll(&mut Context::from_waker(Waker::noop())) {
    Poll::Ready(output) => output,
    Poll::Pending => panic!("the scripted machine never waits")
  }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Event {
  Move(Axis, i32),
  Measure
}

/// What the machine does, and what it did.
struct Script {
  /// The true response, normalized units per step.
  jacobian: Jacobian,
  /// The measurement at position zero.
  m0: Vector4<f32>,
  /// The fraction of a command each axis really moves, chosen by how many measurements came before the move.
  /// The last entry holds from then on.
  response: Vec<[f32; 4]>,
  /// Added to the measurement vector, by measurement; nothing once the list is used up.
  noise: Vec<[f32; 4]>,
  /// A shift of all three readings, by measurement: a head offset, which changes no component of `m`.
  offset: Vec<f32>,
  /// The sensor node's own rejection of a measurement, by measurement; none once the list is used up.
  rejections: Vec<Option<ReadingFault>>,
  /// Where each axis really is, in steps.
  q: Vector4<f32>,
  /// What the node counts: the sum of the steps it ran.
  commanded: [i32; 4],
  /// How the node answers each move, by the number of moves before it; done once the list is used up.
  answers: Vec<Answer>,
  log: Vec<Event>
}

#[derive(Clone)]
struct Machine(Rc<RefCell<Script>>);

impl Machine {
  fn new(m0: [f32; 4]) -> Self {
    Self(Rc::new(RefCell::new(Script {
      jacobian: Jacobian::identity() / 256.0,
      m0: Vector4::from(m0),
      response: vec![[1.0; 4]],
      noise: Vec::new(),
      offset: Vec::new(),
      rejections: Vec::new(),
      q: Vector4::zeros(),
      commanded: [0; 4],
      answers: Vec::new(),
      log: Vec::new()
    })))
  }

  fn with(self, change: impl FnOnce(&mut Script)) -> Self {
    change(&mut self.0.borrow_mut());
    self
  }

  fn log(&self) -> Vec<Event> {
    self.0.borrow().log.clone()
  }

  /// The net command of each axis in each run of moves: one entry per step taken.
  fn steps(&self) -> Vec<[i32; 4]> {
    let mut steps = Vec::new();
    let mut moving: Option<[i32; 4]> = None;
    for event in self.log() {
      match event {
        Event::Move(axis, delta_steps) => moving.get_or_insert([0; 4])[axis.index()] += delta_steps,
        Event::Measure => steps.extend(moving.take())
      }
    }
    steps.extend(moving);
    steps
  }

  /// How many measurements were taken in a row, for each run of them.
  fn measure_runs(&self) -> Vec<usize> {
    let mut runs = Vec::new();
    let mut count = 0;
    for event in self.log() {
      match event {
        Event::Measure => count += 1,
        Event::Move(..) if count > 0 => runs.push(core::mem::take(&mut count)),
        Event::Move(..) => {}
      }
    }
    if count > 0 {
      runs.push(count);
    }
    runs
  }
}

fn measured_before(script: &Script) -> usize {
  script.log.iter().filter(|event| **event == Event::Measure).count()
}

/// How the node answers a move.
#[derive(Clone, Copy, Debug)]
enum Answer {
  /// It ran the move and says where it ended.
  Done,
  /// It ran the move, and the answer was lost.
  Lost,
  /// It ran this many steps of the move and stopped.
  Stopped(i32),
  /// It ran the move but reports this position, which disagrees with the commands.
  Reports(i32)
}

impl Actuator for Machine {
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    let mut script = self.0.borrow_mut();
    let moves_before = script
      .log
      .iter()
      .filter(|event| matches!(event, Event::Move(..)))
      .count();
    let answer = script.answers.get(moves_before).copied().unwrap_or(Answer::Done);
    let ran_steps = match answer {
      Answer::Stopped(ran_steps) => ran_steps,
      Answer::Done | Answer::Lost | Answer::Reports(_) => delta_steps
    };
    let index = measured_before(&script);
    let response = script.response.get(index).or(script.response.last()).copied().unwrap();
    #[expect(clippy::cast_precision_loss, reason = "test steps are small")]
    let moved = ran_steps as f32 * response[axis.index()];
    script.q[axis.index()] += moved;
    script.commanded[axis.index()] += ran_steps;
    script.log.push(Event::Move(axis, delta_steps));
    let pos_steps = script.commanded[axis.index()];
    match answer {
      Answer::Done => Ok(pos_steps),
      Answer::Lost => Err(MoveFault::NoAnswer),
      Answer::Stopped(_) => Err(MoveFault::Stopped { pos_steps }),
      Answer::Reports(reported) => Ok(reported)
    }
  }
}

impl BeamSensor for Machine {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    let mut script = self.0.borrow_mut();
    let index = measured_before(&script);
    let noise = Vector4::from(script.noise.get(index).copied().unwrap_or([0.0; 4]));
    let offset = script.offset.get(index).copied().unwrap_or(0.0);
    let rejection = script.rejections.get(index).copied().flatten();
    let m = script.m0 + script.jacobian * script.q + noise;
    let reading = |h_norm, v_norm| Reading {
      h_norm,
      v_norm,
      sum: 1.0
    };
    let a = reading(offset, offset);
    let b = reading(a.h_norm + m.x, a.v_norm + m.y);
    let c = reading(b.h_norm + m.z, b.v_norm + m.w);
    script.log.push(Event::Measure);
    match rejection {
      Some(fault) => Err(SensorFault::Rejected(fault)),
      None => Ok(Shots { a, b, c })
    }
  }
}

/// The starting values with a quiet sensor: the scripted machine has no noise but what a test adds, and the low
/// judging floor (0.0146) lets the small steps of these tests be judged.
fn quiet() -> Config {
  let mut config = Config::default();
  config.sensor.noise_sigma_norm = 0.001;
  config
}

/// An aligner on the machine, which believes the true Jacobian and finds every axis in the standard direction.
fn on(machine: &Machine, config: &Config) -> Aligner<Machine, Machine> {
  let jacobian = machine.0.borrow().jacobian;
  let calibration = Calibration {
    jacobian,
    q0_steps: [0; 4],
    cond: 1.0,
    dir_sign: config.mechanics.dir_sign
  };
  let standard = AxisState {
    last_dir: LastDir::Plus,
    ..AxisState::default()
  };
  Aligner::new(machine.clone(), machine.clone(), *config, SUM_REF)
    .unwrap()
    .with_calibration(calibration)
    .with_axes(PerAxis::splat(standard))
}

// --- judging a step --------------------------------------------------------------------------------------------

#[test]
fn a_step_that_went_as_predicted_is_measured_once() {
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]);
  let report = run(on(&machine, &quiet()).align()).unwrap();
  assert_eq!(
    machine.steps().first(),
    Some(&[-31, 0, 0, 0]),
    "0.6 * 0.2 * 256 / 1.0025"
  );
  assert_eq!(machine.measure_runs().get(..2), Some([1, 1].as_slice()));
  assert!(report.iterations >= 2);
}

#[test]
fn a_step_the_beam_ignored_is_measured_again_and_the_second_in_a_row_is_a_fault() {
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]).with(|script| script.response = vec![[0.0, 1.0, 1.0, 1.0]]);
  let outcome = run(on(&machine, &quiet()).align());
  assert_eq!(outcome, Err(AlignFault::AxisNotResponding { axis: Axis::M1H }));
  assert_eq!(
    machine.measure_runs(),
    [1, 2, 2],
    "one look before, then two after each of two steps"
  );
}

#[test]
fn the_suspect_streak_counts_one_axis_at_a_time() {
  // The first step finds M1H dead, the second M1V: two suspects in a row, but not on one axis. M1V is misaligned
  // only after the first step (a shift of every later measurement), so that each step is dominated by its dead
  // axis; from a misalignment present from the start, no two steps in a row can both be.
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]).with(|script| {
    script.response = Vec::from([
      [1.0; 4],
      [0.0, 1.0, 1.0, 1.0],
      [0.0, 1.0, 1.0, 1.0],
      [1.0, 0.0, 1.0, 1.0],
      [1.0, 0.0, 1.0, 1.0],
      [1.0; 4]
    ]);
    script.noise = core::iter::once([0.0; 4])
      .chain(core::iter::repeat_n([0.0, 0.2, 0.0, 0.0], 30))
      .collect();
  });
  let outcome = run(on(&machine, &quiet()).align());
  assert!(
    !matches!(outcome, Err(AlignFault::AxisNotResponding { .. })),
    "{outcome:?}"
  );
  assert_eq!(machine.measure_runs().get(..3), Some([1, 2, 2].as_slice()));
}

#[test]
fn a_step_whose_readings_leave_the_linear_range_is_not_judged() {
  // M1H is dead throughout. After the first step every reading is shifted past the linear range (a head offset, so
  // the measurement vector is unchanged). That measurement ends one step and starts the next, so both are coarse:
  // each is measured once and not counted, and the fault comes two steps later than without the shift.
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]).with(|script| {
    script.response = vec![[0.0, 1.0, 1.0, 1.0]];
    script.offset = vec![0.0, 0.65];
  });
  let outcome = run(on(&machine, &quiet()).align());
  assert_eq!(outcome, Err(AlignFault::AxisNotResponding { axis: Axis::M1H }));
  assert_eq!(machine.measure_runs(), [1, 1, 1, 2, 2]);
}

#[test]
fn two_poor_steps_in_a_row_ask_for_a_recalibration() {
  // Each step removes about 60% of the residual; asking for 70% makes both poor.
  let mut config = quiet();
  config.solver.min_residual_reduction_frac = 0.7;
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]);
  let report = run(on(&machine, &config).align()).unwrap();
  assert_eq!(report.recalibrations, 1);
  let poke = config.motion.poke_steps;
  let steps = machine.steps();
  assert_eq!(steps.get(..2), Some([[-31, 0, 0, 0], [-12, 0, 0, 0]].as_slice()));
  assert!(
    machine.log().contains(&Event::Move(Axis::M2V, poke)),
    "the recalibration pokes M2V"
  );
}

/// With `lambda = sigma` the step is half the Gauss-Newton step; a third of that damping makes it 0.9 of it.
fn heavily_damped() -> Config {
  let mut config = quiet();
  config.solver.lambda_frac = 1.0;
  config.solver.alpha = 1.0;
  config
}

#[test]
fn a_good_step_divides_the_damping_by_three() {
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]);
  run(on(&machine, &heavily_damped()).align()).unwrap();
  // 0.5 * 0.2 * 256 = 25.6, leaving 0.0984 (good); then 0.9 * 0.0984 * 256 = 22.7.
  assert_eq!(
    machine.steps().get(..2),
    Some([[-26, 0, 0, 0], [-23, 0, 0, 0]].as_slice())
  );
}

#[test]
fn a_poor_step_multiplies_the_damping_by_three() {
  let mut config = heavily_damped();
  config.solver.min_residual_reduction_frac = 0.7;
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]);
  // Only the first two steps matter here; damped this hard, the attempts run out.
  let _outcome = run(on(&machine, &config).align());
  // 25.6 as before, now poor; then 0.1 * 0.0984 * 256 = 2.5.
  assert_eq!(
    machine.steps().get(..2),
    Some([[-26, 0, 0, 0], [-3, 0, 0, 0]].as_slice())
  );
}

#[test]
fn a_step_whose_prediction_is_exactly_the_judging_floor_is_judged() {
  // Binary-exact thresholds: a change sigma of 1/64 and one sigma make the floor 1/32, and 8 steps of 1/256 reach
  // it exactly. M1H is dead, so a judged step names it and two in a row are a fault.
  let mut config = Config::default();
  config.sensor.noise_sigma_norm = 1.0 / 128.0;
  config.solver.judge_noise_sigmas = 1.0;
  crate::test_support::assert_close(
    calibration::judging_floor(&config.sensor, &config.solver),
    1.0 / 32.0,
    0.0
  );
  let machine = Machine::new([0.05, 0.0, 0.0, 0.0]).with(|script| script.response = vec![[0.0, 1.0, 1.0, 1.0]]);
  let outcome = run(on(&machine, &config).align());
  assert_eq!(machine.steps().first(), Some(&[-8, 0, 0, 0]));
  assert_eq!(outcome, Err(AlignFault::AxisNotResponding { axis: Axis::M1H }));
}

// --- a step that went the wrong way ---------------------------------------------------------------------------

/// The quiet configuration, with one poor step enough for a recalibration: a step's verdict shows in what follows
/// it, a recalibration's poke of M1H or another solver step.
fn one_poor_step() -> Config {
  let mut config = quiet();
  config.solver.poor_steps_before_recalibration = 1;
  config
}

/// The run of moves right after the first step: the recalibration's poke of M1H, or a second solver step.
fn after_the_first_step(machine: &Machine) -> Option<[i32; 4]> {
  machine.steps().get(1).copied()
}

const POKE: [i32; 4] = [45, 0, 0, 0];

#[test]
fn a_calibration_measured_under_other_direction_signs_is_measured_again() {
  let mut config = quiet();
  config.mechanics.dir_sign.m2_v = DirSign::Reversed;
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]);
  let forward = Calibration {
    jacobian: Jacobian::identity() / 256.0,
    q0_steps: [0; 4],
    cond: 1.0,
    dir_sign: PerAxis::splat(DirSign::Forward)
  };
  let mut aligner = Aligner::new(machine.clone(), machine.clone(), config, SUM_REF)
    .unwrap()
    .with_calibration(forward);
  let report = run(aligner.align()).unwrap();
  assert_eq!(report.recalibrations, 1);
  assert_eq!(
    aligner.calibration().map(|calibration| calibration.dir_sign),
    Some(config.mechanics.dir_sign)
  );
}

#[test]
fn a_step_too_small_to_judge_is_poor_if_the_residual_grew_by_more_than_noise() {
  // M1H runs backward. Binary-exact thresholds: the noise of a change is 1/64, and so is the growth after the
  // first step (4 steps of 1/256), which is therefore not poor. The second step, 6 steps, is.
  let mut config = one_poor_step();
  config.sensor.noise_sigma_norm = 1.0 / 128.0;
  config.solver.judge_noise_sigmas = 1.0;
  let machine =
    Machine::new([6.75 / 256.0, 0.0, 0.0, 0.0]).with(|script| script.response = vec![[-1.0, 1.0, 1.0, 1.0]]);
  let _outcome = run(on(&machine, &config).align());
  assert_eq!(
    machine.steps().get(..3),
    Some([[-4, 0, 0, 0], [-6, 0, 0, 0], POKE].as_slice())
  );
}

#[test]
fn a_coarse_step_is_poor_if_the_residual_grew() {
  // M1H runs backward, and every reading is shifted just past the linear range.
  let machine = Machine::new([0.03, 0.0, 0.0, 0.0]).with(|script| {
    script.response = vec![[-1.0, 1.0, 1.0, 1.0]];
    script.offset = vec![0.62; 40];
  });
  let _outcome = run(on(&machine, &one_poor_step()).align());
  assert_eq!(after_the_first_step(&machine), Some(POKE));
}

#[test]
fn a_clamped_step_is_poor_if_the_residual_grew() {
  // The clamp holds every step to 4.5 steps. The first one goes as commanded, but both looks after it read M1V
  // 0.08 further out: no axis fell short, and the residual grew.
  let mut config = one_poor_step();
  config.solver.step_clamp_pokes = 0.1;
  let machine = Machine::new([0.1, 0.0, 0.0, 0.0])
    .with(|script| script.noise = vec![[0.0; 4], [0.0, 0.08, 0.0, 0.0], [0.0, 0.08, 0.0, 0.0]]);
  let _outcome = run(on(&machine, &config).align());
  assert_eq!(machine.steps().first(), Some(&[-5, 0, 0, 0]));
  assert_eq!(after_the_first_step(&machine), Some(POKE));
}

#[test]
fn a_suspect_step_that_grew_the_residual_is_poor_as_well() {
  // M1H runs backward: the judged step names it, and the residual grew, so the recalibration comes before a
  // second suspect could make it a fault.
  let machine = Machine::new([0.2, 0.0, 0.0, 0.0]).with(|script| script.response = vec![[-1.0, 1.0, 1.0, 1.0]]);
  let outcome = run(on(&machine, &one_poor_step()).align());
  // After the step: its measurement, the repeat that a suspect step gets, and the recalibration's baseline.
  assert_eq!(machine.measure_runs().get(..2), Some([1, 3].as_slice()));
  assert_eq!(after_the_first_step(&machine), Some(POKE));
  assert!(
    !matches!(outcome, Err(AlignFault::AxisNotResponding { .. })),
    "{outcome:?}"
  );
}

// --- what the node says about a move ---------------------------------------------------------------------------

/// An aligner on a machine whose node answers as scripted, every axis in the standard direction.
fn answering(answers: &[Answer], config: &Config) -> (Machine, Aligner<Machine, Machine>) {
  let machine = Machine::new([0.0; 4]).with(|script| script.answers = answers.to_vec());
  let aligner = on(&machine, config);
  (machine, aligner)
}

/// The review of 2026-11-15: the node ran a move to 400 and its answer was lost; the aligner kept 0, and a retry
/// took the screw to 800, twice the soft limit.
#[test]
fn after_an_unanswered_move_the_axis_waits_for_its_position_from_the_node() {
  let mut config = quiet();
  config.motion.soft_min_steps = PerAxis::splat(-400);
  config.motion.soft_max_steps = PerAxis::splat(400);
  config.motion.soft_limit_margin_frac = 0.0;
  let (machine, mut aligner) = answering(&[Answer::Lost], &config);
  assert_eq!(
    run(aligner.approach(Axis::M1H, 400)),
    Err(AlignFault::Move {
      axis: Axis::M1H,
      fault: MoveFault::NoAnswer
    })
  );
  assert!(aligner.axes().m1_h.lost && aligner.axes().m1_h.suspect);
  assert_eq!(
    run(aligner.approach(Axis::M1H, 400)),
    Err(AlignFault::PositionUnknown { axis: Axis::M1H })
  );
  assert_eq!(
    run(aligner.calibrate()),
    Err(AlignFault::PositionUnknown { axis: Axis::M1H })
  );
  assert_eq!(machine.log(), [Event::Move(Axis::M1H, 400)], "nothing else moved");
  // The coordinator reads the node's STATUS and gives the position back: now the limit holds.
  aligner.restore_position(Axis::M1H, 400);
  assert!(!aligner.axes().m1_h.lost && aligner.axes().m1_h.suspect);
  assert_eq!(
    run(aligner.approach(Axis::M1H, 400)),
    Err(AlignFault::SoftLimit {
      axis: Axis::M1H,
      target_steps: 800
    })
  );
  assert_eq!(machine.0.borrow().commanded[0], 400);
}

#[test]
fn a_stopped_move_leaves_the_axis_where_the_node_says_it_stopped() {
  let (_machine, mut aligner) = answering(&[Answer::Stopped(30)], &quiet());
  assert_eq!(
    run(aligner.approach(Axis::M2V, 100)),
    Err(AlignFault::Move {
      axis: Axis::M2V,
      fault: MoveFault::Stopped { pos_steps: 30 }
    })
  );
  let state = aligner.axes().m2_v;
  assert_eq!(
    (state.pos_steps, state.suspect, state.lost, state.last_dir),
    (30, true, false, LastDir::Unknown)
  );
}

#[test]
fn the_position_the_node_reports_is_adopted_and_a_disagreement_makes_the_axis_suspect() {
  let (_machine, mut aligner) = answering(&[Answer::Done, Answer::Reports(203)], &quiet());
  run(aligner.approach(Axis::M1V, 100)).unwrap();
  assert_eq!(
    (aligner.axes().m1_v.pos_steps, aligner.axes().m1_v.suspect),
    (100, false)
  );
  run(aligner.approach(Axis::M1V, 100)).unwrap();
  assert_eq!(
    (aligner.axes().m1_v.pos_steps, aligner.axes().m1_v.suspect),
    (203, true)
  );
}
// --- a measurement the sensor node rejects ----------------------------------------------------------------------

#[test]
fn a_measurement_the_sensor_node_rejects_is_taken_again() {
  let saturated = Some(ReadingFault::Saturated);
  let machine = Machine::new([0.0; 4]).with(|script| script.rejections = vec![saturated, saturated]);
  let report = run(on(&machine, &quiet()).align()).unwrap();
  assert_eq!(report.iterations, 0);
  assert_eq!(
    machine.measure_runs(),
    [4],
    "two rejected, then a success and its confirmation"
  );
}

#[test]
fn as_many_rejections_in_a_row_as_invalid_shots_allowed_are_a_reading_fault() {
  let config = quiet();
  let overlap = Some(ReadingFault::Overlap);
  let machine = Machine::new([0.0; 4]).with(|script| script.rejections = vec![overlap; 3]);
  assert_eq!(config.sensor.max_invalid_shots, 3);
  assert_eq!(
    run(on(&machine, &config).align()),
    Err(AlignFault::Reading(ReadingFault::Overlap))
  );
  assert_eq!(machine.measure_runs(), [3]);
}

// --- confirming a success --------------------------------------------------------------------------------------

#[test]
fn a_success_is_the_mean_of_two_measurements() {
  // Already aligned. The second look reads 0.03, outside the tolerance, but the mean, 0.015, is inside.
  let machine = Machine::new([0.0; 4]).with(|script| script.noise = vec![[0.0; 4], [0.03, 0.0, 0.0, 0.0]]);
  let report = run(on(&machine, &quiet()).align()).unwrap();
  assert_eq!((report.iterations, report.residual_norm), (0, 0.015));
  assert_eq!(machine.log(), [Event::Measure, Event::Measure]);
}

#[test]
fn a_mean_outside_the_tolerance_is_corrected_from() {
  // The first look reads 0.02, the second 0.03: the step is taken from the mean, 0.025, not from either.
  let machine = Machine::new([0.0; 4]).with(|script| script.noise = vec![[0.02, 0.0, 0.0, 0.0], [0.03, 0.0, 0.0, 0.0]]);
  run(on(&machine, &quiet()).align()).unwrap();
  assert_eq!(
    machine.steps().first(),
    Some(&[-4, 0, 0, 0]),
    "0.6 * 0.025 * 255.4 = 3.8; from 0.03 it would be 5"
  );
}

#[test]
fn a_mean_is_in_the_linear_range_only_if_both_of_its_measurements_are() {
  // M1H is dead. The first look is inside the tolerance but outside the linear range (shifted readings); the
  // second is linear but reads 0.035. Their mean, 0.025, is corrected from as a coarse measurement: the first step
  // is not judged, so the fault comes after the third step instead of the second.
  let machine = Machine::new([0.025, 0.0, 0.0, 0.0]).with(|script| {
    script.response = vec![[0.0, 1.0, 1.0, 1.0]];
    script.noise = vec![[-0.01, 0.0, 0.0, 0.0], [0.01, 0.0, 0.0, 0.0]];
    script.offset = vec![0.65];
  });
  let outcome = run(on(&machine, &quiet()).align());
  assert_eq!(outcome, Err(AlignFault::AxisNotResponding { axis: Axis::M1H }));
  assert_eq!(machine.steps().len(), 3);
}
