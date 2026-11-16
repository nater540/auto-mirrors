//! The aligner's own checks. The procedure itself is held to a model of the machine in `align-sim`.

use super::*;
use crate::approach::LastDir;

/// A machine that is never reached: these tests stop before anything moves or measures.
struct Unreached;

impl Actuator for Unreached {
  async fn move_rel(&mut self, _axis: Axis, _delta_steps: i32) -> Result<(), MoveFault> {
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
