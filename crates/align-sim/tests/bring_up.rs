//! The bring-up measurements (`align_core::bringup`) against the model of the machine, whose play and stick-slip
//! are known: the backlash bracket must hold the play, the hysteresis loop must be as wide as it, and both must leave
//! the axis where they found it.
//!
//! The model's screw follows its motor through `backlash_steps` of play, and its seat follows the screw once it is
//! `stick_steps` away, so the play a trial sees is the backlash plus up to the stick. Counts are over fixed seeds.

use align_core::aligner::{AlignFault, Aligner, AxisState};
use align_core::approach::LastDir;
use align_core::axis::{Axis, PerAxis};
use align_core::bringup::{BacklashReport, Entry, HysteresisReport, MAX_MOVES_PER_HALF, Phase, Play, Sample, Trial};
use align_core::config::{BringUp, Config, ReadAt};
use align_core::measurement::Shots;
use align_core::quadrant::Reading;
use align_core::rig::{BeamSensor, SensorFault};
use align_sim::block_on;
use align_sim::rig::Rig;
use align_sim::world::{AxisModel, REALISTIC_POSITION_NOISE_NORM, Record, Scenario};

/// Seeds for each case of a count.
const RUNS: u64 = 10;

/// The realistic machine with the given play and stick on every axis.
fn machine(backlash_steps: u32, stick_steps: u32, seed: u64) -> Rig {
  Rig::new(Scenario {
    axes: PerAxis::splat(AxisModel {
      backlash_steps,
      stick_steps,
      skip_probability: 0.0
    }),
    ..Scenario::realistic(seed)
  })
}

#[expect(
  clippy::unwrap_used,
  reason = "a test helper; a configuration the aligner refuses fails the test"
)]
fn aligner(rig: &Rig) -> Aligner<Rig, Rig> {
  rig.aligner(Config::default()).unwrap()
}

fn backlash(aligner: &mut Aligner<Rig, Rig>, axis: Axis) -> (Result<BacklashReport, AlignFault>, Vec<Entry>) {
  let mut entries = Vec::new();
  let report = block_on(aligner.measure_backlash(axis, &mut |entry| entries.push(*entry)));
  (report, entries)
}

fn hysteresis(aligner: &mut Aligner<Rig, Rig>, axis: Axis) -> (Result<HysteresisReport, AlignFault>, Vec<Entry>) {
  let mut entries = Vec::new();
  let report = block_on(aligner.measure_hysteresis(axis, &mut |entry| entries.push(*entry)));
  (report, entries)
}

#[expect(
  clippy::panic,
  reason = "a test helper; a play beyond the trials fails the test that expected one"
)]
fn bracket(report: &BacklashReport) -> (i32, i32) {
  match report.play {
    Play::Bracket {
      above_steps,
      up_to_steps
    } => (above_steps, up_to_steps),
    Play::Beyond { steps } => panic!("no trial moved the beam, up to {steps} steps")
  }
}

fn samples(entries: &[Entry]) -> Vec<Sample> {
  entries
    .iter()
    .filter_map(|entry| match entry {
      Entry::Sample(sample) => Some(*sample),
      Entry::Trial(_) => None
    })
    .collect()
}

fn trials(entries: &[Entry]) -> Vec<Trial> {
  entries
    .iter()
    .filter_map(|entry| match entry {
      Entry::Trial(trial) => Some(*trial),
      Entry::Sample(_) => None
    })
    .collect()
}

/// Each trial's steps, and whether it moved the beam.
fn ran(entries: &[Entry]) -> Vec<(i32, bool)> {
  trials(entries).iter().map(|trial| (trial.steps, trial.moved)).collect()
}

/// A machine with no noise, no stick, and the given play on every axis.
fn noiseless(backlash_steps: u32) -> Rig {
  Rig::new(Scenario {
    axes: PerAxis::splat(AxisModel {
      backlash_steps,
      stick_steps: 0,
      skip_probability: 0.0
    }),
    ..Scenario::IDEAL
  })
}

/// The moves the machine was asked for, by axis.
fn moves(rig: &Rig) -> Vec<(Axis, i32)> {
  rig
    .world()
    .log()
    .iter()
    .filter_map(|record| match record {
      Record::Move { axis, delta_steps } => Some((*axis, *delta_steps)),
      Record::Measure { .. } => None
    })
    .collect()
}

#[expect(clippy::cast_precision_loss, reason = "step counts of a few hundred")]
fn float(steps: u32) -> f32 {
  steps as f32
}

#[test]
fn the_backlash_bracket_holds_the_play_on_every_axis() {
  for axis in Axis::ALL {
    for play in [10, 20, 40, 80] {
      for seed in 0..RUNS {
        let rig = machine(play, 0, seed);
        let report = backlash(&mut aligner(&rig), axis).0.unwrap();
        let (above, up_to) = bracket(&report);
        let case = format!("{axis:?}, play {play}, seed {seed}: ({above}, {up_to}]");
        // The beam cannot move before the play is taken up, and a trial past it by twice the resolution moves it.
        assert!(up_to > i32::try_from(play).unwrap(), "{case}");
        assert!(
          f32::from(i16::try_from(above).unwrap()) < float(play) + 2.0 * report.resolution_steps(),
          "{case}, resolution {}",
          report.resolution_steps()
        );
      }
    }
  }
}

#[test]
fn seat_stick_slip_adds_to_the_play_the_trials_see() {
  for seed in 0..RUNS {
    let rig = machine(20, 3, seed);
    let report = backlash(&mut aligner(&rig), Axis::M1H).0.unwrap();
    let (above, up_to) = bracket(&report);
    assert!(up_to >= 23, "seed {seed}: ({above}, {up_to}]");
  }
}

#[test]
fn a_play_beyond_the_largest_trial_has_no_overshoot_and_no_loop() {
  let rig = machine(200, 0, 1);
  let mut aligner = aligner(&rig);
  let report = backlash(&mut aligner, Axis::M1V).0.unwrap();
  assert_eq!(report.play, Play::Beyond { steps: 128 });
  assert_eq!(report.overshoot_steps, None);
  let loop_report = hysteresis(&mut aligner, Axis::M1V).0.unwrap();
  assert_eq!(
    loop_report.width_steps, None,
    "a cycle of 90 steps either way never takes up the play"
  );
}

#[test]
fn the_overshoot_is_twice_the_bracket_s_high_end_and_at_least_the_floor() {
  let rig = machine(40, 0, 1);
  let report = backlash(&mut aligner(&rig), Axis::M1H).0.unwrap();
  assert_eq!(bracket(&report), (32, 48));
  assert_eq!(report.overshoot_steps, Some(96));
  let rig = machine(10, 0, 1);
  let report = backlash(&mut aligner(&rig), Axis::M1H).0.unwrap();
  assert_eq!(bracket(&report), (8, 12));
  assert_eq!(report.overshoot_steps, Some(60));
}

#[test]
#[expect(clippy::float_cmp, reason = "a perfect machine reads exactly the same every time")]
fn on_a_perfect_machine_every_trial_moves_the_beam_and_the_loop_is_closed() {
  let rig = Rig::new(Scenario::IDEAL);
  let mut aligner = aligner(&rig);
  let report = backlash(&mut aligner, Axis::M2V).0.unwrap();
  assert_eq!(report.repeatability.sigma_norm, 0.0);
  assert_eq!(report.repeatability.drift_norm, 0.0);
  assert_eq!(
    bracket(&report),
    (0, 4),
    "the first trial and the bisection both moved the beam"
  );
  let loop_report = hysteresis(&mut aligner, Axis::M2V).0.unwrap();
  assert!(loop_report.width_steps.unwrap().abs() < 1e-3, "{loop_report}");
}

#[test]
fn a_trial_returns_without_overshoot_and_one_that_moved_the_beam_is_run_again() {
  let rig = machine(20, 0, 1);
  let (report, entries) = backlash(&mut aligner(&rig), Axis::M1H);
  assert_eq!(bracket(&report.unwrap()), (16, 24));
  let ran = ran(&entries);
  assert_eq!(
    ran,
    [(8, false), (16, false), (32, true), (32, true), (24, true), (24, true)]
  );
  // The standard approach, the poke and its undoing, then each trial back and plus again by the same steps.
  let mut expected = vec![-120, 120, 45, -165, 120];
  for (steps, _) in &ran {
    expected.extend([-steps, *steps]);
  }
  assert_eq!(
    moves(&rig),
    expected.into_iter().map(|steps| (Axis::M1H, steps)).collect::<Vec<_>>()
  );
}

#[test]
fn without_noise_a_trial_inside_the_play_does_not_count_as_moving_the_beam() {
  // Its shift is exactly zero, and so is the threshold: motion must be more than the threshold, not equal to it.
  let rig = noiseless(20);
  let (report, entries) = backlash(&mut aligner(&rig), Axis::M1H);
  assert_eq!(bracket(&report.unwrap()), (16, 24));
  assert_eq!(
    ran(&entries),
    [(8, false), (16, false), (32, true), (32, true), (24, true), (24, true)]
  );
}

#[test]
fn a_bracket_one_step_wide_is_not_bisected() {
  let rig = noiseless(0);
  let mut config = Config::default();
  config.bring_up.backlash_trials_steps = [1, 2, 4, 8, 16];
  let (report, entries) = backlash(&mut rig.aligner(config).unwrap(), Axis::M1H);
  assert_eq!(bracket(&report.unwrap()), (0, 1));
  assert_eq!(ran(&entries), [(1, true), (1, true)]);
}

#[test]
fn a_trial_that_moved_the_beam_measures_the_play_by_how_far_short_of_its_steps_the_beam_went() {
  for seed in 0..RUNS {
    let rig = machine(20, 0, seed);
    let (report, entries) = backlash(&mut aligner(&rig), Axis::M1H);
    // A shift has a third of the resolution as its sigma, in steps.
    let resolution = report.unwrap().resolution_steps();
    for trial in trials(&entries).iter().filter(|trial| trial.moved) {
      assert!(
        (trial.play_steps - 20.0).abs() < 2.0 * resolution,
        "seed {seed}, resolution {resolution}: {trial}"
      );
    }
  }
}

#[test]
fn the_backlash_search_logs_its_measurements_in_order() {
  let rig = machine(20, 0, 1);
  let (_, entries) = backlash(&mut aligner(&rig), Axis::M1H);
  let phases: Vec<(Phase, i32, i32)> = samples(&entries)
    .iter()
    .map(|sample| (sample.phase, sample.index, sample.offset_steps))
    .collect();
  let mut expected = vec![
    (Phase::Repeat, 0, 0),
    (Phase::Repeat, 1, 0),
    (Phase::Repeat, 2, 0),
    (Phase::Poke, 0, 45),
    (Phase::Reference, 0, 0),
  ];
  for steps in [8, 16, 32, 32, 24, 24] {
    expected.extend([(Phase::Out, steps, -steps), (Phase::Back, steps, 0)]);
  }
  assert_eq!(phases, expected);
}

#[test]
fn the_hysteresis_cycle_is_two_poke_sizes_either_way_in_eight_moves_each_way() {
  let rig = machine(20, 0, 1);
  let (_, entries) = hysteresis(&mut aligner(&rig), Axis::M2H);
  let points: Vec<(Phase, i32, i32)> = samples(&entries)
    .iter()
    .map(|sample| (sample.phase, sample.index, sample.offset_steps))
    .collect();
  let up = [-90, -68, -45, -23, 0, 22, 45, 67, 90];
  let mut expected = vec![(Phase::Repeat, 0, 0), (Phase::Repeat, 1, 0), (Phase::Repeat, 2, 0)];
  expected.extend((0..).zip(up).map(|(index, offset)| (Phase::Up, index, offset)));
  expected.extend(
    (1..)
      .zip(up.iter().rev().skip(1))
      .map(|(index, offset)| (Phase::Down, index, *offset))
  );
  assert_eq!(points, expected);
  assert!(trials(&entries).is_empty());
}

#[test]
fn the_hysteresis_loop_is_as_wide_as_the_play() {
  for axis in Axis::ALL {
    for play in [0, 20, 40] {
      for seed in 0..RUNS {
        let rig = machine(play, 0, seed);
        let report = hysteresis(&mut aligner(&rig), axis).0.unwrap();
        let width = report.width_steps.unwrap();
        assert!(
          (width - float(play)).abs() < 3.0,
          "{axis:?}, play {play}, seed {seed}: {report}"
        );
      }
    }
  }
}

#[test]
fn the_most_moves_per_half_the_configuration_allows_still_read_the_loop() {
  let rig = machine(20, 0, 1);
  let mut config = Config::default();
  config.bring_up.hysteresis_moves_per_half = MAX_MOVES_PER_HALF;
  let (report, entries) = hysteresis(&mut rig.aligner(config).unwrap(), Axis::M1H);
  let report = report.unwrap();
  let taken = |phase| samples(&entries).iter().filter(|sample| sample.phase == phase).count();
  assert_eq!((taken(Phase::Up), taken(Phase::Down)), (17, 16));
  let width = report.width_steps.unwrap();
  assert!((width - 20.0).abs() < 3.0, "{report}");
}

#[test]
fn seat_stick_slip_widens_the_loop_by_up_to_the_stick() {
  for seed in 0..RUNS {
    let rig = machine(20, 3, seed);
    let report = hysteresis(&mut aligner(&rig), Axis::M1V).0.unwrap();
    let width = report.width_steps.unwrap();
    assert!((19.0..=24.0).contains(&width), "seed {seed}: {report}");
  }
}

#[test]
fn both_procedures_leave_the_axis_where_they_found_it_approached_plus() {
  let rig = machine(20, 3, 1);
  let mut aligner = aligner(&rig);
  block_on(aligner.approach(Axis::M2V, 150)).unwrap();
  let before = moves(&rig).len();
  let report = backlash(&mut aligner, Axis::M2V).0.unwrap();
  assert_eq!(report.start_steps, 150);
  let loop_report = hysteresis(&mut aligner, Axis::M2V).0.unwrap();
  assert_eq!(loop_report.start_steps, 150);
  let expected = AxisState {
    pos_steps: 150,
    last_dir: LastDir::Plus,
    suspect: false,
    lost: false
  };
  assert_eq!(aligner.axes().m2_v, expected);
  assert_eq!(rig.world().commanded_steps(Axis::M2V), 150);
  assert!(
    moves(&rig).iter().skip(before).all(|(axis, _)| *axis == Axis::M2V),
    "no other axis moved"
  );
}

#[test]
fn the_noise_measured_at_the_start_is_the_sensor_s() {
  #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
  let sensor = REALISTIC_POSITION_NOISE_NORM as f32;
  let mut total = 0.0;
  for seed in 0..RUNS {
    let rig = machine(20, 0, seed);
    total += backlash(&mut aligner(&rig), Axis::M1H)
      .0
      .unwrap()
      .repeatability
      .sigma_norm;
  }
  #[expect(clippy::cast_precision_loss, reason = "a handful of runs")]
  let mean = total / RUNS as f32;
  assert!((mean - sensor).abs() < 0.15 * sensor, "mean sigma {mean}");
}

#[test]
fn a_dead_axis_has_no_direction_to_measure_along() {
  let rig = machine(0, 0, 1);
  rig.world_mut().set_axis_model(Axis::M2H, AxisModel::DEAD);
  let mut aligner = aligner(&rig);
  assert_eq!(
    backlash(&mut aligner, Axis::M2H).0,
    Err(AlignFault::WeakColumn { axis: Axis::M2H })
  );
  assert!(aligner.axes().m2_h.suspect);
  assert_eq!(
    hysteresis(&mut aligner, Axis::M2H).0,
    Err(AlignFault::WeakColumn { axis: Axis::M2H })
  );
}

#[test]
fn a_procedure_that_would_leave_the_soft_limits_is_refused_before_anything_moves() {
  let rig = machine(20, 0, 1);
  let mut config = Config::default();
  // [-10, 110] with the margin: the standard approach's overshoot to -120 is outside.
  config.motion.soft_min_steps = PerAxis::splat(0);
  config.motion.soft_max_steps = PerAxis::splat(100);
  let mut aligner = rig.aligner(config).unwrap();
  let (report, entries) = backlash(&mut aligner, Axis::M1H);
  assert!(
    matches!(report, Err(AlignFault::SoftLimit { axis: Axis::M1H, .. })),
    "{report:?}"
  );
  let (report, _) = hysteresis(&mut aligner, Axis::M1H);
  assert!(
    matches!(report, Err(AlignFault::SoftLimit { axis: Axis::M1H, .. })),
    "{report:?}"
  );
  assert!(entries.is_empty());
  assert!(rig.world().log().is_empty(), "nothing moved or measured");
}

#[test]
fn an_axis_whose_position_is_unknown_is_not_measured() {
  let rig = machine(20, 0, 1);
  let mut axes = PerAxis::splat(AxisState::default());
  axes.m1_v.lost = true;
  let mut aligner = aligner(&rig).with_axes(axes);
  assert_eq!(
    backlash(&mut aligner, Axis::M1V).0,
    Err(AlignFault::PositionUnknown { axis: Axis::M1V })
  );
  assert_eq!(
    hysteresis(&mut aligner, Axis::M1V).0,
    Err(AlignFault::PositionUnknown { axis: Axis::M1V })
  );
  assert!(rig.world().log().is_empty());
}

/// The machine's sensor as a bench with one fixed target reads it: A is the beam, and B and C swing far wider than
/// any noise, so a procedure that looked at them would take them for noise or for motion.
struct OnlyA {
  rig: Rig,
  measured: u32
}

impl BeamSensor for OnlyA {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    let shots = self.rig.measure().await?;
    self.measured += 1;
    let swing = if self.measured.is_multiple_of(2) { 0.3 } else { -0.3 };
    let elsewhere = Reading {
      h_norm: swing,
      v_norm: -swing,
      sum: shots.a.sum
    };
    Ok(Shots {
      a: shots.a,
      b: elsewhere,
      c: elsewhere
    })
  }
}

#[expect(
  clippy::unwrap_used,
  reason = "a test helper; a configuration the aligner refuses fails the test"
)]
fn bench_aligner(rig: &Rig, read_at: ReadAt) -> Aligner<Rig, OnlyA> {
  let sensor = OnlyA {
    rig: rig.clone(),
    measured: 0
  };
  Aligner::new(rig.clone(), sensor, bench_config(rig, read_at), sum_ref(rig)).unwrap()
}

/// A bench's configuration: the machine's mechanics, the beam read where `read_at` says, and seven repeats, which
/// give one place's noise to 12 degrees of freedom as three give it at three places.
fn bench_config(rig: &Rig, read_at: ReadAt) -> Config {
  Config {
    mechanics: rig.world().scenario().mechanics,
    bring_up: BringUp {
      read_at,
      repeat_shots: 7,
      ..BringUp::STARTING
    },
    ..Config::default()
  }
}

#[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
fn sum_ref(rig: &Rig) -> f32 {
  rig.world().scenario().beam.sum_ref as f32
}

#[test]
fn a_bench_that_reads_only_at_a_brackets_the_play_and_measures_the_sensor_s_noise() {
  #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
  let sensor = REALISTIC_POSITION_NOISE_NORM as f32;
  for axis in [Axis::M1H, Axis::M1V] {
    for play in [10, 20, 40, 80] {
      let mut sigma_total = 0.0;
      for seed in 0..RUNS {
        let rig = machine(play, 0, seed);
        let mut entries = Vec::new();
        let report =
          block_on(bench_aligner(&rig, ReadAt::AOnly).measure_backlash(axis, &mut |entry| entries.push(*entry)))
            .unwrap();
        let (above, up_to) = bracket(&report);
        let case = format!("{axis:?}, play {play}, seed {seed}: ({above}, {up_to}]");
        assert!(up_to > i32::try_from(play).unwrap(), "{case}");
        assert!(
          f32::from(i16::try_from(above).unwrap()) < float(play) + 2.0 * report.resolution_steps(),
          "{case}, resolution {}",
          report.resolution_steps()
        );
        sigma_total += report.repeatability.sigma_norm;
      }
      // Pooled over all six, the four coordinates never read would make the noise look sqrt(3) times smaller.
      #[expect(clippy::cast_precision_loss, reason = "a handful of runs")]
      let mean = sigma_total / RUNS as f32;
      assert!(
        (mean - sensor).abs() < 0.15 * sensor,
        "{axis:?}, play {play}: mean sigma {mean}"
      );
    }
  }
}

#[test]
fn a_bench_that_reads_only_at_a_reads_the_hysteresis_loop_without_bias() {
  // Read at one place, the loop's width carries about 2.2 steps of noise (one sigma; 200 runs at each play), so the
  // mean over 40 runs is held to a step, and every run to 8 steps (the worst of 600 runs was 6.8).
  const BENCH_RUNS: u64 = 40;
  for play in [0, 20, 40] {
    let mut total = 0.0;
    for seed in 0..BENCH_RUNS {
      let rig = machine(play, 0, seed);
      let report = block_on(bench_aligner(&rig, ReadAt::AOnly).measure_hysteresis(Axis::M1H, &mut |_| {})).unwrap();
      let error = report.width_steps.unwrap() - float(play);
      assert!(error.abs() < 8.0, "play {play}, seed {seed}: {report}");
      total += error;
    }
    #[expect(clippy::cast_precision_loss, reason = "a handful of runs")]
    let mean = total / BENCH_RUNS as f32;
    assert!(mean.abs() < 1.0, "play {play}: mean error {mean} steps");
  }
}

/// A person's readings of the noiseless machine at A: rounded to `resolution_norm`, so that a dot that has not moved
/// reads exactly the same again.
struct ReadByEye {
  rig: Rig,
  resolution_norm: f32
}

impl BeamSensor for ReadByEye {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    let shots = self.rig.measure().await?;
    let round = |norm: f32| (norm / self.resolution_norm).round() * self.resolution_norm;
    let a = Reading {
      h_norm: round(shots.a.h_norm),
      v_norm: round(shots.a.v_norm),
      sum: shots.a.sum
    };
    Ok(Shots { a, b: a, c: a })
  }
}

#[test]
fn readings_that_repeat_exactly_are_judged_with_the_noise_floor() {
  let floor_norm = 0.002;
  for play in [10, 20, 40] {
    let rig = noiseless(play);
    let mut config = bench_config(&rig, ReadAt::AOnly);
    config.bring_up.noise_floor_norm = floor_norm;
    let sensor = ReadByEye {
      rig: rig.clone(),
      resolution_norm: 0.004
    };
    let mut aligner = Aligner::new(rig.clone(), sensor, config, sum_ref(&rig)).unwrap();
    let report = block_on(aligner.measure_backlash(Axis::M1H, &mut |_| {})).unwrap();
    #[expect(clippy::float_cmp, reason = "the floor is copied, not computed")]
    {
      assert_eq!(
        report.repeatability.sigma_norm, floor_norm,
        "the repeats read exactly alike"
      );
    }
    let (above, up_to) = bracket(&report);
    let case = format!(
      "play {play}: ({above}, {up_to}], resolution {}",
      report.resolution_steps()
    );
    assert!(report.resolution_steps() > 0.0, "{case}");
    assert!(up_to > i32::try_from(play).unwrap(), "{case}");
    assert!(
      f32::from(i16::try_from(above).unwrap()) < float(play) + 2.0 * report.resolution_steps(),
      "{case}"
    );
  }
}

#[test]
fn the_places_not_read_are_what_would_otherwise_spoil_the_noise() {
  // The same sensor judged at all three places: B and C swing by 0.6 between measurements, which is what the noise
  // then comes out as. This is what the setting guards against.
  let rig = machine(20, 0, 1);
  let report = block_on(bench_aligner(&rig, ReadAt::AllThree).measure_backlash(Axis::M1H, &mut |_| {}));
  match report {
    Ok(report) => assert!(report.repeatability.sigma_norm > 0.1, "{report}"),
    Err(fault) => assert!(matches!(fault, AlignFault::WeakColumn { .. }), "{fault:?}")
  }
}
