//! The whole system on the host: the aligner driving the coordinator's orchestrator, which drives two mirror nodes
//! and the sensor node over a simulated bus, the relay, the motors, and the Ruida simulated around them.
//!
//! The closed-loop tests (`closed_loop.rs`) hold the alignment logic to the machine; these hold the devices to each
//! other. A frame the protocol and a device disagree on, a move the interlock never allows, or a reading that never
//! reaches the aligner shows here and nowhere else on the host.

use align_core::aligner::AlignReport;
use align_core::axis::{Axis, PerAxis};
use align_core::config::{Config, DirSign};
use align_core::coordinator::State;
use align_core::rig::MoveFault;
use align_proto::{Class, NodeId};
use align_sim::block_on;
use align_sim::system::{Hardware, System, SystemRig};
use align_sim::world::{REALISTIC_POSITION_NOISE_NORM, Scenario};
use coordinator_logic::nodes::Change;
use coordinator_logic::orchestrator::Report;
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

/// The starting configuration with the sensor characterized, as in the closed-loop tests.
fn characterized() -> Config {
  let mut config = Config::default();
  #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
  let noise = REALISTIC_POSITION_NOISE_NORM as f32;
  config.sensor.noise_sigma_norm = noise;
  config
}

fn rig(scenario: Scenario) -> SystemRig {
  SystemRig::new(System::new(scenario, characterized(), Hardware::PLACEHOLDER))
}

/// The noise-free residual of the three-position difference vector.
fn true_residual(rig: &SystemRig) -> f32 {
  rig.system().world().true_shots().measurement().amax()
}

#[expect(
  clippy::unwrap_used,
  reason = "a test helper; a configuration the aligner refuses fails the test"
)]
fn align(rig: &SystemRig) -> AlignReport {
  let mut aligner = rig.aligner(characterized()).unwrap();
  block_on(aligner.align()).unwrap()
}

#[test]
fn the_system_settles_into_idle_with_every_node_present_and_its_motors_configured() {
  let rig = rig(Scenario::IDEAL);
  let system = rig.system();
  assert_eq!(system.state(), State::Idle);
  let reports: Vec<Report> = system.reports().iter().map(|(_, report)| *report).collect();
  for node in [NodeId::MIRROR_1, NodeId::MIRROR_2, NodeId::SENSOR] {
    assert!(
      reports.contains(&Report::Node {
        node,
        change: Change::Appeared
      }),
      "{node:?}"
    );
  }
  for node in [NodeId::MIRROR_1, NodeId::MIRROR_2] {
    assert!(reports.contains(&Report::Configured { node }), "{node:?}");
  }
  assert!(system.now_ms() < 1_000, "settled in {} ms", system.now_ms());
  assert_eq!(system.stats().rejected, 0);
}

#[test]
fn a_move_through_the_system_turns_the_screw_by_the_steps_asked() {
  let rig = rig(Scenario::IDEAL);
  let mut system = rig.system_mut();
  assert_eq!(system.move_rel(Axis::M1H, 100), Ok(100));
  assert_eq!(system.move_rel(Axis::M2V, -40), Ok(-40));
  assert_eq!(system.move_rel(Axis::M1H, -30), Ok(70));
  assert_eq!(system.world().commanded_steps(Axis::M1H), 70);
  assert_eq!(system.world().commanded_steps(Axis::M2V), -40);
  assert_eq!(system.world().commanded_steps(Axis::M1V), 0);
  assert_eq!(system.stats().rejected, 0);
}

#[test]
fn a_screw_wired_the_other_way_turns_the_right_way_through_its_sign() {
  let mut scenario = Scenario::IDEAL;
  scenario.mechanics.dir_sign.m2_h = DirSign::Reversed;
  let rig = rig(scenario);
  let mut system = rig.system_mut();
  assert_eq!(system.move_rel(Axis::M2H, 60), Ok(60));
  assert_eq!(system.world().commanded_steps(Axis::M2H), 60);
}

#[test]
fn a_move_beyond_the_soft_limit_is_refused_by_the_node_and_moves_nothing() {
  let rig = rig(Scenario::IDEAL);
  let mut system = rig.system_mut();
  assert_eq!(system.move_rel(Axis::M1V, 4_000), Ok(4_000));
  assert_eq!(system.move_rel(Axis::M1V, 1_000), Err(MoveFault::SoftLimit));
  assert_eq!(system.world().commanded_steps(Axis::M1V), 4_000);
  // The relay releases within its release time, and the coordinator is Idle again.
  assert!(system.run_until(50, |system| system.state() == State::Idle));
}

#[test]
fn a_measurement_through_the_system_reads_what_the_machine_shows() {
  let scenario = Scenario {
    misalignment_steps: PerAxis {
      m1_h: 20.0,
      m1_v: -10.0,
      m2_h: 15.0,
      m2_v: 5.0
    },
    ..Scenario::IDEAL
  };
  let rig = rig(scenario);
  let shots = rig.system_mut().measure().unwrap();
  let truth = rig.system().world().true_shots();
  for (read, true_reading) in [(shots.a, truth.a), (shots.b, truth.b), (shots.c, truth.c)] {
    // Q15 positions and a Q15 sum: within one count.
    assert!(
      (read.h_norm - true_reading.h_norm).abs() < 1e-4,
      "{read:?} {true_reading:?}"
    );
    assert!(
      (read.v_norm - true_reading.v_norm).abs() < 1e-4,
      "{read:?} {true_reading:?}"
    );
    assert!(
      (read.sum - true_reading.sum / 1_000.0).abs() < 1e-4,
      "{read:?} {true_reading:?}"
    );
  }
  assert_eq!(rig.system().state(), State::Idle);
}

#[test]
fn the_aligner_aligns_an_ideal_machine_through_the_whole_system() {
  let scenario = Scenario {
    misalignment_steps: PerAxis {
      m1_h: 30.0,
      m1_v: -25.0,
      m2_h: -40.0,
      m2_v: 35.0
    },
    ..Scenario::IDEAL
  };
  let rig = rig(scenario);
  assert!(true_residual(&rig) > 0.05);
  let report = align(&rig);
  assert!(report.iterations <= 4, "{report:?}");
  assert_eq!(report.recalibrations, 0);
  assert!(true_residual(&rig) <= 0.02, "true residual {}", true_residual(&rig));
  let system = rig.system();
  assert_eq!(system.stats().rejected, 0);
  assert_eq!(system.state(), State::Idle);
}

#[test]
fn the_aligner_aligns_realistic_machines_through_the_whole_system() {
  for seed in 0..4_u64 {
    let mut draw = StdRng::seed_from_u64(seed);
    let scenario = Scenario {
      misalignment_steps: PerAxis {
        m1_h: draw.random_range(-35.0..35.0),
        m1_v: draw.random_range(-35.0..35.0),
        m2_h: draw.random_range(-50.0..50.0),
        m2_v: draw.random_range(-50.0..50.0)
      },
      head_offset_um: [draw.random_range(-200.0..200.0), draw.random_range(-200.0..200.0)],
      ..Scenario::realistic(seed)
    };
    let rig = rig(scenario);
    let report = align(&rig);
    assert!(
      true_residual(&rig) <= 0.03,
      "seed {seed}: true residual {}, {report:?}",
      true_residual(&rig)
    );
    assert_eq!(rig.system().stats().rejected, 0, "seed {seed}");
  }
}

#[test]
fn a_lost_move_done_is_answered_again_and_the_move_runs_once() {
  let rig = rig(Scenario::IDEAL);
  let mut system = rig.system_mut();
  system.drop_next(Class::MoveDone);
  // The coordinator sends the MOVE_REL again once the answer is late; the node answers from its record.
  assert_eq!(system.move_rel(Axis::M1H, 200), Ok(200));
  assert_eq!(system.stats().dropped, 1);
  assert_eq!(system.world().commanded_steps(Axis::M1H), 200, "run once, not twice");
  assert_eq!(system.move_rel(Axis::M1H, -50), Ok(150));
  assert_eq!(system.world().commanded_steps(Axis::M1H), 150);
}

#[test]
fn a_lost_reading_is_asked_for_again_and_the_measurement_completes() {
  let rig = rig(Scenario::IDEAL);
  let mut system = rig.system_mut();
  system.drop_next(Class::Sensor);
  // Without SENSOR_RESEND and the sensor's kept results, the measurement would time out with no reading.
  assert!(system.measure().is_ok());
  assert_eq!(system.stats().dropped, 1);
  let ended = system
    .reports()
    .iter()
    .filter(|(_, report)| matches!(report, Report::MeasurementEnded { .. }))
    .count();
  assert_eq!(ended, 1);
}
