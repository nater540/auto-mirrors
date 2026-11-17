//! The orchestrator on a simulated bench: a relay whose feedback follows its drive, one motor node, the sensor node,
//! and the frames they would send. Every rule of a move and of a measurement, the refusals, the faults on the way,
//! and a property over random sequences: a MOVE_REL goes out only after INHIBIT 0, with the relay energized and its
//! loop reading open, and FootSW never starts a job while the relay is energized.

extern crate std;

use std::vec;
use std::vec::Vec;

use align_core::approach::SoftLimits;
use align_core::coordinator::FaultReason;
use align_core::quadrant::ReadingFault;
use align_proto::{
  ConfigKey, MoveDone, MoveRel, MoveResult, PROTOCOL_VERSION, SensorReading, SensorResult, SensorStatus, Shot,
  ShotPoint
};

use crate::readings::to_reading;
use proptest::prelude::*;

use super::*;

const CONFIG: Config = Config::STARTING;
const EXPECTED: &[NodeId] = &[NodeId::MIRROR_1];
const MIRROR_1: NodeId = NodeId::MIRROR_1;

fn setup() -> Setup {
  Setup {
    config: CONFIG,
    fw_version: 1,
    expected: EXPECTED,
    wiring: WIRING
  }
}

/// What the orchestrator did, in order.
#[derive(Clone, Debug, PartialEq)]
enum Io {
  Sent(Envelope),
  Relay(bool),
  FootSw(bool),
  Report(Report)
}

/// The simulated bench: records everything, and keeps the relay level for the feedback to follow.
#[derive(Default)]
struct Bench {
  events: Vec<Io>,
  relay: bool,
  footsw: bool
}

impl CoordinatorIo for Bench {
  fn send(&mut self, envelope: Envelope) {
    self.events.push(Io::Sent(envelope));
  }

  fn set_relay(&mut self, energized: bool) {
    self.relay = energized;
    self.events.push(Io::Relay(energized));
  }

  fn set_footsw(&mut self, active: bool) {
    self.footsw = active;
    self.events.push(Io::FootSw(active));
  }

  fn report(&mut self, report: Report) {
    self.events.push(Io::Report(report));
  }
}

impl Bench {
  fn take(&mut self) -> Vec<Io> {
    core::mem::take(&mut self.events)
  }

  fn sent(&self) -> Vec<Message> {
    let sent = self.events.iter().filter_map(|event| match event {
      Io::Sent(envelope) => Some(envelope.message),
      _ => None
    });
    sent.collect()
  }

  fn reports(&self) -> Vec<Report> {
    let reports = self.events.iter().filter_map(|event| match event {
      Io::Report(report) => Some(*report),
      _ => None
    });
    reports.collect()
  }

  fn outcomes(&self) -> Vec<Outcome> {
    let outcomes = self.reports().into_iter().filter_map(|report| match report {
      Report::Outcome(outcome) => Some(outcome),
      _ => None
    });
    outcomes.collect()
  }

  fn move_rels(&self) -> Vec<(NodeId, MoveRel)> {
    let moves = self.events.iter().filter_map(|event| match event {
      Io::Sent(Envelope {
        node,
        message: Message::MoveRel(move_rel)
      }) => Some((*node, *move_rel)),
      _ => None
    });
    moves.collect()
  }

  fn inhibits(&self) -> Vec<bool> {
    let inhibits = self.sent().into_iter().filter_map(|message| match message {
      Message::Inhibit { inhibited, .. } => Some(inhibited),
      _ => None
    });
    inhibits.collect()
  }
}

/// Inputs with nothing firing and no job running, and the loop as the relay leaves it.
fn follow(bench: &Bench) -> Inputs {
  Inputs {
    loop_open: bench.relay,
    job_running: false,
    firing: false
  }
}

fn beat(now_ms: u64) -> Envelope {
  let uptime_ms = u32::try_from(now_ms).unwrap();
  Envelope {
    node: MIRROR_1,
    message: Message::Heartbeat {
      uptime_ms,
      state: 1,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION
    }
  }
}

fn move_done(seq: u8, result: MoveResult, pos_steps: i32) -> Envelope {
  Envelope {
    node: MIRROR_1,
    message: Message::MoveDone(MoveDone {
      seq,
      channel: Channel::ZERO,
      result,
      pos_steps
    })
  }
}

/// Ticks from `from_ms` to `to_ms` one millisecond at a time, the loop following the relay, the node's heartbeat
/// arriving every 100 ms.
fn run(orchestrator: &mut Orchestrator, bench: &mut Bench, from_ms: u64, to_ms: u64) {
  for now_ms in from_ms..=to_ms {
    if now_ms % 100 == 0 {
      orchestrator.on_frame(now_ms, beat(now_ms), bench);
    }
    let inputs = follow(bench);
    orchestrator.tick(now_ms, inputs, bench);
  }
}

/// An orchestrator in Idle at 1000 ms, its node present and configured, everything it did so far cleared.
fn idle() -> (Orchestrator, Bench) {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  run(&mut orchestrator, &mut bench, 0, 1_000);
  assert_eq!(orchestrator.state(), State::Idle);
  bench.take();
  (orchestrator, bench)
}

// --- the self-test and the node's configuration ---------------------------------------------------------------

#[test]
fn it_boots_inhibited_and_waits_for_the_expected_node() {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  for now_ms in 0..500 {
    let inputs = follow(&bench);
    orchestrator.tick(now_ms, inputs, &mut bench);
  }
  assert_eq!(orchestrator.state(), State::Inhibited);
  assert!(!bench.relay);
  assert!(bench.inhibits().iter().all(|inhibited| *inhibited));
}

#[test]
fn the_self_test_passes_once_the_expected_node_is_present_and_the_inputs_are_read() {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  orchestrator.on_frame(0, beat(0), &mut bench);
  assert_eq!(orchestrator.state(), State::Inhibited);
  orchestrator.tick(1, follow(&bench), &mut bench);
  assert_eq!(orchestrator.state(), State::Idle);
  assert!(bench.outcomes().contains(&Outcome::Ready));
  assert!(bench.reports().contains(&Report::State(State::Idle)));
}

#[test]
fn a_node_that_appears_gets_the_soft_limits_and_sign_of_each_of_its_axes() {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  orchestrator.on_frame(0, beat(0), &mut bench);
  let mut expected = Vec::new();
  for axis in [Axis::M1H, Axis::M1V] {
    let wire = WIRING.get(axis);
    let limits = SoftLimits::for_axis(&CONFIG.motion, axis);
    let sign = CONFIG.mechanics.dir_sign.get(axis).factor();
    for (key, value) in [
      (ConfigKey::SoftMinSteps, limits.min_steps),
      (ConfigKey::SoftMaxSteps, limits.max_steps),
      (ConfigKey::DirSign, sign)
    ] {
      expected.push(Io::Sent(Envelope {
        node: MIRROR_1,
        message: Message::Config {
          key,
          channel: wire.channel,
          value
        }
      }));
    }
  }
  let configs: Vec<Io> = bench
    .take()
    .into_iter()
    .filter(|event| {
      matches!(
        event,
        Io::Sent(Envelope {
          message: Message::Config { .. },
          ..
        })
      )
    })
    .collect();
  assert_eq!(configs, expected);
}

#[test]
fn a_node_that_restarted_is_configured_again() {
  let (mut orchestrator, mut bench) = idle();
  // An uptime that jumped back: the node rebooted between two heartbeats.
  let mut restarted = beat(1_100);
  if let Message::Heartbeat { uptime_ms, .. } = &mut restarted.message {
    *uptime_ms = 50;
  }
  orchestrator.on_frame(1_100, restarted, &mut bench);
  let configs = bench
    .sent()
    .into_iter()
    .filter(|message| matches!(message, Message::Config { .. }))
    .count();
  assert_eq!(configs, 6);
  assert!(bench.reports().contains(&Report::Node {
    node: MIRROR_1,
    change: Change::Restarted
  }));
}

#[test]
fn a_loop_that_reads_open_while_the_relay_is_released_faults_and_sends_an_estop() {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  orchestrator.on_frame(0, beat(0), &mut bench);
  let welded = Inputs {
    loop_open: true,
    job_running: false,
    firing: false
  };
  orchestrator.tick(1, welded, &mut bench);
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::RelayDidNotClose));
  assert!(bench.sent().contains(&Message::Estop { reason: 0 }));
}

// --- a move ---------------------------------------------------------------------------------------------------

#[test]
fn a_move_arms_the_relay_lifts_the_inhibit_sends_the_move_and_disarms_when_it_is_done() {
  let (mut orchestrator, mut bench) = idle();
  assert_eq!(orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench), Ok(()));
  assert_eq!(
    bench.take(),
    [
      Io::Relay(true),
      Io::Report(Report::State(State::ArmingMove {
        energized_ms: Some(1_001)
      }))
    ]
  );
  // The next tick reads the loop open: motion is allowed, then the move goes out.
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  assert!(orchestrator.motion_allowed());
  let events = bench.take();
  let inhibit_0 = events.iter().position(|event| {
    matches!(
      event,
      Io::Sent(Envelope {
        message: Message::Inhibit { inhibited: false, .. },
        ..
      })
    )
  });
  let move_rel = events.iter().position(|event| {
    matches!(
      event,
      Io::Sent(Envelope {
        message: Message::MoveRel(_),
        ..
      })
    )
  });
  assert!(inhibit_0 < move_rel, "INHIBIT 0 before the MOVE_REL: {events:?}");
  assert!(events.contains(&Io::Report(Report::MoveSent {
    axis: Axis::M1H,
    steps: 100
  })));
  let sent = Bench {
    events,
    relay: true,
    footsw: false
  };
  assert_eq!(
    sent.move_rels(),
    [(
      MIRROR_1,
      MoveRel {
        seq: 0,
        channel: Channel::ZERO,
        delta_steps: 100,
        finish_plus: false
      }
    )]
  );
  // The node answers half a second later, its heartbeat arriving meanwhile: the relay is released and the inhibit
  // goes back up.
  run(&mut orchestrator, &mut bench, 1_003, 1_599);
  assert!(bench.relay);
  orchestrator.on_frame(1_600, move_done(0, MoveResult::Done, 100), &mut bench);
  assert!(!bench.relay);
  assert!(bench.reports().contains(&Report::MoveEnded {
    axis: Axis::M1H,
    result: Ok(100)
  }));
  orchestrator.tick(1_601, follow(&bench), &mut bench);
  assert_eq!(orchestrator.state(), State::Idle);
  assert!(bench.outcomes().contains(&Outcome::MoveFinished));
  assert_eq!(bench.inhibits().last(), Some(&true));
}

#[test]
fn a_move_the_node_refuses_at_its_soft_limit_disarms_and_faults_until_cleared() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  orchestrator.on_frame(1_010, move_done(0, MoveResult::SoftLimit, 27_508), &mut bench);
  assert!(bench.reports().contains(&Report::MoveEnded {
    axis: Axis::M1H,
    result: Err(MoveFault::SoftLimit)
  }));
  assert!(!bench.relay);
  // Reference 7.3: a soft limit is a fault, cleared by hand.
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::Procedure));
  run(&mut orchestrator, &mut bench, 1_011, 1_200);
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::Procedure));
  orchestrator.clear_fault(1_201, &mut bench);
  run(&mut orchestrator, &mut bench, 1_202, 1_210);
  assert_eq!(orchestrator.state(), State::Idle);
}

#[test]
fn a_rail_lost_under_a_move_disarms_and_faults() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  orchestrator.on_frame(1_100, move_done(0, MoveResult::RailLost, 17), &mut bench);
  assert!(!bench.relay);
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::Procedure));
}

#[test]
fn a_motor_node_s_hardware_fault_faults_the_coordinator_and_estop_repeats_until_cleared() {
  for code in [
    align_proto::FaultCode::PIN_FAULT,
    align_proto::FaultCode::RAIL_LOST_DURING_MOVE,
    align_proto::FaultCode::MOVE_TIMEOUT
  ] {
    let (mut orchestrator, mut bench) = idle();
    orchestrator.request_move(1_001, Axis::M1H, 1_000, &mut bench).unwrap();
    orchestrator.tick(1_002, follow(&bench), &mut bench);
    let fault = Envelope {
      node: MIRROR_1,
      message: Message::Fault {
        channel: Channel::ZERO,
        code,
        detail: 0
      }
    };
    orchestrator.on_frame(1_003, fault, &mut bench);
    assert_eq!(orchestrator.state(), State::Fault(FaultReason::Procedure), "{code:?}");
    assert!(!bench.relay);
    bench.take();
    run(&mut orchestrator, &mut bench, 1_004, 1_300);
    let estops = bench
      .sent()
      .iter()
      .filter(|message| matches!(message, Message::Estop { .. }))
      .count();
    assert_eq!(estops, 3, "one with each heartbeat: {code:?}");
  }
}

#[test]
fn a_bus_off_faults_the_coordinator() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 1_000, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  orchestrator.bus_off(1_003, &mut bench);
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::Procedure));
  assert!(!bench.relay);
  assert!(!orchestrator.motion_allowed());
}

#[test]
fn a_node_that_refuses_a_move_as_not_configured_is_configured_again() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  bench.take();
  let refused = Envelope {
    node: MIRROR_1,
    message: Message::Fault {
      channel: Channel::ZERO,
      code: align_proto::FaultCode::NOT_CONFIGURED,
      detail: 0
    }
  };
  orchestrator.on_frame(1_003, refused, &mut bench);
  assert!(bench.reports().contains(&Report::MoveEnded {
    axis: Axis::M1H,
    result: Err(MoveFault::Refused {
      code: align_proto::FaultCode::NOT_CONFIGURED.0
    })
  }));
  let configs = bench
    .sent()
    .iter()
    .filter(|message| matches!(message, Message::Config { .. }))
    .count();
  assert_eq!(configs, 6);
}

#[test]
fn a_move_with_no_answer_ends_as_unanswered_and_disarms() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  run(&mut orchestrator, &mut bench, 1_002, 20_000);
  let tries = bench.move_rels().len();
  assert_eq!(tries, usize::from(CONFIG.coordinator.move_tries));
  assert!(bench.reports().contains(&Report::MoveEnded {
    axis: Axis::M1H,
    result: Err(MoveFault::NoAnswer)
  }));
  assert_eq!(orchestrator.state(), State::Idle);
  assert!(!bench.relay);
}

#[test]
fn stop_sends_stop_to_the_node_of_the_move_in_flight() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 1_000, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  bench.take();
  orchestrator.stop(&mut bench);
  assert_eq!(
    bench.take(),
    [Io::Sent(Envelope {
      node: MIRROR_1,
      message: Message::Stop { reason: 0 }
    })]
  );
  orchestrator.on_frame(1_500, move_done(0, MoveResult::Stopped, 100), &mut bench);
  assert!(bench.reports().contains(&Report::MoveEnded {
    axis: Axis::M1H,
    result: Err(MoveFault::Stopped { pos_steps: 100 })
  }));
  assert!(!bench.relay);
}

#[test]
fn a_stop_while_arming_sends_no_move_and_disarms() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.stop(&mut bench);
  run(&mut orchestrator, &mut bench, 1_002, 1_010);
  assert!(bench.move_rels().is_empty());
  assert!(bench.reports().contains(&Report::MoveDropped { axis: Axis::M1H }));
  assert_eq!(orchestrator.state(), State::Idle);
  assert!(!bench.relay);
}

// --- refusals -------------------------------------------------------------------------------------------------

#[test]
fn a_move_is_refused_outside_idle() {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  assert_eq!(
    orchestrator.request_move(0, Axis::M1H, 100, &mut bench),
    Err(RequestError::NotIdle {
      state: State::Inhibited
    })
  );
  assert!(!bench.relay);
}

#[test]
fn a_second_move_is_refused_while_one_is_asked_for_or_running() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  assert_eq!(
    orchestrator.request_move(1_001, Axis::M1V, 100, &mut bench),
    Err(RequestError::Busy)
  );
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  assert_eq!(
    orchestrator.request_move(1_002, Axis::M1H, 100, &mut bench),
    Err(RequestError::Busy)
  );
  assert_eq!(bench.move_rels().len(), 1);
}

#[test]
fn a_move_on_an_axis_whose_node_is_absent_is_refused_and_nothing_is_armed() {
  let (mut orchestrator, mut bench) = idle();
  assert_eq!(
    orchestrator.request_move(1_001, Axis::M2H, 100, &mut bench),
    Err(RequestError::NodeAbsent {
      node: NodeId::MIRROR_2.get()
    })
  );
  assert!(bench.take().is_empty());
  assert_eq!(orchestrator.state(), State::Idle);
}

#[test]
fn a_move_is_refused_while_a_job_runs() {
  let (mut orchestrator, mut bench) = idle();
  let job = Inputs {
    loop_open: false,
    job_running: true,
    firing: false
  };
  orchestrator.tick(1_001, job, &mut bench);
  assert_eq!(
    orchestrator.request_move(1_002, Axis::M1H, 100, &mut bench),
    Err(RequestError::Refused)
  );
  assert!(!bench.relay);
}

// --- faults during a move -------------------------------------------------------------------------------------

#[test]
fn a_relay_that_does_not_confirm_faults_and_drops_the_move_unsent() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  let stuck = Inputs {
    loop_open: false,
    job_running: false,
    firing: false
  };
  let timeout_ms = u64::from(CONFIG.coordinator.relay_feedback_timeout_ms);
  for now_ms in 1_002..=1_002 + timeout_ms {
    orchestrator.tick(now_ms, stuck, &mut bench);
  }
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::RelayDidNotOpen));
  assert!(bench.move_rels().is_empty());
  assert!(bench.reports().contains(&Report::MoveDropped { axis: Axis::M1H }));
  assert!(bench.sent().contains(&Message::Estop { reason: 0 }));
  assert!(!bench.relay);
}

#[test]
fn a_fire_while_moving_faults_at_once() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 1_000, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  let fire = Inputs {
    loop_open: true,
    job_running: false,
    firing: true
  };
  orchestrator.tick(1_003, fire, &mut bench);
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::FireWhileArmed));
  assert!(!bench.relay);
  assert!(bench.sent().contains(&Message::Estop { reason: 0 }));
  assert!(!orchestrator.motion_allowed());
}

#[test]
fn a_node_lost_while_moving_inhibits_and_releases_the_relay() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 1_000, &mut bench).unwrap();
  // No heartbeat after the one at 1000 ms: lost after the timeout.
  for now_ms in 1_002..1_400 {
    let inputs = follow(&bench);
    orchestrator.tick(now_ms, inputs, &mut bench);
  }
  assert_eq!(orchestrator.state(), State::Inhibited);
  assert!(!bench.relay);
  assert_eq!(bench.inhibits().last(), Some(&true));
}

#[test]
fn a_fault_is_cleared_by_hand_and_the_self_test_runs_again() {
  let (mut orchestrator, mut bench) = idle();
  let fire = Inputs {
    loop_open: false,
    job_running: false,
    firing: true
  };
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_002, fire, &mut bench);
  assert!(matches!(orchestrator.state(), State::Fault(_)));
  orchestrator.clear_fault(1_003, &mut bench);
  assert_eq!(orchestrator.state(), State::Inhibited);
  run(&mut orchestrator, &mut bench, 1_004, 2_000);
  assert_eq!(orchestrator.state(), State::Idle);
}

// --- the heartbeat and the fire sense -------------------------------------------------------------------------

#[test]
fn the_heartbeat_carries_the_state_code() {
  let (mut orchestrator, mut bench) = idle();
  run(&mut orchestrator, &mut bench, 1_001, 1_100);
  let states: Vec<u8> = bench
    .sent()
    .into_iter()
    .filter_map(|message| match message {
      Message::Heartbeat { state, .. } => Some(state),
      _ => None
    })
    .collect();
  assert_eq!(states, [state_code(State::Idle)]);
  assert_eq!(state_code(State::Inhibited), 0);
  assert_eq!(state_code(State::Moving { since_ms: 0 }), 6);
  assert_eq!(state_code(State::Fault(FaultReason::NoFire)), 8);
}

#[test]
fn fire_edges_closer_than_the_burst_gap_are_one_burst() {
  let gap = u64::from(CONFIG.coordinator.burst_gap_ms);
  let mut burst = Burst::new(CONFIG.coordinator.burst_gap_ms);
  assert_eq!(burst.sample(0, false), None);
  assert_eq!(burst.sample(10, true), Some(Event::Fire));
  assert_eq!(burst.sample(11, false), None);
  let again = 10 + gap - 1;
  assert_eq!(burst.sample(again, false), None, "not yet a whole gap");
  assert_eq!(
    burst.sample(again, true),
    None,
    "an edge inside the gap: the same burst"
  );
  assert_eq!(burst.sample(again + gap - 1, false), None);
  assert_eq!(burst.sample(again + gap, false), Some(Event::FireEnded));
  assert_eq!(burst.sample(again + gap + 1, false), None);
  assert_eq!(burst.sample(again + gap + 2, true), Some(Event::Fire));
}

#[test]
fn a_burst_lasts_while_the_line_stays_active() {
  let gap = u64::from(CONFIG.coordinator.burst_gap_ms);
  let mut burst = Burst::new(CONFIG.coordinator.burst_gap_ms);
  assert_eq!(burst.sample(0, true), Some(Event::Fire));
  for now_ms in 1..=10 * gap {
    assert_eq!(burst.sample(now_ms, true), None);
  }
  assert_eq!(burst.sample(10 * gap + 1, false), None);
  assert_eq!(burst.sample(11 * gap, false), Some(Event::FireEnded));
}

// --- a measurement --------------------------------------------------------------------------------------------

const SENSOR: NodeId = NodeId::SENSOR;

fn sensor_beat(now_ms: u64) -> Envelope {
  Envelope {
    node: SENSOR,
    ..beat(now_ms)
  }
}

/// The sensor's answer to shot `seq`: a valid reading whose position tells the points apart.
fn answer(seq: u8, result: SensorResult) -> Envelope {
  let offset = i16::from(seq) * 1_000;
  Envelope {
    node: SENSOR,
    message: Message::Sensor(SensorReading {
      seq,
      status: SensorStatus {
        result,
        calibrated: true,
        bench: false
      },
      h_q15: 1_000 + offset,
      v_q15: -2_000 - offset,
      sum_q: 50_000
    })
  }
}

/// The SHOT frames sent so far, in the coordinator's name as the protocol has them: sequence number and point.
fn shots_sent(bench: &Bench) -> Vec<(u8, ShotPoint)> {
  let shots = bench.events.iter().filter_map(|event| match event {
    Io::Sent(Envelope {
      node,
      message: Message::Shot(Shot { seq, point, .. })
    }) if *node == NodeId::COORDINATOR => Some((*seq, *point)),
    _ => None
  });
  shots.collect()
}

fn measurements(bench: &Bench) -> Vec<Result<Shots, SensorFault>> {
  let ended = bench.reports().into_iter().filter_map(|report| match report {
    Report::MeasurementEnded { result } => Some(result),
    _ => None
  });
  ended.collect()
}

/// Ticks one millisecond at a time with both nodes' heartbeats arriving, the loop following the relay, and the job
/// as `job` says: whether the Status line is up and whether the laser fires at each time.
fn play(
  orchestrator: &mut Orchestrator,
  bench: &mut Bench,
  from_ms: u64,
  to_ms: u64,
  job: impl Fn(u64) -> (bool, bool)
) {
  for now_ms in from_ms..=to_ms {
    if now_ms % 100 == 0 {
      orchestrator.on_frame(now_ms, beat(now_ms), bench);
    }
    if now_ms % 100 == 50 {
      orchestrator.on_frame(now_ms, sensor_beat(now_ms), bench);
    }
    let (job_running, firing) = job(now_ms);
    let inputs = Inputs {
      loop_open: bench.relay,
      job_running,
      firing
    };
    orchestrator.tick(now_ms, inputs, bench);
  }
}

/// The canned job started at 1000 ms: Status up from 1300 ms to 2000 ms, and `dots` dots of 10 ms, 200 ms apart,
/// from 1400 ms.
fn canned(dots: u64) -> impl Fn(u64) -> (bool, bool) {
  move |now_ms| {
    let running = (1_300..2_000).contains(&now_ms);
    let firing = (0..dots).any(|dot| (1_400 + 200 * dot..1_410 + 200 * dot).contains(&now_ms));
    (running, firing)
  }
}

/// No job.
fn quiet(_: u64) -> (bool, bool) {
  (false, false)
}

/// Idle at 1000 ms with the mirror 1 node and the sensor node present, everything so far cleared.
fn idle_with_sensor() -> (Orchestrator, Bench) {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  play(&mut orchestrator, &mut bench, 0, 999, quiet);
  assert_eq!(orchestrator.state(), State::Idle);
  bench.take();
  (orchestrator, bench)
}

/// The three shots' readings as the aligner gets them.
fn expected_shots(seqs: [u8; 3]) -> Shots {
  let reading = |seq| match answer(seq, SensorResult::Valid).message {
    Message::Sensor(reading) => to_reading(&reading).unwrap(),
    _ => unreachable!()
  };
  Shots {
    a: reading(seqs[0]),
    b: reading(seqs[1]),
    c: reading(seqs[2])
  }
}

#[test]
fn a_measurement_pulses_footsw_for_the_start_pulse_only() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  assert_eq!(orchestrator.request_measurement(1_000, &mut bench), Ok(()));
  assert!(bench.footsw);
  let pulse_ms = u64::from(CONFIG.coordinator.start_pulse_ms);
  play(&mut orchestrator, &mut bench, 1_000, 1_000 + pulse_ms - 1, quiet);
  assert!(bench.footsw, "still low on the Ruida's side");
  play(&mut orchestrator, &mut bench, 1_000 + pulse_ms, 1_000 + pulse_ms, quiet);
  assert!(!bench.footsw);
  let pulses: Vec<bool> = bench
    .take()
    .into_iter()
    .filter_map(|event| match event {
      Io::FootSw(active) => Some(active),
      _ => None
    })
    .collect();
  assert_eq!(pulses, [true, false]);
  assert!(matches!(orchestrator.state(), State::WaitingForFire { .. }));
}

#[test]
fn each_burst_is_a_shot_sent_to_the_sensor_with_its_point() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 1_999, canned(3));
  assert_eq!(
    shots_sent(&bench),
    [(0, ShotPoint::A), (1, ShotPoint::B), (2, ShotPoint::C)]
  );
}

#[test]
fn readings_that_arrive_during_the_job_complete_it_when_the_status_line_falls() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 1_900, canned(3));
  for (seq, _) in shots_sent(&bench) {
    orchestrator.on_frame(1_900, answer(seq, SensorResult::Valid), &mut bench);
  }
  assert!(measurements(&bench).is_empty(), "not before the job ends");
  play(&mut orchestrator, &mut bench, 1_901, 2_000, canned(3));
  assert_eq!(measurements(&bench), [Ok(expected_shots([0, 1, 2]))]);
  assert_eq!(orchestrator.state(), State::Idle);
}

#[test]
fn readings_that_arrive_after_the_job_complete_it_on_the_last() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 2_100, canned(3));
  assert!(matches!(orchestrator.state(), State::CollectingReadings { .. }));
  // Out of order: the readings are placed by sequence number.
  for seq in [2, 0, 1] {
    orchestrator.on_frame(2_100, answer(seq, SensorResult::Valid), &mut bench);
  }
  assert_eq!(measurements(&bench), [Ok(expected_shots([0, 1, 2]))]);
  assert_eq!(orchestrator.state(), State::Idle);
}

#[test]
fn a_new_measurement_forgets_the_readings_of_the_last() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 2_100, canned(3));
  for seq in 0..3 {
    orchestrator.on_frame(2_100, answer(seq, SensorResult::Valid), &mut bench);
  }
  assert_eq!(measurements(&bench).len(), 1);
  play(&mut orchestrator, &mut bench, 2_101, 2_999, quiet);
  bench.take();
  // The same job 2 s later. The sensor answers A and B before C fires: the last job's C must not complete it.
  orchestrator.request_measurement(3_000, &mut bench).unwrap();
  let job = canned(3);
  let later = move |now_ms: u64| job(now_ms - 2_000);
  play(&mut orchestrator, &mut bench, 3_000, 3_700, &later);
  for (seq, _) in shots_sent(&bench) {
    orchestrator.on_frame(3_700, answer(seq, SensorResult::Valid), &mut bench);
  }
  play(&mut orchestrator, &mut bench, 3_701, 4_100, &later);
  assert!(measurements(&bench).is_empty(), "not complete without C's reading");
  orchestrator.on_frame(4_100, answer(5, SensorResult::Valid), &mut bench);
  assert_eq!(measurements(&bench), [Ok(expected_shots([3, 4, 5]))]);
}

#[test]
fn a_late_reading_is_asked_for_again_while_the_measurement_runs() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 2_100, canned(3));
  for seq in [0, 2] {
    orchestrator.on_frame(2_100, answer(seq, SensorResult::Valid), &mut bench);
  }
  bench.take();
  // Shot 1 went out at 1600 ms: asked for again 1000 ms later, then every 1000 ms.
  play(&mut orchestrator, &mut bench, 2_101, 3_700, canned(3));
  let resends: Vec<u8> = bench
    .sent()
    .into_iter()
    .filter_map(|message| match message {
      Message::SensorResend { seq } => Some(seq),
      _ => None
    })
    .collect();
  assert_eq!(resends, [1, 1]);
  orchestrator.on_frame(3_700, answer(1, SensorResult::Valid), &mut bench);
  assert_eq!(measurements(&bench), [Ok(expected_shots([0, 1, 2]))]);
}

#[test]
fn a_reading_the_sensor_rejected_ends_the_measurement_as_rejected() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 2_100, canned(3));
  orchestrator.on_frame(2_100, answer(0, SensorResult::Valid), &mut bench);
  orchestrator.on_frame(2_100, answer(1, SensorResult::Saturated), &mut bench);
  orchestrator.on_frame(2_100, answer(2, SensorResult::Valid), &mut bench);
  assert_eq!(
    measurements(&bench),
    [Err(SensorFault::Rejected(ReadingFault::Saturated))]
  );
}

#[test]
fn a_job_with_a_dot_missing_is_no_measurement() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  play(&mut orchestrator, &mut bench, 1_000, 2_100, canned(2));
  assert_eq!(measurements(&bench), [Err(SensorFault::NoReading)]);
  assert!(bench.outcomes().contains(&Outcome::JobMismatch { fires: 2 }));
  assert_eq!(orchestrator.state(), State::Idle);
}

#[test]
fn readings_that_never_arrive_time_out_as_no_measurement() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  let timeout_ms = u64::from(CONFIG.coordinator.reading_timeout_ms);
  play(&mut orchestrator, &mut bench, 1_000, 2_000 + timeout_ms, canned(3));
  assert_eq!(measurements(&bench), [Err(SensorFault::NoReading)]);
  assert!(bench.outcomes().contains(&Outcome::ReadingsTimedOut));
}

#[test]
fn a_job_that_never_fires_faults_and_ends_the_measurement() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  let timeout_ms = u64::from(CONFIG.coordinator.fire_wait_timeout_ms);
  play(&mut orchestrator, &mut bench, 1_000, 1_000 + timeout_ms, quiet);
  assert_eq!(orchestrator.state(), State::Fault(FaultReason::NoFire));
  assert_eq!(measurements(&bench), [Err(SensorFault::NoReading)]);
}

#[test]
fn readings_outside_a_measurement_change_nothing() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  for seq in 0..3 {
    orchestrator.on_frame(1_000, answer(seq, SensorResult::Valid), &mut bench);
  }
  assert!(bench.take().is_empty());
  assert_eq!(orchestrator.state(), State::Idle);
}

#[test]
fn a_measurement_is_refused_without_the_sensor_outside_idle_and_while_anything_runs() {
  let (mut orchestrator, mut bench) = idle();
  assert_eq!(
    orchestrator.request_measurement(1_001, &mut bench),
    Err(RequestError::NodeAbsent { node: SENSOR.get() })
  );
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_move(1_000, Axis::M1H, 100, &mut bench).unwrap();
  assert_eq!(
    orchestrator.request_measurement(1_000, &mut bench),
    Err(RequestError::Busy)
  );
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  assert_eq!(
    orchestrator.request_measurement(1_001, &mut bench),
    Err(RequestError::Busy)
  );
  assert_eq!(
    orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench),
    Err(RequestError::Busy)
  );
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  assert_eq!(
    orchestrator.request_measurement(0, &mut bench),
    Err(RequestError::NotIdle {
      state: State::Inhibited
    })
  );
  assert!(!bench.footsw);
}

#[test]
fn a_measurement_is_refused_while_the_ruida_reports_a_job() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  play(&mut orchestrator, &mut bench, 1_000, 1_000, |_| (true, false));
  assert_eq!(
    orchestrator.request_measurement(1_001, &mut bench),
    Err(RequestError::Refused)
  );
  assert!(!bench.footsw);
}

#[test]
fn the_sensor_node_is_not_configured_as_a_motor_node() {
  let mut orchestrator = Orchestrator::new(setup());
  let mut bench = Bench::default();
  orchestrator.on_frame(0, sensor_beat(0), &mut bench);
  assert!(
    !bench
      .sent()
      .iter()
      .any(|message| matches!(message, Message::Config { .. }))
  );
  assert!(!bench.reports().contains(&Report::Configured { node: SENSOR }));
}

// --- the guard property ---------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum Step {
  Tick {
    loop_open: Option<bool>,
    job_running: bool,
    firing: bool
  },
  /// A move asked for, then a tick in the same millisecond on inputs sampled before it, as the firmware's loop runs.
  RequestThenTick {
    on_mirror_2: bool,
    loop_open: bool
  },
  Heartbeat,
  Mirror2Heartbeat,
  /// Mirror 1's uptime jumps back: it restarted.
  Restart,
  RequestMove {
    on_mirror_2: bool,
    steps: i32
  },
  Stop,
  Answer(MoveResult),
  NodeFault(u8),
  BusOff,
  ClearFault,
  SensorHeartbeat,
  RequestMeasurement,
  Reading(SensorResult)
}

fn step() -> impl Strategy<Value = Step> {
  prop_oneof![
    8 => (proptest::option::of(any::<bool>()), proptest::bool::weighted(0.05), proptest::bool::weighted(0.05))
      .prop_map(|(loop_open, job_running, firing)| Step::Tick { loop_open, job_running, firing }),
    1 => (any::<bool>(), any::<bool>()).prop_map(|(on_mirror_2, loop_open)| Step::RequestThenTick { on_mirror_2, loop_open }),
    3 => Just(Step::Heartbeat),
    2 => Just(Step::Mirror2Heartbeat),
    1 => Just(Step::Restart),
    2 => (any::<bool>(), -500_i32..500).prop_map(|(on_mirror_2, steps)| Step::RequestMove { on_mirror_2, steps }),
    1 => Just(Step::Stop),
    1 => prop_oneof![
      Just(MoveResult::Done),
      Just(MoveResult::Stopped),
      Just(MoveResult::SoftLimit),
      Just(MoveResult::RailLost)
    ]
    .prop_map(Step::Answer),
    1 => prop_oneof![Just(0x21_u8), Just(0x25), Just(0x2A), Just(0x2C), Just(0x2D)].prop_map(Step::NodeFault),
    1 => Just(Step::BusOff),
    1 => Just(Step::ClearFault),
    2 => Just(Step::SensorHeartbeat),
    1 => Just(Step::RequestMeasurement),
    1 => prop_oneof![Just(SensorResult::Valid), Just(SensorResult::NoShot)].prop_map(Step::Reading)
  ]
}

proptest! {
  /// Whatever happens: a MOVE_REL goes out only when the last INHIBIT sent was INHIBIT 0, the relay is energized,
  /// and a tick in a later millisecond than the energize read the loop open; the relay never energizes while
  /// FootSW is held, nor FootSW while the relay is; and in Fault or Inhibited motion is never allowed and the relay
  /// is released.
  #[test]
  fn a_move_goes_out_only_with_motion_allowed_the_relay_energized_and_the_loop_confirmed_open(
    steps in proptest::collection::vec(step(), 1..300)
  ) {
    let (mut orchestrator, mut bench) = idle();
    let mut now_ms = 1_000_u64;
    let mut last_inhibited = true;
    // When the relay last energized, and whether a tick after that millisecond has read the loop open.
    let mut energized_ms: Option<u64> = None;
    let mut confirmed = false;
    let mut relay = false;
    let mut footsw = false;
    let mirror = |on_mirror_2: bool| if on_mirror_2 { Axis::M2H } else { Axis::M1H };
    for step in steps {
      now_ms += 7;
      let before = bench.events.len();
      // The loop open as read by the tick of this step, if there is one, and when.
      let mut reading: Option<bool> = None;
      match step {
        Step::Tick { loop_open, job_running, firing } => {
          // `None`: the loop follows the relay, as on the bench.
          let loop_open = loop_open.unwrap_or(bench.relay);
          reading = Some(loop_open);
          orchestrator.tick(now_ms, Inputs { loop_open, job_running, firing }, &mut bench);
        }
        Step::RequestThenTick { on_mirror_2, loop_open } => {
          let _ = orchestrator.request_move(now_ms, mirror(on_mirror_2), 100, &mut bench);
          reading = Some(loop_open);
          orchestrator.tick(now_ms, Inputs { loop_open, job_running: false, firing: false }, &mut bench);
        }
        Step::Heartbeat => orchestrator.on_frame(now_ms, beat(now_ms), &mut bench),
        Step::Mirror2Heartbeat => {
          orchestrator.on_frame(now_ms, Envelope { node: NodeId::MIRROR_2, ..beat(now_ms) }, &mut bench);
        }
        Step::Restart => {
          let mut restarted = beat(now_ms);
          if let Message::Heartbeat { uptime_ms, .. } = &mut restarted.message {
            *uptime_ms = 5;
          }
          orchestrator.on_frame(now_ms, restarted, &mut bench);
        }
        Step::RequestMove { on_mirror_2, steps } => {
          let _ = orchestrator.request_move(now_ms, mirror(on_mirror_2), steps, &mut bench);
        }
        Step::Stop => orchestrator.stop(&mut bench),
        Step::Answer(result) => {
          if let Some((node, move_rel)) = bench.move_rels().last().copied() {
            let done = Envelope { node, ..move_done(move_rel.seq, result, 0) };
            orchestrator.on_frame(now_ms, done, &mut bench);
          }
        }
        Step::NodeFault(code) => {
          let seq = bench.move_rels().last().map_or(0, |(_, move_rel)| move_rel.seq);
          let fault = Envelope {
            node: MIRROR_1,
            message: Message::Fault {
              channel: Channel::ZERO,
              code: align_proto::FaultCode(code),
              detail: u16::from(seq)
            }
          };
          orchestrator.on_frame(now_ms, fault, &mut bench);
        }
        Step::BusOff => orchestrator.bus_off(now_ms, &mut bench),
        Step::ClearFault => orchestrator.clear_fault(now_ms, &mut bench),
        Step::SensorHeartbeat => orchestrator.on_frame(now_ms, sensor_beat(now_ms), &mut bench),
        Step::RequestMeasurement => {
          let _ = orchestrator.request_measurement(now_ms, &mut bench);
        }
        Step::Reading(result) => {
          let seq = shots_sent(&bench).last().map_or(0, |(seq, _)| *seq);
          orchestrator.on_frame(now_ms, answer(seq, result), &mut bench);
        }
      }
      for event in &bench.events[before..] {
        match event {
          Io::Relay(level) => {
            prop_assert!(!(*level && footsw), "the relay energized with FootSW held");
            relay = *level;
            energized_ms = level.then_some(now_ms);
            confirmed = false;
          }
          Io::FootSw(level) => {
            prop_assert!(!(*level && relay), "FootSW pulsed with the relay energized");
            footsw = *level;
          }
          Io::Sent(Envelope { message: Message::Inhibit { inhibited, .. }, .. }) => {
            // The tick that sends the beacon has read the loop already: count its reading before the move.
            if let (Some(true), Some(energized)) = (reading, energized_ms)
              && now_ms > energized
            {
              confirmed = true;
            }
            last_inhibited = *inhibited;
          }
          Io::Sent(Envelope { message: Message::MoveRel(_), .. }) => {
            prop_assert!(!last_inhibited, "a MOVE_REL while inhibited");
            prop_assert!(relay, "a MOVE_REL with the relay released");
            prop_assert!(confirmed, "a MOVE_REL with no open reading after the energize's millisecond");
          }
          _ => {}
        }
      }
      if let (Some(true), Some(energized)) = (reading, energized_ms)
        && now_ms > energized
      {
        confirmed = true;
      }
      if matches!(orchestrator.state(), State::Fault(_) | State::Inhibited) {
        prop_assert!(!orchestrator.motion_allowed(), "motion allowed in {:?}", orchestrator.state());
        prop_assert!(!bench.relay, "the relay energized in {:?}", orchestrator.state());
      }
    }
  }
}

// --- nemesis review of 2026-11-17: each test below fails on the code as reviewed -------------------------------

/// The firmware's `control` loop samples the inputs first, then runs the console's commands, then ticks with that
/// sample (`firmware/coordinator/src/main.rs`, `control`). A `jog` energizes the relay inside `request_move`, so the
/// first feedback reading the state machine gets after the energize was taken before it.
#[test]
fn a_feedback_reading_sampled_before_the_energize_does_not_allow_motion() {
  let (mut orchestrator, mut bench) = idle();
  // At the top of the loop the feedback reads open while the relay is still released: contact bounce after the last
  // move's release, or a fault on the feedback wire. The relay has not operated: nothing confirms the loop open.
  let sampled = Inputs {
    loop_open: true,
    job_running: false,
    firing: false
  };
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_001, sampled, &mut bench);
  assert!(
    bench.move_rels().is_empty() && !orchestrator.motion_allowed(),
    "motion allowed on a reading taken before the energize: {:?}",
    bench.events
  );
}

/// Reference 6.6: FootSW held low starts a job. The pulse runs out on its timer whatever the state machine did
/// meanwhile, so a move can be armed while it is still held.
#[test]
fn the_relay_is_never_energized_while_footsw_is_held() {
  let (mut orchestrator, mut bench) = idle_with_sensor();
  orchestrator.request_measurement(1_000, &mut bench).unwrap();
  // The Ruida reports a job and ends it at once (stopped at its panel, or an empty job), inside the 200 ms pulse.
  let aborted = |now_ms: u64| ((1_050..1_100).contains(&now_ms), false);
  play(&mut orchestrator, &mut bench, 1_000, 1_150, aborted);
  assert_eq!(orchestrator.state(), State::Idle);
  // Before the fix the pulse ran on to 1200 ms here; now it ends with the job.
  assert!(!bench.footsw, "FootSW let go when the job ended");
  let _ = orchestrator.request_move(1_151, Axis::M1H, 100, &mut bench);
  play(&mut orchestrator, &mut bench, 1_151, 1_250, quiet);
  let mut footsw = false;
  for event in &bench.events {
    match event {
      Io::FootSw(level) => footsw = *level,
      Io::Relay(true) => assert!(!footsw, "the relay energized with FootSW held: {:?}", bench.events),
      Io::Sent(Envelope {
        message: Message::MoveRel(_),
        ..
      }) => assert!(!footsw, "a MOVE_REL with FootSW held"),
      _ => {}
    }
  }
}

/// The firmware's `EXPECTED` holds mirror 1 only, and `request_move` accepts a move on any present node. When the
/// node running the move goes silent, motion must not stay allowed (reference 7.3: a lost heartbeat inhibits).
#[test]
fn losing_the_node_of_the_move_in_flight_ends_motion_even_if_the_self_test_does_not_need_it() {
  let (mut orchestrator, mut bench) = idle();
  let mirror_2 = |now_ms| Envelope {
    node: NodeId::MIRROR_2,
    ..beat(now_ms)
  };
  orchestrator.on_frame(1_000, mirror_2(1_000), &mut bench);
  orchestrator.request_move(1_001, Axis::M2H, 1_000, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  assert!(orchestrator.motion_allowed());
  // Mirror 2 says nothing more; mirror 1 keeps beating. Mirror 2 is reported lost at 1301 ms.
  run(&mut orchestrator, &mut bench, 1_003, 1_400);
  assert!(bench.reports().contains(&Report::Node {
    node: NodeId::MIRROR_2,
    change: Change::Lost
  }));
  assert!(
    !orchestrator.motion_allowed() && !bench.relay,
    "still {:?} with the relay energized after the moving node was lost",
    orchestrator.state()
  );
}

/// A node reports a failed flash write as FAULT `PERSIST_FAILED` (`node_logic::Node::on_persisted`) and keeps running
/// the move whose suspect record failed to write. Its detail was 0, which the coordinator read as the refusal of move
/// 0, the first move each node gets; it is now `DETAIL_NO_COMMAND`, which no sequence number equals.
#[test]
fn a_failed_record_write_is_not_taken_for_the_refusal_of_move_zero() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  assert_eq!(bench.move_rels().last().map(|(_, move_rel)| move_rel.seq), Some(0));
  let write_failed = Envelope {
    node: MIRROR_1,
    message: Message::Fault {
      channel: Channel::ZERO,
      code: align_proto::FaultCode::PERSIST_FAILED,
      detail: align_proto::DETAIL_NO_COMMAND
    }
  };
  orchestrator.on_frame(1_003, write_failed, &mut bench);
  let ended: Vec<Report> = bench
    .reports()
    .into_iter()
    .filter(|report| matches!(report, Report::MoveEnded { .. }))
    .collect();
  assert!(
    ended.is_empty(),
    "the move the node is running was ended by a write failure: {ended:?}"
  );
}
