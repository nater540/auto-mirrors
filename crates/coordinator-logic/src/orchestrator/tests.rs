//! The orchestrator on a simulated bench: a relay whose feedback follows its drive, one motor node, and the frames
//! the node would send. Every rule of a move, the refusals, the faults on the way, and a property over random
//! sequences: a MOVE_REL goes out only after INHIBIT 0, with the relay energized and its loop reading open.

extern crate std;

use std::vec::Vec;

use align_core::approach::SoftLimits;
use align_core::coordinator::FaultReason;
use align_proto::{ConfigKey, MoveDone, MoveRel, MoveResult, PROTOCOL_VERSION};
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
  Report(Report)
}

/// The simulated bench: records everything, and keeps the relay level for the feedback to follow.
#[derive(Default)]
struct Bench {
  events: Vec<Io>,
  relay: bool
}

impl CoordinatorIo for Bench {
  fn send(&mut self, envelope: Envelope) {
    self.events.push(Io::Sent(envelope));
  }

  fn set_relay(&mut self, energized: bool) {
    self.relay = energized;
    self.events.push(Io::Relay(energized));
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
  let sent = Bench { events, relay: true };
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
fn a_move_the_node_refuses_at_its_soft_limit_disarms_the_same_way() {
  let (mut orchestrator, mut bench) = idle();
  orchestrator.request_move(1_001, Axis::M1H, 100, &mut bench).unwrap();
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  orchestrator.on_frame(1_010, move_done(0, MoveResult::SoftLimit, 27_508), &mut bench);
  assert!(bench.reports().contains(&Report::MoveEnded {
    axis: Axis::M1H,
    result: Err(MoveFault::SoftLimit)
  }));
  assert!(!bench.relay);
  orchestrator.tick(1_011, follow(&bench), &mut bench);
  assert_eq!(orchestrator.state(), State::Idle);
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
    Err(MoveRequestError::NotIdle {
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
    Err(MoveRequestError::Busy)
  );
  orchestrator.tick(1_002, follow(&bench), &mut bench);
  assert_eq!(
    orchestrator.request_move(1_002, Axis::M1H, 100, &mut bench),
    Err(MoveRequestError::Busy)
  );
  assert_eq!(bench.move_rels().len(), 1);
}

#[test]
fn a_move_on_an_axis_whose_node_is_absent_is_refused_and_nothing_is_armed() {
  let (mut orchestrator, mut bench) = idle();
  assert_eq!(
    orchestrator.request_move(1_001, Axis::M2H, 100, &mut bench),
    Err(MoveRequestError::NodeAbsent {
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
    Err(MoveRequestError::Refused)
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

// --- the guard property ---------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
enum Step {
  Tick {
    loop_open: Option<bool>,
    job_running: bool,
    firing: bool
  },
  Heartbeat,
  RequestMove(i32),
  Stop,
  Answer(MoveResult),
  ClearFault
}

fn step() -> impl Strategy<Value = Step> {
  prop_oneof![
    6 => (proptest::option::of(any::<bool>()), proptest::bool::weighted(0.05), proptest::bool::weighted(0.05))
      .prop_map(|(loop_open, job_running, firing)| Step::Tick { loop_open, job_running, firing }),
    3 => Just(Step::Heartbeat),
    2 => (-500_i32..500).prop_map(Step::RequestMove),
    1 => Just(Step::Stop),
    1 => prop_oneof![
      Just(MoveResult::Done),
      Just(MoveResult::Stopped),
      Just(MoveResult::SoftLimit)
    ]
    .prop_map(Step::Answer),
    1 => Just(Step::ClearFault)
  ]
}

proptest! {
  /// Whatever happens, a MOVE_REL goes out only when the last INHIBIT sent was INHIBIT 0, the relay is energized,
  /// and the last feedback read said the loop is open.
  #[test]
  fn a_move_goes_out_only_with_motion_allowed_the_relay_energized_and_the_loop_open(
    steps in proptest::collection::vec(step(), 1..300)
  ) {
    let (mut orchestrator, mut bench) = idle();
    let mut now_ms = 1_000_u64;
    let mut last_inhibited = true;
    let mut last_loop_open = false;
    for step in steps {
      now_ms += 7;
      let before = bench.events.len();
      match step {
        Step::Tick { loop_open, job_running, firing } => {
          // `None`: the loop follows the relay, as on the bench.
          let loop_open = loop_open.unwrap_or(bench.relay);
          last_loop_open = loop_open;
          orchestrator.tick(now_ms, Inputs { loop_open, job_running, firing }, &mut bench);
        }
        Step::Heartbeat => orchestrator.on_frame(now_ms, beat(now_ms), &mut bench),
        Step::RequestMove(steps) => {
          let _ = orchestrator.request_move(now_ms, Axis::M1H, steps, &mut bench);
        }
        Step::Stop => orchestrator.stop(&mut bench),
        Step::Answer(result) => {
          let seq = bench.move_rels().last().map_or(0, |(_, move_rel)| move_rel.seq);
          orchestrator.on_frame(now_ms, move_done(seq, result, 0), &mut bench);
        }
        Step::ClearFault => orchestrator.clear_fault(now_ms, &mut bench)
      }
      let mut relay = bench.events[..before].iter().rev().find_map(|event| match event {
        Io::Relay(level) => Some(*level),
        _ => None
      }).unwrap_or(false);
      for event in &bench.events[before..] {
        match event {
          Io::Relay(level) => relay = *level,
          Io::Sent(Envelope { message: Message::Inhibit { inhibited, .. }, .. }) => last_inhibited = *inhibited,
          Io::Sent(Envelope { message: Message::MoveRel(_), .. }) => {
            prop_assert!(!last_inhibited, "a MOVE_REL while inhibited");
            prop_assert!(relay, "a MOVE_REL with the relay released");
            prop_assert!(last_loop_open, "a MOVE_REL with the loop read closed");
          }
          _ => {}
        }
      }
    }
  }
}
