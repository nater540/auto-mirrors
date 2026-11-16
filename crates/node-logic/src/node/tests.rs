//! Tests of the node state machine: every rule, every refusal, and the guard property over random event sequences.

extern crate std;

use std::vec::Vec;

use align_core::config::{DirSign, Motion, NodeTiming};
use align_proto::{MoveDone, PROTOCOL_VERSION};
use proptest::prelude::*;

use super::*;

#[derive(Clone, Debug, PartialEq)]
enum Io {
  Sent(Message),
  Start(MoveRequest),
  Stop,
  Persist(Channel, AxisRecord)
}

#[derive(Default)]
struct Recorder {
  events: Vec<Io>
}

impl NodeIo for Recorder {
  fn send(&mut self, envelope: Envelope) {
    assert_eq!(
      envelope.node,
      NodeId::MIRROR_1,
      "a node sends only under its own address"
    );
    self.events.push(Io::Sent(envelope.message));
  }
  fn start_move(&mut self, request: MoveRequest) {
    self.events.push(Io::Start(request));
  }
  fn stop_move(&mut self) {
    self.events.push(Io::Stop);
  }
  fn persist(&mut self, channel: Channel, record: AxisRecord) {
    self.events.push(Io::Persist(channel, record));
  }
}

impl Recorder {
  fn take(&mut self) -> Vec<Io> {
    core::mem::take(&mut self.events)
  }

  fn faults(&self) -> Vec<FaultCode> {
    let faults = self.events.iter().filter_map(|event| match event {
      Io::Sent(Message::Fault { code, .. }) => Some(*code),
      _ => None
    });
    faults.collect()
  }

  fn starts(&self) -> Vec<MoveRequest> {
    self
      .events
      .iter()
      .filter_map(|event| {
        if let Io::Start(request) = event {
          Some(*request)
        } else {
          None
        }
      })
      .collect()
  }

  fn dones(&self) -> Vec<MoveDone> {
    let dones = self.events.iter().filter_map(|event| match event {
      Io::Sent(Message::MoveDone(done)) => Some(*done),
      _ => None
    });
    dones.collect()
  }
}

const MOTION: Motion = Motion::STARTING;
const TIMING: NodeTiming = NodeTiming::STARTING;

fn ch(value: u8) -> Channel {
  Channel::new(value).unwrap()
}

fn setup() -> NodeSetup {
  NodeSetup {
    id: NodeId::MIRROR_1,
    populated: 2,
    fw_version: 1,
    motion: MOTION,
    timing: TIMING
  }
}

fn from_coordinator(message: Message) -> Envelope {
  Envelope {
    node: NodeId::COORDINATOR,
    message
  }
}

fn to_node(message: Message) -> Envelope {
  Envelope {
    node: NodeId::MIRROR_1,
    message
  }
}

fn heartbeat() -> Envelope {
  heartbeat_version(PROTOCOL_VERSION)
}

fn heartbeat_version(proto_version: u8) -> Envelope {
  from_coordinator(Message::Heartbeat {
    uptime_ms: 0,
    state: 0,
    fw_version: 1,
    proto_version
  })
}

fn allow() -> Envelope {
  from_coordinator(Message::Inhibit {
    inhibited: false,
    source: 0
  })
}

fn config(key: ConfigKey, channel: u8, value: i32) -> Envelope {
  to_node(Message::Config {
    key,
    channel: ch(channel),
    value
  })
}

fn move_rel(seq: u8, channel: u8, delta_steps: i32, finish_plus: bool) -> Envelope {
  to_node(Message::MoveRel(MoveRel {
    seq,
    channel: ch(channel),
    delta_steps,
    finish_plus
  }))
}

/// A node that can move channel 0 at time 0: restored, permit present, heartbeat fresh, allowed, configured.
fn ready() -> (Node, Recorder) {
  ready_with(setup())
}

fn ready_with(setup: NodeSetup) -> (Node, Recorder) {
  let mut node = Node::new(setup);
  let mut io = Recorder::default();
  node.restore([Restored::Missing; MAX_CHANNELS]);
  node.set_permit(true, &mut io);
  node.on_frame(0, heartbeat(), &mut io);
  node.on_frame(0, allow(), &mut io);
  for channel in 0..2 {
    node.on_frame(0, config(ConfigKey::SoftMinSteps, channel, -4_000), &mut io);
    node.on_frame(0, config(ConfigKey::SoftMaxSteps, channel, 4_000), &mut io);
    node.on_frame(0, config(ConfigKey::DirSign, channel, 1), &mut io);
  }
  assert!(io.take().is_empty(), "setting a node up sends nothing");
  (node, io)
}

/// Lets the coordinator's heartbeat go stale, so that the node forgets it, as on a bench with no coordinator.
fn silence_coordinator(node: &mut Node, io: &mut Recorder) {
  node.tick(1_000, io);
  io.take();
}

/// A configured node whose coordinator has gone silent, in bench mode.
fn bench_ready() -> (Node, Recorder) {
  let (mut node, mut io) = ready();
  silence_coordinator(&mut node, &mut io);
  node.set_bench(true).unwrap();
  (node, io)
}

fn done(steps_issued: i32) -> MoveOutcome {
  MoveOutcome::Finished {
    end: MoveEnd::Done,
    steps_issued,
    phase: Phase::ZERO
  }
}

// --- refusals ---------------------------------------------------------------

#[test]
fn a_node_refuses_moves_until_its_positions_are_restored() {
  let mut node = Node::new(setup());
  let mut io = Recorder::default();
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::NOT_READY]);
  assert!(io.starts().is_empty());
}

#[test]
fn a_node_boots_inhibited_and_moves_only_after_the_coordinator_allows_it() {
  let (mut node, mut io) = ready();
  node.on_frame(
    0,
    from_coordinator(Message::Inhibit {
      inhibited: true,
      source: 0
    }),
    &mut io
  );
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::INHIBITED]);

  let fresh = Node::new(setup());
  assert!(fresh.inhibited);
}

#[test]
fn a_move_refused_for_the_node_s_own_reason_runs_when_resent_after_the_reason_is_gone() {
  let (mut node, mut io) = ready();
  node.on_frame(
    0,
    from_coordinator(Message::Inhibit {
      inhibited: true,
      source: 0
    }),
    &mut io
  );
  node.on_frame(0, move_rel(7, 0, 10, false), &mut io);
  assert_eq!(io.take().len(), 1);
  node.on_frame(0, allow(), &mut io);
  node.on_frame(0, move_rel(7, 0, 10, false), &mut io);
  assert_eq!(io.starts().len(), 1);
}

#[test]
fn a_move_needs_a_heartbeat_no_older_than_the_timeout() {
  let (mut node, mut io) = ready();
  node.on_frame(300, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.starts().len(), 1, "300 ms after the heartbeat is still fresh");

  let (mut node, mut io) = ready();
  node.on_frame(301, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::NO_HEARTBEAT]);

  let mut node = Node::new(setup());
  let mut io = Recorder::default();
  node.restore([Restored::Missing; MAX_CHANNELS]);
  node.set_permit(true, &mut io);
  node.on_frame(0, allow(), &mut io);
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::NO_HEARTBEAT], "never a heartbeat");
}

#[test]
fn a_heartbeat_from_another_node_does_not_count() {
  let (mut node, mut io) = ready();
  node.on_frame(
    200,
    to_node(Message::Heartbeat {
      uptime_ms: 0,
      state: 0,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION
    }),
    &mut io
  );
  node.on_frame(400, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::NO_HEARTBEAT]);
}

#[test]
fn a_move_needs_the_hardware_permit() {
  let (mut node, mut io) = ready();
  node.set_permit(false, &mut io);
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::NO_PERMIT]);
}

#[test]
fn a_move_on_a_channel_without_a_motor_is_refused() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 2, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::CHANNEL_UNAVAILABLE]);
}

#[test]
fn a_move_needs_the_soft_limits_and_direction_sign_configured() {
  for missing in [ConfigKey::SoftMinSteps, ConfigKey::SoftMaxSteps, ConfigKey::DirSign] {
    let mut node = Node::new(setup());
    let mut io = Recorder::default();
    node.restore([Restored::Missing; MAX_CHANNELS]);
    node.set_permit(true, &mut io);
    node.on_frame(0, heartbeat(), &mut io);
    node.on_frame(0, allow(), &mut io);
    for (key, value) in [
      (ConfigKey::SoftMinSteps, -100),
      (ConfigKey::SoftMaxSteps, 100),
      (ConfigKey::DirSign, 1)
    ] {
      if key != missing {
        node.on_frame(0, config(key, 0, value), &mut io);
      }
    }
    node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
    assert_eq!(io.faults(), [FaultCode::NOT_CONFIGURED], "without {missing:?}");
  }
}

#[test]
fn a_move_over_the_step_cap_is_refused() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, MOTION.max_move_steps + 1, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::MOVE_TOO_LONG]);
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, MOTION.max_move_steps, false), &mut io);
  assert_eq!(io.starts().len(), 1);
}

#[test]
fn a_move_over_the_time_cap_is_refused() {
  let (mut node, mut io) = ready();
  // 300 steps at 100 ms each is 30 s, over the 25 s cap.
  node.on_frame(0, config(ConfigKey::StepPeriodUs, 0, 100_000), &mut io);
  node.on_frame(0, move_rel(1, 0, 300, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::MOVE_TOO_LONG]);
  // 240 steps is 24 s plus the dwell and hold: inside.
  node.on_frame(0, move_rel(2, 0, 240, false), &mut io);
  assert_eq!(io.starts().len(), 1);
}

#[test]
fn a_move_that_would_leave_the_soft_limits_answers_soft_limit_without_moving() {
  let (mut node, mut io) = ready();
  node.on_frame(0, config(ConfigKey::SoftMaxSteps, 0, 100), &mut io);
  node.on_frame(0, move_rel(1, 0, 101, false), &mut io);
  assert!(io.starts().is_empty());
  let answer = io.dones();
  assert_eq!(answer.len(), 1);
  assert_eq!(answer[0].result, MoveResult::SoftLimit);
  assert_eq!(answer[0].pos_steps, 0);
  io.take();
  node.on_frame(0, move_rel(1, 0, 101, false), &mut io);
  assert_eq!(io.dones(), answer, "a resend gets the same answer");
}

#[test]
fn the_overshoot_waypoint_of_the_standard_approach_is_held_to_the_soft_limits() {
  let (mut node, mut io) = ready();
  node.on_frame(0, config(ConfigKey::SoftMinSteps, 0, -50), &mut io);
  // -10 with finish_plus from an unknown direction goes to -10 - 120 first, below -50.
  node.on_frame(0, move_rel(1, 0, -10, true), &mut io);
  assert!(io.starts().is_empty());
  assert_eq!(io.dones()[0].result, MoveResult::SoftLimit);
}

// --- moves --------------------------------------------------------------------

#[test]
fn an_accepted_move_is_sent_to_the_firmware_with_the_direction_sign_applied() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 25, false), &mut io);
  let request = io.starts()[0];
  assert_eq!(request.channel, ch(0));
  assert_eq!(request.legs_steps, [0, 25]);
  assert_eq!(request.timing.step_period_us, MOTION.step_period_us);

  let (mut node, mut io) = ready();
  node.on_frame(0, config(ConfigKey::DirSign, 1, -1), &mut io);
  node.on_frame(0, move_rel(1, 1, 25, false), &mut io);
  assert_eq!(io.starts()[0].legs_steps, [0, -25]);
}

#[test]
fn a_finish_plus_move_follows_the_standard_approach_and_ends_plus() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, -30, true), &mut io);
  let overshoot = MOTION.overshoot_steps;
  assert_eq!(io.starts()[0].legs_steps, [-30 - overshoot, overshoot]);
  node.on_move_outcome(10, done(-30), &mut io);
  let record = node.axis(ch(0)).unwrap().record;
  assert_eq!(record.pos_steps, -30);
  assert_eq!(record.last_dir, LastDir::Plus);

  // From plus, a plus move goes straight.
  io.take();
  node.on_frame(20, heartbeat(), &mut io);
  node.on_frame(20, move_rel(2, 0, 15, true), &mut io);
  assert_eq!(io.starts()[0].legs_steps, [0, 15]);
}

#[test]
fn a_zero_move_without_the_approach_answers_done_at_once() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 0, false), &mut io);
  assert!(io.starts().is_empty());
  assert_eq!(io.dones()[0].result, MoveResult::Done);
}

#[test]
fn the_timeout_covers_the_move_and_its_margins() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 200, false), &mut io);
  let expected_ms = 200 * MOTION.step_period_us / 1_000
    + MOTION.energize_dwell_ms
    + MOTION.hold_before_release_ms
    + TIMING.move_timeout_margin_ms
    + TIMING.rail_up_timeout_ms;
  assert_eq!(io.starts()[0].timeout_ms, expected_ms);
}

#[test]
fn a_resend_of_the_running_move_is_ignored_and_a_resend_after_it_gets_the_same_answer() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  io.take();
  node.on_frame(5, move_rel(1, 0, 10, false), &mut io);
  assert!(io.take().is_empty());
  node.on_move_outcome(10, done(10), &mut io);
  let answer = io.dones();
  io.take();
  node.on_frame(20, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.dones(), answer);
  assert!(io.starts().is_empty());
}

#[test]
fn a_new_move_while_one_runs_is_refused_as_busy() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  io.take();
  node.on_frame(5, move_rel(2, 1, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::BUSY]);
}

#[test]
fn the_last_sequence_number_with_different_content_is_refused() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  node.on_move_outcome(10, done(10), &mut io);
  io.take();
  node.on_frame(20, move_rel(1, 0, -10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::SEQ_REUSED]);
}

// --- stopping -----------------------------------------------------------------

#[test]
fn a_stale_heartbeat_stops_a_bus_move_once_and_disarms() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  assert!(node.armed(100));
  io.take();
  node.tick(300, &mut io);
  assert!(!io.events.contains(&Io::Stop));
  node.tick(301, &mut io);
  node.tick(400, &mut io);
  assert_eq!(io.events.iter().filter(|event| **event == Io::Stop).count(), 1);
  assert!(!node.armed(301));
}

#[test]
fn an_estop_stops_the_move_and_inhibits_the_node() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  io.take();
  node.on_frame(10, from_coordinator(Message::Estop { reason: 0 }), &mut io);
  assert!(io.events.contains(&Io::Stop));
  assert!(!node.armed(10));
  node.on_move_outcome(
    20,
    MoveOutcome::Finished {
      end: MoveEnd::Stopped,
      steps_issued: 3,
      phase: Phase::ZERO
    },
    &mut io
  );
  io.take();
  node.on_frame(30, move_rel(2, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::INHIBITED]);
}

#[test]
fn an_inhibit_stops_the_move() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  io.take();
  node.on_frame(
    10,
    from_coordinator(Message::Inhibit {
      inhibited: true,
      source: 1
    }),
    &mut io
  );
  assert_eq!(io.events, [Io::Stop]);
}

#[test]
fn losing_the_permit_stops_the_move() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  io.take();
  node.set_permit(false, &mut io);
  assert_eq!(io.events, [Io::Stop]);
  assert!(!node.armed(0));
}

#[test]
fn a_stop_for_this_node_stops_the_move_and_one_for_another_node_does_not() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  io.take();
  node.on_frame(
    10,
    Envelope {
      node: NodeId::MIRROR_2,
      message: Message::Stop { reason: 0 }
    },
    &mut io
  );
  assert!(io.events.is_empty());
  node.on_frame(10, to_node(Message::Stop { reason: 0 }), &mut io);
  assert_eq!(io.events, [Io::Stop]);
}

#[test]
fn a_move_for_another_node_is_ignored() {
  let (mut node, mut io) = ready();
  let other = Envelope {
    node: NodeId::MIRROR_2,
    message: Message::MoveRel(MoveRel {
      seq: 1,
      channel: ch(0),
      delta_steps: 10,
      finish_plus: false
    })
  };
  node.on_frame(0, other, &mut io);
  assert!(io.events.is_empty());
}

#[test]
fn the_arm_pulse_runs_only_while_a_move_runs() {
  let (mut node, mut io) = ready();
  assert!(!node.armed(0));
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  assert!(node.armed(0));
  node.on_move_outcome(10, done(10), &mut io);
  assert!(!node.armed(10));
}

// --- outcomes -----------------------------------------------------------------

#[test]
fn a_finished_move_updates_the_position_persists_and_answers() {
  let (mut node, mut io) = ready();
  node.on_frame(0, config(ConfigKey::DirSign, 0, -1), &mut io);
  node.on_frame(0, move_rel(4, 0, 25, false), &mut io);
  io.take();
  let phase = Phase::new(3).unwrap();
  node.on_move_outcome(
    10,
    MoveOutcome::Finished {
      end: MoveEnd::Done,
      steps_issued: -25,
      phase
    },
    &mut io
  );
  let record = AxisRecord {
    pos_steps: 25,
    last_dir: LastDir::Plus,
    phase,
    suspect: false,
    dir_sign: Some(DirSign::Reversed)
  };
  assert_eq!(io.events[0], Io::Persist(ch(0), record));
  assert_eq!(
    io.dones(),
    [MoveDone {
      seq: 4,
      channel: ch(0),
      result: MoveResult::Done,
      pos_steps: 25
    }]
  );
  let status = io.events.iter().find_map(|event| match event {
    Io::Sent(Message::Status(status)) => Some(*status),
    _ => None
  });
  let status = status.unwrap();
  assert!(status.flags.contains(StatusFlags::AT_TARGET));
  assert!(
    !status.flags.contains(StatusFlags::PERSISTED),
    "not persisted until the write is confirmed"
  );
  assert!(!status.flags.contains(StatusFlags::MOVING));
}

#[test]
fn a_rail_loss_makes_the_position_suspect_and_is_reported() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(4, 0, 25, false), &mut io);
  io.take();
  node.on_move_outcome(
    10,
    MoveOutcome::Finished {
      end: MoveEnd::RailLost,
      steps_issued: 7,
      phase: Phase::ZERO
    },
    &mut io
  );
  let record = node.axis(ch(0)).unwrap().record;
  assert_eq!(
    (record.pos_steps, record.last_dir, record.suspect),
    (7, LastDir::Unknown, true)
  );
  assert_eq!(io.faults(), [FaultCode::RAIL_LOST_DURING_MOVE]);
  assert_eq!(io.dones()[0].result, MoveResult::RailLost);
}

#[test]
fn a_pin_fault_or_a_timeout_leaves_the_position_unchanged_and_suspect() {
  for (outcome, code) in [
    (MoveOutcome::PinFault { phase: Phase::ZERO }, FaultCode::PIN_FAULT),
    (
      MoveOutcome::TimedOut {
        steps_issued: 0,
        phase: Phase::ZERO
      },
      FaultCode::MOVE_TIMEOUT
    )
  ] {
    let (mut node, mut io) = ready();
    node.on_frame(0, move_rel(4, 0, 25, false), &mut io);
    io.take();
    node.on_move_outcome(10, outcome, &mut io);
    let record = node.axis(ch(0)).unwrap().record;
    assert_eq!((record.pos_steps, record.suspect), (0, true));
    assert_eq!(io.faults(), [code]);
    assert_eq!(io.dones()[0].result, MoveResult::Stopped);
  }
}

#[test]
fn a_stopped_move_keeps_its_count_and_is_not_suspect() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(4, 0, 25, false), &mut io);
  node.on_move_outcome(
    10,
    MoveOutcome::Finished {
      end: MoveEnd::Stopped,
      steps_issued: 9,
      phase: Phase::ZERO
    },
    &mut io
  );
  let record = node.axis(ch(0)).unwrap().record;
  assert_eq!(
    (record.pos_steps, record.last_dir, record.suspect),
    (9, LastDir::Unknown, false)
  );
  assert_eq!(io.dones()[0].result, MoveResult::Stopped);
}

#[test]
fn a_confirmed_write_marks_the_record_persisted_and_a_failed_one_is_reported() {
  let (mut node, mut io) = ready();
  node.on_persisted(ch(0), true, &mut io);
  assert!(node.axis(ch(0)).unwrap().persisted);
  node.on_persisted(ch(1), false, &mut io);
  assert_eq!(io.faults(), [FaultCode::PERSIST_FAILED]);
}

#[test]
fn restored_records_are_used_and_a_corrupt_one_starts_fresh_but_suspect() {
  let mut node = Node::new(setup());
  let stored = AxisRecord {
    pos_steps: 812,
    last_dir: LastDir::Plus,
    phase: Phase::new(2).unwrap(),
    suspect: false,
    dir_sign: Some(DirSign::Forward)
  };
  node.restore([Restored::Valid(stored), Restored::Corrupt, Restored::Missing]);
  assert_eq!(
    node.axis(ch(0)).unwrap(),
    AxisState {
      record: stored,
      at_target: false,
      persisted: true
    }
  );
  assert_eq!(
    node.axis(ch(1)).unwrap().record,
    AxisRecord {
      suspect: true,
      ..AxisRecord::FRESH
    }
  );
  assert_eq!(node.axis(ch(2)).unwrap().record, AxisRecord::FRESH);
}

// --- config -------------------------------------------------------------------

#[test]
fn config_values_are_checked_against_their_bounds() {
  let rejected = [
    (ConfigKey::StepPeriodUs, TIMING.min_step_period_us - 1),
    (ConfigKey::StepPeriodUs, TIMING.max_step_period_us + 1),
    (ConfigKey::OvershootSteps, u32::MAX)
  ];
  for (key, value) in rejected {
    let (mut node, mut io) = ready();
    node.on_frame(0, config(key, 0, i32::try_from(value).unwrap_or(-1)), &mut io);
    assert_eq!(io.faults(), [FaultCode::CONFIG_REJECTED], "{key:?} = {value}");
  }
  for value in [0, 2, -2] {
    let (mut node, mut io) = ready();
    node.on_frame(0, config(ConfigKey::DirSign, 0, value), &mut io);
    assert_eq!(io.faults(), [FaultCode::CONFIG_REJECTED], "dir sign {value}");
  }
  let (mut node, mut io) = ready();
  let accepted_period = i32::try_from(TIMING.min_step_period_us).unwrap();
  node.on_frame(0, config(ConfigKey::StepPeriodUs, 0, accepted_period), &mut io);
  node.on_frame(0, config(ConfigKey::OvershootSteps, 0, 1), &mut io);
  assert!(io.faults().is_empty());
  // An overshoot of zero would let a finish-plus move from minus end minus.
  node.on_frame(0, config(ConfigKey::OvershootSteps, 0, 0), &mut io);
  assert_eq!(io.faults(), [FaultCode::CONFIG_REJECTED]);
}

#[test]
fn config_is_refused_while_a_move_runs_and_on_a_channel_without_a_motor() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  io.take();
  node.on_frame(0, config(ConfigKey::OvershootSteps, 1, 50), &mut io);
  assert_eq!(io.faults(), [FaultCode::CONFIG_REJECTED]);
  io.take();
  node.on_frame(0, config(ConfigKey::OvershootSteps, 2, 50), &mut io);
  assert_eq!(io.faults(), [FaultCode::CHANNEL_UNAVAILABLE]);
}

// --- periodic reports ---------------------------------------------------------

#[test]
fn the_node_sends_its_heartbeat_every_period_with_its_state() {
  let (mut node, mut io) = ready();
  for now_ms in [0, 50, 100, 150, 200] {
    node.tick(now_ms, &mut io);
  }
  let heartbeats: Vec<(u32, u8)> = io
    .events
    .iter()
    .filter_map(|event| match event {
      Io::Sent(Message::Heartbeat { uptime_ms, state, .. }) => Some((*uptime_ms, *state)),
      _ => None
    })
    .collect();
  assert_eq!(heartbeats, [(0, NodeState::Idle as u8), (100, 1), (200, 1)]);

  let fresh = Node::new(setup());
  assert_eq!(fresh.state(), NodeState::NotReady);
}

#[test]
fn the_node_sends_the_status_of_each_motor_every_status_period() {
  let (mut node, mut io) = ready();
  node.tick(0, &mut io);
  node.tick(999, &mut io);
  node.tick(1_000, &mut io);
  let statuses: Vec<u8> = io
    .events
    .iter()
    .filter_map(|event| {
      if let Io::Sent(Message::Status(status)) = event {
        Some(status.channel.get())
      } else {
        None
      }
    })
    .collect();
  assert_eq!(statuses, [0, 1, 0, 1]);
}

// --- bench mode ---------------------------------------------------------------

#[test]
fn bench_moves_need_bench_mode_and_the_permit_but_no_heartbeat() {
  let mut node = Node::new(setup());
  let mut io = Recorder::default();
  node.restore([Restored::Missing; MAX_CHANNELS]);
  assert_eq!(
    node.bench_move(ch(0), 100, 5_000, &mut io),
    Err(BenchRefused::NotInBench)
  );
  node.set_bench(true).unwrap();
  assert_eq!(node.bench_move(ch(0), 100, 5_000, &mut io), Err(BenchRefused::NoPermit));
  node.set_permit(true, &mut io);
  assert_eq!(
    node.bench_move(ch(2), 100, 5_000, &mut io),
    Err(BenchRefused::ChannelUnavailable { channel: 2 })
  );
  assert_eq!(
    node.bench_move(ch(0), 100, 1_999, &mut io),
    Err(BenchRefused::OutOfBounds)
  );
  assert_eq!(
    node.bench_move(ch(0), 4_001, 5_000, &mut io),
    Err(BenchRefused::OutOfBounds)
  );
  assert_eq!(node.bench_move(ch(0), 1_000, 5_000, &mut io), Ok(()));
  assert!(node.armed(10_000), "no heartbeat needed on the bench");
  assert_eq!(node.state(), NodeState::Bench);
  assert_eq!(node.bench_move(ch(1), 10, 5_000, &mut io), Err(BenchRefused::Busy));
  assert_eq!(node.set_bench(false), Err(BenchRefused::Busy));
}

#[test]
fn bus_moves_are_refused_in_bench_mode() {
  let (mut node, mut io) = ready();
  node.set_bench(true).unwrap();
  node.on_frame(0, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::BENCH_MODE]);
}

#[test]
fn a_bench_move_reports_no_move_done_on_the_bus() {
  let (mut node, mut io) = bench_ready();
  node.bench_move(ch(0), 50, 5_000, &mut io).unwrap();
  node.on_move_outcome(10, done(50), &mut io);
  assert!(io.dones().is_empty());
  assert_eq!(node.axis(ch(0)).unwrap().record.pos_steps, 50);
}

// --- boundaries ---------------------------------------------------------------

#[test]
fn a_move_whose_duration_equals_the_time_cap_is_accepted() {
  // 200 steps at 5 ms is 1000 ms, plus one dwell and one hold.
  let max_move_ms = 200 * MOTION.step_period_us / 1_000 + MOTION.energize_dwell_ms + MOTION.hold_before_release_ms;
  let (mut node, mut io) = ready_with(NodeSetup {
    motion: Motion { max_move_ms, ..MOTION },
    ..setup()
  });
  node.on_frame(0, move_rel(1, 0, 200, false), &mut io);
  assert_eq!(io.starts().len(), 1);
  node.on_move_outcome(10, done(200), &mut io);
  io.take();
  node.on_frame(20, heartbeat(), &mut io);
  node.on_frame(20, move_rel(2, 0, 201, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::MOVE_TOO_LONG]);
}

#[test]
fn the_time_cap_counts_a_dwell_and_a_hold_for_each_leg_of_the_approach() {
  let (mut node, mut io) = ready();
  node.on_frame(0, config(ConfigKey::StepPeriodUs, 0, 100_000), &mut io);
  node.on_frame(0, config(ConfigKey::OvershootSteps, 0, 10), &mut io);
  // Legs of -238 and +10: 248 steps at 100 ms is 24.8 s, inside 25 s alone, over it with two dwells and holds.
  node.on_frame(0, move_rel(1, 0, -228, true), &mut io);
  assert_eq!(io.faults(), [FaultCode::MOVE_TOO_LONG]);
}

#[test]
fn the_final_waypoint_of_an_approach_is_held_to_the_soft_limits() {
  // From zero with a soft maximum of 100: -10 with the approach goes to -130 and back to -10, inside.
  let (mut node, mut io) = ready();
  node.on_frame(0, config(ConfigKey::SoftMaxSteps, 0, 100), &mut io);
  node.on_frame(0, move_rel(1, 0, -10, true), &mut io);
  assert_eq!(io.starts().len(), 1);

  // A position above a newly lowered maximum: the reverse waypoint is inside, the end is not.
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 50, false), &mut io);
  node.on_move_outcome(10, done(50), &mut io);
  node.on_frame(20, heartbeat(), &mut io);
  node.on_frame(20, config(ConfigKey::SoftMaxSteps, 0, 20), &mut io);
  io.take();
  node.on_frame(20, move_rel(2, 0, -10, true), &mut io);
  assert!(io.starts().is_empty());
  assert_eq!(io.dones()[0].result, MoveResult::SoftLimit);
}

#[test]
fn a_plain_minus_move_leaves_the_last_direction_minus() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, -10, false), &mut io);
  node.on_move_outcome(10, done(-10), &mut io);
  assert_eq!(node.axis(ch(0)).unwrap().record.last_dir, LastDir::Minus);
}

#[test]
fn a_stop_request_disarms_even_with_the_permit_present() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  node.on_frame(10, to_node(Message::Stop { reason: 0 }), &mut io);
  assert!(!node.armed(10));
}

#[test]
fn a_bus_move_disarms_as_soon_as_its_heartbeat_is_stale_even_before_the_next_tick() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  assert!(node.armed(300));
  assert!(!node.armed(301));
}

#[test]
fn the_console_stop_stops_the_running_move() {
  let (mut node, mut io) = bench_ready();
  node.bench_move(ch(0), 500, 5_000, &mut io).unwrap();
  io.take();
  node.stop(&mut io);
  assert_eq!(io.events, [Io::Stop]);
}

#[test]
fn an_estop_or_inhibit_from_another_node_is_ignored_and_so_is_config_for_another_node() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  io.take();
  let other = |message| Envelope {
    node: NodeId::MIRROR_2,
    message
  };
  node.on_frame(10, other(Message::Estop { reason: 0 }), &mut io);
  node.on_frame(
    10,
    other(Message::Inhibit {
      inhibited: true,
      source: 0
    }),
    &mut io
  );
  node.on_frame(
    10,
    other(Message::Config {
      key: ConfigKey::DirSign,
      channel: ch(0),
      value: 7
    }),
    &mut io
  );
  assert!(io.events.is_empty());
  assert!(node.armed(10));
}

#[test]
fn a_channel_beyond_the_board_is_unavailable_whatever_the_setup_claims() {
  let (mut node, mut io) = ready_with(NodeSetup {
    populated: 4,
    ..setup()
  });
  silence_coordinator(&mut node, &mut io);
  node.set_bench(true).unwrap();
  assert_eq!(
    node.bench_move(ch(3), 10, 5_000, &mut io),
    Err(BenchRefused::ChannelUnavailable { channel: 3 })
  );
}

fn statuses(io: &Recorder) -> Vec<Status> {
  let statuses = io.events.iter().filter_map(|event| match event {
    Io::Sent(Message::Status(status)) => Some(*status),
    _ => None
  });
  statuses.collect()
}

#[test]
fn the_status_flags_report_moving_armed_and_rail_for_the_right_channel() {
  let (mut node, mut io) = ready();
  node.set_rail_ok(true);
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  node.tick(0, &mut io);
  let reported = statuses(&io);
  let moving = StatusFlags::MOVING | StatusFlags::ARMED;
  assert!(reported[0].flags.contains(moving | StatusFlags::RAIL_OK));
  assert!(!reported[1].flags.intersects(moving));
  assert!(reported[1].flags.contains(StatusFlags::RAIL_OK));
}

#[test]
fn the_soft_limit_flag_is_set_at_either_limit_and_not_inside() {
  for (pos, at_limit) in [(-100, true), (-99, false), (99, false), (100, true)] {
    let (mut node, mut io) = ready();
    node.on_frame(0, config(ConfigKey::SoftMinSteps, 0, -100), &mut io);
    node.on_frame(0, config(ConfigKey::SoftMaxSteps, 0, 100), &mut io);
    node.on_frame(0, move_rel(1, 0, pos, false), &mut io);
    node.on_move_outcome(10, done(pos), &mut io);
    let flags = statuses(&io)[0].flags;
    assert_eq!(flags.contains(StatusFlags::SOFT_LIMIT), at_limit, "at {pos}");
  }
}

// --- protocol version ---------------------------------------------------------

#[test]
fn a_coordinator_of_another_protocol_version_is_reported_once_and_refused() {
  let (mut node, mut io) = ready();
  node.on_frame(50, heartbeat_version(PROTOCOL_VERSION + 1), &mut io);
  assert_eq!(
    io.faults(),
    [FaultCode::PROTOCOL_MISMATCH],
    "reported on the first mismatched heartbeat"
  );
  node.on_frame(150, heartbeat_version(PROTOCOL_VERSION + 1), &mut io);
  node.on_frame(250, heartbeat_version(PROTOCOL_VERSION + 1), &mut io);
  let faults: Vec<(FaultCode, u16)> = io
    .events
    .iter()
    .filter_map(|event| match event {
      Io::Sent(Message::Fault { code, detail, .. }) => Some((*code, *detail)),
      _ => None
    })
    .collect();
  assert_eq!(
    faults,
    [(FaultCode::PROTOCOL_MISMATCH, u16::from(PROTOCOL_VERSION + 1))]
  );
  io.take();
  node.on_frame(160, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::PROTOCOL_MISMATCH]);
}

#[test]
fn a_mismatched_heartbeat_does_not_keep_a_move_armed() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 1_000, false), &mut io);
  node.on_frame(250, heartbeat_version(PROTOCOL_VERSION + 1), &mut io);
  assert!(!node.armed(301), "the last matching heartbeat was at 0 ms");
}

#[test]
fn a_matching_heartbeat_after_a_mismatch_restores_moves() {
  let (mut node, mut io) = ready();
  node.on_frame(50, heartbeat_version(PROTOCOL_VERSION + 1), &mut io);
  node.on_frame(100, heartbeat(), &mut io);
  io.take();
  node.on_frame(110, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.starts().len(), 1);
}

#[test]
fn the_node_s_heartbeat_carries_the_protocol_version() {
  let (mut node, mut io) = ready();
  node.tick(0, &mut io);
  let versions: Vec<u8> = io
    .events
    .iter()
    .filter_map(|event| match event {
      Io::Sent(Message::Heartbeat { proto_version, .. }) => Some(*proto_version),
      _ => None
    })
    .collect();
  assert_eq!(versions, [PROTOCOL_VERSION]);
}

// --- review findings, 2026-11-15 ---------------------------------------------------------------------------------

fn persisted(io: &Recorder) -> Vec<AxisRecord> {
  let records = io.events.iter().filter_map(|event| match event {
    Io::Persist(_, record) => Some(*record),
    _ => None
  });
  records.collect()
}

#[test]
fn a_timed_out_move_keeps_the_steps_it_issued_and_is_suspect() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(4, 0, 25, false), &mut io);
  node.on_move_outcome(
    10,
    MoveOutcome::TimedOut {
      steps_issued: 7,
      phase: Phase::new(3).unwrap()
    },
    &mut io
  );
  let record = node.axis(ch(0)).unwrap().record;
  assert_eq!(
    (record.pos_steps, record.suspect, record.phase),
    (7, true, Phase::new(3).unwrap())
  );
}

#[test]
fn a_rail_lost_before_the_first_step_is_reported_but_not_suspect() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(4, 0, 25, false), &mut io);
  io.take();
  let outcome = MoveOutcome::Finished {
    end: MoveEnd::RailLost,
    steps_issued: 0,
    phase: Phase::ZERO
  };
  node.on_move_outcome(10, outcome, &mut io);
  assert!(
    !node.axis(ch(0)).unwrap().record.suspect,
    "no coil was energized, so nothing was lost"
  );
  assert_eq!(io.faults(), [FaultCode::RAIL_LOST_DURING_MOVE]);
  assert_eq!(io.dones()[0].result, MoveResult::RailLost);
}

#[test]
fn a_lost_coordinator_is_forgotten_so_its_first_move_after_returning_is_new_and_needs_permission() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(0, 0, 45, false), &mut io);
  node.on_move_outcome(1_000, done(45), &mut io);
  node.tick(10_000, &mut io);
  io.take();
  // The coordinator is back, restarted, and sends its first poke again.
  node.on_frame(10_010, heartbeat(), &mut io);
  node.on_frame(10_020, move_rel(0, 0, 45, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::INHIBITED], "it must allow motion again first");
  assert!(io.dones().is_empty(), "and the old MOVE_DONE is not replayed");
  node.on_frame(10_030, allow(), &mut io);
  node.on_frame(10_040, move_rel(0, 0, 45, false), &mut io);
  assert_eq!(io.starts().len(), 1, "the poke runs");
}

#[test]
fn a_coordinator_restart_shown_by_its_uptime_is_forgotten_too() {
  let (mut node, mut io) = ready();
  let beat = |uptime_ms| {
    from_coordinator(Message::Heartbeat {
      uptime_ms,
      state: 1,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION
    })
  };
  // The set-up's heartbeats carry uptime 0, so this first one reads as a restart: allow motion again.
  node.on_frame(10, beat(5_000), &mut io);
  node.on_frame(15, allow(), &mut io);
  node.on_frame(20, move_rel(0, 0, 45, false), &mut io);
  node.on_move_outcome(30, done(45), &mut io);
  node.on_frame(110, beat(5_100), &mut io);
  node.on_frame(120, move_rel(0, 0, 45, false), &mut io);
  assert_eq!(io.dones().len(), 2, "same coordinator: the resend is answered");
  io.take();
  // 100 ms later by the local clock, the coordinator says it has been up 40 ms: it restarted.
  node.on_frame(210, beat(40), &mut io);
  node.on_frame(220, move_rel(0, 0, 45, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::INHIBITED]);
}

#[test]
fn an_uptime_that_disagrees_with_the_local_clock_by_exactly_the_heartbeat_timeout_is_the_same_coordinator() {
  let timeout = TIMING.heartbeat_timeout_ms;
  let (mut node, mut io) = ready();
  let beat = |uptime_ms| {
    from_coordinator(Message::Heartbeat {
      uptime_ms,
      state: 1,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION
    })
  };
  node.on_frame(10, beat(5_000), &mut io);
  node.on_frame(15, allow(), &mut io);
  node.on_frame(110, beat(5_100 + timeout), &mut io);
  node.on_frame(120, move_rel(1, 0, 10, false), &mut io);
  assert_eq!(io.starts().len(), 1, "at the boundary: not a restart");
  node.on_move_outcome(130, done(10), &mut io);
  node.on_frame(210, beat(5_200 + 2 * timeout + 1), &mut io);
  io.take();
  node.on_frame(220, move_rel(2, 0, 10, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::INHIBITED], "one millisecond more: a restart");
}

#[test]
fn changing_the_direction_sign_converts_the_stored_position_and_writes_it() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 200, false), &mut io);
  node.on_move_outcome(10, done(200), &mut io);
  io.take();
  node.on_frame(20, config(ConfigKey::DirSign, 0, -1), &mut io);
  let record = node.axis(ch(0)).unwrap().record;
  assert_eq!(
    (record.pos_steps, record.last_dir, record.dir_sign),
    (-200, LastDir::Minus, Some(DirSign::Reversed))
  );
  assert_eq!(persisted(&io), [record], "the converted record is written");
  // From minus, a finish-plus move overshoots instead of going straight.
  io.take();
  node.on_frame(30, move_rel(2, 0, 10, true), &mut io);
  assert_eq!(
    io.starts()[0].legs_steps,
    [-(10 - MOTION.overshoot_steps), -MOTION.overshoot_steps]
  );
}

#[test]
fn the_same_direction_sign_again_changes_nothing() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 200, false), &mut io);
  node.on_move_outcome(10, done(200), &mut io);
  io.take();
  node.on_frame(20, config(ConfigKey::DirSign, 0, 1), &mut io);
  assert!(io.events.is_empty());
  assert_eq!(node.axis(ch(0)).unwrap().record.pos_steps, 200);
}

#[test]
fn a_record_with_steps_but_no_sign_becomes_suspect_when_a_sign_is_set() {
  let mut node = Node::new(setup());
  let mut io = Recorder::default();
  let format_1 = AxisRecord {
    pos_steps: 500,
    ..AxisRecord::FRESH
  };
  node.restore([Restored::Valid(format_1), Restored::Missing, Restored::Missing]);
  node.on_frame(0, config(ConfigKey::DirSign, 0, 1), &mut io);
  node.on_frame(0, config(ConfigKey::DirSign, 1, 1), &mut io);
  assert!(node.axis(ch(0)).unwrap().record.suspect);
  assert!(
    !node.axis(ch(1)).unwrap().record.suspect,
    "a fresh record at zero is fine"
  );
  assert_eq!(node.axis(ch(1)).unwrap().record.dir_sign, Some(DirSign::Forward));
}

#[test]
fn the_record_is_written_suspect_before_every_move_starts() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(1, 0, 25, false), &mut io);
  let expected = AxisRecord {
    suspect: true,
    dir_sign: Some(DirSign::Forward),
    ..AxisRecord::FRESH
  };
  assert_eq!(io.events[0], Io::Persist(ch(0), expected));
  assert!(
    matches!(io.events[1], Io::Start(_)),
    "the write is queued before the move"
  );
  assert!(
    !node.axis(ch(0)).unwrap().record.suspect,
    "only the stored copy is marked"
  );
}

#[test]
fn a_failed_record_write_refuses_moves_until_a_write_succeeds() {
  let (mut node, mut io) = ready();
  node.on_persisted(ch(0), false, &mut io);
  io.take();
  node.on_frame(0, move_rel(1, 0, 25, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::PERSIST_FAILED]);
  node.on_persisted(ch(0), true, &mut io);
  node.on_frame(0, move_rel(1, 0, 25, false), &mut io);
  assert_eq!(io.starts().len(), 1);
}

#[test]
fn an_estop_latches_against_bench_moves_until_an_inhibit_0() {
  let (mut node, mut io) = ready();
  node.on_frame(0, from_coordinator(Message::Estop { reason: 0 }), &mut io);
  silence_coordinator(&mut node, &mut io);
  node.set_bench(true).unwrap();
  assert_eq!(
    node.bench_move(ch(0), 100, 5_000, &mut io),
    Err(BenchRefused::EstopLatched)
  );
  node.on_frame(2_000, allow(), &mut io);
  assert_eq!(node.bench_move(ch(0), 100, 5_000, &mut io), Ok(()));
}

#[test]
fn an_estop_during_a_bench_move_disarms_it() {
  let (mut node, mut io) = bench_ready();
  node.bench_move(ch(0), 1_000, 5_000, &mut io).unwrap();
  assert!(node.armed(2_000));
  node.on_frame(2_000, from_coordinator(Message::Estop { reason: 0 }), &mut io);
  assert!(!node.armed(2_000));
}

#[test]
fn bench_moves_are_refused_while_a_coordinator_is_on_the_bus() {
  let (mut node, mut io) = ready();
  node.set_bench(true).unwrap();
  assert_eq!(
    node.bench_move(ch(0), 100, 5_000, &mut io),
    Err(BenchRefused::CoordinatorPresent)
  );
}

#[test]
fn entering_bench_mode_inhibits_and_forgets_the_last_bus_move() {
  let (mut node, mut io) = ready();
  node.on_frame(0, move_rel(3, 0, 25, false), &mut io);
  node.on_move_outcome(10, done(25), &mut io);
  node.set_bench(true).unwrap();
  node.set_bench(false).unwrap();
  io.take();
  node.on_frame(20, move_rel(3, 0, 25, false), &mut io);
  assert_eq!(io.faults(), [FaultCode::INHIBITED]);
  assert!(io.dones().is_empty(), "no stale MOVE_DONE");
}

// --- the guard property -------------------------------------------------------

#[derive(Clone, Debug)]
enum Event {
  Heartbeat,
  Allow,
  Inhibit,
  Estop,
  Permit(bool),
  Move {
    seq: u8,
    channel: u8,
    delta_steps: i32,
    finish_plus: bool
  },
  Stop,
  Outcome(i32),
  Wait(u16)
}

fn any_event() -> impl Strategy<Value = Event> {
  prop_oneof![
    Just(Event::Heartbeat),
    Just(Event::Allow),
    Just(Event::Inhibit),
    Just(Event::Estop),
    any::<bool>().prop_map(Event::Permit),
    (0_u8..4, 0_u8..3, -500_i32..500, any::<bool>()).prop_map(|(seq, channel, delta_steps, finish_plus)| Event::Move {
      seq,
      channel,
      delta_steps,
      finish_plus
    }),
    Just(Event::Stop),
    (-50_i32..50).prop_map(Event::Outcome),
    (0_u16..400).prop_map(Event::Wait)
  ]
}

proptest! {
  #[test]
  fn no_sequence_of_events_starts_a_move_without_the_permit_a_fresh_heartbeat_and_permission(
    events in proptest::collection::vec(any_event(), 0..60),
  ) {
    let (mut node, mut io) = ready();
    node.on_frame(0, from_coordinator(Message::Inhibit { inhibited: true, source: 0 }), &mut io);
    node.set_permit(false, &mut io);
    io.take();
    let mut now_ms = 0_u64;
    let mut permit = false;
    let mut allowed = false;
    let mut last_heartbeat_ms: Option<u64> = Some(0);
    for event in events {
      match event {
        Event::Heartbeat => {
          last_heartbeat_ms = Some(now_ms);
          node.on_frame(now_ms, heartbeat(), &mut io);
        }
        Event::Allow => {
          allowed = true;
          node.on_frame(now_ms, allow(), &mut io);
        }
        Event::Inhibit => {
          allowed = false;
          node.on_frame(now_ms, from_coordinator(Message::Inhibit { inhibited: true, source: 0 }), &mut io);
        }
        Event::Estop => {
          allowed = false;
          node.on_frame(now_ms, from_coordinator(Message::Estop { reason: 0 }), &mut io);
        }
        Event::Permit(present) => {
          permit = present;
          node.set_permit(present, &mut io);
        }
        Event::Move { seq, channel, delta_steps, finish_plus } => {
          node.on_frame(now_ms, move_rel(seq, channel, delta_steps, finish_plus), &mut io);
        }
        Event::Stop => node.on_frame(now_ms, to_node(Message::Stop { reason: 0 }), &mut io),
        Event::Outcome(steps) => node.on_move_outcome(now_ms, done(steps), &mut io),
        Event::Wait(ms) => {
          now_ms += u64::from(ms);
          node.tick(now_ms, &mut io);
        }
      }
      let fresh = last_heartbeat_ms.is_some_and(|seen| now_ms - seen <= u64::from(TIMING.heartbeat_timeout_ms));
      for event in io.take() {
        if let Io::Start(_) = event {
          prop_assert!(permit && allowed && fresh, "a move started at {now_ms} ms without its conditions");
        }
      }
      // The arm pulse never runs without the same conditions.
      if node.armed(now_ms) {
        prop_assert!(permit && allowed && fresh);
      }
    }
  }
}
