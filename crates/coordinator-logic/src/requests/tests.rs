//! The procedure's requests against the real orchestrator: started in Idle, measurements after the settle time,
//! completed by the orchestrator's reports, and refused or timed out as failures the aligner understands.

extern crate std;

use std::vec::Vec;

use align_core::config::Config;
use align_proto::{Channel, Envelope, Message, MoveDone, MoveResult, NodeId, PROTOCOL_VERSION};

use align_core::aligner::AxisState;
use align_core::approach::LastDir;
use align_core::axis::PerAxis;

use super::*;
use crate::orchestrator::{CoordinatorIo, Inputs, Report, Setup, WIRING};

const CONFIG: Config = Config::STARTING;
const SETTLE_MS: u64 = CONFIG.motion.settle_ms as u64;
const WAIT_LIMIT_MS: u32 = 5_000;

/// Records the reports and keeps the relay level, for the feedback to follow it.
#[derive(Default)]
struct Bench {
  reports: Vec<Report>,
  relay: bool
}

impl CoordinatorIo for Bench {
  fn send(&mut self, _envelope: Envelope) {}

  fn set_relay(&mut self, energized: bool) {
    self.relay = energized;
  }

  fn set_footsw(&mut self, _active: bool) {}

  fn report(&mut self, report: Report) {
    self.reports.push(report);
  }
}

/// The orchestrator, the requests, and the bench, run together as the firmware's control loop runs them.
struct Loop {
  orchestrator: Orchestrator,
  requests: Requests,
  bench: Bench,
  completions: Vec<(u64, Completion)>
}

impl Loop {
  /// Idle at 1000 ms with mirror 1 present.
  fn idle() -> Self {
    let mut this = Self {
      orchestrator: Orchestrator::new(Setup {
        config: CONFIG,
        fw_version: 1,
        expected: &[NodeId::MIRROR_1],
        wiring: WIRING
      }),
      requests: Requests::new(CONFIG.motion.settle_ms, WAIT_LIMIT_MS),
      bench: Bench::default(),
      completions: Vec::new()
    };
    this.run(0, 1_000);
    assert_eq!(this.orchestrator.state(), State::Idle);
    this
  }

  /// One millisecond of the control loop: the tick, then the requests, then the reports.
  fn step(&mut self, now_ms: u64) {
    if now_ms.is_multiple_of(100) {
      let beat = Envelope {
        node: NodeId::MIRROR_1,
        message: Message::Heartbeat {
          uptime_ms: u32::try_from(now_ms).unwrap(),
          state: 1,
          fw_version: 1,
          proto_version: PROTOCOL_VERSION
        }
      };
      self.orchestrator.on_frame(now_ms, beat, &mut self.bench);
    }
    let inputs = Inputs {
      loop_open: self.bench.relay,
      job_running: false,
      firing: false
    };
    self.orchestrator.tick(now_ms, inputs, &mut self.bench);
    if let Some(completion) = self.requests.poll(now_ms, &mut self.orchestrator, &mut self.bench) {
      self.completions.push((now_ms, completion));
    }
    self.drain(now_ms);
  }

  fn drain(&mut self, now_ms: u64) {
    for report in core::mem::take(&mut self.bench.reports) {
      if let Some(completion) = self.requests.on_report(now_ms, &report) {
        self.completions.push((now_ms, completion));
      }
    }
  }

  fn run(&mut self, from_ms: u64, to_ms: u64) {
    for now_ms in from_ms..=to_ms {
      self.step(now_ms);
    }
  }

  /// The node's answer to its move `seq`.
  fn done(&mut self, now_ms: u64, seq: u8, pos_steps: i32) {
    let done = Envelope {
      node: NodeId::MIRROR_1,
      message: Message::MoveDone(MoveDone {
        seq,
        channel: Channel::ZERO,
        result: MoveResult::Done,
        pos_steps
      })
    };
    self.orchestrator.on_frame(now_ms, done, &mut self.bench);
    self.drain(now_ms);
  }
}

#[test]
fn a_move_starts_at_once_in_idle_and_completes_with_the_node_s_answer() {
  let mut control = Loop::idle();
  control
    .requests
    .submit(
      1_001,
      Request::Move {
        axis: Axis::M1H,
        steps: 40
      }
    )
    .unwrap();
  assert!(control.requests.busy());
  control.run(1_001, 1_100);
  assert!(control.completions.is_empty());
  control.done(1_101, 0, 40);
  assert_eq!(control.completions, [(1_101, Completion::Move(Ok(40)))]);
  assert!(!control.requests.busy());
}

#[test]
fn one_request_at_a_time() {
  let mut control = Loop::idle();
  control.requests.submit(1_001, Request::Measure).unwrap();
  assert_eq!(control.requests.submit(1_001, Request::Measure), Err(Busy));
}

#[test]
fn a_request_waits_for_idle_then_starts() {
  let mut control = Loop::idle();
  control
    .requests
    .submit(
      1_001,
      Request::Move {
        axis: Axis::M1H,
        steps: 40
      }
    )
    .unwrap();
  control.run(1_001, 1_010);
  control.done(1_011, 0, 40);
  // Disarming until the next tick reads the loop closed: the next move waits for Idle, then starts.
  control
    .requests
    .submit(
      1_011,
      Request::Move {
        axis: Axis::M1H,
        steps: -40
      }
    )
    .unwrap();
  control.run(1_012, 1_100);
  control.done(1_101, 1, 0);
  let moves: Vec<Completion> = control.completions.iter().map(|(_, completion)| *completion).collect();
  assert_eq!(moves, [Completion::Move(Ok(40)), Completion::Move(Ok(0))]);
}

#[test]
fn a_measurement_waits_for_the_last_move_to_settle() {
  let mut control = Loop::idle();
  control
    .requests
    .submit(
      1_001,
      Request::Move {
        axis: Axis::M1H,
        steps: 40
      }
    )
    .unwrap();
  control.run(1_001, 1_010);
  control.done(1_011, 0, 40);
  // No sensor node on this bench: the measurement is refused when it starts, which shows when it started.
  control.requests.submit(1_011, Request::Measure).unwrap();
  control.run(1_012, 1_011 + SETTLE_MS - 1);
  assert_eq!(control.completions.len(), 1, "still settling");
  control.run(1_011 + SETTLE_MS, 1_011 + SETTLE_MS);
  assert_eq!(
    control.completions.last(),
    Some(&(1_011 + SETTLE_MS, Completion::Measure(Err(SensorFault::NoReading))))
  );
}

#[test]
fn a_refused_request_completes_at_once_as_the_failure_the_aligner_knows() {
  let mut control = Loop::idle();
  // Mirror 2 is not on the bus.
  control
    .requests
    .submit(
      1_001,
      Request::Move {
        axis: Axis::M2H,
        steps: 40
      }
    )
    .unwrap();
  control.step(1_001);
  assert_eq!(
    control.completions,
    [(
      1_001,
      Completion::Move(Err(MoveFault::Refused {
        code: COORDINATOR_REFUSED
      }))
    )]
  );
  assert!(!control.requests.busy());
}

#[test]
fn a_request_that_cannot_start_within_the_limit_fails() {
  let mut control = Loop::idle();
  control.orchestrator.fault(1_000, &mut control.bench);
  control
    .requests
    .submit(
      1_001,
      Request::Move {
        axis: Axis::M1H,
        steps: 40
      }
    )
    .unwrap();
  control.run(1_001, 1_001 + u64::from(WAIT_LIMIT_MS) - 1);
  assert!(control.completions.is_empty(), "the fault may be cleared meanwhile");
  control.step(1_001 + u64::from(WAIT_LIMIT_MS));
  assert_eq!(
    control.completions,
    [(
      1_001 + u64::from(WAIT_LIMIT_MS),
      Completion::Move(Err(MoveFault::Refused {
        code: COORDINATOR_REFUSED
      }))
    )]
  );
}

#[test]
fn only_the_report_of_the_running_request_completes_it() {
  let mut requests = Requests::new(CONFIG.motion.settle_ms, WAIT_LIMIT_MS);
  let ended = Report::MoveEnded {
    axis: Axis::M1H,
    result: Ok(5)
  };
  assert_eq!(requests.on_report(10, &ended), None, "nothing is running");
  let measured = Report::MeasurementEnded {
    result: Err(SensorFault::NoReading)
  };
  assert_eq!(requests.on_report(10, &measured), None);
}

// --- the axes the procedure starts from ---------------------------------------------------------------------------

/// A link that has heard mirror 1, and its channels' last STATUS as given.
fn heard(channels: &[(u8, i32, bool)]) -> crate::link::Link {
  let mut link = crate::link::Link::new(&CONFIG.node, 1);
  let beat = Envelope {
    node: NodeId::MIRROR_1,
    message: Message::Heartbeat {
      uptime_ms: 10,
      state: 1,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION
    }
  };
  link.on_frame(10, beat);
  for &(channel, pos_steps, suspect) in channels {
    let status = Envelope {
      node: NodeId::MIRROR_1,
      message: Message::Status(align_proto::Status {
        channel: Channel::new(channel).unwrap(),
        flags: if suspect {
          align_proto::StatusFlags::SUSPECT
        } else {
          align_proto::StatusFlags::empty()
        },
        pos_steps,
        current_ma: 0
      })
    };
    link.on_frame(10, status);
  }
  link
}

fn known(pos_steps: i32, last_dir: LastDir) -> AxisState {
  AxisState {
    pos_steps,
    last_dir,
    suspect: false,
    lost: false
  }
}

#[test]
fn where_the_node_agrees_the_aligner_keeps_its_state() {
  let link = heard(&[(0, 120, false), (1, -40, false)]);
  let mut current = PerAxis::splat(AxisState::default());
  current.m1_h = known(120, LastDir::Plus);
  current.m1_v = known(-40, LastDir::Minus);
  let axes = starting_axes(&current, &reported(&link, &WIRING));
  assert_eq!(axes.m1_h, known(120, LastDir::Plus));
  assert_eq!(axes.m1_v, known(-40, LastDir::Minus));
}

#[test]
fn where_the_node_disagrees_its_count_is_taken_with_the_direction_unknown() {
  let link = heard(&[(0, 500, false), (1, 0, true)]);
  let mut current = PerAxis::splat(AxisState::default());
  current.m1_h = known(120, LastDir::Plus);
  let axes = starting_axes(&current, &reported(&link, &WIRING));
  assert_eq!(axes.m1_h, known(500, LastDir::Unknown));
  assert!(axes.m1_v.suspect, "the node's own suspect flag");
  assert_eq!(axes.m1_v.pos_steps, 0);
}

#[test]
fn an_axis_whose_node_has_not_reported_is_lost_and_one_that_reports_again_is_found() {
  let link = heard(&[(0, 0, false)]);
  let axes = starting_axes(&PerAxis::splat(AxisState::default()), &reported(&link, &WIRING));
  assert!(!axes.m1_h.lost);
  assert!(axes.m1_v.lost, "no STATUS for channel 1");
  assert!(axes.m2_h.lost && axes.m2_v.lost, "mirror 2 is not on the bus");
  let mut current = PerAxis::splat(AxisState::default());
  current.m1_h.lost = true;
  let found = starting_axes(&current, &reported(&link, &WIRING));
  assert!(!found.m1_h.lost);
  assert!(
    found.m1_h.suspect,
    "it was lost: the count is the node's, the axis suspect"
  );
}
