//! Tests of the coordinator state machine: each transition and fault,
//! then the interlock rules as a property over random event sequences.

extern crate std;

use std::vec;
use std::vec::Vec;

use proptest::prelude::*;

use super::*;

const TIMING: CoordinatorTiming = CoordinatorTiming::STARTING;
const QUIET: u64 = TIMING.quiet_before_move_ms as u64;
const RELAY: u64 = TIMING.relay_feedback_timeout_ms as u64;

/// A coordinator that has passed its self-test.
fn idle() -> Coordinator {
  let mut coordinator = Coordinator::new(TIMING);
  let actions = coordinator.handle(0, Event::SelfTest { passed: true });
  assert_eq!(actions.outcome, Some(Outcome::Ready));
  coordinator
}

/// Runs a full job: start, three fires, Status high then low, readings complete. Returns the shots sent.
fn measure(coordinator: &mut Coordinator, start_ms: u64) -> Vec<(u8, Point)> {
  let mut shots = Vec::new();
  assert!(coordinator.handle(start_ms, Event::StartMeasurement).start_job);
  coordinator.handle(start_ms + 50, Event::JobRunning(true));
  for index in 0..3 {
    if let Some(shot) = coordinator.handle(start_ms + 100 + index * 1_000, Event::Fire).shot {
      shots.push(shot);
    }
  }
  coordinator.handle(start_ms + 3_500, Event::JobRunning(false));
  assert_eq!(
    coordinator.handle(start_ms + 3_600, Event::ReadingsComplete).outcome,
    Some(Outcome::MeasurementReady)
  );
  shots
}

/// Arms a move from Idle at `at_ms`, the relay confirming at once. Returns the time Moving was reached.
fn arm(coordinator: &mut Coordinator, at_ms: u64) -> u64 {
  let actions = coordinator.handle(at_ms, Event::StartMove);
  assert_eq!(actions.relay, Some(Relay::Energize), "quiet already: energized at once");
  let actions = coordinator.handle(at_ms + 10, Event::RelayFeedback { loop_open: true });
  assert_eq!(actions.allow_motion, Some(true));
  assert_eq!(actions.outcome, Some(Outcome::MoveReady));
  assert_eq!(coordinator.state(), State::Moving);
  at_ms + 10
}

#[test]
fn a_coordinator_boots_inhibited_and_needs_a_passed_self_test() {
  let mut coordinator = Coordinator::new(TIMING);
  assert_eq!(coordinator.state(), State::Inhibited);
  coordinator.handle(0, Event::SelfTest { passed: false });
  assert_eq!(coordinator.state(), State::Inhibited);
  assert_eq!(
    coordinator.handle(0, Event::StartMeasurement).outcome,
    Some(Outcome::Refused)
  );
  assert_eq!(coordinator.handle(0, Event::StartMove).outcome, Some(Outcome::Refused));
  coordinator.handle(0, Event::SelfTest { passed: true });
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_job_sends_one_shot_per_fire_for_a_b_and_c_with_rising_sequence_numbers() {
  let mut coordinator = idle();
  assert_eq!(
    measure(&mut coordinator, 0),
    [(0, Point::A), (1, Point::B), (2, Point::C)]
  );
  assert_eq!(
    measure(&mut coordinator, 10_000),
    [(3, Point::A), (4, Point::B), (5, Point::C)]
  );
}

#[test]
fn a_job_with_too_few_or_too_many_fires_is_a_mismatch() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(10, Event::JobRunning(true));
  coordinator.handle(100, Event::Fire);
  coordinator.handle(200, Event::Fire);
  assert_eq!(
    coordinator.handle(300, Event::JobRunning(false)).outcome,
    Some(Outcome::JobMismatch { fires: 2 })
  );
  assert_eq!(coordinator.state(), State::Idle);

  coordinator.handle(1_000, Event::StartMeasurement);
  coordinator.handle(1_010, Event::JobRunning(true));
  for at in [1_100, 1_200, 1_300] {
    assert!(coordinator.handle(at, Event::Fire).shot.is_some());
  }
  assert_eq!(
    coordinator.handle(1_400, Event::Fire).shot,
    None,
    "no SHOT for a fourth dot"
  );
  assert_eq!(
    coordinator.handle(1_500, Event::JobRunning(false)).outcome,
    Some(Outcome::JobMismatch { fires: 4 })
  );
}

#[test]
fn a_job_that_ends_without_firing_is_a_mismatch_and_one_that_never_fires_a_fault() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  // The Status line low before the job has started does not end it.
  coordinator.handle(5, Event::JobRunning(false));
  assert!(matches!(coordinator.state(), State::WaitingForFire { .. }));
  coordinator.handle(10, Event::JobRunning(true));
  assert_eq!(
    coordinator.handle(20, Event::JobRunning(false)).outcome,
    Some(Outcome::JobMismatch { fires: 0 })
  );

  coordinator.handle(1_000, Event::StartMeasurement);
  coordinator.handle(1_000 + u64::from(TIMING.fire_wait_timeout_ms) - 1, Event::Tick);
  assert!(matches!(coordinator.state(), State::WaitingForFire { .. }));
  let actions = coordinator.handle(1_000 + u64::from(TIMING.fire_wait_timeout_ms), Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::NoFire)));
  assert!(actions.estop);
}

#[test]
fn a_job_that_keeps_running_long_after_its_last_fire_is_a_fault() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(10, Event::JobRunning(true));
  coordinator.handle(100, Event::Fire);
  let late = 100 + u64::from(TIMING.fire_wait_timeout_ms);
  assert_eq!(
    coordinator.handle(late, Event::Tick).outcome,
    Some(Outcome::Faulted(FaultReason::JobDidNotEnd))
  );
}

#[test]
fn readings_that_do_not_all_arrive_in_time_invalidate_the_measurement() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(10, Event::JobRunning(true));
  for at in [100, 200, 300] {
    coordinator.handle(at, Event::Fire);
  }
  coordinator.handle(400, Event::JobRunning(false));
  assert!(matches!(coordinator.state(), State::CollectingReadings { .. }));
  coordinator.handle(400 + u64::from(TIMING.reading_timeout_ms) - 1, Event::Tick);
  assert!(matches!(coordinator.state(), State::CollectingReadings { .. }));
  let actions = coordinator.handle(400 + u64::from(TIMING.reading_timeout_ms), Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::ReadingsTimedOut));
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_fire_outside_a_job_is_reported_and_one_after_the_job_invalidates_its_readings() {
  let mut coordinator = idle();
  assert_eq!(
    coordinator.handle(0, Event::Fire).outcome,
    Some(Outcome::UnexpectedFire)
  );
  assert_eq!(coordinator.state(), State::Idle);
  coordinator.handle(1_000, Event::StartMeasurement);
  coordinator.handle(1_010, Event::JobRunning(true));
  for at in [1_100, 1_200, 1_300] {
    coordinator.handle(at, Event::Fire);
  }
  coordinator.handle(1_400, Event::JobRunning(false));
  assert_eq!(
    coordinator.handle(1_500, Event::Fire).outcome,
    Some(Outcome::UnexpectedFire)
  );
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_measurement_is_refused_while_the_ruida_runs_a_job() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::JobRunning(true));
  let actions = coordinator.handle(0, Event::StartMeasurement);
  assert_eq!(actions.outcome, Some(Outcome::Refused));
  assert!(!actions.start_job);
  assert_eq!(coordinator.handle(0, Event::StartMove).outcome, Some(Outcome::Refused));
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_move_waits_for_quiet_before_energizing_the_relay() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::Fire);
  let actions = coordinator.handle(100, Event::StartMove);
  assert_eq!(actions.relay, None, "a fire 100 ms ago");
  assert_eq!(coordinator.handle(QUIET - 1, Event::Tick).relay, None);
  assert_eq!(coordinator.handle(QUIET, Event::Tick).relay, Some(Relay::Energize));
}

#[test]
fn motion_is_allowed_only_once_the_feedback_confirms_the_loop_open() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMove);
  assert_eq!(coordinator.handle(5, Event::Tick).allow_motion, None);
  assert!(matches!(coordinator.state(), State::ArmingMove { .. }));
  let actions = coordinator.handle(8, Event::RelayFeedback { loop_open: true });
  assert_eq!(actions.allow_motion, Some(true));
  assert_eq!(coordinator.state(), State::Moving);
}

#[test]
fn a_relay_that_does_not_open_in_time_is_a_fault_and_is_released() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMove);
  assert!(coordinator.relay_energized());
  coordinator.handle(RELAY, Event::Tick);
  assert!(
    matches!(coordinator.state(), State::ArmingMove { .. }),
    "at the timeout, still waiting"
  );
  let actions = coordinator.handle(RELAY + 1, Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::RelayDidNotOpen)));
  assert_eq!(actions.relay, Some(Relay::Release));
  assert_eq!(actions.allow_motion, Some(false));
  assert!(!coordinator.relay_energized());
}

#[test]
fn a_fire_while_moving_or_arming_is_an_immediate_fault_that_releases_the_relay() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let actions = coordinator.handle(at + 5, Event::Fire);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::FireWhileArmed)));
  assert_eq!(actions.relay, Some(Relay::Release));
  assert_eq!(actions.allow_motion, Some(false));
  assert!(actions.estop);

  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMove);
  let actions = coordinator.handle(1, Event::Fire);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::FireWhileArmed)));
  assert_eq!(actions.relay, Some(Relay::Release));

  // Waiting for quiet, the relay not yet energized: still a fault.
  let mut coordinator = idle();
  coordinator.handle(0, Event::Fire);
  coordinator.handle(10, Event::StartMove);
  let actions = coordinator.handle(20, Event::Fire);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::FireWhileArmed)));
  assert_eq!(actions.relay, None, "nothing to release");
}

#[test]
fn a_job_starting_while_moving_is_an_immediate_fault() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let actions = coordinator.handle(at + 5, Event::JobRunning(true));
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::JobWhileArmed)));
  assert_eq!(actions.relay, Some(Relay::Release));
}

#[test]
fn a_relay_that_drops_while_moving_is_a_fault() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let actions = coordinator.handle(at + 5, Event::RelayFeedback { loop_open: false });
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::RelayDropped)));
  assert_eq!(actions.allow_motion, Some(false));
}

#[test]
fn a_finished_move_disallows_motion_releases_the_relay_and_waits_for_the_loop_to_close() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let actions = coordinator.handle(at + 1_000, Event::MovesDone);
  assert_eq!(actions.allow_motion, Some(false));
  assert_eq!(actions.relay, Some(Relay::Release));
  assert!(matches!(coordinator.state(), State::Disarming { .. }));
  assert_eq!(
    coordinator.handle(at + 1_005, Event::StartMeasurement).outcome,
    Some(Outcome::Refused)
  );
  let actions = coordinator.handle(at + 1_010, Event::RelayFeedback { loop_open: false });
  assert_eq!(actions.outcome, Some(Outcome::MoveFinished));
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_relay_that_does_not_close_after_a_move_is_a_fault() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  coordinator.handle(at + 100, Event::MovesDone);
  coordinator.handle(at + 100 + RELAY, Event::Tick);
  assert!(matches!(coordinator.state(), State::Disarming { .. }));
  let actions = coordinator.handle(at + 101 + RELAY, Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::RelayDidNotClose)));
}

#[test]
fn a_loop_open_while_the_relay_is_released_is_a_fault_in_any_state() {
  // A welded contact or a wiring fault: the loop reads open with the relay never energized.
  let mut coordinator = idle();
  let actions = coordinator.handle(10, Event::RelayFeedback { loop_open: true });
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::RelayDidNotClose)));

  let mut coordinator = Coordinator::new(TIMING);
  coordinator.handle(0, Event::RelayFeedback { loop_open: true });
  assert_eq!(
    coordinator.state(),
    State::Fault(FaultReason::RelayDidNotClose),
    "even before the self-test"
  );
}

#[test]
fn a_lost_node_inhibits_and_releases_the_relay_which_must_then_close() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let actions = coordinator.handle(at + 50, Event::NodeLost);
  assert_eq!(actions.relay, Some(Relay::Release));
  assert_eq!(actions.allow_motion, Some(false));
  assert_eq!(actions.outcome, Some(Outcome::Inhibited));
  assert_eq!(coordinator.state(), State::Inhibited);
  // The loop has the feedback timeout to close.
  coordinator.handle(at + 50 + RELAY, Event::Tick);
  assert_eq!(coordinator.state(), State::Inhibited);
  let actions = coordinator.handle(at + 51 + RELAY, Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::RelayDidNotClose)));
}

#[test]
fn a_procedure_fault_stops_everything_until_cleared_and_then_needs_a_self_test() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let actions = coordinator.handle(at + 1, Event::ProcedureFault);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::Procedure)));
  assert_eq!(actions.relay, Some(Relay::Release));
  assert!(actions.estop);
  assert_eq!(
    coordinator.handle(at + 2, Event::StartMove).outcome,
    Some(Outcome::Refused)
  );
  coordinator.handle(at + 3, Event::RelayFeedback { loop_open: false });
  assert_eq!(
    coordinator.handle(at + 4, Event::ClearFault).outcome,
    Some(Outcome::Inhibited)
  );
  assert_eq!(
    coordinator.handle(at + 5, Event::StartMove).outcome,
    Some(Outcome::Refused)
  );
  coordinator.handle(at + 6, Event::SelfTest { passed: true });
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_fault_keeps_its_reason_through_a_lost_node_and_a_second_fault() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(u64::from(TIMING.fire_wait_timeout_ms), Event::Tick);
  let fault = State::Fault(FaultReason::NoFire);
  assert_eq!(coordinator.state(), fault);
  let actions = coordinator.handle(40_000, Event::NodeLost);
  assert_eq!(
    (coordinator.state(), actions),
    (fault, Actions::default()),
    "a lost node does not clear a fault"
  );
  let actions = coordinator.handle(40_001, Event::ProcedureFault);
  assert_eq!(
    (coordinator.state(), actions),
    (fault, Actions::default()),
    "nor does a second fault replace it"
  );
}

#[test]
fn events_with_nothing_to_do_report_nothing() {
  let mut coordinator = idle();
  for event in [
    Event::Tick,
    Event::ReadingsComplete,
    Event::MovesDone,
    Event::JobRunning(false),
    Event::ClearFault
  ] {
    assert_eq!(coordinator.handle(10, event), Actions::default(), "{event:?}");
  }
  assert_eq!(coordinator.state(), State::Idle);
}

#[test]
fn a_job_of_two_shots_sends_no_shot_for_a_third_fire() {
  let timing = CoordinatorTiming {
    shots_per_job: 2,
    ..TIMING
  };
  let mut coordinator = Coordinator::new(timing);
  coordinator.handle(0, Event::SelfTest { passed: true });
  coordinator.handle(0, Event::StartMeasurement);
  assert_eq!(coordinator.handle(10, Event::Fire).shot, Some((0, Point::A)));
  assert_eq!(coordinator.handle(20, Event::Fire).shot, Some((1, Point::B)));
  assert_eq!(coordinator.handle(30, Event::Fire).shot, None);
}

#[test]
fn the_point_order_is_a_b_c() {
  assert_eq!(
    [0, 1, 2, 3].map(Point::from_index),
    [Some(Point::A), Some(Point::B), Some(Point::C), None]
  );
}

#[derive(Clone, Copy, Debug)]
enum Input {
  Wait(u16),
  Event(Event)
}

fn any_input() -> impl Strategy<Value = Input> {
  let event = prop_oneof![
    any::<bool>().prop_map(|passed| Event::SelfTest { passed }),
    Just(Event::StartMeasurement),
    Just(Event::Fire),
    any::<bool>().prop_map(Event::JobRunning),
    Just(Event::ReadingsComplete),
    Just(Event::StartMove),
    Just(Event::MovesDone),
    any::<bool>().prop_map(|loop_open| Event::RelayFeedback { loop_open }),
    Just(Event::NodeLost),
    Just(Event::ProcedureFault),
    Just(Event::ClearFault),
    Just(Event::Tick)
  ];
  prop_oneof![(0_u16..800).prop_map(Input::Wait), event.prop_map(Input::Event)]
}

proptest! {
  #[test]
  fn the_interlock_rules_hold_for_every_event_sequence(inputs in proptest::collection::vec(any_input(), 0..120)) {
    let mut coordinator = Coordinator::new(TIMING);
    let mut now = 0_u64;
    let mut relay_on = false;
    let mut last_fire: Option<u64> = None;
    let mut job_running = false;
    let mut motion_allowed = false;
    for input in inputs {
      let event = match input {
        Input::Wait(ms) => {
          now += u64::from(ms);
          Event::Tick
        }
        Input::Event(event) => event
      };
      let before = coordinator.state();
      let actions = coordinator.handle(now, event);
      if let Event::Fire = event {
        last_fire = Some(now);
      }
      if let Event::JobRunning(running) = event {
        job_running = running;
      }
      match actions.relay {
        Some(Relay::Energize) => {
          prop_assert!(!job_running, "energized with a job running");
          prop_assert!(last_fire.is_none_or(|fire| now - fire >= QUIET), "energized within the quiet time");
          relay_on = true;
        }
        Some(Relay::Release) => relay_on = false,
        None => {}
      }
      prop_assert_eq!(relay_on, coordinator.relay_energized());
      if relay_on {
        prop_assert!(
          matches!(coordinator.state(), State::ArmingMove { .. } | State::Moving),
          "energized in {:?}", coordinator.state()
        );
      }
      if let Some(allowed) = actions.allow_motion {
        motion_allowed = allowed;
      }
      if actions.allow_motion == Some(true) {
        prop_assert_eq!(coordinator.state(), State::Moving);
        prop_assert!(relay_on);
      }
      if motion_allowed {
        prop_assert_eq!(coordinator.state(), State::Moving, "motion still allowed outside Moving");
      }
      if before == State::Moving && coordinator.state() != State::Moving {
        prop_assert_eq!(actions.allow_motion, Some(false), "left Moving without disallowing motion");
      }
      if matches!(event, Event::Fire | Event::JobRunning(true))
        && matches!(before, State::ArmingMove { .. } | State::Moving)
      {
        prop_assert!(matches!(coordinator.state(), State::Fault(_)));
        prop_assert!(!coordinator.relay_energized());
      }
      if let Some(Outcome::Faulted(_)) = actions.outcome {
        prop_assert!(actions.estop);
        prop_assert!(!coordinator.relay_energized());
      }
    }
  }
}
