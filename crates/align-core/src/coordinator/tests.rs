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

/// A coordinator that has read the loop closed and the Status line low, and passed its self-test.
fn idle() -> Coordinator {
  let mut coordinator = Coordinator::new(TIMING);
  coordinator.handle(0, Event::RelayFeedback { loop_open: false });
  coordinator.handle(0, Event::JobRunning(false));
  let actions = coordinator.handle(0, Event::SelfTest { passed: true });
  assert_eq!(actions.outcome, Some(Outcome::Ready));
  coordinator
}

/// A burst of fire edges at `at_ms`, ending 20 ms later. Returns what its start did.
fn fire(coordinator: &mut Coordinator, at_ms: u64) -> Actions {
  let actions = coordinator.handle(at_ms, Event::Fire);
  coordinator.handle(at_ms + 20, Event::FireEnded);
  actions
}

/// Runs a full job: start, three fires, Status high then low, readings complete. Returns the shots sent.
fn measure(coordinator: &mut Coordinator, start_ms: u64) -> Vec<(u8, Point)> {
  let mut shots = Vec::new();
  assert!(coordinator.handle(start_ms, Event::StartMeasurement).start_job);
  coordinator.handle(start_ms + 50, Event::JobRunning(true));
  for index in 0..3 {
    if let Some(shot) = fire(coordinator, start_ms + 100 + index * 1_000).shot {
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
  assert!(matches!(coordinator.state(), State::Moving { .. }));
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
  coordinator.handle(0, Event::RelayFeedback { loop_open: false });
  coordinator.handle(0, Event::JobRunning(false));
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
  fire(&mut coordinator, 100);
  fire(&mut coordinator, 200);
  assert_eq!(
    coordinator.handle(300, Event::JobRunning(false)).outcome,
    Some(Outcome::JobMismatch { fires: 2 })
  );
  assert_eq!(coordinator.state(), State::Idle);

  coordinator.handle(1_000, Event::StartMeasurement);
  coordinator.handle(1_010, Event::JobRunning(true));
  for at in [1_100, 1_200, 1_300] {
    assert!(fire(&mut coordinator, at).shot.is_some());
  }
  assert_eq!(fire(&mut coordinator, 1_400).shot, None, "no SHOT for a fourth dot");
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
  assert_eq!(fire(&mut coordinator, 0).outcome, Some(Outcome::UnexpectedFire));
  assert_eq!(coordinator.state(), State::Idle);
  coordinator.handle(1_000, Event::StartMeasurement);
  coordinator.handle(1_010, Event::JobRunning(true));
  for at in [1_100, 1_200, 1_300] {
    fire(&mut coordinator, at);
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
  fire(&mut coordinator, 0);
  let actions = coordinator.handle(100, Event::StartMove);
  assert_eq!(actions.relay, None, "a burst that ended 80 ms ago");
  assert_eq!(coordinator.handle(QUIET + 19, Event::Tick).relay, None);
  assert_eq!(
    coordinator.handle(QUIET + 20, Event::Tick).relay,
    Some(Relay::Energize),
    "quiet from its end"
  );
}

#[test]
fn motion_is_allowed_only_once_the_feedback_confirms_the_loop_open() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMove);
  assert_eq!(coordinator.handle(5, Event::Tick).allow_motion, None);
  assert!(matches!(coordinator.state(), State::ArmingMove { .. }));
  let actions = coordinator.handle(8, Event::RelayFeedback { loop_open: true });
  assert_eq!(actions.allow_motion, Some(true));
  assert_eq!(coordinator.state(), State::Moving { since_ms: 8 });
  assert!(coordinator.motion_allowed());
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
  fire(&mut coordinator, 0);
  coordinator.handle(30, Event::StartMove);
  let actions = coordinator.handle(40, Event::Fire);
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

// --- the nemesis review of 2026-11-15 ---------------------------------------------------------------------------

#[test]
fn readings_complete_before_the_job_ends_make_it_ready_when_it_ends() {
  // The sensor answers about 200 ms after each shot; the Status line falls when the head is back, much later.
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(50, Event::JobRunning(true));
  for at in [100, 1_100, 2_100] {
    fire(&mut coordinator, at);
  }
  assert_eq!(coordinator.handle(2_300, Event::ReadingsComplete), Actions::default());
  assert_eq!(
    coordinator.handle(3_500, Event::JobRunning(false)).outcome,
    Some(Outcome::MeasurementReady)
  );
  assert_eq!(coordinator.state(), State::Idle);
  // The next job starts without them.
  coordinator.handle(5_000, Event::StartMeasurement);
  coordinator.handle(5_050, Event::JobRunning(true));
  for at in [5_100, 6_100, 7_100] {
    fire(&mut coordinator, at);
  }
  coordinator.handle(8_000, Event::JobRunning(false));
  assert!(matches!(coordinator.state(), State::CollectingReadings { .. }));
}

#[test]
fn a_fire_or_a_job_while_the_loop_still_reads_open_after_a_release_is_a_fault() {
  let at = 0;
  for (after_release, event, reason) in [
    (Event::MovesDone, Event::Fire, FaultReason::FireWhileArmed),
    (Event::MovesDone, Event::JobRunning(true), FaultReason::JobWhileArmed),
    (Event::NodeLost, Event::Fire, FaultReason::FireWhileArmed),
    (Event::MovesDone, Event::FireEnded, FaultReason::FireWhileArmed)
  ] {
    let mut coordinator = idle();
    let moving = arm(&mut coordinator, at);
    coordinator.handle(moving + 1_000, after_release);
    let actions = coordinator.handle(moving + 1_005, event);
    assert_eq!(
      actions.outcome,
      Some(Outcome::Faulted(reason)),
      "{event:?} after {after_release:?}"
    );
  }
}

#[test]
fn after_a_release_nothing_starts_until_the_loop_reads_closed_again() {
  for release in [Event::NodeLost, Event::ProcedureFault] {
    let mut coordinator = idle();
    let at = arm(&mut coordinator, 0);
    coordinator.handle(at + 10, release);
    coordinator.handle(at + 11, Event::ClearFault);
    assert_eq!(
      coordinator.handle(at + 12, Event::SelfTest { passed: true }).outcome,
      Some(Outcome::Refused),
      "the loop last read open"
    );
    assert_eq!(coordinator.state(), State::Inhibited);
    coordinator.handle(at + 20, Event::RelayFeedback { loop_open: false });
    coordinator.handle(at + 21, Event::SelfTest { passed: true });
    assert_eq!(coordinator.state(), State::Idle);
  }
}

#[test]
fn motion_needs_a_feedback_reading_taken_after_the_energize() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMove);
  assert!(coordinator.relay_energized());
  assert_eq!(
    coordinator.handle(1, Event::Tick).allow_motion,
    None,
    "the closed reading from before says nothing"
  );
  assert_eq!(
    coordinator
      .handle(2, Event::RelayFeedback { loop_open: false })
      .allow_motion,
    None
  );
  assert_eq!(
    coordinator
      .handle(3, Event::RelayFeedback { loop_open: true })
      .allow_motion,
    Some(true)
  );
}

#[test]
fn the_relay_is_energized_only_while_the_loop_reads_closed() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  coordinator.handle(at + 100, Event::MovesDone);
  // Disarming: the loop has not closed yet, so nothing may start.
  assert_eq!(
    coordinator.handle(at + 105, Event::StartMove).outcome,
    Some(Outcome::Refused)
  );
  coordinator.handle(at + 110, Event::RelayFeedback { loop_open: false });
  assert_eq!(coordinator.state(), State::Idle);
  assert_eq!(
    coordinator.handle(at + 1_000, Event::StartMove).relay,
    Some(Relay::Energize)
  );
}

#[test]
fn a_burst_holds_off_the_relay_until_quiet_after_its_end() {
  // A panel pulse fires from 0 to 1500 ms: the burst starts at 0 and ends at 1500.
  let mut coordinator = idle();
  coordinator.handle(0, Event::Fire);
  assert_eq!(
    coordinator.handle(100, Event::StartMove).outcome,
    Some(Outcome::Refused),
    "no move during a burst"
  );
  assert_eq!(
    coordinator.handle(200, Event::StartMeasurement).outcome,
    Some(Outcome::Refused),
    "nor a job"
  );
  coordinator.handle(1_500, Event::FireEnded);
  coordinator.handle(1_600, Event::StartMove);
  assert_eq!(coordinator.handle(1_500 + QUIET - 1, Event::Tick).relay, None);
  assert_eq!(
    coordinator.handle(1_500 + QUIET, Event::Tick).relay,
    Some(Relay::Energize)
  );
}

#[test]
fn a_burst_of_many_edges_is_one_shot() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(50, Event::JobRunning(true));
  let mut shots = Vec::new();
  for dot in [100, 1_100, 2_100] {
    shots.extend(coordinator.handle(dot, Event::Fire).shot);
    coordinator.handle(dot + 30, Event::FireEnded);
  }
  assert_eq!(shots, [(0, Point::A), (1, Point::B), (2, Point::C)]);
  coordinator.handle(3_000, Event::JobRunning(false));
  assert!(matches!(coordinator.state(), State::CollectingReadings { .. }));
}

#[test]
fn a_status_low_read_before_the_line_rose_does_not_end_the_job() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  fire(&mut coordinator, 100);
  assert_eq!(coordinator.handle(130, Event::JobRunning(false)), Actions::default());
  assert_eq!(coordinator.state(), State::Measuring { job_seen: false });
  assert_eq!(
    coordinator.handle(200, Event::StartMove).outcome,
    Some(Outcome::Refused),
    "no move while the job may still be running"
  );
  coordinator.handle(300, Event::JobRunning(true));
  for at in [1_100, 2_100] {
    fire(&mut coordinator, at);
  }
  coordinator.handle(3_000, Event::JobRunning(false));
  assert!(matches!(coordinator.state(), State::CollectingReadings { .. }));
}

#[test]
fn a_job_whose_status_line_never_rose_ends_in_a_fault() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  for at in [100, 1_100, 2_100] {
    fire(&mut coordinator, at);
  }
  coordinator.handle(2_200, Event::ReadingsComplete);
  let last = 2_120 + u64::from(TIMING.fire_wait_timeout_ms);
  coordinator.handle(last - 1, Event::Tick);
  assert_eq!(coordinator.state(), State::Measuring { job_seen: false });
  assert_eq!(
    coordinator.handle(last, Event::Tick).outcome,
    Some(Outcome::Faulted(FaultReason::StatusNeverHigh))
  );
}

#[test]
fn time_that_goes_backward_is_a_fault_and_becomes_the_new_reference() {
  let mut coordinator = idle();
  coordinator.handle(10_000, Event::StartMeasurement);
  let actions = coordinator.handle(9_999, Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::ClockWentBack)));
  assert!(actions.estop);
  assert_eq!(
    coordinator.handle(10_000, Event::ClearFault).outcome,
    Some(Outcome::Inhibited),
    "the fault clears on the new clock"
  );
  // Energized, then the clock steps back: released at once.
  let mut coordinator = idle();
  coordinator.handle(500, Event::StartMove);
  let actions = coordinator.handle(400, Event::Tick);
  assert_eq!(actions.relay, Some(Relay::Release));
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::ClockWentBack)));
}

#[test]
fn nothing_starts_until_the_feedback_and_the_status_line_have_been_read() {
  for reported in [
    &[][..],
    &[Event::RelayFeedback { loop_open: false }][..],
    &[Event::JobRunning(false)][..]
  ] {
    let mut coordinator = Coordinator::new(TIMING);
    for event in reported {
      coordinator.handle(0, *event);
    }
    assert_eq!(
      coordinator.handle(1, Event::SelfTest { passed: true }).outcome,
      Some(Outcome::Refused),
      "{reported:?}"
    );
    assert_eq!(coordinator.state(), State::Inhibited);
  }
  // A Status line read high is still a reading: the Idle state then refuses work until it falls.
  let mut coordinator = Coordinator::new(TIMING);
  coordinator.handle(0, Event::RelayFeedback { loop_open: false });
  coordinator.handle(0, Event::JobRunning(true));
  assert_eq!(
    coordinator.handle(1, Event::SelfTest { passed: true }).outcome,
    Some(Outcome::Ready)
  );
  assert_eq!(coordinator.handle(2, Event::StartMove).outcome, Some(Outcome::Refused));
}

#[test]
fn a_move_that_never_ends_is_a_fault_after_the_backstop_timeout() {
  let mut coordinator = idle();
  let at = arm(&mut coordinator, 0);
  let timeout = u64::from(TIMING.move_timeout_ms);
  coordinator.handle(at + timeout - 1, Event::Tick);
  assert!(coordinator.motion_allowed());
  let actions = coordinator.handle(at + timeout, Event::Tick);
  assert_eq!(actions.outcome, Some(Outcome::Faulted(FaultReason::MoveDidNotEnd)));
  assert_eq!(actions.relay, Some(Relay::Release));
  assert!(!coordinator.motion_allowed());
}

#[test]
fn a_failed_self_test_in_idle_inhibits() {
  let mut coordinator = idle();
  let actions = coordinator.handle(10, Event::SelfTest { passed: false });
  assert_eq!(actions.outcome, Some(Outcome::Inhibited));
  assert_eq!(coordinator.state(), State::Inhibited);
}

#[test]
fn a_job_nobody_asked_for_while_the_readings_arrive_voids_the_measurement() {
  let mut coordinator = idle();
  coordinator.handle(0, Event::StartMeasurement);
  coordinator.handle(50, Event::JobRunning(true));
  for at in [100, 1_100, 2_100] {
    fire(&mut coordinator, at);
  }
  coordinator.handle(2_200, Event::JobRunning(false));
  assert_eq!(
    coordinator.handle(2_300, Event::JobRunning(true)).outcome,
    Some(Outcome::UnexpectedJob)
  );
  assert_eq!(coordinator.handle(2_400, Event::ReadingsComplete), Actions::default());
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
    Just(Event::FireEnded),
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
    let mut firing = false;
    let mut last_fire: Option<u64> = None;
    let mut job_running: Option<bool> = None;
    // The last feedback reading, and whether one reading the loop open came after the last energize.
    let mut feedback: Option<bool> = None;
    let mut open_since_energize = false;
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
      match event {
        Event::Fire | Event::FireEnded => {
          firing = event == Event::Fire;
          last_fire = Some(now);
        }
        Event::JobRunning(running) => job_running = Some(running),
        Event::RelayFeedback { loop_open } => {
          feedback = Some(loop_open);
          open_since_energize |= loop_open;
        }
        _ => {}
      }
      match actions.relay {
        Some(Relay::Energize) => {
          prop_assert!(!firing, "energized during a burst");
          prop_assert_eq!(job_running, Some(false), "energized without the Status line read low");
          prop_assert!(last_fire.is_none_or(|fire| now - fire >= QUIET), "energized within the quiet time");
          prop_assert_eq!(feedback, Some(false), "energized without the loop read closed");
          relay_on = true;
          open_since_energize = false;
        }
        Some(Relay::Release) => relay_on = false,
        None => {}
      }
      prop_assert_eq!(relay_on, coordinator.relay_energized());
      if relay_on {
        prop_assert!(
          matches!(coordinator.state(), State::ArmingMove { .. } | State::Moving { .. }),
          "energized in {:?}", coordinator.state()
        );
      }
      if let Some(allowed) = actions.allow_motion {
        motion_allowed = allowed;
      }
      if actions.allow_motion == Some(true) {
        let moving_now = matches!(coordinator.state(), State::Moving { .. });
        prop_assert!(moving_now);
        prop_assert!(relay_on);
        prop_assert!(open_since_energize, "motion allowed without a feedback reading after the energize");
      }
      prop_assert_eq!(motion_allowed, coordinator.motion_allowed(), "the level disagrees with the changes");
      let moving = |state: State| matches!(state, State::Moving { .. });
      if moving(before) && !moving(coordinator.state()) {
        prop_assert_eq!(actions.allow_motion, Some(false), "left Moving without disallowing motion");
      }
      let armed = matches!(before, State::ArmingMove { .. } | State::Moving { .. }) || feedback == Some(true);
      if matches!(event, Event::Fire | Event::FireEnded | Event::JobRunning(true))
        && armed
        && !matches!(before, State::Fault(_))
      {
        prop_assert!(matches!(coordinator.state(), State::Fault(_)), "{:?} while armed", event);
        prop_assert!(!coordinator.relay_energized());
      }
      if let Some(Outcome::Faulted(_)) = actions.outcome {
        prop_assert!(actions.estop);
        prop_assert!(!coordinator.relay_energized());
      }
    }
  }
}
