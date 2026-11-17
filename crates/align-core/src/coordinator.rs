//! The coordinator state machine (reference 7.3), a pure function of state and event (hand-off section 5).
//!
//! It decides one thing: when the force-guided relay may open the laser's enable loop and the nodes may move their
//! motors. Everything else (talking to the nodes, pairing sensor readings, the alignment procedure) asks it.
//! The firmware feeds it the Ruida's signals and the relay's feedback, and carries out the [`Actions`] it returns.
//!
//! The events are named for what happens, not for the Ruida's pins, so that a test frame without a Ruida can drive
//! the same machine. A start is "start the job", a burst of fire edges is [`Event::Fire`] then
//! [`Event::FireEnded`], and the Status line is [`Event::JobRunning`].
//!
//! States beyond the table of reference 7.3, which leaves these steps implicit (proposed in `docs/SENSOR_PLAN.md`
//! section 13; the owner approved the protocol that needs them):
//!
//! - **`CollectingReadings`:** the job has ended and the sensor's readings are still arriving (SENSOR frames come
//!   after the shots). Reference 7.3 goes from Measuring straight to the next state.
//! - **`ArmingMove`:** the quiet time after the last fire, then the relay energized and its feedback awaited. Motors
//!   may move only once the feedback confirms the laser loop open (reference 7.3, the Moving row).
//! - **`Disarming`:** the nodes told to stop, the relay released, its feedback awaited before a new job may start.
//!
//! Rules that hold in every state:
//!
//! - A fire, or the Ruida reporting a job running, while the relay is energized or about to be, or while the loop
//!   still reads open after a release, is an immediate fault (reference 7.3: "Any fire edge in Moving is an
//!   immediate Fault"). The fault releases the relay at once.
//! - The relay's feedback is watched all the time, not only when entering Moving. The loop reading open while the
//!   relay is released (after its release time), or closed while it is energized and confirmed, is a fault: a
//!   welded contact, a dropped relay, or a wiring fault.
//! - A lost node puts the coordinator back in Inhibited, releasing the relay.
//! - Time that goes backward is a fault: every timeout here would otherwise stop.
//!
//! What the review of 2026-11-15 settled:
//!
//! - **Nothing is assumed about an input until it is reported.** The relay feedback and the Status line start
//!   unknown, and the self-test passes only once the loop has read closed and the Status line has been read.
//! - **The relay is energized only while the loop reads closed, and motion is allowed only on a feedback reading
//!   taken after it was energized**, in a later millisecond. A reading from before says nothing about this energize,
//!   and one stamped with the energize's own millisecond may have been sampled before it.
//! - **A fire is a burst, with a start and an end.** The start counts a shot; the quiet time before a move counts
//!   from the end, so that a burst longer than the quiet time cannot let the relay energize while it lasts. A move
//!   or a measurement is refused while a burst is in progress.
//! - **A job ends only on a Status low that follows a Status high.** A low read before the line rose is stale. A job
//!   whose Status line never rose ends in a fault after the fire timeout: the line is broken, and while the machine
//!   waits no move can be armed.
//! - **Readings may be complete before the Status line falls** (the sensor answers about 200 ms after each shot).
//!   They are kept, and the measurement is ready when the job ends.

use crate::config::CoordinatorTiming;

/// A measurement point of the canned job (reference 1.4), in the order the job fires them.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Point {
  /// Gantry near, head near.
  A,
  /// Gantry far, head near.
  B,
  /// Gantry far, head far.
  C
}

impl Point {
  /// The point fired `index`-th in a job, from 0.
  pub const fn from_index(index: u8) -> Option<Self> {
    match index {
      0 => Some(Self::A),
      1 => Some(Self::B),
      2 => Some(Self::C),
      _ => None
    }
  }
}

/// FIRM (reference 6.7): one job fires one dot at each of A, B, and C.
pub const SHOTS_PER_JOB: u8 = 3;

/// Why the coordinator stopped everything.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, thiserror::Error)]
pub enum FaultReason {
  /// The laser fired while the relay was energized or about to be.
  #[error("the laser fired while the motor permit was being given or held")]
  FireWhileArmed,
  /// The Ruida reported a job running while the relay was energized or about to be.
  #[error("the Ruida started a job while the motor permit was being given or held")]
  JobWhileArmed,
  /// A started job did not fire within the timeout (reference 7.3).
  #[error("the job did not fire within the timeout")]
  NoFire,
  /// A job kept running long after its last fire.
  #[error("the job did not end after its last fire")]
  JobDidNotEnd,
  /// A job fired, and the Status line never reported it running: the line is broken or not wired.
  #[error("the job fired but the Status line never reported it running")]
  StatusNeverHigh,
  /// The relay was energized and its feedback did not confirm the laser loop open in time.
  #[error("the relay did not confirm the laser loop open")]
  RelayDidNotOpen,
  /// The relay was released and its feedback did not confirm the laser loop closed in time.
  #[error("the relay did not confirm the laser loop closed")]
  RelayDidNotClose,
  /// The feedback reported the loop closed while the relay was energized and confirmed open.
  #[error("the relay dropped while the motor permit was held")]
  RelayDropped,
  /// A move ran longer than the backstop timeout.
  #[error("a move did not end within the timeout")]
  MoveDidNotEnd,
  /// The time given was earlier than the time before it.
  #[error("the clock went backward")]
  ClockWentBack,
  /// The alignment procedure or a node reported a fault (a stall, a soft limit, bus-off, invalid shots).
  #[error("the alignment procedure or a node reported a fault")]
  Procedure
}

/// The coordinator's state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
  /// After boot, a cleared fault, or a lost node. Waiting for a passed self-test.
  Inhibited,
  /// Ready for a measurement or a move.
  Idle,
  /// The job has been started; waiting for its first fire.
  WaitingForFire {
    /// When the job was started.
    since_ms: u64,
    /// Whether the Ruida has reported the job running yet.
    job_seen: bool
  },
  /// The job is firing its dots.
  Measuring {
    /// Whether the Ruida has reported the job running yet.
    job_seen: bool
  },
  /// The job has ended; the sensor's readings are arriving.
  CollectingReadings {
    /// When the job ended.
    since_ms: u64
  },
  /// Waiting for quiet, then for the relay to confirm the laser loop open.
  ArmingMove {
    /// When the relay was energized, once it has been.
    energized_ms: Option<u64>
  },
  /// The motors may move.
  Moving {
    /// When the motor permit was given.
    since_ms: u64
  },
  /// The nodes have been told to stop and the relay released; waiting for the loop to close.
  Disarming {
    /// When the relay was released.
    since_ms: u64
  },
  /// Everything stopped until the fault is cleared by hand.
  Fault(FaultReason)
}

/// Something that happened.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
  /// The firmware's self-test finished (relay feedback, nodes present and of the right protocol version).
  SelfTest {
    /// Whether it passed.
    passed: bool
  },
  /// The procedure wants a measurement: start the job.
  StartMeasurement,
  /// The laser began to fire: the first edge of a burst. Edges closer than `burst_gap_ms` are one burst, one shot.
  Fire,
  /// The burst ended: no fire edge for `burst_gap_ms`.
  FireEnded,
  /// The Ruida's Status input: a job is running.
  JobRunning(bool),
  /// Every reading of the job has arrived.
  ReadingsComplete,
  /// The procedure wants to move motors.
  StartMove,
  /// Every move has ended (every MOVE_DONE received).
  MovesDone,
  /// The relay's feedback contact: whether the laser enable loop is open.
  RelayFeedback {
    /// The loop is open (the relay is energized and its normally closed contact open).
    loop_open: bool
  },
  /// A node's heartbeat was lost, or a node came back with another protocol version.
  NodeLost,
  /// The procedure or a node reported a fault.
  ProcedureFault,
  /// The operator cleared the fault.
  ClearFault,
  /// Time passed.
  Tick
}

/// The relay drive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Relay {
  /// Energize: the laser loop opens and the motor permit closes.
  Energize,
  /// Release: the laser loop closes and the motor permit opens.
  Release
}

/// What the coordinator reports to the procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
  /// The self-test passed: ready.
  Ready,
  /// The job fired every dot and every reading arrived.
  MeasurementReady,
  /// The job ended with this many fires instead of the expected number: the measurement is invalid.
  JobMismatch {
    /// The fires seen.
    fires: u8
  },
  /// The readings did not all arrive in time: the measurement is invalid.
  ReadingsTimedOut,
  /// A fire came when no job was running, or after the job had ended.
  UnexpectedFire,
  /// The Ruida started a job nobody asked for while the readings were arriving: the measurement is invalid.
  UnexpectedJob,
  /// The request was refused in this state; nothing changed.
  Refused,
  /// The motors may move.
  MoveReady,
  /// The move has ended and the laser loop is closed again.
  MoveFinished,
  /// Back in Inhibited after a lost node or a failed self-test.
  Inhibited,
  /// Everything stopped.
  Faulted(FaultReason)
}

/// What the firmware must do now.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Actions {
  /// Drive the relay.
  pub relay: Option<Relay>,
  /// Tell every node that motion is allowed (`true`, INHIBIT 0) or not (`false`, INHIBIT 1).
  /// A change only: [`Coordinator::motion_allowed`] gives the level, for repeating it.
  pub allow_motion: Option<bool>,
  /// Send ESTOP to every node.
  pub estop: bool,
  /// Start the job (pulse FootSW).
  pub start_job: bool,
  /// Tell the sensor a shot was fired: its sequence number and point.
  pub shot: Option<(u8, Point)>,
  /// Report to the procedure.
  pub outcome: Option<Outcome>
}

/// The coordinator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Coordinator {
  timing: CoordinatorTiming,
  state: State,
  relay_energized: bool,
  released_ms: Option<u64>,
  /// The last feedback reading. `None` before the first, and after each energize until the next.
  loop_open: Option<bool>,
  /// The Status line as last reported; `None` before it has been.
  job_running: Option<bool>,
  /// A burst of fire edges has started and not yet ended.
  firing: bool,
  /// When a burst last started or ended.
  last_fire_ms: Option<u64>,
  fires_in_job: u8,
  /// Every reading of the running job arrived before it ended.
  readings_complete: bool,
  next_seq: u8,
  last_now_ms: u64
}

impl Coordinator {
  /// A coordinator after boot: Inhibited, relay released, no input read yet.
  pub const fn new(timing: CoordinatorTiming) -> Self {
    Self {
      timing,
      state: State::Inhibited,
      relay_energized: false,
      released_ms: None,
      loop_open: None,
      job_running: None,
      firing: false,
      last_fire_ms: None,
      fires_in_job: 0,
      readings_complete: false,
      next_seq: 0,
      last_now_ms: 0
    }
  }

  /// The state.
  pub const fn state(&self) -> State {
    self.state
  }

  /// Whether the relay is energized, as commanded.
  pub const fn relay_energized(&self) -> bool {
    self.relay_energized
  }

  /// Whether the motors may move: the level that [`Actions::allow_motion`] changes, for the firmware to repeat to
  /// the nodes, so that a lost INHIBIT frame is sent again.
  pub const fn motion_allowed(&self) -> bool {
    matches!(self.state, State::Moving { .. })
  }

  /// Handles one event at `now_ms` and returns what to do. Time must not go backward; if it does, that is a fault,
  /// and the new time becomes the reference.
  pub fn handle(&mut self, now_ms: u64, event: Event) -> Actions {
    let mut actions = Actions::default();
    let clock_went_back = now_ms < self.last_now_ms;
    self.last_now_ms = now_ms;
    if clock_went_back && !matches!(self.state, State::Fault(_)) {
      self.fault(now_ms, FaultReason::ClockWentBack, &mut actions);
      return actions;
    }
    if let Some(reason) = self.global_fault(now_ms, event) {
      self.fault(now_ms, reason, &mut actions);
      return actions;
    }
    match event {
      Event::NodeLost if !matches!(self.state, State::Fault(_) | State::Inhibited) => {
        self.inhibit(now_ms, &mut actions);
        return actions;
      }
      Event::ProcedureFault if !matches!(self.state, State::Fault(_)) => {
        self.fault(now_ms, FaultReason::Procedure, &mut actions);
        return actions;
      }
      _ => {}
    }
    self.step(now_ms, event, &mut actions);
    actions
  }

  /// The rules of every state: arming faults and relay feedback. Also records fires, the job line, and feedback.
  fn global_fault(&mut self, now_ms: u64, event: Event) -> Option<FaultReason> {
    // A loop that still reads open after a release is as good as armed: the laser cannot fire through it.
    let armed = matches!(self.state, State::ArmingMove { .. } | State::Moving { .. })
      || self.relay_energized
      || self.loop_open == Some(true);
    match event {
      Event::Fire | Event::FireEnded => {
        self.firing = event == Event::Fire;
        self.last_fire_ms = Some(now_ms);
        if armed {
          return Some(FaultReason::FireWhileArmed);
        }
      }
      Event::JobRunning(running) => {
        self.job_running = Some(running);
        if running && armed {
          return Some(FaultReason::JobWhileArmed);
        }
      }
      Event::RelayFeedback { loop_open } => self.loop_open = Some(loop_open),
      _ => {}
    }
    if matches!(self.state, State::Fault(_)) {
      return None;
    }
    if matches!(self.state, State::Moving { .. }) && self.loop_open != Some(true) {
      return Some(FaultReason::RelayDropped);
    }
    // After a release the loop has the feedback timeout to close; Disarming judges its own release.
    let settled = self
      .released_ms
      .is_none_or(|released| now_ms.saturating_sub(released) > u64::from(self.timing.relay_feedback_timeout_ms));
    let open_released = !self.relay_energized && self.loop_open == Some(true);
    if open_released && settled && !matches!(self.state, State::Disarming { .. }) {
      return Some(FaultReason::RelayDidNotClose);
    }
    None
  }

  fn step(&mut self, now_ms: u64, event: Event, actions: &mut Actions) {
    let handled = match self.state {
      State::Inhibited | State::Idle | State::Fault(_) => self.step_rest(now_ms, event, actions),
      State::WaitingForFire { .. } | State::Measuring { .. } | State::CollectingReadings { .. } => {
        self.step_job(now_ms, event, actions)
      }
      State::ArmingMove { .. } | State::Moving { .. } | State::Disarming { .. } => {
        self.step_move(now_ms, event, actions)
      }
    };
    if !handled && matches!(event, Event::StartMeasurement | Event::StartMove) {
      actions.outcome = Some(Outcome::Refused);
    }
  }

  /// Whether a job may start or the relay may be armed:
  /// the loop reads closed, the Status line reads low, and no burst is in progress.
  fn ready_to_start(&self) -> bool {
    self.loop_open == Some(false) && self.job_running == Some(false) && !self.firing
  }

  /// `Inhibited`, `Idle`, and `Fault`. Returns whether the event was handled.
  fn step_rest(&mut self, now_ms: u64, event: Event, actions: &mut Actions) -> bool {
    match (self.state, event) {
      (State::Inhibited, Event::SelfTest { passed: true }) => {
        // Only on inputs that have been read: the loop closed, and the Status line in either state.
        if self.loop_open == Some(false) && self.job_running.is_some() {
          self.state = State::Idle;
          actions.outcome = Some(Outcome::Ready);
        } else {
          actions.outcome = Some(Outcome::Refused);
        }
      }
      (State::Idle, Event::SelfTest { passed: false }) => self.inhibit(now_ms, actions),
      (State::Idle, Event::StartMeasurement) => {
        if self.ready_to_start() {
          self.fires_in_job = 0;
          self.readings_complete = false;
          self.state = State::WaitingForFire {
            since_ms: now_ms,
            job_seen: false
          };
          actions.start_job = true;
        } else {
          actions.outcome = Some(Outcome::Refused);
        }
      }
      (State::Idle, Event::StartMove) => {
        if self.ready_to_start() {
          self.state = State::ArmingMove { energized_ms: None };
          self.arm(now_ms, actions);
        } else {
          actions.outcome = Some(Outcome::Refused);
        }
      }
      (State::Inhibited | State::Idle, Event::Fire) => actions.outcome = Some(Outcome::UnexpectedFire),
      (State::Fault(_), Event::ClearFault) => {
        self.state = State::Inhibited;
        actions.outcome = Some(Outcome::Inhibited);
      }
      _ => return false
    }
    true
  }

  /// `WaitingForFire`, `Measuring`, and `CollectingReadings`. Returns whether the event was handled.
  fn step_job(&mut self, now_ms: u64, event: Event, actions: &mut Actions) -> bool {
    let timing = self.timing;
    match (self.state, event) {
      (State::WaitingForFire { job_seen, .. } | State::Measuring { job_seen }, Event::Fire) => {
        self.state = State::Measuring { job_seen };
        let index = self.fires_in_job;
        self.fires_in_job = self.fires_in_job.saturating_add(1);
        if let Some(point) = Point::from_index(index) {
          actions.shot = Some((self.next_seq, point));
          self.next_seq = self.next_seq.wrapping_add(1);
        }
      }
      (State::WaitingForFire { since_ms, .. }, Event::JobRunning(true)) => {
        self.state = State::WaitingForFire {
          since_ms,
          job_seen: true
        };
      }
      (State::WaitingForFire { job_seen: true, .. }, Event::JobRunning(false)) => {
        self.state = State::Idle;
        actions.outcome = Some(Outcome::JobMismatch { fires: 0 });
      }
      (State::WaitingForFire { since_ms, .. }, Event::Tick) => {
        if elapsed(now_ms, since_ms) >= timing.fire_wait_timeout_ms {
          self.fault(now_ms, FaultReason::NoFire, actions);
        }
      }
      (State::Measuring { .. }, Event::JobRunning(true)) => self.state = State::Measuring { job_seen: true },
      // A low read before the line rose is stale; the job ends on a low after a high.
      (State::Measuring { job_seen: true }, Event::JobRunning(false)) => self.end_job(now_ms, actions),
      (State::Measuring { job_seen }, Event::Tick) => {
        let last_fire = self.last_fire_ms.unwrap_or(now_ms);
        if elapsed(now_ms, last_fire) >= timing.fire_wait_timeout_ms {
          let reason = if job_seen {
            FaultReason::JobDidNotEnd
          } else {
            FaultReason::StatusNeverHigh
          };
          self.fault(now_ms, reason, actions);
        }
      }
      // The sensor answers each shot within a few hundred milliseconds, often before the job ends.
      (State::Measuring { .. }, Event::ReadingsComplete) => self.readings_complete = true,
      (State::CollectingReadings { .. }, Event::ReadingsComplete) => {
        self.state = State::Idle;
        actions.outcome = Some(Outcome::MeasurementReady);
      }
      (State::CollectingReadings { since_ms }, Event::Tick) => {
        if elapsed(now_ms, since_ms) >= timing.reading_timeout_ms {
          self.state = State::Idle;
          actions.outcome = Some(Outcome::ReadingsTimedOut);
        }
      }
      (State::CollectingReadings { .. }, Event::Fire) => {
        self.state = State::Idle;
        actions.outcome = Some(Outcome::UnexpectedFire);
      }
      (State::CollectingReadings { .. }, Event::JobRunning(true)) => {
        self.state = State::Idle;
        actions.outcome = Some(Outcome::UnexpectedJob);
      }
      _ => return false
    }
    true
  }

  /// The job's Status line fell after rising: the measurement is ready, waits for its readings, or is invalid.
  fn end_job(&mut self, now_ms: u64, actions: &mut Actions) {
    if self.fires_in_job != SHOTS_PER_JOB {
      self.state = State::Idle;
      actions.outcome = Some(Outcome::JobMismatch {
        fires: self.fires_in_job
      });
    } else if self.readings_complete {
      self.state = State::Idle;
      actions.outcome = Some(Outcome::MeasurementReady);
    } else {
      self.state = State::CollectingReadings { since_ms: now_ms };
    }
  }

  /// `ArmingMove`, `Moving`, and `Disarming`. Returns whether the event was handled.
  fn step_move(&mut self, now_ms: u64, event: Event, actions: &mut Actions) -> bool {
    match (self.state, event) {
      (State::ArmingMove { .. }, Event::Tick | Event::RelayFeedback { .. }) => self.arm(now_ms, actions),
      (State::Moving { .. }, Event::MovesDone) => {
        actions.allow_motion = Some(false);
        self.release(now_ms, actions);
        self.state = State::Disarming { since_ms: now_ms };
      }
      (State::Moving { since_ms }, Event::Tick) => {
        if elapsed(now_ms, since_ms) >= self.timing.move_timeout_ms {
          self.fault(now_ms, FaultReason::MoveDidNotEnd, actions);
        }
      }
      (State::Disarming { since_ms }, Event::Tick | Event::RelayFeedback { .. }) => {
        if self.loop_open == Some(false) {
          self.state = State::Idle;
          actions.outcome = Some(Outcome::MoveFinished);
        } else if elapsed(now_ms, since_ms) > self.timing.relay_feedback_timeout_ms {
          self.fault(now_ms, FaultReason::RelayDidNotClose, actions);
        }
      }
      _ => return false
    }
    true
  }

  /// In `ArmingMove`: energize once quiet, then wait for a feedback reading taken after the energize.
  fn arm(&mut self, now_ms: u64, actions: &mut Actions) {
    let State::ArmingMove { energized_ms } = self.state else {
      return;
    };
    let timing = self.timing;
    match energized_ms {
      None => {
        let quiet = self
          .last_fire_ms
          .is_none_or(|fire| elapsed(now_ms, fire) >= timing.quiet_before_move_ms);
        // A fire or a job start here has already faulted, so only the quiet time and the loop are waited for.
        if quiet && self.ready_to_start() {
          self.relay_energized = true;
          self.released_ms = None;
          self.loop_open = None;
          actions.relay = Some(Relay::Energize);
          self.state = State::ArmingMove {
            energized_ms: Some(now_ms)
          };
        }
      }
      Some(energized) => {
        // A reading stamped with the energize's own millisecond was sampled before it: the firmware samples its
        // inputs at the top of the millisecond and may energize later in it (review of 2026-11-17).
        if self.loop_open == Some(true) && now_ms > energized {
          self.state = State::Moving { since_ms: now_ms };
          actions.allow_motion = Some(true);
          actions.outcome = Some(Outcome::MoveReady);
        } else if elapsed(now_ms, energized) > timing.relay_feedback_timeout_ms {
          self.fault(now_ms, FaultReason::RelayDidNotOpen, actions);
        }
      }
    }
  }

  /// Back to `Inhibited`: the relay released and motion disallowed.
  fn inhibit(&mut self, now_ms: u64, actions: &mut Actions) {
    self.release(now_ms, actions);
    actions.allow_motion = Some(false);
    self.state = State::Inhibited;
    actions.outcome = Some(Outcome::Inhibited);
  }

  fn release(&mut self, now_ms: u64, actions: &mut Actions) {
    if self.relay_energized {
      self.relay_energized = false;
      self.released_ms = Some(now_ms);
      actions.relay = Some(Relay::Release);
    }
  }

  fn fault(&mut self, now_ms: u64, reason: FaultReason, actions: &mut Actions) {
    self.release(now_ms, actions);
    actions.allow_motion = Some(false);
    actions.estop = true;
    actions.outcome = Some(Outcome::Faulted(reason));
    self.state = State::Fault(reason);
  }
}

/// Milliseconds from `since` to `now`, saturating into `u32`.
fn elapsed(now_ms: u64, since_ms: u64) -> u32 {
  u32::try_from(now_ms.saturating_sub(since_ms)).unwrap_or(u32::MAX)
}

#[cfg(test)]
mod tests;
