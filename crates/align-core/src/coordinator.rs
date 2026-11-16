//! The coordinator state machine (reference 7.3), a pure function of state and event (hand-off section 5).
//!
//! It decides one thing: when the force-guided relay may open the laser's enable loop and the nodes may move their
//! motors. Everything else (talking to the nodes, pairing sensor readings, the alignment procedure) asks it.
//! The firmware feeds it the Ruida's signals and the relay's feedback, and carries out the [`Actions`] it returns.
//!
//! The events are named for what happens, not for the Ruida's pins, so that a test frame without a Ruida can drive
//! the same machine: a start becomes "start the job", a fire edge becomes [`Event::Fire`], and the Status line
//! becomes [`Event::JobRunning`].
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
//! - A fire, or the Ruida reporting a job running, while the relay is energized or about to be, is an immediate
//!   fault (reference 7.3: "Any fire edge in Moving is an immediate Fault"). The fault releases the relay at once.
//! - The relay's feedback is watched all the time, not only when entering Moving. The loop reading open while the
//!   relay is released (after its release time), or closed while it is energized and confirmed, is a fault: a
//!   welded contact, a dropped relay, or a wiring fault.
//! - A lost node puts the coordinator back in Inhibited, releasing the relay.

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
  /// The relay was energized and its feedback did not confirm the laser loop open in time.
  #[error("the relay did not confirm the laser loop open")]
  RelayDidNotOpen,
  /// The relay was released and its feedback did not confirm the laser loop closed in time.
  #[error("the relay did not confirm the laser loop closed")]
  RelayDidNotClose,
  /// The feedback reported the loop closed while the relay was energized and confirmed open.
  #[error("the relay dropped while the motor permit was held")]
  RelayDropped,
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
  Measuring,
  /// The job has ended; the sensor's readings are arriving.
  CollectingReadings {
    /// When the job ended.
    since_ms: u64
  },
  /// Waiting for quiet, then for the relay to confirm the laser loop open.
  ArmingMove {
    /// When the move was asked for.
    since_ms: u64,
    /// When the relay was energized, once it has been.
    energized_ms: Option<u64>
  },
  /// The motors may move.
  Moving,
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
  /// The laser fired once (one burst of fire edges).
  Fire,
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
  /// The request was refused in this state; nothing changed.
  Refused,
  /// The motors may move.
  MoveReady,
  /// The move has ended and the laser loop is closed again.
  MoveFinished,
  /// Back in Inhibited after a lost node.
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
  loop_open: bool,
  job_running: bool,
  last_fire_ms: Option<u64>,
  fires_in_job: u8,
  next_seq: u8
}

impl Coordinator {
  /// A coordinator after boot: Inhibited, relay released.
  pub const fn new(timing: CoordinatorTiming) -> Self {
    Self {
      timing,
      state: State::Inhibited,
      relay_energized: false,
      released_ms: None,
      loop_open: false,
      job_running: false,
      last_fire_ms: None,
      fires_in_job: 0,
      next_seq: 0
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

  /// Handles one event at `now_ms` (non-decreasing) and returns what to do.
  pub fn handle(&mut self, now_ms: u64, event: Event) -> Actions {
    let mut actions = Actions::default();
    if let Some(reason) = self.global_fault(now_ms, event) {
      self.fault(now_ms, reason, &mut actions);
      return actions;
    }
    match event {
      Event::NodeLost if !matches!(self.state, State::Fault(_) | State::Inhibited) => {
        self.release(now_ms, &mut actions);
        actions.allow_motion = Some(false);
        self.state = State::Inhibited;
        actions.outcome = Some(Outcome::Inhibited);
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
    let armed = matches!(self.state, State::ArmingMove { .. } | State::Moving) || self.relay_energized;
    match event {
      Event::Fire => {
        self.last_fire_ms = Some(now_ms);
        if armed {
          return Some(FaultReason::FireWhileArmed);
        }
      }
      Event::JobRunning(running) => {
        self.job_running = running;
        if running && armed {
          return Some(FaultReason::JobWhileArmed);
        }
      }
      Event::RelayFeedback { loop_open } => self.loop_open = loop_open,
      _ => {}
    }
    if matches!(self.state, State::Fault(_)) {
      return None;
    }
    if self.state == State::Moving && !self.loop_open {
      return Some(FaultReason::RelayDropped);
    }
    // After a release the loop has the feedback timeout to close; Disarming judges its own release.
    let settled = self
      .released_ms
      .is_none_or(|released| now_ms.saturating_sub(released) > u64::from(self.timing.relay_feedback_timeout_ms));
    if !self.relay_energized && self.loop_open && settled && !matches!(self.state, State::Disarming { .. }) {
      return Some(FaultReason::RelayDidNotClose);
    }
    None
  }

  fn step(&mut self, now_ms: u64, event: Event, actions: &mut Actions) {
    let handled = match self.state {
      State::Inhibited | State::Idle | State::Fault(_) => self.step_rest(now_ms, event, actions),
      State::WaitingForFire { .. } | State::Measuring | State::CollectingReadings { .. } => {
        self.step_job(now_ms, event, actions)
      }
      State::ArmingMove { .. } | State::Moving | State::Disarming { .. } => self.step_move(now_ms, event, actions)
    };
    if !handled && matches!(event, Event::StartMeasurement | Event::StartMove) {
      actions.outcome = Some(Outcome::Refused);
    }
  }

  /// `Inhibited`, `Idle`, and `Fault`. Returns whether the event was handled.
  fn step_rest(&mut self, now_ms: u64, event: Event, actions: &mut Actions) -> bool {
    match (self.state, event) {
      (State::Inhibited, Event::SelfTest { passed: true }) => {
        self.state = State::Idle;
        actions.outcome = Some(Outcome::Ready);
      }
      (State::Idle, Event::StartMeasurement) => {
        if self.job_running {
          actions.outcome = Some(Outcome::Refused);
        } else {
          self.fires_in_job = 0;
          self.state = State::WaitingForFire {
            since_ms: now_ms,
            job_seen: false
          };
          actions.start_job = true;
        }
      }
      (State::Idle, Event::StartMove) => {
        if self.job_running {
          actions.outcome = Some(Outcome::Refused);
        } else {
          self.state = State::ArmingMove {
            since_ms: now_ms,
            energized_ms: None
          };
          self.arm(now_ms, actions);
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
      (State::WaitingForFire { .. } | State::Measuring, Event::Fire) => {
        self.state = State::Measuring;
        let index = self.fires_in_job;
        self.fires_in_job = self.fires_in_job.saturating_add(1);
        if let Some(point) = Point::from_index(index).filter(|_| index < timing.shots_per_job) {
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
      (State::Measuring, Event::JobRunning(false)) => {
        if self.fires_in_job == timing.shots_per_job {
          self.state = State::CollectingReadings { since_ms: now_ms };
        } else {
          self.state = State::Idle;
          actions.outcome = Some(Outcome::JobMismatch {
            fires: self.fires_in_job
          });
        }
      }
      (State::Measuring, Event::Tick) => {
        let last_fire = self.last_fire_ms.unwrap_or(now_ms);
        if elapsed(now_ms, last_fire) >= timing.fire_wait_timeout_ms {
          self.fault(now_ms, FaultReason::JobDidNotEnd, actions);
        }
      }
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
      _ => return false
    }
    true
  }

  /// `ArmingMove`, `Moving`, and `Disarming`. Returns whether the event was handled.
  fn step_move(&mut self, now_ms: u64, event: Event, actions: &mut Actions) -> bool {
    match (self.state, event) {
      (State::ArmingMove { .. }, Event::Tick | Event::RelayFeedback { .. }) => self.arm(now_ms, actions),
      (State::Moving, Event::MovesDone) => {
        actions.allow_motion = Some(false);
        self.release(now_ms, actions);
        self.state = State::Disarming { since_ms: now_ms };
      }
      (State::Disarming { since_ms }, Event::Tick | Event::RelayFeedback { .. }) => {
        if !self.loop_open {
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

  /// In `ArmingMove`: energize once quiet, then wait for the feedback.
  fn arm(&mut self, now_ms: u64, actions: &mut Actions) {
    let State::ArmingMove { since_ms, energized_ms } = self.state else {
      return;
    };
    let timing = self.timing;
    match energized_ms {
      None => {
        let quiet = self
          .last_fire_ms
          .is_none_or(|fire| elapsed(now_ms, fire) >= timing.quiet_before_move_ms);
        if quiet && !self.job_running {
          self.relay_energized = true;
          self.released_ms = None;
          actions.relay = Some(Relay::Energize);
          self.state = State::ArmingMove {
            since_ms,
            energized_ms: Some(now_ms)
          };
        }
      }
      Some(energized) => {
        if self.loop_open {
          self.state = State::Moving;
          actions.allow_motion = Some(true);
          actions.outcome = Some(Outcome::MoveReady);
        } else if elapsed(now_ms, energized) > timing.relay_feedback_timeout_ms {
          self.fault(now_ms, FaultReason::RelayDidNotOpen, actions);
        }
      }
    }
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
