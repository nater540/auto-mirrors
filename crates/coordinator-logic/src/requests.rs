//! The alignment procedure's requests to the orchestrator, one at a time: a move or a measurement.
//!
//! The aligner (`align_core::aligner::Aligner`) asks through its `Actuator` and `BeamSensor` traits and waits for the
//! answer. The firmware's control loop owns the orchestrator, so the request crosses over: [`Requests::submit`]
//! takes it, [`Requests::poll`] starts it when the coordinator can take it, and [`Requests::on_report`] finds its end
//! among the orchestrator's reports. `align-sim`'s whole-system simulation runs the same code.
//!
//! - **A request starts in Idle.** After a move the coordinator is Disarming for the relay's release time; the next
//!   request waits for that, and for anything else that is not Idle, up to the wait limit.
//! - **A measurement starts only once the last move has settled**: `settle_ms` after the move ended, which is when
//!   its coils released (`align_core::config::Motion::settle_ms`; nothing else waits for it).
//! - **A request that cannot start fails at once**, as the aligner knows a failure: a move as
//!   `MoveFault::Refused` with code [`COORDINATOR_REFUSED`] (nothing moved), a measurement as
//!   `SensorFault::NoReading`. So does one still waiting at the limit, as when the coordinator is faulted.
//! - **A started request always ends in a report**: the orchestrator ends a move on the node's answer, a refusal, or
//!   none after its tries, and a measurement on every way out of the job.
//! - **The console's `stop` ends the procedure** ([`Requests::stop`], review of 2026-11-19): the request waiting to
//!   start fails at once, and every later one fails before it starts until [`Requests::resume`], which the firmware
//!   calls once the procedure has ended. A running move is stopped by the orchestrator's STOP; a running measurement
//!   ends with its job, which the Ruida runs to its end.
//!
//! [`starting_axes`] gives the procedure the nodes' counts before it runs: the aligner's own state of an axis where
//! the node's count agrees, the node's where it does not, and lost where the node has not reported. An axis the
//! console moved since the last procedure has its last direction unknown, whatever its count.

use align_core::aligner::AxisState;
use align_core::approach::LastDir;
use align_core::axis::{Axis, PerAxis};
use align_core::coordinator::State;
use align_core::measurement::Shots;
use align_core::rig::{MoveFault, SensorFault};
use align_proto::StatusFlags;

use crate::link::Link;
use crate::orchestrator::{CoordinatorIo, Orchestrator, Report, Wire};

/// The code of a move the coordinator itself refused before sending it. No node sends 0: node codes start at 0x10.
pub const COORDINATOR_REFUSED: u8 = 0;

/// A request of the procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
  /// Move one axis.
  Move {
    /// The axis.
    axis: Axis,
    /// Steps, signed; plus advances the screw.
    steps: i32
  },
  /// Run the canned job and read the sensor.
  Measure
}

/// How a request ended, as the aligner's traits return it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Completion {
  /// The move's answer: the node's position, or why not.
  Move(Result<i32, MoveFault>),
  /// The measurement's three readings, or why not.
  Measure(Result<Shots, SensorFault>)
}

/// A request was submitted while another had not completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("a request is already waiting or running")]
pub struct Busy;

/// What is running.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Running {
  Move(Axis),
  Measure
}

/// The one request of the procedure, from its submission to its completion.
#[derive(Clone, Copy, Debug)]
pub struct Requests {
  settle_ms: u64,
  wait_limit_ms: u64,
  /// A request not yet started, and when it was submitted.
  queued: Option<(Request, u64)>,
  running: Option<Running>,
  /// No measurement starts before this time: the last move's end plus the settle time.
  settled_ms: u64,
  /// The operator stopped the procedure: requests fail before they start.
  stopped: bool
}

impl Requests {
  /// Nothing submitted. `settle_ms` is the wait after a move before a measurement; `wait_limit_ms` how long a
  /// request may wait for the coordinator to take it.
  pub const fn new(settle_ms: u32, wait_limit_ms: u32) -> Self {
    Self {
      settle_ms: settle_ms as u64,
      wait_limit_ms: wait_limit_ms as u64,
      queued: None,
      running: None,
      settled_ms: 0,
      stopped: false
    }
  }

  /// Stops the procedure: the request waiting to start fails at once, its completion returned, and every later one
  /// fails before it starts until [`Self::resume`]. A running request ends by its report.
  pub fn stop(&mut self) -> Option<Completion> {
    self.stopped = true;
    self.queued.take().map(|(request, _)| failure(request))
  }

  /// Takes requests again, once the stopped procedure has ended.
  pub const fn resume(&mut self) {
    self.stopped = false;
  }

  /// Whether the procedure was stopped and has not been resumed.
  pub const fn stopped(&self) -> bool {
    self.stopped
  }

  /// Whether a request is waiting or running.
  pub const fn busy(&self) -> bool {
    self.queued.is_some() || self.running.is_some()
  }

  /// Takes a request, to start when the coordinator can.
  ///
  /// # Errors
  ///
  /// [`Busy`] while another request has not completed.
  pub fn submit(&mut self, now_ms: u64, request: Request) -> Result<(), Busy> {
    if self.busy() {
      return Err(Busy);
    }
    self.queued = Some((request, now_ms));
    Ok(())
  }

  /// Starts the waiting request if the coordinator can take it now. Returns the completion of one that cannot be
  /// started: refused, or still waiting at the limit.
  pub fn poll(
    &mut self,
    now_ms: u64,
    orchestrator: &mut Orchestrator,
    io: &mut impl CoordinatorIo
  ) -> Option<Completion> {
    let (request, submitted_ms) = self.queued?;
    if self.stopped {
      self.queued = None;
      return Some(failure(request));
    }
    let can_start =
      orchestrator.state() == State::Idle && (matches!(request, Request::Move { .. }) || now_ms >= self.settled_ms);
    if !can_start {
      if now_ms.saturating_sub(submitted_ms) >= self.wait_limit_ms {
        self.queued = None;
        return Some(failure(request));
      }
      return None;
    }
    self.queued = None;
    let started = match request {
      Request::Move { axis, steps } => orchestrator
        .request_move(now_ms, axis, steps, io)
        .map(|()| Running::Move(axis)),
      Request::Measure => orchestrator.request_measurement(now_ms, io).map(|()| Running::Measure)
    };
    match started {
      Ok(running) => {
        self.running = Some(running);
        None
      }
      Err(_) => Some(failure(request))
    }
  }

  /// A report of the orchestrator, at the local time `now_ms`. Returns the completion of the running request if
  /// this report ends it.
  pub fn on_report(&mut self, now_ms: u64, report: &Report) -> Option<Completion> {
    let completion = match (self.running?, *report) {
      (Running::Move(running), Report::MoveEnded { axis, result }) if axis == running => {
        self.settled_ms = now_ms.saturating_add(self.settle_ms);
        Completion::Move(result)
      }
      // Never sent, so nothing moved.
      (Running::Move(running), Report::MoveDropped { axis }) if axis == running => {
        Completion::Move(Err(MoveFault::Refused {
          code: COORDINATOR_REFUSED
        }))
      }
      (Running::Measure, Report::MeasurementEnded { result }) => Completion::Measure(result),
      _ => return None
    };
    self.running = None;
    Some(completion)
  }
}

/// What a node last reported of one axis: its count, and whether it holds it suspect.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reported {
  /// The node's count, in steps from the commissioning position.
  pub pos_steps: i32,
  /// The node's STATUS carries its SUSPECT flag.
  pub suspect: bool
}

/// What the nodes last reported of each axis, through `wiring`; `None` where the node is not present or has not
/// reported the channel. The control loop takes this from its link and hands it to the procedure.
pub fn reported(link: &Link, wiring: &PerAxis<Wire>) -> PerAxis<Option<Reported>> {
  let of = |axis: Axis| {
    let wire = wiring.get(axis);
    let status = link
      .heard(wire.node)
      .filter(|_| link.present(wire.node))
      .and_then(|heard| heard.status.get(usize::from(wire.channel.get())).copied().flatten())?;
    Some(Reported {
      pos_steps: status.pos_steps,
      suspect: status.flags.contains(StatusFlags::SUSPECT)
    })
  };
  PerAxis {
    m1_h: of(Axis::M1H),
    m1_v: of(Axis::M1V),
    m2_h: of(Axis::M2H),
    m2_v: of(Axis::M2V)
  }
}

/// What the procedure starts from on each axis, given what it knew (`current`), what the nodes last reported, and
/// which axes the console moved since the last procedure (`jogged`).
///
/// - **The node's count agrees:** the aligner's state is kept, its last direction with it, and the node's suspect
///   flag added. Where the console moved the axis, the last direction is unknown: jogs that sum to zero leave the
///   count where it was, but not the direction the gear play was last taken up in (review of 2026-11-19).
/// - **It differs**, or the aligner had lost the axis: the node's count is taken, as `Aligner::restore_position`
///   takes it, with the last direction unknown so that the next move takes up the backlash in full.
/// - **The node is not present, or has not reported the channel:** the axis is lost and does not move.
pub fn starting_axes(
  current: &PerAxis<AxisState>,
  reported: &PerAxis<Option<Reported>>,
  jogged: &PerAxis<bool>
) -> PerAxis<AxisState> {
  let mut axes = *current;
  for axis in Axis::ALL {
    let state = axes.get_mut(axis);
    match reported.get(axis) {
      None => state.lost = true,
      Some(node) => {
        if state.lost || node.pos_steps != state.pos_steps {
          *state = AxisState {
            pos_steps: node.pos_steps,
            last_dir: LastDir::Unknown,
            suspect: node.suspect || state.lost,
            lost: false
          };
        } else {
          state.suspect |= node.suspect;
          if jogged.get(axis) {
            state.last_dir = LastDir::Unknown;
          }
        }
      }
    }
  }
  axes
}

/// How a request that never started ends.
const fn failure(request: Request) -> Completion {
  match request {
    Request::Move { .. } => Completion::Move(Err(MoveFault::Refused {
      code: COORDINATOR_REFUSED
    })),
    Request::Measure => Completion::Measure(Err(SensorFault::NoReading))
  }
}

#[cfg(test)]
mod tests;
