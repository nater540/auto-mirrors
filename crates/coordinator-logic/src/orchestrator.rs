//! The coordinator as a whole, apart from its hardware: the state machine (`align_core::coordinator`), the link to
//! the nodes ([`crate::link`]), and the move in flight ([`crate::moves`]), driven by sampled inputs, frames, and
//! requests.
//!
//! The firmware samples its inputs every millisecond and calls [`Orchestrator::tick`], passes every decoded frame to
//! [`Orchestrator::on_frame`], and carries out what comes back through [`CoordinatorIo`]: frames to send, the relay
//! drive, and reports to log.
//!
//! A move (the console's `jog` now, the aligner's moves later) runs in the order of reference 7.3:
//!
//! 1. **Asked for** only in Idle, with the axis's node present: the state machine is told `StartMove`.
//! 2. **Arming:** after the quiet time, the relay is energized. Motion waits for a feedback reading taken after.
//! 3. **Moving:** the inhibit level goes out as INHIBIT 0 and then the MOVE_REL, in that order on the one transmit
//!    queue, so that the node is allowed to move before the move reaches it.
//! 4. **Ended** by the node's MOVE_DONE or refusal, or by no answer: `MovesDone`, the relay released, INHIBIT 1.
//! 5. **Disarming** until the loop reads closed again, then Idle.
//!
//! Rules around that order:
//!
//! - A move asked for but not yet sent is dropped when the state machine leaves arming any other way (a fault, a
//!   lost node). Moving with nothing to send (a stop while arming) disarms at once.
//! - A MOVE_REL is sent again only while Moving. Outside it, a move whose answer is late is given up as unanswered.
//! - A node that appears or restarts is sent the soft limits and the direction sign of each of its axes, which it
//!   keeps in RAM only (CLAUDE.md, M4). A node that restarts during a move has forgotten it: the move is unanswered.
//! - Only the nodes in [`Setup::expected`] are needed by the self-test, and only their loss inhibits.
//!
//! A measurement (the console's `measure` now, the aligner's later) runs the canned job (reference 6.7):
//!
//! 1. **Asked for** only in Idle, with the sensor node present: the state machine is told `StartMeasurement`, the
//!    readings of any earlier job are forgotten, and FootSW is held active for `start_pulse_ms`.
//! 2. **Each burst** of fire edges is a shot: the state machine numbers it and names its point, and SHOT goes to the
//!    sensor node. A reading still missing `resend_after_ms` after its shot is asked for again with SENSOR_RESEND.
//! 3. **Ended** by the state machine: the three readings when the job fired three dots and every reading arrived,
//!    `SensorFault::Rejected` when the sensor rejected one, and `SensorFault::NoReading` for every other way out
//!    (a dot missing, readings timed out, an unexpected job or fire, a fault, a lost node).
//!
//! The SHOT carries no edge age: a 1 ms sample places the edge no closer than that. An edge interrupt can, once the
//! Mod5 scope check settles what a dot looks like (`docs/SENSOR_PLAN.md` section 14).

use align_core::approach::SoftLimits;
use align_core::axis::{Axis, PerAxis};
use align_core::config::Config;
use align_core::coordinator::{Actions, Coordinator, Event, Outcome, Point, Relay, State};
use align_core::measurement::Shots;
use align_core::rig::{MoveFault, SensorFault};
use align_proto::{Channel, ConfigKey, Envelope, Message, NodeId, Shot, ShotPoint};

use crate::link::{Beacon, Link};
use crate::moves::{Moves, Progress};
use crate::nodes::Change;
use crate::readings::Readings;

/// Where an axis's motor is wired: a node and a channel on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Wire {
  /// The motor node.
  pub node: NodeId,
  /// The channel on that node.
  pub channel: Channel
}

/// A channel number known to be valid, for the constant below.
const fn channel(value: u8) -> Channel {
  match Channel::new(value) {
    Some(channel) => channel,
    None => Channel::ZERO
  }
}

const _: () = assert!(Channel::new(1).is_some(), "channel 1 exists");

/// PROVISIONAL: each mirror's node drives its horizontal screw on channel 0 and its vertical screw on channel 1.
/// Which channel turns which screw is wiring (CLAUDE.md, M3); change this table, not the code, when it differs.
pub const WIRING: PerAxis<Wire> = PerAxis {
  m1_h: Wire {
    node: NodeId::MIRROR_1,
    channel: channel(0)
  },
  m1_v: Wire {
    node: NodeId::MIRROR_1,
    channel: channel(1)
  },
  m2_h: Wire {
    node: NodeId::MIRROR_2,
    channel: channel(0)
  },
  m2_v: Wire {
    node: NodeId::MIRROR_2,
    channel: channel(1)
  }
};

/// What the orchestrator needs to know.
#[derive(Clone, Copy, Debug)]
pub struct Setup {
  /// The configuration, from which the nodes' soft limits and signs and every timeout come.
  pub config: Config,
  /// This firmware's version, for the heartbeat.
  pub fw_version: u8,
  /// The nodes the self-test needs present. Their loss inhibits; others are watched and reported only.
  pub expected: &'static [NodeId],
  /// Where each axis's motor is wired.
  pub wiring: PerAxis<Wire>
}

/// The inputs as sampled, each in its logical sense: the firmware applies the wiring's polarity.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Inputs {
  /// The relay's feedback: the laser enable loop is open.
  pub loop_open: bool,
  /// The Ruida's Status line: a job is running.
  pub job_running: bool,
  /// The fire sense: the laser is firing.
  pub firing: bool
}

/// Something worth logging.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Report {
  /// The state machine is in a new state.
  State(State),
  /// The state machine reported an outcome.
  Outcome(Outcome),
  /// What changed about a node.
  Node {
    /// The node.
    node: NodeId,
    /// What changed.
    change: Change
  },
  /// A node's axes were sent their soft limits and direction signs.
  Configured {
    /// The node.
    node: NodeId
  },
  /// A move went out.
  MoveSent {
    /// The axis.
    axis: Axis,
    /// The steps asked for.
    steps: i32
  },
  /// A move ended: the position the node reports, or why not.
  MoveEnded {
    /// The axis.
    axis: Axis,
    /// The position, or the fault.
    result: Result<i32, MoveFault>
  },
  /// A move asked for was given up before it went out: the motors were never allowed to move for it.
  MoveDropped {
    /// The axis.
    axis: Axis
  },
  /// A measurement ended: the three readings, or why there are none (see the module documentation).
  MeasurementEnded {
    /// The readings, or the fault.
    result: Result<Shots, SensorFault>
  }
}

/// What the firmware does for the orchestrator.
pub trait CoordinatorIo {
  /// Queues a frame for the bus.
  fn send(&mut self, envelope: Envelope);
  /// Drives the relay: energized (`true`, the laser loop open, the motor permit given) or released.
  fn set_relay(&mut self, energized: bool);
  /// Drives FootSW: active (`true`, the Ruida's input pulled low, which starts the job) or not.
  fn set_footsw(&mut self, active: bool);
  /// Logs something that happened.
  fn report(&mut self, report: Report);
}

/// Why a move or a measurement was not started.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RequestError {
  /// A move or a measurement is already asked for or running.
  #[error("a move or a measurement is already asked for or running")]
  Busy,
  /// The coordinator is not in Idle.
  #[error("the coordinator is in {state:?}, not Idle")]
  NotIdle {
    /// The state it is in.
    state: State
  },
  /// The node it needs (the axis's, or the sensor) is not on the bus.
  #[error("node {node} is not present")]
  NodeAbsent {
    /// The node.
    node: u8
  },
  /// The state machine refused: the loop does not read closed, a job is running, or a burst is in progress.
  #[error("the interlock is not ready: the loop reads open, a job is running, or the laser is firing")]
  Refused
}

/// The SHOT point of a measurement point.
const fn shot_point(point: Point) -> ShotPoint {
  match point {
    Point::A => ShotPoint::A,
    Point::B => ShotPoint::B,
    Point::C => ShotPoint::C
  }
}

/// The heartbeat's state byte: the state's place in [`State`], from 0 (Inhibited) to 8 (Fault).
pub const fn state_code(state: State) -> u8 {
  match state {
    State::Inhibited => 0,
    State::Idle => 1,
    State::WaitingForFire { .. } => 2,
    State::Measuring { .. } => 3,
    State::CollectingReadings { .. } => 4,
    State::ArmingMove { .. } => 5,
    State::Moving { .. } => 6,
    State::Disarming { .. } => 7,
    State::Fault(_) => 8
  }
}

/// Fire edges as bursts (reference 6.7): a burst starts with the line active and ends after `gap_ms` inactive.
#[derive(Clone, Copy, Debug)]
struct Burst {
  gap_ms: u64,
  /// When the line was last seen active, while a burst is in progress.
  active_ms: Option<u64>
}

impl Burst {
  const fn new(gap_ms: u32) -> Self {
    Self {
      gap_ms: gap_ms as u64,
      active_ms: None
    }
  }

  /// The line as sampled at `now_ms`. Returns `Fire` when a burst starts and `FireEnded` when one ends.
  fn sample(&mut self, now_ms: u64, firing: bool) -> Option<Event> {
    match (self.active_ms, firing) {
      (None, true) => {
        self.active_ms = Some(now_ms);
        Some(Event::Fire)
      }
      (Some(_), true) => {
        self.active_ms = Some(now_ms);
        None
      }
      (Some(active_ms), false) if now_ms.saturating_sub(active_ms) >= self.gap_ms => {
        self.active_ms = None;
        Some(Event::FireEnded)
      }
      _ => None
    }
  }
}

/// The coordinator's logic as one value: see the module documentation.
#[derive(Clone, Debug)]
pub struct Orchestrator {
  setup: Setup,
  machine: Coordinator,
  link: Link,
  moves: Moves,
  burst: Burst,
  /// The Status line as last told to the state machine.
  job_running: Option<bool>,
  /// A move asked for and not yet sent.
  pending: Option<(Axis, i32)>,
  /// The axis of the move in flight.
  in_flight: Option<Axis>,
  /// The shots of the current job and their readings.
  readings: Readings,
  /// A measurement was asked for and has not ended.
  measuring: bool,
  /// When FootSW is to go inactive, while it is active.
  footsw_until_ms: Option<u64>
}

impl Orchestrator {
  /// After boot: Inhibited, the relay released, no node seen.
  pub fn new(setup: Setup) -> Self {
    let config = &setup.config;
    Self {
      setup,
      machine: Coordinator::new(config.coordinator),
      link: Link::new(&config.node, setup.fw_version),
      moves: Moves::new(config.motion, config.node, config.coordinator.move_tries),
      burst: Burst::new(config.coordinator.burst_gap_ms),
      job_running: None,
      pending: None,
      in_flight: None,
      readings: Readings::new(&config.coordinator),
      measuring: false,
      footsw_until_ms: None
    }
  }

  /// The state machine's state.
  pub const fn state(&self) -> State {
    self.machine.state()
  }

  /// Whether the nodes may move.
  pub const fn motion_allowed(&self) -> bool {
    self.machine.motion_allowed()
  }

  /// Whether the relay is energized, as commanded.
  pub const fn relay_energized(&self) -> bool {
    self.machine.relay_energized()
  }

  /// The link, for what each node last said.
  pub const fn link(&self) -> &Link {
    &self.link
  }

  /// One sample of the inputs at the local time `now_ms`, and everything that is due.
  pub fn tick(&mut self, now_ms: u64, inputs: Inputs, io: &mut impl CoordinatorIo) {
    if self.footsw_until_ms.is_some_and(|until_ms| now_ms >= until_ms) {
      self.footsw_until_ms = None;
      io.set_footsw(false);
    }
    // The inputs first, so that the state machine judges this tick on fresh readings.
    if let Some(event) = self.burst.sample(now_ms, inputs.firing) {
      self.handle(now_ms, event, io);
    }
    if self.job_running != Some(inputs.job_running) {
      self.job_running = Some(inputs.job_running);
      self.handle(now_ms, Event::JobRunning(inputs.job_running), io);
    }
    // Every sample, not only changes: a reading after each energize is what allows motion.
    self.handle(
      now_ms,
      Event::RelayFeedback {
        loop_open: inputs.loop_open
      },
      io
    );
    self.handle(now_ms, Event::Tick, io);
    if self.state() == State::Inhibited && !inputs.loop_open && self.expected_present() {
      self.handle(now_ms, Event::SelfTest { passed: true }, io);
    }
    self.tick_move(now_ms, io);
    if self.measuring {
      for seq in self.readings.tick(now_ms).into_iter().flatten() {
        io.send(Envelope {
          node: NodeId::SENSOR,
          message: Message::SensorResend { seq }
        });
      }
    }
    let beacon = Beacon {
      state: state_code(self.state()),
      inhibited: !self.motion_allowed()
    };
    let lost = self.link.tick(now_ms, beacon, &mut |envelope| io.send(envelope));
    for (node, change) in crate::nodes::WATCHED.iter().zip(lost) {
      if let Some(change) = change {
        self.node_changed(now_ms, *node, change, io);
      }
    }
    // After the beacon, so that INHIBIT 0 is queued before the MOVE_REL.
    self.send_pending(now_ms, io);
  }

  /// A decoded frame from the bus, at the local time `now_ms`.
  pub fn on_frame(&mut self, now_ms: u64, envelope: Envelope, io: &mut impl CoordinatorIo) {
    if let Some((node, change)) = self.link.on_frame(now_ms, envelope) {
      self.node_changed(now_ms, node, change, io);
    }
    if let Some(result) = self.moves.on_frame(&envelope) {
      self.finish(now_ms, result, io);
    }
    if let Message::Sensor(reading) = envelope.message
      && envelope.node == NodeId::SENSOR
      && self.measuring
      && self.readings.on_reading(reading)
    {
      self.handle(now_ms, Event::ReadingsComplete, io);
    }
  }

  /// Asks for a move of `steps` on `axis`.
  ///
  /// # Errors
  ///
  /// A [`RequestError`] naming why the move was not started; nothing changed then.
  pub fn request_move(
    &mut self,
    now_ms: u64,
    axis: Axis,
    steps: i32,
    io: &mut impl CoordinatorIo
  ) -> Result<(), RequestError> {
    self.ready_for(self.setup.wiring.get(axis).node)?;
    let refused = self.handle(now_ms, Event::StartMove, io) == Some(Outcome::Refused);
    if refused || !matches!(self.state(), State::ArmingMove { .. }) {
      return Err(RequestError::Refused);
    }
    self.pending = Some((axis, steps));
    Ok(())
  }

  /// Asks for a measurement: the canned job started, a shot sent to the sensor for each of its dots, and the three
  /// readings collected. Its end is reported as [`Report::MeasurementEnded`].
  ///
  /// # Errors
  ///
  /// A [`RequestError`] naming why the measurement was not started; nothing changed then.
  pub fn request_measurement(&mut self, now_ms: u64, io: &mut impl CoordinatorIo) -> Result<(), RequestError> {
    self.ready_for(NodeId::SENSOR)?;
    let refused = self.handle(now_ms, Event::StartMeasurement, io) == Some(Outcome::Refused);
    if refused || !matches!(self.state(), State::WaitingForFire { .. }) {
      return Err(RequestError::Refused);
    }
    self.measuring = true;
    Ok(())
  }

  /// Stops the move: STOP to the node of the move in flight, or the move asked for dropped before it goes out.
  pub fn stop(&mut self, io: &mut impl CoordinatorIo) {
    if let Some((axis, _)) = self.pending.take() {
      io.report(Report::MoveDropped { axis });
    }
    if let Some(axis) = self.in_flight {
      io.send(Envelope {
        node: self.setup.wiring.get(axis).node,
        message: Message::Stop { reason: 0 }
      });
    }
  }

  /// The operator cleared the fault.
  pub fn clear_fault(&mut self, now_ms: u64, io: &mut impl CoordinatorIo) {
    self.handle(now_ms, Event::ClearFault, io);
  }

  /// Hands one event to the state machine, carries out its actions, and reports what changed.
  /// Returns its outcome.
  fn handle(&mut self, now_ms: u64, event: Event, io: &mut impl CoordinatorIo) -> Option<Outcome> {
    let before = self.machine.state();
    let actions: Actions = self.machine.handle(now_ms, event);
    if let Some(relay) = actions.relay {
      io.set_relay(relay == Relay::Energize);
    }
    if actions.start_job {
      // Before any shot of this job can arrive: nothing of an earlier job may count.
      self.readings.begin();
      self.footsw_until_ms = Some(now_ms.saturating_add(u64::from(self.setup.config.coordinator.start_pulse_ms)));
      io.set_footsw(true);
    }
    if let Some((seq, point)) = actions.shot {
      self.readings.on_shot(now_ms, seq, point);
      io.send(Envelope {
        node: NodeId::SENSOR,
        message: Message::Shot(Shot {
          seq,
          point: shot_point(point),
          edge_age_10us: None
        })
      });
    }
    if actions.estop {
      io.send(Envelope {
        node: NodeId::COORDINATOR,
        message: Message::Estop { reason: 0 }
      });
    }
    // `allow_motion` goes out as the inhibit level with the next beacon, which `tick` sends before any move.
    if let Some(outcome) = actions.outcome {
      io.report(Report::Outcome(outcome));
    }
    let after = self.machine.state();
    if after != before {
      io.report(Report::State(after));
    }
    let in_job = matches!(
      after,
      State::WaitingForFire { .. } | State::Measuring { .. } | State::CollectingReadings { .. }
    );
    if self.measuring && !in_job {
      self.measuring = false;
      let result = match (actions.outcome, self.readings.measurement()) {
        (Some(Outcome::MeasurementReady), Some(Ok(shots))) => Ok(shots),
        (Some(Outcome::MeasurementReady), Some(Err(fault))) => Err(SensorFault::Rejected(fault)),
        _ => Err(SensorFault::NoReading)
      };
      io.report(Report::MeasurementEnded { result });
    }
    actions.outcome
  }

  /// The conditions every request shares: nothing else asked for or running, Idle, and `node` on the bus.
  fn ready_for(&self, node: NodeId) -> Result<(), RequestError> {
    if self.pending.is_some() || self.in_flight.is_some() || self.measuring {
      return Err(RequestError::Busy);
    }
    let state = self.state();
    if state != State::Idle {
      return Err(RequestError::NotIdle { state });
    }
    if !self.link.present(node) {
      return Err(RequestError::NodeAbsent { node: node.get() });
    }
    Ok(())
  }

  /// The move in flight and the move asked for, before the beacon.
  fn tick_move(&mut self, now_ms: u64, io: &mut impl CoordinatorIo) {
    let moving = matches!(self.state(), State::Moving { .. });
    match self.moves.tick(now_ms) {
      Progress::Waiting => {}
      Progress::Resend(envelope) if moving => io.send(envelope),
      Progress::Resend(_) => {
        self.moves.abandon();
        self.finish(now_ms, Err(MoveFault::NoAnswer), io);
      }
      Progress::Ended(result) => self.finish(now_ms, result, io)
    }
    let arming = matches!(self.state(), State::ArmingMove { .. });
    let moving = matches!(self.state(), State::Moving { .. });
    if let Some((axis, _)) = self.pending
      && !arming
      && !moving
    {
      self.pending = None;
      io.report(Report::MoveDropped { axis });
    }
    if moving && self.pending.is_none() && self.in_flight.is_none() {
      self.handle(now_ms, Event::MovesDone, io);
    }
  }

  /// Sends the move asked for, once motion is allowed.
  fn send_pending(&mut self, now_ms: u64, io: &mut impl CoordinatorIo) {
    if !self.motion_allowed() {
      return;
    }
    let Some((axis, steps)) = self.pending.take() else {
      return;
    };
    let wire = self.setup.wiring.get(axis);
    // `start` refuses only a second move in flight or a node without motors, which `request_move` and the wiring
    // rule out; if it ever does, the motors were allowed for nothing and are disarmed at once.
    if let Ok(envelope) = self.moves.start(now_ms, wire.node, wire.channel, steps) {
      io.send(envelope);
      self.in_flight = Some(axis);
      io.report(Report::MoveSent { axis, steps });
    } else {
      io.report(Report::MoveDropped { axis });
      self.handle(now_ms, Event::MovesDone, io);
    }
  }

  /// The move in flight ended.
  fn finish(&mut self, now_ms: u64, result: Result<i32, MoveFault>, io: &mut impl CoordinatorIo) {
    if let Some(axis) = self.in_flight.take() {
      io.report(Report::MoveEnded { axis, result });
    }
    self.handle(now_ms, Event::MovesDone, io);
  }

  fn node_changed(&mut self, now_ms: u64, node: NodeId, change: Change, io: &mut impl CoordinatorIo) {
    io.report(Report::Node { node, change });
    let expected = self.setup.expected.contains(&node);
    match change {
      Change::Appeared => self.configure(node, io),
      Change::Restarted => {
        self.configure(node, io);
        let ours = self
          .in_flight
          .is_some_and(|axis| self.setup.wiring.get(axis).node == node);
        if ours {
          self.moves.abandon();
          self.finish(now_ms, Err(MoveFault::NoAnswer), io);
        }
      }
      Change::Lost | Change::ProtocolMismatch { .. } if expected => {
        self.handle(now_ms, Event::NodeLost, io);
      }
      Change::Lost | Change::ProtocolMismatch { .. } => {}
    }
  }

  /// Sends a node the soft limits and the direction sign of each axis wired to it.
  /// A node with no axis wired to it (the sensor) is sent nothing.
  fn configure(&self, node: NodeId, io: &mut impl CoordinatorIo) {
    let config = &self.setup.config;
    let mut configured = false;
    for axis in Axis::ALL {
      let wire = self.setup.wiring.get(axis);
      if wire.node != node {
        continue;
      }
      configured = true;
      let limits = SoftLimits::for_axis(&config.motion, axis);
      let sign = config.mechanics.dir_sign.get(axis).factor();
      for (key, value) in [
        (ConfigKey::SoftMinSteps, limits.min_steps),
        (ConfigKey::SoftMaxSteps, limits.max_steps),
        (ConfigKey::DirSign, sign)
      ] {
        io.send(Envelope {
          node,
          message: Message::Config {
            key,
            channel: wire.channel,
            value
          }
        });
      }
    }
    if configured {
      io.report(Report::Configured { node });
    }
  }

  fn expected_present(&self) -> bool {
    self.setup.expected.iter().all(|node| self.link.present(*node))
  }
}

#[cfg(test)]
mod tests;
