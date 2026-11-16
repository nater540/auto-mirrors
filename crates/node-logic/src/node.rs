//! The motor node as a state machine: frames, time, and move outcomes in; frames, moves, stops, and records out.
//!
//! The firmware owns the hardware and calls in; this module decides. It never moves a motor by itself.
//! A move starts only for a received command (a MOVE_REL from the coordinator, or a bench command from the
//! console), and only with the hardware permit present. A bus move also needs a fresh coordinator heartbeat
//! and the coordinator's explicit permission (no INHIBIT in force). The arm pulse runs only while a move runs.
//!
//! These checks are the last layer, never the only one (hand-off 3.3): the force-guided relay and the switched
//! motor rail cut motion in hardware whatever this code decides.
//!
//! Decisions the documents leave open, made here:
//!
//! - A node boots inhibited. Only an INHIBIT with `inhibited = 0` from the coordinator allows bus moves.
//! - An ESTOP stops the move and inhibits, so that a node never resumes on its own after an emergency stop.
//! - Moves are refused until the positions are restored from flash, and until the coordinator has sent the
//!   channel's soft limits and direction sign (CONFIG): those are machine facts this node cannot know.
//! - A refused command is answered with a FAULT whose detail is the move's sequence number or the config key.
//! - The suspect flag is set by a rail loss, a pin fault, or a cut-off move. Nothing clears it yet: the protocol
//!   has no message for that, and the coordinator's convergence is what makes a position trusted again.
//! - Soft limits are enforced on every waypoint of the standard approach, suspect or not.
//! - A coordinator heartbeat counts only when it carries this node's protocol version. A mismatch is reported once
//!   (FAULT 0x34) and moves are refused until the versions match.

use align_core::approach::{self, LastDir, SoftLimits};
use align_core::config::{DirSign, Motion, NodeTiming};
use align_proto::{
  Channel, ConfigKey, Envelope, FaultCode, Message, MoveDecision, MoveRefused, MoveRel, MoveRelTracker, MoveResult,
  NodeId, PROTOCOL_VERSION, Status, StatusFlags
};
use stepper::{MoveEnd, Phase, Timing};

use crate::record::AxisRecord;

/// The most stepper channels a node board has (addendum A.7: three, two populated).
pub const MAX_CHANNELS: usize = 3;

/// What the node needs to know about itself.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NodeSetup {
  /// This node's bus address.
  pub id: NodeId,
  /// Channels `0..populated` have motors.
  pub populated: u8,
  /// Firmware version, reported in the heartbeat.
  pub fw_version: u8,
  /// Move timing and caps (addendum A.9).
  pub motion: Motion,
  /// Heartbeat, status, rail, and step period bounds.
  pub timing: NodeTiming
}

/// What the node reported as its state in the heartbeat `state` byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeState {
  /// Positions not restored from flash yet.
  NotReady = 0,
  /// Ready, no move running.
  Idle = 1,
  /// A move is running.
  Moving = 2,
  /// Driven from the console; bus moves are refused.
  Bench = 3
}

/// How a stored record came back at boot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Restored {
  /// No record: a motor never moved under this firmware.
  Missing,
  /// A valid record.
  Valid(AxisRecord),
  /// A record that could not be read. The motor starts fresh but suspect.
  Corrupt
}

/// A move for the firmware to run on one channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MoveRequest {
  /// The channel to move.
  pub channel: Channel,
  /// The legs, in physical steps (the direction sign applied), run in order; either may be zero.
  pub legs_steps: [i32; 2],
  /// Step period, dwell, and hold.
  pub timing: Timing,
  /// Cut the move off as a fault if it has not ended within this many milliseconds.
  pub timeout_ms: u32
}

/// How a move the firmware ran ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveOutcome {
  /// The stepper returned.
  Finished {
    /// Why it ended.
    end: MoveEnd,
    /// Physical steps issued over all legs.
    steps_issued: i32,
    /// The phase the coils were left in.
    phase: Phase
  },
  /// A coil output failed; the steps issued are not known.
  PinFault {
    /// The phase the stepper last applied.
    phase: Phase
  },
  /// The move ran past its timeout and was dropped; the steps issued are not known.
  TimedOut {
    /// The phase the stepper last applied.
    phase: Phase
  }
}

/// What the node asks of the firmware.
pub trait NodeIo {
  /// Queue a frame for the bus.
  fn send(&mut self, envelope: Envelope);
  /// Run a move. The firmware reports it back through [`Node::on_move_outcome`].
  fn start_move(&mut self, request: MoveRequest);
  /// Stop the running move at the next step.
  fn stop_move(&mut self);
  /// Write a record to flash. The firmware reports the result through [`Node::on_persisted`].
  fn persist(&mut self, channel: Channel, record: AxisRecord);
}

/// The state of one motor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisState {
  /// What is stored, or is about to be.
  pub record: AxisRecord,
  /// The last move reached its target.
  pub at_target: bool,
  /// The record in flash matches `record`.
  pub persisted: bool
}

/// Why a bench move was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum BenchRefused {
  /// The node is not in bench mode.
  #[error("not in bench mode: enter it first")]
  NotInBench,
  /// Positions are not restored yet.
  #[error("positions are not restored from flash yet")]
  NotReady,
  /// A move is running.
  #[error("a move is running")]
  Busy,
  /// The hardware motor permit is not present.
  #[error("the motor permit is not present")]
  NoPermit,
  /// The channel has no motor.
  #[error("channel {channel} has no motor on this node")]
  ChannelUnavailable {
    /// The channel asked for.
    channel: u8
  },
  /// The move exceeds the step or time cap, or the step period is outside its bounds.
  #[error("the move exceeds the step or time cap, or its step period is out of bounds")]
  OutOfBounds
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ChannelConfig {
  step_period_us: u32,
  overshoot_steps: i32,
  soft_min_steps: Option<i32>,
  soft_max_steps: Option<i32>,
  dir_sign: Option<DirSign>
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Origin {
  Bus,
  Bench
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Running {
  channel: Channel,
  origin: Origin,
  /// The legs in logical steps (plus advances the screw).
  legs_steps: [i32; 2],
  dir_sign: DirSign,
  stop_requested: bool
}

/// The node.
#[derive(Clone, Debug)]
#[expect(
  clippy::struct_excessive_bools,
  reason = "independent inputs and latches (restored, inhibited, permit, rail, bench), not one state in disguise"
)]
pub struct Node {
  setup: NodeSetup,
  axes: [AxisState; MAX_CHANNELS],
  configs: [ChannelConfig; MAX_CHANNELS],
  restored: bool,
  inhibited: bool,
  permit: bool,
  rail_ok: bool,
  bench: bool,
  /// The coordinator's last heartbeat carried another protocol version: its heartbeats do not count, and the
  /// mismatch has been reported once.
  protocol_mismatch: bool,
  last_heartbeat_ms: Option<u64>,
  tracker: MoveRelTracker,
  running: Option<Running>,
  next_heartbeat_ms: u64,
  next_status_ms: u64
}

impl Node {
  /// A node after boot: not restored, inhibited, no permit seen, nothing running.
  pub fn new(setup: NodeSetup) -> Self {
    let config = ChannelConfig {
      step_period_us: setup.motion.step_period_us,
      overshoot_steps: setup.motion.overshoot_steps,
      soft_min_steps: None,
      soft_max_steps: None,
      dir_sign: None
    };
    let axis = AxisState {
      record: AxisRecord::FRESH,
      at_target: false,
      persisted: false
    };
    Self {
      setup,
      axes: [axis; MAX_CHANNELS],
      configs: [config; MAX_CHANNELS],
      restored: false,
      inhibited: true,
      permit: false,
      rail_ok: false,
      bench: false,
      protocol_mismatch: false,
      last_heartbeat_ms: None,
      tracker: MoveRelTracker::new(),
      running: None,
      next_heartbeat_ms: 0,
      next_status_ms: 0
    }
  }

  /// Takes the records read from flash at boot. Until this is called, every move is refused.
  pub fn restore(&mut self, records: [Restored; MAX_CHANNELS]) {
    for (axis, restored) in self.axes.iter_mut().zip(records) {
      *axis = match restored {
        Restored::Missing => AxisState {
          record: AxisRecord::FRESH,
          at_target: false,
          persisted: false
        },
        Restored::Valid(record) => AxisState {
          record,
          at_target: false,
          persisted: true
        },
        Restored::Corrupt => AxisState {
          record: AxisRecord {
            suspect: true,
            ..AxisRecord::FRESH
          },
          at_target: false,
          persisted: false
        }
      };
    }
    self.restored = true;
  }

  /// The state of one channel, if it exists on a node board.
  pub fn axis(&self, channel: Channel) -> Option<AxisState> {
    self.axes.get(usize::from(channel.get())).copied()
  }

  /// What the heartbeat reports.
  pub fn state(&self) -> NodeState {
    if !self.restored {
      NodeState::NotReady
    } else if self.bench {
      NodeState::Bench
    } else if self.running.is_some() {
      NodeState::Moving
    } else {
      NodeState::Idle
    }
  }

  /// Whether the arm pulse should run now: only while a move runs and every condition for it still holds.
  pub fn armed(&self, now_ms: u64) -> bool {
    let Some(running) = self.running else {
      return false;
    };
    if running.stop_requested || !self.permit {
      return false;
    }
    match running.origin {
      Origin::Bench => self.bench,
      Origin::Bus => !self.bench && !self.inhibited && self.heartbeat_fresh(now_ms)
    }
  }

  /// The hardware motor permit input, sampled. Losing it stops a running move.
  pub fn set_permit(&mut self, present: bool, io: &mut impl NodeIo) {
    self.permit = present;
    if !present {
      self.request_stop(io);
    }
  }

  /// The rail sense input, sampled, for STATUS.
  pub const fn set_rail_ok(&mut self, ok: bool) {
    self.rail_ok = ok;
  }

  /// Handles a frame from the bus.
  pub fn on_frame(&mut self, now_ms: u64, envelope: Envelope, io: &mut impl NodeIo) {
    let from_coordinator = envelope.node == NodeId::COORDINATOR;
    let to_me = envelope.node == self.setup.id;
    match envelope.message {
      Message::Heartbeat { proto_version, .. } if from_coordinator => self.on_heartbeat(now_ms, proto_version, io),
      Message::Estop { .. } if from_coordinator => {
        self.inhibited = true;
        self.request_stop(io);
      }
      Message::Inhibit { inhibited, .. } if from_coordinator => {
        self.inhibited = inhibited;
        if inhibited {
          self.request_stop(io);
        }
      }
      Message::Stop { .. } if to_me => self.request_stop(io),
      Message::MoveRel(move_rel) if to_me => self.on_move_rel(now_ms, move_rel, io),
      Message::Config { key, channel, value } if to_me => self.on_config(key, channel, value, io),
      _ => {}
    }
  }

  /// Sends what is due: the heartbeat, and the periodic STATUS. Stops a bus move whose heartbeat went stale.
  pub fn tick(&mut self, now_ms: u64, io: &mut impl NodeIo) {
    if let Some(running) = self.running
      && running.origin == Origin::Bus
      && !self.heartbeat_fresh(now_ms)
    {
      self.request_stop(io);
    }
    if now_ms >= self.next_heartbeat_ms {
      self.next_heartbeat_ms = now_ms.saturating_add(u64::from(self.setup.timing.heartbeat_period_ms));
      let uptime_ms = u32::try_from(now_ms & u64::from(u32::MAX)).unwrap_or(u32::MAX);
      let state = self.state() as u8;
      self.send(
        io,
        Message::Heartbeat {
          uptime_ms,
          state,
          fw_version: self.setup.fw_version,
          proto_version: PROTOCOL_VERSION
        }
      );
    }
    if now_ms >= self.next_status_ms {
      self.next_status_ms = now_ms.saturating_add(u64::from(self.setup.timing.status_period_ms));
      for channel in self.channels() {
        self.send_status(now_ms, channel, io);
      }
    }
  }

  /// Takes the outcome of the running move: updates the position, answers the coordinator, and persists.
  pub fn on_move_outcome(&mut self, now_ms: u64, outcome: MoveOutcome, io: &mut impl NodeIo) {
    let Some(running) = self.running.take() else {
      return;
    };
    let Some(axis) = self.axes.get_mut(usize::from(running.channel.get())) else {
      return;
    };
    let (end, steps_issued, phase) = match outcome {
      MoveOutcome::Finished {
        end,
        steps_issued,
        phase
      } => (Some(end), steps_issued, phase),
      MoveOutcome::PinFault { phase } | MoveOutcome::TimedOut { phase } => (None, 0, phase)
    };
    let record = &mut axis.record;
    record.pos_steps = record
      .pos_steps
      .saturating_add(steps_issued.saturating_mul(running.dir_sign.factor()));
    record.phase = phase;
    record.last_dir = if end == Some(MoveEnd::Done) {
      final_dir(running.legs_steps, record.last_dir)
    } else {
      LastDir::Unknown
    };
    record.suspect |= end != Some(MoveEnd::Done) && end != Some(MoveEnd::Stopped);
    axis.at_target = end == Some(MoveEnd::Done);
    axis.persisted = false;
    let record = *record;
    io.persist(running.channel, record);

    let (result, fault) = match outcome {
      MoveOutcome::Finished { end: MoveEnd::Done, .. } => (MoveResult::Done, None),
      MoveOutcome::Finished {
        end: MoveEnd::Stopped, ..
      } => (MoveResult::Stopped, None),
      MoveOutcome::Finished {
        end: MoveEnd::RailLost, ..
      } => (MoveResult::RailLost, Some(FaultCode::RAIL_LOST_DURING_MOVE)),
      MoveOutcome::PinFault { .. } => (MoveResult::Stopped, Some(FaultCode::PIN_FAULT)),
      MoveOutcome::TimedOut { .. } => (MoveResult::Stopped, Some(FaultCode::MOVE_TIMEOUT))
    };
    if running.origin == Origin::Bus
      && let Some(done) = self.tracker.finish(result, record.pos_steps)
    {
      if let Some(code) = fault {
        self.send(
          io,
          Message::Fault {
            channel: running.channel,
            code,
            detail: u16::from(done.seq)
          }
        );
      }
      self.send(io, Message::MoveDone(done));
    }
    self.send_status(now_ms, running.channel, io);
  }

  /// Takes the result of a flash write.
  pub fn on_persisted(&mut self, channel: Channel, ok: bool, io: &mut impl NodeIo) {
    if ok {
      if let Some(axis) = self.axes.get_mut(usize::from(channel.get())) {
        axis.persisted = true;
      }
    } else {
      self.send(
        io,
        Message::Fault {
          channel,
          code: FaultCode::PERSIST_FAILED,
          detail: 0
        }
      );
    }
  }

  /// Enters or leaves bench mode, in which the console drives the motors and bus moves are refused.
  /// The mode cannot change while a move runs.
  ///
  /// # Errors
  ///
  /// [`BenchRefused::Busy`] while a move runs.
  pub fn set_bench(&mut self, on: bool) -> Result<(), BenchRefused> {
    if self.running.is_some() {
      return Err(BenchRefused::Busy);
    }
    self.bench = on;
    Ok(())
  }

  /// Starts a console move of `steps` logical steps at `step_period_us`, without the standard approach and without
  /// soft limits: the bench has none configured. The permit, the caps, and the step period bounds still apply.
  ///
  /// # Errors
  ///
  /// A [`BenchRefused`] naming the first condition not met.
  pub fn bench_move(
    &mut self,
    channel: Channel,
    steps: i32,
    step_period_us: u32,
    io: &mut impl NodeIo
  ) -> Result<(), BenchRefused> {
    if !self.bench {
      return Err(BenchRefused::NotInBench);
    }
    if !self.restored {
      return Err(BenchRefused::NotReady);
    }
    if self.running.is_some() {
      return Err(BenchRefused::Busy);
    }
    if !self.permit {
      return Err(BenchRefused::NoPermit);
    }
    if !self.is_populated(channel) {
      return Err(BenchRefused::ChannelUnavailable { channel: channel.get() });
    }
    let timing = &self.setup.timing;
    if !(timing.min_step_period_us..=timing.max_step_period_us).contains(&step_period_us) {
      return Err(BenchRefused::OutOfBounds);
    }
    let dir_sign = self
      .config(channel)
      .and_then(|config| config.dir_sign)
      .unwrap_or(DirSign::Forward);
    let request = self
      .request(channel, [0, steps], step_period_us, dir_sign)
      .ok_or(BenchRefused::OutOfBounds)?;
    self.running = Some(Running {
      channel,
      origin: Origin::Bench,
      legs_steps: [0, steps],
      dir_sign,
      stop_requested: false
    });
    io.start_move(request);
    Ok(())
  }

  /// Stops the running move, whatever started it.
  pub fn stop(&mut self, io: &mut impl NodeIo) {
    self.request_stop(io);
  }

  fn on_move_rel(&mut self, now_ms: u64, move_rel: MoveRel, io: &mut impl NodeIo) {
    let refuse = |node: &Self, io: &mut _, code| {
      node.send(
        io,
        Message::Fault {
          channel: move_rel.channel,
          code,
          detail: u16::from(move_rel.seq)
        }
      );
    };
    match self.tracker.check(move_rel) {
      Ok(MoveDecision::Run) => {}
      Ok(MoveDecision::AlreadyRunning) => return,
      Ok(MoveDecision::AlreadyDone(done)) => return self.send(io, Message::MoveDone(done)),
      Err(MoveRefused::Busy { .. }) => return refuse(self, io, FaultCode::BUSY),
      Err(MoveRefused::SeqReused { .. }) => return refuse(self, io, FaultCode::SEQ_REUSED)
    }
    let checks = [
      (self.bench, FaultCode::BENCH_MODE),
      (!self.restored, FaultCode::NOT_READY),
      (self.running.is_some(), FaultCode::BUSY),
      (self.inhibited, FaultCode::INHIBITED),
      (self.protocol_mismatch, FaultCode::PROTOCOL_MISMATCH),
      (!self.heartbeat_fresh(now_ms), FaultCode::NO_HEARTBEAT),
      (!self.permit, FaultCode::NO_PERMIT),
      (!self.is_populated(move_rel.channel), FaultCode::CHANNEL_UNAVAILABLE)
    ];
    if let Some((_, code)) = checks.iter().find(|(failed, _)| *failed) {
      return refuse(self, io, *code);
    }
    let Some((config, axis)) = self.config(move_rel.channel).zip(self.axis(move_rel.channel)) else {
      return refuse(self, io, FaultCode::CHANNEL_UNAVAILABLE);
    };
    let (Some(min_steps), Some(max_steps), Some(dir_sign)) =
      (config.soft_min_steps, config.soft_max_steps, config.dir_sign)
    else {
      return refuse(self, io, FaultCode::NOT_CONFIGURED);
    };

    let legs_steps = if move_rel.finish_plus {
      approach::plan(move_rel.delta_steps, axis.record.last_dir, config.overshoot_steps).legs()
    } else {
      [0, move_rel.delta_steps]
    };
    let Some(request) = self.request(move_rel.channel, legs_steps, config.step_period_us, dir_sign) else {
      return refuse(self, io, FaultCode::MOVE_TOO_LONG);
    };

    self.tracker.accept(move_rel);
    let limits = SoftLimits { min_steps, max_steps };
    let pos = i64::from(axis.record.pos_steps);
    let [first, second] = legs_steps.map(i64::from);
    if !limits.contains(pos + first) || !limits.contains(pos + first + second) {
      if let Some(done) = self.tracker.finish(MoveResult::SoftLimit, axis.record.pos_steps) {
        self.send(io, Message::MoveDone(done));
      }
      return;
    }
    if legs_steps == [0, 0] {
      if let Some(done) = self.tracker.finish(MoveResult::Done, axis.record.pos_steps) {
        self.send(io, Message::MoveDone(done));
      }
      return;
    }
    self.running = Some(Running {
      channel: move_rel.channel,
      origin: Origin::Bus,
      legs_steps,
      dir_sign,
      stop_requested: false
    });
    io.start_move(request);
  }

  fn on_config(&mut self, key: ConfigKey, channel: Channel, value: i32, io: &mut impl NodeIo) {
    let timing = self.setup.timing;
    let max_move_steps = self.setup.motion.max_move_steps;
    let busy = self.running.is_some();
    let populated = self.is_populated(channel);
    let accepted = populated
      && !busy
      && self
        .configs
        .get_mut(usize::from(channel.get()))
        .is_some_and(|config| match key {
          ConfigKey::StepPeriodUs => u32::try_from(value)
            .ok()
            .filter(|period| (timing.min_step_period_us..=timing.max_step_period_us).contains(period))
            .map(|period| config.step_period_us = period)
            .is_some(),
          ConfigKey::OvershootSteps => (0..=max_move_steps)
            .contains(&value)
            .then(|| config.overshoot_steps = value)
            .is_some(),
          ConfigKey::SoftMinSteps => {
            config.soft_min_steps = Some(value);
            true
          }
          ConfigKey::SoftMaxSteps => {
            config.soft_max_steps = Some(value);
            true
          }
          ConfigKey::DirSign => match value {
            1 => {
              config.dir_sign = Some(DirSign::Forward);
              true
            }
            -1 => {
              config.dir_sign = Some(DirSign::Reversed);
              true
            }
            _ => false
          }
        });
    if !accepted {
      let code = if populated {
        FaultCode::CONFIG_REJECTED
      } else {
        FaultCode::CHANNEL_UNAVAILABLE
      };
      self.send(
        io,
        Message::Fault {
          channel,
          code,
          detail: key as u16
        }
      );
    }
  }

  /// The firmware request for logical legs, or `None` when the move exceeds the step or time cap (addendum A.5).
  fn request(
    &self,
    channel: Channel,
    legs_steps: [i32; 2],
    step_period_us: u32,
    dir_sign: DirSign
  ) -> Option<MoveRequest> {
    let motion = &self.setup.motion;
    let total_steps = legs_steps.iter().map(|leg| u64::from(leg.unsigned_abs())).sum::<u64>();
    if total_steps > u64::from(motion.max_move_steps.unsigned_abs()) {
      return None;
    }
    let legs_run = legs_steps.iter().filter(|leg| **leg != 0).count();
    let overhead_ms = u64::from(motion.energize_dwell_ms + motion.hold_before_release_ms) * legs_run as u64;
    let duration_ms = total_steps * u64::from(step_period_us) / 1_000 + overhead_ms;
    if duration_ms > u64::from(motion.max_move_ms) {
      return None;
    }
    let margin_ms = u64::from(self.setup.timing.move_timeout_margin_ms + self.setup.timing.rail_up_timeout_ms);
    let timeout_ms = u32::try_from(duration_ms + margin_ms).unwrap_or(u32::MAX);
    Some(MoveRequest {
      channel,
      legs_steps: legs_steps.map(|leg| leg.saturating_mul(dir_sign.factor())),
      timing: Timing {
        step_period_us,
        energize_dwell_ms: motion.energize_dwell_ms,
        hold_before_release_ms: motion.hold_before_release_ms
      },
      timeout_ms
    })
  }

  fn request_stop(&mut self, io: &mut impl NodeIo) {
    if let Some(running) = &mut self.running
      && !running.stop_requested
    {
      running.stop_requested = true;
      io.stop_move();
    }
  }

  /// A coordinator heartbeat refreshes the watchdog only when its protocol version matches. A mismatch is
  /// reported once when it starts, and its heartbeats then go stale, which stops a running move.
  fn on_heartbeat(&mut self, now_ms: u64, proto_version: u8, io: &mut impl NodeIo) {
    let mismatch = proto_version != PROTOCOL_VERSION;
    if mismatch && !self.protocol_mismatch {
      let detail = u16::from(proto_version);
      self.send(
        io,
        Message::Fault {
          channel: Channel::ZERO,
          code: FaultCode::PROTOCOL_MISMATCH,
          detail
        }
      );
    }
    self.protocol_mismatch = mismatch;
    if !mismatch {
      self.last_heartbeat_ms = Some(now_ms);
    }
  }

  fn heartbeat_fresh(&self, now_ms: u64) -> bool {
    self
      .last_heartbeat_ms
      .is_some_and(|seen| now_ms.saturating_sub(seen) <= u64::from(self.setup.timing.heartbeat_timeout_ms))
  }

  fn is_populated(&self, channel: Channel) -> bool {
    channel.get() < self.setup.populated && usize::from(channel.get()) < MAX_CHANNELS
  }

  fn config(&self, channel: Channel) -> Option<ChannelConfig> {
    self.configs.get(usize::from(channel.get())).copied()
  }

  fn channels(&self) -> impl Iterator<Item = Channel> + use<> {
    let populated = self.setup.populated.min(u8::try_from(MAX_CHANNELS).unwrap_or(u8::MAX));
    (0..populated).filter_map(Channel::new)
  }

  fn send_status(&self, now_ms: u64, channel: Channel, io: &mut impl NodeIo) {
    let Some(axis) = self.axis(channel) else {
      return;
    };
    let config = self.config(channel);
    let at_limit = config.is_some_and(|config| {
      config.soft_min_steps.is_some_and(|min| axis.record.pos_steps <= min)
        || config.soft_max_steps.is_some_and(|max| axis.record.pos_steps >= max)
    });
    let moving = self.running.is_some_and(|running| running.channel == channel);
    let pairs = [
      (moving, StatusFlags::MOVING),
      (axis.at_target, StatusFlags::AT_TARGET),
      (axis.record.suspect, StatusFlags::SUSPECT),
      (self.rail_ok, StatusFlags::RAIL_OK),
      (at_limit, StatusFlags::SOFT_LIMIT),
      (self.inhibited, StatusFlags::INHIBITED),
      (axis.persisted, StatusFlags::PERSISTED),
      (moving && self.armed(now_ms), StatusFlags::ARMED)
    ];
    let mut flags = StatusFlags::empty();
    for (set, flag) in pairs {
      flags.set(flag, set);
    }
    let status = Status {
      channel,
      flags,
      pos_steps: axis.record.pos_steps,
      current_ma: 0
    };
    self.send(io, Message::Status(status));
  }

  fn send(&self, io: &mut impl NodeIo, message: Message) {
    io.send(Envelope {
      node: self.setup.id,
      message
    });
  }
}

/// The direction of the last leg that moved, or the previous direction when no leg moved.
fn final_dir(legs_steps: [i32; 2], previous: LastDir) -> LastDir {
  let last_moving = legs_steps.iter().rev().find_map(|leg| match leg.signum() {
    1 => Some(LastDir::Plus),
    -1 => Some(LastDir::Minus),
    _ => None
  });
  last_moving.unwrap_or(previous)
}

#[cfg(test)]
mod tests;
