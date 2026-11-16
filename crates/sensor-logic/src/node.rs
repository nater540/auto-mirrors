//! The sensor node as a state machine (`docs/SENSOR_PLAN.md`, revision 4, section 7.5).
//!
//! The reading comes from the five buttons through [`ShotMethod`]; everything else is the node a detector will
//! sit behind: SHOT handling with duplicate detection, SENSOR replies in shot order after a delay, SENSOR_RESEND from
//! the last eight results, SENSOR_CMD, the protocol version check, heartbeats, and bench mode.
//!
//! The sensor drives nothing and never fires the laser (R7). Its only output is frames.
//!
//! Decisions made here:
//! - The position is taken when the SHOT arrives, as a detector would see the beam at the fire.
//! - In bench mode, SHOT frames are ignored and a button chord fires a bench shot, reported with the bench bit set
//!   and the sensor's own sequence number. Outside bench mode, chords fire nothing: a reading needs a SHOT.
//! - A coordinator heartbeat of another protocol version is reported once (FAULT 0x34); SHOT frames are then
//!   ignored until the versions match.
//! - A coordinator reboot (its uptime goes backwards) or a heartbeat silence over `coordinator_loss_ms` drops the
//!   pending shots and forgets every seen `seq`, so that a restarted coordinator's numbers are new.

use align_core::config::{NodeTiming, SensorTiming};
use align_proto::{
  Channel, CmdStatus, Envelope, FaultCode, Message, NodeId, PROTOCOL_VERSION, SUM_Q_ONE, SUM_Q_SATURATED, SensorCmd,
  SensorCmdReply, SensorOp, SensorReading, SensorResult, SensorStatus, Shot
};

use crate::buttons::{ButtonBench, ButtonSet, position};

/// Results kept for SENSOR_RESEND, and SHOT numbers remembered for duplicate detection.
pub const KEPT: usize = 8;

/// Shots that may wait for their reading at once.
pub const MAX_PENDING: usize = 4;

/// What the node asks of the firmware.
pub trait SensorIo {
  /// Queue a frame for the bus.
  fn send(&mut self, envelope: Envelope);
}

/// Produces the reading of one shot. The buttons are the first method; a detector's method replaces it (S3).
pub trait ShotMethod {
  /// The reading for a shot fired now, before the result and flags are added.
  fn read(&self) -> Reading;
}

/// What a method measured for one shot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reading {
  /// The outcome.
  pub result: SensorResult,
  /// Horizontal position, Q15; ignored when the result has no position.
  pub h_q15: i16,
  /// Vertical position, Q15.
  pub v_q15: i16,
  /// The sum relative to `S_ref`, as `sum_q`.
  pub sum_q: u16
}

/// The buttons as a [`ShotMethod`]: the held set is the beam position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ButtonMethod {
  /// The bench parameters.
  pub bench: ButtonBench,
  /// The buttons held now.
  pub held: ButtonSet
}

impl ShotMethod for ButtonMethod {
  fn read(&self) -> Reading {
    match position(self.held, self.bench.corner_q15) {
      Some((h_q15, v_q15)) => Reading {
        result: SensorResult::Valid,
        h_q15,
        v_q15,
        sum_q: SUM_Q_ONE
      },
      // A rejected chord is no beam on the sensor.
      None => Reading {
        result: SensorResult::NoShot,
        h_q15: 0,
        v_q15: 0,
        sum_q: 0
      }
    }
  }
}

/// What the sensor reports as its state in the heartbeat `state` byte (`docs/SENSOR_PLAN.md` section 3.5).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorState {
  /// Waiting for shots.
  Idle = 1,
  /// At least one shot waits for its reading.
  Busy = 2,
  /// Bench mode: buttons fire shots, SHOT frames are ignored.
  Bench = 3
}

/// The node's own parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSetup {
  /// Firmware version, reported in the heartbeat.
  pub fw_version: u8,
  /// The heartbeat period.
  pub timing: NodeTiming,
  /// Duplicate and coordinator-loss windows.
  pub sensor_timing: SensorTiming,
  /// The button bench.
  pub bench: ButtonBench
}

/// A result injected from the console for the next shot, for testing the coordinator's handling of it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Injection {
  /// The next shot reports this result instead of the method's.
  pub next_result: Option<SensorResult>,
  /// Every shot reports this `sum_q` instead of the method's, until cleared.
  pub sum_q: Option<u16>
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Pending {
  seq: u8,
  bench: bool,
  due_ms: u64,
  /// The order shots arrived in: readings go out in this order (a reading is never overtaken).
  arrival: u32,
  reading: Reading
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Seen {
  seq: u8,
  at_ms: u64
}

/// The sensor node.
#[derive(Clone, Debug)]
pub struct SensorNode {
  setup: SensorSetup,
  method: ButtonMethod,
  physical_held: ButtonSet,
  console_held: ButtonSet,
  bench_mode: bool,
  protocol_mismatch: bool,
  coordinator_uptime_ms: Option<u32>,
  last_heartbeat_ms: Option<u64>,
  seen: [Option<Seen>; KEPT],
  results: [Option<SensorReading>; KEPT],
  pending: [Option<Pending>; MAX_PENDING],
  injection: Injection,
  bench_seq: u8,
  arrivals: u32,
  next_heartbeat_ms: u64,
  write_cursor: usize
}

impl SensorNode {
  /// A sensor after boot: not in bench mode, no buttons held, nothing pending.
  pub fn new(setup: SensorSetup) -> Self {
    Self {
      setup,
      method: ButtonMethod {
        bench: setup.bench,
        held: ButtonSet::empty()
      },
      physical_held: ButtonSet::empty(),
      console_held: ButtonSet::empty(),
      bench_mode: false,
      protocol_mismatch: false,
      coordinator_uptime_ms: None,
      last_heartbeat_ms: None,
      seen: [None; KEPT],
      results: [None; KEPT],
      pending: [None; MAX_PENDING],
      injection: Injection::default(),
      bench_seq: 0,
      arrivals: 0,
      next_heartbeat_ms: 0,
      write_cursor: 0
    }
  }

  /// What the heartbeat reports.
  pub fn state(&self) -> SensorState {
    if self.bench_mode {
      SensorState::Bench
    } else if self.pending.iter().any(Option::is_some) {
      SensorState::Busy
    } else {
      SensorState::Idle
    }
  }

  /// Whether bench mode is on.
  pub const fn bench_mode(&self) -> bool {
    self.bench_mode
  }

  /// The physical buttons held now, debounced.
  pub fn set_held(&mut self, held: ButtonSet) {
    self.physical_held = held;
    self.method.held = self.physical_held | self.console_held;
  }

  /// The buttons the console holds, added to the physical ones. Takes effect at once, so that a console `hold`
  /// followed straight away by a `shot` reads the new position.
  pub fn set_console_held(&mut self, held: ButtonSet) {
    self.console_held = held;
    self.method.held = self.physical_held | self.console_held;
  }

  /// The buttons held now, physical and console together.
  pub const fn held(&self) -> ButtonSet {
    self.method.held
  }

  /// The console's injection for the coming shots.
  pub const fn injection_mut(&mut self) -> &mut Injection {
    &mut self.injection
  }

  /// Sets the delay from a shot to its reading.
  pub const fn set_reply_delay_ms(&mut self, delay_ms: u32) {
    self.method.bench.reply_delay_ms = delay_ms;
  }

  /// The delay from a shot to its reading.
  pub const fn reply_delay_ms(&self) -> u32 {
    self.method.bench.reply_delay_ms
  }

  /// Enters or leaves bench mode.
  pub const fn set_bench(&mut self, on: bool) {
    self.bench_mode = on;
  }

  /// The newest bus reading sent for `seq`, if it is among the last eight. Bench readings are never kept here:
  /// SENSOR_RESEND serves the coordinator's shots only.
  pub fn result(&self, seq: u8) -> Option<SensorReading> {
    (1..=KEPT)
      .filter_map(|back| self.results.get((self.write_cursor + KEPT - back) % KEPT))
      .flatten()
      .find(|reading| reading.seq == seq)
      .copied()
  }

  /// Handles a frame from the bus.
  pub fn on_frame(&mut self, now_ms: u64, envelope: Envelope, io: &mut impl SensorIo) {
    let from_coordinator = envelope.node == NodeId::COORDINATOR;
    let to_me = envelope.node == NodeId::SENSOR;
    match envelope.message {
      Message::Heartbeat {
        uptime_ms,
        proto_version,
        ..
      } if from_coordinator => {
        self.on_heartbeat(now_ms, uptime_ms, proto_version, io);
      }
      Message::Shot(shot) if from_coordinator => self.on_shot(now_ms, shot, io),
      Message::SensorResend { seq } if to_me => {
        if let Some(reading) = self.result(seq) {
          Self::send(io, Message::Sensor(reading));
        }
      }
      Message::SensorCmd(command) if to_me => self.on_command(command, io),
      _ => {}
    }
  }

  /// A completed button chord. In bench mode it fires a bench shot; otherwise it does nothing (a reading needs a
  /// SHOT). The held set is reported separately through [`Self::set_held`].
  pub fn on_chord(&mut self, now_ms: u64, chord: ButtonSet, io: &mut impl SensorIo) {
    if !self.bench_mode {
      return;
    }
    let reading = ButtonMethod {
      bench: self.method.bench,
      held: chord
    }
    .read();
    let seq = self.bench_seq;
    self.bench_seq = self.bench_seq.wrapping_add(1);
    self.schedule(now_ms, seq, true, reading, io);
  }

  /// A shot from the console, at the held position, reported with the bench bit and the bench counter.
  /// It is never an answer to the coordinator, so it can neither block nor impersonate a real shot.
  /// The console's injection applies to it.
  pub fn console_shot(&mut self, now_ms: u64, io: &mut impl SensorIo) {
    let reading = self.method.read();
    let seq = self.bench_seq;
    self.bench_seq = self.bench_seq.wrapping_add(1);
    self.schedule(now_ms, seq, true, reading, io);
  }

  /// Sends what is due: readings whose delay has passed, in shot order, and the heartbeat. Drops pending shots and
  /// seen numbers after a coordinator silence.
  pub fn tick(&mut self, now_ms: u64, io: &mut impl SensorIo) {
    if let Some(seen) = self.last_heartbeat_ms
      && now_ms.saturating_sub(seen) > u64::from(self.setup.sensor_timing.coordinator_loss_ms)
    {
      self.last_heartbeat_ms = None;
      self.coordinator_uptime_ms = None;
      self.forget_bus_shots();
    }
    // Each pass removes the shot it reports, so the loop ends when nothing more is due.
    while let Some(pending) = self.take_due(now_ms) {
      self.report(pending, io);
    }
    if now_ms >= self.next_heartbeat_ms {
      self.next_heartbeat_ms = now_ms.saturating_add(u64::from(self.setup.timing.heartbeat_period_ms));
      let uptime_ms = u32::try_from(now_ms & u64::from(u32::MAX)).unwrap_or(u32::MAX);
      let state = self.state() as u8;
      let heartbeat = Message::Heartbeat {
        uptime_ms,
        state,
        fw_version: self.setup.fw_version,
        proto_version: PROTOCOL_VERSION
      };
      Self::send(io, heartbeat);
    }
  }

  fn on_heartbeat(&mut self, now_ms: u64, uptime_ms: u32, proto_version: u8, io: &mut impl SensorIo) {
    let mismatch = proto_version != PROTOCOL_VERSION;
    if mismatch && !self.protocol_mismatch {
      let detail = u16::from(proto_version);
      Self::send(
        io,
        Message::Fault {
          channel: Channel::ZERO,
          code: FaultCode::PROTOCOL_MISMATCH,
          detail
        }
      );
    }
    self.protocol_mismatch = mismatch;
    if mismatch {
      return;
    }
    // A reboot is a coordinator uptime that disagrees with the local clock by more than the loss window. Comparing
    // with wrapping arithmetic keeps the 49.7-day wrap of the u32 uptime from looking like one.
    if let (Some(previous), Some(seen_ms)) = (self.coordinator_uptime_ms, self.last_heartbeat_ms) {
      let remote = i64::from(uptime_ms.wrapping_sub(previous));
      let local = i64::try_from(now_ms.saturating_sub(seen_ms)).unwrap_or(i64::MAX);
      if remote.abs_diff(local) > u64::from(self.setup.sensor_timing.coordinator_loss_ms) {
        self.forget_bus_shots();
      }
    }
    self.coordinator_uptime_ms = Some(uptime_ms);
    self.last_heartbeat_ms = Some(now_ms);
  }

  fn on_shot(&mut self, now_ms: u64, shot: Shot, io: &mut impl SensorIo) {
    if self.bench_mode || self.protocol_mismatch {
      return;
    }
    let window = u64::from(self.setup.sensor_timing.dedup_window_ms);
    let duplicate = self
      .seen
      .iter()
      .flatten()
      .any(|seen| seen.seq == shot.seq && now_ms.saturating_sub(seen.at_ms) <= window);
    if duplicate {
      return;
    }
    if let Some(slot) = self
      .seen
      .iter_mut()
      .min_by_key(|slot| slot.map_or(0, |seen| seen.at_ms.saturating_add(1)))
    {
      *slot = Some(Seen {
        seq: shot.seq,
        at_ms: now_ms
      });
    }
    let reading = self.method.read();
    self.schedule(now_ms, shot.seq, false, reading, io);
  }

  fn on_command(&mut self, command: SensorCmd, io: &mut impl SensorIo) {
    let status = match (command.op, command.arg) {
      (SensorOp::Bench, 0 | 1) => {
        self.bench_mode = command.arg == 1;
        CmdStatus::Ok
      }
      // Bench with another argument, and every calibration operation: there is no detector to calibrate yet.
      _ => CmdStatus::Refused
    };
    let reply = SensorCmdReply {
      op: command.op,
      status,
      value: 0,
      extra: 0
    };
    Self::send(io, Message::SensorCmdReply(reply));
  }

  /// Queues a shot's reading for `reply_delay_ms` from now.
  /// With the queue full, the oldest pending shot is reported at once as [`SensorResult::TimedOut`] to make room:
  /// the coordinator then knows it, rather than waiting for its own timeout.
  fn schedule(&mut self, now_ms: u64, seq: u8, bench: bool, mut reading: Reading, io: &mut impl SensorIo) {
    if let Some(result) = self.injection.next_result.take() {
      // A result that carries a position, over a reading that had none, reads as a centered full beam rather than
      // a "valid" reading with no signal.
      if result.has_position() && !reading.result.has_position() {
        reading = Reading {
          result,
          h_q15: 0,
          v_q15: 0,
          sum_q: SUM_Q_ONE
        };
      }
      reading.result = result;
    }
    if let Some(sum_q) = self.injection.sum_q {
      reading.sum_q = sum_q;
    }
    let due_ms = now_ms.saturating_add(u64::from(self.method.bench.reply_delay_ms));
    let arrival = self.arrivals;
    self.arrivals = self.arrivals.wrapping_add(1);
    let pending = Pending {
      seq,
      bench,
      due_ms,
      arrival,
      reading
    };
    if let Some(slot) = self.pending.iter_mut().find(|slot| slot.is_none()) {
      *slot = Some(pending);
      return;
    }
    let oldest = self
      .pending
      .iter()
      .enumerate()
      .filter_map(|(index, slot)| slot.map(|waiting| (index, waiting.arrival.wrapping_sub(arrival))))
      .min_by_key(|(_, age)| *age)
      .map(|(index, _)| index);
    if let Some(slot) = oldest.and_then(|index| self.pending.get_mut(index))
      && let Some(mut given_way) = slot.replace(pending)
    {
      given_way.reading = Reading {
        result: SensorResult::TimedOut,
        h_q15: 0,
        v_q15: 0,
        sum_q: 0
      };
      self.report(given_way, io);
    }
  }

  /// Removes and returns the oldest pending shot if it is due. A later shot waits behind an earlier one, so that
  /// readings go out in shot order whatever their delays.
  fn take_due(&mut self, now_ms: u64) -> Option<Pending> {
    let next = self.arrivals;
    self
      .pending
      .iter_mut()
      .filter(|slot| slot.is_some())
      .min_by_key(|slot| slot.map_or(u32::MAX, |pending| pending.arrival.wrapping_sub(next)))
      .filter(|slot| slot.is_some_and(|pending| pending.due_ms <= now_ms))
      .and_then(Option::take)
  }

  fn report(&mut self, pending: Pending, io: &mut impl SensorIo) {
    let reading = pending.reading;
    let has_position = reading.result.has_position();
    let sum_q = match reading.result {
      SensorResult::TimedOut | SensorResult::FrontEndFault => 0,
      _ => reading.sum_q
    };
    let message = SensorReading {
      seq: pending.seq,
      status: SensorStatus {
        result: reading.result,
        calibrated: false,
        bench: pending.bench
      },
      h_q15: if has_position { reading.h_q15 } else { 0 },
      v_q15: if has_position { reading.v_q15 } else { 0 },
      sum_q
    };
    if !pending.bench {
      if let Some(slot) = self.results.get_mut(self.write_cursor) {
        *slot = Some(message);
      }
      self.write_cursor = (self.write_cursor + 1) % KEPT;
    }
    Self::send(io, Message::Sensor(message));
  }

  fn forget_bus_shots(&mut self) {
    self.seen = [None; KEPT];
    self.results = [None; KEPT];
    for slot in &mut self.pending {
      if slot.is_some_and(|pending| !pending.bench) {
        *slot = None;
      }
    }
  }

  fn send(io: &mut impl SensorIo, message: Message) {
    io.send(Envelope {
      node: NodeId::SENSOR,
      message
    });
  }
}

/// `sum_q` for a sum given in percent of `S_ref`, saturating at [`SUM_Q_SATURATED`] from 200%.
pub fn sum_q_from_percent(percent: u16) -> u16 {
  let scaled = u32::from(percent) * u32::from(SUM_Q_ONE) / 100;
  u16::try_from(scaled).unwrap_or(SUM_Q_SATURATED)
}

#[cfg(test)]
mod tests;
