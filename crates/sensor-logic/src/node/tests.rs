//! Tests of the sensor node: SHOT to SENSOR, duplicates, resend, commands, the protocol version, bench mode, and
//! coordinator loss, plus a property that no frame sequence panics or answers a shot twice.

extern crate std;

use std::vec::Vec;

use align_core::config::{NodeTiming, SensorTiming};
use align_proto::{
  CmdStatus, Envelope, FaultCode, Message, NodeId, PROTOCOL_VERSION, SUM_Q_ONE, SensorCmd, SensorCmdReply, SensorOp,
  SensorReading, SensorResult, Shot, ShotPoint
};
use proptest::prelude::*;

use super::*;
use crate::buttons::{ButtonBench, ButtonSet};

#[derive(Default)]
struct Recorder {
  sent: Vec<Message>
}

impl SensorIo for Recorder {
  fn send(&mut self, envelope: Envelope) {
    assert_eq!(
      envelope.node,
      NodeId::SENSOR,
      "the sensor sends only under its own address"
    );
    self.sent.push(envelope.message);
  }
}

impl Recorder {
  fn take(&mut self) -> Vec<Message> {
    core::mem::take(&mut self.sent)
  }

  fn readings(&self) -> Vec<SensorReading> {
    self
      .sent
      .iter()
      .filter_map(|message| {
        if let Message::Sensor(reading) = message {
          Some(*reading)
        } else {
          None
        }
      })
      .collect()
  }

  fn faults(&self) -> Vec<FaultCode> {
    let faults = self.sent.iter().filter_map(|message| match message {
      Message::Fault { code, .. } => Some(*code),
      _ => None
    });
    faults.collect()
  }
}

const BENCH: ButtonBench = ButtonBench::STARTING;
const DELAY: u64 = BENCH.reply_delay_ms as u64;
const C: i16 = BENCH.corner_q15;

fn setup() -> SensorSetup {
  SensorSetup {
    fw_version: 1,
    timing: NodeTiming::STARTING,
    sensor_timing: SensorTiming::STARTING,
    bench: BENCH
  }
}

fn coordinator(message: Message) -> Envelope {
  Envelope {
    node: NodeId::COORDINATOR,
    message
  }
}

fn to_sensor(message: Message) -> Envelope {
  Envelope {
    node: NodeId::SENSOR,
    message
  }
}

fn shot(seq: u8) -> Envelope {
  coordinator(Message::Shot(Shot {
    seq,
    point: ShotPoint::A,
    edge_age_10us: Some(10)
  }))
}

fn heartbeat(uptime_ms: u32, proto_version: u8) -> Envelope {
  coordinator(Message::Heartbeat {
    uptime_ms,
    state: 1,
    fw_version: 1,
    proto_version
  })
}

/// A node that has heard the coordinator, with its first heartbeat already sent and cleared.
fn node() -> (SensorNode, Recorder) {
  let mut node = SensorNode::new(setup());
  let mut io = Recorder::default();
  node.on_frame(0, heartbeat(1_000, PROTOCOL_VERSION), &mut io);
  node.tick(0, &mut io);
  io.take();
  (node, io)
}

/// Ticks every 10 ms from `from` to `to`, keeping the coordinator's heartbeat alive.
fn run(node: &mut SensorNode, io: &mut Recorder, from: u64, to: u64) {
  for now in (from..=to).step_by(10) {
    if now % 100 == 0 {
      node.on_frame(
        now,
        heartbeat(u32::try_from(now).unwrap() + 1_000, PROTOCOL_VERSION),
        io
      );
    }
    node.tick(now, io);
  }
}

#[test]
fn a_shot_is_answered_after_the_reply_delay_with_the_held_position() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::B);
  node.on_frame(0, shot(7), &mut io);
  run(&mut node, &mut io, 0, DELAY - 10);
  assert!(io.readings().is_empty(), "not before the delay");
  run(&mut node, &mut io, DELAY, DELAY);
  let readings = io.readings();
  assert_eq!(readings.len(), 1);
  let reading = readings[0];
  assert_eq!(
    (reading.seq, reading.h_q15, reading.v_q15, reading.sum_q),
    (7, -C, C, SUM_Q_ONE)
  );
  assert_eq!(reading.status.result, SensorResult::Valid);
  assert!(!reading.status.calibrated, "buttons are never calibrated");
  assert!(!reading.status.bench);
}

#[test]
fn the_position_is_taken_when_the_shot_arrives() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::A);
  node.on_frame(0, shot(1), &mut io);
  node.set_held(ButtonSet::C);
  run(&mut node, &mut io, 0, DELAY);
  assert_eq!((io.readings()[0].h_q15, io.readings()[0].v_q15), (C, C));
}

#[test]
fn nothing_held_is_centered_and_a_rejected_chord_is_no_shot() {
  let (mut node, mut io) = node();
  node.on_frame(0, shot(1), &mut io);
  node.set_held(ButtonSet::A | ButtonSet::C);
  node.on_frame(0, shot(2), &mut io);
  run(&mut node, &mut io, 0, DELAY);
  let readings = io.readings();
  assert_eq!(
    (readings[0].h_q15, readings[0].v_q15, readings[0].status.result),
    (0, 0, SensorResult::Valid)
  );
  assert_eq!(readings[1].status.result, SensorResult::NoShot);
  assert_eq!((readings[1].h_q15, readings[1].v_q15, readings[1].sum_q), (0, 0, 0));
}

#[test]
fn readings_go_out_in_shot_order() {
  let (mut node, mut io) = node();
  for (at, seq) in [(0, 10), (30, 11), (60, 12)] {
    node.on_frame(at, shot(seq), &mut io);
  }
  run(&mut node, &mut io, 0, DELAY + 100);
  assert_eq!(
    io.readings().iter().map(|reading| reading.seq).collect::<Vec<_>>(),
    [10, 11, 12]
  );
}

#[test]
fn a_duplicate_shot_within_the_window_is_ignored_and_a_later_one_is_answered() {
  let (mut node, mut io) = node();
  node.on_frame(0, shot(5), &mut io);
  node.on_frame(50, shot(5), &mut io);
  run(&mut node, &mut io, 0, 2_000);
  assert_eq!(io.readings().len(), 1);
  node.on_frame(2_001, shot(5), &mut io);
  run(&mut node, &mut io, 2_010, 2_010 + DELAY);
  assert_eq!(io.readings().len(), 2, "outside the dedup window it is a new shot");
}

#[test]
fn a_coordinator_reboot_forgets_seen_numbers_and_pending_shots() {
  let (mut node, mut io) = node();
  node.on_frame(0, shot(1), &mut io);
  // The coordinator restarts: its uptime goes backwards.
  node.on_frame(50, heartbeat(20, PROTOCOL_VERSION), &mut io);
  node.tick(DELAY, &mut io);
  assert!(io.readings().is_empty(), "the pending shot was dropped");
  node.on_frame(DELAY + 10, shot(1), &mut io);
  node.tick(2 * DELAY + 10, &mut io);
  assert_eq!(io.readings().len(), 1, "seq 1 is new again");
}

#[test]
fn coordinator_silence_drops_pending_bus_shots() {
  let mut node = SensorNode::new(setup());
  let mut io = Recorder::default();
  node.on_frame(0, heartbeat(1_000, PROTOCOL_VERSION), &mut io);
  node.set_reply_delay_ms(5_000);
  node.on_frame(10, shot(3), &mut io);
  node.tick(1_000, &mut io);
  assert_eq!(node.state(), SensorState::Busy);
  node.tick(1_001, &mut io);
  assert_eq!(node.state(), SensorState::Idle);
  node.tick(6_000, &mut io);
  assert!(io.readings().is_empty());
}

#[test]
fn a_resend_repeats_a_kept_result_and_ignores_an_unknown_one() {
  let (mut node, mut io) = node();
  node.on_frame(0, shot(9), &mut io);
  run(&mut node, &mut io, 0, DELAY);
  let first = io.readings()[0];
  io.take();
  node.on_frame(DELAY + 1, to_sensor(Message::SensorResend { seq: 9 }), &mut io);
  node.on_frame(DELAY + 1, to_sensor(Message::SensorResend { seq: 99 }), &mut io);
  assert_eq!(io.readings(), [first]);
}

#[test]
fn only_the_last_eight_results_are_kept() {
  let (mut node, mut io) = node();
  for seq in 0..10_u8 {
    node.on_frame(u64::from(seq) * 300, shot(seq), &mut io);
    run(&mut node, &mut io, u64::from(seq) * 300, u64::from(seq) * 300 + DELAY);
  }
  assert!(node.result(1).is_none());
  assert!(node.result(2).is_some());
  assert!(node.result(9).is_some());
}

#[test]
fn the_bench_command_switches_bench_mode_and_calibration_commands_are_refused() {
  let (mut node, mut io) = node();
  let command = |op, arg| to_sensor(Message::SensorCmd(SensorCmd { op, arg }));
  node.on_frame(0, command(SensorOp::Bench, 1), &mut io);
  assert!(node.bench_mode());
  node.on_frame(0, command(SensorOp::Bench, 0), &mut io);
  assert!(!node.bench_mode());
  node.on_frame(0, command(SensorOp::Bench, 2), &mut io);
  for op in [
    SensorOp::Noise,
    SensorOp::Tau,
    SensorOp::SRef,
    SensorOp::Save,
    SensorOp::Report
  ] {
    node.on_frame(0, command(op, 0), &mut io);
  }
  let statuses: Vec<(SensorOp, CmdStatus)> = io
    .sent
    .iter()
    .filter_map(|message| match message {
      Message::SensorCmdReply(SensorCmdReply { op, status, .. }) => Some((*op, *status)),
      _ => None
    })
    .collect();
  assert_eq!(
    statuses[..3],
    [
      (SensorOp::Bench, CmdStatus::Ok),
      (SensorOp::Bench, CmdStatus::Ok),
      (SensorOp::Bench, CmdStatus::Refused)
    ]
  );
  assert!(statuses[3..].iter().all(|(_, status)| *status == CmdStatus::Refused));
  assert_eq!(statuses.len(), 8);
}

#[test]
fn commands_and_resends_for_another_node_are_ignored() {
  let (mut node, mut io) = node();
  let other = |message| Envelope {
    node: NodeId::MIRROR_1,
    message
  };
  node.on_frame(
    0,
    other(Message::SensorCmd(SensorCmd {
      op: SensorOp::Bench,
      arg: 1
    })),
    &mut io
  );
  node.on_frame(0, other(Message::SensorResend { seq: 0 }), &mut io);
  node.on_frame(
    0,
    other(Message::Shot(Shot {
      seq: 1,
      point: ShotPoint::A,
      edge_age_10us: None
    })),
    &mut io
  );
  assert!(io.sent.is_empty());
  assert!(!node.bench_mode());
  node.tick(DELAY, &mut io);
  assert!(
    io.readings().is_empty(),
    "a SHOT from a node other than the coordinator is ignored"
  );
}

#[test]
fn a_coordinator_of_another_protocol_version_is_reported_once_and_its_shots_are_ignored() {
  let (mut node, mut io) = node();
  node.on_frame(10, heartbeat(1_010, PROTOCOL_VERSION + 1), &mut io);
  assert_eq!(io.faults(), [FaultCode::PROTOCOL_MISMATCH]);
  node.on_frame(20, heartbeat(1_020, PROTOCOL_VERSION + 1), &mut io);
  node.on_frame(30, shot(1), &mut io);
  node.tick(30 + DELAY, &mut io);
  assert_eq!(io.faults(), [FaultCode::PROTOCOL_MISMATCH]);
  assert!(io.readings().is_empty());
  node.on_frame(40, heartbeat(1_040, PROTOCOL_VERSION), &mut io);
  node.on_frame(50, shot(2), &mut io);
  node.tick(50 + DELAY, &mut io);
  assert_eq!(io.readings().len(), 1, "a matching heartbeat restores shots");
}

#[test]
fn in_bench_mode_a_chord_fires_a_bench_shot_and_shot_frames_are_ignored() {
  let (mut node, mut io) = node();
  node.set_bench(true);
  node.on_frame(0, shot(1), &mut io);
  node.on_chord(0, ButtonSet::D, &mut io);
  node.on_chord(10, ButtonSet::A | ButtonSet::B, &mut io);
  run(&mut node, &mut io, 0, DELAY + 10);
  let readings = io.readings();
  assert_eq!(readings.len(), 2);
  assert!(readings.iter().all(|reading| reading.status.bench));
  assert_eq!((readings[0].seq, readings[0].h_q15, readings[0].v_q15), (0, C, -C));
  assert_eq!((readings[1].seq, readings[1].h_q15, readings[1].v_q15), (1, 0, C));
  assert_eq!(node.state(), SensorState::Bench);
}

#[test]
fn outside_bench_mode_a_chord_fires_nothing() {
  let (mut node, mut io) = node();
  node.on_chord(0, ButtonSet::A, &mut io);
  node.tick(DELAY, &mut io);
  assert!(io.readings().is_empty());
}

#[test]
fn an_injected_result_applies_to_the_next_shot_only_and_an_injected_sum_until_cleared() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::A);
  node.injection_mut().next_result = Some(SensorResult::Saturated);
  node.injection_mut().sum_q = Some(sum_q_from_percent(150));
  node.on_frame(0, shot(1), &mut io);
  node.on_frame(10, shot(2), &mut io);
  node.injection_mut().sum_q = None;
  node.on_frame(20, shot(3), &mut io);
  run(&mut node, &mut io, 0, DELAY + 20);
  let readings = io.readings();
  assert_eq!(readings[0].status.result, SensorResult::Saturated);
  assert_eq!(
    (readings[0].h_q15, readings[0].sum_q),
    (C, 49_152),
    "Saturated keeps its position"
  );
  assert_eq!(
    (readings[1].status.result, readings[1].sum_q),
    (SensorResult::Valid, 49_152)
  );
  assert_eq!(readings[2].sum_q, SUM_Q_ONE);
}

#[test]
fn a_result_without_a_position_reports_zeros_and_a_failed_measurement_no_sum() {
  for (result, sum_kept) in [
    (SensorResult::NoShot, true),
    (SensorResult::BaselineUnstable, true),
    (SensorResult::TimedOut, false),
    (SensorResult::FrontEndFault, false)
  ] {
    let (mut node, mut io) = node();
    node.set_held(ButtonSet::A);
    node.injection_mut().next_result = Some(result);
    node.on_frame(0, shot(1), &mut io);
    run(&mut node, &mut io, 0, DELAY);
    let reading = io.readings()[0];
    assert_eq!((reading.h_q15, reading.v_q15), (0, 0), "{result:?}");
    assert_eq!(reading.sum_q == SUM_Q_ONE, sum_kept, "{result:?}");
  }
}

#[test]
fn a_full_queue_reports_its_oldest_shot_as_timed_out_at_once() {
  let (mut node, mut io) = node();
  node.set_reply_delay_ms(10_000);
  for seq in 0..5_u8 {
    node.on_frame(u64::from(seq), shot(seq), &mut io);
  }
  let readings = io.readings();
  assert_eq!(readings.len(), 1);
  assert_eq!(
    (readings[0].seq, readings[0].status.result),
    (0, SensorResult::TimedOut)
  );
}

#[test]
fn the_heartbeat_carries_the_state_and_the_protocol_version() {
  let mut node = SensorNode::new(setup());
  let mut io = Recorder::default();
  node.tick(0, &mut io);
  node.set_bench(true);
  node.tick(50, &mut io);
  node.tick(100, &mut io);
  let beats: Vec<(u8, u8)> = io
    .sent
    .iter()
    .filter_map(|message| match message {
      Message::Heartbeat {
        state, proto_version, ..
      } => Some((*state, *proto_version)),
      _ => None
    })
    .collect();
  assert_eq!(
    beats,
    [
      (SensorState::Idle as u8, PROTOCOL_VERSION),
      (SensorState::Bench as u8, PROTOCOL_VERSION)
    ]
  );
}

#[test]
fn the_reply_delay_can_be_changed() {
  let (mut node, mut io) = node();
  assert_eq!(node.reply_delay_ms(), BENCH.reply_delay_ms);
  node.set_reply_delay_ms(30);
  assert_eq!(node.reply_delay_ms(), 30);
  node.on_frame(0, shot(1), &mut io);
  node.tick(29, &mut io);
  assert!(io.readings().is_empty());
  node.tick(30, &mut io);
  assert_eq!(io.readings().len(), 1);
}

#[test]
fn a_heartbeat_from_another_node_is_not_the_coordinator_s() {
  let (mut node, mut io) = node();
  let other = Envelope {
    node: NodeId::MIRROR_1,
    message: Message::Heartbeat {
      uptime_ms: 5,
      state: 1,
      fw_version: 1,
      proto_version: PROTOCOL_VERSION + 1
    }
  };
  node.on_frame(10, other, &mut io);
  assert!(
    io.faults().is_empty(),
    "another node's version is not a coordinator mismatch"
  );
  node.on_frame(20, shot(1), &mut io);
  node.tick(20 + DELAY, &mut io);
  assert_eq!(
    io.readings().len(),
    1,
    "and its lower uptime is not a coordinator reboot"
  );
}

#[test]
fn a_resend_for_another_node_is_ignored_even_for_a_kept_result() {
  let (mut node, mut io) = node();
  node.on_frame(0, shot(4), &mut io);
  run(&mut node, &mut io, 0, DELAY);
  io.take();
  node.on_frame(
    DELAY + 1,
    Envelope {
      node: NodeId::MIRROR_2,
      message: Message::SensorResend { seq: 4 }
    },
    &mut io
  );
  assert!(io.readings().is_empty());
}

#[test]
fn the_heartbeat_uptime_is_the_node_s_clock_in_milliseconds() {
  let mut node = SensorNode::new(setup());
  let mut io = Recorder::default();
  node.tick(1_234, &mut io);
  assert!(matches!(io.sent[0], Message::Heartbeat { uptime_ms: 1_234, .. }));
}

// --- review findings, 2026-11-15 ---------------------------------------------------------------------------------

#[test]
fn a_resend_after_a_coordinator_reboot_returns_the_new_epoch_s_reading() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::A);
  node.on_frame(0, shot(0), &mut io);
  run(&mut node, &mut io, 0, DELAY);
  // The coordinator restarts; its seq 0 is a new shot, read with C held, and that SENSOR frame is lost.
  node.on_frame(DELAY + 10, heartbeat(5, PROTOCOL_VERSION), &mut io);
  node.set_held(ButtonSet::C);
  node.on_frame(DELAY + 20, shot(0), &mut io);
  node.tick(2 * DELAY + 20, &mut io);
  io.take();
  node.on_frame(2 * DELAY + 30, to_sensor(Message::SensorResend { seq: 0 }), &mut io);
  let resent = io.readings();
  assert_eq!(resent.len(), 1);
  assert_eq!((resent[0].h_q15, resent[0].v_q15), (-C, -C));
}

#[test]
fn a_resend_never_returns_a_bench_reading() {
  let (mut node, mut io) = node();
  node.set_bench(true);
  for at in [0, 10, 20] {
    node.on_chord(at, ButtonSet::D, &mut io);
  }
  run(&mut node, &mut io, 0, DELAY + 20);
  node.set_bench(false);
  node.set_held(ButtonSet::B);
  node.on_frame(DELAY + 30, shot(1), &mut io);
  node.tick(2 * DELAY + 30, &mut io);
  io.take();
  node.on_frame(2 * DELAY + 40, to_sensor(Message::SensorResend { seq: 1 }), &mut io);
  let resent = io.readings();
  assert_eq!(resent.len(), 1);
  assert!(!resent[0].status.bench);
  assert_eq!((resent[0].h_q15, resent[0].v_q15), (-C, C));
}

#[test]
fn the_newest_kept_result_for_a_seq_is_the_one_resent() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::A);
  node.on_frame(0, shot(7), &mut io);
  run(&mut node, &mut io, 0, DELAY);
  node.set_held(ButtonSet::D);
  // Outside the dedup window, the same seq is a new shot.
  node.on_frame(3_000, shot(7), &mut io);
  run(&mut node, &mut io, 3_000, 3_000 + DELAY);
  io.take();
  node.on_frame(3_500, to_sensor(Message::SensorResend { seq: 7 }), &mut io);
  assert_eq!((io.readings()[0].h_q15, io.readings()[0].v_q15), (C, -C));
}

#[test]
fn console_held_buttons_apply_at_once_and_add_to_the_physical_ones() {
  let (mut node, mut io) = node();
  node.set_console_held(ButtonSet::A);
  node.on_frame(0, shot(1), &mut io);
  node.set_held(ButtonSet::B);
  node.on_frame(10, shot(2), &mut io);
  node.set_console_held(ButtonSet::empty());
  node.on_frame(20, shot(3), &mut io);
  run(&mut node, &mut io, 0, DELAY + 20);
  let positions: Vec<(i16, i16)> = io
    .readings()
    .iter()
    .map(|reading| (reading.h_q15, reading.v_q15))
    .collect();
  assert_eq!(
    positions,
    [(C, C), (0, C), (-C, C)],
    "A, then A and B (top edge), then B"
  );
}

#[test]
fn a_console_shot_is_a_bench_reading_that_never_blocks_a_bus_shot() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::A);
  node.console_shot(0, &mut io);
  // The coordinator's own shot 0 arrives just after: it must be answered.
  node.on_frame(10, shot(0), &mut io);
  run(&mut node, &mut io, 0, DELAY + 10);
  let readings = io.readings();
  assert_eq!(readings.len(), 2);
  assert!(
    readings[0].status.bench,
    "the console shot is marked so the coordinator ignores it"
  );
  assert!(!readings[1].status.bench);
  assert_eq!(readings[1].seq, 0);
}

#[test]
fn a_button_held_both_physically_and_on_the_console_stays_held() {
  let (mut node, _) = node();
  node.set_held(ButtonSet::A);
  node.set_console_held(ButtonSet::A);
  assert_eq!(node.held(), ButtonSet::A);
  node.set_held(ButtonSet::A);
  assert_eq!(node.held(), ButtonSet::A);
}

#[test]
fn an_uptime_that_disagrees_with_the_local_clock_by_exactly_the_loss_window_is_not_a_reboot() {
  let loss = SensorTiming::STARTING.coordinator_loss_ms;
  let mut node = SensorNode::new(setup());
  let mut io = Recorder::default();
  node.on_frame(0, heartbeat(10_000, PROTOCOL_VERSION), &mut io);
  node.on_frame(10, shot(1), &mut io);
  // 100 ms of local time, 100 + loss ms of coordinator uptime: at the boundary, still the same coordinator.
  node.on_frame(100, heartbeat(10_100 + loss, PROTOCOL_VERSION), &mut io);
  node.tick(10 + DELAY, &mut io);
  assert_eq!(io.readings().len(), 1);
  // One more millisecond of disagreement is a reboot.
  node.on_frame(200, shot(2), &mut io);
  node.on_frame(300, heartbeat(10_300 + 2 * loss + 1, PROTOCOL_VERSION), &mut io);
  node.tick(200 + DELAY, &mut io);
  assert_eq!(
    io.readings().len(),
    1,
    "the pending shot was dropped as a coordinator reboot"
  );
}

#[test]
fn the_uptime_wrap_after_49_days_is_not_a_coordinator_reboot() {
  let mut node = SensorNode::new(setup());
  let mut io = Recorder::default();
  let start = u32::MAX - 50;
  node.on_frame(0, heartbeat(start, PROTOCOL_VERSION), &mut io);
  node.on_frame(10, shot(1), &mut io);
  node.on_frame(100, heartbeat(start.wrapping_add(100), PROTOCOL_VERSION), &mut io);
  node.tick(10 + DELAY, &mut io);
  assert_eq!(io.readings().len(), 1, "the pending shot survived the wrap");
}

#[test]
fn readings_go_out_in_shot_order_even_when_a_slot_is_reused() {
  let (mut node, mut io) = node();
  node.on_frame(0, shot(1), &mut io);
  node.on_frame(DELAY, shot(2), &mut io);
  node.tick(DELAY, &mut io);
  node.on_frame(DELAY, shot(3), &mut io);
  node.set_reply_delay_ms(10);
  node.on_frame(DELAY + 1, shot(4), &mut io);
  run(&mut node, &mut io, DELAY, 3 * DELAY);
  let order: Vec<u8> = io.readings().iter().map(|reading| reading.seq).collect();
  assert_eq!(order, [1, 2, 3, 4]);
}

#[test]
fn an_injected_result_with_a_position_over_a_rejected_chord_reads_as_a_centered_full_beam() {
  let (mut node, mut io) = node();
  node.set_held(ButtonSet::A | ButtonSet::C);
  node.injection_mut().next_result = Some(SensorResult::Valid);
  node.on_frame(0, shot(1), &mut io);
  run(&mut node, &mut io, 0, DELAY);
  let reading = io.readings()[0];
  assert_eq!(
    (reading.status.result, reading.h_q15, reading.v_q15, reading.sum_q),
    (SensorResult::Valid, 0, 0, SUM_Q_ONE)
  );
}

#[test]
fn sum_percent_converts_to_sum_q_and_saturates_from_200() {
  assert_eq!(sum_q_from_percent(100), SUM_Q_ONE);
  assert_eq!(sum_q_from_percent(50), 16_384);
  assert_eq!(sum_q_from_percent(199), 65_208);
  assert_eq!(sum_q_from_percent(200), u16::MAX);
  assert_eq!(sum_q_from_percent(u16::MAX), u16::MAX);
}

#[derive(Clone, Debug)]
enum Event {
  Shot(u8),
  Resend(u8),
  Heartbeat(u32, bool),
  Bench(bool),
  Chord(u8),
  Hold(u8),
  Wait(u16)
}

fn any_event() -> impl Strategy<Value = Event> {
  prop_oneof![
    (0_u8..6).prop_map(Event::Shot),
    (0_u8..6).prop_map(Event::Resend),
    (any::<u32>(), any::<bool>()).prop_map(|(uptime, matching)| Event::Heartbeat(uptime, matching)),
    any::<bool>().prop_map(Event::Bench),
    (0_u8..32).prop_map(Event::Chord),
    (0_u8..32).prop_map(Event::Hold),
    (0_u16..1_500).prop_map(Event::Wait)
  ]
}

proptest! {
  #[test]
  fn no_event_sequence_panics_or_answers_more_bus_shots_than_were_fired(
    events in proptest::collection::vec(any_event(), 0..80)
  ) {
    let (mut node, mut io) = node();
    let mut now = 0_u64;
    let mut shots = 0_usize;
    let mut answers = 0_usize;
    for event in events {
      let resend = matches!(event, Event::Resend(_));
      match event {
        Event::Shot(seq) => {
          shots += 1;
          node.on_frame(now, shot(seq), &mut io);
        }
        Event::Resend(seq) => node.on_frame(now, to_sensor(Message::SensorResend { seq }), &mut io),
        Event::Heartbeat(uptime, matching) => {
          let version = if matching { PROTOCOL_VERSION } else { PROTOCOL_VERSION + 1 };
          node.on_frame(now, heartbeat(uptime, version), &mut io);
        }
        Event::Bench(on) => node.set_bench(on),
        Event::Chord(bits) => node.on_chord(now, ButtonSet::from_bits_truncate(bits), &mut io),
        Event::Hold(bits) => node.set_held(ButtonSet::from_bits_truncate(bits)),
        Event::Wait(ms) => {
          now += u64::from(ms);
          node.tick(now, &mut io);
        }
      }
      if !resend {
        answers += io.readings().iter().filter(|reading| !reading.status.bench).count();
      }
      io.take();
    }
    prop_assert!(answers <= shots, "{answers} answers to {shots} shots");
  }
}
