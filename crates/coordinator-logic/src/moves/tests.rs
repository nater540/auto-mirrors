//! One move in flight, one rule at a time.

use align_core::config::Config;

use super::*;

const NODE: NodeId = NodeId::MIRROR_1;

fn moves() -> Moves {
  let config = Config::default();
  Moves::new(config.motion, config.node, config.coordinator.move_tries)
}

fn channel(index: u8) -> Channel {
  Channel::new(index).unwrap()
}

/// The time a move of `steps` may take before it is sent again, with the starting values: 5 ms a step, 120 ms of
/// dwell and hold, and 1000 + 50 + 300 ms of margins.
fn answer_after_ms(steps: u64) -> u64 {
  steps * 5 + 120 + 1_350
}

fn sent(envelope: Envelope) -> MoveRel {
  match envelope.message {
    Message::MoveRel(move_rel) => move_rel,
    other => panic!("not a MOVE_REL: {other:?}")
  }
}

fn done(node: NodeId, seq: u8, channel: Channel, result: MoveResult, pos_steps: i32) -> Envelope {
  Envelope {
    node,
    message: Message::MoveDone(MoveDone {
      seq,
      channel,
      result,
      pos_steps
    })
  }
}

fn fault(node: NodeId, channel: Channel, code: FaultCode, detail: u16) -> Envelope {
  Envelope {
    node,
    message: Message::Fault { channel, code, detail }
  }
}

#[test]
fn a_move_is_one_move_rel_without_the_node_s_own_approach() {
  let mut moves = moves();
  let envelope = moves.start(0, NODE, channel(1), -40).unwrap();
  assert_eq!(envelope.node, NODE);
  assert_eq!(
    sent(envelope),
    MoveRel {
      seq: 0,
      channel: channel(1),
      delta_steps: -40,
      finish_plus: false
    }
  );
  assert!(moves.in_flight());
}

#[test]
fn each_result_of_the_move_done_becomes_what_the_actuator_returns() {
  for (result, expected) in [
    (MoveResult::Done, Ok(140)),
    (MoveResult::Stopped, Err(MoveFault::Stopped { pos_steps: 140 })),
    (MoveResult::RailLost, Err(MoveFault::RailLost { pos_steps: 140 })),
    (MoveResult::SoftLimit, Err(MoveFault::SoftLimit))
  ] {
    let mut moves = moves();
    moves.start(0, NODE, channel(0), 40).unwrap();
    assert_eq!(moves.on_frame(&done(NODE, 0, channel(0), result, 140)), Some(expected));
    assert!(!moves.in_flight());
  }
}

#[test]
fn only_the_answer_of_the_move_in_flight_counts() {
  let mut moves = moves();
  moves.start(0, NODE, channel(0), 40).unwrap();
  for stranger in [
    done(NodeId::MIRROR_2, 0, channel(0), MoveResult::Done, 1),
    done(NODE, 1, channel(0), MoveResult::Done, 1),
    done(NODE, 0, channel(1), MoveResult::Done, 1)
  ] {
    assert_eq!(moves.on_frame(&stranger), None, "{stranger:?}");
  }
  assert!(moves.in_flight());
  assert_eq!(
    moves.on_frame(&done(NODE, 0, channel(0), MoveResult::Done, 40)),
    Some(Ok(40))
  );
  assert_eq!(
    moves.on_frame(&done(NODE, 0, channel(0), MoveResult::Done, 40)),
    None,
    "a second copy of the answer, after a resend, changes nothing"
  );
}

#[test]
fn a_refusal_ends_the_move_and_the_faults_that_come_with_a_move_done_do_not() {
  let mut moves = moves();
  moves.start(0, NODE, channel(0), 40).unwrap();
  for companion in [
    FaultCode::RAIL_LOST_DURING_MOVE,
    FaultCode::PIN_FAULT,
    FaultCode::MOVE_TIMEOUT
  ] {
    assert_eq!(moves.on_frame(&fault(NODE, channel(0), companion, 0)), None);
  }
  assert_eq!(
    moves.on_frame(&fault(NODE, channel(0), FaultCode::INHIBITED, 7)),
    None,
    "another move's number"
  );
  assert_eq!(
    moves.on_frame(&fault(NODE, channel(1), FaultCode::INHIBITED, 0)),
    None,
    "another channel"
  );
  assert_eq!(
    moves.on_frame(&fault(NODE, channel(0), FaultCode::INHIBITED, 256)),
    None,
    "not a sequence number"
  );
  assert_eq!(
    moves.on_frame(&fault(NODE, channel(0), FaultCode::INHIBITED, 0)),
    Some(Err(MoveFault::Refused { code: 0x21 }))
  );
  for refusal in REFUSALS {
    let mut moves = self::moves();
    moves.start(0, NODE, channel(0), 40).unwrap();
    assert!(
      moves.on_frame(&fault(NODE, channel(0), refusal, 0)).is_some(),
      "{refusal:?}"
    );
  }
}

#[test]
fn a_late_answer_sends_the_move_again_and_the_last_try_gives_up() {
  let mut moves = moves();
  let first = moves.start(1_000, NODE, channel(0), 100).unwrap();
  let after = answer_after_ms(100);
  assert_eq!(moves.tick(1_000 + after - 1), Progress::Waiting);
  assert_eq!(moves.tick(1_000 + after), Progress::Resend(first), "the same frame");
  assert_eq!(moves.tick(1_000 + 2 * after - 1), Progress::Waiting);
  assert_eq!(moves.tick(1_000 + 2 * after), Progress::Resend(first));
  assert_eq!(
    moves.tick(1_000 + 3 * after),
    Progress::Ended(Err(MoveFault::NoAnswer)),
    "three tries"
  );
  assert!(!moves.in_flight());
  assert_eq!(moves.tick(1_000_000), Progress::Waiting);
}

#[test]
fn the_answer_to_a_resend_ends_the_move() {
  let mut moves = moves();
  moves.start(0, NODE, channel(0), 100).unwrap();
  moves.tick(answer_after_ms(100));
  assert_eq!(
    moves.on_frame(&done(NODE, 0, channel(0), MoveResult::Done, 100)),
    Some(Ok(100))
  );
}

#[test]
fn one_move_at_a_time_on_a_motor_node() {
  let mut moves = moves();
  moves.start(0, NODE, channel(0), 40).unwrap();
  assert_eq!(
    moves.start(0, NodeId::MIRROR_2, channel(0), 40),
    Err(SendError::Busy { node: 1 })
  );
  moves.abandon();
  assert!(!moves.in_flight());
  assert_eq!(
    moves.start(0, NodeId::SENSOR, channel(0), 40),
    Err(SendError::NotAMotorNode { node: 3 })
  );
}

#[test]
fn sequence_numbers_count_per_node_and_wrap() {
  let mut moves = moves();
  let mut seqs = |node| {
    let seq = sent(moves.start(0, node, channel(0), 1).unwrap()).seq;
    moves.abandon();
    seq
  };
  assert_eq!(
    [seqs(NODE), seqs(NODE), seqs(NodeId::MIRROR_2), seqs(NODE)],
    [0, 1, 0, 2]
  );
  for _ in 0..253 {
    seqs(NODE);
  }
  assert_eq!(seqs(NODE), 0, "after 255");
}

#[test]
fn at_least_one_try_whatever_the_setting() {
  let config = Config::default();
  let mut moves = Moves::new(config.motion, config.node, 0);
  moves.start(0, NODE, channel(0), 0).unwrap();
  assert_eq!(
    moves.tick(answer_after_ms(0)),
    Progress::Ended(Err(MoveFault::NoAnswer))
  );
}
