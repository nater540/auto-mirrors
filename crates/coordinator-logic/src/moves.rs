//! The coordinator's side of one move: MOVE_REL out, MOVE_DONE back (reference 5.4, addendum A.8).
//!
//! The aligner moves one axis at a time and plans the standard approach itself, so every leg is one MOVE_REL with
//! `finish_plus` clear: the node runs exactly the steps it is sent, and the overshoot never runs twice (the seam
//! CLAUDE.md left for M5). This module keeps the one move in flight and turns what comes back into what
//! `align_core::rig::Actuator::move_rel` returns:
//!
//! - **MOVE_DONE** from the node, on the channel and with the sequence number of the move: `Done` is the position
//!   the node reports; `Stopped` and `RailLost` carry it in the fault; `SoftLimit` moved nothing.
//! - **FAULT** from the node, on that channel, with a refusal code and the move's sequence number as its detail:
//!   the node refused the move before running it. The faults that come with a MOVE_DONE (a rail lost, a pin fault,
//!   a move cut off) are left to the MOVE_DONE.
//! - **No answer** by the time the node's own time limit has passed, plus the heartbeat timeout for the frame to
//!   arrive: the same MOVE_REL is sent again, up to `move_tries` times. The node's duplicate tracker runs it once
//!   and answers a repeat of a finished move with the same MOVE_DONE. After the last try, the answer is `NoAnswer`.
//!
//! Sequence numbers count per node, so that a node never sees its last number come back with other content
//! (refused as `SEQ_REUSED`) because moves on the other node used up the numbers in between.

use align_core::config::{Motion, NodeTiming};
use align_core::rig::MoveFault;
use align_proto::{Channel, Envelope, FaultCode, Message, MoveDone, MoveRel, MoveResult, NodeId};

/// The motor nodes, whose sequence numbers are counted apart.
const MOTOR_NODES: [NodeId; 2] = [NodeId::MIRROR_1, NodeId::MIRROR_2];

/// The fault codes with which a node refuses a move before running it.
const REFUSALS: [FaultCode; 12] = [
  FaultCode::NOT_READY,
  FaultCode::INHIBITED,
  FaultCode::NO_HEARTBEAT,
  FaultCode::NO_PERMIT,
  FaultCode::CHANNEL_UNAVAILABLE,
  FaultCode::NOT_CONFIGURED,
  FaultCode::MOVE_TOO_LONG,
  FaultCode::BUSY,
  FaultCode::SEQ_REUSED,
  FaultCode::BENCH_MODE,
  FaultCode::PERSIST_FAILED,
  FaultCode::PROTOCOL_MISMATCH
];

/// Why a move could not be sent.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SendError {
  /// Another move is in flight; the aligner moves one axis at a time.
  #[error("a move is already in flight on node {node}")]
  Busy {
    /// The node of the move in flight.
    node: u8
  },
  /// The node is not a motor node.
  #[error("node {node} has no motors")]
  NotAMotorNode {
    /// The node asked for.
    node: u8
  }
}

/// The move in flight.
#[derive(Clone, Copy, Debug)]
struct InFlight {
  node: NodeId,
  move_rel: MoveRel,
  sent_ms: u64,
  tries: u8,
  answer_after_ms: u64
}

/// What became of the move in flight.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Progress {
  /// Still waiting.
  Waiting,
  /// Send this MOVE_REL again: its answer is late.
  Resend(Envelope),
  /// The move is over: the position the node reports, or why not.
  Ended(Result<i32, MoveFault>)
}

/// The one move in flight, and the next sequence number of each motor node.
#[derive(Clone, Debug)]
pub struct Moves {
  motion: Motion,
  node_timing: NodeTiming,
  tries: u8,
  next_seq: [u8; MOTOR_NODES.len()],
  in_flight: Option<InFlight>
}

impl Moves {
  /// Nothing in flight. `motion` and `node_timing` are what the nodes were configured with, from which the time a
  /// move may take is computed; `tries` is how many times a MOVE_REL is sent before its answer counts as lost.
  pub fn new(motion: Motion, node_timing: NodeTiming, tries: u8) -> Self {
    Self {
      motion,
      node_timing,
      tries: tries.max(1),
      next_seq: [0; MOTOR_NODES.len()],
      in_flight: None
    }
  }

  /// Starts a move of `delta_steps` on a node's channel at the local time `now_ms`.
  /// Returns the MOVE_REL to send.
  ///
  /// # Errors
  ///
  /// [`SendError::Busy`] while another move is in flight,
  /// and [`SendError::NotAMotorNode`] for a node that is not one.
  pub fn start(
    &mut self,
    now_ms: u64,
    node: NodeId,
    channel: Channel,
    delta_steps: i32
  ) -> Result<Envelope, SendError> {
    if let Some(in_flight) = self.in_flight {
      return Err(SendError::Busy {
        node: in_flight.node.get()
      });
    }
    let index = MOTOR_NODES
      .iter()
      .position(|motor_node| *motor_node == node)
      .ok_or(SendError::NotAMotorNode { node: node.get() })?;
    let seq = self.next_seq.get(index).copied().unwrap_or(0);
    if let Some(next) = self.next_seq.get_mut(index) {
      *next = seq.wrapping_add(1);
    }
    let move_rel = MoveRel {
      seq,
      channel,
      delta_steps,
      finish_plus: false
    };
    self.in_flight = Some(InFlight {
      node,
      move_rel,
      sent_ms: now_ms,
      tries: 1,
      answer_after_ms: self.answer_after_ms(delta_steps)
    });
    Ok(envelope(node, move_rel))
  }

  /// Whether a move is in flight.
  pub fn in_flight(&self) -> bool {
    self.in_flight.is_some()
  }

  /// A frame from a node. Returns the end of the move in flight if this frame is its answer.
  pub fn on_frame(&mut self, envelope: &Envelope) -> Option<Result<i32, MoveFault>> {
    let in_flight = self.in_flight?;
    if envelope.node != in_flight.node {
      return None;
    }
    let ours = |channel: Channel, seq: u8| channel == in_flight.move_rel.channel && seq == in_flight.move_rel.seq;
    let ended = match envelope.message {
      Message::MoveDone(MoveDone {
        seq,
        channel,
        result,
        pos_steps
      }) if ours(channel, seq) => match result {
        MoveResult::Done => Ok(pos_steps),
        MoveResult::Stopped => Err(MoveFault::Stopped { pos_steps }),
        MoveResult::RailLost => Err(MoveFault::RailLost { pos_steps }),
        MoveResult::SoftLimit => Err(MoveFault::SoftLimit)
      },
      Message::Fault { channel, code, detail }
        if REFUSALS.contains(&code) && u8::try_from(detail).is_ok_and(|seq| ours(channel, seq)) =>
      {
        Err(MoveFault::Refused { code: code.0 })
      }
      _ => return None
    };
    self.in_flight = None;
    Some(ended)
  }

  /// At the local time `now_ms`: sends the move again if its answer is late,
  /// or gives it up as unanswered after the last try.
  pub fn tick(&mut self, now_ms: u64) -> Progress {
    let Some(in_flight) = self.in_flight.as_mut() else {
      return Progress::Waiting;
    };
    if now_ms.saturating_sub(in_flight.sent_ms) < in_flight.answer_after_ms {
      return Progress::Waiting;
    }
    if in_flight.tries >= self.tries {
      self.in_flight = None;
      return Progress::Ended(Err(MoveFault::NoAnswer));
    }
    in_flight.tries = in_flight.tries.saturating_add(1);
    in_flight.sent_ms = now_ms;
    Progress::Resend(envelope(in_flight.node, in_flight.move_rel))
  }

  /// Forgets the move in flight, as when its node restarted: the node forgot it too, and its position is read from
  /// its STATUS. The caller ends the move as unanswered.
  pub fn abandon(&mut self) {
    self.in_flight = None;
  }

  /// How long to wait for the answer to a move: the node's own limit for it (its duration, the dwell and hold, the
  /// rail coming up, and its margin), and the heartbeat timeout for the MOVE_DONE to arrive.
  fn answer_after_ms(&self, delta_steps: i32) -> u64 {
    let motion = &self.motion;
    let timing = &self.node_timing;
    let steps_ms = u64::from(delta_steps.unsigned_abs()) * u64::from(motion.step_period_us) / 1_000;
    let overhead_ms = u64::from(motion.energize_dwell_ms) + u64::from(motion.hold_before_release_ms);
    let margins_ms = u64::from(timing.move_timeout_margin_ms)
      + u64::from(timing.rail_up_timeout_ms)
      + u64::from(timing.heartbeat_timeout_ms);
    steps_ms + overhead_ms + margins_ms
  }
}

/// A MOVE_REL to a node.
fn envelope(node: NodeId, move_rel: MoveRel) -> Envelope {
  Envelope {
    node,
    message: Message::MoveRel(move_rel)
  }
}

#[cfg(test)]
mod tests;
