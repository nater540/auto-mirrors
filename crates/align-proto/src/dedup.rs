//! Idempotent MOVE_REL by sequence number (hand-off milestone M3, reference 5.4).
//!
//! The coordinator resends a MOVE_REL when it does not hear a MOVE_DONE in time. Without a check, a resend whose
//! original did arrive moves the screw twice, and with no position feedback nothing would notice. The node keeps
//! the last move it accepted: a frame that repeats it exactly is answered, not run again.
//!
//! One tracker serves one node, because a node runs one move at a time (hand-off 3.1). While a move runs, a new
//! move is refused rather than queued: the coordinator sends the next only after the MOVE_DONE.
//!
//! The tracker lives in RAM. After a node reboot it is empty and accepts any move, so the node firmware must not
//! answer a move until it has restored position from flash; a resend that straddles a reboot is caught by the
//! coordinator, which sees the node's uptime restart in its heartbeat.

use crate::message::{MoveDone, MoveRel, MoveResult};

/// What the node does with a MOVE_REL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveDecision {
  /// A new move: run it, then call [`MoveRelTracker::finish`].
  Run,
  /// A repeat of the move that is running: do nothing. The MOVE_DONE follows when it ends.
  AlreadyRunning,
  /// A repeat of the move that has ended: send this MOVE_DONE again, and do not move.
  AlreadyDone(MoveDone),
}

/// Why a MOVE_REL was refused. The node does not move, and reports the fault.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum MoveRefused {
  /// The sequence number of the last move came back with different content: the coordinator and the node
  /// disagree about which move that number means, so neither is run.
  #[error("MoveRel seq {seq} was already used for {accepted:?}, and now arrives as {received:?}")]
  SeqReused {
    /// The sequence number.
    seq: u8,
    /// The move accepted under that number.
    accepted: MoveRel,
    /// The move received now.
    received: MoveRel,
  },
  /// A different move arrived while one is running.
  #[error("MoveRel seq {received} arrived while seq {running} is still running")]
  Busy {
    /// The sequence number of the running move.
    running: u8,
    /// The sequence number received.
    received: u8,
  },
}

/// The last move a node accepted, and how it ended.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MoveRelTracker {
  last: Option<MoveRel>,
  done: Option<MoveDone>,
}

impl MoveRelTracker {
  /// A tracker that has accepted no move.
  pub const fn new() -> Self {
    Self { last: None, done: None }
  }

  /// Decide what to do with a received MOVE_REL, and remember it when it is to be run.
  ///
  /// # Errors
  ///
  /// [`MoveRefused::SeqReused`] for the last sequence number with different content, [`MoveRefused::Busy`] for a
  /// new move while one runs. Either way the node must not move.
  pub fn on_move_rel(&mut self, received: MoveRel) -> Result<MoveDecision, MoveRefused> {
    let decision = self.check(received)?;
    if decision == MoveDecision::Run {
      self.accept(received);
    }
    Ok(decision)
  }

  /// Decide what to do with a received MOVE_REL without remembering it. A node that may still refuse a new move
  /// for its own reasons (inhibited, no permit) checks first and calls [`Self::accept`] only when it runs the move,
  /// so that a resend after the reason is gone is a new move rather than a repeat of a refused one.
  ///
  /// # Errors
  ///
  /// As [`Self::on_move_rel`].
  pub fn check(&self, received: MoveRel) -> Result<MoveDecision, MoveRefused> {
    let Some(accepted) = self.last else {
      return Ok(MoveDecision::Run);
    };
    if received.seq == accepted.seq {
      if received != accepted {
        return Err(MoveRefused::SeqReused {
          seq: received.seq,
          accepted,
          received,
        });
      }
      return Ok(
        self
          .done
          .map_or(MoveDecision::AlreadyRunning, MoveDecision::AlreadyDone),
      );
    }
    if self.done.is_none() {
      return Err(MoveRefused::Busy {
        running: accepted.seq,
        received: received.seq,
      });
    }
    Ok(MoveDecision::Run)
  }

  /// Remember a move as the one now running. Call it only for a move that [`Self::check`]
  /// answered with [`MoveDecision::Run`].
  pub fn accept(&mut self, received: MoveRel) {
    self.last = Some(received);
    self.done = None;
  }

  /// Record that the running move ended, and return the MOVE_DONE to send. `None` when no move is running.
  pub fn finish(&mut self, result: MoveResult, pos_steps: i32) -> Option<MoveDone> {
    let running = self.last.filter(|_| self.done.is_none())?;
    let done = MoveDone {
      seq: running.seq,
      channel: running.channel,
      result,
      pos_steps,
    };
    self.done = Some(done);
    Some(done)
  }

  /// The move that is running, if one is.
  pub fn running(&self) -> Option<MoveRel> {
    self.last.filter(|_| self.done.is_none())
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;
  use crate::frame::Channel;

  fn move_rel(seq: u8, delta_steps: i32) -> MoveRel {
    MoveRel {
      seq,
      channel: Channel::new(1).unwrap(),
      delta_steps,
      finish_plus: true,
    }
  }

  #[test]
  fn the_first_move_runs() {
    let mut tracker = MoveRelTracker::new();
    assert_eq!(tracker.on_move_rel(move_rel(4, 100)), Ok(MoveDecision::Run));
    assert_eq!(tracker.running(), Some(move_rel(4, 100)));
  }

  #[test]
  fn a_resend_of_a_running_move_does_not_run_it_again() {
    let mut tracker = MoveRelTracker::new();
    tracker.on_move_rel(move_rel(4, 100)).unwrap();
    assert_eq!(tracker.on_move_rel(move_rel(4, 100)), Ok(MoveDecision::AlreadyRunning));
  }

  #[test]
  fn a_resend_of_a_finished_move_is_answered_with_the_same_move_done() {
    let mut tracker = MoveRelTracker::new();
    tracker.on_move_rel(move_rel(4, 100)).unwrap();
    let done = tracker.finish(MoveResult::Done, 1100).unwrap();
    assert_eq!(
      done,
      MoveDone {
        seq: 4,
        channel: Channel::new(1).unwrap(),
        result: MoveResult::Done,
        pos_steps: 1100
      }
    );
    assert_eq!(
      tracker.on_move_rel(move_rel(4, 100)),
      Ok(MoveDecision::AlreadyDone(done))
    );
    assert_eq!(
      tracker.on_move_rel(move_rel(4, 100)),
      Ok(MoveDecision::AlreadyDone(done))
    );
    assert_eq!(tracker.running(), None);
  }

  #[test]
  fn the_last_sequence_number_with_different_content_is_refused() {
    let mut tracker = MoveRelTracker::new();
    tracker.on_move_rel(move_rel(4, 100)).unwrap();
    tracker.finish(MoveResult::Done, 100).unwrap();
    assert_eq!(
      tracker.on_move_rel(move_rel(4, -100)),
      Err(MoveRefused::SeqReused {
        seq: 4,
        accepted: move_rel(4, 100),
        received: move_rel(4, -100)
      })
    );
  }

  #[test]
  fn a_new_move_while_one_runs_is_refused() {
    let mut tracker = MoveRelTracker::new();
    tracker.on_move_rel(move_rel(4, 100)).unwrap();
    assert_eq!(
      tracker.on_move_rel(move_rel(5, 100)),
      Err(MoveRefused::Busy {
        running: 4,
        received: 5
      })
    );
    assert_eq!(tracker.running(), Some(move_rel(4, 100)));
  }

  #[test]
  fn a_new_move_after_the_last_finished_runs() {
    let mut tracker = MoveRelTracker::new();
    tracker.on_move_rel(move_rel(255, 100)).unwrap();
    tracker.finish(MoveResult::Stopped, 40).unwrap();
    assert_eq!(tracker.on_move_rel(move_rel(0, 100)), Ok(MoveDecision::Run));
  }

  #[test]
  fn a_checked_move_is_not_remembered_until_it_is_accepted() {
    let mut tracker = MoveRelTracker::new();
    assert_eq!(tracker.check(move_rel(4, 100)), Ok(MoveDecision::Run));
    assert_eq!(tracker.running(), None);
    // Refused for a reason of the node's own, then resent: still a new move.
    assert_eq!(tracker.check(move_rel(4, 100)), Ok(MoveDecision::Run));
    tracker.accept(move_rel(4, 100));
    assert_eq!(tracker.running(), Some(move_rel(4, 100)));
    assert_eq!(tracker.check(move_rel(4, 100)), Ok(MoveDecision::AlreadyRunning));
  }

  #[test]
  fn finishing_with_no_move_running_returns_nothing() {
    let mut tracker = MoveRelTracker::new();
    assert_eq!(tracker.finish(MoveResult::Done, 0), None);
    tracker.on_move_rel(move_rel(1, 10)).unwrap();
    tracker.finish(MoveResult::Done, 10).unwrap();
    assert_eq!(tracker.finish(MoveResult::Done, 10), None);
  }

  proptest! {
    #[test]
    fn however_often_each_move_is_resent_each_runs_exactly_once(
      deltas in proptest::collection::vec(any::<i32>(), 1..40),
      resends in proptest::collection::vec(0_usize..4, 40),
    ) {
      let mut tracker = MoveRelTracker::new();
      let mut runs = 0;
      for (i, (delta, extra)) in deltas.iter().zip(resends).enumerate() {
        let seq = u8::try_from(i % 256).unwrap();
        for _ in 0..=extra {
          if tracker.on_move_rel(move_rel(seq, *delta)).unwrap() == MoveDecision::Run {
            runs += 1;
          }
        }
        tracker.finish(MoveResult::Done, 0).unwrap();
        // Resends after the MOVE_DONE are answered, never run.
        for _ in 0..extra {
          let decision = tracker.on_move_rel(move_rel(seq, *delta)).unwrap();
          prop_assert!(matches!(decision, MoveDecision::AlreadyDone(_)));
        }
      }
      prop_assert_eq!(runs, deltas.len());
    }
  }
}
