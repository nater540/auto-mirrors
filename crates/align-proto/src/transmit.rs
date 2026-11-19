//! What a board says about frames the bus did not take: once when sends start failing, a reminder each minute while
//! they keep failing, and once when one gets through again.
//!
//! A board alone on the bus (no other node powered to acknowledge its frames) fails every send, ten heartbeats a
//! second. Logged even once a second, that buried the bench console the owner types into (2026-11-18). The firmware
//! logs what [`TransmitWatch`] returns; the reason a frame failed is the firmware's to add.

use core::fmt;

/// How often a board that still cannot send says so, after the first time.
pub const REMIND_MS: u64 = 60_000;

/// What to tell the operator about the bus after a send.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransmitNote {
  /// A send failed after sends were getting through (or before any had).
  StartedFailing,
  /// Sends are still failing: `failed` sends since they started.
  StillFailing {
    /// Failed sends since the first one, that one included.
    failed: u32
  },
  /// A send got through after `failed` had not.
  Recovered {
    /// Failed sends before this one got through.
    failed: u32
  }
}

impl fmt::Display for TransmitNote {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Self::StartedFailing => write!(
        f,
        "CAN frames are not getting through (is another board on the bus to acknowledge them?); \
         quiet now, with a reminder each minute"
      ),
      Self::StillFailing { failed } => write!(f, "CAN frames still not getting through: {failed} failed so far"),
      Self::Recovered { failed } => write!(f, "CAN frames getting through again, after {failed} failed")
    }
  }
}

/// The state of a board's sends, from one send to the next.
#[derive(Clone, Debug, Default)]
pub struct TransmitWatch {
  failing: Option<Failing>
}

#[derive(Clone, Copy, Debug)]
struct Failing {
  failed: u32,
  noted_ms: u64
}

impl TransmitWatch {
  /// A watch that has seen no failure.
  pub const fn new() -> Self {
    Self { failing: None }
  }

  /// A send got through.
  pub fn sent(&mut self) -> Option<TransmitNote> {
    self
      .failing
      .take()
      .map(|failing| TransmitNote::Recovered { failed: failing.failed })
  }

  /// A send failed at `now_ms`.
  pub fn failed(&mut self, now_ms: u64) -> Option<TransmitNote> {
    let Some(failing) = self.failing.as_mut() else {
      self.failing = Some(Failing {
        failed: 1,
        noted_ms: now_ms
      });
      return Some(TransmitNote::StartedFailing);
    };
    failing.failed = failing.failed.saturating_add(1);
    if now_ms.saturating_sub(failing.noted_ms) < REMIND_MS {
      return None;
    }
    failing.noted_ms = now_ms;
    Some(TransmitNote::StillFailing { failed: failing.failed })
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn the_first_failure_is_told_and_the_next_ones_within_a_minute_are_not() {
    let mut watch = TransmitWatch::new();
    assert_eq!(watch.failed(1_000), Some(TransmitNote::StartedFailing));
    for now_ms in (1_100..1_000 + REMIND_MS).step_by(100) {
      assert_eq!(watch.failed(now_ms), None, "at {now_ms} ms");
    }
  }

  #[test]
  fn a_board_still_failing_is_reminded_each_minute_with_the_count() {
    let mut watch = TransmitWatch::new();
    watch.failed(0);
    for now_ms in (100..REMIND_MS).step_by(100) {
      watch.failed(now_ms);
    }
    assert_eq!(
      watch.failed(REMIND_MS),
      Some(TransmitNote::StillFailing { failed: 601 })
    );
    assert_eq!(watch.failed(REMIND_MS + 100), None);
    assert_eq!(
      watch.failed(2 * REMIND_MS),
      Some(TransmitNote::StillFailing { failed: 603 })
    );
  }

  #[test]
  fn a_send_that_gets_through_after_failures_is_told_once_with_their_count() {
    let mut watch = TransmitWatch::new();
    watch.failed(0);
    watch.failed(100);
    watch.failed(200);
    assert_eq!(watch.sent(), Some(TransmitNote::Recovered { failed: 3 }));
    assert_eq!(watch.sent(), None);
  }

  #[test]
  fn sends_that_get_through_are_not_told() {
    let mut watch = TransmitWatch::new();
    assert_eq!(watch.sent(), None);
    assert_eq!(watch.sent(), None);
  }

  #[test]
  fn failing_again_after_a_recovery_is_told_again() {
    let mut watch = TransmitWatch::new();
    watch.failed(0);
    watch.sent();
    assert_eq!(watch.failed(500), Some(TransmitNote::StartedFailing));
  }

  #[test]
  fn what_is_told_carries_the_count() {
    extern crate std;
    use std::string::ToString;
    assert!(
      TransmitNote::StillFailing { failed: 601 }
        .to_string()
        .contains("601 failed")
    );
    assert!(
      TransmitNote::Recovered { failed: 3 }
        .to_string()
        .contains("after 3 failed")
    );
    assert!(
      TransmitNote::StartedFailing
        .to_string()
        .contains("reminder each minute")
    );
  }

  #[test]
  fn time_going_backward_does_not_remind() {
    let mut watch = TransmitWatch::new();
    watch.failed(10_000);
    assert_eq!(watch.failed(5_000), None);
  }
}
