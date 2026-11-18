//! The bench console's lines for a program, beside the lines it prints for people: the node's state, each channel's
//! count, and how each bench move ended or why it was refused.
//!
//! A program at the console (the `bench-rig` crate) drives moves the way a person does, by typing `jog`, and needs
//! their ends in a form that does not change when a `Debug` print does. Each line is comma separated and starts with
//! [`BENCH_TAG`]; the firmware logs it whole, so it may follow the logger's own prefix. [`BenchLine::parse`] reads one
//! back, so the format has one home and its tests hold both directions.
//!
//! - `BENCH1,status,<state>,<permit>,<rail>`: the first line of `status`, flags as 0 or 1.
//! - `BENCH1,channel,<channel>,<pos_steps>,<suspect>`: one per populated channel, after the status line.
//! - `BENCH1,moved,<channel>,<end>,<pos_steps>,<suspect>`: a bench move ended, with the node's count after it.
//! - `BENCH1,refused,<channel>,<reason>`: a bench move was refused before it started; nothing moved.

use core::fmt;

use stepper::MoveEnd;

use crate::node::{BenchRefused, MoveOutcome, NodeState};

/// The first field of every line for a program, with the version of the format.
pub const BENCH_TAG: &str = "BENCH1";

/// How a bench move ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BenchEnd {
  /// Every step was issued.
  Done,
  /// Stopped by `stop`, a key, or a lost permit, with its steps counted.
  Stopped,
  /// The motor rail read as down: the count is not trusted.
  RailLost,
  /// A coil output failed: the steps issued are not known.
  PinFault,
  /// The move ran past its timeout and was dropped, with its steps counted.
  TimedOut
}

impl BenchEnd {
  /// How a move the firmware ran ended, as the line names it.
  pub const fn of(outcome: &MoveOutcome) -> Self {
    match outcome {
      MoveOutcome::Finished { end, .. } => match end {
        MoveEnd::Done => BenchEnd::Done,
        MoveEnd::Stopped => BenchEnd::Stopped,
        MoveEnd::RailLost => BenchEnd::RailLost
      },
      MoveOutcome::PinFault { .. } => BenchEnd::PinFault,
      MoveOutcome::TimedOut { .. } => BenchEnd::TimedOut
    }
  }

  const ALL: [Self; 5] = [
    BenchEnd::Done,
    BenchEnd::Stopped,
    BenchEnd::RailLost,
    BenchEnd::PinFault,
    BenchEnd::TimedOut
  ];

  /// The end's name in a line.
  pub const fn name(self) -> &'static str {
    match self {
      BenchEnd::Done => "done",
      BenchEnd::Stopped => "stopped",
      BenchEnd::RailLost => "rail_lost",
      BenchEnd::PinFault => "pin_fault",
      BenchEnd::TimedOut => "timed_out"
    }
  }
}

/// One line for a program at the console.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BenchLine {
  /// The node's state and its two inputs: the first line of `status`.
  Status {
    /// The state the heartbeat reports.
    state: NodeState,
    /// The motor permit is present.
    permit: bool,
    /// The motor rail is up.
    rail: bool
  },
  /// A populated channel's count, after the status line.
  Channel {
    /// The channel.
    channel: u8,
    /// The node's count, in full steps from the commissioning position.
    pos_steps: i32,
    /// The count is not trusted.
    suspect: bool
  },
  /// A bench move ended.
  Moved {
    /// The channel.
    channel: u8,
    /// How it ended.
    end: BenchEnd,
    /// The node's count after it.
    pos_steps: i32,
    /// The count is not trusted.
    suspect: bool
  },
  /// A bench move was refused before it started: nothing moved.
  Refused {
    /// The channel asked for.
    channel: u8,
    /// Why.
    reason: BenchRefused
  }
}

const STATES: [NodeState; 4] = [
  NodeState::NotReady,
  NodeState::Idle,
  NodeState::Moving,
  NodeState::Bench
];

/// A state's name in a line.
pub const fn state_name(state: NodeState) -> &'static str {
  match state {
    NodeState::NotReady => "not_ready",
    NodeState::Idle => "idle",
    NodeState::Moving => "moving",
    NodeState::Bench => "bench"
  }
}

/// A refusal's name in a line. The channel a `ChannelUnavailable` carries is the line's own.
pub const fn refusal_name(reason: BenchRefused) -> &'static str {
  match reason {
    BenchRefused::NotInBench => "not_in_bench",
    BenchRefused::NotReady => "not_ready",
    BenchRefused::Busy => "busy",
    BenchRefused::NoPermit => "no_permit",
    BenchRefused::ChannelUnavailable { .. } => "channel_unavailable",
    BenchRefused::OutOfBounds => "out_of_bounds",
    BenchRefused::CoordinatorPresent => "coordinator_present",
    BenchRefused::EstopLatched => "estop_latched",
    BenchRefused::PersistFailed => "persist_failed"
  }
}

fn refusal_named(name: &str, channel: u8) -> Option<BenchRefused> {
  [
    BenchRefused::NotInBench,
    BenchRefused::NotReady,
    BenchRefused::Busy,
    BenchRefused::NoPermit,
    BenchRefused::ChannelUnavailable { channel },
    BenchRefused::OutOfBounds,
    BenchRefused::CoordinatorPresent,
    BenchRefused::EstopLatched,
    BenchRefused::PersistFailed
  ]
  .into_iter()
  .find(|reason| refusal_name(*reason) == name)
}

fn flag(on: bool) -> u8 {
  u8::from(on)
}

fn parse_flag(word: Option<&str>) -> Option<bool> {
  match word? {
    "0" => Some(false),
    "1" => Some(true),
    _ => None
  }
}

fn parse_number<T: core::str::FromStr>(word: Option<&str>) -> Option<T> {
  word?.parse().ok()
}

impl BenchLine {
  /// Reads the line for a program in `text`, which may carry the logger's prefix before [`BENCH_TAG`]. `None` for a
  /// line without one, or one that does not fit its kind: a program must not act on a line it cannot read whole.
  pub fn parse(text: &str) -> Option<Self> {
    let start = text.find(BENCH_TAG)?;
    let mut fields = text.get(start..)?.trim_end().split(',');
    if fields.next()? != BENCH_TAG {
      return None;
    }
    let line = match fields.next()? {
      "status" => {
        let state_word = fields.next()?;
        BenchLine::Status {
          state: STATES.into_iter().find(|state| state_name(*state) == state_word)?,
          permit: parse_flag(fields.next())?,
          rail: parse_flag(fields.next())?
        }
      }
      "channel" => BenchLine::Channel {
        channel: parse_number(fields.next())?,
        pos_steps: parse_number(fields.next())?,
        suspect: parse_flag(fields.next())?
      },
      "moved" => {
        let channel = parse_number(fields.next())?;
        let end_word = fields.next()?;
        BenchLine::Moved {
          channel,
          end: BenchEnd::ALL.into_iter().find(|end| end.name() == end_word)?,
          pos_steps: parse_number(fields.next())?,
          suspect: parse_flag(fields.next())?
        }
      }
      "refused" => {
        let channel = parse_number(fields.next())?;
        BenchLine::Refused {
          channel,
          reason: refusal_named(fields.next()?, channel)?
        }
      }
      _ => return None
    };
    fields.next().is_none().then_some(line)
  }
}

impl fmt::Display for BenchLine {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match *self {
      BenchLine::Status { state, permit, rail } => write!(
        f,
        "{BENCH_TAG},status,{},{},{}",
        state_name(state),
        flag(permit),
        flag(rail)
      ),
      BenchLine::Channel {
        channel,
        pos_steps,
        suspect
      } => write!(f, "{BENCH_TAG},channel,{channel},{pos_steps},{}", flag(suspect)),
      BenchLine::Moved {
        channel,
        end,
        pos_steps,
        suspect
      } => write!(
        f,
        "{BENCH_TAG},moved,{channel},{},{pos_steps},{}",
        end.name(),
        flag(suspect)
      ),
      BenchLine::Refused { channel, reason } => {
        write!(f, "{BENCH_TAG},refused,{channel},{}", refusal_name(reason))
      }
    }
  }
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::string::ToString;

  use proptest::prelude::*;
  use stepper::Phase;

  use super::*;

  #[test]
  fn each_kind_of_line_has_its_fixed_form() {
    let lines = [
      (
        BenchLine::Status {
          state: NodeState::Bench,
          permit: true,
          rail: false
        },
        "BENCH1,status,bench,1,0"
      ),
      (
        BenchLine::Channel {
          channel: 0,
          pos_steps: -1_234,
          suspect: true
        },
        "BENCH1,channel,0,-1234,1"
      ),
      (
        BenchLine::Moved {
          channel: 1,
          end: BenchEnd::RailLost,
          pos_steps: 40,
          suspect: true
        },
        "BENCH1,moved,1,rail_lost,40,1"
      ),
      (
        BenchLine::Refused {
          channel: 0,
          reason: BenchRefused::NoPermit
        },
        "BENCH1,refused,0,no_permit"
      )
    ];
    for (line, text) in lines {
      assert_eq!(line.to_string(), text);
      assert_eq!(BenchLine::parse(text), Some(line), "{text}");
    }
  }

  #[test]
  fn a_line_is_found_after_the_logger_s_prefix() {
    assert_eq!(
      BenchLine::parse("INFO - BENCH1,moved,0,done,-40,0\r"),
      Some(BenchLine::Moved {
        channel: 0,
        end: BenchEnd::Done,
        pos_steps: -40,
        suspect: false
      })
    );
  }

  #[test]
  fn a_line_that_does_not_fit_its_kind_is_not_read() {
    for text in [
      "INFO - channel 0: Finished { end: Done, steps_issued: 40 }",
      "BENCH1",
      "BENCH1,moved,0,done,40",
      "BENCH1,moved,0,done,40,0,extra",
      "BENCH1,moved,0,finished,40,0",
      "BENCH1,moved,0,done,forty,0",
      "BENCH1,moved,0,done,40,2",
      "BENCH1,moved,256,done,40,0",
      "BENCH1,status,asleep,1,1",
      "BENCH1,refused,0,tired",
      "BENCH1,waved,0",
      "BENCH2,moved,0,done,40,0",
      "BENCH1X,moved,0,done,40,0"
    ] {
      assert_eq!(BenchLine::parse(text), None, "{text}");
    }
  }

  #[test]
  fn a_move_s_end_is_named_from_its_outcome() {
    let finished = |end| MoveOutcome::Finished {
      end,
      steps_issued: 3,
      phase: Phase::ZERO
    };
    assert_eq!(BenchEnd::of(&finished(MoveEnd::Done)), BenchEnd::Done);
    assert_eq!(BenchEnd::of(&finished(MoveEnd::Stopped)), BenchEnd::Stopped);
    assert_eq!(BenchEnd::of(&finished(MoveEnd::RailLost)), BenchEnd::RailLost);
    assert_eq!(
      BenchEnd::of(&MoveOutcome::PinFault { phase: Phase::ZERO }),
      BenchEnd::PinFault
    );
    assert_eq!(
      BenchEnd::of(&MoveOutcome::TimedOut {
        steps_issued: 3,
        phase: Phase::ZERO
      }),
      BenchEnd::TimedOut
    );
  }

  fn any_refusal() -> impl Strategy<Value = BenchRefused> {
    prop_oneof![
      Just(BenchRefused::NotInBench),
      Just(BenchRefused::NotReady),
      Just(BenchRefused::Busy),
      Just(BenchRefused::NoPermit),
      Just(BenchRefused::OutOfBounds),
      Just(BenchRefused::CoordinatorPresent),
      Just(BenchRefused::EstopLatched),
      Just(BenchRefused::PersistFailed)
    ]
  }

  fn any_line() -> impl Strategy<Value = BenchLine> {
    prop_oneof![
      (prop::sample::select(STATES.to_vec()), any::<bool>(), any::<bool>())
        .prop_map(|(state, permit, rail)| BenchLine::Status { state, permit, rail }),
      (any::<u8>(), any::<i32>(), any::<bool>()).prop_map(|(channel, pos_steps, suspect)| BenchLine::Channel {
        channel,
        pos_steps,
        suspect
      }),
      (
        any::<u8>(),
        prop::sample::select(BenchEnd::ALL.to_vec()),
        any::<i32>(),
        any::<bool>()
      )
        .prop_map(|(channel, end, pos_steps, suspect)| BenchLine::Moved {
          channel,
          end,
          pos_steps,
          suspect
        }),
      (any::<u8>(), any_refusal()).prop_map(|(channel, reason)| BenchLine::Refused { channel, reason }),
      any::<u8>().prop_map(|channel| BenchLine::Refused {
        channel,
        reason: BenchRefused::ChannelUnavailable { channel }
      })
    ]
  }

  proptest! {
    #[test]
    fn every_line_reads_back_as_itself(line in any_line(), prefix in "[A-Z]{0,5}( - )?") {
      let text = std::format!("{prefix}{line}");
      prop_assert_eq!(BenchLine::parse(&text), Some(line));
    }
  }
}
