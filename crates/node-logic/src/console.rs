//! The bench console: one command per line over USB-Serial-JTAG.
//!
//! The parser is a few lines over `split_whitespace`.
//! The console crates checked, `embedded-cli` 0.2.1 (last released March 2024) and `noline` 0.5.1 (December 2024),
//! are past the one-year dormancy rule, and line editing is not needed here.
//!
//! Motors move from the console only in bench mode, with the hardware permit present (see [`crate::node`]).

/// A console command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
  /// List the commands.
  Help,
  /// Print the node state and every channel's position.
  Status,
  /// Enter (`true`) or leave bench mode.
  Bench(bool),
  /// Move one channel by a number of logical full steps, at a rate or the configured one.
  Jog {
    /// The channel.
    channel: u8,
    /// Steps, signed; plus advances the screw.
    steps: i32,
    /// Pulses per second, or `None` for the configured step period.
    rate_pps: Option<u32>
  },
  /// Run the addendum A.6 torque margin cycles on one channel at one rate.
  Torque {
    /// The channel.
    channel: u8,
    /// Pulses per second.
    rate_pps: u32
  },
  /// Stop the running move.
  Stop
}

/// The help text, one command per line.
pub const HELP: &str = "\
help                          this list
status                        node state, and each channel's position, phase, and flags
bench on|off                  enter or leave bench mode (bus moves are refused while on)
jog <channel> <steps> [pps]   move one channel; plus advances the screw
torque <channel> <pps>        addendum A.6 cycles: +amplitude then -amplitude, repeated
stop                          stop the running move";

/// Why a line is not a command.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ParseError {
  /// The line is empty.
  #[error("empty line")]
  Empty,
  /// The first word names no command.
  #[error("unknown command; `help` lists them")]
  UnknownCommand,
  /// An argument is missing, extra, or not a number.
  #[error("wrong arguments; `help` shows the form")]
  BadArguments
}

/// Parses one line.
///
/// # Errors
///
/// A [`ParseError`] for an empty line, an unknown command, or arguments that do not fit it.
pub fn parse(line: &str) -> Result<Command, ParseError> {
  let mut words = line.split_whitespace();
  let command = words.next().ok_or(ParseError::Empty)?;
  let mut arg = || words.next();
  let command = match command {
    "help" | "?" => Command::Help,
    "status" => Command::Status,
    "stop" => Command::Stop,
    "bench" => match arg() {
      Some("on") => Command::Bench(true),
      Some("off") => Command::Bench(false),
      _ => return Err(ParseError::BadArguments)
    },
    "jog" => {
      let channel = number(arg())?;
      let steps = number(arg())?;
      let rate_pps = arg().map(|word| number(Some(word))).transpose()?;
      Command::Jog {
        channel,
        steps,
        rate_pps
      }
    }
    "torque" => Command::Torque {
      channel: number(arg())?,
      rate_pps: number(arg())?
    },
    _ => return Err(ParseError::UnknownCommand)
  };
  if words.next().is_some() {
    return Err(ParseError::BadArguments);
  }
  Ok(command)
}

fn number<T: core::str::FromStr>(word: Option<&str>) -> Result<T, ParseError> {
  word.and_then(|word| word.parse().ok()).ok_or(ParseError::BadArguments)
}

/// The step period for a rate, in microseconds; `None` for zero.
pub fn period_us(rate_pps: u32) -> Option<u32> {
  1_000_000_u32.checked_div(rate_pps)
}

/// The moves of one torque margin run (addendum A.6): `+amplitude`, `-amplitude`, repeated `cycles` times.
pub fn torque_moves(amplitude_steps: i32, cycles: u8) -> impl Iterator<Item = i32> {
  (0..cycles).flat_map(move |_| [amplitude_steps, amplitude_steps.saturating_neg()])
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::vec::Vec;

  use super::*;

  #[test]
  fn every_command_parses() {
    assert_eq!(parse("help"), Ok(Command::Help));
    assert_eq!(parse("  status  "), Ok(Command::Status));
    assert_eq!(parse("bench on"), Ok(Command::Bench(true)));
    assert_eq!(parse("bench off"), Ok(Command::Bench(false)));
    assert_eq!(
      parse("jog 1 -250"),
      Ok(Command::Jog {
        channel: 1,
        steps: -250,
        rate_pps: None
      })
    );
    assert_eq!(
      parse("jog 0 100 150"),
      Ok(Command::Jog {
        channel: 0,
        steps: 100,
        rate_pps: Some(150)
      })
    );
    assert_eq!(
      parse("torque 0 300"),
      Ok(Command::Torque {
        channel: 0,
        rate_pps: 300
      })
    );
    assert_eq!(parse("stop"), Ok(Command::Stop));
  }

  #[test]
  fn malformed_lines_are_rejected() {
    assert_eq!(parse(""), Err(ParseError::Empty));
    assert_eq!(parse("   "), Err(ParseError::Empty));
    assert_eq!(parse("fire"), Err(ParseError::UnknownCommand));
    assert_eq!(parse("bench maybe"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog 1"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog x 10"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog 1 10 fast"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog 1 10 100 extra"), Err(ParseError::BadArguments));
    assert_eq!(parse("torque 0"), Err(ParseError::BadArguments));
    assert_eq!(parse("stop now"), Err(ParseError::BadArguments));
  }

  #[test]
  fn a_rate_converts_to_its_step_period() {
    assert_eq!(period_us(200), Some(5_000));
    assert_eq!(period_us(300), Some(3_333));
    assert_eq!(period_us(0), None);
  }

  #[test]
  fn a_torque_run_alternates_plus_and_minus_and_returns_to_its_start() {
    let moves: Vec<i32> = torque_moves(1_000, 3).collect();
    assert_eq!(moves, [1_000, -1_000, 1_000, -1_000, 1_000, -1_000]);
    assert_eq!(moves.iter().sum::<i32>(), 0);
    assert_eq!(torque_moves(1_000, 10).count(), 20);
  }
}
