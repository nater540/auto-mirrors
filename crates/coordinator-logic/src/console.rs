//! The coordinator's console: one command per line over USB-Serial-JTAG.
//!
//! The parser is a few lines over `split_whitespace`, as the node's is (`node_logic::console`, which records why
//! no console crate was taken). A `jog` runs the whole interlock of [`crate::orchestrator`]: the relay armed and
//! confirmed, the move, and the relay released. A `measure` runs the canned job and collects the sensor's readings.
//! `calibrate`, `align`, and `poke` run the alignment procedure (`align_core::aligner`) through the same interlock.

use align_core::axis::Axis;

/// A console command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
  /// List the commands.
  Help,
  /// Print the state, the inputs, the bus, and what each node last said.
  Status,
  /// Move one axis through the interlock.
  Jog {
    /// The axis.
    axis: Axis,
    /// Steps, signed; plus advances the screw.
    steps: i32
  },
  /// Stop the move.
  Stop,
  /// Run the canned job and collect the sensor's three readings.
  Measure,
  /// Measure the Jacobian by poking each axis, and keep it.
  Calibrate,
  /// Align: calibrate first if no calibration is kept, then step until the residual is inside the tolerance.
  Align,
  /// Measure, move one axis by the poke size, measure, move back, and print the column of the Jacobian it gives.
  Poke {
    /// The axis.
    axis: Axis
  },
  /// Clear a fault: back to Inhibited, from which the self-test runs again.
  Clear
}

/// The help text, one command per line.
pub const HELP: &str = "\
help                  this list
status                the state, the relay and inputs, the bus, each node, and each channel's last STATUS
jog <axis> <steps>    move one axis (m1h, m1v, m2h, m2v) through the interlock; plus advances the screw
stop                  stop the move
measure               run the canned job (FootSW), a shot to the sensor per dot, and collect the three readings
calibrate             poke each axis and keep the measured Jacobian
align                 calibrate if needed, then step until aligned; prints the report
poke <axis>           measure, poke one axis, measure, move it back; prints its Jacobian column
clear                 clear a fault; the self-test then runs again";

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
  let command = match words.next().ok_or(ParseError::Empty)? {
    "help" | "?" => Command::Help,
    "status" => Command::Status,
    "stop" => Command::Stop,
    "measure" => Command::Measure,
    "calibrate" => Command::Calibrate,
    "align" => Command::Align,
    "poke" => Command::Poke {
      axis: words.next().and_then(axis).ok_or(ParseError::BadArguments)?
    },
    "clear" => Command::Clear,
    "jog" => {
      let axis = words.next().and_then(axis).ok_or(ParseError::BadArguments)?;
      let steps = words
        .next()
        .and_then(|word| word.parse().ok())
        .ok_or(ParseError::BadArguments)?;
      Command::Jog { axis, steps }
    }
    _ => return Err(ParseError::UnknownCommand)
  };
  if words.next().is_some() {
    return Err(ParseError::BadArguments);
  }
  Ok(command)
}

/// An axis by its console name, in the order of `align_core::axis` (case does not matter).
fn axis(word: &str) -> Option<Axis> {
  let names = [
    ("m1h", Axis::M1H),
    ("m1v", Axis::M1V),
    ("m2h", Axis::M2H),
    ("m2v", Axis::M2V)
  ];
  names
    .iter()
    .find(|(name, _)| name.eq_ignore_ascii_case(word))
    .map(|(_, axis)| *axis)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn every_command_parses() {
    assert_eq!(parse("help"), Ok(Command::Help));
    assert_eq!(parse("?"), Ok(Command::Help));
    assert_eq!(parse("  status  "), Ok(Command::Status));
    assert_eq!(parse("stop"), Ok(Command::Stop));
    assert_eq!(parse("clear"), Ok(Command::Clear));
    assert_eq!(parse("measure"), Ok(Command::Measure));
    assert_eq!(parse("calibrate"), Ok(Command::Calibrate));
    assert_eq!(parse("align"), Ok(Command::Align));
    assert_eq!(parse("poke m2v"), Ok(Command::Poke { axis: Axis::M2V }));
    assert_eq!(
      parse("jog m1h 100"),
      Ok(Command::Jog {
        axis: Axis::M1H,
        steps: 100
      })
    );
    assert_eq!(
      parse("jog M2V -40"),
      Ok(Command::Jog {
        axis: Axis::M2V,
        steps: -40
      })
    );
  }

  #[test]
  fn every_axis_has_a_name() {
    for (word, expected) in [
      ("m1h", Axis::M1H),
      ("m1v", Axis::M1V),
      ("m2h", Axis::M2H),
      ("m2v", Axis::M2V)
    ] {
      assert_eq!(axis(word), Some(expected));
    }
  }

  #[test]
  fn malformed_lines_are_rejected() {
    assert_eq!(parse(""), Err(ParseError::Empty));
    assert_eq!(parse("   "), Err(ParseError::Empty));
    assert_eq!(parse("fire"), Err(ParseError::UnknownCommand));
    assert_eq!(parse("status now"), Err(ParseError::BadArguments));
    assert_eq!(parse("help me"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog m1h"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog m3h 10"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog 0 10"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog m1h ten"), Err(ParseError::BadArguments));
    assert_eq!(parse("jog m1h 10 200"), Err(ParseError::BadArguments));
    assert_eq!(parse("stop now"), Err(ParseError::BadArguments));
    assert_eq!(parse("measure twice"), Err(ParseError::BadArguments));
    assert_eq!(parse("poke"), Err(ParseError::BadArguments));
    assert_eq!(parse("poke m5h"), Err(ParseError::BadArguments));
    assert_eq!(parse("align now"), Err(ParseError::BadArguments));
  }

  #[test]
  fn the_help_names_every_command() {
    for command in [
      "help",
      "status",
      "jog",
      "stop",
      "measure",
      "calibrate",
      "align",
      "poke",
      "clear"
    ] {
      assert!(HELP.lines().any(|line| line.starts_with(command)), "{command}");
    }
  }
}
