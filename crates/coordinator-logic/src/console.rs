//! The coordinator's console: one command per line over USB-Serial-JTAG.
//!
//! The parser is a few lines over `split_whitespace`, as the node's is (`node_logic::console`, which records why
//! no console crate was taken). This first set only reads: the hand-off's jog, poke, calibrate, and align arrive
//! with the state machine that gates them (M5).

/// A console command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
  /// List the commands.
  Help,
  /// Print the bus, and what each node last said.
  Status
}

/// The help text, one command per line.
pub const HELP: &str = "\
help      this list
status    the bus, each node present or not, its last heartbeat, and each channel's last STATUS";

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
    _ => return Err(ParseError::UnknownCommand)
  };
  if words.next().is_some() {
    return Err(ParseError::BadArguments);
  }
  Ok(command)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn every_command_parses() {
    assert_eq!(parse("help"), Ok(Command::Help));
    assert_eq!(parse("?"), Ok(Command::Help));
    assert_eq!(parse("  status  "), Ok(Command::Status));
  }

  #[test]
  fn malformed_lines_are_rejected() {
    assert_eq!(parse(""), Err(ParseError::Empty));
    assert_eq!(parse("   "), Err(ParseError::Empty));
    assert_eq!(parse("jog 0 10"), Err(ParseError::UnknownCommand));
    assert_eq!(parse("status now"), Err(ParseError::BadArguments));
    assert_eq!(parse("help me"), Err(ParseError::BadArguments));
  }

  #[test]
  fn the_help_names_every_command() {
    for command in ["help", "status"] {
      assert!(HELP.lines().any(|line| line.starts_with(command)), "{command}");
    }
  }
}
