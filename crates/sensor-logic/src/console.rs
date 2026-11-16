//! The sensor's bench console: one command per line over USB-Serial-JTAG (`docs/SENSOR_PLAN.md` section 7.5).
//!
//! The parser is a few lines over `split_whitespace`, as on the motor node. Button sets are written as letters:
//! `a`, `b`, `c`, `d` for the corners and `x` for the center, so `ab` is the top edge and `xa` halfway to A.

use align_proto::{SensorResult, ShotPoint};

use crate::buttons::ButtonSet;

/// A console command.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
  /// List the commands.
  Help,
  /// Print the state, the held buttons, the injection, and the last results.
  Status,
  /// Enter (`true`) or leave bench mode.
  Bench(bool),
  /// Act as a chord of these buttons: in bench mode, a bench shot.
  Press(ButtonSet),
  /// Hold these buttons, as the position for the next SHOT, until `release`.
  Hold(ButtonSet),
  /// Release every held button.
  Release,
  /// Inject a SHOT through the receive path, as if from the coordinator.
  Shot {
    /// Where the gantry was.
    point: ShotPoint,
    /// The edge age, in 10 us units, or unknown.
    edge_age_10us: Option<u16>
  },
  /// Set the delay from a shot to its reading.
  Delay(u32),
  /// Report this result for the next shot.
  Next(SensorResult),
  /// Report this sum, in percent of `S_ref`, until `sim sum off`.
  Sum(Option<u16>)
}

/// The help text, one command per line.
pub const HELP: &str = "\
help                          this list
status                        state, held buttons, injection, and the last results
bench on|off                  enter or leave bench mode (buttons fire shots; SHOT frames are ignored)
press <set>                   act as a button chord, for example `press a` or `press ab` (x is the center)
hold <set> | release          hold buttons as the position for the next SHOT, or release them
shot [a|b|c|other] [age]      inject a SHOT from the coordinator; age in 10 us units
sim delay <ms>                delay from a shot to its reading
sim next <result>             valid, noshot, saturated, negative, baseline, overlap, timeout, frontend
sim sum <percent>|off         report this sum in percent of S_ref, or the method's";

/// Why a line is not a command.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ParseError {
  /// The line is empty.
  #[error("empty line")]
  Empty,
  /// The first word names no command.
  #[error("unknown command; `help` lists them")]
  UnknownCommand,
  /// An argument is missing, extra, or malformed.
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
  let first = words.next().ok_or(ParseError::Empty)?;
  let second = words.next();
  let third = words.next();
  let command = match (first, second, third) {
    ("help" | "?", None, None) => Command::Help,
    ("status", None, None) => Command::Status,
    ("bench", Some("on"), None) => Command::Bench(true),
    ("bench", Some("off"), None) => Command::Bench(false),
    ("press", Some(set), None) => Command::Press(buttons(set)?),
    ("hold", Some(set), None) => Command::Hold(buttons(set)?),
    ("release", None, None) => Command::Release,
    ("shot", point, age) => Command::Shot {
      point: point.map_or(Ok(ShotPoint::Other), shot_point)?,
      edge_age_10us: age
        .map(|word| word.parse().map_err(|_| ParseError::BadArguments))
        .transpose()?
    },
    ("sim", Some("delay"), Some(ms)) => Command::Delay(ms.parse().map_err(|_| ParseError::BadArguments)?),
    ("sim", Some("next"), Some(name)) => Command::Next(result(name)?),
    ("sim", Some("sum"), Some("off")) => Command::Sum(None),
    ("sim", Some("sum"), Some(percent)) => Command::Sum(Some(percent.parse().map_err(|_| ParseError::BadArguments)?)),
    ("help" | "?" | "status" | "bench" | "press" | "hold" | "release" | "sim", ..) => {
      return Err(ParseError::BadArguments);
    }
    _ => return Err(ParseError::UnknownCommand)
  };
  if words.next().is_some() {
    return Err(ParseError::BadArguments);
  }
  Ok(command)
}

/// A button set from letters: `a`, `b`, `c`, `d`, and `x` for the center. Empty or with another letter: an error.
fn buttons(word: &str) -> Result<ButtonSet, ParseError> {
  let mut set = ButtonSet::empty();
  for letter in word.chars() {
    set |= match letter {
      'a' => ButtonSet::A,
      'b' => ButtonSet::B,
      'c' => ButtonSet::C,
      'd' => ButtonSet::D,
      'x' => ButtonSet::CENTER,
      _ => return Err(ParseError::BadArguments)
    };
  }
  if set.is_empty() {
    Err(ParseError::BadArguments)
  } else {
    Ok(set)
  }
}

fn shot_point(word: &str) -> Result<ShotPoint, ParseError> {
  match word {
    "a" => Ok(ShotPoint::A),
    "b" => Ok(ShotPoint::B),
    "c" => Ok(ShotPoint::C),
    "other" => Ok(ShotPoint::Other),
    _ => Err(ParseError::BadArguments)
  }
}

fn result(word: &str) -> Result<SensorResult, ParseError> {
  Ok(match word {
    "valid" => SensorResult::Valid,
    "noshot" => SensorResult::NoShot,
    "saturated" => SensorResult::Saturated,
    "negative" => SensorResult::NegativeQuadrant,
    "baseline" => SensorResult::BaselineUnstable,
    "overlap" => SensorResult::Overlap,
    "timeout" => SensorResult::TimedOut,
    "frontend" => SensorResult::FrontEndFault,
    _ => return Err(ParseError::BadArguments)
  })
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn every_command_parses() {
    assert_eq!(parse("help"), Ok(Command::Help));
    assert_eq!(parse(" status "), Ok(Command::Status));
    assert_eq!(parse("bench on"), Ok(Command::Bench(true)));
    assert_eq!(parse("bench off"), Ok(Command::Bench(false)));
    assert_eq!(parse("press a"), Ok(Command::Press(ButtonSet::A)));
    assert_eq!(parse("press xd"), Ok(Command::Press(ButtonSet::CENTER | ButtonSet::D)));
    assert_eq!(parse("hold bc"), Ok(Command::Hold(ButtonSet::B | ButtonSet::C)));
    assert_eq!(parse("release"), Ok(Command::Release));
    assert_eq!(
      parse("shot"),
      Ok(Command::Shot {
        point: ShotPoint::Other,
        edge_age_10us: None
      })
    );
    assert_eq!(
      parse("shot b 120"),
      Ok(Command::Shot {
        point: ShotPoint::B,
        edge_age_10us: Some(120)
      })
    );
    let shot = |point, edge_age_10us| Ok(Command::Shot { point, edge_age_10us });
    assert_eq!(parse("shot a"), shot(ShotPoint::A, None));
    assert_eq!(parse("shot c 5"), shot(ShotPoint::C, Some(5)));
    assert_eq!(parse("shot other 7"), shot(ShotPoint::Other, Some(7)));
    assert_eq!(parse("sim delay 500"), Ok(Command::Delay(500)));
    assert_eq!(parse("sim next timeout"), Ok(Command::Next(SensorResult::TimedOut)));
    assert_eq!(parse("sim sum 150"), Ok(Command::Sum(Some(150))));
    assert_eq!(parse("sim sum off"), Ok(Command::Sum(None)));
  }

  #[test]
  fn every_result_name_parses_to_its_result() {
    let names = [
      ("valid", SensorResult::Valid),
      ("noshot", SensorResult::NoShot),
      ("saturated", SensorResult::Saturated),
      ("negative", SensorResult::NegativeQuadrant),
      ("baseline", SensorResult::BaselineUnstable),
      ("overlap", SensorResult::Overlap),
      ("timeout", SensorResult::TimedOut),
      ("frontend", SensorResult::FrontEndFault)
    ];
    for (name, expected) in names {
      assert_eq!(result(name), Ok(expected));
    }
  }

  #[test]
  fn malformed_lines_are_rejected() {
    assert_eq!(parse(""), Err(ParseError::Empty));
    assert_eq!(parse("fire"), Err(ParseError::UnknownCommand));
    assert_eq!(parse("press"), Err(ParseError::BadArguments));
    assert_eq!(parse("press e"), Err(ParseError::BadArguments));
    assert_eq!(parse("hold"), Err(ParseError::BadArguments));
    assert_eq!(parse("release now"), Err(ParseError::BadArguments));
    assert_eq!(parse("shot d"), Err(ParseError::BadArguments));
    assert_eq!(parse("shot a soon"), Err(ParseError::BadArguments));
    assert_eq!(parse("shot a 1 2"), Err(ParseError::BadArguments));
    assert_eq!(parse("sim next maybe"), Err(ParseError::BadArguments));
    assert_eq!(parse("sim delay"), Err(ParseError::BadArguments));
    assert_eq!(parse("sim sum lots"), Err(ParseError::BadArguments));
    assert_eq!(parse("bench maybe"), Err(ParseError::BadArguments));
    assert_eq!(parse("status now"), Err(ParseError::BadArguments));
  }
}
