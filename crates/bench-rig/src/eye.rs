//! The beam read by eye: after each move, the operator types where the dot sits on a paper target, and that is the
//! measurement's reading at A. B and C are not read on a bench (`config::ReadAt::AOnly`); they are given as a centered
//! beam, which every check passes and the bring-up leaves out.
//!
//! - **Millimeters on the target, normalized by the target's half-width** (`target_mm`): a reading of `target_mm`
//!   is 1. The bring-up's results are in steps and do not depend on it; it sets which readings are on the target.
//! - **A reading must be two finite numbers inside the target's usable area** (the sensor's edge limit times the
//!   half-width), or it is asked for again. `q`, or the end of the input, stops the procedure as a measurement with no
//!   reading.

use std::io::{BufRead, Write};

use align_core::config::SensorLimits;
use align_core::measurement::Shots;
use align_core::quadrant::Reading;
use align_core::rig::{BeamSensor, SensorFault};

use crate::console::Log;

/// The sum every typed reading carries: the reference sum the aligner is given, so no reading is short of signal.
pub const SUM_REF: f32 = 1.0;

/// The readings typed at a terminal.
pub struct ReadByEye<I, O> {
  input: I,
  prompt: O,
  log: Log,
  target_mm: f32,
  edge_mm: f32,
  taken: u32
}

impl<I: BufRead, O: Write> ReadByEye<I, O> {
  /// Readings typed on `input` after a prompt on `prompt`, on a target `target_mm` from its center to its edge.
  pub fn new(input: I, prompt: O, log: Log, target_mm: f32, limits: &SensorLimits) -> Self {
    Self {
      input,
      prompt,
      log,
      target_mm,
      edge_mm: limits.edge_max_norm * target_mm,
      taken: 0
    }
  }

  /// How many readings were taken.
  pub fn taken(&self) -> u32 {
    self.taken
  }

  /// Asks until a usable reading is typed; `None` when the operator stops.
  fn ask(&mut self) -> Option<(f32, f32)> {
    loop {
      let _ = write!(
        self.prompt,
        "reading {}: where is the dot? h v in mm (h right, v up; q stops): ",
        self.taken + 1
      );
      let _ = self.prompt.flush();
      let mut line = String::new();
      match self.input.read_line(&mut line) {
        Ok(0) | Err(_) => return None,
        Ok(_) => {}
      }
      match parse_dot(&line, self.edge_mm) {
        Typed::Stop => return None,
        Typed::Dot(h_mm, v_mm) => {
          self.taken += 1;
          let _ = writeln!(self.log.borrow_mut(), "eye reading {}: {h_mm} {v_mm} mm", self.taken);
          return Some((h_mm, v_mm));
        }
        Typed::Unusable(why) => {
          let _ = writeln!(self.prompt, "  {why}");
        }
      }
    }
  }
}

/// What the operator typed.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Typed {
  /// The dot, horizontal and vertical, in millimeters.
  Dot(f32, f32),
  /// Stop the procedure.
  Stop,
  /// Not a reading, and why.
  Unusable(&'static str)
}

fn parse_dot(line: &str, edge_mm: f32) -> Typed {
  let line = line.trim();
  if line.eq_ignore_ascii_case("q") {
    return Typed::Stop;
  }
  let mut words = line
    .split(|c: char| c.is_whitespace() || c == ',')
    .filter(|word| !word.is_empty());
  let (Some(h), Some(v), None) = (words.next(), words.next(), words.next()) else {
    return Typed::Unusable("type two numbers, h then v, in millimeters");
  };
  let (Ok(h_mm), Ok(v_mm)) = (h.parse::<f32>(), v.parse::<f32>()) else {
    return Typed::Unusable("type two numbers, h then v, in millimeters");
  };
  if !(h_mm.is_finite() && v_mm.is_finite()) {
    return Typed::Unusable("type two numbers, h then v, in millimeters");
  }
  if h_mm.abs() > edge_mm || v_mm.abs() > edge_mm {
    return Typed::Unusable("that is beyond the target's usable area: re-aim, or give a larger --target-mm");
  }
  Typed::Dot(h_mm, v_mm)
}

impl<I: BufRead, O: Write> BeamSensor for ReadByEye<I, O> {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    self.read()
  }
}

/// Lent to the aligner, so that the count of readings is still there after the procedure.
impl<I: BufRead, O: Write> BeamSensor for &mut ReadByEye<I, O> {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    self.read()
  }
}

impl<I: BufRead, O: Write> ReadByEye<I, O> {
  fn read(&mut self) -> Result<Shots, SensorFault> {
    let (h_mm, v_mm) = self.ask().ok_or(SensorFault::NoReading)?;
    let center = Reading {
      h_norm: 0.0,
      v_norm: 0.0,
      sum: SUM_REF
    };
    Ok(Shots {
      a: Reading {
        h_norm: h_mm / self.target_mm,
        v_norm: v_mm / self.target_mm,
        sum: SUM_REF
      },
      b: center,
      c: center
    })
  }
}

#[cfg(test)]
mod tests {
  use std::cell::RefCell;
  use std::io::Cursor;
  use std::rc::Rc;

  use align_core::config::Config;
  use align_sim::block_on;

  use super::*;

  fn eye(typed: &str) -> ReadByEye<Cursor<Vec<u8>>, Vec<u8>> {
    let log: Log = Rc::new(RefCell::new(Vec::<u8>::new()));
    ReadByEye::new(
      Cursor::new(typed.as_bytes().to_vec()),
      Vec::new(),
      log,
      100.0,
      &Config::default().sensor
    )
  }

  #[test]
  fn a_reading_is_two_numbers_in_millimeters_normalized_by_the_target() {
    let mut eye = eye("12.5 -4\n");
    let shots = block_on(eye.measure()).unwrap();
    assert_eq!(
      shots.a,
      Reading {
        h_norm: 0.125,
        v_norm: -0.04,
        sum: SUM_REF
      }
    );
    assert_eq!(
      (shots.b.h_norm, shots.c.v_norm),
      (0.0, 0.0),
      "B and C are a centered beam"
    );
    assert_eq!(eye.taken(), 1);
  }

  #[test]
  fn what_is_not_a_reading_is_asked_for_again() {
    let mut eye = eye("\n12\nabc 3\n1 2 3\nNaN 0\ninf 1\n95 0\n0 -91\n1.5,2\n");
    let shots = block_on(eye.measure()).unwrap();
    assert_eq!((shots.a.h_norm, shots.a.v_norm), (0.015, 0.02));
    let asked = String::from_utf8_lossy(&eye.prompt).into_owned();
    assert_eq!(asked.matches("where is the dot").count(), 9, "{asked}");
    assert_eq!(asked.matches("beyond the target").count(), 2, "{asked}");
  }

  #[test]
  fn only_usable_readings_are_counted() {
    let mut eye = eye("1 2\nabc\n3 4\n");
    block_on(eye.measure()).unwrap();
    block_on(eye.measure()).unwrap();
    assert_eq!(eye.taken(), 2);
  }

  #[test]
  fn q_or_the_end_of_the_input_stops_with_no_reading() {
    assert_eq!(block_on(eye("q\n").measure()), Err(SensorFault::NoReading));
    assert_eq!(block_on(eye("").measure()), Err(SensorFault::NoReading));
    assert_eq!(block_on(eye("x\n").measure()), Err(SensorFault::NoReading));
  }

  #[test]
  fn the_usable_area_is_the_sensor_s_edge_limit_of_the_target() {
    // edge_max_norm 0.9 of a 100 mm half-width.
    assert_eq!(parse_dot("90 -90", 90.0), Typed::Dot(90.0, -90.0));
    assert!(matches!(parse_dot("90.01 0", 90.0), Typed::Unusable(_)));
  }
}
