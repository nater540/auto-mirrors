//! One bring-up procedure on the bench: the configuration a person reading a paper target needs, the checks before
//! the first move, and the run itself, its `BU1` lines written as they happen.
//!
//! - **The beam is read at one place** (`ReadAt::AOnly`), **judged with the reading's own uncertainty as the noise
//!   floor** (`--reading-mm` over the target's half-width): readings typed by a person repeat exactly.
//! - **Three repeats are kept.** Seven would give one place's noise the degrees of freedom three give at three places,
//!   but a person's repeats of a still dot are the same numbers: the floor carries the noise, not the repeats.
//! - **The poke may be set larger than the configured 45 steps**, which is sized for a 4 mm beam on a detector at 1 m
//!   (addendum A.3). The poke must move the dot well beyond the reading's uncertainty, or no direction is found.
//! - **The count must be trusted, the permit present, and the rail up** before the first move. The aligner then checks
//!   every move of the procedure against the soft limits, which count from the commissioning position, before the
//!   first one runs.

use std::io::Write;

use align_core::aligner::{AlignFault, Aligner, AxisState};
use align_core::approach::LastDir;
use align_core::axis::{Axis, PerAxis};
use align_core::bringup::{Entry, SAMPLE_COLUMNS};
use align_core::config::{Config, ReadAt};
use align_core::rig::{Actuator, BeamSensor};
use align_sim::block_on;

use crate::console::NodeStatus;
use crate::eye::SUM_REF;

/// Which measurement to run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Procedure {
  /// The backlash bracket (reference 3.4, addendum A.5).
  Backlash,
  /// The hysteresis loop (reference 3.4).
  Hysteresis
}

/// What the operator chose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
  /// The axis measured.
  pub axis: Axis,
  /// The measurement.
  pub procedure: Procedure,
  /// The target's half-width: how far from its center a dot can still be read, in millimeters.
  pub target_mm: f32,
  /// One sigma of a typed reading, in millimeters: how finely the operator reads the dot.
  pub reading_mm: f32,
  /// The poke, in full steps, or `None` for the configured one.
  pub poke_steps: Option<i32>
}

/// Why the bench cannot start.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum NotReady {
  /// The node's count of the axis is not trusted.
  #[error(
    "the node's count is suspect: set the screw to the middle of its travel and type `zero <channel>` at the node's \
     console, in bench mode"
  )]
  Suspect,
  /// The motor permit is absent.
  #[error("the motor permit is absent: fit the permit jumper")]
  NoPermit,
  /// The motor rail reads down.
  #[error("the motor rail reads down: fit the rail jumper")]
  NoRail
}

/// Checks a node's status before anything moves.
///
/// # Errors
///
/// The first [`NotReady`] that applies.
pub fn check_start(status: &NodeStatus) -> Result<(), NotReady> {
  if status.suspect {
    return Err(NotReady::Suspect);
  }
  if !status.permit {
    return Err(NotReady::NoPermit);
  }
  if !status.rail {
    return Err(NotReady::NoRail);
  }
  Ok(())
}

/// The configuration for a bench: the default one, read at one place, with the reading's uncertainty as the noise
/// floor and the poke the operator chose.
pub fn config(settings: &Settings) -> Config {
  let mut config = Config::default();
  config.bring_up.read_at = ReadAt::AOnly;
  config.bring_up.noise_floor_norm = settings.reading_mm / settings.target_mm;
  if let Some(poke_steps) = settings.poke_steps {
    config.motion.poke_steps = poke_steps;
  }
  config
}

/// The aligner's state of the measured axis, from the node's count: trusted, and its last direction unknown, so the
/// procedure's first move is a full standard approach. The other axes are not moved.
pub fn starting_axes(axis: Axis, status: &NodeStatus) -> PerAxis<AxisState> {
  let mut axes = PerAxis::splat(AxisState::default());
  *axes.get_mut(axis) = AxisState {
    pos_steps: status.pos_steps,
    last_dir: LastDir::Unknown,
    suspect: false,
    lost: false
  };
  axes
}

/// Runs the procedure, writing [`SAMPLE_COLUMNS`] and then each `BU1` line to `out` as it happens; returns the
/// report's line.
///
/// # Errors
///
/// The aligner's fault: a configuration it refuses, a move the soft limits forbid, a move that failed, a reading the
/// operator did not give, or an axis that did not move the dot.
pub fn run<A: Actuator, S: BeamSensor>(
  actuator: A,
  sensor: S,
  settings: &Settings,
  status: &NodeStatus,
  out: &mut impl Write
) -> Result<String, AlignFault> {
  let mut aligner =
    Aligner::new(actuator, sensor, config(settings), SUM_REF)?.with_axes(starting_axes(settings.axis, status));
  // The log is the record; a failed write to it must not stop a motor mid-procedure.
  let _ = writeln!(out, "{SAMPLE_COLUMNS}");
  let mut observe = |entry: &Entry| {
    let _ = writeln!(out, "{entry}");
    let _ = out.flush();
  };
  let report = match settings.procedure {
    Procedure::Backlash => block_on(aligner.measure_backlash(settings.axis, &mut observe))?.to_string(),
    Procedure::Hysteresis => block_on(aligner.measure_hysteresis(settings.axis, &mut observe))?.to_string()
  };
  Ok(report)
}

#[cfg(test)]
mod tests {
  use std::cell::RefCell;
  use std::io::{BufReader, Read};
  use std::rc::Rc;

  use node_logic::NodeState;

  use super::*;
  use crate::console::{Log, NodeConsole};
  use crate::eye::ReadByEye;
  use crate::fake_node::{FakeNode, Machine};
  use crate::motors::{BenchMotors, Motor};

  /// The most lines an operator types before walking away. The longest procedure here takes about 20 readings. A
  /// reading parser that rejects everything asks again forever, and on 2026-11-18 such a mutant filled the prompt
  /// buffer to 12 GB and took the machine down: walking away ends the procedure as `NoReading` instead.
  const OPERATOR_LINES: u32 = 500;

  /// An operator who reads the fake machine's dot to the nearest `resolution_mm`, each time a reading is asked for,
  /// up to `lines_left` times.
  struct Operator {
    machine: Machine,
    resolution_mm: f32,
    line: Vec<u8>,
    at: usize,
    lines_left: u32
  }

  impl Read for Operator {
    fn read(&mut self, buffer: &mut [u8]) -> std::io::Result<usize> {
      if self.at == self.line.len() {
        let Some(left) = self.lines_left.checked_sub(1) else {
          return Ok(0);
        };
        self.lines_left = left;
        let (h_mm, v_mm) = self.machine.dot_mm();
        let round = |mm: f32| (mm / self.resolution_mm).round() * self.resolution_mm;
        self.line = format!("{} {}\n", round(h_mm), round(v_mm)).into_bytes();
        self.at = 0;
      }
      let count = buffer.len().min(self.line.len() - self.at);
      buffer[..count].copy_from_slice(&self.line[self.at..self.at + count]);
      self.at += count;
      Ok(count)
    }
  }

  fn settings(procedure: Procedure) -> Settings {
    Settings {
      axis: Axis::M1H,
      procedure,
      target_mm: 100.0,
      reading_mm: 0.1,
      poke_steps: None
    }
  }

  /// Runs `procedure` against a fake node whose screw has `play_steps` of play, read by eye to 0.1 mm.
  fn bench(play_steps: i32, procedure: Procedure) -> (Result<String, AlignFault>, String, Machine) {
    let machine = Machine::with_play(play_steps, 0.05);
    let log: Log = Rc::new(RefCell::new(Vec::<u8>::new()));
    let mut console = NodeConsole::new(FakeNode::spawn(machine.clone()), "node-a", log.clone());
    let status = console.status(0).unwrap();
    console.bench(true).unwrap();
    let settings = settings(procedure);
    let motion = config(&settings).motion;
    let motor = Motor {
      axis: Axis::M1H,
      console: 0,
      channel: 0
    };
    let mut motors = BenchMotors::new(vec![console], vec![motor], &motion);
    let operator = Operator {
      machine: machine.clone(),
      resolution_mm: 0.1,
      line: Vec::new(),
      at: 0,
      lines_left: OPERATOR_LINES
    };
    let eye = ReadByEye::new(
      BufReader::new(operator),
      Vec::new(),
      log,
      100.0,
      &config(&settings).sensor
    );
    let mut out = Vec::new();
    let report = run(&mut motors, eye, &settings, &status, &mut out);
    (report, String::from_utf8_lossy(&out).into_owned(), machine)
  }

  #[test]
  fn the_backlash_bracket_holds_the_screw_s_play_read_by_eye() {
    for play in [20, 40, 80] {
      let (report, lines, _) = bench(play, Procedure::Backlash);
      let report = report.unwrap();
      assert!(report.starts_with("BU1,backlash,m1h"), "{report}");
      // The bracket's ends are in the report; the trials that moved and did not are among the lines.
      assert!(lines.contains("BU1,trial,m1h"), "{lines}");
      let field = |name: &str| -> f32 {
        report
          .split(',')
          .find_map(|field| field.strip_prefix(name))
          .and_then(|value| value.parse().ok())
          .unwrap()
      };
      let (above, up_to) = (field("play_above_steps="), field("play_up_to_steps="));
      #[expect(clippy::cast_precision_loss, reason = "a play of a few dozen steps")]
      let play = play as f32;
      assert!(up_to > play, "play {play}: {report}");
      assert!(above < play + 2.0 * field("resolution_steps="), "play {play}: {report}");
    }
  }

  #[test]
  fn the_hysteresis_loop_read_by_eye_is_as_wide_as_the_play() {
    let (report, _, _) = bench(40, Procedure::Hysteresis);
    let report = report.unwrap();
    let width: f32 = report
      .split(',')
      .find_map(|field| field.strip_prefix("width_steps="))
      .and_then(|value| value.parse().ok())
      .unwrap();
    assert!((width - 40.0).abs() < 4.0, "{report}");
  }

  #[test]
  fn the_procedure_leaves_the_motor_where_it_started_and_moves_nothing_else() {
    let (_, _, machine) = bench(40, Procedure::Backlash);
    let jogs: Vec<String> = machine
      .received()
      .into_iter()
      .filter(|line| line.starts_with("jog"))
      .collect();
    assert!(jogs.iter().all(|line| line.starts_with("jog 0 ")), "{jogs:?}");
    let net: i32 = jogs
      .iter()
      .filter_map(|line| line.split_whitespace().nth(2)?.parse::<i32>().ok())
      .sum();
    assert_eq!(net, 0, "{jogs:?}");
  }

  #[test]
  fn the_lines_start_with_the_columns_of_a_sample() {
    let (_, lines, _) = bench(20, Procedure::Hysteresis);
    assert_eq!(lines.lines().next(), Some(SAMPLE_COLUMNS));
    assert!(lines.lines().any(|line| line.starts_with("BU1,sample,m1h,")), "{lines}");
  }

  #[test]
  fn the_bench_reads_at_one_place_with_the_reading_as_its_noise_floor() {
    let mut settings = settings(Procedure::Backlash);
    settings.poke_steps = Some(200);
    let config = config(&settings);
    assert_eq!(config.bring_up.read_at, ReadAt::AOnly);
    assert!((config.bring_up.noise_floor_norm - 0.001).abs() < 1e-9);
    assert_eq!(config.motion.poke_steps, 200);
    assert_eq!(config.bring_up.repeat_shots, Config::default().bring_up.repeat_shots);
  }

  #[test]
  fn nothing_starts_on_a_suspect_count_without_the_permit_or_with_the_rail_down() {
    let good = NodeStatus {
      state: NodeState::Bench,
      permit: true,
      rail: true,
      pos_steps: 0,
      suspect: false
    };
    assert_eq!(check_start(&good), Ok(()));
    assert_eq!(
      check_start(&NodeStatus { suspect: true, ..good }),
      Err(NotReady::Suspect)
    );
    assert_eq!(
      check_start(&NodeStatus { permit: false, ..good }),
      Err(NotReady::NoPermit)
    );
    assert_eq!(check_start(&NodeStatus { rail: false, ..good }), Err(NotReady::NoRail));
  }

  #[test]
  fn the_operator_stopping_ends_the_procedure_with_no_reading() {
    let machine = Machine::with_play(20, 0.05);
    let log: Log = Rc::new(RefCell::new(Vec::<u8>::new()));
    let mut console = NodeConsole::new(FakeNode::spawn(machine.clone()), "node-a", log.clone());
    let status = console.status(0).unwrap();
    console.bench(true).unwrap();
    let settings = settings(Procedure::Backlash);
    let motor = Motor {
      axis: Axis::M1H,
      console: 0,
      channel: 0
    };
    let mut motors = BenchMotors::new(vec![console], vec![motor], &config(&settings).motion);
    let eye = ReadByEye::new(
      std::io::Cursor::new(b"0 0\nq\n".to_vec()),
      Vec::new(),
      log,
      100.0,
      &config(&settings).sensor
    );
    let report = run(&mut motors, eye, &settings, &status, &mut Vec::new());
    assert_eq!(report, Err(AlignFault::Sensor(align_core::rig::SensorFault::NoReading)));
  }
}
