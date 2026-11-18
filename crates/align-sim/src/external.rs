//! A machine outside this program, behind the `Actuator` and `BeamSensor` traits: every move and measurement the
//! aligner asks for is one line written out, answered by one line read back. `tools/beamviz` answers from a
//! ray-traced machine of its own, so the aligner of `align-core` runs unchanged against optics written apart from
//! [`crate::world`].
//!
//! One exchange at a time:
//!
//! | Asked | Answered |
//! |---|---|
//! | `move <axis> <delta_steps>` | `done <pos_steps>`: the node's count after the move |
//! | `measure` | `shots` and twelve numbers: quadrants a, b, c, d at A, then at B, then at C |
//!
//! Quadrant a is upper right, b upper left, c lower left, and d lower right ([`Quadrants`]); they are normalized
//! here, by the code the coordinator runs. After the alignment, [`External::finish`] writes a `report` line, a
//! `calibration` line with the Jacobian the aligner measured, and `end`.
//!
//! An answer that is missing or cannot be read ends the move as [`MoveFault::NoAnswer`] or the measurement as
//! [`SensorFault::NoReading`], as a silent node or sensor would, and the aligner faults.

use std::cell::RefCell;
use std::fmt;
use std::io::{BufRead, Write};
use std::rc::Rc;

use align_core::aligner::{AlignFault, AlignReport};
use align_core::axis::Axis;
use align_core::calibration::Calibration;
use align_core::measurement::Shots;
use align_core::quadrant::{Quadrants, Reading};
use align_core::rig::{Actuator, BeamSensor, MoveFault, SensorFault};

/// The other program, shared by the aligner's actuator and sensor: clone it once for each role.
pub struct External<R, W>(Rc<RefCell<Link<R, W>>>);

impl<R, W> Clone for External<R, W> {
  fn clone(&self) -> Self {
    Self(Rc::clone(&self.0))
  }
}

/// The two ends of the conversation.
struct Link<R, W> {
  input: R,
  output: W,
  line: String
}

impl<R: BufRead, W: Write> External<R, W> {
  /// Talk to the other program through `input` (its answers) and `output` (the questions).
  pub fn new(input: R, output: W) -> Self {
    Self(Rc::new(RefCell::new(Link {
      input,
      output,
      line: String::new()
    })))
  }

  /// Write one question and read its answer, without the line end. `None` if either fails or the answer is empty.
  fn ask(&self, question: fmt::Arguments<'_>) -> Option<String> {
    let mut link = self.0.borrow_mut();
    writeln!(link.output, "{question}").ok()?;
    link.output.flush().ok()?;
    let Link { input, line, .. } = &mut *link;
    line.clear();
    match input.read_line(line) {
      Ok(0) | Err(_) => None,
      Ok(_) => Some(line.trim().to_owned()).filter(|answer| !answer.is_empty())
    }
  }

  /// Write how the alignment ended and what it measured, then `end`.
  ///
  /// # Errors
  ///
  /// A failed write.
  pub fn finish(
    &self,
    outcome: &Result<AlignReport, AlignFault>,
    calibration: Option<&Calibration>
  ) -> std::io::Result<()> {
    let mut link = self.0.borrow_mut();
    let output = &mut link.output;
    match outcome {
      Ok(report) => writeln!(
        output,
        "report converged iterations={} recalibrations={} residual={:.5} cond={:.3} head_h={:+.4} head_v={:+.4} \
         centering={}",
        report.iterations,
        report.recalibrations,
        report.residual_norm,
        report.cond,
        report.head_offset.h_norm,
        report.head_offset.v_norm,
        if report.needs_mechanical_centering { "yes" } else { "no" }
      )?,
      Err(fault) => writeln!(output, "report fault {fault}")?
    }
    match calibration {
      Some(calibration) => {
        let entries: Vec<String> = calibration
          .jacobian
          .row_iter()
          .flat_map(|row| row.iter().map(|value| format!("{value:e}")).collect::<Vec<_>>())
          .collect();
        writeln!(output, "calibration {}", entries.join(" "))?;
      }
      None => writeln!(output, "calibration none")?
    }
    writeln!(output, "end")?;
    output.flush()
  }
}

/// The twelve quadrant values of a `shots` answer, as three readings.
fn shots(answer: &str) -> Option<Shots> {
  let mut words = answer.split_whitespace();
  if words.next() != Some("shots") {
    return None;
  }
  let mut values = [0.0_f32; 12];
  for value in &mut values {
    *value = words.next()?.parse().ok()?;
  }
  if words.next().is_some() {
    return None;
  }
  let reading = |at: usize| -> Option<Reading> {
    let quadrants = Quadrants {
      a: *values.get(at)?,
      b: *values.get(at + 1)?,
      c: *values.get(at + 2)?,
      d: *values.get(at + 3)?
    };
    // With no usable signal a sensor node still reports: a zero position and the sum it saw, which the aligner's
    // checks then reject (as `crate::world` does).
    Some(quadrants.normalize().unwrap_or(Reading {
      h_norm: 0.0,
      v_norm: 0.0,
      sum: quadrants.sum().max(0.0)
    }))
  };
  Some(Shots {
    a: reading(0)?,
    b: reading(4)?,
    c: reading(8)?
  })
}

impl<R: BufRead, W: Write> Actuator for External<R, W> {
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    let answer = self
      .ask(format_args!("move {} {delta_steps}", axis.name()))
      .ok_or(MoveFault::NoAnswer)?;
    answer
      .strip_prefix("done ")
      .and_then(|count| count.trim().parse().ok())
      .ok_or(MoveFault::NoAnswer)
  }
}

impl<R: BufRead, W: Write> BeamSensor for External<R, W> {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    self
      .ask(format_args!("measure"))
      .as_deref()
      .and_then(shots)
      .ok_or(SensorFault::NoReading)
  }
}

#[cfg(test)]
mod tests {
  use std::io::{BufReader, Cursor, PipeReader, PipeWriter};
  use std::thread;

  use align_core::aligner::Aligner;
  use align_core::axis::PerAxis;
  use align_core::config::Config;

  use super::*;
  use crate::block_on;
  use crate::world::{Scenario, World};

  /// An external machine whose answers are `answers`, and what it was asked.
  fn scripted(answers: &str) -> External<Cursor<Vec<u8>>, Vec<u8>> {
    External::new(Cursor::new(answers.as_bytes().to_vec()), Vec::new())
  }

  fn asked(external: &External<Cursor<Vec<u8>>, Vec<u8>>) -> String {
    String::from_utf8(external.0.borrow().output.clone()).unwrap()
  }

  #[test]
  fn a_move_is_one_line_and_its_answer_is_the_node_s_count() {
    let mut external = scripted("done -40\n");
    assert_eq!(block_on(external.move_rel(Axis::M2V, -12)), Ok(-40));
    assert_eq!(asked(&external), "move m2v -12\n");
  }

  #[test]
  fn a_move_without_a_readable_answer_has_no_answer() {
    for answers in ["", "\n", "done\n", "done twelve\n", "shots 1\n"] {
      let mut external = scripted(answers);
      assert_eq!(
        block_on(external.move_rel(Axis::M1H, 5)),
        Err(MoveFault::NoAnswer),
        "{answers:?}"
      );
    }
  }

  #[test]
  fn a_measurement_normalizes_the_twelve_quadrants_into_three_readings() {
    // A: centered. B: all on the right half (a and d). C: all on the upper half (a and b), at half the power.
    let mut external = scripted("shots 1 1 1 1  2 0 0 2  1 1 0 0\n");
    let shots = block_on(external.measure()).unwrap();
    assert_eq!(asked(&external), "measure\n");
    assert_eq!((shots.a.h_norm, shots.a.v_norm, shots.a.sum), (0.0, 0.0, 4.0));
    assert_eq!((shots.b.h_norm, shots.b.v_norm, shots.b.sum), (1.0, 0.0, 4.0));
    assert_eq!((shots.c.h_norm, shots.c.v_norm, shots.c.sum), (0.0, 1.0, 2.0));
  }

  #[test]
  fn a_dark_shot_reads_as_zero_at_the_center_with_the_sum_it_saw() {
    let mut external = scripted("shots 1 1 1 1  0 0 0 0  1 1 1 1\n");
    let shots = block_on(external.measure()).unwrap();
    assert_eq!((shots.b.h_norm, shots.b.v_norm, shots.b.sum), (0.0, 0.0, 0.0));
  }

  #[test]
  fn a_measurement_without_twelve_numbers_has_no_reading() {
    for answers in [
      "",
      "shots 1 1 1 1 1 1 1 1 1 1 1\n",
      "shots 1 1 1 1 1 1 1 1 1 1 1 1 1\n",
      "done 3\n",
      "shots x 1 1 1 1 1 1 1 1 1 1 1\n"
    ] {
      let mut external = scripted(answers);
      assert_eq!(block_on(external.measure()), Err(SensorFault::NoReading), "{answers:?}");
    }
  }

  #[test]
  fn the_finish_says_how_it_ended_and_what_was_measured() {
    let external = scripted("");
    external
      .finish(&Err(AlignFault::PositionUnknown { axis: Axis::M1V }), None)
      .unwrap();
    let written = asked(&external);
    let lines: Vec<&str> = written.lines().collect();
    assert!(lines[0].starts_with("report fault "), "{}", lines[0]);
    assert_eq!(lines[1..], ["calibration none", "end"]);
  }

  #[test]
  fn a_measured_calibration_is_written_row_by_row() {
    let external = scripted("");
    let report = AlignReport {
      iterations: 2,
      recalibrations: 0,
      residual_norm: 0.0124,
      cond: 3.6,
      head_offset: align_core::measurement::HeadOffset {
        h_norm: -0.05,
        v_norm: 0.03
      },
      needs_mechanical_centering: false
    };
    let calibration = Calibration {
      jacobian: align_core::solver::Jacobian::from_fn(|row, column| {
        #[expect(clippy::cast_precision_loss, reason = "indices below four")]
        let entry = (row * 4 + column) as f32;
        entry
      }),
      q0_steps: [0; 4],
      cond: 3.6,
      dir_sign: Config::default().mechanics.dir_sign
    };
    external.finish(&Ok(report), Some(&calibration)).unwrap();
    let written = asked(&external);
    let lines: Vec<&str> = written.lines().collect();
    assert_eq!(
      lines[0],
      "report converged iterations=2 recalibrations=0 residual=0.01240 cond=3.600 head_h=-0.0500 head_v=+0.0300 \
       centering=no"
    );
    let entries: Vec<f32> = lines[1]
      .split_whitespace()
      .skip(1)
      .map(|word| word.parse().unwrap())
      .collect();
    let expected: Vec<f32> = (0..16_u8).map(f32::from).collect();
    assert_eq!(entries, expected);
    assert_eq!(lines[2], "end");
  }

  /// The quadrants that normalize to `reading`.
  fn quadrants(reading: Reading) -> [f32; 4] {
    let quarter = reading.sum / 4.0;
    let (h, v) = (reading.h_norm, reading.v_norm);
    [
      quarter * (1.0 + h + v),
      quarter * (1.0 - h + v),
      quarter * (1.0 - h - v),
      quarter * (1.0 + h - v)
    ]
  }

  /// Answers the questions on `questions` from the simulated machine, until `end`.
  fn serve(mut world: World, questions: PipeReader, mut answers: PipeWriter) -> World {
    let mut lines = BufReader::new(questions).lines();
    while let Some(Ok(line)) = lines.next() {
      let words: Vec<&str> = line.split_whitespace().collect();
      match words.as_slice() {
        ["move", axis, delta] => {
          let axis = Axis::ALL
            .into_iter()
            .find(|candidate| candidate.name() == *axis)
            .unwrap();
          world.move_rel(axis, delta.parse().unwrap());
          writeln!(answers, "done {}", world.commanded_steps(axis)).unwrap();
        }
        ["measure"] => {
          let shots = world.measure();
          let values: Vec<String> = [shots.a, shots.b, shots.c]
            .into_iter()
            .flat_map(quadrants)
            .map(|value| value.to_string())
            .collect();
          writeln!(answers, "shots {}", values.join(" ")).unwrap();
        }
        ["end"] => break,
        _ => {}
      }
    }
    world
  }

  #[test]
  fn the_aligner_converges_through_the_lines_as_it_does_against_the_model_directly() {
    let mut scenario = Scenario::realistic(3);
    scenario.misalignment_steps = PerAxis {
      m1_h: 25.0,
      m1_v: -20.0,
      m2_h: -35.0,
      m2_v: 30.0
    };
    let (question_reader, question_writer) = std::io::pipe().unwrap();
    let (answer_reader, answer_writer) = std::io::pipe().unwrap();
    let server = thread::spawn(move || serve(World::new(scenario), question_reader, answer_writer));
    let external = External::new(BufReader::new(answer_reader), question_writer);
    #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
    let sum_ref = scenario.beam.sum_ref as f32;
    let mut aligner = Aligner::new(external.clone(), external.clone(), Config::default(), sum_ref).unwrap();
    let outcome = block_on(aligner.align());
    external.finish(&outcome, aligner.calibration()).unwrap();
    let world = server.join().unwrap();
    assert!(outcome.is_ok(), "{outcome:?}");
    assert!(world.true_shots().measurement().amax() < 0.03);
  }
}
