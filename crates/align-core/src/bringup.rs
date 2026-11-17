//! Bring-up measurements of one axis through the beam (milestone M6, reference 3.4): its backlash, and its
//! hysteresis.
//!
//! The procedures are methods of the aligner, [`Aligner::measure_backlash`] and [`Aligner::measure_hysteresis`]:
//! they move and measure through the same traits as the alignment, so the coordinator runs them over the bus and the
//! simulator against its model. This module holds what they report, how a measurement is reduced to one number, and
//! the lines they are logged as ([`Sample`], [`Trial`], and the two reports print them).
//!
//! Both begin as reference 3.4 does: a standard approach to the starting position, run even when the axis last
//! moved plus so that the repeats follow a move, then [`BringUp::repeat_shots`] measurements there, for the noise
//! and the drift.
//!
//! - **Backlash** (reference 3.4, with the search of addendum A.5). A plus poke finds which way the axis moves the
//!   beam and how far per step, and is undone. Each trial then moves the axis back by `b` steps, measures, returns it
//!   by `b` without overshoot, and measures again, for `b` = 8, 16, 32, 64, 128 until the beam moves. One more
//!   trial halfway between the last that did not and the first that did narrows the bracket ("bisect once").
//!   Backlash and seat stick-slip together are more than the bracket's low end and at most its high end, as finely
//!   as the noise allows: the report gives that resolution in steps.
//! - **Hysteresis** (reference 3.4). The axis cycles from two poke sizes below its starting position to two above
//!   and back, in eight moves each way, with a measurement after each. The loop's width in steps is the effective
//!   hysteresis, a jump right after a reversal is seat stick-slip, and repeats that drift ask for a longer settle.
//!
//! What the documents leave open, and what is done here:
//!
//! - **A measurement is reduced to the six coordinates of its readings** (h and v at A, B, and C), not to the
//!   difference vector: one axis moves all six, and the absolute readings carry the most of its motion.
//! - **Motion is projected onto the axis's own direction** among the six, found by a plus move (the poke, or the
//!   plus half of the cycle). Noise across that direction is discarded, so the projection has the noise of one
//!   coordinate.
//! - **The noise is measured** from the repeats (each coordinate's deviation from its mean, pooled over the six),
//!   not taken from the configuration, whose noise is a placeholder. "More than 3 sigma" is read as 3 sigmas of a
//!   shift, the difference of two measurements: `3 * sqrt(2)` sigmas of a coordinate.
//! - **Each trial is judged against the measurement just before it**, the previous trial's return, not against the
//!   first: a sticking seat leaves the mirror a step or two from where it was after a return, which would otherwise
//!   read as motion in the next trial.
//! - **A trial counts as moving the beam only if it does so twice**: one that moves it is run again at once. Three
//!   repeats give the noise only to 12 degrees of freedom, and in the simulator a single trial called noise motion
//!   in one run in 40, which reports the play too small and the overshoot with it. Motion the confirmation misses
//!   pushes the bracket up instead, which errs toward a larger overshoot.
//! - **The poke and the cycle must move the beam** by more than `detect_sigmas` times the noise of a shift over all
//!   six coordinates. An axis that does not is reported as `AlignFault::WeakColumn`, as a dead axis is when
//!   calibrating: no direction can be found for it.
//! - **The loop's width is read at its middle level**, halfway between its lowest and highest projected reading,
//!   where each half of the cycle crosses it, interpolating between measurements. It needs the loop to be open
//!   there: a backlash larger than the cycle leaves the beam still, and no width.
//! - **A reading outside the solver's linear range is used**, as long as it passes the validity checks: the beam's
//!   response is monotonic there, which is all the backlash search needs, and the loop's width is read in its middle.
//!
//! The log lines are comma separated and start with [`LOG_TAG`], so that they can be found in a console capture and
//! pasted back for analysis. A procedure gives each [`Entry`] to its caller as it happens, so that a run cut short
//! still leaves its data; [`SAMPLE_COLUMNS`] names the fields of a sample line, and the trials and reports name each
//! of theirs.
//!
//! [`Aligner::measure_backlash`]: crate::aligner::Aligner::measure_backlash
//! [`Aligner::measure_hysteresis`]: crate::aligner::Aligner::measure_hysteresis
//! [`BringUp::repeat_shots`]: crate::config::BringUp::repeat_shots

use core::f32::consts::SQRT_2;
use core::fmt;

use crate::axis::Axis;
use crate::measurement::Shots;

/// The first field of every bring-up log line, with the version of the format.
pub const LOG_TAG: &str = "BU1";

/// The fields of a sample line, as a line of their own: the procedures' first line.
pub const SAMPLE_COLUMNS: &str =
  "BU1,columns,axis,phase,index,offset_steps,pos_steps,a_h,a_v,a_sum,b_h,b_v,b_sum,c_h,c_v,c_sum";

/// The most repeats a procedure takes at its start.
pub const MAX_REPEAT_SHOTS: u8 = 8;

/// The most moves in each half of the hysteresis cycle.
pub const MAX_MOVES_PER_HALF: u8 = 16;

/// The six coordinates of a measurement: h and v at A, then at B, then at C.
pub type Coordinates = [f32; 6];

/// What a measurement of a bring-up procedure was taken for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
  /// At the starting position, after the standard approach: the noise and the drift.
  Repeat,
  /// After the plus poke that finds the axis's direction.
  Poke,
  /// At the starting position again, the poke undone: what the first backlash trial is judged against.
  Reference,
  /// A backlash trial, moved back by its steps.
  Out,
  /// A backlash trial, returned by its steps: what the next trial is judged against.
  Back,
  /// The hysteresis cycle, going plus.
  Up,
  /// The hysteresis cycle, going minus.
  Down
}

impl Phase {
  /// The phase's name in the log.
  pub const fn name(self) -> &'static str {
    match self {
      Phase::Repeat => "repeat",
      Phase::Poke => "poke",
      Phase::Reference => "reference",
      Phase::Out => "out",
      Phase::Back => "back",
      Phase::Up => "up",
      Phase::Down => "down"
    }
  }
}

/// One measurement of a bring-up procedure, as it is logged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
  /// The axis measured.
  pub axis: Axis,
  /// What the measurement was taken for.
  pub phase: Phase,
  /// For [`Phase::Out`] and [`Phase::Back`], the trial's steps; otherwise the measurement's place in its phase,
  /// from 0.
  pub index: i32,
  /// The node's count less the count at the starting position.
  pub offset_steps: i32,
  /// The node's count.
  pub pos_steps: i32,
  /// The readings.
  pub shots: Shots
}

impl fmt::Display for Sample {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{LOG_TAG},sample,{},{},{},{},{}",
      self.axis.name(),
      self.phase.name(),
      self.index,
      self.offset_steps,
      self.pos_steps
    )?;
    for reading in [self.shots.a, self.shots.b, self.shots.c] {
      write!(f, ",{:.5},{:.5},{:.4}", reading.h_norm, reading.v_norm, reading.sum)?;
    }
    Ok(())
  }
}

/// How repeatable the starting position read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Repeatability {
  /// One sigma of a coordinate, pooled over the six: the sensor's noise with the motors still.
  pub sigma_norm: f32,
  /// The last repeat less the first, along the axis's direction: a mirror still moving after the settle time.
  pub drift_norm: f32
}

/// What a procedure logs as it goes: each measurement, and each backlash trial as it is judged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Entry {
  /// A measurement.
  Sample(Sample),
  /// A backlash trial, after its two measurements.
  Trial(Trial)
}

impl fmt::Display for Entry {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Entry::Sample(sample) => sample.fmt(f),
      Entry::Trial(trial) => trial.fmt(f)
    }
  }
}

/// One backlash trial: back by its steps, and plus again by as many.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Trial {
  /// The axis.
  pub axis: Axis,
  /// Steps moved back, and returned.
  pub steps: i32,
  /// How far the beam moved back along the axis's direction, from the measurement before the trial.
  pub shift_norm: f32,
  /// Where the return left the beam along the direction, from the measurement before the trial: a seat that did
  /// not come back.
  pub return_norm: f32,
  /// The trial's steps less the steps of motion its shift shows: the play, for a trial that moved the beam.
  pub play_steps: f32,
  /// The shift was more than the threshold. The trial is run again when it was, and the beam must move both times.
  pub moved: bool
}

impl fmt::Display for Trial {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{LOG_TAG},trial,{},steps={},shift_norm={:+.5},return_norm={:+.5},play_steps={:.1},moved={}",
      self.axis.name(),
      self.steps,
      self.shift_norm,
      self.return_norm,
      self.play_steps,
      if self.moved { "yes" } else { "no" }
    )
  }
}

/// The play the backlash trials found: backlash and seat stick-slip together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Play {
  /// More than `above_steps`, the largest trial that did not move the beam (0 if the first did), and at most
  /// `up_to_steps`, the smallest that did.
  Bracket {
    /// The low end, exclusive.
    above_steps: i32,
    /// The high end, inclusive.
    up_to_steps: i32
  },
  /// No trial moved the beam: the play is more than the largest.
  Beyond {
    /// The largest trial.
    steps: i32
  }
}

impl Play {
  /// Addendum A.5's overshoot for this play: twice its high end, and never less than `floor_steps`. `None` for a play
  /// beyond the trials, which no overshoot is known to take up.
  pub fn overshoot_steps(self, floor_steps: i32) -> Option<i32> {
    match self {
      Play::Bracket { up_to_steps, .. } => Some(up_to_steps.saturating_mul(2).max(floor_steps)),
      Play::Beyond { .. } => None
    }
  }
}

/// What a backlash measurement found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BacklashReport {
  /// The axis.
  pub axis: Axis,
  /// The node's count at the starting position.
  pub start_steps: i32,
  /// The noise and the drift at the starting position.
  pub repeatability: Repeatability,
  /// How far the beam moves along the axis's direction per step, from the poke.
  pub slope_norm_per_step: f32,
  /// A trial moved the beam when its shift was more than this.
  pub threshold_norm: f32,
  /// What the trials found.
  pub play: Play,
  /// Addendum A.5's overshoot for the play ([`Play::overshoot_steps`]).
  pub overshoot_steps: Option<i32>
}

impl BacklashReport {
  /// The least motion, in steps, a trial tells from noise: the threshold over the slope.
  pub fn resolution_steps(&self) -> f32 {
    self.threshold_norm / self.slope_norm_per_step
  }
}

impl fmt::Display for BacklashReport {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{LOG_TAG},backlash,{},start_steps={},sigma_norm={:.5},drift_norm={:+.5},slope_norm_per_step={:.6},\
       threshold_norm={:.5},resolution_steps={:.1},",
      self.axis.name(),
      self.start_steps,
      self.repeatability.sigma_norm,
      self.repeatability.drift_norm,
      self.slope_norm_per_step,
      self.threshold_norm,
      self.resolution_steps()
    )?;
    match self.play {
      Play::Bracket {
        above_steps,
        up_to_steps
      } => write!(f, "play_above_steps={above_steps},play_up_to_steps={up_to_steps},")?,
      Play::Beyond { steps } => write!(f, "play_beyond_steps={steps},")?
    }
    match self.overshoot_steps {
      Some(steps) => write!(f, "overshoot_steps={steps}"),
      None => write!(f, "overshoot_steps=none")
    }
  }
}

/// What a hysteresis cycle found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HysteresisReport {
  /// The axis.
  pub axis: Axis,
  /// The node's count at the starting position.
  pub start_steps: i32,
  /// How far either side of the starting position the cycle went.
  pub amplitude_steps: i32,
  /// Moves in each half of the cycle.
  pub moves_per_half: u8,
  /// The noise and the drift at the starting position.
  pub repeatability: Repeatability,
  /// How far the beam moved along the axis's direction over the plus half of the cycle.
  pub span_norm: f32,
  /// The loop's width at its middle level, in steps: how much further the plus half had to go than the minus half
  /// to reach the same reading. `None` when a half never crossed that level.
  pub width_steps: Option<f32>
}

impl fmt::Display for HysteresisReport {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "{LOG_TAG},hysteresis,{},start_steps={},amplitude_steps={},moves_per_half={},sigma_norm={:.5},\
       drift_norm={:+.5},span_norm={:.5},",
      self.axis.name(),
      self.start_steps,
      self.amplitude_steps,
      self.moves_per_half,
      self.repeatability.sigma_norm,
      self.repeatability.drift_norm,
      self.span_norm
    )?;
    match self.width_steps {
      Some(width) => write!(f, "width_steps={width:.1}"),
      None => write!(f, "width_steps=none")
    }
  }
}

/// The six coordinates of a measurement, in the order of the log.
pub fn coordinates(shots: &Shots) -> Coordinates {
  [
    shots.a.h_norm,
    shots.a.v_norm,
    shots.b.h_norm,
    shots.b.v_norm,
    shots.c.h_norm,
    shots.c.v_norm
  ]
}

/// The mean of measurements, coordinate by coordinate. `None` for none.
pub(crate) fn mean(of: &[Coordinates]) -> Option<Coordinates> {
  if of.is_empty() {
    return None;
  }
  #[expect(clippy::cast_precision_loss, reason = "a handful of repeats")]
  let count = of.len() as f32;
  let mut sum = [0.0_f32; 6];
  for reading in of {
    for (total, value) in sum.iter_mut().zip(reading) {
      *total += value;
    }
  }
  Some(sum.map(|total| total / count))
}

/// One sigma of a coordinate, from measurements of one position: each coordinate's deviations from its own mean,
/// pooled over the six (`n - 1` degrees of freedom each). `None` for fewer than two measurements.
pub(crate) fn pooled_sigma(repeats: &[Coordinates]) -> Option<f32> {
  let center = mean(repeats)?;
  let freedom = repeats.len().checked_sub(1).filter(|freedom| *freedom > 0)?;
  let squares: f32 = repeats
    .iter()
    .flat_map(|reading| {
      reading.iter().zip(&center).map(|(value, mean)| {
        let deviation = value - mean;
        deviation * deviation
      })
    })
    .sum();
  #[expect(clippy::cast_precision_loss, reason = "a handful of repeats")]
  let degrees = (6 * freedom) as f32;
  Some(libm::sqrtf(squares / degrees))
}

/// The distance between two measurements over the six coordinates.
pub(crate) fn distance(from: &Coordinates, to: &Coordinates) -> f32 {
  libm::sqrtf(from.iter().zip(to).map(|(a, b)| (b - a) * (b - a)).sum())
}

/// The least shift along an axis's direction that counts as the beam moving: `detect_sigmas` sigmas of the difference
/// of two measurements, each coordinate's sigma being `sigma_norm`.
pub(crate) fn shift_threshold_norm(detect_sigmas: f32, sigma_norm: f32) -> f32 {
  detect_sigmas * SQRT_2 * sigma_norm
}

/// The way the beam moved from `from` to `to`, if it moved further than noise over all six coordinates:
/// [`shift_threshold_norm`] times `sqrt(6)`, the distance two measurements of one position stand apart. `None` for a
/// move no larger than that.
pub(crate) fn moved_direction(
  from: &Coordinates,
  to: &Coordinates,
  detect_sigmas: f32,
  sigma_norm: f32
) -> Option<Direction> {
  let noise = shift_threshold_norm(detect_sigmas, sigma_norm) * libm::sqrtf(6.0);
  if distance(from, to) > noise {
    Direction::between(from, to)
  } else {
    None
  }
}

/// A unit vector among the six coordinates: the way an axis moves the beam.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Direction(Coordinates);

impl Direction {
  /// The direction from `from` to `to`. `None` when they are the same, or not finite.
  pub(crate) fn between(from: &Coordinates, to: &Coordinates) -> Option<Self> {
    let length = distance(from, to);
    if !(length.is_finite() && length > 0.0) {
      return None;
    }
    let mut unit = [0.0_f32; 6];
    for ((component, a), b) in unit.iter_mut().zip(from).zip(to) {
      *component = (b - a) / length;
    }
    Some(Self(unit))
  }

  /// How far the beam moved from `from` to `to` along this direction; negative for motion the other way.
  pub(crate) fn along(&self, from: &Coordinates, to: &Coordinates) -> f32 {
    self
      .0
      .iter()
      .zip(from)
      .zip(to)
      .map(|((unit, a), b)| unit * (b - a))
      .sum()
  }
}

/// The width of a hysteresis loop at its middle level, in steps: where the plus half crosses that level less where
/// the minus half does. Each half is `(position_steps, projected_norm)` in the order it was travelled, the plus half
/// from its lowest position, the minus half from its highest. `None` when a half never crosses the level, or the
/// loop has no height.
pub(crate) fn loop_width(up: &[(f32, f32)], down: &[(f32, f32)]) -> Option<f32> {
  let (low, high) = up
    .iter()
    .chain(down)
    .fold((f32::INFINITY, f32::NEG_INFINITY), |(low, high), &(_, reading)| {
      (low.min(reading), high.max(reading))
    });
  if !(low.is_finite() && high.is_finite() && high > low) {
    return None;
  }
  let level = low + (high - low) / 2.0;
  Some(crossing(up, level)? - crossing(down, level)?)
}

/// Where a half of the loop first crosses `level`, interpolating between the measurements on either side.
fn crossing(half: &[(f32, f32)], level: f32) -> Option<f32> {
  half.windows(2).find_map(|pair| {
    let &[(x0, s0), (x1, s1)] = pair else {
      return None;
    };
    let crosses = (s0 <= level && level <= s1) || (s1 <= level && level <= s0);
    if !crosses {
      return None;
    }
    // A flat pair at the level itself gives no fraction: its first position is where the level is reached.
    let fraction = (level - s0) / (s1 - s0);
    Some(if fraction.is_finite() {
      x0 + fraction * (x1 - x0)
    } else {
      x0
    })
  })
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::string::ToString;
  use std::vec::Vec;

  use super::*;
  use crate::quadrant::Reading;
  use crate::test_support::assert_close;

  fn reading(h_norm: f32, v_norm: f32) -> Reading {
    Reading {
      h_norm,
      v_norm,
      sum: 1.0
    }
  }

  #[test]
  #[expect(clippy::float_cmp, reason = "the coordinates are copied, not computed")]
  fn the_six_coordinates_are_h_and_v_at_a_then_b_then_c() {
    let shots = Shots {
      a: reading(0.1, 0.2),
      b: reading(0.3, 0.4),
      c: reading(0.5, 0.6)
    };
    assert_eq!(coordinates(&shots), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
  }

  #[test]
  fn readings_that_never_change_have_no_noise() {
    let same = [0.1, -0.2, 0.3, 0.0, 0.5, 0.25];
    assert_eq!(pooled_sigma(&[same, same, same]), Some(0.0));
  }

  #[test]
  fn the_noise_is_pooled_over_the_six_coordinates() {
    // One coordinate reads 0 then 0.06: its mean is 0.03, its squares 2 * 0.03^2, over six degrees of freedom.
    let mut second = [0.0; 6];
    second[3] = 0.06;
    let sigma = pooled_sigma(&[[0.0; 6], second]).unwrap();
    assert_close(sigma, libm::sqrtf(2.0 * 0.03 * 0.03 / 6.0), 1e-7);
  }

  #[test]
  fn a_shift_is_judged_at_three_sigmas_of_the_difference_of_two_measurements() {
    // 3 * sqrt(2) * 0.002.
    assert_close(shift_threshold_norm(3.0, 0.002), 0.008_485_281, 1e-8);
  }

  #[test]
  fn a_move_has_a_direction_only_beyond_the_noise_of_a_distance_over_the_six_coordinates() {
    // The noise of a distance: 3 * sqrt(2) * 0.002 * sqrt(6) = 0.020_784_61.
    let still = [0.0; 6];
    let along_h = |moved_norm: f32| [moved_norm, 0.0, 0.0, 0.0, 0.0, 0.0];
    assert_eq!(moved_direction(&still, &along_h(0.0207), 3.0, 0.002), None);
    assert_eq!(
      moved_direction(&still, &along_h(0.0209), 3.0, 0.002),
      Direction::between(&still, &along_h(1.0))
    );
    let noise = shift_threshold_norm(3.0, 0.002) * libm::sqrtf(6.0);
    assert_eq!(
      moved_direction(&still, &along_h(noise), 3.0, 0.002),
      None,
      "a move of exactly the noise is not motion"
    );
    assert!(moved_direction(&still, &along_h(noise.next_up()), 3.0, 0.002).is_some());
    assert_eq!(
      moved_direction(&still, &still, 3.0, 0.0),
      None,
      "no move, even without noise"
    );
  }

  #[test]
  fn one_measurement_has_no_noise_to_pool() {
    assert_eq!(pooled_sigma(&[[0.0; 6]]), None);
    assert_eq!(pooled_sigma(&[]), None);
  }

  #[test]
  fn motion_along_the_direction_counts_in_full_and_across_it_not_at_all() {
    let from = [0.0; 6];
    let to = [0.3, 0.0, 0.4, 0.0, 0.0, 0.0];
    let direction = Direction::between(&from, &to).unwrap();
    assert_close(direction.along(&from, &to), 0.5, 1e-6);
    assert_close(direction.along(&to, &from), -0.5, 1e-6);
    let across = [0.0, 0.7, 0.0, -0.2, 0.0, 0.0];
    assert_close(direction.along(&from, &across), 0.0, 1e-6);
  }

  #[test]
  fn no_direction_comes_from_a_beam_that_did_not_move_or_a_reading_that_is_not_a_number() {
    let here = [0.1; 6];
    assert_eq!(Direction::between(&here, &here), None);
    let mut broken = here;
    broken[2] = f32::NAN;
    assert_eq!(Direction::between(&here, &broken), None);
  }

  /// A half of a loop: `(position_steps, projected_norm)` in the order travelled.
  type Half = Vec<(f32, f32)>;

  /// A loop of pure backlash `play`, measured after each of `moves` moves from `-amplitude` to `amplitude` and back:
  /// going plus the screw stays put until the play is taken up, then follows; going minus it lags by the play.
  fn backlash_loop(play: f32, amplitude: f32, moves: u8) -> (Half, Half) {
    let slope = 0.01;
    let positions: Vec<f32> = (0..=moves)
      .map(|i| -amplitude + 2.0 * amplitude * f32::from(i) / f32::from(moves))
      .collect();
    let up = positions
      .iter()
      .map(|&x| (x, slope * x.max(-amplitude + play)))
      .collect();
    let down = positions
      .iter()
      .rev()
      .map(|&x| (x, slope * (x + play).min(amplitude)))
      .collect();
    (up, down)
  }

  #[test]
  fn the_width_of_a_backlash_loop_is_its_play() {
    for play in [0.0, 5.0, 20.0, 60.0] {
      let (up, down) = backlash_loop(play, 90.0, 8);
      assert_close(loop_width(&up, &down).unwrap(), play, 1e-3);
    }
  }

  #[test]
  fn a_loop_whose_halves_never_cross_its_middle_has_no_width() {
    let up = [(0.0, 0.0), (10.0, 0.0)];
    let down = [(10.0, 1.0), (0.0, 1.0)];
    assert_eq!(loop_width(&up, &down), None, "each half is flat");
    // Both halves lie on the level of a loop with no height, and would cross it at their first positions.
    assert_eq!(
      loop_width(&[(0.0, 0.0), (10.0, 0.0)], &[(10.0, 0.0), (0.0, 0.0)]),
      None,
      "no height"
    );
  }

  #[test]
  fn a_loop_with_an_infinite_reading_has_no_width() {
    for bad in [f32::INFINITY, f32::NEG_INFINITY] {
      let up = [(-1.0, 0.0), (1.0, bad)];
      let down = [(1.0, bad), (-1.0, 0.0)];
      assert_eq!(loop_width(&up, &down), None, "{bad}");
    }
    assert_eq!(loop_width(&[], &[]), None, "no readings");
  }

  #[test]
  fn the_overshoot_is_twice_the_play_and_never_below_the_floor() {
    let bracket = |above_steps, up_to_steps| Play::Bracket {
      above_steps,
      up_to_steps
    };
    assert_eq!(bracket(16, 24).overshoot_steps(60), Some(60));
    assert_eq!(bracket(32, 48).overshoot_steps(60), Some(96));
    assert_eq!(Play::Beyond { steps: 128 }.overshoot_steps(60), None);
  }

  #[test]
  fn a_sample_is_one_line_of_the_columns_named_first() {
    let sample = Sample {
      axis: Axis::M2V,
      phase: Phase::Out,
      index: 16,
      offset_steps: -16,
      pos_steps: 104,
      shots: Shots {
        a: reading(0.012_34, -0.5),
        b: reading(0.0, 0.25),
        c: reading(-0.125, 0.0)
      }
    };
    let line = Entry::Sample(sample).to_string();
    assert_eq!(
      line,
      "BU1,sample,m2v,out,16,-16,104,0.01234,-0.50000,1.0000,0.00000,0.25000,1.0000,-0.12500,0.00000,1.0000"
    );
    assert_eq!(
      line.split(',').count(),
      SAMPLE_COLUMNS.split(',').count(),
      "every column filled"
    );
    assert!(SAMPLE_COLUMNS.starts_with(LOG_TAG));
  }

  #[test]
  fn the_reports_name_every_field() {
    let repeatability = Repeatability {
      sigma_norm: 0.002,
      drift_norm: -0.0001
    };
    let trial = Trial {
      axis: Axis::M1H,
      steps: 32,
      shift_norm: 0.05,
      return_norm: -0.001,
      play_steps: 26.04,
      moved: true
    };
    let report = BacklashReport {
      axis: Axis::M1H,
      start_steps: 120,
      repeatability,
      slope_norm_per_step: 0.008,
      threshold_norm: 0.008,
      play: Play::Bracket {
        above_steps: 16,
        up_to_steps: 24
      },
      overshoot_steps: Some(60)
    };
    assert_eq!(
      Entry::Trial(trial).to_string(),
      "BU1,trial,m1h,steps=32,shift_norm=+0.05000,return_norm=-0.00100,play_steps=26.0,moved=yes"
    );
    assert_eq!(
      report.to_string(),
      "BU1,backlash,m1h,start_steps=120,sigma_norm=0.00200,drift_norm=-0.00010,slope_norm_per_step=0.008000,\
       threshold_norm=0.00800,resolution_steps=1.0,play_above_steps=16,play_up_to_steps=24,overshoot_steps=60"
    );
    let beyond = BacklashReport {
      play: Play::Beyond { steps: 128 },
      overshoot_steps: None,
      ..report
    };
    assert!(
      beyond
        .to_string()
        .ends_with("play_beyond_steps=128,overshoot_steps=none")
    );
    let hysteresis = HysteresisReport {
      axis: Axis::M2H,
      start_steps: 0,
      amplitude_steps: 90,
      moves_per_half: 8,
      repeatability,
      span_norm: 0.6,
      width_steps: Some(20.04)
    };
    assert_eq!(
      hysteresis.to_string(),
      "BU1,hysteresis,m2h,start_steps=0,amplitude_steps=90,moves_per_half=8,sigma_norm=0.00200,drift_norm=-0.00010,\
       span_norm=0.60000,width_steps=20.0"
    );
    let closed = HysteresisReport {
      width_steps: None,
      ..hysteresis
    };
    assert!(closed.to_string().ends_with("width_steps=none"));
  }
}
