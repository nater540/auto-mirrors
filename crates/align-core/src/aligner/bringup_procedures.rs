//! The bring-up measurements of [`crate::bringup`] on the aligner's own moves and measurements, so that they keep its
//! account of each axis and its checks: every move of a procedure is checked against the soft limits and the
//! per-move cap before the first one runs, and every measurement passes the validity checks.

use crate::approach::{self, LastDir};
use crate::axis::Axis;
use crate::bringup::{
  self, BacklashReport, Coordinates, Direction, Entry, HysteresisReport, MAX_MOVES_PER_HALF, MAX_REPEAT_SHOTS, Phase,
  Play, Repeatability, Sample, Trial
};
use crate::config::BringUp;
use crate::rig::{Actuator, BeamSensor};

use super::{AlignFault, Aligner, Planned};

/// Positions in the hysteresis cycle's plus half, the lowest included.
const CYCLE_POINTS: usize = MAX_MOVES_PER_HALF as usize + 1;

/// Where a procedure measures from: the axis, and the node's count at the starting position.
#[derive(Clone, Copy, Debug)]
struct Origin {
  axis: Axis,
  start_steps: i32
}

/// What a backlash trial is judged by.
#[derive(Clone, Copy, Debug)]
struct Gauge {
  direction: Direction,
  slope_norm_per_step: f32,
  threshold_norm: f32
}

/// The measurements at the starting position.
struct Repeats {
  readings: [Coordinates; MAX_REPEAT_SHOTS as usize],
  count: usize
}

impl Repeats {
  fn taken(&self) -> &[Coordinates] {
    self.readings.get(..self.count).unwrap_or(&[])
  }

  /// One sigma of a coordinate, never below the configured floor, and the mean of the repeats.
  fn noise(&self, bring_up: &BringUp) -> Result<(f32, Coordinates), AlignFault> {
    bringup::pooled_sigma(self.taken(), bring_up.read_at)
      .map(|sigma_norm| sigma_norm.max(bring_up.noise_floor_norm))
      .zip(bringup::mean(self.taken()))
      .ok_or(AlignFault::Config("repeat_shots must be at least 2"))
  }

  /// The last repeat less the first, along `direction`.
  fn drift(&self, direction: &Direction) -> f32 {
    match (self.taken().first(), self.taken().last()) {
      (Some(first), Some(last)) => direction.along(first, last),
      _ => 0.0
    }
  }
}

impl<A: Actuator, S: BeamSensor> Aligner<A, S> {
  /// Measures the play of one axis, backlash and seat stick-slip together, through the beam (reference 3.4 with the
  /// search of addendum A.5; [`crate::bringup`] has the procedure and its choices).
  ///
  /// `observe` is given every measurement and every trial as it happens, so that a run cut short still leaves its
  /// data in the log. The axis ends where it started, approached plus; no other axis moves.
  ///
  /// # Errors
  ///
  /// [`AlignFault::PositionUnknown`], [`AlignFault::SoftLimit`], or [`AlignFault::MoveTooLong`] before anything moved:
  /// every move is checked first. [`AlignFault::WeakColumn`] when the poke did not move the beam, and any fault of a
  /// move or a measurement.
  pub async fn measure_backlash(
    &mut self,
    axis: Axis,
    observe: &mut impl FnMut(&Entry)
  ) -> Result<BacklashReport, AlignFault> {
    let bring_up = self.config.bring_up;
    let poke_steps = self.config.motion.poke_steps;
    let largest = bring_up.backlash_trials_steps.iter().copied().fold(0, i32::max);
    let start = self.check_start(axis)?;
    let poked = self.check_approach(axis, start, poke_steps)?;
    self.check_approach(axis, poked, poke_steps.saturating_neg())?;
    self.check_legs(axis, start, [largest.saturating_neg(), largest])?;

    let origin = self.approach_afresh(axis).await?;
    let repeats = self.repeats(origin, observe).await?;
    // The poke finds which way the axis moves the beam, and how far a step does; it is undone at once.
    self.leg(axis, poke_steps).await?;
    let poke = self.sample(origin, Phase::Poke, 0, observe).await?;
    self.approach(axis, poke_steps.saturating_neg()).await?;
    let mut before = self.sample(origin, Phase::Reference, 0, observe).await?;

    let (sigma_norm, center) = repeats.noise(&self.config.bring_up)?;
    let direction = self.direction_of(axis, &center, &poke, sigma_norm)?;
    let gauge = Gauge {
      direction,
      slope_norm_per_step: direction.along(&center, &poke) / steps_f32(poke_steps),
      threshold_norm: bringup::shift_threshold_norm(bring_up.detect_sigmas, sigma_norm)
    };
    let mut above_steps = 0;
    let mut up_to = None;
    for steps in bring_up.backlash_trials_steps {
      if self
        .confirmed_trial(origin, steps, &mut before, &gauge, observe)
        .await?
      {
        up_to = Some(steps);
        break;
      }
      above_steps = steps;
    }
    // Reference 3.4: bisect once, unless the bracket is a step wide. The middle is always below the high end.
    if let Some(high) = up_to {
      let middle = above_steps + (high - above_steps) / 2;
      if above_steps < middle {
        if self
          .confirmed_trial(origin, middle, &mut before, &gauge, observe)
          .await?
        {
          up_to = Some(middle);
        } else {
          above_steps = middle;
        }
      }
    }
    let play = match up_to {
      Some(up_to_steps) => Play::Bracket {
        above_steps,
        up_to_steps
      },
      None => Play::Beyond { steps: largest }
    };
    Ok(BacklashReport {
      axis,
      start_steps: origin.start_steps,
      repeatability: Repeatability {
        sigma_norm,
        drift_norm: repeats.drift(&direction)
      },
      slope_norm_per_step: gauge.slope_norm_per_step,
      threshold_norm: gauge.threshold_norm,
      play,
      overshoot_steps: play.overshoot_steps(bring_up.overshoot_floor_steps)
    })
  }

  /// Cycles one axis through its hysteresis loop, measuring after every move, and reads the loop's width (reference
  /// 3.4; [`crate::bringup`] has the procedure and its choices).
  ///
  /// `observe` is given every measurement as it is taken, so that a run cut short still leaves its data in the log.
  /// The axis ends where it started, approached plus; no other axis moves.
  ///
  /// # Errors
  ///
  /// [`AlignFault::PositionUnknown`], [`AlignFault::SoftLimit`], or [`AlignFault::MoveTooLong`] before anything moved:
  /// every move is checked first. [`AlignFault::WeakColumn`] when the cycle did not move the beam, and any fault of
  /// a move or a measurement.
  pub async fn measure_hysteresis(
    &mut self,
    axis: Axis,
    observe: &mut impl FnMut(&Entry)
  ) -> Result<HysteresisReport, AlignFault> {
    let bring_up = self.config.bring_up;
    let moves = bring_up.hysteresis_moves_per_half.min(MAX_MOVES_PER_HALF);
    let amplitude = bring_up
      .hysteresis_amplitude_pokes
      .saturating_mul(self.config.motion.poke_steps);
    let positions = cycle_positions(amplitude, moves);
    let path = positions.get(..=usize::from(moves)).unwrap_or(&[]);
    let start = self.check_start(axis)?;
    let bottom = self.check_legs(axis, start, [amplitude.saturating_neg()])?;
    let top = self.check_legs(axis, bottom, moves_along(path))?;
    let end = self.check_legs(axis, top, moves_along(path).rev().map(i32::saturating_neg))?;
    self.check_approach(axis, end, amplitude)?;

    let origin = self.approach_afresh(axis).await?;
    let repeats = self.repeats(origin, observe).await?;
    // The plus half from its lowest point, then the minus half from its highest, which the halves share: one reading
    // per position, each half `(position, reading)` in the order travelled.
    let half = usize::from(moves);
    let mut up = [(0_i32, [0.0_f32; 6]); CYCLE_POINTS];
    let mut down = up;
    self.leg(axis, amplitude.saturating_neg()).await?;
    let reading = self.sample(origin, Phase::Up, 0, observe).await?;
    if let (Some(slot), Some(&position)) = (up.first_mut(), path.first()) {
      *slot = (position, reading);
    }
    for (((index, step), &position), slot) in (1..)
      .zip(moves_along(path))
      .zip(path.iter().skip(1))
      .zip(up.iter_mut().skip(1))
    {
      self.leg(axis, step).await?;
      *slot = (position, self.sample(origin, Phase::Up, index, observe).await?);
    }
    if let (Some(slot), Some(&highest)) = (down.first_mut(), up.get(half)) {
      *slot = highest;
    }
    for (((index, step), &position), slot) in (1..)
      .zip(moves_along(path).rev().map(i32::saturating_neg))
      .zip(path.iter().rev().skip(1))
      .zip(down.iter_mut().skip(1))
    {
      self.leg(axis, step).await?;
      *slot = (position, self.sample(origin, Phase::Down, index, observe).await?);
    }
    self.approach(axis, amplitude).await?;

    let (Some(&(_, lowest)), Some(&(_, highest))) = (up.first(), up.get(half)) else {
      return Err(AlignFault::WeakColumn { axis });
    };
    let (sigma_norm, _) = repeats.noise(&self.config.bring_up)?;
    let direction = self.direction_of(axis, &lowest, &highest, sigma_norm)?;
    let project = |travelled: &[(i32, Coordinates); CYCLE_POINTS]| {
      let mut projected = [(0.0_f32, 0.0_f32); CYCLE_POINTS];
      for (slot, (position, reading)) in projected.iter_mut().zip(travelled) {
        *slot = (steps_f32(*position), direction.along(&lowest, reading));
      }
      projected
    };
    let (up, down) = (project(&up), project(&down));
    let (up, down) = (up.get(..=half).unwrap_or(&[]), down.get(..=half).unwrap_or(&[]));
    Ok(HysteresisReport {
      axis,
      start_steps: origin.start_steps,
      amplitude_steps: amplitude,
      moves_per_half: moves,
      repeatability: Repeatability {
        sigma_norm,
        drift_norm: repeats.drift(&direction)
      },
      span_norm: direction.along(&lowest, &highest),
      width_steps: bringup::loop_width(up, down)
    })
  }

  /// Checks the standard approach every procedure starts with, run whatever the axis's last direction, and returns
  /// where it leaves the axis.
  pub(super) fn check_start(&self, axis: Axis) -> Result<Planned, AlignFault> {
    let from = Planned {
      last_dir: LastDir::Unknown,
      ..Planned::from(self.known(axis)?)
    };
    self.check_approach(axis, from, 0)
  }

  /// The standard approach to where the axis is, run even when it last moved plus, so that the repeats that follow
  /// come right after a move as every later measurement does.
  async fn approach_afresh(&mut self, axis: Axis) -> Result<Origin, AlignFault> {
    let plan = approach::plan(0, LastDir::Unknown, self.config.motion.overshoot_steps);
    for leg in plan.legs() {
      self.leg(axis, leg).await?;
    }
    Ok(Origin {
      axis,
      start_steps: self.axes.get(axis).pos_steps
    })
  }

  /// The repeats at the starting position.
  async fn repeats(&mut self, origin: Origin, observe: &mut impl FnMut(&Entry)) -> Result<Repeats, AlignFault> {
    let mut repeats = Repeats {
      readings: [[0.0; 6]; MAX_REPEAT_SHOTS as usize],
      count: 0
    };
    for index in 0..self.config.bring_up.repeat_shots.min(MAX_REPEAT_SHOTS) {
      let reading = self.sample(origin, Phase::Repeat, i32::from(index), observe).await?;
      if let Some(slot) = repeats.readings.get_mut(usize::from(index)) {
        *slot = reading;
        repeats.count += 1;
      }
    }
    Ok(repeats)
  }

  /// Whether a backlash trial of `steps` moves the beam: run once, and once more if it did, the beam having to move
  /// both times. `before` is the reading the next trial is judged against, the last return's.
  async fn confirmed_trial(
    &mut self,
    origin: Origin,
    steps: i32,
    before: &mut Coordinates,
    gauge: &Gauge,
    observe: &mut impl FnMut(&Entry)
  ) -> Result<bool, AlignFault> {
    if !self.trial(origin, steps, before, gauge, observe).await? {
      return Ok(false);
    }
    self.trial(origin, steps, before, gauge, observe).await
  }

  /// One backlash trial: back by `steps`, measured, plus by `steps` without overshoot, measured. Logs the trial,
  /// leaves the return's reading in `before`, and returns whether the beam moved.
  async fn trial(
    &mut self,
    origin: Origin,
    steps: i32,
    before: &mut Coordinates,
    gauge: &Gauge,
    observe: &mut impl FnMut(&Entry)
  ) -> Result<bool, AlignFault> {
    self.leg(origin.axis, steps.saturating_neg()).await?;
    let out = self.sample(origin, Phase::Out, steps, observe).await?;
    self.leg(origin.axis, steps).await?;
    let back = self.sample(origin, Phase::Back, steps, observe).await?;
    let shift_norm = gauge.direction.along(&out, before);
    let trial = Trial {
      axis: origin.axis,
      steps,
      shift_norm,
      return_norm: gauge.direction.along(before, &back),
      play_steps: steps_f32(steps) - shift_norm / gauge.slope_norm_per_step,
      moved: shift_norm > gauge.threshold_norm
    };
    observe(&Entry::Trial(trial));
    *before = back;
    Ok(trial.moved)
  }

  /// The direction the axis moved the beam from `from` to `to`, if it moved it further than noise
  /// ([`bringup::moved_direction`]); otherwise the axis is suspect, as a dead axis is when calibrating.
  fn direction_of(
    &mut self,
    axis: Axis,
    from: &Coordinates,
    to: &Coordinates,
    sigma_norm: f32
  ) -> Result<Direction, AlignFault> {
    let bring_up = &self.config.bring_up;
    bringup::moved_direction(from, to, bring_up.detect_sigmas, sigma_norm, bring_up.read_at).ok_or_else(|| {
      self.axes.get_mut(axis).suspect = true;
      AlignFault::WeakColumn { axis }
    })
  }

  /// One measurement of a procedure, given to `observe` and reduced to its coordinates.
  async fn sample(
    &mut self,
    origin: Origin,
    phase: Phase,
    index: i32,
    observe: &mut impl FnMut(&Entry)
  ) -> Result<Coordinates, AlignFault> {
    let shots = self.measure().await?.shots;
    let pos_steps = self.axes.get(origin.axis).pos_steps;
    observe(&Entry::Sample(Sample {
      axis: origin.axis,
      phase,
      index,
      offset_steps: pos_steps.saturating_sub(origin.start_steps),
      pos_steps,
      shots
    }));
    Ok(bringup::coordinates(&shots, self.config.bring_up.read_at))
  }
}

/// The positions of the hysteresis cycle's plus half, relative to the start: from `-amplitude` to `amplitude` in
/// `moves` moves as even as whole steps allow. The first `moves + 1` entries are used.
fn cycle_positions(amplitude: i32, moves: u8) -> [i32; CYCLE_POINTS] {
  let mut positions = [0; CYCLE_POINTS];
  let lowest = i64::from(amplitude).saturating_neg();
  let span = i64::from(amplitude).saturating_mul(2);
  let moves = i64::from(moves.max(1));
  for (index, slot) in (0_i64..)
    .zip(positions.iter_mut())
    .take_while(|(index, _)| *index <= moves)
  {
    *slot = i32::try_from(lowest + span * index / moves).unwrap_or(0);
  }
  positions
}

/// The moves between consecutive positions.
fn moves_along(path: &[i32]) -> impl DoubleEndedIterator<Item = i32> + '_ {
  path.windows(2).map(|pair| match pair {
    [from, to] => to.saturating_sub(*from),
    _ => 0
  })
}

/// A step count as a float, for the projections.
fn steps_f32(steps: i32) -> f32 {
  #[expect(
    clippy::cast_precision_loss,
    reason = "step counts of a few thousand are exact in f32"
  )]
  let steps = steps as f32;
  steps
}

#[cfg(test)]
mod tests {
  extern crate std;

  use super::*;

  #[test]
  fn the_cycle_goes_from_minus_to_plus_its_amplitude_in_moves_as_even_as_whole_steps_allow() {
    let positions = cycle_positions(90, 8);
    assert_eq!(positions.get(..9).unwrap(), [-90, -68, -45, -23, 0, 22, 45, 67, 90]);
    assert_eq!(
      moves_along(positions.get(..9).unwrap()).collect::<std::vec::Vec<_>>(),
      [22, 23, 22, 23, 22, 23, 22, 23]
    );
    assert_eq!(cycle_positions(5, 1).get(..2).unwrap(), [-5, 5]);
    let most = cycle_positions(90, MAX_MOVES_PER_HALF);
    assert_eq!(
      (most.first(), most.get(usize::from(MAX_MOVES_PER_HALF))),
      (Some(&-90), Some(&90)),
      "the most moves the configuration allows still reach the top"
    );
  }

  #[test]
  fn the_drift_is_the_last_repeat_less_the_first_along_the_axis() {
    let along_h = Direction::between(&[0.0; 6], &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]).unwrap();
    let mut readings = [[0.0; 6]; MAX_REPEAT_SHOTS as usize];
    readings[0][0] = 0.1;
    readings[1][0] = 0.9;
    readings[2] = [0.4, 5.0, 0.0, 0.0, 0.0, 0.0];
    let drift = Repeats { readings, count: 3 }.drift(&along_h);
    assert!((drift - 0.3).abs() < 1e-6, "{drift}");
    let none = Repeats { readings, count: 0 }.drift(&along_h);
    assert!(none.abs() < f32::EPSILON, "no repeats, no drift: {none}");
  }
}
