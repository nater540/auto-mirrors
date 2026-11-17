//! The alignment procedure: establish the standard direction, calibrate, then iterate to zero (hand-off 3.2,
//! reference 3.1 and 3.2, addendum A.5).
//!
//! [`Aligner`] owns an [`Actuator`], a [`BeamSensor`], the configuration, and what the coordinator knows about
//! each axis. It strictly alternates moving and measuring. Every outcome that is not success is an
//! [`AlignFault`] value. The bring-up measurements of [`crate::bringup`] are methods of it too, so that they keep
//! its account of each axis and its checks.
//!
//! What the documents leave open, and what is done here:
//!
//! - **A reading outside the solver's linear range** (reference 2.2: "use sign and coarse steps only"). The step
//!   is still taken, because the sensor response is monotonic, but nothing is learned from it: no Broyden update,
//!   no suspect judgment. It points the right way only if the stored Jacobian has the right signs, so a step
//!   after which the residual grew by more than noise counts as a poor step, coarse, small, or clamped (review of
//!   2026-11-15: with one column's sign wrong, 49 runs in 50 walked the beam off the sensor).
//! - **A step too small to see.** Addendum A.5 compares the measured change with the prediction after every
//!   step. For a step of a few counts the prediction is inside sensor noise and seat stick-slip, the comparison
//!   is meaningless, and applying it reports a healthy axis as not responding (the simulator shows this). A step
//!   whose predicted change is below [`crate::calibration::judging_floor`] is therefore taken but not judged: no
//!   suspect, no Broyden update, no damping change. The reference already skips the Broyden update for small
//!   steps (3.2); this extends the same guard. The floor is the noise of the whole change `dm`, four readings to
//!   a component, not of one measurement component: the review of 2026-11-15 showed that the smaller floor named
//!   a healthy axis in one run of eight at the configured noise.
//! - **A success is confirmed** by a second measurement, with no move between: the mean of the two, which has
//!   `1 / sqrt(2)` of the noise, must be inside the tolerance as well.
//!   The repeatability check of reference 2.3 (`Reading::check_repeat`) is not applied to the pair: it allows
//!   `repeat_sigma` (3) sigmas of one coordinate for a difference of two readings, whose sigma is `sqrt(2)` times
//!   larger, so noise alone would fail about one confirmation in five over the six coordinates.
//! - **The Broyden updates are not stored.** They correct the Jacobian within one alignment, where they help; kept
//!   across sessions, the noise of each update accumulates (the same review).
//! - **A step limited by the clamp** cannot reduce the residual by the required fraction however well it went, so
//!   it is a poor step only if the residual grew.
//! - **Which axis is suspect** when four moved together, and telling a dead axis from a wrong Jacobian:
//!   [`crate::solver::unresponsive_axis`]. A mismatch in which no axis fell short is a Jacobian error, and the
//!   Broyden update corrects it.
//! - **A reading that leaves the linear range during a calibration poke** is accepted as long as it passes the
//!   validity checks. The reference rejects a calibration only on conditioning and weak columns.

use nalgebra::Vector4;

use crate::approach::{self, LastDir, SoftLimits};
use crate::axis::{Axis, PerAxis};
use crate::bringup;
use crate::calibration::{self, Calibration};
use crate::config::{BringUp, Config, Solver};
use crate::measurement::{HeadOffset, Shots};
use crate::quadrant::ReadingFault;
use crate::rig::{Actuator, BeamSensor, MoveFault, SensorFault};
use crate::solver::{self, Conditioning, Jacobian, SolveFault};

/// What the coordinator knows about one axis. This is the record that is persisted after every move.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AxisState {
  /// Commanded position, in steps from the commissioning position. Not a measured position.
  pub pos_steps: i32,
  /// Direction of the last move.
  pub last_dir: LastDir,
  /// Set when a move failed or the beam did not respond as predicted; cleared by the next convergence.
  pub suspect: bool,
  /// The position is unknown: a move was sent and never answered, so the node may have run all of it, part, or none.
  /// The axis does not move again until its position is read from the node and given back with
  /// [`Aligner::restore_position`], so that the soft limits are checked against a count that is true.
  pub lost: bool
}

/// The result of a successful alignment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AlignReport {
  /// Solver steps taken, over all attempts.
  pub iterations: u8,
  /// Recalibrations that were needed.
  pub recalibrations: u8,
  /// Infinity norm of the final measurement vector.
  pub residual_norm: f32,
  /// The reading at point A: the constant offset that tilt cannot remove.
  pub head_offset: HeadOffset,
  /// Whether that offset is large enough to ask for mechanical centering.
  pub needs_mechanical_centering: bool,
  /// Condition number of the Jacobian in use at the end.
  pub cond: f32
}

/// Why an alignment or a calibration stopped without success.
#[derive(Clone, Copy, Debug, PartialEq, thiserror::Error)]
pub enum AlignFault {
  /// A configuration value makes the procedure impossible.
  #[error("configuration: {0}")]
  Config(&'static str),
  /// A move failed.
  #[error("axis {axis:?}: {fault}")]
  Move {
    /// The axis that was moving.
    axis: Axis,
    /// What the actuator reported.
    fault: MoveFault
  },
  /// A move would have left the soft limits. Nothing of the step, poke, or approach it belonged to was moved;
  /// manual pre-alignment is required.
  #[error("axis {axis:?}: a move to {target_steps} steps is outside the soft limits")]
  SoftLimit {
    /// The axis.
    axis: Axis,
    /// The commanded position the move would have reached.
    target_steps: i64
  },
  /// A single move would have been longer than the per-move cap.
  /// Nothing of the step, poke, or approach it belonged to was moved.
  #[error("axis {axis:?}: a move of {steps} steps is longer than the per-move cap")]
  MoveTooLong {
    /// The axis.
    axis: Axis,
    /// The length of the refused leg.
    steps: i32
  },
  /// The sensor delivered nothing.
  #[error("sensor: {0}")]
  Sensor(SensorFault),
  /// Readings arrived but failed the validity checks on every allowed shot. Coarse acquisition is required.
  #[error("reading rejected: {0}")]
  Reading(ReadingFault),
  /// A poke barely moved the beam: the axis is dead or disconnected. A calibration reports it, and so do the
  /// bring-up measurements, for the poke or the cycle that finds an axis's direction.
  #[error("axis {axis:?}: a poke did not move the beam")]
  WeakColumn {
    /// The axis.
    axis: Axis
  },
  /// The calibrated Jacobian was rejected.
  #[error("calibration rejected: {0}")]
  Calibration(SolveFault),
  /// The solver could not produce a step.
  #[error("solver: {0}")]
  Solve(SolveFault),
  /// An axis was asked to move while its position is unknown, after a move that went unanswered.
  #[error("axis {axis:?}: the position is unknown since a move went unanswered; read it from the node first")]
  PositionUnknown {
    /// The axis.
    axis: Axis
  },
  /// The beam did not respond to an axis on consecutive steps: skipping, a coupler that slid off,
  /// or a screw at the end of its travel.
  #[error("axis {axis:?} is not responding")]
  AxisNotResponding {
    /// The axis.
    axis: Axis
  },
  /// The residual stayed above the tolerance after every allowed iteration and recalibration.
  #[error("not converged: residual {residual_norm} after the last allowed recalibration")]
  NotConverged {
    /// Infinity norm of the last measurement vector.
    residual_norm: f32
  }
}

/// Where an axis would be after the moves checked so far.
#[derive(Clone, Copy, Debug)]
struct Planned {
  pos_steps: i64,
  last_dir: LastDir
}

impl From<AxisState> for Planned {
  fn from(state: AxisState) -> Self {
    Self {
      pos_steps: i64::from(state.pos_steps),
      last_dir: state.last_dir
    }
  }
}

/// One accepted measurement.
#[derive(Clone, Copy, Debug)]
struct Measured {
  shots: Shots,
  /// The control target built from the shots.
  vector: Vector4<f32>,
  /// Whether every reading is inside the solver's linear range.
  linear: bool
}

/// How one solver attempt ended.
enum Attempt {
  Converged(Measured),
  /// The Jacobian is no longer trusted; `residual_norm` is where the attempt stopped.
  Recalibrate {
    residual_norm: f32
  }
}

/// What is carried from one solver step to the next.
struct Progress {
  jacobian: Jacobian,
  lambda: f32,
  poor_steps: u8,
  suspect_streak: Option<(Axis, u8)>
}

/// What a judged step asks for.
enum Verdict {
  Continue,
  Recalibrate
}

/// Runs calibration and alignment against an actuator and a sensor.
pub struct Aligner<A, S> {
  actuator: A,
  sensor: S,
  config: Config,
  sum_ref: f32,
  axes: PerAxis<AxisState>,
  calibration: Option<Calibration>,
  target: fn(&Shots) -> Vector4<f32>
}

impl<A: Actuator, S: BeamSensor> Aligner<A, S> {
  /// An aligner with no stored calibration, every axis at position zero with an unknown last direction.
  ///
  /// `sum_ref` is the sensor's reference sum for the power in use, from commissioning.
  ///
  /// # Errors
  ///
  /// [`AlignFault::Config`] naming the first value that makes the procedure unsafe or impossible: a NaN, a
  /// reference sum that is not positive, a fraction outside its range. Every comparison with a NaN is false, so
  /// an unchecked NaN threshold silently turns its check off (a NaN `sum_ref` passes a beam that is off the sensor).
  pub fn new(actuator: A, sensor: S, config: Config, sum_ref: f32) -> Result<Self, AlignFault> {
    check_config(&config, sum_ref)?;
    Ok(Self {
      actuator,
      sensor,
      config,
      sum_ref,
      axes: PerAxis::splat(AxisState::default()),
      calibration: None,
      target: Shots::measurement
    })
  }

  /// Restores a stored calibration.
  #[must_use]
  pub fn with_calibration(mut self, calibration: Calibration) -> Self {
    self.calibration = Some(calibration);
    self
  }

  /// Restores the stored axis records.
  #[must_use]
  pub fn with_axes(mut self, axes: PerAxis<AxisState>) -> Self {
    self.axes = axes;
    self
  }

  /// Replaces the control target. The default is [`Shots::measurement`], the three-position difference vector.
  /// This is the seam the hand-off asks for around that decision (section 2, item 1); the simulator uses it to
  /// show that an absolute target fails.
  #[must_use]
  pub fn with_target(mut self, target: fn(&Shots) -> Vector4<f32>) -> Self {
    self.target = target;
    self
  }

  /// What is known about each axis.
  pub fn axes(&self) -> &PerAxis<AxisState> {
    &self.axes
  }

  /// Gives back the position of an axis read from its node (its `STATUS`), after a move that went unanswered.
  /// The axis stays suspect, and its last direction is unknown, until the next convergence.
  pub fn restore_position(&mut self, axis: Axis, pos_steps: i32) {
    let state = self.axes.get_mut(axis);
    state.pos_steps = pos_steps;
    state.lost = false;
    state.suspect = true;
    state.last_dir = LastDir::Unknown;
  }

  /// Where an axis is, if that is known.
  fn known(&self, axis: Axis) -> Result<AxisState, AlignFault> {
    let state = self.axes.get(axis);
    if state.lost {
      return Err(AlignFault::PositionUnknown { axis });
    }
    Ok(state)
  }

  /// The calibration in use, if any: the Jacobian measured by the last poking, never one updated by Broyden.
  pub fn calibration(&self) -> Option<&Calibration> {
    self.calibration.as_ref()
  }

  /// Moves one axis by `delta_steps` with the standard approach, so that it ends in the plus direction.
  ///
  /// Every leg is checked against the soft limits and the per-move cap before the first one runs.
  ///
  /// # Errors
  ///
  /// [`AlignFault::PositionUnknown`], [`AlignFault::SoftLimit`], or [`AlignFault::MoveTooLong`] before anything
  /// moved. [`AlignFault::Move`] if the actuator failed: the axis is then suspect, and its last direction unknown.
  pub async fn approach(&mut self, axis: Axis, delta_steps: i32) -> Result<(), AlignFault> {
    let state = self.known(axis)?;
    self.check_approach(axis, Planned::from(state), delta_steps)?;
    let plan = approach::plan(delta_steps, state.last_dir, self.config.motion.overshoot_steps);
    for leg in plan.legs() {
      self.leg(axis, leg).await?;
    }
    Ok(())
  }

  /// Checks an approach of `delta_steps` from `from` against the per-move cap and the soft limits, without moving,
  /// and returns where it would leave the axis. A later approach is checked from there, so that a sequence of
  /// moves (a poke and its return, or the four axes of a step) is refused whole before any of it runs.
  fn check_approach(&self, axis: Axis, from: Planned, delta_steps: i32) -> Result<Planned, AlignFault> {
    let plan = approach::plan(delta_steps, from.last_dir, self.config.motion.overshoot_steps);
    self.check_legs(axis, from, plan.legs())
  }

  /// Checks moves of `legs` from `from` against the per-move cap and the soft limits, without moving, and returns
  /// where they would leave the axis. A leg of zero steps moves nothing, but its position is still checked: an axis
  /// outside its limits is refused even an approach that would bring it back.
  fn check_legs(&self, axis: Axis, from: Planned, legs: impl IntoIterator<Item = i32>) -> Result<Planned, AlignFault> {
    let motion = self.config.motion;
    let limits = SoftLimits::for_axis(&motion, axis);
    let longest = u32::try_from(motion.max_move_steps).unwrap_or(0);
    let mut planned = from;
    for leg in legs {
      if leg.unsigned_abs() > longest {
        return Err(AlignFault::MoveTooLong { axis, steps: leg });
      }
      let target_steps = planned.pos_steps.saturating_add(i64::from(leg));
      if !limits.contains(target_steps) {
        return Err(AlignFault::SoftLimit { axis, target_steps });
      }
      planned = Planned {
        pos_steps: target_steps,
        last_dir: match leg.signum() {
          1 => LastDir::Plus,
          -1 => LastDir::Minus,
          _ => planned.last_dir
        }
      };
    }
    Ok(planned)
  }

  /// Measures the Jacobian by poking each axis, and stores it.
  ///
  /// The axes are first brought to the standard direction. Each poke is a plus move, measured,
  /// and then undone with the overshoot and return.
  ///
  /// # Errors
  ///
  /// [`AlignFault::WeakColumn`] for an axis that did not move the beam, [`AlignFault::Calibration`] for a
  /// singular or ill-conditioned result, and any fault of a move or a measurement.
  pub async fn calibrate(&mut self) -> Result<Calibration, AlignFault> {
    let poke_steps = self.config.motion.poke_steps;
    // Every poke and its return are checked before the first move runs, so that a refused return leg cannot leave
    // an axis poked. The standard approach needs no check of its own: it leaves the position where it was, and
    // its overshoot reaches the same lowest point as the return leg's, by a shorter leg.
    for axis in Axis::ALL {
      let standard = Planned {
        pos_steps: i64::from(self.known(axis)?.pos_steps),
        last_dir: LastDir::Plus
      };
      let poked = self.check_approach(axis, standard, poke_steps)?;
      self.check_approach(axis, poked, -poke_steps)?;
    }
    for axis in Axis::ALL {
      if self.axes.get(axis).last_dir != LastDir::Plus {
        self.approach(axis, 0).await?;
      }
    }
    let q0_steps = self.axes.to_array().map(|state| state.pos_steps);
    let baseline = self.measure().await?.vector;
    let mut poked = [Vector4::zeros(); 4];
    for (axis, after) in Axis::ALL.into_iter().zip(poked.iter_mut()) {
      self.approach(axis, poke_steps).await?;
      *after = self.measure().await?.vector;
      self.approach(axis, -poke_steps).await?;
    }
    let poke = solver::steps_to_vector([poke_steps; 4]).x;
    let jacobian = calibration::jacobian_from_pokes(&baseline, &poked, poke);
    if let Some(axis) = calibration::weak_column(&jacobian, poke, &self.config.sensor, &self.config.solver) {
      self.axes.get_mut(axis).suspect = true;
      return Err(AlignFault::WeakColumn { axis });
    }
    let conditioning = self.accept(&jacobian).map_err(AlignFault::Calibration)?;
    let calibration = Calibration {
      jacobian,
      q0_steps,
      cond: conditioning.cond,
      dir_sign: self.config.mechanics.dir_sign
    };
    self.calibration = Some(calibration);
    Ok(calibration)
  }

  /// Aligns the beam: calibrates if there is no stored Jacobian, then iterates until the
  /// measurement vector is inside the tolerance.
  ///
  /// An attempt is at most `max_iterations` solver steps. When an attempt ends without convergence, or the
  /// Jacobian stops being trusted, the Jacobian is measured again and another attempt runs, at most
  /// `max_recalibrations` times.
  ///
  /// # Errors
  ///
  /// [`AlignFault::NotConverged`] when the attempts are used up, [`AlignFault::AxisNotResponding`] when the beam
  /// ignores an axis on consecutive steps, and any fault of a calibration, a move, or a measurement.
  pub async fn align(&mut self) -> Result<AlignReport, AlignFault> {
    let mut iterations = 0_u8;
    let mut recalibrations = 0_u8;
    if self.calibration.is_none() {
      self.calibrate().await?;
    }
    loop {
      match self.attempt(&mut iterations).await? {
        Attempt::Converged(last) => {
          let head_offset = last.shots.head_offset();
          return Ok(AlignReport {
            iterations,
            recalibrations,
            residual_norm: last.vector.amax(),
            head_offset,
            needs_mechanical_centering: head_offset.needs_mechanical_centering(&self.config.sensor),
            cond: self.calibration.map_or(f32::NAN, |calibration| calibration.cond)
          });
        }
        Attempt::Recalibrate { residual_norm } => {
          if recalibrations >= self.config.solver.max_recalibrations {
            return Err(AlignFault::NotConverged { residual_norm });
          }
          recalibrations = recalibrations.saturating_add(1);
          self.calibrate().await?;
        }
      }
    }
  }

  /// One solver attempt with the stored Jacobian.
  async fn attempt(&mut self, iterations: &mut u8) -> Result<Attempt, AlignFault> {
    let solver_config = self.config.solver;
    let mut current = self.measure().await?;
    let Some(calibration) = self.calibration else {
      return Ok(Attempt::Recalibrate {
        residual_norm: current.vector.amax()
      });
    };
    // A calibration measured under other direction signs has the wrong sign in those columns: every step would
    // push those axes the wrong way (the review of 2026-11-15 walked the beam off the sensor that way).
    let signs_match = calibration.dir_sign == self.config.mechanics.dir_sign;
    let Some(conditioning) = self.accept(&calibration.jacobian).ok().filter(|_| signs_match) else {
      return Ok(Attempt::Recalibrate {
        residual_norm: current.vector.amax()
      });
    };
    let mut progress = Progress {
      jacobian: calibration.jacobian,
      lambda: conditioning.damping(&solver_config),
      poor_steps: 0,
      suspect_streak: None
    };
    let poke = solver::steps_to_vector([self.config.motion.poke_steps; 4]).x;
    let limit = solver::step_limit(poke, &solver_config);

    let mut steps_left = solver_config.max_iterations;
    loop {
      if solver::converged(&current.vector, &solver_config) {
        // A success is confirmed by a second measurement, with no move between: the mean of the two must be inside
        // the tolerance too. On one measurement, 9 alignments in 1000 stopped with the true residual above 0.03 at
        // the configured noise (review of 2026-11-15). Requiring both inside instead tripled the recalibrations at
        // that noise, for one false success fewer. A mean outside is the next measurement to correct from.
        let again = self.measure().await?;
        let mean = Measured {
          vector: (current.vector + again.vector) / 2.0,
          linear: current.linear && again.linear,
          ..again
        };
        if solver::converged(&mean.vector, &solver_config) {
          return Ok(self.converged(mean));
        }
        current = mean;
      }
      if steps_left == 0 {
        break;
      }
      steps_left -= 1;
      let dq = solver::dls_step(
        &progress.jacobian,
        &current.vector,
        progress.lambda,
        solver_config.alpha,
        &limit
      )
      .map_err(AlignFault::Solve)?;
      let steps = solver::round_to_steps(&dq);
      if steps == [0; 4] {
        // The residual is above the tolerance, yet the correction is under half a step on every axis.
        break;
      }
      *iterations = iterations.saturating_add(1);
      // The whole step is checked before any axis moves, so that a refusal does not leave it half taken.
      for (axis, delta_steps) in Axis::ALL.into_iter().zip(steps) {
        if delta_steps != 0 {
          self.check_approach(axis, Planned::from(self.known(axis)?), delta_steps)?;
        }
      }
      for (axis, delta_steps) in Axis::ALL.into_iter().zip(steps) {
        if delta_steps != 0 {
          self.approach(axis, delta_steps).await?;
        }
      }
      let taken = solver::steps_to_vector(steps);
      let clamped = dq.iter().zip(limit.iter()).any(|(step, bound)| step.abs() >= *bound);
      let mut next = self.measure().await?;
      if self.looks_suspect(&progress.jacobian, &current, &next, &taken) {
        // Addendum A.5: repeat the measurement before the move is called suspect.
        next = self.measure().await?;
      }
      let verdict = self.judge(&mut progress, &current, &next, &taken, clamped)?;
      current = next;
      if matches!(verdict, Verdict::Recalibrate) {
        return Ok(Attempt::Recalibrate {
          residual_norm: current.vector.amax()
        });
      }
    }

    Ok(Attempt::Recalibrate {
      residual_norm: current.vector.amax()
    })
  }

  /// A confirmed success: every axis is trusted again.
  ///
  /// The Broyden updates of the attempt are not stored: each carries the noise of one step, and a stored Jacobian
  /// that gathered them over sessions drifted a column 50% wrong in 479 of 1200 sessions in the review of
  /// 2026-11-15. The stored calibration changes only when the axes are poked again.
  fn converged(&mut self, last: Measured) -> Attempt {
    for axis in Axis::ALL {
      self.axes.get_mut(axis).suspect = false;
    }
    Attempt::Converged(last)
  }

  /// Judges one completed step: suspect, learned from, or poor.
  fn judge(
    &mut self,
    progress: &mut Progress,
    before: &Measured,
    after: &Measured,
    taken: &Vector4<f32>,
    clamped: bool
  ) -> Result<Verdict, AlignFault> {
    let solver_config = self.config.solver;
    // A residual that grew by more than the noise of a change says the step went the wrong way, however small
    // the step or far the readings: the stored Jacobian has a wrong sign. That needs no judging floor and holds
    // outside the linear range too, where the response is still monotonic. The review of 2026-11-15 saw a wrong
    // column walk the beam off the sensor in 49 runs of 50 through steps too small or too coarse to be judged.
    let grown = after.vector.norm() - before.vector.norm();
    let worse = grown > solver_config.judge_noise_sigmas * calibration::change_sigma(&self.config.sensor);
    let judgeable = before.linear
      && after.linear
      && (progress.jacobian * taken).norm() >= calibration::judging_floor(&self.config.sensor, &solver_config);
    if !judgeable {
      // A coarse step, or one too small to judge: inside noise and stick-slip, or outside the linear range.
      // Nothing is learned from it about the Jacobian.
      return Ok(if worse {
        Self::poor_step(progress, &solver_config)
      } else {
        Verdict::Continue
      });
    }
    let dm = after.vector - before.vector;
    let unresponsive = if solver::prediction_mismatch(&progress.jacobian, taken, &dm, &solver_config) {
      let change_sigma = calibration::change_sigma(&self.config.sensor);
      solver::unresponsive_axis(&progress.jacobian, taken, &dm, change_sigma, &solver_config)
    } else {
      None
    };
    if let Some(axis) = unresponsive {
      self.axes.get_mut(axis).suspect = true;
      let streak = match progress.suspect_streak {
        Some((previous, count)) if previous == axis => count.saturating_add(1),
        _ => 1
      };
      progress.suspect_streak = Some((axis, streak));
      if streak >= solver_config.suspect_fault_streak {
        return Err(AlignFault::AxisNotResponding { axis });
      }
      // An axis that went backward is named too, and its column's sign is what is wrong. A dead axis leaves the
      // residual where it was; a reversed one makes it grow, and that is a poor step besides.
      return Ok(if worse {
        Self::poor_step(progress, &solver_config)
      } else {
        Verdict::Continue
      });
    }
    // Either the step went as predicted, or it did not and no axis fell short: then the Jacobian is wrong, and
    // the update below is how it is corrected.
    progress.suspect_streak = None;

    if let Some(updated) = solver::broyden_update(&progress.jacobian, taken, &dm, &solver_config) {
      if self.accept(&updated).is_err() {
        // Reference 3.2: discard the update and recalibrate.
        return Ok(Verdict::Recalibrate);
      }
      progress.jacobian = updated;
    }

    let reduced_enough =
      after.vector.norm() <= (1.0 - solver_config.min_residual_reduction_frac) * before.vector.norm();
    // A clamped step cannot reduce the residual by the required fraction however well it went, so it is good as
    // long as the residual did not clearly grow.
    if reduced_enough || (clamped && !worse) {
      progress.poor_steps = 0;
      progress.lambda /= solver_config.lambda_adjust_factor;
      return Ok(Verdict::Continue);
    }
    Ok(Self::poor_step(progress, &solver_config))
  }

  /// Counts a poor step: more damping, and a recalibration after `poor_steps_before_recalibration` in a row.
  fn poor_step(progress: &mut Progress, solver_config: &Solver) -> Verdict {
    progress.poor_steps = progress.poor_steps.saturating_add(1);
    progress.lambda *= solver_config.lambda_adjust_factor;
    if progress.poor_steps >= solver_config.poor_steps_before_recalibration {
      Verdict::Recalibrate
    } else {
      Verdict::Continue
    }
  }

  /// Whether a step can be judged at all and, if so, whether the beam disagreed with the prediction.
  fn looks_suspect(&self, jacobian: &Jacobian, before: &Measured, after: &Measured, taken: &Vector4<f32>) -> bool {
    before.linear
      && after.linear
      && (jacobian * taken).norm() >= calibration::judging_floor(&self.config.sensor, &self.config.solver)
      && solver::prediction_mismatch(jacobian, taken, &(after.vector - before.vector), &self.config.solver)
  }

  /// Conditioning of a Jacobian, if it is inside the configured limit.
  fn accept(&self, jacobian: &Jacobian) -> Result<Conditioning, SolveFault> {
    let conditioning = Conditioning::of(jacobian)?;
    conditioning.check(&self.config.solver)?;
    Ok(conditioning)
  }

  /// One measurement that passes the validity checks, retried up to `max_invalid_shots` times.
  async fn measure(&mut self) -> Result<Measured, AlignFault> {
    let sensor_limits = self.config.sensor;
    let mut rejected = ReadingFault::NoSignal;
    for _ in 0..sensor_limits.max_invalid_shots.max(1) {
      let shots = match self.sensor.measure().await {
        Ok(shots) => shots,
        // The node's own rejection is an invalid shot, taken again like one that fails the checks below.
        Err(SensorFault::Rejected(fault)) => {
          rejected = fault;
          continue;
        }
        Err(fault) => return Err(AlignFault::Sensor(fault))
      };
      let readings = [shots.a, shots.b, shots.c];
      match readings
        .iter()
        .try_for_each(|reading| reading.check(self.sum_ref, &sensor_limits))
      {
        Ok(()) => {
          return Ok(Measured {
            shots,
            vector: (self.target)(&shots),
            linear: readings.iter().all(|reading| reading.in_solver_range(&sensor_limits))
          });
        }
        Err(fault) => rejected = fault
      }
    }
    Err(AlignFault::Reading(rejected))
  }

  /// One leg of an approach, with the bookkeeping that follows it.
  async fn leg(&mut self, axis: Axis, steps: i32) -> Result<(), AlignFault> {
    if steps == 0 {
      return Ok(());
    }
    let outcome = self.actuator.move_rel(axis, steps).await;
    let state = self.axes.get_mut(axis);
    match outcome {
      Ok(pos_steps) => {
        // The node's count is adopted; one that disagrees with this count means one of them is wrong.
        if pos_steps != state.pos_steps.saturating_add(steps) {
          state.suspect = true;
        }
        state.pos_steps = pos_steps;
        state.last_dir = if steps > 0 { LastDir::Plus } else { LastDir::Minus };
        Ok(())
      }
      Err(fault) => {
        state.suspect = true;
        state.last_dir = LastDir::Unknown;
        match fault {
          MoveFault::Stopped { pos_steps } | MoveFault::RailLost { pos_steps } => state.pos_steps = pos_steps,
          MoveFault::SoftLimit | MoveFault::Refused { .. } => {}
          // Addendum A.5 asks not to trust the position for the soft limits after a lost move; with no answer
          // there is no position at all to trust, so the axis waits for one from its node.
          MoveFault::NoAnswer => state.lost = true
        }
        Err(AlignFault::Move { axis, fault })
      }
    }
  }
}

/// Whether `value` is a number in `[low, high]`. False for a NaN.
fn within(value: f32, low: f32, high: f32) -> bool {
  low <= value && value <= high
}

/// Whether `value` is a finite number above zero.
fn positive(value: f32) -> bool {
  within(value, f32::MIN_POSITIVE, f32::MAX)
}

/// The first configuration value the procedure cannot run with, if any. Only the values the aligner reads.
fn check_config(config: &Config, sum_ref: f32) -> Result<(), AlignFault> {
  let (motion, sensor, solver) = (&config.motion, &config.sensor, &config.solver);
  let limits_ordered = Axis::ALL
    .into_iter()
    .all(|axis| motion.soft_min_steps.get(axis) <= motion.soft_max_steps.get(axis));
  let checks = [
    (positive(sum_ref), "the reference sum must be a positive number"),
    (motion.poke_steps > 0, "poke_steps must be positive"),
    (motion.overshoot_steps > 0, "overshoot_steps must be positive"),
    (motion.max_move_steps > 0, "max_move_steps must be positive"),
    (limits_ordered, "soft_min_steps must not be above soft_max_steps"),
    (
      within(motion.soft_limit_margin_frac, 0.0, 1.0),
      "soft_limit_margin_frac must be in [0, 1]"
    ),
    (
      within(sensor.sum_min_frac, f32::MIN_POSITIVE, 1.0),
      "sum_min_frac must be in (0, 1]"
    ),
    (
      within(sensor.edge_max_norm, f32::MIN_POSITIVE, 1.0),
      "edge_max_norm must be in (0, 1]"
    ),
    (
      within(sensor.solver_max_norm, f32::MIN_POSITIVE, sensor.edge_max_norm),
      "solver_max_norm must be in (0, edge_max_norm]"
    ),
    (
      within(sensor.offset_warn_frac, 0.0, 1.0),
      "offset_warn_frac must be in [0, 1]"
    ),
    (
      positive(sensor.noise_sigma_norm),
      "noise_sigma_norm must be a positive number"
    ),
    (within(solver.alpha, f32::MIN_POSITIVE, 1.0), "alpha must be in (0, 1]"),
    (within(solver.lambda_frac, 0.0, 1.0), "lambda_frac must be in [0, 1]"),
    (
      positive(solver.converge_tol_norm),
      "converge_tol_norm must be a positive number"
    ),
    (solver.max_iterations > 0, "max_iterations must be positive"),
    (
      within(solver.cond_max, 1.0, f32::MAX),
      "cond_max must be a number of at least 1"
    ),
    (
      positive(solver.step_clamp_pokes),
      "step_clamp_pokes must be a positive number"
    ),
    (
      within(solver.prediction_mismatch_frac, f32::MIN_POSITIVE, 1.0),
      "prediction_mismatch_frac must be in (0, 1]"
    ),
    (
      within(solver.broyden_min_step_norm_steps, 0.0, f32::MAX),
      "broyden_min_step_norm_steps must be a number of at least 0"
    ),
    (
      within(solver.lambda_adjust_factor, 1.0, f32::MAX),
      "lambda_adjust_factor must be a number of at least 1"
    ),
    (
      within(solver.min_residual_reduction_frac, 0.0, 1.0),
      "min_residual_reduction_frac must be in [0, 1]"
    ),
    (
      solver.poor_steps_before_recalibration > 0,
      "poor_steps_before_recalibration must be positive"
    ),
    (
      positive(solver.min_column_sigmas),
      "min_column_sigmas must be a positive number"
    ),
    (solver.suspect_fault_streak > 0, "suspect_fault_streak must be positive"),
    (
      positive(solver.judge_noise_sigmas),
      "judge_noise_sigmas must be a positive number"
    )
  ];
  match checks
    .into_iter()
    .chain(bring_up_checks(&config.bring_up))
    .find(|(ok, _)| !ok)
  {
    Some((_, what)) => Err(AlignFault::Config(what)),
    None => Ok(())
  }
}

/// The checks of [`check_config`] on the bring-up measurements, each with what it requires.
fn bring_up_checks(bring_up: &BringUp) -> [(bool, &'static str); 6] {
  let trials = &bring_up.backlash_trials_steps;
  [
    (
      (2..=bringup::MAX_REPEAT_SHOTS).contains(&bring_up.repeat_shots),
      "repeat_shots must be at least 2 and at most MAX_REPEAT_SHOTS"
    ),
    (
      trials.first().is_some_and(|first| *first > 0) && trials.windows(2).all(|pair| pair.first() < pair.last()),
      "backlash_trials_steps must be positive and increasing"
    ),
    (
      positive(bring_up.detect_sigmas),
      "detect_sigmas must be a positive number"
    ),
    (
      bring_up.hysteresis_amplitude_pokes > 0,
      "hysteresis_amplitude_pokes must be positive"
    ),
    (
      (1..=bringup::MAX_MOVES_PER_HALF).contains(&bring_up.hysteresis_moves_per_half),
      "hysteresis_moves_per_half must be at least 1 and at most MAX_MOVES_PER_HALF"
    ),
    (
      bring_up.overshoot_floor_steps >= 0,
      "overshoot_floor_steps must not be negative"
    )
  ]
}

mod bringup_procedures;

#[cfg(test)]
mod tests;
