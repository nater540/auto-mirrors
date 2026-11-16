//! The alignment procedure: establish the standard direction, calibrate, then iterate to zero (hand-off 3.2,
//! reference 3.1 and 3.2, addendum A.5).
//!
//! [`Aligner`] owns an [`Actuator`], a [`BeamSensor`], the configuration, and what the coordinator knows about
//! each axis. It strictly alternates moving and measuring. Every outcome that is not success is an
//! [`AlignFault`] value.
//!
//! What the documents leave open, and what is done here:
//!
//! - **A reading outside the solver's linear range** (reference 2.2: "use sign and coarse steps only"). The step
//!   is still taken, because the sensor response is monotonic and the step points the right way, but nothing is
//!   learned from it: no Broyden update, no suspect judgment, no damping change.
//! - **A step too small to see.** Addendum A.5 compares the measured change with the prediction after every
//!   step. For a step of a few counts the prediction is inside sensor noise and seat stick-slip, the comparison
//!   is meaningless, and applying it reports a healthy axis as not responding (the simulator shows this). A step
//!   whose predicted change is below [`crate::calibration::noise_floor`] is therefore taken but not judged: no
//!   suspect, no Broyden update, no damping change. The reference already skips the Broyden update for small
//!   steps (3.2); this extends the same guard.
//! - **A step limited by the clamp** cannot reduce the residual by the required fraction however well it went, so
//!   it is not judged as a poor step.
//! - **Which axis is suspect** when four moved together, and telling a dead axis from a wrong Jacobian:
//!   [`crate::solver::unresponsive_axis`]. A mismatch in which no axis fell short is a Jacobian error, and the
//!   Broyden update corrects it.
//! - **A reading that leaves the linear range during a calibration poke** is accepted as long as it passes the
//!   validity checks. The reference rejects a calibration only on conditioning and weak columns.

use nalgebra::Vector4;

use crate::approach::{self, LastDir, SoftLimits};
use crate::axis::{Axis, PerAxis};
use crate::calibration::{self, Calibration};
use crate::config::Config;
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
  pub suspect: bool
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
  /// A move would have left the soft limits. Nothing was moved; manual pre-alignment is required.
  #[error("axis {axis:?}: a move to {target_steps} steps is outside the soft limits")]
  SoftLimit {
    /// The axis.
    axis: Axis,
    /// The commanded position the move would have reached.
    target_steps: i64
  },
  /// A single move would have been longer than the per-move cap. Nothing was moved.
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
  /// A calibration poke barely moved the measurement: the axis is dead or disconnected.
  #[error("axis {axis:?}: the calibration poke did not move the beam")]
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
  pub fn new(actuator: A, sensor: S, config: Config, sum_ref: f32) -> Self {
    Self {
      actuator,
      sensor,
      config,
      sum_ref,
      axes: PerAxis::splat(AxisState::default()),
      calibration: None,
      target: Shots::measurement
    }
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

  /// The calibration in use, if any. After a successful alignment it carries the Broyden updates.
  pub fn calibration(&self) -> Option<&Calibration> {
    self.calibration.as_ref()
  }

  /// Moves one axis by `delta_steps` with the standard approach, so that it ends in the plus direction.
  ///
  /// Every leg is checked against the soft limits and the per-move cap before the first one runs.
  ///
  /// # Errors
  ///
  /// [`AlignFault::SoftLimit`] or [`AlignFault::MoveTooLong`] before anything moved, or [`AlignFault::Move`] if
  /// the actuator failed; the axis is then marked suspect and its last direction unknown.
  pub async fn approach(&mut self, axis: Axis, delta_steps: i32) -> Result<(), AlignFault> {
    let motion = self.config.motion;
    let state = self.axes.get(axis);
    let plan = approach::plan(delta_steps, state.last_dir, motion.overshoot_steps);
    let limits = SoftLimits::for_axis(&motion, axis);
    let longest = u32::try_from(motion.max_move_steps).unwrap_or(0);
    let mut target_steps = i64::from(state.pos_steps);
    for leg in plan.legs() {
      if leg.unsigned_abs() > longest {
        return Err(AlignFault::MoveTooLong { axis, steps: leg });
      }
      target_steps += i64::from(leg);
      if !limits.contains(target_steps) {
        return Err(AlignFault::SoftLimit { axis, target_steps });
      }
    }
    for leg in plan.legs() {
      self.leg(axis, leg).await?;
    }
    Ok(())
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
    if poke_steps <= 0 {
      return Err(AlignFault::Config("poke_steps must be positive"));
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
      cond: conditioning.cond
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
    let Ok(conditioning) = self.accept(&calibration.jacobian) else {
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

    for _ in 0..solver_config.max_iterations {
      if solver::converged(&current.vector, &solver_config) {
        break;
      }
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

    if !solver::converged(&current.vector, &solver_config) {
      return Ok(Attempt::Recalibrate {
        residual_norm: current.vector.amax()
      });
    }
    if let Some(stored) = self.calibration.as_mut() {
      stored.jacobian = progress.jacobian;
      if let Ok(updated) = Conditioning::of(&progress.jacobian) {
        stored.cond = updated.cond;
      }
    }
    for axis in Axis::ALL {
      self.axes.get_mut(axis).suspect = false;
    }
    Ok(Attempt::Converged(current))
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
    if !(before.linear && after.linear) {
      // A coarse step: the data is outside the linear range and teaches nothing.
      return Ok(Verdict::Continue);
    }
    let noise_floor = calibration::noise_floor(&self.config.sensor, &solver_config);
    if (progress.jacobian * taken).norm() < noise_floor {
      // Too small to see: inside noise and stick-slip.
      return Ok(Verdict::Continue);
    }
    let dm = after.vector - before.vector;
    let unresponsive = if solver::prediction_mismatch(&progress.jacobian, taken, &dm, &solver_config) {
      solver::unresponsive_axis(&progress.jacobian, taken, &dm, noise_floor, &solver_config)
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
      return Ok(Verdict::Continue);
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
    if reduced_enough || clamped {
      progress.poor_steps = 0;
      progress.lambda /= solver_config.lambda_adjust_factor;
    } else {
      progress.poor_steps = progress.poor_steps.saturating_add(1);
      progress.lambda *= solver_config.lambda_adjust_factor;
      if progress.poor_steps >= solver_config.poor_steps_before_recalibration {
        return Ok(Verdict::Recalibrate);
      }
    }
    Ok(Verdict::Continue)
  }

  /// Whether a step can be judged at all and, if so, whether the beam disagreed with the prediction.
  fn looks_suspect(&self, jacobian: &Jacobian, before: &Measured, after: &Measured, taken: &Vector4<f32>) -> bool {
    before.linear
      && after.linear
      && (jacobian * taken).norm() >= calibration::noise_floor(&self.config.sensor, &self.config.solver)
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
      let shots = self.sensor.measure().await.map_err(AlignFault::Sensor)?;
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
      Ok(()) => {
        state.pos_steps = state.pos_steps.saturating_add(steps);
        state.last_dir = if steps > 0 { LastDir::Plus } else { LastDir::Minus };
        Ok(())
      }
      Err(fault) => {
        state.suspect = true;
        state.last_dir = LastDir::Unknown;
        Err(AlignFault::Move { axis, fault })
      }
    }
  }
}
