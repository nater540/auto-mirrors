//! The model of the machine: mirrors, screws, and the quadrant sensor.
//!
//! Optics, per transverse direction (reference 1.3):
//!
//! ```text
//! s(Ly, Lx) = s0 + e1 * Ly + e2 * Lx
//! e1 = angle after mirror 1 (moved by mirror 1 only)
//! e2 = angle after mirror 2 (moved by both mirrors: mirror 1 changes the angle into mirror 2)
//! ```
//!
//! `Ly` is the path from mirror 1 to mirror 2, `Lx` from mirror 2 to the head, and `s0` a constant
//! offset that no tilt removes. A screw `j` sits at `seat_j` steps and the beam is aligned when every
//! `seat_j` equals minus the scenario's misalignment.
//!
//! Each screw is a chain of three positions, all in steps:
//!
//! - `motor`: where the rotor is. It follows the commanded count unless a step is skipped.
//! - `thread`: the screw itself, which follows the motor through gearbox and thread backlash, a play of
//!   `backlash_steps`. After plus travel the screw equals the motor.
//! - `seat`: the mirror plate on its three over-constrained seats. It sticks until the screw has moved
//!   `stick_steps` away, then slips onto it.
//!
//! The sensor is a square four-quadrant detector under a Gaussian beam of 1/e^2 radius `w` (reference 2.2). With
//! a large aperture the normalized position is `erf(sqrt(2) * x0 / w)`, which is the sensor's linear range limit.

use align_core::axis::{Axis, PerAxis};
use align_core::config::Mechanics;
use align_core::measurement::Shots;
use align_core::quadrant::{Quadrants, Reading};
use align_core::solver::Jacobian;
use align_core::units;
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use rand_distr::StandardNormal;

/// Path lengths at the ends of gantry travel, in millimeters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Geometry {
  /// Mirror 1 to mirror 2 with the gantry at Y near.
  pub ly_near_mm: f64,
  /// Mirror 1 to mirror 2 with the gantry at Y far.
  pub ly_far_mm: f64,
  /// Mirror 2 to the head at X near.
  pub lx_near_mm: f64,
  /// Mirror 2 to the head at X far.
  pub lx_far_mm: f64
}

impl Geometry {
  /// UNVERIFIED placeholder (hand-off open item 5): spans of 500 mm in Y and 600 mm in X,
  /// inside the 0.1 to 1.5 m range the reference assumes.
  pub const PLACEHOLDER: Self = Self {
    ly_near_mm: 100.0,
    ly_far_mm: 600.0,
    lx_near_mm: 100.0,
    lx_far_mm: 700.0
  };

  /// The path lengths `(Ly, Lx)` at the measurement points A, B, and C.
  pub const fn points(&self) -> [(f64, f64); 3] {
    [
      (self.ly_near_mm, self.lx_near_mm),
      (self.ly_far_mm, self.lx_near_mm),
      (self.ly_far_mm, self.lx_far_mm)
    ]
  }
}

/// The beam and the detector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Beam {
  /// Gaussian 1/e^2 radius at the head.
  pub radius_um: f64,
  /// Half the side of the square detector.
  pub aperture_half_um: f64,
  /// Sum of the four quadrants for a centered beam.
  pub sum_ref: f64,
  /// Noise on each quadrant, one sigma, in the units of `sum_ref`.
  pub quadrant_noise_sigma: f64
}

impl Beam {
  /// UNVERIFIED placeholder (hand-off open item 6): the 4 mm radius of the reference's worked
  /// example on a 20 mm detector, without noise.
  pub const PLACEHOLDER: Self = Self {
    radius_um: 4_000.0,
    aperture_half_um: 10_000.0,
    sum_ref: 1_000.0,
    quadrant_noise_sigma: 0.0
  };

  /// Sets the quadrant noise so that one normalized coordinate of a centered beam has the given sigma. The
  /// coordinate is `(right - left) / sum`, so its sigma is twice the quadrant sigma over the sum.
  #[must_use]
  pub fn with_position_noise(mut self, sigma_norm: f64) -> Self {
    self.quadrant_noise_sigma = sigma_norm * self.sum_ref / 2.0;
    self
  }
}

/// The imperfections of one screw.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AxisModel {
  /// Play between motor and screw.
  pub backlash_steps: u32,
  /// Screw travel the seat resists before it slips. Zero or one means the seat follows every step.
  pub stick_steps: u32,
  /// Chance that a commanded step does not move the rotor. `1.0` is a dead axis.
  pub skip_probability: f64
}

impl AxisModel {
  /// A screw that does exactly what it is told.
  pub const IDEAL: Self = Self {
    backlash_steps: 0,
    stick_steps: 0,
    skip_probability: 0.0
  };
  /// An axis whose motor never moves.
  pub const DEAD: Self = Self {
    backlash_steps: 0,
    stick_steps: 0,
    skip_probability: 1.0
  };
}

/// Everything that defines one simulated machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Scenario {
  /// Mount geometry; the same struct the alignment code is configured with.
  pub mechanics: Mechanics,
  /// Path lengths.
  pub geometry: Geometry,
  /// Beam and detector.
  pub beam: Beam,
  /// Imperfections of each screw.
  pub axes: PerAxis<AxisModel>,
  /// Fraction of each screw's effect that leaks into the other transverse direction.
  pub cross_coupling: f64,
  /// Constant beam offset at the head, horizontal then vertical.
  pub head_offset_um: [f64; 2],
  /// How far each screw starts from its aligned position.
  pub misalignment_steps: PerAxis<f64>,
  /// Seed for the noise and the skipped steps.
  pub seed: u64
}

impl Scenario {
  /// A perfect machine, already aligned: no backlash, no stick-slip, no noise, no offset.
  pub const IDEAL: Self = Self {
    mechanics: Mechanics::STARTING,
    geometry: Geometry::PLACEHOLDER,
    beam: Beam::PLACEHOLDER,
    axes: PerAxis::splat(AxisModel::IDEAL),
    cross_coupling: 0.0,
    head_offset_um: [0.0; 2],
    misalignment_steps: PerAxis::splat(0.0),
    seed: 0
  };

  /// A machine with every imperfection at an UNVERIFIED placeholder size: 20 steps of backlash (about 3.5 degrees
  /// of screw; addendum A.0 expects "several degrees"), a seat that sticks for 3 steps, position noise of 0.002
  /// normalized, and 5% cross-coupling. Already aligned; set the misalignment and the offset per test.
  pub fn realistic(seed: u64) -> Self {
    Self {
      beam: Beam::PLACEHOLDER.with_position_noise(0.002),
      axes: PerAxis::splat(AxisModel {
        backlash_steps: 20,
        stick_steps: 3,
        skip_probability: 0.0
      }),
      cross_coupling: 0.05,
      seed,
      ..Self::IDEAL
    }
  }
}

/// The true beam angles relative to the machine axes, in microradians. All zero is perfect alignment.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AngleErrors {
  /// Leg 1 (mirror 1 to mirror 2), horizontal.
  pub leg1_h_urad: f64,
  /// Leg 1, vertical.
  pub leg1_v_urad: f64,
  /// Leg 2 (mirror 2 to the head), horizontal.
  pub leg2_h_urad: f64,
  /// Leg 2, vertical.
  pub leg2_v_urad: f64
}

impl AngleErrors {
  /// The largest of the four, as a magnitude.
  pub fn largest_urad(&self) -> f64 {
    [self.leg1_h_urad, self.leg1_v_urad, self.leg2_h_urad, self.leg2_v_urad]
      .into_iter()
      .fold(0.0, |largest, angle| largest.max(angle.abs()))
  }
}

/// One thing the rig was asked to do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Record {
  /// A move command.
  Move {
    /// The axis.
    axis: Axis,
    /// The commanded steps.
    delta_steps: i32
  },
  /// A measurement cycle.
  Measure {
    /// Infinity norm of the noise-free three-position difference vector at that moment.
    true_residual_norm: f32
  }
}

/// The state of one screw.
#[derive(Clone, Copy, Debug)]
struct Screw {
  commanded: i64,
  motor: i64,
  thread: i64,
  seat: i64,
  lowest_commanded: i64,
  highest_commanded: i64
}

impl Screw {
  const START: Self = Self {
    commanded: 0,
    motor: 0,
    thread: 0,
    seat: 0,
    lowest_commanded: 0,
    highest_commanded: 0
  };
}

/// A simulated machine.
#[derive(Debug)]
pub struct World {
  scenario: Scenario,
  screws: PerAxis<Screw>,
  rng: StdRng,
  log: Vec<Record>
}

impl World {
  /// A machine in the scenario's starting state: every screw at commanded position zero, engaged on the plus side.
  pub fn new(scenario: Scenario) -> Self {
    Self {
      scenario,
      screws: PerAxis::splat(Screw::START),
      rng: StdRng::seed_from_u64(scenario.seed),
      log: Vec::new()
    }
  }

  /// The scenario this machine was built from.
  pub const fn scenario(&self) -> &Scenario {
    &self.scenario
  }

  /// Replaces the imperfections of one screw, for a fault that appears during a run.
  pub fn set_axis_model(&mut self, axis: Axis, model: AxisModel) {
    *self.scenario.axes.get_mut(axis) = model;
  }

  /// Commands a move, one step at a time through the motor, the backlash, and the seat.
  pub fn move_rel(&mut self, axis: Axis, delta_steps: i32) {
    self.log.push(Record::Move { axis, delta_steps });
    let model = self.scenario.axes.get(axis);
    let direction = i64::from(delta_steps.signum());
    let backlash = i64::from(model.backlash_steps);
    let stick = i64::from(model.stick_steps);
    for _ in 0..delta_steps.unsigned_abs() {
      let skipped = model.skip_probability > 0.0 && self.rng.random_bool(model.skip_probability.clamp(0.0, 1.0));
      let screw = self.screws.get_mut(axis);
      screw.commanded += direction;
      screw.lowest_commanded = screw.lowest_commanded.min(screw.commanded);
      screw.highest_commanded = screw.highest_commanded.max(screw.commanded);
      if !skipped {
        screw.motor += direction;
      }
      screw.thread = screw.thread.clamp(screw.motor, screw.motor + backlash);
      if (screw.thread - screw.seat).abs() >= stick {
        screw.seat = screw.thread;
      }
    }
  }

  /// One measurement cycle: a noisy reading at each of A, B, and C.
  pub fn measure(&mut self) -> Shots {
    let true_residual_norm = self.true_shots().measurement().amax();
    self.log.push(Record::Measure { true_residual_norm });
    let [a, b, c] = self.scenario.geometry.points().map(|(ly_mm, lx_mm)| {
      let noise = core::array::from_fn(|_| {
        let unit: f64 = self.rng.sample(StandardNormal);
        unit * self.scenario.beam.quadrant_noise_sigma
      });
      self.reading(ly_mm, lx_mm, noise)
    });
    Shots { a, b, c }
  }

  /// The readings a noise-free sensor would give now.
  pub fn true_shots(&self) -> Shots {
    let [a, b, c] = self
      .scenario
      .geometry
      .points()
      .map(|(ly_mm, lx_mm)| self.reading(ly_mm, lx_mm, [0.0; 4]));
    Shots { a, b, c }
  }

  /// The true beam angles.
  pub fn angle_errors(&self) -> AngleErrors {
    let coupling = self.scenario.cross_coupling;
    let tilt = |axis: Axis| {
      #[expect(clippy::cast_precision_loss, reason = "seat positions are a few thousand steps")]
      let offset_steps = self.screws.get(axis).seat as f64 + self.scenario.misalignment_steps.get(axis);
      offset_steps * self.gain_urad_per_step(axis)
    };
    let leg1_h_urad = tilt(Axis::M1H) + coupling * tilt(Axis::M1V);
    let leg1_v_urad = tilt(Axis::M1V) + coupling * tilt(Axis::M1H);
    AngleErrors {
      leg1_h_urad,
      leg1_v_urad,
      leg2_h_urad: leg1_h_urad + tilt(Axis::M2H) + coupling * tilt(Axis::M2V),
      leg2_v_urad: leg1_v_urad + tilt(Axis::M2V) + coupling * tilt(Axis::M2H)
    }
  }

  /// The true beam position at the head for the given path lengths, horizontal then vertical, in micrometers.
  pub fn beam_um(&self, ly_mm: f64, lx_mm: f64) -> [f64; 2] {
    let angles = self.angle_errors();
    let [offset_h, offset_v] = self.scenario.head_offset_um;
    // Microradians times millimeters is nanometers.
    [
      offset_h + (angles.leg1_h_urad * ly_mm + angles.leg2_h_urad * lx_mm) * 1.0e-3,
      offset_v + (angles.leg1_v_urad * ly_mm + angles.leg2_v_urad * lx_mm) * 1.0e-3
    ]
  }

  /// The Jacobian of the three-position difference vector at the center of the sensor, from the model's own
  /// parameters. The alignment code never sees this; tests use it as the truth to compare with and to corrupt.
  pub fn true_jacobian(&self) -> Jacobian {
    let geometry = self.scenario.geometry;
    let slope_per_um = f64::from(units::GAUSSIAN_CENTER_SLOPE) / self.scenario.beam.radius_um;
    let dy = (geometry.ly_far_mm - geometry.ly_near_mm) * 1.0e-3 * slope_per_um;
    let dx = (geometry.lx_far_mm - geometry.lx_near_mm) * 1.0e-3 * slope_per_um;
    let coupling = self.scenario.cross_coupling;
    let [g1h, g1v, g2h, g2v] = Axis::ALL.map(|axis| self.gain_urad_per_step(axis));
    #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
    let entry = |value: f64| value as f32;
    Jacobian::new(
      entry(g1h * dy),
      entry(coupling * g1v * dy),
      0.0,
      0.0,
      entry(coupling * g1h * dy),
      entry(g1v * dy),
      0.0,
      0.0,
      entry(g1h * dx),
      entry(coupling * g1v * dx),
      entry(g2h * dx),
      entry(coupling * g2v * dx),
      entry(coupling * g1h * dx),
      entry(g1v * dx),
      entry(coupling * g2h * dx),
      entry(g2v * dx)
    )
  }

  /// The commanded position of an axis: what the node counts, not where the screw is.
  pub fn commanded_steps(&self, axis: Axis) -> i64 {
    self.screws.get(axis).commanded
  }

  /// The lowest and highest commanded position an axis has reached, including the overshoot of every approach.
  pub fn commanded_range(&self, axis: Axis) -> (i64, i64) {
    let screw = self.screws.get(axis);
    (screw.lowest_commanded, screw.highest_commanded)
  }

  /// Everything the rig was asked to do, in order.
  pub fn log(&self) -> &[Record] {
    &self.log
  }

  /// Beam angle per full step of one axis, in microradians.
  fn gain_urad_per_step(&self, axis: Axis) -> f64 {
    let mechanics = &self.scenario.mechanics;
    f64::from(mechanics.deflect_k.get(axis) * units::mirror_tilt_urad(mechanics, axis, 1.0))
  }

  /// What the sensor node reports for a beam at the given path lengths, with the given noise on its quadrants.
  fn reading(&self, ly_mm: f64, lx_mm: f64, noise: [f64; 4]) -> Reading {
    let beam = self.scenario.beam;
    let [x_um, y_um] = self.beam_um(ly_mm, lx_mm);
    let (left, right) = halves(x_um, &beam);
    let (bottom, top) = halves(y_um, &beam);
    let [noise_a, noise_b, noise_c, noise_d] = noise;
    #[expect(clippy::cast_possible_truncation, reason = "the sensor node reports f32")]
    let signal = |fraction: f64, noise: f64| (beam.sum_ref * fraction + noise) as f32;
    let quadrants = Quadrants {
      a: signal(right * top, noise_a),
      b: signal(left * top, noise_b),
      c: signal(left * bottom, noise_c),
      d: signal(right * bottom, noise_d)
    };
    // With no usable signal the node still reports: a zero position and the sum it saw.
    quadrants.normalize().unwrap_or(Reading {
      h_norm: 0.0,
      v_norm: 0.0,
      sum: quadrants.sum().max(0.0)
    })
  }
}

/// The fractions of a Gaussian beam centered at `center_um` that fall on the negative and the positive half of
/// the detector along one direction.
fn halves(center_um: f64, beam: &Beam) -> (f64, f64) {
  let scale = core::f64::consts::SQRT_2 / beam.radius_um;
  let at_center = libm::erf(scale * center_um);
  let positive = 0.5 * (libm::erf(scale * (beam.aperture_half_um - center_um)) + at_center);
  let negative = 0.5 * (libm::erf(scale * (beam.aperture_half_um + center_um)) - at_center);
  (negative, positive)
}

#[cfg(test)]
mod tests {
  use super::*;

  #[track_caller]
  fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
      (actual - expected).abs() <= tolerance,
      "expected {expected} within {tolerance}, got {actual}"
    );
  }

  fn misaligned(steps: [f64; 4]) -> Scenario {
    let [m1_h, m1_v, m2_h, m2_v] = steps;
    Scenario {
      misalignment_steps: PerAxis { m1_h, m1_v, m2_h, m2_v },
      ..Scenario::IDEAL
    }
  }

  #[test]
  fn an_aligned_ideal_machine_reads_zero_everywhere_with_the_full_sum() {
    let world = World::new(Scenario::IDEAL);
    let shots = world.true_shots();
    for reading in [shots.a, shots.b, shots.c] {
      assert_close(f64::from(reading.h_norm), 0.0, 1e-6);
      assert_close(f64::from(reading.v_norm), 0.0, 1e-6);
      assert_close(f64::from(reading.sum), 1_000.0, 0.01);
    }
    assert_close(world.angle_errors().largest_urad(), 0.0, 0.0);
  }

  #[test]
  fn one_step_on_mirror_1_moves_the_beam_over_both_legs_and_on_mirror_2_over_the_second_only() {
    // 14.02 urad per step on a horizontal axis: 14.02 um per meter of path.
    let mut world = World::new(Scenario::IDEAL);
    world.move_rel(Axis::M1H, 1);
    let [at_a, _] = world.beam_um(100.0, 100.0);
    let [at_c, _] = world.beam_um(600.0, 700.0);
    assert_close(at_a, 14.02 * 0.2, 0.01);
    assert_close(at_c, 14.02 * 1.3, 0.01);

    let mut world = World::new(Scenario::IDEAL);
    world.move_rel(Axis::M2H, 1);
    let [at_a, _] = world.beam_um(100.0, 100.0);
    let [at_c, _] = world.beam_um(600.0, 700.0);
    assert_close(at_a, 14.02 * 0.1, 0.01);
    assert_close(at_c, 14.02 * 0.7, 0.01);
  }

  #[test]
  fn a_vertical_screw_moves_the_beam_vertically_by_the_smaller_factor() {
    let mut world = World::new(Scenario::IDEAL);
    world.move_rel(Axis::M2V, 10);
    let [h_um, v_um] = world.beam_um(600.0, 700.0);
    assert_close(h_um, 0.0, 1e-9);
    assert_close(v_um, 9.913 * 0.7 * 10.0, 0.05);
  }

  #[test]
  fn the_model_jacobian_is_block_triangular_and_matches_small_real_moves() {
    let world = World::new(Scenario::IDEAL);
    let jacobian = world.true_jacobian();
    // Mirror 2 does not change the first leg.
    for (row, column) in [(0, 2), (0, 3), (1, 2), (1, 3)] {
      assert_close(f64::from(jacobian[(row, column)]), 0.0, 0.0);
    }
    for axis in Axis::ALL {
      let mut world = World::new(Scenario::IDEAL);
      world.move_rel(axis, 5);
      let measured = world.true_shots().measurement() / 5.0;
      let column = jacobian.column(axis.index());
      assert!(
        (measured - column).amax() <= 2e-5,
        "{axis:?}: {measured} against {column}"
      );
    }
  }

  #[test]
  fn the_model_jacobian_at_equal_spans_has_the_condition_number_of_the_reference_table() {
    let mut scenario = Scenario::IDEAL;
    scenario.geometry = Geometry {
      ly_near_mm: 100.0,
      ly_far_mm: 600.0,
      lx_near_mm: 100.0,
      lx_far_mm: 600.0
    };
    // Equal gains on all four axes, as the reference table assumes.
    scenario.mechanics.deflect_k = PerAxis::splat(2.0);
    let jacobian = World::new(scenario).true_jacobian();
    let conditioning = align_core::solver::Conditioning::of(&jacobian).unwrap();
    assert_close(f64::from(conditioning.cond), 2.62, 0.005);
  }

  #[test]
  fn the_normalized_position_follows_the_error_function_of_the_reference() {
    // 0.28 w off center is the 5% linearity point: X = erf(sqrt(2) * 0.28) = 0.42.
    let mut scenario = Scenario::IDEAL;
    scenario.head_offset_um = [0.28 * 4_000.0, 0.0];
    let reading = World::new(scenario).true_shots().a;
    assert_close(f64::from(reading.h_norm), 0.42, 0.005);
    assert_close(f64::from(reading.v_norm), 0.0, 1e-6);
  }

  #[test]
  fn a_beam_that_leaves_the_detector_loses_its_sum() {
    let mut scenario = Scenario::IDEAL;
    scenario.head_offset_um = [30_000.0, 0.0];
    let reading = World::new(scenario).true_shots().a;
    assert!(reading.sum < 1.0, "sum {}", reading.sum);
  }

  #[test]
  fn a_constant_head_offset_shifts_every_reading_and_leaves_the_difference_vector_alone() {
    let mut scenario = misaligned([10.0, -8.0, 15.0, 5.0]);
    let without = World::new(scenario).true_shots();
    scenario.head_offset_um = [300.0, -200.0];
    let with = World::new(scenario).true_shots();
    assert!((with.a.h_norm - without.a.h_norm).abs() > 0.05);
    // Not exactly equal: the error function bends, which is the linear range limit.
    assert!((with.measurement() - without.measurement()).amax() < 0.01);
  }

  #[test]
  fn backlash_swallows_the_first_steps_after_a_reversal() {
    let mut scenario = Scenario::IDEAL;
    scenario.axes.m2_h = AxisModel {
      backlash_steps: 20,
      stick_steps: 0,
      skip_probability: 0.0
    };
    let mut world = World::new(scenario);
    world.move_rel(Axis::M2H, 50);
    let advanced = world.angle_errors().leg2_h_urad;
    world.move_rel(Axis::M2H, -20);
    assert_close(world.angle_errors().leg2_h_urad, advanced, 1e-9);
    world.move_rel(Axis::M2H, -10);
    assert_close(world.angle_errors().leg2_h_urad, advanced - 10.0 * 14.02, 0.05);
    // Coming back, the same 20 steps are lost again before the screw follows.
    world.move_rel(Axis::M2H, 30);
    assert_close(world.angle_errors().leg2_h_urad, advanced, 1e-9);
  }

  #[test]
  fn a_move_that_ends_in_the_plus_direction_lands_where_it_was_commanded_despite_backlash() {
    let mut scenario = Scenario::IDEAL;
    scenario.axes.m1_v = AxisModel {
      backlash_steps: 20,
      stick_steps: 0,
      skip_probability: 0.0
    };
    let mut world = World::new(scenario);
    world.move_rel(Axis::M1V, -170);
    world.move_rel(Axis::M1V, 120);
    assert_close(world.angle_errors().leg1_v_urad, -50.0 * 9.913, 0.05);
  }

  #[test]
  fn a_sticking_seat_holds_then_slips_onto_the_screw() {
    let mut scenario = Scenario::IDEAL;
    scenario.axes.m2_h = AxisModel {
      backlash_steps: 0,
      stick_steps: 3,
      skip_probability: 0.0
    };
    let mut world = World::new(scenario);
    world.move_rel(Axis::M2H, 2);
    assert_close(world.angle_errors().leg2_h_urad, 0.0, 0.0);
    world.move_rel(Axis::M2H, 1);
    assert_close(world.angle_errors().leg2_h_urad, 3.0 * 14.02, 0.02);
  }

  #[test]
  fn a_dead_axis_counts_steps_and_moves_nothing() {
    let mut scenario = Scenario::IDEAL;
    scenario.axes.m1_h = AxisModel::DEAD;
    let mut world = World::new(scenario);
    world.move_rel(Axis::M1H, 100);
    assert_eq!(world.commanded_steps(Axis::M1H), 100);
    assert_close(world.angle_errors().largest_urad(), 0.0, 0.0);
  }

  #[test]
  fn skipped_steps_leave_the_screw_short_of_its_count() {
    let mut scenario = Scenario::IDEAL;
    scenario.axes.m1_h = AxisModel {
      backlash_steps: 0,
      stick_steps: 0,
      skip_probability: 0.1
    };
    scenario.seed = 7;
    let mut world = World::new(scenario);
    world.move_rel(Axis::M1H, 1_000);
    let moved_steps = world.angle_errors().leg1_h_urad / 14.02;
    assert!((850.0..950.0).contains(&moved_steps), "moved {moved_steps} of 1000");
  }

  #[test]
  fn the_same_seed_repeats_the_same_noise() {
    let scenario = Scenario {
      beam: Beam::PLACEHOLDER.with_position_noise(0.01),
      seed: 42,
      ..Scenario::IDEAL
    };
    let first = World::new(scenario).measure();
    let second = World::new(scenario).measure();
    assert_eq!(first, second);
    let other = World::new(Scenario { seed: 43, ..scenario }).measure();
    assert_ne!(first, other);
  }

  #[test]
  fn position_noise_has_the_sigma_that_was_asked_for() {
    let scenario = Scenario {
      beam: Beam::PLACEHOLDER.with_position_noise(0.01),
      seed: 1,
      ..Scenario::IDEAL
    };
    let mut world = World::new(scenario);
    let samples: Vec<f64> = (0..2_000).map(|_| f64::from(world.measure().a.h_norm)).collect();
    #[expect(clippy::cast_precision_loss, reason = "a count of 2000")]
    let count = samples.len() as f64;
    let mean = samples.iter().sum::<f64>() / count;
    let sigma = (samples.iter().map(|sample| (sample - mean).powi(2)).sum::<f64>() / count).sqrt();
    assert_close(mean, 0.0, 0.001);
    assert_close(sigma, 0.01, 0.001);
  }

  #[test]
  fn the_log_and_the_commanded_range_record_every_move() {
    let mut world = World::new(Scenario::IDEAL);
    world.move_rel(Axis::M2V, -170);
    world.move_rel(Axis::M2V, 120);
    let _ = world.measure();
    assert_eq!(world.commanded_range(Axis::M2V), (-170, 0));
    assert_eq!(world.commanded_steps(Axis::M2V), -50);
    assert_eq!(world.log().len(), 3);
    assert_eq!(
      world.log()[0],
      Record::Move {
        axis: Axis::M2V,
        delta_steps: -170
      }
    );
  }
}
