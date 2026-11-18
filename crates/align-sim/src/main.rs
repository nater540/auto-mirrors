//! Command-line runner for the simulator: one alignment with its log, a sweep over conditioning, or the bring-up
//! measurements of one axis with the log lines the coordinator prints, against the model alone or, with `--system`,
//! through the whole system (`align_sim::system`).
//!
//! Thin on purpose: the model is in `align_sim::world` and the procedure in `align_core::aligner`.

use std::collections::BTreeMap;
use std::io::{self, Write};

use align_core::aligner::{AlignFault, AlignReport, Aligner};
use align_core::axis::{Axis, PerAxis};
use align_core::bringup::{Entry, SAMPLE_COLUMNS};
use align_core::config::Config;
use align_core::rig::{Actuator, BeamSensor};
use align_core::solver::Conditioning;
use align_core::units;
use align_sim::block_on;
use align_sim::external::External;
use align_sim::rig::Rig;
use align_sim::system::{Hardware, System, SystemRig};
use align_sim::world::{AxisModel, Beam, Geometry, Record, Scenario, World};
use anyhow::Context;
use clap::{Args, Parser, Subcommand};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

#[derive(Parser)]
#[command(about = "Simulated mirror alignment: the align-core procedure against a model of the machine")]
struct Cli {
  #[command(subcommand)]
  command: Command
}

#[derive(Subcommand)]
enum Command {
  /// Run one alignment and print every move and measurement
  Run(RunArgs),
  /// Run many alignments at each Y span and print how conditioning affects convergence
  Sweep(SweepArgs),
  /// Measure one axis's backlash, then its hysteresis, and print the `BU1` lines the coordinator's `backlash` and
  /// `hysteresis` print, on a machine whose play is known
  BringUp(BringUpArgs),
  /// Run one alignment against a machine in another program, one line per move or measurement on standard output,
  /// answered on standard input (`align_sim::external`): `tools/beamviz` answers with a ray-traced machine
  External(ExternalArgs)
}

/// The imperfections of the simulated machine. Every default is an UNVERIFIED placeholder.
#[derive(Args, Clone, Copy)]
struct Imperfections {
  /// Play between motor and screw, in steps, on every axis
  #[arg(long, default_value_t = 20)]
  backlash: u32,
  /// Screw travel the seat resists before it slips, in steps
  #[arg(long, default_value_t = 3)]
  stick: u32,
  /// Chance that a commanded step is skipped
  #[arg(long, default_value_t = 0.0)]
  skip: f64,
  /// Sensor noise on one normalized coordinate, one sigma
  #[arg(long, default_value_t = 0.002)]
  noise: f64,
  /// Fraction of each screw's effect that leaks into the other direction
  #[arg(long, default_value_t = 0.05)]
  coupling: f64
}

/// The test frame's optics. Without these options the placeholders apply, unchanged.
#[derive(Args, Clone, Copy)]
struct Optics {
  /// Span of head travel in X, in millimeters
  #[arg(long, default_value_t = 600.0)]
  x_span: f64,
  /// Gaussian 1/e^2 beam radius at the head, in micrometers. When given, the poke size is computed from it
  /// (reference 3.1, clamped to three times the backlash as addendum A.3 says), and a sweep scales its random
  /// misalignments and offsets by the radius over the 4 mm placeholder, so that they stay inside the linear range.
  #[arg(long)]
  beam_radius: Option<f64>,
  /// Far-field half-angle divergence of the beam, in microradians: B and C see a larger beam than A, and a head
  /// offset no longer cancels in the difference vector
  #[arg(long, default_value_t = 0.0)]
  divergence_urad: f64
}

#[derive(Args)]
struct RunArgs {
  /// Seed for the noise and the skipped steps
  #[arg(long, default_value_t = 1)]
  seed: u64,
  /// Starting misalignment of the four axes, in steps: mirror 1 horizontal and vertical, then mirror 2
  #[arg(long, num_args = 4, allow_negative_numbers = true, default_values_t = [25.0, -20.0, -35.0, 30.0])]
  misalign: Vec<f64>,
  /// Constant beam offset at the head, horizontal then vertical, in micrometers
  #[arg(long, num_args = 2, allow_negative_numbers = true, default_values_t = [100.0, -50.0])]
  offset: Vec<f64>,
  /// Span of gantry travel in Y, in millimeters
  #[arg(long, default_value_t = 500.0)]
  y_span: f64,
  #[command(flatten)]
  optics: Optics,
  #[command(flatten)]
  imperfections: Imperfections
}

#[derive(Args)]
struct SweepArgs {
  /// Alignments at each Y span
  #[arg(long, default_value_t = 200)]
  runs: u32,
  /// Y spans to try, in millimeters; the X span is `--x-span`
  #[arg(long, num_args = 1.., default_values_t = [500.0, 200.0, 100.0, 60.0, 40.0, 25.0])]
  y_spans: Vec<f64>,
  /// Seed for the misalignments
  #[arg(long, default_value_t = 1)]
  seed: u64,
  /// Start from a stored Jacobian whose every column is wrong by this fraction,
  /// up or down at random, instead of calibrating first
  #[arg(long)]
  jacobian_error: Option<f32>,
  /// Run every alignment through the whole system: the coordinator's orchestrator, the nodes, and the sensor node on
  /// a simulated bus, with the interlock and the Ruida simulated. Adds the mean simulated time of an alignment.
  #[arg(long, conflicts_with = "jacobian_error")]
  system: bool,
  #[command(flatten)]
  optics: Optics,
  #[command(flatten)]
  imperfections: Imperfections
}

#[derive(Args)]
struct ExternalArgs {
  /// The sum of the four quadrants for a centered beam, in the other program's units
  #[arg(long, default_value_t = 1_000.0)]
  sum_ref: f64,
  /// Gaussian 1/e^2 beam radius at the head, in micrometers, from which the poke size is computed (as for `run`)
  #[arg(long)]
  beam_radius: Option<f64>,
  /// The backlash the poke is kept to at least three times of, in steps (addendum A.3)
  #[arg(long, default_value_t = 20)]
  backlash: u32
}

#[derive(Args)]
struct BringUpArgs {
  /// The axis: m1h, m1v, m2h, or m2v
  #[arg(long, default_value = "m1h", value_parser = parse_axis)]
  axis: Axis,
  /// Seed for the noise
  #[arg(long, default_value_t = 1)]
  seed: u64,
  /// Span of gantry travel in Y, in millimeters
  #[arg(long, default_value_t = 500.0)]
  y_span: f64,
  /// Measure through the whole system: the coordinator's orchestrator, the nodes, and the sensor node on a simulated
  /// bus. Adds the simulated time each measurement took.
  #[arg(long)]
  system: bool,
  #[command(flatten)]
  optics: Optics,
  #[command(flatten)]
  imperfections: Imperfections
}

fn main() -> anyhow::Result<()> {
  let cli = Cli::parse();
  let mut out = io::stdout().lock();
  match cli.command {
    Command::Run(args) => run(&args, &mut out),
    Command::Sweep(args) => sweep(&args, &mut out),
    Command::BringUp(args) => bring_up(&args, &mut out),
    Command::External(args) => external(&args, &mut out)
  }
}

/// An axis by its name.
fn parse_axis(word: &str) -> Result<Axis, String> {
  Axis::ALL
    .into_iter()
    .find(|axis| axis.name().eq_ignore_ascii_case(word))
    .ok_or_else(|| format!("`{word}` is not an axis: m1h, m1v, m2h, or m2v"))
}

fn scenario(imperfections: Imperfections, optics: Optics, y_span_mm: f64, seed: u64) -> Scenario {
  let placeholder = Geometry::PLACEHOLDER;
  let beam = Beam {
    radius_um: optics.beam_radius.unwrap_or(Beam::PLACEHOLDER.radius_um),
    divergence_urad: optics.divergence_urad,
    ..Beam::PLACEHOLDER
  };
  Scenario {
    geometry: Geometry {
      ly_far_mm: placeholder.ly_near_mm + y_span_mm,
      lx_far_mm: placeholder.lx_near_mm + optics.x_span,
      ..placeholder
    },
    beam: beam.with_position_noise(imperfections.noise),
    axes: PerAxis::splat(AxisModel {
      backlash_steps: imperfections.backlash,
      stick_steps: imperfections.stick,
      skip_probability: imperfections.skip
    }),
    cross_coupling: imperfections.coupling,
    seed,
    ..Scenario::IDEAL
  }
}

fn run(args: &RunArgs, out: &mut impl Write) -> anyhow::Result<()> {
  let [m1_h, m1_v, m2_h, m2_v] = args.misalign.as_slice() else {
    anyhow::bail!("--misalign takes four values");
  };
  let [offset_h, offset_v] = args.offset.as_slice() else {
    anyhow::bail!("--offset takes two values");
  };
  let mut scenario = scenario(args.imperfections, args.optics, args.y_span, args.seed);
  scenario.misalignment_steps = PerAxis {
    m1_h: *m1_h,
    m1_v: *m1_v,
    m2_h: *m2_h,
    m2_v: *m2_v
  };
  scenario.head_offset_um = [*offset_h, *offset_v];

  let rig = Rig::new(scenario);
  let model_cond = Conditioning::of(&rig.world().true_jacobian())
    .context("the model Jacobian is singular")?
    .cond;
  writeln!(out, "model Jacobian condition number: {model_cond:.2}")?;
  let (config, formula) = config_for(args.optics, args.imperfections.backlash);
  if let Some(formula) = formula {
    writeln!(out, "poke: {} steps (formula {formula:.1})", config.motion.poke_steps)?;
  }
  writeln!(
    out,
    "start: true residual {:.4}",
    rig.world().true_shots().measurement().amax()
  )?;

  let mut aligner = rig.aligner(config)?;
  let outcome = block_on(aligner.align());

  for record in rig.world().log() {
    match record {
      Record::Move { axis, delta_steps } => writeln!(out, "  move    {axis:?} {delta_steps:+}")?,
      Record::Measure { true_residual_norm } => writeln!(out, "  measure true residual {true_residual_norm:.4}")?
    }
  }
  match outcome {
    Ok(report) => writeln!(
      out,
      "converged: {} steps, {} recalibrations, residual {:.4}, cond {:.2}, head offset ({:+.3}, {:+.3}){}",
      report.iterations,
      report.recalibrations,
      report.residual_norm,
      report.cond,
      report.head_offset.h_norm,
      report.head_offset.v_norm,
      if report.needs_mechanical_centering {
        ", mechanical centering required"
      } else {
        ""
      }
    )?,
    Err(fault) => writeln!(out, "fault: {fault}")?
  }
  let world = rig.world();
  writeln!(out, "end: true residual {:.4}", world.true_shots().measurement().amax())?;
  writeln!(
    out,
    "end: largest true beam angle {:.1} urad",
    world.angle_errors().largest_urad()
  )?;
  for axis in Axis::ALL {
    let (lowest, highest) = world.commanded_range(axis);
    writeln!(
      out,
      "end: {axis:?} at {:+} steps (ranged {lowest:+} to {highest:+}){}",
      world.commanded_steps(axis),
      if aligner.axes().get(axis).suspect {
        ", suspect"
      } else {
        ""
      }
    )?;
  }
  Ok(())
}

fn external(args: &ExternalArgs, out: &mut impl Write) -> anyhow::Result<()> {
  let optics = Optics {
    x_span: 0.0,
    beam_radius: args.beam_radius,
    divergence_urad: 0.0
  };
  let (config, _) = config_for(optics, args.backlash);
  let machine = External::new(io::stdin().lock(), out);
  #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
  let sum_ref = args.sum_ref as f32;
  let mut aligner = Aligner::new(machine.clone(), machine.clone(), config, sum_ref)?;
  let outcome = block_on(aligner.align());
  machine.finish(&outcome, aligner.calibration())?;
  Ok(())
}

fn bring_up(args: &BringUpArgs, out: &mut impl Write) -> anyhow::Result<()> {
  let scenario = scenario(args.imperfections, args.optics, args.y_span, args.seed);
  let (config, _) = config_for(args.optics, args.imperfections.backlash);
  let imperfections = args.imperfections;
  writeln!(
    out,
    "model: backlash {} steps and a seat that sticks for {} on every axis, noise {} per coordinate; the lines below \
     should find them",
    imperfections.backlash, imperfections.stick, imperfections.noise
  )?;
  if args.system {
    let rig = SystemRig::new(System::new(scenario, config, Hardware::PLACEHOLDER));
    let clock = rig.clone();
    measure_bring_up(&mut rig.aligner(config)?, args.axis, || clock.system().now_ms(), out)
  } else {
    let rig = Rig::new(scenario);
    measure_bring_up(&mut rig.aligner(config)?, args.axis, || 0, out)
  }
}

/// Both bring-up measurements of one axis, with their log lines and, from a clock that runs, the time each took.
fn measure_bring_up<A: Actuator, S: BeamSensor>(
  aligner: &mut Aligner<A, S>,
  axis: Axis,
  now_ms: impl Fn() -> u64,
  out: &mut impl Write
) -> anyhow::Result<()> {
  let mut entries = Vec::new();
  let started_ms = now_ms();
  writeln!(out, "{SAMPLE_COLUMNS}")?;
  let backlash = block_on(aligner.measure_backlash(axis, &mut |entry| entries.push(*entry)));
  print_bring_up(&mut entries, backlash, out)?;
  let backlash_ms = now_ms().saturating_sub(started_ms);
  writeln!(out, "{SAMPLE_COLUMNS}")?;
  let hysteresis = block_on(aligner.measure_hysteresis(axis, &mut |entry| entries.push(*entry)));
  print_bring_up(&mut entries, hysteresis, out)?;
  let hysteresis_ms = now_ms().saturating_sub(started_ms).saturating_sub(backlash_ms);
  if backlash_ms > 0 {
    #[expect(clippy::cast_precision_loss, reason = "simulated times of minutes")]
    let seconds = |ms: u64| ms as f64 / 1_000.0;
    writeln!(
      out,
      "simulated time: backlash {:.1} s, hysteresis {:.1} s",
      seconds(backlash_ms),
      seconds(hysteresis_ms)
    )?;
  }
  Ok(())
}

/// The entries a procedure logged, then its report or fault.
fn print_bring_up<R: core::fmt::Display>(
  entries: &mut Vec<Entry>,
  outcome: Result<R, AlignFault>,
  out: &mut impl Write
) -> anyhow::Result<()> {
  for entry in entries.drain(..) {
    writeln!(out, "{entry}")?;
  }
  match outcome {
    Ok(report) => writeln!(out, "{report}")?,
    Err(fault) => writeln!(out, "fault: {fault}")?
  }
  Ok(())
}

fn sweep(args: &SweepArgs, out: &mut impl Write) -> anyhow::Result<()> {
  let (config, formula) = config_for(args.optics, args.imperfections.backlash);
  let scale = misalignment_scale(args.optics);
  sweep_header(args, &config, formula, scale, out)?;
  let mut rejected_frames = 0_u64;
  for &y_span_mm in &args.y_spans {
    let mut misalignments = StdRng::seed_from_u64(args.seed);
    let mut converged = 0_u32;
    let mut first_attempt = 0_u32;
    let mut recalibrated = 0_u32;
    let mut steps = 0_u32;
    let mut model_cond = f32::NAN;
    let mut faults = BTreeMap::new();
    let mut simulated_ms = 0_u64;
    for run in 0..args.runs {
      let mut scenario = scenario(
        args.imperfections,
        args.optics,
        y_span_mm,
        args.seed.wrapping_add(u64::from(run))
      );
      // The same range the closed-loop tests draw from: inside the sensor's linear range at every point.
      scenario.misalignment_steps = PerAxis {
        m1_h: misalignments.random_range(-35.0..35.0) * scale,
        m1_v: misalignments.random_range(-35.0..35.0) * scale,
        m2_h: misalignments.random_range(-50.0..50.0) * scale,
        m2_v: misalignments.random_range(-50.0..50.0) * scale
      };
      scenario.head_offset_um = [
        misalignments.random_range(-200.0..200.0) * scale,
        misalignments.random_range(-200.0..200.0) * scale
      ];
      if let Ok(conditioning) = Conditioning::of(&World::new(scenario).true_jacobian()) {
        model_cond = conditioning.cond;
      }
      let (outcome, run_ms, rejected) = align_once(args, &config, scenario, &mut misalignments)?;
      simulated_ms += run_ms;
      rejected_frames += rejected;
      match outcome {
        Ok(report) => {
          converged += 1;
          steps += u32::from(report.iterations);
          if report.recalibrations == 0 {
            first_attempt += 1;
          } else {
            recalibrated += 1;
          }
        }
        Err(fault) => *faults.entry(fault_kind(&fault)).or_insert(0_u32) += 1
      }
    }
    let mean_steps = if converged == 0 {
      f64::NAN
    } else {
      f64::from(steps) / f64::from(converged)
    };
    write!(
      out,
      "{y_span_mm:>9.0}  {model_cond:>10.1}  {converged:>9}  {first_attempt:>13}  {mean_steps:>10.2}  \
       {recalibrated:>12}  {:>6}",
      args.runs - converged
    )?;
    if args.system {
      #[expect(
        clippy::cast_precision_loss,
        reason = "a sweep's simulated time is far below 2^52 ms"
      )]
      let mean_time_s = simulated_ms as f64 / 1_000.0 / f64::from(args.runs.max(1));
      write!(out, "  {mean_time_s:>11.1}")?;
    }
    writeln!(out)?;
    for (kind, count) in &faults {
      writeln!(out, "{:>9}  {count} x {kind}", "")?;
    }
  }
  if args.system {
    writeln!(out, "frames the protocol and a device disagreed on: {rejected_frames}")?;
  }
  Ok(())
}

/// The lines above a sweep's table, and its column names.
fn sweep_header(
  args: &SweepArgs,
  config: &Config,
  formula: Option<f32>,
  scale: f64,
  out: &mut impl Write
) -> anyhow::Result<()> {
  writeln!(
    out,
    "{} runs per row, default config, random misalignment and head offset",
    args.runs
  )?;
  if let Some(formula) = formula {
    writeln!(
      out,
      "beam radius {:.0} um, X span {:.0} mm: poke {} steps (formula {formula:.1}), misalignment scaled by {scale:.2}",
      args.optics.beam_radius.unwrap_or_default(),
      args.optics.x_span,
      config.motion.poke_steps
    )?;
  }
  if args.system {
    writeln!(
      out,
      "through the whole system: orchestrator, nodes, and sensor on a simulated bus, placeholder hardware timing"
    )?;
  }
  writeln!(
    out,
    "y_span_mm  model_cond  converged  first_attempt  mean_steps  recalibrated  faults{}",
    if args.system { "  mean_time_s" } else { "" }
  )?;
  Ok(())
}

/// One alignment of a sweep: against the model, or through the whole system with `--system`. Returns its outcome,
/// the simulated time it took (zero against the model), and the frames the bus rejected.
fn align_once(
  args: &SweepArgs,
  config: &Config,
  scenario: Scenario,
  draws: &mut StdRng
) -> anyhow::Result<(Result<AlignReport, AlignFault>, u64, u64)> {
  if args.system {
    let rig = SystemRig::new(System::new(scenario, *config, Hardware::PLACEHOLDER));
    let started_ms = rig.system().now_ms();
    let outcome = block_on(rig.aligner(*config)?.align());
    let system = rig.system();
    return Ok((
      outcome,
      system.now_ms().saturating_sub(started_ms),
      system.stats().rejected
    ));
  }
  let rig = Rig::new(scenario);
  let mut aligner = rig.aligner(*config)?;
  if let Some(error) = args.jacobian_error {
    let scale = core::array::from_fn(|_| {
      if draws.random_bool(0.5) {
        1.0 + error
      } else {
        1.0 - error
      }
    });
    aligner = aligner.with_calibration(rig.stored_calibration(scale));
  }
  Ok((block_on(aligner.align()), 0, 0))
}

/// The default configuration, with the poke size computed for the beam when its radius is given: the formula of
/// reference 3.1 for the configured target signal and one step's beam movement at 1 m, clamped to
/// `[3 x backlash, poke_max_steps]` as addendum A.3 says. Returns the configuration and the unclamped poke.
fn config_for(optics: Optics, backlash_steps: u32) -> (Config, Option<f32>) {
  let mut config = Config::default();
  let Some(radius_um) = optics.beam_radius else {
    return (config, None);
  };
  #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
  let radius_um = radius_um as f32;
  let per_step_um = units::beam_displacement_um(&config.mechanics, Axis::M1H, 1.0, 1_000.0);
  let formula = units::poke_steps(config.motion.poke_target_norm, radius_um, per_step_um);
  #[expect(clippy::cast_precision_loss, reason = "backlash is a few hundred steps at most")]
  let clamped = units::clamp_poke_steps(formula, backlash_steps as f32, config.motion.poke_max_steps as f32);
  #[expect(clippy::cast_possible_truncation, reason = "a poke is a few thousand steps at most")]
  let poke = clamped.round() as i32;
  config.motion.poke_steps = poke;
  (config, Some(formula))
}

/// How much to scale a sweep's random misalignments and offsets: the beam radius over the 4 mm placeholder.
fn misalignment_scale(optics: Optics) -> f64 {
  optics
    .beam_radius
    .map_or(1.0, |radius_um| radius_um / Beam::PLACEHOLDER.radius_um)
}

/// A short name for a fault, for the tally of a sweep.
fn fault_kind(fault: &AlignFault) -> &'static str {
  match fault {
    AlignFault::Config(_) => "configuration",
    AlignFault::Move { .. } => "move failed",
    AlignFault::SoftLimit { .. } => "soft limit",
    AlignFault::MoveTooLong { .. } => "move too long",
    AlignFault::Sensor(_) => "no reading",
    AlignFault::Reading(_) => "reading rejected",
    AlignFault::WeakColumn { .. } => "weak calibration column",
    AlignFault::Calibration(_) => "calibration rejected (conditioning)",
    AlignFault::Solve(_) => "solver",
    AlignFault::PositionUnknown { .. } => "position unknown",
    AlignFault::AxisNotResponding { .. } => "axis not responding",
    AlignFault::NotConverged { .. } => "not converged"
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  const NO_BEAM: Optics = Optics {
    x_span: 600.0,
    beam_radius: None,
    divergence_urad: 0.0
  };

  #[test]
  fn without_a_beam_radius_the_configuration_and_the_misalignments_are_the_defaults() {
    let (config, formula) = config_for(NO_BEAM, 20);
    assert_eq!(config, Config::default());
    assert_eq!(formula, None);
    assert!((misalignment_scale(NO_BEAM) - 1.0).abs() < f64::EPSILON);
  }

  #[test]
  fn a_small_beam_s_poke_is_raised_to_three_times_the_backlash() {
    // 0.25 / ((1.596 / 1000) * 14.0) is about 11.2 steps, under 3 x 20.
    let optics = Optics {
      x_span: 800.0,
      beam_radius: Some(1_000.0),
      divergence_urad: 0.0
    };
    let (config, formula) = config_for(optics, 20);
    let formula = formula.unwrap();
    assert!((formula - 11.2).abs() < 0.1, "{formula}");
    assert_eq!(config.motion.poke_steps, 60);
    assert!((misalignment_scale(optics) - 0.25).abs() < 1e-12);
  }

  #[test]
  fn a_large_beam_s_poke_follows_the_formula() {
    // Twice the 4 mm placeholder radius: 0.25 / ((1.596 / 8000) * 14.0), about 89.4 steps, above the clamp.
    let optics = Optics {
      x_span: 800.0,
      beam_radius: Some(8_000.0),
      divergence_urad: 0.0
    };
    let (config, formula) = config_for(optics, 20);
    assert!((formula.unwrap() - 89.4).abs() < 0.1);
    assert_eq!(config.motion.poke_steps, 89);
  }

  #[test]
  fn the_scenario_uses_the_x_span_and_the_beam_radius() {
    let optics = Optics {
      x_span: 800.0,
      beam_radius: Some(1_500.0),
      divergence_urad: 0.0
    };
    let imperfections = Imperfections {
      backlash: 20,
      stick: 3,
      skip: 0.0,
      noise: 0.0,
      coupling: 0.0
    };
    let scenario = scenario(imperfections, optics, 600.0, 1);
    let geometry = scenario.geometry;
    assert!((geometry.lx_far_mm - geometry.lx_near_mm - 800.0).abs() < 1e-9);
    assert!((geometry.ly_far_mm - geometry.ly_near_mm - 600.0).abs() < 1e-9);
    assert!((scenario.beam.radius_um - 1_500.0).abs() < 1e-9);
  }
}
