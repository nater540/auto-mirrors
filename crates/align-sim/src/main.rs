//! Command-line runner for the simulator: one alignment with its log, or a sweep over conditioning.
//!
//! Thin on purpose: the model is in `align_sim::world` and the procedure in `align_core::aligner`.

use std::collections::BTreeMap;
use std::io::{self, Write};

use align_core::aligner::AlignFault;
use align_core::axis::{Axis, PerAxis};
use align_core::config::Config;
use align_core::solver::Conditioning;
use align_sim::block_on;
use align_sim::rig::Rig;
use align_sim::world::{AxisModel, Beam, Geometry, Record, Scenario};
use anyhow::Context;
use clap::{Args, Parser, Subcommand};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};

#[derive(Parser)]
#[command(about = "Simulated mirror alignment: the align-core procedure against a model of the machine")]
struct Cli {
  #[command(subcommand)]
  command: Command,
}

#[derive(Subcommand)]
enum Command {
  /// Run one alignment and print every move and measurement
  Run(RunArgs),
  /// Run many alignments at each Y span and print how conditioning affects convergence
  Sweep(SweepArgs),
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
  coupling: f64,
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
  imperfections: Imperfections,
}

#[derive(Args)]
struct SweepArgs {
  /// Alignments at each Y span
  #[arg(long, default_value_t = 200)]
  runs: u32,
  /// Y spans to try, in millimeters; the X span stays at 600
  #[arg(long, num_args = 1.., default_values_t = [500.0, 200.0, 100.0, 60.0, 40.0, 25.0])]
  y_spans: Vec<f64>,
  /// Seed for the misalignments
  #[arg(long, default_value_t = 1)]
  seed: u64,
  /// Start from a stored Jacobian whose every column is wrong by this fraction, up or down at random, instead of
  /// calibrating first
  #[arg(long)]
  jacobian_error: Option<f32>,
  #[command(flatten)]
  imperfections: Imperfections,
}

fn main() -> anyhow::Result<()> {
  let cli = Cli::parse();
  let mut out = io::stdout().lock();
  match cli.command {
    Command::Run(args) => run(&args, &mut out),
    Command::Sweep(args) => sweep(&args, &mut out),
  }
}

fn scenario(imperfections: Imperfections, y_span_mm: f64, seed: u64) -> Scenario {
  let placeholder = Geometry::PLACEHOLDER;
  Scenario {
    geometry: Geometry {
      ly_far_mm: placeholder.ly_near_mm + y_span_mm,
      ..placeholder
    },
    beam: Beam::PLACEHOLDER.with_position_noise(imperfections.noise),
    axes: PerAxis::splat(AxisModel {
      backlash_steps: imperfections.backlash,
      stick_steps: imperfections.stick,
      skip_probability: imperfections.skip,
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
  let mut scenario = scenario(args.imperfections, args.y_span, args.seed);
  scenario.misalignment_steps = PerAxis {
    m1_h: *m1_h,
    m1_v: *m1_v,
    m2_h: *m2_h,
    m2_v: *m2_v,
  };
  scenario.head_offset_um = [*offset_h, *offset_v];

  let rig = Rig::new(scenario);
  let model_cond = Conditioning::of(&rig.world().true_jacobian())
    .context("the model Jacobian is singular")?
    .cond;
  writeln!(out, "model Jacobian condition number: {model_cond:.2}")?;
  writeln!(
    out,
    "start: true residual {:.4}",
    rig.world().true_shots().measurement().amax()
  )?;

  let mut aligner = rig.aligner(Config::default());
  let outcome = block_on(aligner.align());

  for record in rig.world().log() {
    match record {
      Record::Move { axis, delta_steps } => writeln!(out, "  move    {axis:?} {delta_steps:+}")?,
      Record::Measure { true_residual_norm } => writeln!(out, "  measure true residual {true_residual_norm:.4}")?,
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
      },
    )?,
    Err(fault) => writeln!(out, "fault: {fault}")?,
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
      },
    )?;
  }
  Ok(())
}

fn sweep(args: &SweepArgs, out: &mut impl Write) -> anyhow::Result<()> {
  let config = Config::default();
  writeln!(
    out,
    "{} runs per row, default config, random misalignment and head offset",
    args.runs
  )?;
  writeln!(
    out,
    "y_span_mm  model_cond  converged  first_attempt  mean_steps  recalibrated  faults"
  )?;
  for &y_span_mm in &args.y_spans {
    let mut misalignments = StdRng::seed_from_u64(args.seed);
    let mut converged = 0_u32;
    let mut first_attempt = 0_u32;
    let mut recalibrated = 0_u32;
    let mut steps = 0_u32;
    let mut model_cond = f32::NAN;
    let mut faults = BTreeMap::new();
    for run in 0..args.runs {
      let mut scenario = scenario(args.imperfections, y_span_mm, args.seed.wrapping_add(u64::from(run)));
      // The same range the closed-loop tests draw from: inside the sensor's linear range at every point.
      scenario.misalignment_steps = PerAxis {
        m1_h: misalignments.random_range(-35.0..35.0),
        m1_v: misalignments.random_range(-35.0..35.0),
        m2_h: misalignments.random_range(-50.0..50.0),
        m2_v: misalignments.random_range(-50.0..50.0),
      };
      scenario.head_offset_um = [
        misalignments.random_range(-200.0..200.0),
        misalignments.random_range(-200.0..200.0),
      ];
      let rig = Rig::new(scenario);
      if let Ok(conditioning) = Conditioning::of(&rig.world().true_jacobian()) {
        model_cond = conditioning.cond;
      }
      let mut aligner = rig.aligner(config);
      if let Some(error) = args.jacobian_error {
        let scale = core::array::from_fn(|_| {
          if misalignments.random_bool(0.5) {
            1.0 + error
          } else {
            1.0 - error
          }
        });
        aligner = aligner.with_calibration(rig.stored_calibration(scale));
      }
      match block_on(aligner.align()) {
        Ok(report) => {
          converged += 1;
          steps += u32::from(report.iterations);
          if report.recalibrations == 0 {
            first_attempt += 1;
          } else {
            recalibrated += 1;
          }
        }
        Err(fault) => *faults.entry(fault_kind(&fault)).or_insert(0_u32) += 1,
      }
    }
    let mean_steps = if converged == 0 {
      f64::NAN
    } else {
      f64::from(steps) / f64::from(converged)
    };
    writeln!(
      out,
      "{y_span_mm:>9.0}  {model_cond:>10.1}  {converged:>9}  {first_attempt:>13}  {mean_steps:>10.2}  \
       {recalibrated:>12}  {:>6}",
      args.runs - converged,
    )?;
    for (kind, count) in &faults {
      writeln!(out, "{:>9}  {count} x {kind}", "")?;
    }
  }
  Ok(())
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
    AlignFault::AxisNotResponding { .. } => "axis not responding",
    AlignFault::NotConverged { .. } => "not converged",
  }
}
