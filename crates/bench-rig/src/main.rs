//! `bench-rig`: the backlash or the hysteresis of one axis on a bench, a node's motor turning a Series E screw and the
//! operator reading the pointer's dot off a paper target (milestone M6 before the test frame exists).
//!
//! Thin on purpose: the console is `bench_rig::console`, the motors `bench_rig::motors`, the readings `bench_rig::eye`,
//! and the run `bench_rig::procedure`. It opens only the ports named, sends `status` until a node of this firmware
//! answers, and moves nothing until the operator types `go`.

use std::cell::RefCell;
use std::fs::{self, File};
use std::io::{self, BufRead, Write};
use std::path::PathBuf;
use std::rc::Rc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use align_core::axis::Axis;
use anyhow::{Context, bail};
use bench_rig::console::{Log, NodeConsole, NodeStatus};
use bench_rig::eye::ReadByEye;
use bench_rig::motors::{BenchMotors, Motor};
use bench_rig::procedure::{self, Procedure, Settings};
use clap::{Parser, Subcommand, ValueEnum};
use node_logic::NodeState;
use serial2::SerialPort;

/// How long a read of a port waits before the console looks at its deadline again.
const READ_WAIT: Duration = Duration::from_millis(50);

#[derive(Parser)]
#[command(
  about = "Measure one axis's backlash or hysteresis on a bench: the node's motor turns the screw through its USB \
           console, and you type where the pointer's dot sits on a paper target. Nothing moves until you type `go`."
)]
struct Cli {
  #[command(subcommand)]
  procedure: Which,
  /// A motor: `<axis>=<port>[:<channel>]`, as `m1h=/dev/cu.usbmodem1101:0`. Channel 0 when left out. Name each
  /// motor of the bench; only the measured axis moves.
  #[arg(long = "motor", required = true, value_parser = parse_motor)]
  motors: Vec<MotorArg>,
  /// One sigma of your reading of the dot's center, in millimeters: how finely you read it. Readings of a dot that
  /// has not moved repeat exactly, so this is the noise every shift is judged against.
  #[arg(long)]
  reading_mm: f32,
  /// The target's half-width in millimeters: how far from its center you can still read the dot.
  #[arg(long, default_value_t = 100.0)]
  target_mm: f32,
  /// The poke in full steps [default: the configured 45, sized for a detector at 1 m]. It must move the dot well
  /// beyond your reading: at 14 um a step for each meter of throw, 200 steps move it 8.4 mm at 3 m.
  #[arg(long)]
  poke_steps: Option<i32>,
  /// The log of every console line, reading, and `BU1` line [default: target/bench-logs/<time>.log].
  #[arg(long)]
  log: Option<PathBuf>
}

#[derive(Subcommand)]
enum Which {
  /// The play of one axis, backlash and seat stick-slip together, as a bracket in steps (reference 3.4).
  Backlash {
    /// The axis measured.
    axis: AxisArg
  },
  /// The hysteresis loop of one axis: two poke sizes either side of its count and back (reference 3.4).
  Hysteresis {
    /// The axis measured.
    axis: AxisArg
  }
}

/// An axis named on the command line.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum AxisArg {
  M1h,
  M1v,
  M2h,
  M2v
}

impl From<AxisArg> for Axis {
  fn from(axis: AxisArg) -> Self {
    match axis {
      AxisArg::M1h => Axis::M1H,
      AxisArg::M1v => Axis::M1V,
      AxisArg::M2h => Axis::M2H,
      AxisArg::M2v => Axis::M2V
    }
  }
}

/// A motor named on the command line.
#[derive(Clone, Debug, PartialEq, Eq)]
struct MotorArg {
  axis: AxisArg,
  port: String,
  channel: u8
}

fn parse_motor(text: &str) -> Result<MotorArg, String> {
  let (axis, rest) = text
    .split_once('=')
    .ok_or_else(|| "a motor is <axis>=<port>[:<channel>]".to_owned())?;
  let axis = AxisArg::from_str(axis, true).map_err(|_| format!("`{axis}` is not m1h, m1v, m2h, or m2v"))?;
  let (port, channel) = match rest.rsplit_once(':') {
    Some((port, channel)) => (
      port,
      channel
        .parse()
        .map_err(|_| format!("`{channel}` is not a channel number"))?
    ),
    None => (rest, 0)
  };
  if port.is_empty() {
    return Err("the port is empty".to_owned());
  }
  Ok(MotorArg {
    axis,
    port: port.to_owned(),
    channel
  })
}

/// Writes to the terminal and the log at once.
struct Tee {
  log: Log
}

impl Write for Tee {
  fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
    io::stdout().write_all(bytes)?;
    self.log.borrow_mut().write_all(bytes)?;
    Ok(bytes.len())
  }

  fn flush(&mut self) -> io::Result<()> {
    io::stdout().flush()?;
    self.log.borrow_mut().flush()
  }
}

fn main() -> anyhow::Result<()> {
  let cli = Cli::parse();
  let (settings, measured) = settings(&cli)?;
  let log_path = cli.log.clone().unwrap_or_else(default_log);
  let log = open_log(&log_path)?;
  let mut terminal = io::stdout();
  writeln!(terminal, "log: {}", log_path.display())?;
  let mut bench = open_bench(&cli.motors, &settings, &log)?;
  let mut input = io::stdin().lock();
  let Some(start) = prepare(&mut bench, &settings, &measured, &mut input)? else {
    writeln!(terminal, "nothing moved")?;
    return Ok(());
  };

  let mut eye = ReadByEye::new(
    &mut input,
    io::stdout(),
    log.clone(),
    settings.target_mm,
    &procedure::config(&settings).sensor
  );
  let mut out = Tee { log: log.clone() };
  let result = procedure::run(&mut bench, &mut eye, &settings, &start.status, &mut out);
  let readings = eye.taken();
  if start.entered_bench
    && let Some((console, _)) = bench.console_of(settings.axis)
  {
    // Leaving bench mode is tidiness, not safety: the node is inhibited either way until a coordinator allows it.
    let _ = console.bench(false);
  }
  match result {
    Ok(report) => {
      writeln!(out, "{report}")?;
      writeln!(terminal, "{readings} readings; the log is {}", log_path.display())?;
      Ok(())
    }
    Err(fault) => {
      let problem = bench
        .last_problem()
        .map(|problem| format!(" ({problem})"))
        .unwrap_or_default();
      let _ = writeln!(log.borrow_mut(), "stopped: {fault}{problem}");
      bail!("the procedure stopped after {readings} readings: {fault}{problem}")
    }
  }
}

/// The run the command line asks for, and the measured axis's motor.
fn settings(cli: &Cli) -> anyhow::Result<(Settings, MotorArg)> {
  let (procedure, axis) = match cli.procedure {
    Which::Backlash { axis } => (Procedure::Backlash, axis),
    Which::Hysteresis { axis } => (Procedure::Hysteresis, axis)
  };
  if !(cli.reading_mm.is_finite() && cli.reading_mm > 0.0) {
    bail!("--reading-mm must be a positive number of millimeters");
  }
  if !(cli.target_mm.is_finite() && cli.target_mm > 0.0) {
    bail!("--target-mm must be a positive number of millimeters");
  }
  let Some(measured) = cli.motors.iter().find(|motor| motor.axis == axis).cloned() else {
    bail!("no --motor names {:?}, the axis to measure", Axis::from(axis));
  };
  let settings = Settings {
    axis: axis.into(),
    procedure,
    target_mm: cli.target_mm,
    reading_mm: cli.reading_mm,
    poke_steps: cli.poke_steps
  };
  Ok((settings, measured))
}

fn open_log(path: &PathBuf) -> anyhow::Result<Log> {
  if let Some(parent) = path.parent() {
    fs::create_dir_all(parent).with_context(|| format!("creating {}", parent.display()))?;
  }
  let file = File::create(path).with_context(|| format!("creating {}", path.display()))?;
  Ok(Rc::new(RefCell::new(file)))
}

/// One console a port, in the order the ports were first named, and each motor on its console.
fn open_bench(named: &[MotorArg], settings: &Settings, log: &Log) -> anyhow::Result<BenchMotors<SerialPort>> {
  let mut ports: Vec<String> = Vec::new();
  let mut motors = Vec::new();
  for motor in named {
    let console = if let Some(index) = ports.iter().position(|port| *port == motor.port) {
      index
    } else {
      ports.push(motor.port.clone());
      ports.len() - 1
    };
    motors.push(Motor {
      axis: motor.axis.into(),
      console,
      channel: motor.channel
    });
  }
  let mut consoles = Vec::new();
  for port in &ports {
    let mut serial = SerialPort::open(port, 115_200).with_context(|| format!("opening {port}"))?;
    serial
      .set_read_timeout(READ_WAIT)
      .with_context(|| format!("setting up {port}"))?;
    consoles.push(NodeConsole::new(serial, port.clone(), log.clone()));
  }
  Ok(BenchMotors::new(consoles, motors, &procedure::config(settings).motion))
}

/// How the procedure starts: the node's status before it, and whether bench mode was entered for it.
struct Start {
  status: NodeStatus,
  entered_bench: bool
}

/// Reads the measured motor's status, checks it, asks the operator to type `go`, and enters bench mode. `None` when
/// the operator types anything else: nothing has moved.
fn prepare(
  bench: &mut BenchMotors<SerialPort>,
  settings: &Settings,
  measured: &MotorArg,
  input: &mut impl BufRead
) -> anyhow::Result<Option<Start>> {
  let mut terminal = io::stdout();
  let Some((console, channel)) = bench.console_of(settings.axis) else {
    bail!("no console for {:?}", settings.axis);
  };
  let status = console.status(channel).with_context(|| {
    format!(
      "asking {} for its status: is a node with this firmware on it? (`just flash node` writes it)",
      measured.port
    )
  })?;
  writeln!(
    terminal,
    "{}: state {:?}, permit {}, rail {}; channel {channel} at {} steps{}",
    measured.port,
    status.state,
    status.permit,
    status.rail,
    status.pos_steps,
    if status.suspect { " (suspect)" } else { "" }
  )?;
  procedure::check_start(&status)?;
  write!(
    terminal,
    "{:?} of {:?}: channel {channel} on {} moves a few hundred steps either side of {} and ends where it started. \
     After each move you are asked where the dot is: 20 to 40 readings. Ctrl-C ends the program; a move already \
     sent runs to its end. Type `go` to start: ",
    settings.procedure, settings.axis, measured.port, status.pos_steps
  )?;
  terminal.flush()?;
  let mut answer = String::new();
  input.read_line(&mut answer)?;
  if answer.trim() != "go" {
    return Ok(None);
  }
  let entered_bench = status.state != NodeState::Bench;
  if entered_bench {
    console.bench(true)?;
    let now = console.status(channel)?;
    if now.state != NodeState::Bench {
      bail!(
        "{} did not enter bench mode (state {:?}): is a coordinator on its bus? Bench moves need it silent",
        measured.port,
        now.state
      );
    }
  }
  Ok(Some(Start { status, entered_bench }))
}

fn default_log() -> PathBuf {
  let seconds = SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .map(|since| since.as_secs())
    .unwrap_or_default();
  PathBuf::from("target/bench-logs").join(format!("bench-{seconds}.log"))
}

#[cfg(test)]
mod tests {
  use clap::CommandFactory;

  use super::*;

  #[test]
  fn the_command_line_is_consistent() {
    Cli::command().debug_assert();
  }

  #[test]
  fn a_motor_is_an_axis_a_port_and_a_channel_that_defaults_to_zero() {
    assert_eq!(
      parse_motor("m1h=/dev/cu.usbmodem1101:1"),
      Ok(MotorArg {
        axis: AxisArg::M1h,
        port: "/dev/cu.usbmodem1101".to_owned(),
        channel: 1
      })
    );
    assert_eq!(
      parse_motor("M2V=COM3").map(|motor| (motor.axis, motor.channel)),
      Ok((AxisArg::M2v, 0))
    );
    for wrong in ["m1h", "m3h=/dev/x", "m1h=/dev/x:one", "m1h=", "m1h=:0"] {
      assert!(parse_motor(wrong).is_err(), "{wrong}");
    }
  }

  #[test]
  fn a_procedure_needs_a_motor_and_a_reading_uncertainty() {
    assert!(Cli::try_parse_from(["bench-rig", "backlash", "m1h", "--reading-mm", "0.25"]).is_err());
    assert!(Cli::try_parse_from(["bench-rig", "backlash", "m1h", "--motor", "m1h=/dev/x"]).is_err());
    let cli = Cli::try_parse_from([
      "bench-rig",
      "--motor",
      "m1h=/dev/x",
      "--reading-mm",
      "0.25",
      "--poke-steps",
      "200",
      "hysteresis",
      "m1h"
    ])
    .unwrap();
    assert!(matches!(cli.procedure, Which::Hysteresis { axis: AxisArg::M1h }));
    assert_eq!(cli.poke_steps, Some(200));
  }
}
