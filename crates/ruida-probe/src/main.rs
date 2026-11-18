//! `ruida-probe`: the read-only first test of the Ruida link (reference 6.5) from this computer, or against a
//! simulated controller on the loopback interface with `--sim`.
//!
//! Thin on purpose: the protocol is `ruida_proto`, the run is `ruida_probe::probe`, the simulation `ruida_probe::sim`.

use std::io::{self, Write};
use std::net::{IpAddr, Ipv4Addr, SocketAddr, UdpSocket};

use align_core::config::Config;
use anyhow::Context;
use clap::{Parser, ValueEnum};
use ruida_probe::probe::{self, LOG_TAG, Settings};
use ruida_probe::sim::{self, SimController};
use ruida_proto::{CONTROLLER_PORT, ChecksumBasis, Limits, REPLY_PORT};

#[derive(Parser)]
#[command(
  about = "Read the Ruida controller's card ID, version, status, and position over UDP (reference 6.5), \
           logged as RU1 lines. Sends only the five memory reads; never a move, a job, or a key."
)]
struct Cli {
  /// The controller's IP address. Close LightBurn on this computer first: only one program can bind port 40200.
  #[arg(long, required_unless_present = "sim", conflicts_with = "sim")]
  controller: Option<IpAddr>,
  /// After the five reads, poll the status and the X and Y positions for this many seconds (reference 6.5 steps 4
  /// and 6).
  #[arg(long, default_value_t = 0)]
  poll_s: u64,
  /// Probe a simulated controller on this computer's loopback interface instead: nothing leaves the computer.
  #[arg(long)]
  sim: bool,
  /// The simulated controller's checksum basis [default: swizzled].
  #[arg(long, value_enum, conflicts_with = "controller")]
  sim_basis: Option<Basis>
}

/// A checksum basis named on the command line.
#[derive(Clone, Copy, ValueEnum)]
enum Basis {
  Swizzled,
  Plain
}

impl From<Basis> for ChecksumBasis {
  fn from(basis: Basis) -> Self {
    match basis {
      Basis::Swizzled => ChecksumBasis::Swizzled,
      Basis::Plain => ChecksumBasis::Plain
    }
  }
}

fn main() -> anyhow::Result<()> {
  let cli = Cli::parse();
  let link = Config::default().ruida;
  let settings = Settings {
    limits: Limits {
      timeout_ms: link.reply_timeout_ms,
      tries: link.query_tries
    },
    period_ms: link.poll_period_ms,
    poll_ms: cli.poll_s.saturating_mul(1_000)
  };
  let mut out = io::stdout().lock();
  if let Some(address) = cli.controller {
    probe_controller(address, &settings, &mut out)
  } else {
    probe_simulation(cli.sim_basis.unwrap_or(Basis::Swizzled).into(), &settings, &mut out)
  }
}

/// The probe against the controller at `address`, answering on port 40200 of this computer.
fn probe_controller(address: IpAddr, settings: &Settings, out: &mut impl Write) -> anyhow::Result<()> {
  let socket = UdpSocket::bind(SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), REPLY_PORT)).with_context(|| {
    format!(
      "binding UDP port {REPLY_PORT}, where the controller answers: close LightBurn or any other Ruida program on \
       this computer (reference 6.5 step 1)"
    )
  })?;
  probe::run(&socket, SocketAddr::new(address, CONTROLLER_PORT), settings, out)?;
  Ok(())
}

/// The probe against a simulated controller on the loopback interface, and what the controller received.
fn probe_simulation(basis: ChecksumBasis, settings: &Settings, out: &mut impl Write) -> anyhow::Result<()> {
  let controller = sim::serve(SimController::new(basis)).context("starting the simulation")?;
  let socket = UdpSocket::bind("127.0.0.1:0").context("binding a loopback port for the probe")?;
  probe::run(&socket, controller.address(), settings, out)?;
  let controller = controller.stop().context("stopping the simulation")?;
  writeln!(
    out,
    "{LOG_TAG},sim,basis={},commands_received={}",
    controller.basis.name(),
    controller.received().len()
  )?;
  Ok(())
}

#[cfg(test)]
mod tests {
  use super::*;

  fn parse(args: &[&str]) -> Result<Cli, clap::Error> {
    Cli::try_parse_from(std::iter::once("ruida-probe").chain(args.iter().copied()))
  }

  #[test]
  fn a_controller_or_the_simulation_must_be_named_and_not_both() {
    assert!(parse(&[]).is_err());
    assert!(parse(&["--controller", "192.0.2.1", "--sim"]).is_err());
    assert_eq!(
      parse(&["--controller", "192.0.2.1"]).unwrap().controller,
      Some("192.0.2.1".parse().unwrap())
    );
    assert!(parse(&["--sim"]).unwrap().sim);
  }

  #[test]
  fn the_simulated_basis_is_refused_without_the_simulation() {
    assert!(parse(&["--controller", "192.0.2.1", "--sim-basis", "plain"]).is_err());
    let cli = parse(&["--sim", "--sim-basis", "plain"]).unwrap();
    assert!(matches!(cli.sim_basis, Some(Basis::Plain)));
    assert!(parse(&["--sim"]).unwrap().sim_basis.is_none());
  }

  #[test]
  fn polling_is_off_unless_asked_for() {
    assert_eq!(parse(&["--sim"]).unwrap().poll_s, 0);
    assert_eq!(parse(&["--sim", "--poll-s", "30"]).unwrap().poll_s, 30);
  }
}
