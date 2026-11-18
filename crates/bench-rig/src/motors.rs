//! The bench's motors behind the aligner's `Actuator`: each axis is a channel on a node's console, and a move is a
//! `jog` answered by the node's count.
//!
//! - **The node's count is the position**, as on the bus (`Actuator::move_rel`): a move that ended returns it, and
//!   one cut short carries it in its fault.
//! - **A refusal carries code 0**: the bench console's refusals have no fault code on the bus. No node sends 0, and
//!   the reason is in the log and in [`BenchMotors::last_problem`].
//! - **An axis with no motor is refused** the same way, before anything is sent.

use std::io::{Read, Write};
use std::time::Duration;

use align_core::axis::Axis;
use align_core::config::Motion;
use align_core::rig::{Actuator, MoveFault};
use node_logic::bench_line::BenchEnd;

use crate::console::{Jog, NodeConsole};

/// The code of a refusal from the bench console, which has none on the bus.
pub const BENCH_REFUSED: u8 = 0;

/// Time beyond a move's own steps before it is stopped: the energize dwell, the hold before release, and the console.
const MOVE_MARGIN: Duration = Duration::from_secs(3);

/// Which console and channel turn an axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Motor {
  /// The axis.
  pub axis: Axis,
  /// Its node, as an index into the consoles.
  pub console: usize,
  /// Its channel on that node.
  pub channel: u8
}

/// The motors of the bench.
pub struct BenchMotors<P> {
  consoles: Vec<NodeConsole<P>>,
  motors: Vec<Motor>,
  step_period: Duration,
  last_problem: Option<String>
}

impl<P: Read + Write> BenchMotors<P> {
  /// The motors on `consoles`, moving at the configured step period.
  pub fn new(consoles: Vec<NodeConsole<P>>, motors: Vec<Motor>, motion: &Motion) -> Self {
    Self {
      consoles,
      motors,
      step_period: Duration::from_micros(u64::from(motion.step_period_us)),
      last_problem: None
    }
  }

  /// What went wrong with the last move that failed, in words.
  pub fn last_problem(&self) -> Option<&str> {
    self.last_problem.as_deref()
  }

  /// The console and channel of `axis`.
  pub fn console_of(&mut self, axis: Axis) -> Option<(&mut NodeConsole<P>, u8)> {
    let motor = *self.motors.iter().find(|motor| motor.axis == axis)?;
    Some((self.consoles.get_mut(motor.console)?, motor.channel))
  }

  /// Every console, as the command line gave them.
  pub fn consoles_mut(&mut self) -> &mut [NodeConsole<P>] {
    &mut self.consoles
  }

  fn wait_for(&self, steps: i32) -> Duration {
    self
      .step_period
      .saturating_mul(steps.unsigned_abs())
      .saturating_add(MOVE_MARGIN)
  }

  fn jog(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    let wait = self.wait_for(delta_steps);
    let Some((console, channel)) = self.console_of(axis) else {
      self.last_problem = Some(format!("no motor turns {axis:?} on this bench"));
      return Err(MoveFault::Refused { code: BENCH_REFUSED });
    };
    let node = console.node().to_owned();
    let result = console.jog(channel, delta_steps, wait);
    let (outcome, problem) = match result {
      Ok(Jog::Moved {
        end: BenchEnd::Done,
        pos_steps,
        ..
      }) => (Ok(pos_steps), None),
      Ok(Jog::Moved { end, pos_steps, .. }) => {
        let fault = match end {
          BenchEnd::RailLost => MoveFault::RailLost { pos_steps },
          BenchEnd::PinFault => MoveFault::NoAnswer,
          BenchEnd::Done | BenchEnd::Stopped | BenchEnd::TimedOut => MoveFault::Stopped { pos_steps }
        };
        (
          Err(fault),
          Some(format!("{node} channel {channel}: the move ended {}", end.name()))
        )
      }
      Ok(Jog::Refused(reason)) => (
        Err(MoveFault::Refused { code: BENCH_REFUSED }),
        Some(format!("{node} channel {channel} refused the move: {reason}"))
      ),
      Err(error) => (Err(MoveFault::NoAnswer), Some(error.to_string()))
    };
    if problem.is_some() {
      self.last_problem = problem;
    }
    outcome
  }
}

impl<P: Read + Write> Actuator for BenchMotors<P> {
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    self.jog(axis, delta_steps)
  }
}

/// Lent to the aligner, so that the consoles and the last problem are still there after the procedure.
impl<P: Read + Write> Actuator for &mut BenchMotors<P> {
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<i32, MoveFault> {
    self.jog(axis, delta_steps)
  }
}

#[cfg(test)]
mod tests {
  use std::cell::RefCell;
  use std::rc::Rc;

  use align_core::config::Config;
  use align_sim::block_on;

  use super::*;
  use crate::console::Log;
  use crate::fake_node::{FakeNode, Machine, Port};

  fn motors(machine: &Machine, bench: bool) -> BenchMotors<Port> {
    let log: Log = Rc::new(RefCell::new(Vec::<u8>::new()));
    let mut console = NodeConsole::new(FakeNode::spawn(machine.clone()), "node-a", log);
    console.status(0).unwrap();
    if bench {
      console.bench(true).unwrap();
    }
    let motor = Motor {
      axis: Axis::M1H,
      console: 0,
      channel: 0
    };
    BenchMotors::new(vec![console], vec![motor], &Config::default().motion)
  }

  #[test]
  fn a_move_returns_the_node_s_count() {
    let machine = Machine::default();
    machine.set_count(0, 100);
    let mut motors = motors(&machine, true);
    assert_eq!(block_on(motors.move_rel(Axis::M1H, -30)), Ok(70));
    assert_eq!(motors.last_problem(), None);
  }

  #[test]
  fn a_refused_move_and_an_axis_without_a_motor_are_refused_with_the_reason_kept() {
    let machine = Machine::default();
    let mut motors = motors(&machine, false);
    assert_eq!(
      block_on(motors.move_rel(Axis::M1H, 10)),
      Err(MoveFault::Refused { code: BENCH_REFUSED })
    );
    assert!(
      motors
        .last_problem()
        .is_some_and(|problem| problem.contains("not in bench mode"))
    );
    assert_eq!(
      block_on(motors.move_rel(Axis::M2V, 10)),
      Err(MoveFault::Refused { code: BENCH_REFUSED })
    );
    assert!(motors.last_problem().is_some_and(|problem| problem.contains("M2V")));
    assert!(
      !machine
        .received()
        .iter()
        .any(|line| line.starts_with("jog") && line.contains("M2V"))
    );
  }

  #[test]
  fn a_move_cut_short_carries_the_node_s_count() {
    let machine = Machine::default();
    let mut motors = motors(&machine, true);
    machine.hold_moves_until_stopped();
    motors.step_period = Duration::ZERO;
    // The wait is the margin alone; the fake holds the move until the stop that follows it.
    assert_eq!(
      block_on(motors.move_rel(Axis::M1H, 40)),
      Err(MoveFault::Stopped { pos_steps: 20 })
    );
    assert!(motors.last_problem().is_some_and(|problem| problem.contains("stopped")));
  }

  #[test]
  fn the_wait_for_a_move_is_its_steps_at_the_configured_rate_and_a_margin() {
    let machine = Machine::default();
    let motors = motors(&machine, true);
    // 400 steps at 5000 us is 2 s, and the margin 3 s.
    assert_eq!(motors.wait_for(-400), Duration::from_secs(5));
  }
}
