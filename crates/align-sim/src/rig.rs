//! The simulated machine behind the `Actuator` and `BeamSensor` traits.
//!
//! The aligner owns its actuator and its sensor separately, and here both are the same machine, so a [`Rig`] is a
//! shared handle: clone it once for each role and keep a third to look at the truth afterwards.

use std::cell::{Ref, RefCell, RefMut};
use std::rc::Rc;

use align_core::aligner::Aligner;
use align_core::axis::Axis;
use align_core::calibration::Calibration;
use align_core::config::Config;
use align_core::measurement::Shots;
use align_core::rig::{Actuator, BeamSensor, MoveFault, SensorFault};
use align_core::solver::Conditioning;

use crate::world::{Scenario, World};

/// A shared handle to one simulated machine.
#[derive(Clone, Debug)]
pub struct Rig(Rc<RefCell<World>>);

impl Rig {
  /// A machine in the scenario's starting state.
  pub fn new(scenario: Scenario) -> Self {
    Self(Rc::new(RefCell::new(World::new(scenario))))
  }

  /// The machine, to read its truth.
  ///
  /// # Panics
  ///
  /// If the machine is mutably borrowed, which no method of this crate leaves it.
  pub fn world(&self) -> Ref<'_, World> {
    self.0.borrow()
  }

  /// The machine, to change it during a run.
  ///
  /// # Panics
  ///
  /// If the machine is borrowed, which no method of this crate leaves it.
  pub fn world_mut(&self) -> RefMut<'_, World> {
    self.0.borrow_mut()
  }

  /// An aligner that drives this machine, configured with the machine's own mechanics and reference sum.
  pub fn aligner(&self, mut config: Config) -> Aligner<Rig, Rig> {
    let (mechanics, sum_ref) = {
      let world = self.world();
      (world.scenario().mechanics, world.scenario().beam.sum_ref)
    };
    config.mechanics = mechanics;
    #[expect(clippy::cast_possible_truncation, reason = "the alignment code works in f32")]
    let sum_ref = sum_ref as f32;
    Aligner::new(self.clone(), self.clone(), config, sum_ref)
  }
}

impl Rig {
  /// The calibration a healthy copy of this machine would have stored, with column `j` of the Jacobian multiplied
  /// by `scale[j]`: all ones for a correct Jacobian, `0.7` or `1.3` for one that is 30% wrong.
  pub fn stored_calibration(&self, scale: [f32; 4]) -> Calibration {
    let mut jacobian = self.world().true_jacobian();
    for (mut column, factor) in jacobian.column_iter_mut().zip(scale) {
      column *= factor;
    }
    let cond = Conditioning::of(&jacobian).map_or(f32::NAN, |conditioning| conditioning.cond);
    Calibration {
      jacobian,
      q0_steps: [0; 4],
      cond,
    }
  }
}

impl Actuator for Rig {
  /// The simulated node has no feedback either: it always reports success, whatever the screw did.
  async fn move_rel(&mut self, axis: Axis, delta_steps: i32) -> Result<(), MoveFault> {
    self.world_mut().move_rel(axis, delta_steps);
    Ok(())
  }
}

impl BeamSensor for Rig {
  async fn measure(&mut self) -> Result<Shots, SensorFault> {
    Ok(self.world_mut().measure())
  }
}
