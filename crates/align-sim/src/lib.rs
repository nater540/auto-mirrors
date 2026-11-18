//! Two-mirror optics and mount simulator, with the closed-loop tests that hold the alignment logic to it.
//!
//! - [`world`]: the model. Two mirrors, four screws with backlash, stick-slip and skipped steps, and a quadrant
//!   sensor with noise, a finite aperture, and a constant head offset (reference 1.3 and 2.2).
//! - [`rig`]: the model behind the `Actuator` and `BeamSensor` traits of `align-core`, so the same
//!   [`align_core::aligner::Aligner`] that will run on the coordinator runs here.
//! - [`system`]: the whole system behind the same traits: the coordinator's orchestrator, the two mirror nodes, and
//!   the sensor node on a simulated bus, with the relay, the motors, and the Ruida simulated around them.
//! - [`external`]: a machine in another program behind the same traits, one line per move or measurement:
//!   `tools/beamviz` answers with a ray-traced machine of its own.
//! - [`block_on`]: runs the aligner's futures to completion on the host.
//!
//! Every number that describes the machine here (path lengths, beam radius, backlash, noise) is an UNVERIFIED
//! placeholder until the owner's measurements arrive. They are scenario parameters, not configuration.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

pub mod external;
pub mod rig;
pub mod system;
pub mod world;

/// Runs a future to completion on the calling thread.
///
/// The simulated rig answers at once, so the futures of the aligner never wait and the first poll finishes them.
/// A future that does wait is polled again in a loop.
pub fn block_on<F: Future>(future: F) -> F::Output {
  let mut future = pin!(future);
  let mut context = Context::from_waker(Waker::noop());
  loop {
    if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
      return output;
    }
    std::thread::yield_now();
  }
}
