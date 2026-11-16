//! 28BYJ-48 geared stepper driver on a ULN2003, generic over `embedded_hal::digital::OutputPin`.
//!
//! The drive scheme is addendum A.4: full-step, two-phase-on (the highest-torque mode, uniform torque per state);
//! coils off between moves; energize the stored phase, dwell, step, hold the last state, release. The driver issues
//! physical steps and reports how many it issued. Position, direction sign, soft limits, and the standard approach
//! belong to the node logic above it.
//!
//! Timing comes from `embedded_hal_async::delay::DelayNs` (embassy-time's `Delay` in firmware, a fake in tests).
//!
//! Safety: the coils are released whenever a move ends, including when its future is dropped part-way (a timeout,
//! a `select` that took the other branch). [`Coils`] releases them in `Drop`. The coils are never the interlock,
//! though: the switched motor rail is (addendum A.7). With the rail off, no pin state here can turn a motor.
#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};

use embedded_hal::digital::{OutputPin, PinState};
use embedded_hal_async::delay::DelayNs;

/// The four full-step, two-phase-on states, IN1 to IN4 (blue, pink, yellow, orange), from the addendum A.4 table.
/// Walking the table forward turns the motor one way, backward the other; which way advances the screw is wiring
/// and thread hand, set per axis at commissioning (`dir_sign`).
pub const SEQUENCE: [[bool; 4]; 4] = [
  [true, true, false, false],
  [false, true, true, false],
  [false, false, true, true],
  [true, false, false, true]
];

/// A row of [`SEQUENCE`]. Stored with the position so that a re-energized motor starts where it stopped
/// (addendum A.4: otherwise the rotor may jump by up to two motor steps).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub struct Phase(u8);

impl Phase {
  /// The first row of the table, where a motor that was never moved is assumed to be.
  pub const ZERO: Self = Self(0);

  /// The phase with this index, if it is 0 to 3.
  pub const fn new(index: u8) -> Option<Self> {
    if index < 4 { Some(Self(index)) } else { None }
  }

  /// The index, 0 to 3.
  pub const fn index(self) -> u8 {
    self.0
  }

  /// The next phase forward (`forward == true`) or backward through [`SEQUENCE`].
  #[must_use]
  pub const fn step(self, forward: bool) -> Self {
    Self(if forward { (self.0 + 1) % 4 } else { (self.0 + 3) % 4 })
  }

  /// The coil states of this phase, IN1 to IN4.
  pub const fn coils(self) -> [bool; 4] {
    match self.0 {
      0 => SEQUENCE[0],
      1 => SEQUENCE[1],
      2 => SEQUENCE[2],
      _ => SEQUENCE[3]
    }
  }
}

/// The timing of one move (addendum A.4 and A.9).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
  /// Time between steps, in microseconds.
  pub step_period_us: u32,
  /// Time from energizing the stored phase to the first step, in milliseconds.
  pub energize_dwell_ms: u32,
  /// Time the last state is held before the coils are released, in milliseconds.
  pub hold_before_release_ms: u32
}

/// Why a move ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum MoveEnd {
  /// Every step was issued.
  Done,
  /// The stop flag was set.
  Stopped,
  /// The motor rail read as down: steps after it were not issued, and steps just before it may not have turned
  /// the motor. The position is not trusted afterwards.
  RailLost
}

/// What a move did.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MoveReport {
  /// Why it ended.
  pub end: MoveEnd,
  /// Steps issued, signed like the request. Without feedback this is what was commanded, not what turned.
  pub steps_issued: i32
}

/// A coil output failed. The driver tried to release every coil before returning it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("a coil output pin failed ({0:?}); every coil was then set off, as far as the pins allowed")]
pub struct PinFault<E: core::fmt::Debug>(pub E);

/// The four coil outputs, IN1 to IN4. Releases every coil when dropped.
pub struct Coils<P: OutputPin> {
  pins: [P; 4]
}

impl<P: OutputPin> Coils<P> {
  /// Takes the four outputs and sets every coil off.
  ///
  /// # Errors
  ///
  /// [`PinFault`] if a pin could not be set.
  pub fn new(pins: [P; 4]) -> Result<Self, PinFault<P::Error>> {
    let mut coils = Self { pins };
    coils.release()?;
    Ok(coils)
  }

  /// Sets every coil off. Tries every pin even when one fails, and reports the first failure.
  ///
  /// # Errors
  ///
  /// [`PinFault`] with the first pin error.
  pub fn release(&mut self) -> Result<(), PinFault<P::Error>> {
    let mut first = Ok(());
    for pin in &mut self.pins {
      if let Err(error) = pin.set_low() {
        first = first.and(Err(PinFault(error)));
      }
    }
    first
  }

  fn energize(&mut self, phase: Phase) -> Result<(), PinFault<P::Error>> {
    for (pin, on) in self.pins.iter_mut().zip(phase.coils()) {
      if let Err(error) = pin.set_state(PinState::from(on)) {
        // A half-applied state is released at once rather than left on.
        let _released = self.release();
        return Err(PinFault(error));
      }
    }
    Ok(())
  }
}

impl<P: OutputPin> Drop for Coils<P> {
  fn drop(&mut self) {
    // Nothing can report from here; `release` already tries every pin.
    let _released = self.release();
  }
}

/// One motor: its coils and the phase it is in.
pub struct Stepper<P: OutputPin> {
  coils: Coils<P>,
  phase: Phase
}

impl<P: OutputPin> Stepper<P> {
  /// A stepper on these coils, resuming from a stored phase. The coils are off.
  pub const fn new(coils: Coils<P>, phase: Phase) -> Self {
    Self { coils, phase }
  }

  /// The phase the motor is in: store it with the position after every move.
  pub const fn phase(&self) -> Phase {
    self.phase
  }

  /// Turns the motor by `steps` physical full steps (positive walks [`SEQUENCE`] forward), then releases the coils.
  ///
  /// Before every step the stop flag and the rail are checked; a set flag or a rail that reads down ends the move.
  /// A move that issued every step holds the last state for the hold time before release, so that the rotor
  /// settles; a move that ended early releases at once. Zero steps touches no coil.
  ///
  /// # Errors
  ///
  /// [`PinFault`] if a coil output failed. The coils were released as far as the pins allowed.
  /// The motor may have moved by the steps issued before the fault, which the error does not report:
  /// treat the position as suspect.
  pub async fn run<D: DelayNs>(
    &mut self,
    steps: i32,
    timing: Timing,
    delay: &mut D,
    stop: &AtomicBool,
    mut rail_ok: impl FnMut() -> bool
  ) -> Result<MoveReport, PinFault<P::Error>> {
    if steps == 0 {
      return Ok(MoveReport {
        end: MoveEnd::Done,
        steps_issued: 0
      });
    }
    let forward = steps > 0;
    let unit = if forward { 1 } else { -1 };
    let mut issued = 0_i32;
    let mut end = MoveEnd::Done;

    if let Some(early) = check(stop, &mut rail_ok) {
      return Ok(MoveReport {
        end: early,
        steps_issued: 0
      });
    }
    self.coils.energize(self.phase)?;
    delay.delay_ms(timing.energize_dwell_ms).await;

    for _ in 0..steps.unsigned_abs() {
      if let Some(early) = check(stop, &mut rail_ok) {
        end = early;
        break;
      }
      self.phase = self.phase.step(forward);
      issued += unit;
      self.coils.energize(self.phase)?;
      delay.delay_us(timing.step_period_us).await;
    }

    if end == MoveEnd::Done {
      delay.delay_ms(timing.hold_before_release_ms).await;
    }
    self.coils.release()?;
    Ok(MoveReport {
      end,
      steps_issued: issued
    })
  }
}

fn check(stop: &AtomicBool, rail_ok: &mut impl FnMut() -> bool) -> Option<MoveEnd> {
  if stop.load(Ordering::Acquire) {
    Some(MoveEnd::Stopped)
  } else if !rail_ok() {
    Some(MoveEnd::RailLost)
  } else {
    None
  }
}

#[cfg(test)]
mod tests {
  extern crate std;

  use core::cell::RefCell;
  use core::convert::Infallible;
  use core::future::Future;
  use core::pin::pin;
  use core::task::{Context, Poll, Waker};
  use std::rc::Rc;
  use std::vec::Vec;

  use proptest::prelude::*;

  use super::*;

  /// What the fake hardware saw, in order: coil states after every pin write, and the waits.
  #[derive(Clone, Debug, PartialEq)]
  enum Event {
    Coils([bool; 4]),
    WaitUs(u64)
  }

  #[derive(Default)]
  struct Bench {
    levels: [bool; 4],
    events: Vec<Event>,
    /// Fail the pin with this index on its next write.
    fail_pin: Option<usize>
  }

  type Shared = Rc<RefCell<Bench>>;

  struct FakePin {
    index: usize,
    bench: Shared
  }

  #[derive(Debug, PartialEq)]
  struct PinBroke;

  impl embedded_hal::digital::Error for PinBroke {
    fn kind(&self) -> embedded_hal::digital::ErrorKind {
      embedded_hal::digital::ErrorKind::Other
    }
  }

  impl embedded_hal::digital::ErrorType for FakePin {
    type Error = PinBroke;
  }

  impl OutputPin for FakePin {
    fn set_low(&mut self) -> Result<(), PinBroke> {
      self.write(false)
    }
    fn set_high(&mut self) -> Result<(), PinBroke> {
      self.write(true)
    }
  }

  impl FakePin {
    fn write(&mut self, level: bool) -> Result<(), PinBroke> {
      let mut bench = self.bench.borrow_mut();
      if bench.fail_pin == Some(self.index) {
        bench.fail_pin = None;
        return Err(PinBroke);
      }
      bench.levels[self.index] = level;
      // The driver writes IN1 to IN4 in order: record the coil state once the sweep reaches IN4.
      if self.index == 3 {
        let levels = bench.levels;
        bench.events.push(Event::Coils(levels));
      }
      Ok(())
    }
  }

  struct FakeDelay {
    bench: Shared,
    /// Pend forever instead of completing, to test a dropped move.
    hang: bool
  }

  impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, ns: u32) {
      self.bench.borrow_mut().events.push(Event::WaitUs(u64::from(ns) / 1000));
      if self.hang {
        core::future::pending::<()>().await;
      }
    }
    async fn delay_us(&mut self, us: u32) {
      self.delay_ns(us.saturating_mul(1000)).await;
    }
    async fn delay_ms(&mut self, ms: u32) {
      self.bench.borrow_mut().events.push(Event::WaitUs(u64::from(ms) * 1000));
      if self.hang {
        core::future::pending::<()>().await;
      }
    }
  }

  const TIMING: Timing = Timing {
    step_period_us: 5_000,
    energize_dwell_ms: 20,
    hold_before_release_ms: 100
  };

  fn bench() -> (Shared, Stepper<FakePin>, FakeDelay) {
    let bench: Shared = Rc::default();
    let pins = [0, 1, 2, 3].map(|index| FakePin {
      index,
      bench: bench.clone()
    });
    let stepper = Stepper::new(Coils::new(pins).unwrap(), Phase::default());
    bench.borrow_mut().events.clear();
    let delay = FakeDelay {
      bench: bench.clone(),
      hang: false
    };
    (bench, stepper, delay)
  }

  /// Runs a future that never truly waits (the fake delay completes at once).
  fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut context = Context::from_waker(Waker::noop());
    loop {
      if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
        return output;
      }
    }
  }

  fn coil_states(bench: &Shared) -> Vec<[bool; 4]> {
    let mut states: Vec<[bool; 4]> = Vec::new();
    for event in &bench.borrow().events {
      if let Event::Coils(levels) = event {
        // Several writes make up one release; keep each distinct state once.
        if states.last() != Some(levels) {
          states.push(*levels);
        }
      }
    }
    states
  }

  const OFF: [bool; 4] = [false; 4];

  #[test]
  fn the_sequence_is_the_addendum_a_4_table() {
    assert_eq!(
      SEQUENCE,
      [
        [true, true, false, false],
        [false, true, true, false],
        [false, false, true, true],
        [true, false, false, true]
      ]
    );
  }

  #[test]
  fn every_state_has_exactly_two_coils_on_and_neighbors_share_one() {
    for (index, row) in SEQUENCE.iter().enumerate() {
      assert_eq!(row.iter().filter(|on| **on).count(), 2);
      let next = SEQUENCE[(index + 1) % 4];
      let shared = row.iter().zip(next).filter(|(a, b)| **a && *b).count();
      assert_eq!(shared, 1, "a full step moves one coil, not two");
    }
  }

  #[test]
  fn new_coils_start_off() {
    let bench: Shared = Rc::default();
    bench.borrow_mut().levels = [true; 4];
    let pins = [0, 1, 2, 3].map(|index| FakePin {
      index,
      bench: bench.clone()
    });
    let _coils = Coils::new(pins).unwrap();
    assert_eq!(bench.borrow().levels, OFF);
  }

  #[test]
  fn a_forward_move_energizes_the_stored_phase_then_walks_the_table_and_releases() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    let report = block_on(stepper.run(3, TIMING, &mut delay, &stop, || true)).unwrap();
    assert_eq!(
      report,
      MoveReport {
        end: MoveEnd::Done,
        steps_issued: 3
      }
    );
    assert_eq!(
      coil_states(&bench),
      [SEQUENCE[0], SEQUENCE[1], SEQUENCE[2], SEQUENCE[3], OFF]
    );
    assert_eq!(stepper.phase(), Phase::new(3).unwrap());
  }

  #[test]
  fn a_backward_move_walks_the_table_the_other_way() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    let report = block_on(stepper.run(-2, TIMING, &mut delay, &stop, || true)).unwrap();
    assert_eq!(report.steps_issued, -2);
    assert_eq!(coil_states(&bench), [SEQUENCE[0], SEQUENCE[3], SEQUENCE[2], OFF]);
    assert_eq!(stepper.phase(), Phase::new(2).unwrap());
  }

  #[test]
  fn the_waits_follow_the_energize_dwell_step_hold_release_order() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    block_on(stepper.run(2, TIMING, &mut delay, &stop, || true)).unwrap();
    let events = bench.borrow().events.clone();
    let waits: Vec<u64> = events
      .iter()
      .filter_map(|event| if let Event::WaitUs(us) = event { Some(*us) } else { None })
      .collect();
    assert_eq!(waits, [20_000, 5_000, 5_000, 100_000]);
    // The release comes after the hold.
    assert_eq!(events.last(), Some(&Event::Coils(OFF)));
  }

  #[test]
  fn zero_steps_touches_no_coil_and_does_not_wait() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    let report = block_on(stepper.run(0, TIMING, &mut delay, &stop, || true)).unwrap();
    assert_eq!(
      report,
      MoveReport {
        end: MoveEnd::Done,
        steps_issued: 0
      }
    );
    assert!(bench.borrow().events.is_empty());
  }

  #[test]
  fn a_set_stop_flag_ends_the_move_before_the_next_step_and_releases_without_holding() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    let mut checks = 0;
    // The flag is set by the third check: before the first step, after one step, then stopped.
    let rail = || {
      checks += 1;
      if checks == 2 {
        stop.store(true, Ordering::Release);
      }
      true
    };
    let report = block_on(stepper.run(10, TIMING, &mut delay, &stop, rail)).unwrap();
    assert_eq!(
      report,
      MoveReport {
        end: MoveEnd::Stopped,
        steps_issued: 1
      }
    );
    assert_eq!(coil_states(&bench), [SEQUENCE[0], SEQUENCE[1], OFF]);
    assert!(!bench.borrow().events.contains(&Event::WaitUs(100_000)));
  }

  #[test]
  fn a_stop_flag_set_before_the_move_touches_no_coil() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(true);
    let report = block_on(stepper.run(10, TIMING, &mut delay, &stop, || true)).unwrap();
    assert_eq!(
      report,
      MoveReport {
        end: MoveEnd::Stopped,
        steps_issued: 0
      }
    );
    assert!(bench.borrow().events.is_empty());
  }

  #[test]
  fn a_rail_that_drops_ends_the_move_as_rail_lost() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    let mut checks = 0;
    let rail = || {
      checks += 1;
      checks <= 3
    };
    let report = block_on(stepper.run(10, TIMING, &mut delay, &stop, rail)).unwrap();
    assert_eq!(
      report,
      MoveReport {
        end: MoveEnd::RailLost,
        steps_issued: 2
      }
    );
    assert_eq!(coil_states(&bench).last(), Some(&OFF));
  }

  #[test]
  fn a_rail_down_before_the_move_touches_no_coil() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    let report = block_on(stepper.run(10, TIMING, &mut delay, &stop, || false)).unwrap();
    assert_eq!(
      report,
      MoveReport {
        end: MoveEnd::RailLost,
        steps_issued: 0
      }
    );
    assert!(bench.borrow().events.is_empty());
  }

  #[test]
  fn dropping_a_move_part_way_releases_the_coils() {
    let (bench, mut stepper, _) = bench();
    let stop = AtomicBool::new(false);
    let mut hanging = FakeDelay {
      bench: bench.clone(),
      hang: true
    };
    {
      let mut future = pin!(stepper.run(10, TIMING, &mut hanging, &stop, || true));
      let mut context = Context::from_waker(Waker::noop());
      assert!(future.as_mut().poll(&mut context).is_pending());
      assert_eq!(bench.borrow().levels, SEQUENCE[0], "energized during the dwell");
    }
    drop(stepper);
    assert_eq!(bench.borrow().levels, OFF);
  }

  #[test]
  fn a_failing_pin_is_reported_and_the_other_coils_are_released() {
    let (bench, mut stepper, mut delay) = bench();
    let stop = AtomicBool::new(false);
    bench.borrow_mut().fail_pin = Some(1);
    let result = block_on(stepper.run(5, TIMING, &mut delay, &stop, || true));
    assert_eq!(result, Err(PinFault(PinBroke)));
    let levels = bench.borrow().levels;
    assert_eq!([levels[0], levels[2], levels[3]], [false; 3]);
  }

  #[test]
  fn release_tries_every_pin_even_after_one_fails() {
    let bench: Shared = Rc::default();
    let pins = [0, 1, 2, 3].map(|index| FakePin {
      index,
      bench: bench.clone()
    });
    let mut coils = Coils::new(pins).unwrap();
    bench.borrow_mut().levels = [true; 4];
    bench.borrow_mut().fail_pin = Some(0);
    assert_eq!(coils.release(), Err(PinFault(PinBroke)));
    assert_eq!(bench.borrow().levels, [true, false, false, false]);
  }

  #[test]
  fn phases_outside_the_table_do_not_exist() {
    assert_eq!(Phase::new(3).map(Phase::index), Some(3));
    assert_eq!(Phase::new(4), None);
  }

  proptest! {
    #[test]
    fn a_move_and_its_reverse_return_to_the_same_phase(start in 0_u8..4, steps in -500_i32..500) {
      let (_, _, mut delay) = bench();
      let bench_ref = delay.bench.clone();
      let pins = [0, 1, 2, 3].map(|index| FakePin { index, bench: bench_ref.clone() });
      let mut stepper = Stepper::new(Coils::new(pins).unwrap(), Phase::new(start).unwrap());
      let stop = AtomicBool::new(false);
      let there = block_on(stepper.run(steps, TIMING, &mut delay, &stop, || true)).unwrap();
      prop_assert_eq!(there.steps_issued, steps);
      let back = block_on(stepper.run(-steps, TIMING, &mut delay, &stop, || true)).unwrap();
      prop_assert_eq!(back.steps_issued, -steps);
      prop_assert_eq!(stepper.phase(), Phase::new(start).unwrap());
      prop_assert_eq!(bench_ref.borrow().levels, OFF);
    }

    #[test]
    fn the_phase_after_a_move_is_the_start_plus_the_steps_modulo_four(start in 0_u8..4, steps in -500_i32..500) {
      let (_, _, mut delay) = bench();
      let bench_ref = delay.bench.clone();
      let pins = [0, 1, 2, 3].map(|index| FakePin { index, bench: bench_ref.clone() });
      let mut stepper = Stepper::new(Coils::new(pins).unwrap(), Phase::new(start).unwrap());
      let stop = AtomicBool::new(false);
      block_on(stepper.run(steps, TIMING, &mut delay, &stop, || true)).unwrap();
      let expected = u8::try_from((i32::from(start) + steps).rem_euclid(4)).unwrap();
      prop_assert_eq!(stepper.phase().index(), expected);
    }
  }

  // Keeps `Infallible` in use: the esp-hal pins have this error type, and the driver must accept it.
  #[test]
  fn an_infallible_pin_type_is_accepted() {
    struct NeverFails;
    impl embedded_hal::digital::ErrorType for NeverFails {
      type Error = Infallible;
    }
    impl OutputPin for NeverFails {
      fn set_low(&mut self) -> Result<(), Infallible> {
        Ok(())
      }
      fn set_high(&mut self) -> Result<(), Infallible> {
        Ok(())
      }
    }
    let coils = Coils::new([NeverFails, NeverFails, NeverFails, NeverFails]).unwrap();
    let _stepper = Stepper::new(coils, Phase::default());
  }
}
