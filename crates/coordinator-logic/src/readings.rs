//! Pairing the sensor's readings with the shots of a job (`docs/SENSOR_PLAN.md` section 2).
//!
//! For each fire the state machine announces a shot (`Actions::shot`: its sequence number and point), and the
//! firmware sends SHOT. The sensor answers each with SENSOR, in shot order, some time after the shot.
//! This module keeps the shots of the current job, takes the readings that answer them, and asks again with
//! SENSOR_RESEND for one that is late. Once every point has its reading, it gives the three for the aligner,
//! or the first one the sensor node rejected.
//!
//! - A reading counts only if its sequence number is a shot of this job. A bench reading (the bench bit), one for
//!   a shot of an earlier job, and a second copy of one already held are ignored.
//! - A reading still missing `resend_after_ms` after its shot, or after it was last asked for, is asked for again,
//!   until the state machine's `reading_timeout_ms` ends the job (`Outcome::ReadingsTimedOut`).

use align_core::config::CoordinatorTiming;
use align_core::coordinator::Point;
use align_core::measurement::Shots;
use align_core::quadrant::{Reading, ReadingFault};
use align_proto::{SUM_Q_ONE, SensorReading, SensorResult};

/// The points of a job, in the order it fires them.
const POINTS: usize = 3;

/// One shot of the job, and its reading once it arrives.
#[derive(Clone, Copy, Debug)]
struct Expected {
  seq: u8,
  /// When the reading was last asked for: the shot, or the last SENSOR_RESEND.
  asked_ms: u64,
  reading: Option<SensorReading>
}

/// The shots of the current job and their readings.
#[derive(Clone, Debug)]
pub struct Readings {
  resend_after_ms: u32,
  /// By point: A, B, C.
  shots: [Option<Expected>; POINTS]
}

impl Readings {
  /// No job yet.
  pub fn new(timing: &CoordinatorTiming) -> Self {
    Self {
      resend_after_ms: timing.resend_after_ms,
      shots: [None; POINTS]
    }
  }

  /// Forgets the last job: called when a measurement starts, so that nothing of an earlier job is counted.
  pub fn begin(&mut self) {
    self.shots = [None; POINTS];
  }

  /// A shot of this job, as the state machine announced it, at the local time `now_ms`.
  pub fn on_shot(&mut self, now_ms: u64, seq: u8, point: Point) {
    if let Some(slot) = self.shots.get_mut(index(point)) {
      *slot = Some(Expected {
        seq,
        asked_ms: now_ms,
        reading: None
      });
    }
  }

  /// A SENSOR frame. Returns whether every point now has its reading, which is when the state machine is told
  /// (`Event::ReadingsComplete`); a reading that completes nothing, or does not count, returns `false`.
  pub fn on_reading(&mut self, reading: SensorReading) -> bool {
    if reading.status.bench {
      return false;
    }
    let waiting = self
      .shots
      .iter_mut()
      .flatten()
      .find(|expected| expected.seq == reading.seq && expected.reading.is_none());
    match waiting {
      Some(expected) => {
        expected.reading = Some(reading);
        self.complete()
      }
      None => false
    }
  }

  /// The shots whose reading is late at the local time `now_ms`, by point, to ask for again with SENSOR_RESEND.
  pub fn tick(&mut self, now_ms: u64) -> [Option<u8>; POINTS] {
    let resend_after_ms = u64::from(self.resend_after_ms);
    let mut resends = [None; POINTS];
    for (slot, resend) in self.shots.iter_mut().zip(resends.iter_mut()) {
      if let Some(expected) = slot
        && expected.reading.is_none()
        && now_ms.saturating_sub(expected.asked_ms) >= resend_after_ms
      {
        expected.asked_ms = now_ms;
        *resend = Some(expected.seq);
      }
    }
    resends
  }

  /// Whether every point of the job has its reading.
  pub fn complete(&self) -> bool {
    self
      .shots
      .iter()
      .all(|slot| slot.is_some_and(|expected| expected.reading.is_some()))
  }

  /// The job's three readings once it is complete, or the first, in point order, that the sensor node rejected.
  /// `None` until every point has its reading.
  pub fn measurement(&self) -> Option<Result<Shots, ReadingFault>> {
    let [a, b, c] = self.shots.map(|slot| slot.and_then(|expected| expected.reading));
    Some(shots(&a?, &b?, &c?))
  }
}

/// The three readings of a job, or the first the sensor node rejected.
fn shots(a: &SensorReading, b: &SensorReading, c: &SensorReading) -> Result<Shots, ReadingFault> {
  Ok(Shots {
    a: to_reading(a)?,
    b: to_reading(b)?,
    c: to_reading(c)?
  })
}

/// Where a point's shot is kept.
const fn index(point: Point) -> usize {
  match point {
    Point::A => 0,
    Point::B => 1,
    Point::C => 2
  }
}

/// A SENSOR reading as the aligner reads it: the position from Q15 and the sum relative to `S_ref`, so that the
/// aligner's reference sum is `1.0`.
///
/// # Errors
///
/// The [`ReadingFault`] that matches the sensor node's own rejection: `NoShot` is a sum too low to use, and the
/// results whose position is absent or not to be trusted have their own faults.
pub fn to_reading(reading: &SensorReading) -> Result<Reading, ReadingFault> {
  match reading.status.result {
    SensorResult::Valid => Ok(Reading {
      h_norm: f32::from(reading.h_q15) / Q15_ONE,
      v_norm: f32::from(reading.v_q15) / Q15_ONE,
      sum: f32::from(reading.sum_q) / f32::from(SUM_Q_ONE)
    }),
    SensorResult::NoShot => Err(ReadingFault::SumLow),
    SensorResult::Saturated => Err(ReadingFault::Saturated),
    SensorResult::NegativeQuadrant => Err(ReadingFault::NegativeQuadrant),
    SensorResult::Overlap => Err(ReadingFault::Overlap),
    SensorResult::BaselineUnstable => Err(ReadingFault::BaselineUnstable),
    SensorResult::TimedOut => Err(ReadingFault::DecayNotEnded),
    SensorResult::FrontEndFault => Err(ReadingFault::FrontEndFault)
  }
}

/// A Q15 value of `1.0`: positions are `i16` fractions of it.
const Q15_ONE: f32 = 32_768.0;

#[cfg(test)]
mod tests;
