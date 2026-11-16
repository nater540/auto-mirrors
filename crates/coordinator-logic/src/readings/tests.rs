//! Pairing readings with shots, one rule at a time.
#![expect(
  clippy::float_cmp,
  reason = "Q15 and sum_q values divided by powers of two are exact, so these conversions compare exactly"
)]

use align_proto::{SUM_Q_SATURATED, SensorStatus};

use super::*;

const TIMING: CoordinatorTiming = CoordinatorTiming::STARTING;
const RESEND_MS: u64 = TIMING.resend_after_ms as u64;

fn status(result: SensorResult) -> SensorStatus {
  SensorStatus {
    result,
    calibrated: true,
    bench: false
  }
}

fn reading(seq: u8, h_q15: i16, v_q15: i16) -> SensorReading {
  SensorReading {
    seq,
    status: status(SensorResult::Valid),
    h_q15,
    v_q15,
    sum_q: SUM_Q_ONE
  }
}

/// A job with shots 10, 11, and 12 at A, B, and C, fired at 0, 100, and 200 ms.
fn job() -> Readings {
  let mut readings = Readings::new(&TIMING);
  readings.begin();
  readings.on_shot(0, 10, Point::A);
  readings.on_shot(100, 11, Point::B);
  readings.on_shot(200, 12, Point::C);
  readings
}

#[test]
fn the_job_is_complete_when_every_point_has_its_reading() {
  let mut readings = job();
  assert!(!readings.on_reading(reading(10, 0, 0)));
  assert!(!readings.on_reading(reading(11, 0, 0)));
  assert_eq!(readings.measurement(), None);
  assert!(readings.on_reading(reading(12, 0, 0)));
  assert!(readings.complete());
  assert!(matches!(readings.measurement(), Some(Ok(_))));
}

#[test]
fn readings_are_placed_by_their_sequence_number_whatever_order_they_come_in() {
  let mut readings = job();
  readings.on_reading(reading(12, 3, 0));
  readings.on_reading(reading(10, 1, 0));
  readings.on_reading(reading(11, 2, 0));
  let Some(Ok(shots)) = readings.measurement() else {
    panic!("complete")
  };
  let h = [shots.a, shots.b, shots.c].map(|reading| reading.h_norm * Q15_ONE);
  assert_eq!(h, [1.0, 2.0, 3.0]);
}

#[test]
fn bench_readings_strangers_and_second_copies_do_not_count() {
  let mut readings = job();
  let bench = SensorReading {
    status: SensorStatus {
      bench: true,
      ..status(SensorResult::Valid)
    },
    ..reading(10, 0, 0)
  };
  assert!(!readings.on_reading(bench));
  assert!(!readings.on_reading(reading(9, 0, 0)), "a shot of an earlier job");
  readings.on_reading(reading(10, 500, 0));
  readings.on_reading(reading(10, -500, 0));
  readings.on_reading(reading(11, 0, 0));
  readings.on_reading(reading(12, 0, 0));
  let Some(Ok(shots)) = readings.measurement() else {
    panic!("complete")
  };
  assert!(shots.a.h_norm > 0.0, "the first copy is kept");
}

#[test]
fn a_new_job_forgets_the_last_one() {
  let mut readings = job();
  readings.on_reading(reading(10, 0, 0));
  readings.begin();
  assert!(!readings.complete());
  assert_eq!(readings.measurement(), None);
  readings.on_shot(1_000, 13, Point::A);
  assert!(!readings.on_reading(reading(10, 0, 0)), "the old shot is gone");
  assert_eq!(
    readings.tick(10_000),
    [Some(13), None, None],
    "and only this job's shots are asked for"
  );
}

#[test]
fn a_job_with_a_point_missing_is_never_complete() {
  let mut readings = Readings::new(&TIMING);
  readings.on_shot(0, 10, Point::A);
  readings.on_shot(100, 11, Point::B);
  readings.on_reading(reading(10, 0, 0));
  assert!(!readings.on_reading(reading(11, 0, 0)));
  assert_eq!(readings.measurement(), None);
}

#[test]
fn a_late_reading_is_asked_for_again_at_each_interval_until_it_arrives() {
  let mut readings = job();
  readings.on_reading(reading(10, 0, 0));
  assert_eq!(
    readings.tick(RESEND_MS + 99),
    [None, None, None],
    "B was shot at 100 ms"
  );
  assert_eq!(readings.tick(RESEND_MS + 100), [None, Some(11), None]);
  assert_eq!(readings.tick(RESEND_MS + 200), [None, None, Some(12)]);
  assert_eq!(
    readings.tick(2 * RESEND_MS + 99),
    [None, None, None],
    "not again before the interval"
  );
  assert_eq!(readings.tick(2 * RESEND_MS + 100), [None, Some(11), None]);
  readings.on_reading(reading(11, 0, 0));
  assert_eq!(readings.tick(10 * RESEND_MS), [None, None, Some(12)]);
}

#[test]
fn the_first_reading_the_node_rejected_in_point_order_spoils_the_measurement() {
  let mut readings = job();
  let rejected = |seq, result| SensorReading {
    status: status(result),
    ..reading(seq, 0, 0)
  };
  readings.on_reading(reading(10, 0, 0));
  readings.on_reading(rejected(11, SensorResult::Overlap));
  readings.on_reading(rejected(12, SensorResult::Saturated));
  assert_eq!(readings.measurement(), Some(Err(ReadingFault::Overlap)));
}

#[test]
fn a_valid_reading_becomes_a_position_in_units_of_one_and_a_sum_relative_to_s_ref() {
  let converted = to_reading(&SensorReading {
    sum_q: SUM_Q_ONE / 2,
    ..reading(1, i16::MIN, i16::MAX)
  })
  .unwrap();
  assert_eq!(converted.h_norm, -1.0);
  assert!((converted.v_norm - 1.0).abs() < 1.0e-4);
  assert_eq!(converted.sum, 0.5);
  let saturated_sum = to_reading(&SensorReading {
    sum_q: SUM_Q_SATURATED,
    ..reading(1, 0, 0)
  })
  .unwrap();
  assert!(saturated_sum.sum >= 1.99);
}

#[test]
fn every_rejection_of_the_node_has_its_fault() {
  for (result, fault) in [
    (SensorResult::NoShot, ReadingFault::SumLow),
    (SensorResult::Saturated, ReadingFault::Saturated),
    (SensorResult::NegativeQuadrant, ReadingFault::NegativeQuadrant),
    (SensorResult::Overlap, ReadingFault::Overlap),
    (SensorResult::BaselineUnstable, ReadingFault::BaselineUnstable),
    (SensorResult::TimedOut, ReadingFault::DecayNotEnded),
    (SensorResult::FrontEndFault, ReadingFault::FrontEndFault)
  ] {
    let sensor = SensorReading {
      status: status(result),
      ..reading(1, 100, 100)
    };
    assert_eq!(to_reading(&sensor), Err(fault), "{result:?}");
  }
}
