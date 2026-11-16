//! The sensor node's messages (`docs/SENSOR_PLAN.md` section 3, approved by the owner on 2026-11-15).
//!
//! SHOT tells the sensor that the laser fired; SENSOR answers with the reading; SENSOR_RESEND asks again for a reading
//! that was lost; SENSOR_CMD and its reply run commissioning from the coordinator's console. The byte layouts are in
//! [`crate::message`]; this module holds the field types and their byte values.

/// The layout version of this protocol, carried in every HEARTBEAT. Version 2 adds the sensor classes 0xA to 0xD,
/// redefines SENSOR, and lengthens HEARTBEAT to carry this byte. A node refuses to act on a coordinator whose
/// heartbeat carries another version.
pub const PROTOCOL_VERSION: u8 = 2;

/// Where the gantry was when the laser fired (reference 1.4).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShotPoint {
  /// Y near, X near.
  A,
  /// Y far, X near.
  B,
  /// Y far, X far.
  C,
  /// Not a measurement point: a bench or test shot.
  Other
}

impl ShotPoint {
  /// The byte on the wire.
  pub const fn to_byte(self) -> u8 {
    match self {
      Self::A => 0,
      Self::B => 1,
      Self::C => 2,
      Self::Other => 0xFF
    }
  }

  /// The point a byte names, if it names one.
  pub const fn from_byte(value: u8) -> Option<Self> {
    Some(match value {
      0 => Self::A,
      1 => Self::B,
      2 => Self::C,
      0xFF => Self::Other,
      _ => return None
    })
  }
}

/// The laser fired (SHOT, class 0xA, coordinator to every node).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Shot {
  /// Shot number; the SENSOR reply echoes it.
  pub seq: u8,
  /// Where the gantry was.
  pub point: ShotPoint,
  /// Time from the start of the fire burst to queuing this frame, in units of 10 us; `None` when not known.
  /// On the wire `0xFFFF` means unknown, so a known age saturates at `0xFFFE` (655.34 ms).
  pub edge_age_10us: Option<u16>
}

/// The wire value of an unknown edge age.
pub const EDGE_AGE_UNKNOWN: u16 = 0xFFFF;

/// The outcome of one shot at the sensor, the low five bits of SENSOR's `status` byte.
///
/// The codes are sent in priority order: when several apply, the first in this list wins
/// (`FrontEndFault`, `TimedOut`, `BaselineUnstable`, `NoShot`, `Saturated`, `Overlap`, `NegativeQuadrant`, `Valid`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SensorResult {
  /// Every node-side check passed.
  Valid = 0,
  /// No rise at the fire: no fire, the beam off the sensor, or clipped upstream.
  NoShot = 1,
  /// A raw sample reached the saturation level during the shot.
  Saturated = 2,
  /// A quadrant integral fell below `-3` of its propagated sigma: a baseline error.
  NegativeQuadrant = 3,
  /// The baseline before the burst was not flat enough to fit.
  BaselineUnstable = 4,
  /// The previous shot's tail model left a residual above its test.
  Overlap = 6,
  /// The decay did not end within the cap, or its samples left the buffer.
  TimedOut = 7,
  /// The front end failed during the shot.
  FrontEndFault = 9
}

impl SensorResult {
  /// The result a code names, if it names one. Codes 5 and 8 were retired in plan revision 3.
  pub const fn from_code(code: u8) -> Option<Self> {
    Some(match code {
      0 => Self::Valid,
      1 => Self::NoShot,
      2 => Self::Saturated,
      3 => Self::NegativeQuadrant,
      4 => Self::BaselineUnstable,
      6 => Self::Overlap,
      7 => Self::TimedOut,
      9 => Self::FrontEndFault,
      _ => return None
    })
  }

  /// Whether the reading carries a position at all. Some results carry one that is not to be trusted.
  pub const fn has_position(self) -> bool {
    matches!(
      self,
      Self::Valid | Self::Saturated | Self::Overlap | Self::NegativeQuadrant
    )
  }
}

/// SENSOR's `status` byte.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SensorStatus {
  /// What happened to the shot.
  pub result: SensorResult,
  /// The sensor's `S_ref` and `tau_cal` come from commissioning, not from defaults.
  pub calibrated: bool,
  /// A bench shot: fired by a button, not answered to a SHOT.
  pub bench: bool
}

const RESULT_MASK: u8 = 0b0001_1111;
const CALIBRATED_BIT: u8 = 0b0010_0000;
const BENCH_BIT: u8 = 0b1000_0000;

impl SensorStatus {
  /// The byte on the wire. Bit 6 is reserved and written as zero.
  pub const fn to_byte(self) -> u8 {
    let mut byte = self.result as u8;
    if self.calibrated {
      byte |= CALIBRATED_BIT;
    }
    if self.bench {
      byte |= BENCH_BIT;
    }
    byte
  }

  /// The status a byte carries, if its result code is defined. Bit 6 is reserved and ignored.
  pub const fn from_byte(byte: u8) -> Option<Self> {
    match SensorResult::from_code(byte & RESULT_MASK) {
      Some(result) => Some(Self {
        result,
        calibrated: byte & CALIBRATED_BIT != 0,
        bench: byte & BENCH_BIT != 0
      }),
      None => None
    }
  }
}

/// `sum_q` of a sum equal to `S_ref`.
pub const SUM_Q_ONE: u16 = 32_768;

/// `sum_q` of a sum at or above twice `S_ref`: never a usable sum.
pub const SUM_Q_SATURATED: u16 = u16::MAX;

/// A beam reading (SENSOR, class 0x7, sensor to coordinator).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SensorReading {
  /// The SHOT `seq` it answers; for a bench shot, the sensor's own counter.
  pub seq: u8,
  /// The result and its flags.
  pub status: SensorStatus,
  /// Normalized horizontal position, Q15 (`32767` is about `+1.0`); 0 when the result has no position.
  pub h_q15: i16,
  /// Normalized vertical position, Q15.
  pub v_q15: i16,
  /// The sum relative to `S_ref`, [`SUM_Q_ONE`] being equal.
  /// It is filled whenever anything was integrated, so that a [`SensorResult::NoShot`] still says how much signal
  /// there was; it is 0 for [`SensorResult::TimedOut`] and [`SensorResult::FrontEndFault`].
  pub sum_q: u16
}

/// A commissioning operation (SENSOR_CMD).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SensorOp {
  /// Measure sigma per quadrant for `arg` seconds, beam off.
  Noise = 1,
  /// Fit tau per quadrant on the next shot's decay.
  Tau = 2,
  /// Average the sum of the next `arg` shots, with the beam on the sensor.
  SRef = 3,
  /// Write the calibration record.
  Save = 4,
  /// Report quadrant `arg`'s tau (in `value`, microseconds) and sigma (in `extra`, counts).
  Report = 5,
  /// Leave (`arg` 0) or enter (`arg` 1) bench mode.
  Bench = 6
}

impl SensorOp {
  /// The operation a byte names, if it names one.
  pub const fn from_byte(value: u8) -> Option<Self> {
    Some(match value {
      1 => Self::Noise,
      2 => Self::Tau,
      3 => Self::SRef,
      4 => Self::Save,
      5 => Self::Report,
      6 => Self::Bench,
      _ => return None
    })
  }
}

/// A commissioning command (SENSOR_CMD, class 0xC, coordinator to sensor).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SensorCmd {
  /// What to do.
  pub op: SensorOp,
  /// Its argument; see [`SensorOp`].
  pub arg: u16
}

/// How a command went.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum CmdStatus {
  /// Done.
  Ok = 0,
  /// Another command or a shot is in progress.
  Busy = 1,
  /// Not allowed now, or the argument is out of range.
  Refused = 2,
  /// Tried and failed.
  Failed = 3
}

impl CmdStatus {
  /// The status a byte names, if it names one.
  pub const fn from_byte(value: u8) -> Option<Self> {
    Some(match value {
      0 => Self::Ok,
      1 => Self::Busy,
      2 => Self::Refused,
      3 => Self::Failed,
      _ => return None
    })
  }
}

/// The answer to one SENSOR_CMD (SENSOR_CMD_REPLY, class 0xD, sensor to coordinator).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SensorCmdReply {
  /// The operation answered.
  pub op: SensorOp,
  /// How it went.
  pub status: CmdStatus,
  /// The result value, in the unit the operation names; 0 when it has none.
  pub value: i32,
  /// A second value where the operation names one (Report: sigma in counts); 0 otherwise.
  pub extra: u16
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;

  #[test]
  fn the_shot_points_have_the_plan_s_byte_values() {
    let points = [ShotPoint::A, ShotPoint::B, ShotPoint::C, ShotPoint::Other];
    assert_eq!(points.map(ShotPoint::to_byte), [0, 1, 2, 0xFF]);
    for point in points {
      assert_eq!(ShotPoint::from_byte(point.to_byte()), Some(point));
    }
    assert_eq!(ShotPoint::from_byte(3), None);
    assert_eq!(ShotPoint::from_byte(0xFE), None);
  }

  #[test]
  fn the_result_codes_have_the_plan_s_values_and_5_and_8_are_retired() {
    let results = [
      (SensorResult::Valid, 0),
      (SensorResult::NoShot, 1),
      (SensorResult::Saturated, 2),
      (SensorResult::NegativeQuadrant, 3),
      (SensorResult::BaselineUnstable, 4),
      (SensorResult::Overlap, 6),
      (SensorResult::TimedOut, 7),
      (SensorResult::FrontEndFault, 9)
    ];
    for (result, code) in results {
      assert_eq!(result as u8, code);
      assert_eq!(SensorResult::from_code(code), Some(result));
    }
    for retired in [5, 8, 10, 31] {
      assert_eq!(SensorResult::from_code(retired), None);
    }
  }

  #[test]
  fn only_the_results_the_plan_names_carry_a_position() {
    let with_position = [
      SensorResult::Valid,
      SensorResult::Saturated,
      SensorResult::Overlap,
      SensorResult::NegativeQuadrant
    ];
    let without = [
      SensorResult::NoShot,
      SensorResult::BaselineUnstable,
      SensorResult::TimedOut,
      SensorResult::FrontEndFault
    ];
    assert!(with_position.iter().all(|result| result.has_position()));
    assert!(without.iter().all(|result| !result.has_position()));
  }

  #[test]
  fn the_status_byte_packs_the_code_low_calibrated_in_bit_5_and_bench_in_bit_7() {
    let status = SensorStatus {
      result: SensorResult::Overlap,
      calibrated: true,
      bench: true
    };
    assert_eq!(status.to_byte(), 0b1010_0110);
    let plain = SensorStatus {
      result: SensorResult::Valid,
      calibrated: false,
      bench: false
    };
    assert_eq!(plain.to_byte(), 0);
  }

  #[test]
  fn the_reserved_status_bit_is_ignored_and_an_undefined_code_is_rejected() {
    let with_reserved = 0b0100_0001;
    assert_eq!(
      SensorStatus::from_byte(with_reserved),
      Some(SensorStatus {
        result: SensorResult::NoShot,
        calibrated: false,
        bench: false
      })
    );
    assert_eq!(SensorStatus::from_byte(5), None);
    assert_eq!(SensorStatus::from_byte(0b1010_1000), None);
  }

  #[test]
  fn the_sensor_ops_and_command_statuses_have_the_plan_s_values() {
    let ops = [
      SensorOp::Noise,
      SensorOp::Tau,
      SensorOp::SRef,
      SensorOp::Save,
      SensorOp::Report,
      SensorOp::Bench
    ];
    for (op, byte) in ops.iter().zip(1_u8..) {
      assert_eq!(*op as u8, byte);
      assert_eq!(SensorOp::from_byte(byte), Some(*op));
    }
    assert_eq!(SensorOp::from_byte(0), None);
    assert_eq!(SensorOp::from_byte(7), None);
    let statuses = [CmdStatus::Ok, CmdStatus::Busy, CmdStatus::Refused, CmdStatus::Failed];
    for (status, byte) in statuses.iter().zip(0_u8..) {
      assert_eq!(*status as u8, byte);
      assert_eq!(CmdStatus::from_byte(byte), Some(*status));
    }
    assert_eq!(CmdStatus::from_byte(4), None);
  }

  #[test]
  fn sum_q_has_one_at_32768_and_saturation_at_the_top() {
    assert_eq!(SUM_Q_ONE, 1 << 15);
    assert_eq!(SUM_Q_SATURATED, 65_535);
  }

  proptest! {
    #[test]
    fn every_status_byte_with_a_defined_code_survives_a_round_trip_up_to_the_reserved_bit(byte in any::<u8>()) {
      if let Some(status) = SensorStatus::from_byte(byte) {
        prop_assert_eq!(status.to_byte(), byte & !0b0100_0000);
      }
    }
  }
}
