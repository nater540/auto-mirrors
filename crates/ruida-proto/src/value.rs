//! The controller's 35-bit values (reference 6.3), the machine status bits, and what each query's answer reads as.
//!
//! A value is five data bytes of seven bits each, most significant first. MeerK40t (`rdjob.py`) reads the card ID
//! unsigned and the status and positions signed (`decode35`); reference 6.3's `dec35` folds into a `u32`, which
//! drops the top three bits, so the full 35 are kept here.

use core::fmt;

use crate::packet::Query;

/// The largest unsigned 35-bit value.
const U35_MAX: u64 = (1 << 35) - 1;
/// The bits a data byte carries.
const DATA_BITS: u8 = 0x7F;

/// A value read unsigned: the card ID, the mainboard version, the status word.
pub fn decode_u35(bytes: [u8; 5]) -> u64 {
  bytes
    .iter()
    .fold(0, |value, &byte| (value << 7) | u64::from(byte & DATA_BITS))
}

/// A value read as a signed 35-bit number (MeerK40t `signed35`): the positions.
pub fn decode_i35(bytes: [u8; 5]) -> i64 {
  let value = bytes
    .iter()
    .fold(0_i64, |value, &byte| (value << 7) | i64::from(byte & DATA_BITS));
  if value >= 1 << 34 { value - (1 << 35) } else { value }
}

/// The five data bytes that carry `value`, or `None` above 35 bits. The simulated controller answers with these.
pub fn encode_u35(value: u64) -> Option<[u8; 5]> {
  if value > U35_MAX {
    return None;
  }
  let byte = |shift: u32| u8::try_from((value >> shift) & u64::from(DATA_BITS)).ok();
  Some([byte(28)?, byte(21)?, byte(14)?, byte(7)?, byte(0)?])
}

/// The five data bytes that carry a signed `value`, or `None` outside 35 bits.
pub fn encode_i35(value: i64) -> Option<[u8; 5]> {
  if !(-(1 << 34)..1 << 34).contains(&value) {
    return None;
  }
  encode_u35(u64::try_from(value.rem_euclid(1 << 35)).ok()?)
}

/// The machine status word of query `04 00`, with the bits MeerK40t has "identified so far" (reference 6.4).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MachineStatus(u64);

impl MachineStatus {
  /// VERIFIED (reference 6.4, MeerK40t `rdjob.py`): an axis is moving.
  pub const MOVING: u64 = 0x0100_0000;
  /// VERIFIED (same): the part has ended.
  pub const PART_END: u64 = 0x02;
  /// VERIFIED (same): a job is running.
  pub const JOB_RUNNING: u64 = 0x01;

  /// The word as read.
  pub const fn new(raw: u64) -> Self {
    Self(raw)
  }

  /// The whole word, the bits not yet identified included.
  pub const fn raw(self) -> u64 {
    self.0
  }

  /// Whether an axis is moving.
  pub const fn moving(self) -> bool {
    self.0 & Self::MOVING != 0
  }

  /// Whether the part has ended.
  pub const fn part_end(self) -> bool {
    self.0 & Self::PART_END != 0
  }

  /// Whether a job is running.
  pub const fn job_running(self) -> bool {
    self.0 & Self::JOB_RUNNING != 0
  }
}

/// What one query's answer reads as.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reading {
  /// The card ID, unsigned.
  CardId {
    /// The ID. Reference 6.0: the sources disagree on what an RDC644X reports, so it is never compared.
    id: u64
  },
  /// The mainboard version, unsigned. No source says how it is encoded.
  MainboardVersion {
    /// The value as read.
    raw: u64
  },
  /// The machine status word.
  Status(MachineStatus),
  /// The current X position.
  PositionX {
    /// In micrometers (reference 6.3), as the controller counts them. Reference 6.5 step 4 checks the unit and sign
    /// against a jog from the panel.
    x_um: i64
  },
  /// The current Y position.
  PositionY {
    /// In micrometers, as for X.
    y_um: i64
  }
}

impl Reading {
  /// The reading of `query`'s answer value.
  pub fn of(query: Query, value: [u8; 5]) -> Self {
    match query {
      Query::CardId => Reading::CardId { id: decode_u35(value) },
      Query::MainboardVersion => Reading::MainboardVersion { raw: decode_u35(value) },
      Query::Status => Reading::Status(MachineStatus::new(decode_u35(value))),
      Query::PositionX => Reading::PositionX {
        x_um: decode_i35(value)
      },
      Query::PositionY => Reading::PositionY {
        y_um: decode_i35(value)
      }
    }
  }
}

/// `yes` or `no`, as the log writes a flag.
const fn yes_no(flag: bool) -> &'static str {
  if flag { "yes" } else { "no" }
}

/// The reading as `key=value` fields for a log line.
impl fmt::Display for Reading {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match *self {
      Reading::CardId { id } => write!(f, "card_id={id:#010x}"),
      Reading::MainboardVersion { raw } => write!(f, "mainboard_version={raw:#010x}"),
      Reading::Status(status) => write!(
        f,
        "status={:#010x},moving={},part_end={},job_running={}",
        status.raw(),
        yes_no(status.moving()),
        yes_no(status.part_end()),
        yes_no(status.job_running())
      ),
      Reading::PositionX { x_um } => write!(f, "x_um={x_um}"),
      Reading::PositionY { y_um } => write!(f, "y_um={y_um}")
    }
  }
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::format;

  use proptest::prelude::*;

  use super::*;

  #[test]
  fn a_value_is_seven_bits_a_byte_most_significant_first() {
    assert_eq!(decode_u35([0, 0, 0, 0, 1]), 1);
    assert_eq!(decode_u35([0, 0, 0, 1, 0]), 1 << 7);
    assert_eq!(decode_u35([1, 0, 0, 0, 0]), 1 << 28);
    assert_eq!(decode_u35([0x7F; 5]), U35_MAX);
    // The card ID the EduTech Wiki gives, 0x65006500, as five data bytes.
    assert_eq!(decode_u35([0x06, 0x28, 0x01, 0x4A, 0x00]), 0x6500_6500);
  }

  #[test]
  fn the_eighth_bit_of_a_byte_is_not_part_of_the_value() {
    assert_eq!(decode_u35([0x80, 0x80, 0x80, 0x80, 0x81]), 1);
    assert_eq!(decode_i35([0xFF, 0xFF, 0xFF, 0xFF, 0xFF]), -1);
  }

  #[test]
  fn a_position_with_the_35th_bit_set_is_negative() {
    assert_eq!(decode_i35([0x7F; 5]), -1);
    assert_eq!(decode_i35([0x40, 0, 0, 0, 0]), -(1 << 34));
    assert_eq!(decode_i35([0x3F, 0x7F, 0x7F, 0x7F, 0x7F]), (1 << 34) - 1);
    assert_eq!(decode_i35([0, 0, 0, 0x07, 0x68]), 1000);
  }

  #[test]
  fn a_value_beyond_35_bits_cannot_be_encoded() {
    assert_eq!(encode_u35(U35_MAX), Some([0x7F; 5]));
    assert_eq!(encode_u35(U35_MAX + 1), None);
    assert_eq!(encode_i35((1 << 34) - 1), Some([0x3F, 0x7F, 0x7F, 0x7F, 0x7F]));
    assert_eq!(encode_i35(1 << 34), None);
    assert_eq!(encode_i35(-(1 << 34)), Some([0x40, 0, 0, 0, 0]));
    assert_eq!(encode_i35(-(1 << 34) - 1), None);
    assert_eq!(encode_i35(-1), Some([0x7F; 5]));
  }

  proptest! {
    #[test]
    fn an_unsigned_value_reads_back_as_written(value in 0..=U35_MAX) {
      let bytes = encode_u35(value).unwrap();
      prop_assert!(bytes.iter().all(|&byte| byte <= DATA_BITS));
      prop_assert_eq!(decode_u35(bytes), value);
    }

    #[test]
    fn a_signed_value_reads_back_as_written(value in -(1_i64 << 34)..1 << 34) {
      let bytes = encode_i35(value).unwrap();
      prop_assert!(bytes.iter().all(|&byte| byte <= DATA_BITS));
      prop_assert_eq!(decode_i35(bytes), value);
    }
  }

  #[test]
  fn the_status_bits_are_meerk40t_s() {
    let idle = MachineStatus::new(0);
    assert!(!idle.moving() && !idle.part_end() && !idle.job_running());
    let all = MachineStatus::new(0x0100_0003);
    assert!(all.moving() && all.part_end() && all.job_running());
    assert!(MachineStatus::new(0x0100_0000).moving());
    assert!(!MachineStatus::new(0x00FF_FFFC).moving());
    assert!(MachineStatus::new(0x02).part_end());
    assert!(!MachineStatus::new(0x01).part_end());
    assert!(MachineStatus::new(0x01).job_running());
    assert!(!MachineStatus::new(0x02).job_running());
    assert_eq!(all.raw(), 0x0100_0003);
  }

  #[test]
  fn each_query_reads_its_value_its_own_way() {
    let minus_one = [0x7F; 5];
    assert_eq!(Reading::of(Query::CardId, minus_one), Reading::CardId { id: U35_MAX });
    assert_eq!(
      Reading::of(Query::MainboardVersion, minus_one),
      Reading::MainboardVersion { raw: U35_MAX }
    );
    assert_eq!(
      Reading::of(Query::Status, minus_one),
      Reading::Status(MachineStatus::new(U35_MAX))
    );
    assert_eq!(
      Reading::of(Query::PositionX, minus_one),
      Reading::PositionX { x_um: -1 }
    );
    assert_eq!(
      Reading::of(Query::PositionY, minus_one),
      Reading::PositionY { y_um: -1 }
    );
  }

  #[test]
  fn a_reading_is_written_as_key_value_fields() {
    assert_eq!(format!("{}", Reading::CardId { id: 0x6500_6500 }), "card_id=0x65006500");
    assert_eq!(
      format!("{}", Reading::MainboardVersion { raw: 0x12 }),
      "mainboard_version=0x00000012"
    );
    assert_eq!(
      format!("{}", Reading::Status(MachineStatus::new(0x0100_0001))),
      "status=0x01000001,moving=yes,part_end=no,job_running=yes"
    );
    assert_eq!(
      format!("{}", Reading::Status(MachineStatus::new(0x02))),
      "status=0x00000002,moving=no,part_end=yes,job_running=no"
    );
    assert_eq!(format!("{}", Reading::PositionX { x_um: -1250 }), "x_um=-1250");
    assert_eq!(format!("{}", Reading::PositionY { y_um: 300_000 }), "y_um=300000");
  }
}
