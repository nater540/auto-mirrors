//! The position record a node keeps in flash for each motor (reference 3.6, with the fields addendum A.1 adds).
//!
//! It is written after every move, only while the motors are stopped (hand-off 3.3), and read once at boot.
//! The firmware stores it with sequential-storage, which adds a CRC and keeps only the newest record per key.
//! This module fixes the bytes, little-endian, behind a format version so that a later layout can be told apart.
//! Format 2 adds the direction sign the position is expressed in.
//! A format 1 record still reads, with that sign unknown.

use align_core::approach::LastDir;
use align_core::config::DirSign;
use stepper::Phase;

/// Bytes in one encoded record (format 2).
pub const RECORD_LEN: usize = 9;

const FORMAT_VERSION: u8 = 2;
const FORMAT_1: u8 = 1;
const FORMAT_1_LEN: usize = 8;

/// What a node remembers about one motor across a reboot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisRecord {
  /// The commanded position, in full steps, in the plus-is-advancing sense of `dir_sign`.
  pub pos_steps: i32,
  /// The direction of the last completed move, for the standard approach.
  pub last_dir: LastDir,
  /// The phase the coils were left in (addendum A.4).
  pub phase: Phase,
  /// The position is not trusted: since it was last known, there was a rail loss, a pin fault,
  /// a cut-off move, or a reset during a move.
  pub suspect: bool,
  /// The direction sign `pos_steps` and `last_dir` are expressed in. A position means nothing without it: the same
  /// count is a screw on the other side of zero under the other sign. `None` in a fresh or format 1 record.
  pub dir_sign: Option<DirSign>
}

/// Why stored bytes are not a record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RecordError {
  /// The stored value has the wrong length for its format.
  #[error("a position record is {RECORD_LEN} bytes (format 1: {FORMAT_1_LEN}); the stored one is {len}")]
  WrongLength {
    /// The stored length.
    len: usize
  },
  /// The record was written by a layout this firmware does not know.
  #[error("position record format {version} is unknown (this firmware reads formats 1 and {FORMAT_VERSION})")]
  UnknownVersion {
    /// The stored format version.
    version: u8
  },
  /// A byte is outside the values its field defines.
  #[error("position record field {field} holds {value}, which it cannot")]
  BadField {
    /// The field's name.
    field: &'static str,
    /// The stored byte.
    value: u8
  }
}

impl AxisRecord {
  /// The record of a motor never moved: position zero, phase zero, direction unknown, trusted, no sign yet.
  pub const FRESH: Self = Self {
    pos_steps: 0,
    last_dir: LastDir::Unknown,
    phase: Phase::ZERO,
    suspect: false,
    dir_sign: None
  };

  /// The stored bytes, in format 2.
  pub fn to_bytes(&self) -> [u8; RECORD_LEN] {
    let [p0, p1, p2, p3] = self.pos_steps.to_le_bytes();
    let last_dir = match self.last_dir {
      LastDir::Unknown => 0,
      LastDir::Plus => 1,
      LastDir::Minus => 2
    };
    let dir_sign = match self.dir_sign {
      None => 0,
      Some(DirSign::Forward) => 1,
      Some(DirSign::Reversed) => 2
    };
    [
      FORMAT_VERSION,
      p0,
      p1,
      p2,
      p3,
      last_dir,
      self.phase.index(),
      u8::from(self.suspect),
      dir_sign
    ]
  }

  /// A record from stored bytes, in format 2 or format 1.
  ///
  /// # Errors
  ///
  /// A [`RecordError`] for an unknown format version, a length that does not fit the format,
  /// or a field byte out of range.
  pub fn from_bytes(bytes: &[u8]) -> Result<Self, RecordError> {
    let wrong_length = RecordError::WrongLength { len: bytes.len() };
    let (head, sign) = match *bytes {
      [FORMAT_VERSION, p0, p1, p2, p3, last_dir, phase, suspect, sign] => {
        ([p0, p1, p2, p3, last_dir, phase, suspect], sign)
      }
      [FORMAT_1, p0, p1, p2, p3, last_dir, phase, suspect] => ([p0, p1, p2, p3, last_dir, phase, suspect], 0),
      [FORMAT_VERSION | FORMAT_1, ..] | [] => return Err(wrong_length),
      [version, ..] => return Err(RecordError::UnknownVersion { version })
    };
    let [p0, p1, p2, p3, last_dir, phase, suspect] = head;
    let last_dir = match last_dir {
      0 => LastDir::Unknown,
      1 => LastDir::Plus,
      2 => LastDir::Minus,
      value => {
        return Err(RecordError::BadField {
          field: "last_dir",
          value
        });
      }
    };
    let phase = Phase::new(phase).ok_or(RecordError::BadField {
      field: "phase",
      value: phase
    })?;
    let suspect = match suspect {
      0 => false,
      1 => true,
      value => {
        return Err(RecordError::BadField {
          field: "suspect",
          value
        });
      }
    };
    let dir_sign = match sign {
      0 => None,
      1 => Some(DirSign::Forward),
      2 => Some(DirSign::Reversed),
      value => {
        return Err(RecordError::BadField {
          field: "dir_sign",
          value
        });
      }
    };
    Ok(Self {
      pos_steps: i32::from_le_bytes([p0, p1, p2, p3]),
      last_dir,
      phase,
      suspect,
      dir_sign
    })
  }

  /// The same physical state expressed under the other direction sign: the position negated and the last direction
  /// swapped. A position counted under one sign is a screw on the other side of zero under the other.
  #[must_use]
  pub const fn flipped(self, dir_sign: DirSign) -> Self {
    let last_dir = match self.last_dir {
      LastDir::Plus => LastDir::Minus,
      LastDir::Minus => LastDir::Plus,
      LastDir::Unknown => LastDir::Unknown
    };
    Self {
      pos_steps: self.pos_steps.saturating_neg(),
      last_dir,
      dir_sign: Some(dir_sign),
      ..self
    }
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;

  #[test]
  fn the_layout_is_version_position_direction_phase_suspect_sign() {
    let record = AxisRecord {
      pos_steps: -2,
      last_dir: LastDir::Minus,
      phase: Phase::new(3).unwrap(),
      suspect: true,
      dir_sign: Some(DirSign::Reversed)
    };
    assert_eq!(record.to_bytes(), [2, 0xFE, 0xFF, 0xFF, 0xFF, 2, 3, 1, 2]);
  }

  #[test]
  fn a_format_1_record_reads_with_an_unknown_sign() {
    let record = AxisRecord::from_bytes(&[1, 0xFE, 0xFF, 0xFF, 0xFF, 2, 3, 1]).unwrap();
    let expected = AxisRecord {
      pos_steps: -2,
      last_dir: LastDir::Minus,
      phase: Phase::new(3).unwrap(),
      suspect: true,
      dir_sign: None
    };
    assert_eq!(record, expected);
  }

  #[test]
  fn a_record_of_the_wrong_length_for_its_format_is_rejected() {
    assert_eq!(
      AxisRecord::from_bytes(&[2; 8]),
      Err(RecordError::WrongLength { len: 8 })
    );
    assert_eq!(
      AxisRecord::from_bytes(&[2; 10]),
      Err(RecordError::WrongLength { len: 10 })
    );
    assert_eq!(
      AxisRecord::from_bytes(&[1; 9]),
      Err(RecordError::WrongLength { len: 9 })
    );
    assert_eq!(AxisRecord::from_bytes(&[]), Err(RecordError::WrongLength { len: 0 }));
  }

  #[test]
  fn a_record_of_an_unknown_format_is_rejected() {
    let mut bytes = AxisRecord::FRESH.to_bytes();
    bytes[0] = 3;
    assert_eq!(
      AxisRecord::from_bytes(&bytes),
      Err(RecordError::UnknownVersion { version: 3 })
    );
    assert_eq!(
      AxisRecord::from_bytes(&[0; 9]),
      Err(RecordError::UnknownVersion { version: 0 })
    );
  }

  #[test]
  fn out_of_range_fields_are_rejected() {
    let fresh = AxisRecord::FRESH.to_bytes();
    for (index, field, value) in [
      (5, "last_dir", 3),
      (6, "phase", 4),
      (7, "suspect", 2),
      (8, "dir_sign", 3)
    ] {
      let mut bytes = fresh;
      bytes[index] = value;
      assert_eq!(
        AxisRecord::from_bytes(&bytes),
        Err(RecordError::BadField { field, value })
      );
    }
  }

  #[test]
  fn flipping_the_sign_negates_the_position_and_swaps_the_direction() {
    let record = AxisRecord {
      pos_steps: 3_900,
      last_dir: LastDir::Plus,
      phase: Phase::new(1).unwrap(),
      suspect: false,
      dir_sign: Some(DirSign::Forward)
    };
    let flipped = record.flipped(DirSign::Reversed);
    assert_eq!(
      (flipped.pos_steps, flipped.last_dir, flipped.dir_sign),
      (-3_900, LastDir::Minus, Some(DirSign::Reversed))
    );
    assert_eq!(flipped.phase, record.phase, "the coils are where they were");
    assert_eq!(
      AxisRecord {
        last_dir: LastDir::Minus,
        ..record
      }
      .flipped(DirSign::Reversed)
      .last_dir,
      LastDir::Plus
    );
    assert_eq!(AxisRecord::FRESH.flipped(DirSign::Forward).last_dir, LastDir::Unknown);
  }

  proptest! {
    #[test]
    fn every_record_survives_a_round_trip(
      pos_steps in any::<i32>(), dir in 0_u8..3, phase in 0_u8..4, suspect in any::<bool>(), sign in 0_u8..3
    ) {
      let last_dir = [LastDir::Unknown, LastDir::Plus, LastDir::Minus][usize::from(dir)];
      let dir_sign = [None, Some(DirSign::Forward), Some(DirSign::Reversed)][usize::from(sign)];
      let record = AxisRecord { pos_steps, last_dir, phase: Phase::new(phase).unwrap(), suspect, dir_sign };
      prop_assert_eq!(AxisRecord::from_bytes(&record.to_bytes()), Ok(record));
    }

    #[test]
    fn decoding_any_bytes_never_panics(bytes in proptest::collection::vec(any::<u8>(), 0..12)) {
      let _ = AxisRecord::from_bytes(&bytes);
    }
  }
}
