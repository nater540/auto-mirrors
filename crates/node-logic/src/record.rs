//! The position record a node keeps in flash for each motor (reference 3.6, with the fields addendum A.1 adds).
//!
//! It is written after every move, only while the motors are stopped (hand-off 3.3), and read once at boot.
//! The firmware stores it with sequential-storage, which adds a CRC and keeps only the newest record per key.
//! This module fixes the bytes, little-endian, behind a format version so that a later layout can be told apart.

use align_core::approach::LastDir;
use stepper::Phase;

/// Bytes in one encoded record.
pub const RECORD_LEN: usize = 8;

const FORMAT_VERSION: u8 = 1;

/// What a node remembers about one motor across a reboot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AxisRecord {
  /// The commanded position, in full steps, in the plus-is-advancing sense.
  pub pos_steps: i32,
  /// The direction of the last completed move, for the standard approach.
  pub last_dir: LastDir,
  /// The phase the coils were left in (addendum A.4).
  pub phase: Phase,
  /// The position is not trusted: a rail loss, a pin fault, or a cut-off move happened since it was last known.
  pub suspect: bool
}

/// Why stored bytes are not a record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum RecordError {
  /// The stored value has the wrong length.
  #[error("a position record is {RECORD_LEN} bytes; the stored one is {len}")]
  WrongLength {
    /// The stored length.
    len: usize
  },
  /// The record was written by a layout this firmware does not know.
  #[error("position record format {version} is unknown (this firmware writes format {FORMAT_VERSION})")]
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
  /// The record of a motor never moved: position zero, phase zero, direction unknown, trusted.
  pub const FRESH: Self = Self {
    pos_steps: 0,
    last_dir: LastDir::Unknown,
    phase: Phase::ZERO,
    suspect: false
  };

  /// The stored bytes.
  pub fn to_bytes(&self) -> [u8; RECORD_LEN] {
    let [p0, p1, p2, p3] = self.pos_steps.to_le_bytes();
    let last_dir = match self.last_dir {
      LastDir::Unknown => 0,
      LastDir::Plus => 1,
      LastDir::Minus => 2
    };
    [
      FORMAT_VERSION,
      p0,
      p1,
      p2,
      p3,
      last_dir,
      self.phase.index(),
      u8::from(self.suspect)
    ]
  }

  /// A record from stored bytes.
  ///
  /// # Errors
  ///
  /// A [`RecordError`] for a wrong length, an unknown format version, or a field byte out of range.
  pub fn from_bytes(bytes: &[u8]) -> Result<Self, RecordError> {
    let &[version, p0, p1, p2, p3, last_dir, phase, suspect] = bytes else {
      return Err(RecordError::WrongLength { len: bytes.len() });
    };
    if version != FORMAT_VERSION {
      return Err(RecordError::UnknownVersion { version });
    }
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
    Ok(Self {
      pos_steps: i32::from_le_bytes([p0, p1, p2, p3]),
      last_dir,
      phase,
      suspect
    })
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;

  #[test]
  fn the_layout_is_version_position_direction_phase_suspect() {
    let record = AxisRecord {
      pos_steps: -2,
      last_dir: LastDir::Minus,
      phase: Phase::new(3).unwrap(),
      suspect: true
    };
    assert_eq!(record.to_bytes(), [1, 0xFE, 0xFF, 0xFF, 0xFF, 2, 3, 1]);
  }

  #[test]
  fn a_record_of_the_wrong_length_is_rejected() {
    assert_eq!(
      AxisRecord::from_bytes(&[1; 7]),
      Err(RecordError::WrongLength { len: 7 })
    );
    assert_eq!(
      AxisRecord::from_bytes(&[1; 9]),
      Err(RecordError::WrongLength { len: 9 })
    );
  }

  #[test]
  fn a_record_of_an_unknown_format_is_rejected() {
    let mut bytes = AxisRecord::FRESH.to_bytes();
    bytes[0] = 2;
    assert_eq!(
      AxisRecord::from_bytes(&bytes),
      Err(RecordError::UnknownVersion { version: 2 })
    );
  }

  #[test]
  fn out_of_range_fields_are_rejected() {
    let fresh = AxisRecord::FRESH.to_bytes();
    for (index, field, value) in [(5, "last_dir", 3), (6, "phase", 4), (7, "suspect", 2)] {
      let mut bytes = fresh;
      bytes[index] = value;
      assert_eq!(
        AxisRecord::from_bytes(&bytes),
        Err(RecordError::BadField { field, value })
      );
    }
  }

  proptest! {
    #[test]
    fn every_record_survives_a_round_trip(
      pos_steps in any::<i32>(), dir in 0_u8..3, phase in 0_u8..4, suspect in any::<bool>(),
    ) {
      let last_dir = [LastDir::Unknown, LastDir::Plus, LastDir::Minus][usize::from(dir)];
      let record = AxisRecord { pos_steps, last_dir, phase: Phase::new(phase).unwrap(), suspect };
      prop_assert_eq!(AxisRecord::from_bytes(&record.to_bytes()), Ok(record));
    }

    #[test]
    fn decoding_any_bytes_never_panics(bytes in proptest::collection::vec(any::<u8>(), 0..12)) {
      let _ = AxisRecord::from_bytes(&bytes);
    }
  }
}
