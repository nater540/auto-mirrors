//! The records the coordinator keeps in flash (reference 3.6): the calibration, written after each one, and the last
//! good alignment, written after each success. Without them a reboot forgets the Jacobian, and the next alignment
//! pokes every axis again.
//!
//! The firmware stores them with sequential-storage, which adds a CRC and keeps only the newest record per key, as
//! the node does its positions (`node_logic::record`). This module fixes the bytes: little-endian, behind a format
//! byte so that a later layout can be told apart. Reading is strict: a stored Jacobian drives moves, so a value that
//! is not finite, a condition number below one, or a direction sign byte out of range makes the record unreadable, and
//! the coordinator calibrates again. Whether a readable calibration is still usable (its conditioning against the
//! configured limit, its direction signs against the configuration) the aligner decides, as for any calibration.

use align_core::aligner::AxisState;
use align_core::axis::{Axis, PerAxis};
use align_core::calibration::Calibration;
use align_core::config::DirSign;
use align_core::solver::Jacobian;

/// The key of the calibration record.
pub const CALIBRATION_KEY: u8 = 0;
/// The key of the last good alignment.
pub const LAST_GOOD_KEY: u8 = 1;
/// Bytes in a calibration record: the format, the Jacobian row by row, `q0`, the condition number, four signs.
pub const CALIBRATION_LEN: usize = 1 + 16 * 4 + 4 * 4 + 4 + 4;
/// Bytes in a last good record: the format, four positions, the residual.
pub const LAST_GOOD_LEN: usize = 1 + 4 * 4 + 4;

const FORMAT: u8 = 1;
const CALIBRATION: &str = "calibration";
const LAST_GOOD: &str = "last good alignment";

/// The screws' positions at the end of the last alignment that converged, and how close it came.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LastGood {
  /// Every axis's commanded position, in full steps, in the order of [`Axis::ALL`].
  pub pos_steps: [i32; 4],
  /// The largest component of the measurement vector the alignment ended with, normalized.
  pub residual_norm: f32
}

impl LastGood {
  /// The record of an alignment that ended with the axes at `axes` and the residual `residual_norm`.
  pub fn of(axes: &PerAxis<AxisState>, residual_norm: f32) -> Self {
    Self {
      pos_steps: Axis::ALL.map(|axis| axes.get(axis).pos_steps),
      residual_norm
    }
  }

  /// The stored bytes.
  pub fn to_bytes(&self) -> [u8; LAST_GOOD_LEN] {
    let mut writer = Writer::new();
    writer.put(&[FORMAT]);
    for pos_steps in self.pos_steps {
      writer.put(&pos_steps.to_le_bytes());
    }
    writer.put(&self.residual_norm.to_le_bytes());
    writer.bytes
  }

  /// A record from stored bytes.
  ///
  /// # Errors
  ///
  /// A [`StoredError`] for a wrong length, an unknown format, or a residual that is not finite or is negative.
  pub fn from_bytes(bytes: &[u8]) -> Result<Self, StoredError> {
    let mut reader = Reader::new(bytes, LAST_GOOD, LAST_GOOD_LEN)?;
    let mut pos_steps = [0_i32; 4];
    for slot in &mut pos_steps {
      *slot = i32::from_le_bytes(reader.take()?);
    }
    let residual_norm = f32::from_le_bytes(reader.take()?);
    if !residual_norm.is_finite() {
      return Err(StoredError::NotFinite {
        record: LAST_GOOD,
        field: "residual"
      });
    }
    if residual_norm < 0.0 {
      return Err(StoredError::OutOfRange {
        record: LAST_GOOD,
        field: "residual"
      });
    }
    Ok(Self {
      pos_steps,
      residual_norm
    })
  }
}

/// The stored bytes of a calibration.
pub fn calibration_to_bytes(calibration: &Calibration) -> [u8; CALIBRATION_LEN] {
  let mut writer = Writer::new();
  writer.put(&[FORMAT]);
  for row in calibration.jacobian.row_iter() {
    for entry in &row {
      writer.put(&entry.to_le_bytes());
    }
  }
  for q0 in calibration.q0_steps {
    writer.put(&q0.to_le_bytes());
  }
  writer.put(&calibration.cond.to_le_bytes());
  for axis in Axis::ALL {
    writer.put(&[match calibration.dir_sign.get(axis) {
      DirSign::Forward => 1,
      DirSign::Reversed => 2
    }]);
  }
  writer.bytes
}

/// A calibration from stored bytes.
///
/// # Errors
///
/// A [`StoredError`] for a wrong length, an unknown format, a Jacobian entry or condition number that is not finite,
/// a condition number below one, or a direction sign byte other than 1 (forward) or 2 (reversed).
pub fn calibration_from_bytes(bytes: &[u8]) -> Result<Calibration, StoredError> {
  let mut reader = Reader::new(bytes, CALIBRATION, CALIBRATION_LEN)?;
  let mut entries = [0.0_f32; 16];
  for entry in &mut entries {
    *entry = f32::from_le_bytes(reader.take()?);
  }
  if !entries.iter().all(|entry| entry.is_finite()) {
    return Err(StoredError::NotFinite {
      record: CALIBRATION,
      field: "jacobian"
    });
  }
  let mut q0_steps = [0_i32; 4];
  for slot in &mut q0_steps {
    *slot = i32::from_le_bytes(reader.take()?);
  }
  let cond = f32::from_le_bytes(reader.take()?);
  if !cond.is_finite() {
    return Err(StoredError::NotFinite {
      record: CALIBRATION,
      field: "cond"
    });
  }
  // A condition number is a ratio of the largest singular value to the smallest.
  if cond < 1.0 {
    return Err(StoredError::OutOfRange {
      record: CALIBRATION,
      field: "cond"
    });
  }
  let [m1_h, m1_v, m2_h, m2_v] = reader.take::<4>()?.map(|byte| match byte {
    1 => Ok(DirSign::Forward),
    2 => Ok(DirSign::Reversed),
    value => Err(StoredError::BadField {
      record: CALIBRATION,
      field: "dir_sign",
      value
    })
  });
  Ok(Calibration {
    jacobian: Jacobian::from_fn(|row, column| {
      entries
        .get(row.saturating_mul(4).saturating_add(column))
        .copied()
        .unwrap_or(f32::NAN)
    }),
    q0_steps,
    cond,
    dir_sign: PerAxis {
      m1_h: m1_h?,
      m1_v: m1_v?,
      m2_h: m2_h?,
      m2_v: m2_v?
    }
  })
}

/// Why stored bytes are not a record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum StoredError {
  /// The stored value has the wrong length.
  #[error("a stored {record} is {expected} bytes; this one is {len}")]
  WrongLength {
    /// Which record.
    record: &'static str,
    /// Its length in this firmware.
    expected: usize,
    /// The stored length.
    len: usize
  },
  /// The record was written in a layout this firmware does not know.
  #[error("stored {record} format {format} is unknown (this firmware reads format {FORMAT})")]
  UnknownFormat {
    /// Which record.
    record: &'static str,
    /// The stored format.
    format: u8
  },
  /// A number is NaN or infinite.
  #[error("stored {record} field {field} is not a finite number")]
  NotFinite {
    /// Which record.
    record: &'static str,
    /// Which field.
    field: &'static str
  },
  /// A number is finite but outside what its field can hold.
  #[error("stored {record} field {field} is out of its range")]
  OutOfRange {
    /// Which record.
    record: &'static str,
    /// Which field.
    field: &'static str
  },
  /// A byte is outside the values its field defines.
  #[error("stored {record} field {field} holds {value}, which it cannot")]
  BadField {
    /// Which record.
    record: &'static str,
    /// Which field.
    field: &'static str,
    /// The stored byte.
    value: u8
  }
}

/// Fills a record's bytes in order.
struct Writer<const N: usize> {
  bytes: [u8; N],
  at: usize
}

impl<const N: usize> Writer<N> {
  const fn new() -> Self {
    Self { bytes: [0; N], at: 0 }
  }

  fn put(&mut self, data: &[u8]) {
    for (slot, byte) in self.bytes.iter_mut().skip(self.at).zip(data) {
      *slot = *byte;
    }
    self.at = self.at.saturating_add(data.len());
  }
}

/// Reads a record's bytes in order, after checking its length and format.
struct Reader<'a> {
  bytes: &'a [u8],
  record: &'static str,
  at: usize
}

impl<'a> Reader<'a> {
  fn new(bytes: &'a [u8], record: &'static str, expected: usize) -> Result<Self, StoredError> {
    if bytes.len() != expected {
      return Err(StoredError::WrongLength {
        record,
        expected,
        len: bytes.len()
      });
    }
    match bytes.first() {
      Some(&FORMAT) => Ok(Self { bytes, record, at: 1 }),
      Some(&format) => Err(StoredError::UnknownFormat { record, format }),
      None => Err(StoredError::WrongLength {
        record,
        expected,
        len: 0
      })
    }
  }

  fn take<const K: usize>(&mut self) -> Result<[u8; K], StoredError> {
    let end = self.at.saturating_add(K);
    let taken = self
      .bytes
      .get(self.at..end)
      .and_then(|slice| <[u8; K]>::try_from(slice).ok());
    self.at = end;
    taken.ok_or(StoredError::WrongLength {
      record: self.record,
      expected: end,
      len: self.bytes.len()
    })
  }
}

#[cfg(test)]
mod tests {
  use align_core::approach::LastDir;
  use proptest::prelude::*;

  use super::*;

  fn calibration() -> Calibration {
    Calibration {
      #[expect(clippy::cast_precision_loss, reason = "indices below sixteen")]
      jacobian: Jacobian::from_fn(|row, column| 0.001 * (row * 4 + column + 1) as f32),
      q0_steps: [-35, -14, 4, 1],
      cond: 3.627,
      dir_sign: PerAxis {
        m1_h: DirSign::Forward,
        m1_v: DirSign::Reversed,
        m2_h: DirSign::Forward,
        m2_v: DirSign::Reversed
      }
    }
  }

  #[test]
  fn a_calibration_reads_back_as_written() {
    let bytes = calibration_to_bytes(&calibration());
    assert_eq!(calibration_from_bytes(&bytes), Ok(calibration()));
  }

  #[test]
  fn a_calibration_is_its_format_then_the_jacobian_row_by_row_then_q0_cond_and_the_signs() {
    let bytes = calibration_to_bytes(&calibration());
    assert_eq!(bytes.len(), 89);
    assert_eq!(bytes[0], 1);
    // Row 0, column 1 is the second entry; row 1, column 0 the fifth.
    assert_eq!(bytes[5..9], (0.001_f32 * 2.0).to_le_bytes());
    assert_eq!(bytes[17..21], (0.001_f32 * 5.0).to_le_bytes());
    assert_eq!(bytes[65..69], (-35_i32).to_le_bytes());
    assert_eq!(bytes[77..81], 1_i32.to_le_bytes());
    assert_eq!(bytes[81..85], 3.627_f32.to_le_bytes());
    assert_eq!(bytes[85..89], [1, 2, 1, 2]);
  }

  #[test]
  fn a_calibration_of_another_length_or_format_is_unreadable() {
    let bytes = calibration_to_bytes(&calibration());
    assert_eq!(
      calibration_from_bytes(&bytes[..88]),
      Err(StoredError::WrongLength {
        record: CALIBRATION,
        expected: 89,
        len: 88
      })
    );
    assert!(matches!(
      calibration_from_bytes(&[]),
      Err(StoredError::WrongLength { len: 0, .. })
    ));
    let mut longer = bytes.to_vec();
    longer.push(0);
    assert!(matches!(
      calibration_from_bytes(&longer),
      Err(StoredError::WrongLength { len: 90, .. })
    ));
    let mut other_format = bytes;
    other_format[0] = 2;
    assert_eq!(
      calibration_from_bytes(&other_format),
      Err(StoredError::UnknownFormat {
        record: CALIBRATION,
        format: 2
      })
    );
  }

  #[test]
  fn a_jacobian_entry_that_is_not_finite_makes_the_calibration_unreadable_wherever_it_is() {
    for entry in 0..16 {
      for value in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let mut bytes = calibration_to_bytes(&calibration());
        let at = 1 + 4 * entry;
        bytes[at..at + 4].copy_from_slice(&value.to_le_bytes());
        assert_eq!(
          calibration_from_bytes(&bytes),
          Err(StoredError::NotFinite {
            record: CALIBRATION,
            field: "jacobian"
          })
        );
      }
    }
  }

  #[test]
  fn a_condition_number_must_be_finite_and_at_least_one() {
    let with_cond = |cond: f32| {
      let mut bytes = calibration_to_bytes(&calibration());
      bytes[81..85].copy_from_slice(&cond.to_le_bytes());
      calibration_from_bytes(&bytes)
    };
    assert!(matches!(
      with_cond(f32::NAN),
      Err(StoredError::NotFinite { field: "cond", .. })
    ));
    assert!(matches!(
      with_cond(f32::INFINITY),
      Err(StoredError::NotFinite { field: "cond", .. })
    ));
    assert!(matches!(
      with_cond(0.999),
      Err(StoredError::OutOfRange { field: "cond", .. })
    ));
    assert!(matches!(
      with_cond(-3.0),
      Err(StoredError::OutOfRange { field: "cond", .. })
    ));
    assert_eq!(with_cond(1.0).map(|calibration| calibration.cond), Ok(1.0));
  }

  #[test]
  fn a_direction_sign_byte_other_than_forward_or_reversed_makes_the_calibration_unreadable() {
    for at in 85..89 {
      for value in [0, 3, 255] {
        let mut bytes = calibration_to_bytes(&calibration());
        bytes[at] = value;
        assert_eq!(
          calibration_from_bytes(&bytes),
          Err(StoredError::BadField {
            record: CALIBRATION,
            field: "dir_sign",
            value
          })
        );
      }
    }
  }

  proptest! {
    #[test]
    fn any_finite_calibration_reads_back_as_written(
      entries in prop::array::uniform16(-1.0e3_f32..1.0e3),
      q0_steps in prop::array::uniform4(any::<i32>()),
      cond in 1.0_f32..1.0e6,
      signs in prop::array::uniform4(any::<bool>())
    ) {
      let sign = |reversed: bool| if reversed { DirSign::Reversed } else { DirSign::Forward };
      let calibration = Calibration {
        jacobian: Jacobian::from_row_slice(&entries),
        q0_steps,
        cond,
        dir_sign: PerAxis { m1_h: sign(signs[0]), m1_v: sign(signs[1]), m2_h: sign(signs[2]), m2_v: sign(signs[3]) }
      };
      prop_assert_eq!(calibration_from_bytes(&calibration_to_bytes(&calibration)), Ok(calibration));
    }
  }

  #[test]
  fn a_last_good_alignment_is_its_format_the_positions_and_the_residual() {
    let last_good = LastGood {
      pos_steps: [-30, -12, 6, 3],
      residual_norm: 0.0124
    };
    let bytes = last_good.to_bytes();
    assert_eq!(bytes.len(), 21);
    assert_eq!(bytes[0], 1);
    assert_eq!(bytes[1..5], (-30_i32).to_le_bytes());
    assert_eq!(bytes[13..17], 3_i32.to_le_bytes());
    assert_eq!(bytes[17..21], 0.0124_f32.to_le_bytes());
    assert_eq!(LastGood::from_bytes(&bytes), Ok(last_good));
  }

  #[test]
  fn a_last_good_alignment_needs_its_length_its_format_and_a_finite_residual_of_zero_or_more() {
    let bytes = LastGood {
      pos_steps: [1, 2, 3, 4],
      residual_norm: 0.0
    }
    .to_bytes();
    assert!(LastGood::from_bytes(&bytes).is_ok());
    assert!(matches!(
      LastGood::from_bytes(&bytes[..20]),
      Err(StoredError::WrongLength { len: 20, .. })
    ));
    let mut other_format = bytes;
    other_format[0] = 0;
    assert!(matches!(
      LastGood::from_bytes(&other_format),
      Err(StoredError::UnknownFormat { format: 0, .. })
    ));
    for (value, error) in [
      (
        f32::NAN,
        StoredError::NotFinite {
          record: LAST_GOOD,
          field: "residual"
        }
      ),
      (
        f32::INFINITY,
        StoredError::NotFinite {
          record: LAST_GOOD,
          field: "residual"
        }
      ),
      (
        -0.001,
        StoredError::OutOfRange {
          record: LAST_GOOD,
          field: "residual"
        }
      )
    ] {
      let mut changed = bytes;
      changed[17..21].copy_from_slice(&value.to_le_bytes());
      assert_eq!(LastGood::from_bytes(&changed), Err(error));
    }
  }

  #[test]
  fn the_last_good_alignment_takes_each_axis_s_position_in_axis_order() {
    let state = |pos_steps: i32| AxisState {
      pos_steps,
      last_dir: LastDir::Plus,
      suspect: false,
      lost: false
    };
    let axes = PerAxis {
      m1_h: state(-30),
      m1_v: state(-12),
      m2_h: state(6),
      m2_v: state(3)
    };
    assert_eq!(
      LastGood::of(&axes, 0.0124),
      LastGood {
        pos_steps: [-30, -12, 6, 3],
        residual_norm: 0.0124
      }
    );
  }

  #[test]
  fn the_records_have_their_own_keys() {
    assert_ne!(CALIBRATION_KEY, LAST_GOOD_KEY);
  }
}
