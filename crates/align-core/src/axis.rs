//! The four motorized adjuster screws and their fixed order.
//!
//! The order is `q = [M1_H, M1_V, M2_H, M2_V]` everywhere: in the solver's vectors, on the bus, and in logs.
//! Mirror 1 is frame mounted; mirror 2 rides the Y gantry. The third screw on each mount is a manual pivot.

/// Number of motorized axes.
pub const AXIS_COUNT: usize = 4;

/// One motorized adjuster screw.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Axis {
  /// Mirror 1, the screw that steers the beam horizontally.
  M1H,
  /// Mirror 1, the screw that steers the beam vertically.
  M1V,
  /// Mirror 2, the screw that steers the beam horizontally.
  M2H,
  /// Mirror 2, the screw that steers the beam vertically.
  M2V,
}

impl Axis {
  /// Every axis, in the fixed order.
  pub const ALL: [Axis; AXIS_COUNT] = [Axis::M1H, Axis::M1V, Axis::M2H, Axis::M2V];

  /// Position of this axis in `q`.
  pub const fn index(self) -> usize {
    match self {
      Axis::M1H => 0,
      Axis::M1V => 1,
      Axis::M2H => 2,
      Axis::M2V => 3,
    }
  }
}

/// One value for each axis, addressed by name so that no lookup can go out of range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PerAxis<T> {
  /// Value for [`Axis::M1H`].
  pub m1_h: T,
  /// Value for [`Axis::M1V`].
  pub m1_v: T,
  /// Value for [`Axis::M2H`].
  pub m2_h: T,
  /// Value for [`Axis::M2V`].
  pub m2_v: T,
}

impl<T: Copy> PerAxis<T> {
  /// The same value on every axis.
  pub const fn splat(value: T) -> Self {
    Self {
      m1_h: value,
      m1_v: value,
      m2_h: value,
      m2_v: value,
    }
  }

  /// The value for one axis.
  pub const fn get(&self, axis: Axis) -> T {
    match axis {
      Axis::M1H => self.m1_h,
      Axis::M1V => self.m1_v,
      Axis::M2H => self.m2_h,
      Axis::M2V => self.m2_v,
    }
  }

  /// The values in the fixed axis order.
  pub const fn to_array(&self) -> [T; AXIS_COUNT] {
    [self.m1_h, self.m1_v, self.m2_h, self.m2_v]
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn the_axis_order_is_m1_h_m1_v_m2_h_m2_v() {
    for (position, axis) in Axis::ALL.into_iter().enumerate() {
      assert_eq!(axis.index(), position);
    }
    assert_eq!(Axis::ALL, [Axis::M1H, Axis::M1V, Axis::M2H, Axis::M2V]);
  }

  #[test]
  fn a_per_axis_value_is_read_back_by_axis_and_in_order() {
    let values = PerAxis {
      m1_h: 10,
      m1_v: 20,
      m2_h: 30,
      m2_v: 40,
    };
    assert_eq!(values.to_array(), [10, 20, 30, 40]);
    for axis in Axis::ALL {
      assert_eq!(values.get(axis), values.to_array()[axis.index()]);
    }
  }

  #[test]
  fn splat_puts_one_value_on_every_axis() {
    assert_eq!(PerAxis::splat(7).to_array(), [7; AXIS_COUNT]);
  }
}
