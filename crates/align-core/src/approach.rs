//! The standard approach and the soft limits (hand-off 3.2, reference 3.4, addendum A.5).
//!
//! Every move ends in the plus direction, the screw advancing against the springs, so that gearbox and thread
//! backlash are always taken up on the same side. A move that would end going the other way first overshoots by
//! `overshoot_steps` and then comes back:
//!
//! ```text
//! fn approach(axis, target):
//!   if last_dir == Plus and target >= pos: move_to(target)
//!   else: move_to(target - OVERSHOOT); move_to(target)
//! ```
//!
//! The soft limits are a runaway guard in commanded steps. The steppers have no feedback, so they are not a
//! position truth, and they are widened by a margin for lost steps.

use crate::axis::Axis;
use crate::config::Motion;

/// The direction an axis last moved in.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum LastDir {
  /// Not known: after boot without a stored record, or after a failed move.
  #[default]
  Unknown,
  /// The plus direction: backlash is taken up.
  Plus,
  /// The minus direction.
  Minus,
}

/// The legs of one approach. Either may be zero.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Approach {
  /// First leg: zero, or a negative move past the target.
  pub reverse_steps: i32,
  /// Second leg: zero, or a positive move onto the target.
  pub forward_steps: i32,
}

impl Approach {
  /// The legs in the order they are executed.
  pub const fn legs(&self) -> [i32; 2] {
    [self.reverse_steps, self.forward_steps]
  }
}

/// Plans a relative move of `delta_steps` that ends in the plus direction.
///
/// A plus move from an axis that last moved plus is made directly. So is a plus move of at least one overshoot,
/// whatever came before: that much plus travel takes up the backlash by itself. Everything else goes past the
/// target by the overshoot and comes back. A request for no move on an axis whose last direction is not plus
/// still runs the overshoot and return, which is how the standard direction is established.
pub fn plan(delta_steps: i32, last_dir: LastDir, overshoot_steps: i32) -> Approach {
  let overshoot = overshoot_steps.max(0);
  let direct = Approach {
    reverse_steps: 0,
    forward_steps: delta_steps,
  };
  if last_dir == LastDir::Plus && delta_steps >= 0 {
    return direct;
  }
  let reverse = delta_steps.saturating_sub(overshoot);
  if reverse >= 0 {
    return direct;
  }
  Approach {
    reverse_steps: reverse,
    forward_steps: overshoot,
  }
}

/// The commanded range of one axis, margin included.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SoftLimits {
  /// Lowest allowed commanded position, in steps.
  pub min_steps: i32,
  /// Highest allowed commanded position, in steps.
  pub max_steps: i32,
}

impl SoftLimits {
  /// The limits of one axis, widened on both sides by `soft_limit_margin_frac` of the configured travel (addendum A.5).
  pub fn for_axis(motion: &Motion, axis: Axis) -> Self {
    let min = f64::from(motion.soft_min_steps.get(axis));
    let max = f64::from(motion.soft_max_steps.get(axis));
    let margin = libm::round((max - min).max(0.0) * f64::from(motion.soft_limit_margin_frac).max(0.0));
    Self {
      min_steps: saturate(min - margin),
      max_steps: saturate(max + margin),
    }
  }

  /// Whether a commanded position is inside the limits.
  pub fn contains(&self, pos_steps: i64) -> bool {
    i64::from(self.min_steps) <= pos_steps && pos_steps <= i64::from(self.max_steps)
  }
}

/// Converts to `i32`, saturating at the ends of its range. A NaN becomes zero.
fn saturate(value: f64) -> i32 {
  #[expect(
    clippy::cast_possible_truncation,
    reason = "a float to int `as` cast saturates, which is the intent"
  )]
  let saturated = value as i32;
  saturated
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;
  use crate::axis::PerAxis;

  const OVERSHOOT: i32 = 120;

  #[test]
  fn a_plus_move_after_a_plus_move_is_made_directly() {
    assert_eq!(
      plan(30, LastDir::Plus, OVERSHOOT),
      Approach {
        reverse_steps: 0,
        forward_steps: 30
      }
    );
  }

  #[test]
  fn a_minus_move_overshoots_and_comes_back_in_the_plus_direction() {
    assert_eq!(
      plan(-50, LastDir::Plus, OVERSHOOT),
      Approach {
        reverse_steps: -170,
        forward_steps: 120
      }
    );
  }

  #[test]
  fn a_short_plus_move_after_a_minus_move_backs_off_first() {
    assert_eq!(
      plan(30, LastDir::Minus, OVERSHOOT),
      Approach {
        reverse_steps: -90,
        forward_steps: 120
      }
    );
    assert_eq!(
      plan(30, LastDir::Unknown, OVERSHOOT),
      Approach {
        reverse_steps: -90,
        forward_steps: 120
      }
    );
  }

  #[test]
  fn a_plus_move_of_at_least_one_overshoot_is_made_directly_whatever_came_before() {
    assert_eq!(
      plan(120, LastDir::Minus, OVERSHOOT),
      Approach {
        reverse_steps: 0,
        forward_steps: 120
      }
    );
    assert_eq!(
      plan(500, LastDir::Unknown, OVERSHOOT),
      Approach {
        reverse_steps: 0,
        forward_steps: 500
      }
    );
  }

  #[test]
  fn no_move_is_needed_to_stay_put_after_a_plus_move() {
    assert_eq!(plan(0, LastDir::Plus, OVERSHOOT).legs(), [0, 0]);
  }

  #[test]
  fn staying_put_with_an_unknown_direction_establishes_the_standard_direction() {
    assert_eq!(
      plan(0, LastDir::Unknown, OVERSHOOT),
      Approach {
        reverse_steps: -120,
        forward_steps: 120
      }
    );
    assert_eq!(
      plan(0, LastDir::Minus, OVERSHOOT),
      Approach {
        reverse_steps: -120,
        forward_steps: 120
      }
    );
  }

  #[test]
  fn a_negative_overshoot_is_read_as_none() {
    assert_eq!(
      plan(-50, LastDir::Plus, -10),
      Approach {
        reverse_steps: -50,
        forward_steps: 0
      }
    );
  }

  #[test]
  fn the_soft_limits_are_widened_by_ten_percent_of_travel() {
    let motion = Motion::STARTING;
    let limits = SoftLimits::for_axis(&motion, Axis::M2H);
    assert_eq!(
      limits,
      SoftLimits {
        min_steps: -4_800,
        max_steps: 4_800
      }
    );
    assert!(limits.contains(-4_800));
    assert!(limits.contains(4_800));
    assert!(!limits.contains(-4_801));
    assert!(!limits.contains(4_801));
  }

  #[test]
  fn each_axis_has_its_own_limits() {
    let mut motion = Motion::STARTING;
    motion.soft_min_steps = PerAxis {
      m1_h: -100,
      m1_v: -200,
      m2_h: 0,
      m2_v: -1,
    };
    motion.soft_max_steps = PerAxis {
      m1_h: 100,
      m1_v: 300,
      m2_h: 50,
      m2_v: 1,
    };
    motion.soft_limit_margin_frac = 0.0;
    assert_eq!(
      SoftLimits::for_axis(&motion, Axis::M1H),
      SoftLimits {
        min_steps: -100,
        max_steps: 100
      }
    );
    assert_eq!(
      SoftLimits::for_axis(&motion, Axis::M1V),
      SoftLimits {
        min_steps: -200,
        max_steps: 300
      }
    );
    assert_eq!(
      SoftLimits::for_axis(&motion, Axis::M2H),
      SoftLimits {
        min_steps: 0,
        max_steps: 50
      }
    );
    assert_eq!(
      SoftLimits::for_axis(&motion, Axis::M2V),
      SoftLimits {
        min_steps: -1,
        max_steps: 1
      }
    );
  }

  #[test]
  fn limits_at_the_ends_of_the_integer_range_saturate_and_a_bad_margin_adds_nothing() {
    let mut motion = Motion::STARTING;
    motion.soft_min_steps = PerAxis::splat(i32::MIN);
    motion.soft_max_steps = PerAxis::splat(i32::MAX);
    assert_eq!(
      SoftLimits::for_axis(&motion, Axis::M1H),
      SoftLimits {
        min_steps: i32::MIN,
        max_steps: i32::MAX
      }
    );
    motion = Motion::STARTING;
    motion.soft_limit_margin_frac = -1.0;
    assert_eq!(
      SoftLimits::for_axis(&motion, Axis::M1H),
      SoftLimits {
        min_steps: -4_000,
        max_steps: 4_000
      }
    );
  }

  proptest! {
    #[test]
    fn an_approach_always_adds_up_to_the_requested_move(
      delta in -10_000_i32..10_000, overshoot in 0_i32..1_000, last in 0_u8..3,
    ) {
      let last_dir = [LastDir::Unknown, LastDir::Plus, LastDir::Minus][usize::from(last)];
      let approach = plan(delta, last_dir, overshoot);
      prop_assert_eq!(approach.reverse_steps + approach.forward_steps, delta);
    }

    /// The point of the standard approach: the last motion is never in the minus direction, and when the plan
    /// reverses at all it comes back by a full overshoot.
    #[test]
    fn an_approach_always_ends_in_the_plus_direction(
      delta in -10_000_i32..10_000, overshoot in 1_i32..1_000, last in 0_u8..3,
    ) {
      let last_dir = [LastDir::Unknown, LastDir::Plus, LastDir::Minus][usize::from(last)];
      let approach = plan(delta, last_dir, overshoot);
      prop_assert!(approach.reverse_steps <= 0);
      prop_assert!(approach.forward_steps >= 0);
      if approach.reverse_steps < 0 {
        prop_assert_eq!(approach.forward_steps, overshoot);
      } else if last_dir != LastDir::Plus {
        prop_assert!(approach.forward_steps >= overshoot);
      }
    }
  }
}
