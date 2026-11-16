//! The measurement vector: differences of three readings (hand-off 3.2, reference 1.4).
//!
//! The beam is read at three gantry positions:
//!
//! | Point | Gantry Y | Head X |
//! |---|---|---|
//! | A | near | near |
//! | B | far | near |
//! | C | far | far |
//!
//! `m = [hB - hA, vB - vA, hC - hB, vC - vB]`, and the target is zero. Moving along one leg changes only that
//! leg's lever arm, so each difference depends on a beam angle alone and a constant offset at the head cancels.
//! The absolute reading at A is kept as a health metric and is never a control target:
//! four tilt actuators cannot remove a constant offset.

use nalgebra::Vector4;

use crate::config::SensorLimits;
use crate::quadrant::Reading;

/// The readings at the three gantry positions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shots {
  /// Y near, X near.
  pub a: Reading,
  /// Y far, X near.
  pub b: Reading,
  /// Y far, X far.
  pub c: Reading
}

/// The constant beam offset at the head, as read at point A. A health metric only.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadOffset {
  /// Horizontal offset, normalized.
  pub h_norm: f32,
  /// Vertical offset, normalized.
  pub v_norm: f32
}

impl Shots {
  /// The measurement vector `m = [hB - hA, vB - vA, hC - hB, vC - vB]`.
  pub fn measurement(&self) -> Vector4<f32> {
    Vector4::new(
      self.b.h_norm - self.a.h_norm,
      self.b.v_norm - self.a.v_norm,
      self.c.h_norm - self.b.h_norm,
      self.c.v_norm - self.b.v_norm
    )
  }

  /// The reading at A, kept as the head offset health metric.
  pub fn head_offset(&self) -> HeadOffset {
    HeadOffset {
      h_norm: self.a.h_norm,
      v_norm: self.a.v_norm
    }
  }
}

impl HeadOffset {
  /// Whether the offset is large enough that the head needs mechanical centering: above the configured fraction
  /// of the solver's linear range on either coordinate (reference 1.4).
  pub fn needs_mechanical_centering(&self, limits: &SensorLimits) -> bool {
    let allowed = limits.offset_warn_frac * limits.solver_max_norm;
    self.h_norm.abs() > allowed || self.v_norm.abs() > allowed
  }
}

#[cfg(test)]
mod tests {
  use proptest::prelude::*;

  use super::*;
  use crate::test_support::assert_close;

  const LIMITS: SensorLimits = SensorLimits::STARTING;

  fn reading(h_norm: f32, v_norm: f32) -> Reading {
    Reading {
      h_norm,
      v_norm,
      sum: 100.0
    }
  }

  #[test]
  fn the_measurement_vector_is_b_minus_a_then_c_minus_b() {
    let shots = Shots {
      a: reading(0.10, -0.20),
      b: reading(0.25, 0.05),
      c: reading(-0.05, 0.30)
    };
    let m = shots.measurement();
    assert_close(m.x, 0.15, 1e-6);
    assert_close(m.y, 0.25, 1e-6);
    assert_close(m.z, -0.30, 1e-6);
    assert_close(m.w, 0.25, 1e-6);
  }

  #[test]
  fn a_beam_that_stays_put_across_the_three_positions_measures_zero_whatever_its_offset() {
    let shots = Shots {
      a: reading(0.17, -0.11),
      b: reading(0.17, -0.11),
      c: reading(0.17, -0.11)
    };
    assert_eq!(shots.measurement(), Vector4::zeros());
    assert_eq!(
      shots.head_offset(),
      HeadOffset {
        h_norm: 0.17,
        v_norm: -0.11
      }
    );
  }

  #[test]
  fn an_offset_above_thirty_percent_of_the_solver_range_needs_mechanical_centering() {
    // 30% of the 0.6 solver range is 0.18.
    assert!(
      !HeadOffset {
        h_norm: 0.17,
        v_norm: -0.17
      }
      .needs_mechanical_centering(&LIMITS)
    );
    assert!(
      HeadOffset {
        h_norm: 0.19,
        v_norm: 0.0
      }
      .needs_mechanical_centering(&LIMITS)
    );
    assert!(
      HeadOffset {
        h_norm: 0.0,
        v_norm: -0.19
      }
      .needs_mechanical_centering(&LIMITS)
    );
  }

  proptest! {
    /// The reason for decision 3.2: a constant offset at the head is invisible to the control target.
    #[test]
    fn a_constant_head_offset_does_not_change_the_measurement_vector(
      points in proptest::array::uniform6(-0.4_f32..0.4),
      offset_h in -0.3_f32..0.3,
      offset_v in -0.3_f32..0.3
    ) {
      let [ah, av, bh, bv, ch, cv] = points;
      let plain = Shots { a: reading(ah, av), b: reading(bh, bv), c: reading(ch, cv) };
      let shifted = Shots {
        a: reading(ah + offset_h, av + offset_v),
        b: reading(bh + offset_h, bv + offset_v),
        c: reading(ch + offset_h, cv + offset_v)
      };
      let difference = plain.measurement() - shifted.measurement();
      prop_assert!(difference.amax() <= 1e-6);
    }
  }
}
