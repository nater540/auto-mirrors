//! The five bench buttons that stand in for the detector (`docs/SENSOR_PLAN.md` sections 7.2 and 7.5).
//!
//! Each button is a quadrant corner, A upper right, B upper left, C lower left, D lower right (the quadrant layout of
//! reference 2.1), or the center. The logic here is a pure function of sampled levels and time:
//! [`Debouncer`] turns raw samples into stable levels, [`Chords`] groups presses into one event, and [`position`]
//! maps a set of buttons to a beam position.
//!
//! No crate was used: `debouncr` has had no release since 2020, `button-driver` handles single-button clicks without
//! chords, and debouncing five levels is a few lines.

bitflags::bitflags! {
  /// A set of buttons.
  #[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
  pub struct ButtonSet: u8 {
    /// Corner A, upper right: `(+, +)`.
    const A = 0b0_0001;
    /// Corner B, upper left: `(-, +)`.
    const B = 0b0_0010;
    /// Corner C, lower left: `(-, -)`.
    const C = 0b0_0100;
    /// Corner D, lower right: `(+, -)`.
    const D = 0b0_1000;
    /// The center.
    const CENTER = 0b1_0000;
  }
}

/// The parameters of the button bench. Not machine configuration: they shape a stand-in, like a simulator scenario.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ButtonBench {
  /// How far a corner is from the center, in Q15 normalized units on each axis (0.4: inside the solver range of 0.6).
  pub corner_q15: i16,
  /// A level must hold this long to count.
  pub debounce_ms: u32,
  /// Buttons pressed within this long of the first one form one chord.
  pub chord_ms: u32,
  /// The time from a SHOT to its reading. A detector takes far longer; this exercises the coordinator's waiting.
  pub reply_delay_ms: u32
}

impl ButtonBench {
  /// The starting values of `docs/SENSOR_PLAN.md` sections 7.2 and 7.5.
  pub const STARTING: Self = Self {
    corner_q15: 13_107,
    debounce_ms: 20,
    chord_ms: 50,
    reply_delay_ms: 200
  };
}

/// Debounces the five levels together: a change counts once the raw levels have held it for `debounce_ms`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Debouncer {
  stable: ButtonSet,
  candidate: ButtonSet,
  candidate_since_ms: u64
}

impl Debouncer {
  /// Takes one sample of the raw levels and returns the stable set.
  pub fn sample(&mut self, raw: ButtonSet, now_ms: u64, debounce_ms: u32) -> ButtonSet {
    if raw != self.candidate {
      self.candidate = raw;
      self.candidate_since_ms = now_ms;
    }
    if now_ms.saturating_sub(self.candidate_since_ms) >= u64::from(debounce_ms) {
      self.stable = self.candidate;
    }
    self.stable
  }
}

/// Groups presses into chords: every button pressed within `chord_ms` of the first one belongs to it. A chord is
/// reported once, when its window closes, or earlier if every button is released first. A new chord starts only
/// after every button has been released.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Chords {
  chord: ButtonSet,
  first_ms: Option<u64>,
  reported: bool
}

impl Chords {
  /// Takes the stable set and returns a chord when one is complete.
  pub fn update(&mut self, stable: ButtonSet, now_ms: u64, chord_ms: u32) -> Option<ButtonSet> {
    let Some(first_ms) = self.first_ms else {
      if !stable.is_empty() {
        self.chord = stable;
        self.first_ms = Some(now_ms);
        self.reported = false;
      }
      return None;
    };
    let open = now_ms.saturating_sub(first_ms) < u64::from(chord_ms);
    if open {
      self.chord |= stable;
    }
    let released = stable.is_empty();
    let report = !self.reported && (!open || released);
    if report {
      self.reported = true;
    }
    let chord = self.chord;
    if released {
      self.first_ms = None;
    }
    report.then_some(chord)
  }
}

/// Where the beam is for a set of buttons, in Q15 normalized units `(h, v)` (`docs/SENSOR_PLAN.md` section 7.2).
///
/// | Set | Position |
/// |---|---|
/// | none | centered |
/// | one corner | that corner |
/// | two adjacent corners | the midpoint of their edge |
/// | the center | centered |
/// | the center and one corner | halfway to that corner |
/// | opposite corners, or three buttons or more | `None`: rejected |
pub fn position(set: ButtonSet, corner_q15: i16) -> Option<(i16, i16)> {
  let c = corner_q15;
  let half = c / 2;
  let corner = |button: ButtonSet| -> (i16, i16) {
    match button {
      b if b == ButtonSet::A => (c, c),
      b if b == ButtonSet::B => (-c, c),
      b if b == ButtonSet::C => (-c, -c),
      _ => (c, -c)
    }
  };
  let corners = set.difference(ButtonSet::CENTER);
  let center = set.contains(ButtonSet::CENTER);
  match (corners.bits().count_ones(), center) {
    (0, _) => Some((0, 0)),
    (1, false) => Some(corner(corners)),
    (1, true) => {
      let (h, v) = corner(corners);
      Some((h.signum() * half, v.signum() * half))
    }
    (2, false) => {
      let edge = [
        (ButtonSet::A.union(ButtonSet::B), (0, c)),
        (ButtonSet::B.union(ButtonSet::C), (-c, 0)),
        (ButtonSet::C.union(ButtonSet::D), (0, -c)),
        (ButtonSet::D.union(ButtonSet::A), (c, 0))
      ];
      edge.iter().find(|(pair, _)| *pair == corners).map(|(_, at)| *at)
    }
    _ => None
  }
}

#[cfg(test)]
mod tests {
  extern crate std;

  use std::vec::Vec;

  use proptest::prelude::*;

  use super::*;

  const BENCH: ButtonBench = ButtonBench::STARTING;
  const C: i16 = BENCH.corner_q15;

  #[test]
  fn the_corner_offset_is_0_4_in_q15() {
    // 0.4 of 32768 is 13107.2.
    assert_eq!(i32::from(C) * 5, 65_535);
  }

  #[test]
  fn each_corner_has_the_signs_of_its_quadrant() {
    assert_eq!(position(ButtonSet::A, C), Some((C, C)));
    assert_eq!(position(ButtonSet::B, C), Some((-C, C)));
    assert_eq!(position(ButtonSet::C, C), Some((-C, -C)));
    assert_eq!(position(ButtonSet::D, C), Some((C, -C)));
  }

  #[test]
  fn adjacent_corners_give_the_midpoint_of_their_edge() {
    assert_eq!(position(ButtonSet::A | ButtonSet::B, C), Some((0, C)));
    assert_eq!(position(ButtonSet::B | ButtonSet::C, C), Some((-C, 0)));
    assert_eq!(position(ButtonSet::C | ButtonSet::D, C), Some((0, -C)));
    assert_eq!(position(ButtonSet::D | ButtonSet::A, C), Some((C, 0)));
  }

  #[test]
  fn the_center_and_nothing_are_centered_and_center_with_a_corner_is_halfway() {
    assert_eq!(position(ButtonSet::empty(), C), Some((0, 0)));
    assert_eq!(position(ButtonSet::CENTER, C), Some((0, 0)));
    assert_eq!(position(ButtonSet::CENTER | ButtonSet::B, C), Some((-C / 2, C / 2)));
    assert_eq!(position(ButtonSet::CENTER | ButtonSet::D, C), Some((C / 2, -C / 2)));
  }

  #[test]
  fn opposite_corners_and_three_or_more_buttons_are_rejected() {
    assert_eq!(position(ButtonSet::A | ButtonSet::C, C), None);
    assert_eq!(position(ButtonSet::B | ButtonSet::D, C), None);
    assert_eq!(position(ButtonSet::A | ButtonSet::B | ButtonSet::C, C), None);
    assert_eq!(position(ButtonSet::A | ButtonSet::B | ButtonSet::CENTER, C), None);
    assert_eq!(position(ButtonSet::all(), C), None);
  }

  #[test]
  fn a_level_counts_only_after_holding_for_the_debounce_time() {
    let mut debouncer = Debouncer::default();
    assert_eq!(debouncer.sample(ButtonSet::A, 0, 20), ButtonSet::empty());
    assert_eq!(debouncer.sample(ButtonSet::A, 19, 20), ButtonSet::empty());
    assert_eq!(debouncer.sample(ButtonSet::A, 20, 20), ButtonSet::A);
  }

  #[test]
  fn a_bounce_restarts_the_debounce_time() {
    let mut debouncer = Debouncer::default();
    debouncer.sample(ButtonSet::A, 0, 20);
    debouncer.sample(ButtonSet::empty(), 10, 20);
    debouncer.sample(ButtonSet::A, 15, 20);
    assert_eq!(debouncer.sample(ButtonSet::A, 30, 20), ButtonSet::empty());
    assert_eq!(debouncer.sample(ButtonSet::A, 35, 20), ButtonSet::A);
  }

  /// Runs stable levels through the chord logic at 5 ms steps and collects the chords.
  fn chords_of(levels: &[(u64, ButtonSet)]) -> Vec<(u64, ButtonSet)> {
    let mut chords = Chords::default();
    let mut out = Vec::new();
    let end = levels.last().map_or(0, |(at, _)| *at) + 200;
    let mut level = ButtonSet::empty();
    for now in (0..=end).step_by(5) {
      for (at, set) in levels {
        if *at == now {
          level = *set;
        }
      }
      if let Some(chord) = chords.update(level, now, BENCH.chord_ms) {
        out.push((now, chord));
      }
    }
    out
  }

  #[test]
  fn a_single_press_is_one_chord_when_its_window_closes() {
    let chords = chords_of(&[(10, ButtonSet::A), (300, ButtonSet::empty())]);
    assert_eq!(chords, [(60, ButtonSet::A)]);
  }

  #[test]
  fn a_quick_tap_is_reported_on_release() {
    let chords = chords_of(&[(10, ButtonSet::A), (30, ButtonSet::empty())]);
    assert_eq!(chords, [(30, ButtonSet::A)]);
  }

  #[test]
  fn buttons_pressed_within_the_window_join_the_chord_and_later_ones_do_not() {
    let chords = chords_of(&[
      (10, ButtonSet::A),
      (40, ButtonSet::A | ButtonSet::B),
      (100, ButtonSet::A | ButtonSet::B | ButtonSet::C),
      (300, ButtonSet::empty())
    ]);
    assert_eq!(chords, [(60, ButtonSet::A | ButtonSet::B)]);
  }

  #[test]
  fn a_new_chord_needs_every_button_released_first() {
    let chords = chords_of(&[
      (10, ButtonSet::A),
      (100, ButtonSet::A | ButtonSet::B),
      (200, ButtonSet::empty()),
      (250, ButtonSet::D),
      (400, ButtonSet::empty())
    ]);
    assert_eq!(chords, [(60, ButtonSet::A), (300, ButtonSet::D)]);
  }

  proptest! {
    #[test]
    fn every_position_is_inside_the_corner_square(bits in 0_u8..32) {
      if let Some((h, v)) = position(ButtonSet::from_bits_truncate(bits), C) {
        prop_assert!(h.abs() <= C && v.abs() <= C);
      }
    }

    #[test]
    fn every_chord_is_reported_once_and_is_a_set_that_was_pressed(
      levels in proptest::collection::vec((0_u8..32, 1_u64..40), 1..20),
    ) {
      let mut at = 0;
      let mut timeline = Vec::new();
      for (bits, gap) in levels {
        timeline.push((at, ButtonSet::from_bits_truncate(bits)));
        at += gap * 5;
      }
      timeline.push((at, ButtonSet::empty()));
      let pressed = timeline.iter().fold(ButtonSet::empty(), |all, (_, set)| all | *set);
      let releases = timeline.windows(2).filter(|pair| !pair[0].1.is_empty() && pair[1].1.is_empty()).count();
      let chords = chords_of(&timeline);
      prop_assert!(chords.len() <= releases.max(1));
      for (_, chord) in chords {
        prop_assert!(!chord.is_empty());
        prop_assert!(pressed.contains(chord));
      }
    }
  }
}
