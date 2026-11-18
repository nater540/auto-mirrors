"""The ray-traced machine: what aligned means on it, how it compares with the Rust model, and the screw chain ported
from `align_sim::world`."""

import math
from dataclasses import replace

import numpy as np
import pytest

from beamviz.machine import AXES, FrameErrors, Machine, Scenario, Screw, ScrewModel

GAUSSIAN_CENTER_SLOPE = 2.0 * math.sqrt(2.0 / math.pi)
"""`align_core::units::GAUSSIAN_CENTER_SLOPE` (reference 2.2)."""


def ideal() -> Scenario:
  return Scenario().ideal()


def test_a_machine_built_to_the_drawing_is_aligned_with_every_screw_at_zero():
  machine = Machine(ideal())
  assert np.max(np.abs(machine.residual())) < 1e-12
  for trace in machine.traces():
    assert np.hypot(*trace.spot_um) < 1e-6
    assert not trace.lost
  assert all(abs(angle) < 1e-6 for angle in machine.leg_angles_urad().values())


def test_a_sensor_off_the_beam_line_moves_every_spot_alike_and_leaves_m_at_zero():
  errors = replace(ideal().errors, sensor_h_um=150.0, sensor_v_um=-80.0)
  machine = Machine(replace(ideal(), errors=errors))
  assert np.max(np.abs(machine.residual())) < 1e-12
  for trace in machine.traces():
    assert trace.spot_um == pytest.approx((-150.0, 80.0), abs=1e-6)


def test_a_tube_pointing_error_misaligns_and_the_exact_solution_points_both_legs_along_their_travel():
  errors = replace(ideal().errors, tube_h_urad=300.0, tube_v_urad=-200.0, racking_urad=250.0, y_pitch_urad=100.0)
  machine = Machine(replace(ideal(), errors=errors))
  assert np.max(np.abs(machine.residual())) > 0.01
  seats = machine.aligned_seats()
  assert np.max(np.abs(machine.residual(seats))) < 1e-9
  for screw, axis in zip(machine.screws.values(), AXES, strict=True):
    screw.seat = round(seats[axis])
  # Whole steps leave a fraction of a step: about 14 urad of beam per step of a horizontal screw.
  assert all(abs(angle) < 15.0 for angle in machine.leg_angles_urad().values())


def test_mirror_2_does_not_move_the_beam_between_a_and_b():
  jacobian = Machine(Scenario()).jacobian()
  leg1_rows = jacobian[0:2]
  assert np.max(np.abs(leg1_rows[:, 2:4])) < 0.01 * np.max(np.abs(leg1_rows[:, 0:2]))


def test_the_exact_jacobian_of_a_drawn_machine_has_the_rust_model_s_magnitudes():
  # `align_sim::world::World::true_jacobian`: k * tilt per step * span * slope / radius, with the steps per revolution
  # of this machine. The exact model gets there by tracing rays; agreement checks both.
  scenario = ideal()
  jacobian = Machine(scenario).jacobian()
  mechanics = scenario.mechanics
  tilt_rad = mechanics.um_per_step / (mechanics.lever_mm * 1e3)
  slope_per_um = GAUSSIAN_CENTER_SLOPE / scenario.beam.radius_um
  layout = scenario.layout
  dy_um = (layout.ly_far_mm - layout.ly_near_mm) * 1e3
  dx_um = (layout.lx_far_mm - layout.lx_near_mm) * 1e3
  expected = {
    (0, 0): 2.0 * tilt_rad * dy_um * slope_per_um,
    (1, 1): math.sqrt(2.0) * tilt_rad * dy_um * slope_per_um,
    (2, 0): 2.0 * tilt_rad * dx_um * slope_per_um,
    (2, 2): 2.0 * tilt_rad * dx_um * slope_per_um,
    (3, 1): math.sqrt(2.0) * tilt_rad * dx_um * slope_per_um,
    (3, 3): math.sqrt(2.0) * tilt_rad * dx_um * slope_per_um
  }
  for (row, column), value in expected.items():
    assert abs(jacobian[row, column]) == pytest.approx(value, rel=1e-3), (row, column)
  for row in range(4):
    for column in range(4):
      if (row, column) not in expected:
        assert abs(jacobian[row, column]) < 1e-3 * value


def test_a_centered_beam_reads_zero_and_an_offset_one_follows_the_error_function():
  # Reference 2.2: with a large aperture the normalized position is erf(sqrt(2) x0 / w).
  machine = Machine(ideal())
  for trace in machine.traces():
    a, b, c, d = machine.quadrants(trace, noisy=False)
    assert a == pytest.approx(b) == pytest.approx(c) == pytest.approx(d)
  # The 10 mm half aperture clips the Gaussian's tails by a few parts per million; a wide one does not.
  errors = replace(ideal().errors, sensor_h_um=-1000.0)
  wide = replace(ideal().beam, aperture_half_um=1e6)
  (h_norm, v_norm, total), *_ = Machine(replace(ideal(), errors=errors, beam=wide)).readings()
  assert h_norm == pytest.approx(math.erf(math.sqrt(2.0) * 1000.0 / 4000.0), rel=1e-6)
  assert v_norm == pytest.approx(0.0, abs=1e-12)
  assert total == pytest.approx(1000.0, rel=1e-6)


def test_noise_on_the_quadrants_gives_the_asked_sigma_of_a_normalized_coordinate():
  machine = Machine(replace(Scenario(), errors=FrameErrors(**dict.fromkeys(FrameErrors.__dataclass_fields__, 0.0))))
  trace = machine.traces()[0]
  samples = []
  for _ in range(4000):
    a, b, c, d = machine.quadrants(trace, noisy=True)
    samples.append(((a + d) - (b + c)) / (a + b + c + d))
  assert np.std(samples) == pytest.approx(0.002, rel=0.05)


def test_a_beam_that_misses_mirror_2_is_lost_and_reads_dark():
  errors = replace(ideal().errors, tube_h_urad=30_000.0)
  machine = Machine(replace(ideal(), errors=errors))
  far = machine.traces()[1]
  assert far.lost
  assert sum(machine.quadrants(far, noisy=False)) == 0.0


# --- the screw chain, as `align_sim::world` tests it ---


def test_a_move_that_ends_in_the_plus_direction_lands_where_it_was_commanded_despite_backlash():
  rng = np.random.default_rng(0)
  model = ScrewModel(backlash_steps=20, stick_steps=0)
  screw = Screw()
  screw.move(-50, model, rng)
  screw.move(30, model, rng)
  assert (screw.commanded, screw.seat) == (-20, -20)


def test_backlash_swallows_the_first_steps_after_a_reversal():
  rng = np.random.default_rng(0)
  model = ScrewModel(backlash_steps=20, stick_steps=0)
  screw = Screw()
  screw.move(-10, model, rng)
  assert screw.seat == 0
  screw.move(-15, model, rng)
  assert screw.seat == -5


def test_a_sticking_seat_holds_then_slips_onto_the_screw():
  rng = np.random.default_rng(0)
  model = ScrewModel(backlash_steps=0, stick_steps=3)
  screw = Screw()
  screw.move(2, model, rng)
  assert screw.seat == 0
  screw.move(1, model, rng)
  assert screw.seat == 3


def test_a_dead_screw_counts_its_steps_and_moves_nothing():
  rng = np.random.default_rng(0)
  screw = Screw()
  screw.move(40, ScrewModel(0, 0, skip_probability=1.0), rng)
  assert (screw.commanded, screw.motor, screw.seat) == (40, 0, 0)
