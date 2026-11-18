"""The exact optics against the factors the documents derive with small angles (reference 1.2, addendum A.9)."""

import math

import numpy as np
import pytest

from beamviz.optics import UP, Mount, angle_between_urad, meet_plane, reflect, unit


def test_a_45_degree_mirror_folds_a_beam_along_minus_x_onto_plus_y():
  folded = reflect(np.array([-1.0, 0.0, 0.0]), unit(np.array([1.0, 1.0, 0.0])))
  assert np.allclose(folded, [0.0, 1.0, 0.0])


def test_a_ray_meets_a_plane_where_the_plane_is_and_never_along_it():
  along_y = np.array([0.0, 1.0, 0.0])
  hit = meet_plane(np.zeros(3), along_y, np.array([5.0, 7.0, 3.0]), along_y)
  assert np.allclose(hit, [0.0, 7.0, 0.0])
  with pytest.raises(ValueError, match="along the plane"):
    meet_plane(np.zeros(3), np.array([1.0, 0.0, 0.0]), np.array([0.0, 7.0, 0.0]), np.array([0.0, 1.0, 0.0]))


def test_an_untouched_mount_keeps_its_drawn_normal_and_passes_through_its_center():
  mount = Mount(np.array([10.0, 20.0, 0.0]), unit(np.array([1.0, 1.0, 0.0])), 35.0)
  point, normal = mount.surface(0.0, 0.0)
  assert np.allclose(normal, mount.normal)
  assert abs(np.dot(mount.center - point, normal)) < 1e-12
  assert np.allclose(np.cross(mount.across, UP), mount.normal)


def _beam_turn_urad(h_um: float, v_um: float) -> tuple[float, float]:
  """How far a screw advance turns the beam folded by mirror 1, horizontally and vertically."""
  mount = Mount(np.zeros(3), unit(np.array([1.0, 1.0, 0.0])), 35.0)
  incoming = np.array([-1.0, 0.0, 0.0])
  _, normal = mount.surface(h_um, v_um)
  return angle_between_urad(reflect(incoming, normal), np.array([0.0, 1.0, 0.0]))


def test_a_horizontal_screw_turns_the_beam_by_twice_the_plate_s_tilt():
  # Addendum A.9: deflect_k 2.0 for a horizontal screw. One micrometer on a 35 mm lever is 28.57 urad of tilt.
  tilt_urad = 1.0 / 35_000.0 * 1e6
  turn_h, turn_v = _beam_turn_urad(1.0, 0.0)
  assert abs(turn_h) == pytest.approx(2.0 * tilt_urad, rel=1e-4)
  assert abs(turn_v) < 1e-3


def test_a_vertical_screw_turns_the_beam_by_the_square_root_of_two_times_the_tilt():
  # Addendum A.9: deflect_k 1.414 for a vertical screw on a 45 degree mirror (reference 1.2).
  tilt_urad = 1.0 / 35_000.0 * 1e6
  turn_h, turn_v = _beam_turn_urad(0.0, 1.0)
  assert abs(turn_v) == pytest.approx(math.sqrt(2.0) * tilt_urad, rel=1e-4)
  assert abs(turn_h) < 1e-3


def test_large_tilts_are_exact_where_the_small_angle_factors_drift():
  # 500 um on 35 mm is 14 mrad of tilt: the factor 2 still holds for the horizontal screw, being a rotation about the
  # vertical, which a 45 degree fold keeps exact.
  turn_h, _ = _beam_turn_urad(500.0, 0.0)
  assert abs(turn_h) * 1e-6 == pytest.approx(2.0 * math.atan(500.0 / 35_000.0), rel=1e-6)


def test_angles_are_measured_left_and_up_from_the_reference():
  along_x = np.array([1.0, 0.0, 0.0])
  left_h, _ = angle_between_urad(unit(np.array([1.0, 1e-4, 0.0])), along_x)
  _, up_v = angle_between_urad(unit(np.array([1.0, 0.0, 1e-4])), along_x)
  assert left_h == pytest.approx(100.0, rel=1e-6)
  assert up_v == pytest.approx(100.0, rel=1e-6)
