"""Exact geometric optics for the flying-optics path: a ray meeting a plane, a reflection off it, and a mirror mount
tilted by its adjuster screws. Nothing here uses a small-angle approximation, so it checks the factors that the
documents derive with one (reference 1.2: a horizontal screw on a 45 degree mirror turns the beam by twice the plate's
tilt, a vertical screw by the square root of two times it)."""

from dataclasses import dataclass

import numpy as np

UP = np.array([0.0, 0.0, 1.0])


def unit(vector: np.ndarray) -> np.ndarray:
  """The vector scaled to length one."""
  return vector / np.linalg.norm(vector)


def reflect(direction: np.ndarray, normal: np.ndarray) -> np.ndarray:
  """The direction of a ray after a plane mirror with the given unit normal."""
  return direction - 2.0 * np.dot(direction, normal) * normal


def meet_plane(origin: np.ndarray, direction: np.ndarray, point: np.ndarray, normal: np.ndarray) -> np.ndarray:
  """Where the ray from `origin` along `direction` meets the plane through `point` with the given normal."""
  across = np.dot(direction, normal)
  if abs(across) < 1e-12:
    raise ValueError("the ray runs along the plane")
  return origin + (np.dot(point - origin, normal) / across) * direction


@dataclass(frozen=True)
class Mount:
  """A three-point mirror mount in an L pattern (Cloudray Series E, addendum A.12). The plate rests on a fixed pivot
  at one corner and on two screw tips: the horizontal screw `lever_mm` from the pivot along the plate's horizontal,
  the vertical screw `lever_mm` above it. The mirror's center, where the beam lands, is half a lever arm from the
  pivot both ways (AS MODELED: where the mirror sits between the seats is not in the documents)."""

  center: np.ndarray
  """Where the beam lands on the mirror as drawn, in millimeters."""
  normal: np.ndarray
  """The mirror's unit normal as drawn: horizontal, the plate standing upright."""
  lever_mm: float

  @property
  def across(self) -> np.ndarray:
    """The plate's horizontal, toward the horizontal screw: across, up, and the normal are right-handed."""
    return np.cross(UP, self.normal)

  @property
  def pivot(self) -> np.ndarray:
    return self.center - 0.5 * self.lever_mm * (self.across + UP)

  def surface(self, h_um: float, v_um: float) -> tuple[np.ndarray, np.ndarray]:
    """The mirror's plane, a point on it and its unit normal, with the horizontal screw's tip advanced `h_um` and the
    vertical screw's `v_um` along the normal as drawn. The plate turns about the line through the pivot and the
    other screw, exactly, and its surface moves where the beam lands as the lever geometry makes it."""
    h_tip = self.lever_mm * self.across + 1e-3 * h_um * self.normal
    v_tip = self.lever_mm * UP + 1e-3 * v_um * self.normal
    return self.pivot, unit(np.cross(h_tip, v_tip))


def angle_between_urad(direction: np.ndarray, reference: np.ndarray) -> tuple[float, float]:
  """How far `direction` points from `reference`, in microradians: horizontally (about the vertical, positive to the
  left of `reference` seen from above) and vertically (positive up)."""
  side = unit(np.cross(UP, reference))
  height = np.cross(reference, side)
  along = np.dot(direction, reference)
  return (
    float(np.arctan2(np.dot(direction, side), along) * 1e6),
    float(np.arctan2(np.dot(direction, height), along) * 1e6)
  )
