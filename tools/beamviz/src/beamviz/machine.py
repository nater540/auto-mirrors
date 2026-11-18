"""The machine, ray-traced: the tube, mirror 1 on the frame, mirror 2 riding the gantry, and the quadrant sensor on
the head, with the errors a real frame has.

It is built apart from the Rust model (`align_sim::world`), which is linear and treats each transverse direction on
its own. Here the beam is reflected off tilted planes in three dimensions, the gantry and the head travel along axes
that are not quite square to each other, and the sensor sits off the head's beam line. Aligned means what it means
on the machine: the spot does not move on the sensor as the gantry and the head travel.

Layout, in millimeters: mirror 1 at the origin; the tube's beam arrives along -X; mirror 1 sends it along the Y
travel to mirror 2, which sends it along the X travel to the head. Z is up. The sensor's h is horizontal across the
beam (+Y for a beam along +X), v is up; quadrant a is upper right (+h, +v), b upper left, c lower left, d lower right,
as `align_core::quadrant::Quadrants` has them.

The screw chain (motor, backlash, sticking seat) is the Rust model's, ported line for line: it is the mechanics the
aligner was designed against, not the optics this model checks.
"""

import math
from dataclasses import dataclass, field, fields, replace

import numpy as np

from beamviz.optics import UP, Mount, angle_between_urad, meet_plane, reflect, unit

AXES = ("m1h", "m1v", "m2h", "m2v")
POINTS = ("A", "B", "C")


@dataclass(frozen=True)
class Mechanics:
  """The screws and mounts. The aligner configures its own (`align_core::config::Mechanics`); these are the truth."""

  steps_per_rev: float = 2048.0
  """VERIFIED on the bench, 2026-11-17: the 28BYJ-48 is a true 64:1, not the configured 2037.8864."""
  screw_lead_um: float = 500.0
  """AS MODELED (addendum A.12): M6 x 0.5 adjusters."""
  lever_mm: float = 35.0
  """AS MODELED (addendum A.12)."""
  mirror_radius_mm: float = 12.5
  """UNVERIFIED: a 25 mm mirror. A beam that lands farther from its center than this is lost."""

  @property
  def um_per_step(self) -> float:
    return self.screw_lead_um / self.steps_per_rev


@dataclass(frozen=True)
class Layout:
  """Where things are. The path lengths are the Rust model's placeholder (`Geometry::PLACEHOLDER`)."""

  tube_mm: float = 400.0
  """UNVERIFIED: tube output to mirror 1."""
  ly_near_mm: float = 100.0
  ly_far_mm: float = 600.0
  lx_near_mm: float = 100.0
  lx_far_mm: float = 700.0

  def points(self) -> tuple[tuple[float, float], ...]:
    """The path lengths `(Ly, Lx)` at A (Y near, X near), B (Y far, X near), and C (Y far, X far)."""
    return (
      (self.ly_near_mm, self.lx_near_mm),
      (self.ly_far_mm, self.lx_near_mm),
      (self.ly_far_mm, self.lx_far_mm)
    )


@dataclass(frozen=True)
class FrameErrors:
  """How the built machine differs from the drawing. All UNVERIFIED: sizes a hand-built frame might have."""

  tube_h_urad: float = 250.0
  tube_v_urad: float = -150.0
  """The tube's beam, off the line it should arrive on."""
  m1_h_urad: float = 120.0
  m1_v_urad: float = -90.0
  m2_h_urad: float = -180.0
  m2_v_urad: float = 140.0
  """Each mirror plate as mounted, off the drawing, with its screws at their commissioning position."""
  racking_urad: float = 300.0
  """The head's X travel, turned about the vertical from square to the gantry's Y travel."""
  y_pitch_urad: float = 120.0
  """The gantry's Y travel, climbing."""
  x_pitch_urad: float = -80.0
  """The head's X travel, climbing."""
  sensor_h_um: float = 150.0
  sensor_v_um: float = -80.0
  """The sensor's center, off the head's beam line: the head offset, which no tilt removes."""


@dataclass(frozen=True)
class Beam:
  """The beam and the detector: the Rust model's placeholder (`Beam::PLACEHOLDER`) with its realistic noise."""

  radius_um: float = 4000.0
  """UNVERIFIED: the 1/e^2 radius at A."""
  divergence_urad: float = 0.0
  """UNVERIFIED: the radius grows by this much per meter of path beyond A."""
  aperture_half_um: float = 10_000.0
  sum_ref: float = 1000.0
  noise_norm: float = 0.002
  """UNVERIFIED: one sigma of a normalized coordinate of a centered beam."""


@dataclass(frozen=True)
class ScrewModel:
  """The imperfections of one screw: the Rust model's realistic placeholder."""

  backlash_steps: int = 20
  stick_steps: int = 3
  skip_probability: float = 0.0


@dataclass(frozen=True)
class Scenario:
  mechanics: Mechanics = field(default_factory=Mechanics)
  layout: Layout = field(default_factory=Layout)
  errors: FrameErrors = field(default_factory=FrameErrors)
  beam: Beam = field(default_factory=Beam)
  screws: ScrewModel = field(default_factory=ScrewModel)
  seed: int = 1

  def ideal(self) -> "Scenario":
    """The same machine built exactly to the drawing, with perfect screws and a noiseless sensor."""
    perfect = FrameErrors(**{error.name: 0.0 for error in fields(FrameErrors)})
    return replace(self, errors=perfect, beam=replace(self.beam, noise_norm=0.0), screws=ScrewModel(0, 0, 0.0))


@dataclass
class Screw:
  """One screw: the commanded count, the motor, the screw itself through the backlash, and the seat on the plate."""

  commanded: int = 0
  motor: int = 0
  thread: int = 0
  seat: int = 0

  def move(self, delta_steps: int, model: ScrewModel, rng: np.random.Generator) -> None:
    """One step at a time through the motor, the backlash, and the seat, as `align_sim::world::World::move_rel`."""
    direction = int(math.copysign(1, delta_steps))
    for _ in range(abs(delta_steps)):
      skipped = model.skip_probability > 0.0 and rng.random() < model.skip_probability
      self.commanded += direction
      if not skipped:
        self.motor += direction
      self.thread = min(max(self.thread, self.motor), self.motor + model.backlash_steps)
      if abs(self.thread - self.seat) >= model.stick_steps:
        self.seat = self.thread


@dataclass(frozen=True)
class Trace:
  """The beam's path to the head with the gantry and the head at one of A, B, C."""

  points: np.ndarray
  """The tube's output, mirror 1, mirror 2, and the sensor: where the beam is at each, in millimeters."""
  spot_um: tuple[float, float]
  """Where the beam's center lands on the sensor, h then v, from the sensor's center."""
  radius_um: float
  lost: bool
  """The beam missed a mirror."""


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
  """The rotation by `angle` radians about the unit `axis` (Rodrigues)."""
  x, y, z = axis
  k = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
  return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


def _halves(center_um: float, radius_um: float, aperture_half_um: float) -> tuple[float, float]:
  """The fractions of a Gaussian beam on the negative and positive half of the detector, along one direction
  (reference 2.2, as `align_sim::world::halves`)."""
  scale = math.sqrt(2.0) / radius_um
  at_center = math.erf(scale * center_um)
  positive = 0.5 * (math.erf(scale * (aperture_half_um - center_um)) + at_center)
  negative = 0.5 * (math.erf(scale * (aperture_half_um + center_um)) - at_center)
  return negative, positive


class Machine:
  """One machine in a scenario: screws that move, and a beam that is traced whenever it is looked at."""

  def __init__(self, scenario: Scenario):
    self.scenario = scenario
    self.screws = {axis: Screw() for axis in AXES}
    self.rng = np.random.default_rng(scenario.seed)
    errors = scenario.errors
    urad = 1e-6
    # The travel directions, as built.
    self.y_travel = _rotation(np.array([1.0, 0.0, 0.0]), errors.y_pitch_urad * urad) @ np.array([0.0, 1.0, 0.0])
    x_travel = _rotation(UP, errors.racking_urad * urad) @ np.array([1.0, 0.0, 0.0])
    x_side = unit(np.cross(UP, x_travel))
    self.x_travel = _rotation(x_side, -errors.x_pitch_urad * urad) @ x_travel
    # The tube's beam: along -X as drawn, turned by its pointing error.
    tube_direction = np.array([-1.0, 0.0, 0.0])
    tube_direction = _rotation(UP, errors.tube_h_urad * urad) @ tube_direction
    tube_side = unit(np.cross(UP, tube_direction))
    self.tube_direction = unit(_rotation(tube_side, -errors.tube_v_urad * urad) @ tube_direction)
    self.tube_origin = -scenario.layout.tube_mm * self.tube_direction
    lever = scenario.mechanics.lever_mm
    self.mount1 = Mount(np.zeros(3), unit(np.array([1.0, 1.0, 0.0])), lever)
    # Mirror 2's mount as drawn when the gantry is at mirror 1; it rides the Y travel.
    self.mount2_home = Mount(np.zeros(3), unit(np.array([1.0, -1.0, 0.0])), lever)
    # A plate set wrong is a plate whose screw tips sit off the drawing: tilt times lever arm.
    um_per_urad = lever * 1e-3
    self.mount_error_um = {
      "m1h": errors.m1_h_urad * um_per_urad,
      "m1v": errors.m1_v_urad * um_per_urad,
      "m2h": errors.m2_h_urad * um_per_urad,
      "m2v": errors.m2_v_urad * um_per_urad
    }

  # --- the screws ---

  def move(self, axis: str, delta_steps: int) -> int:
    """Command a move; the node's count after it."""
    screw = self.screws[axis]
    screw.move(delta_steps, self.scenario.screws, self.rng)
    return screw.commanded

  def seats(self) -> dict[str, float]:
    return {axis: float(screw.seat) for axis, screw in self.screws.items()}

  # --- the optics ---

  def trace(self, ly_mm: float, lx_mm: float, seats: dict[str, float] | None = None) -> Trace:
    """The beam with the gantry at `ly_mm` from mirror 1 and the head at `lx_mm` from mirror 2, with the screws'
    seats at `seats` (steps), or where they are."""
    seats = self.seats() if seats is None else seats
    tip = {axis: seats[axis] * self.scenario.mechanics.um_per_step + self.mount_error_um[axis] for axis in AXES}
    radius_mm = self.scenario.mechanics.mirror_radius_mm
    lost = False

    point1, normal1 = self.mount1.surface(tip["m1h"], tip["m1v"])
    hit1 = meet_plane(self.tube_origin, self.tube_direction, point1, normal1)
    lost |= bool(np.linalg.norm(hit1 - self.mount1.center) > radius_mm)
    leg1 = reflect(self.tube_direction, normal1)

    shift = ly_mm * self.y_travel
    mount2 = Mount(self.mount2_home.center + shift, self.mount2_home.normal, self.mount2_home.lever_mm)
    point2, normal2 = mount2.surface(tip["m2h"], tip["m2v"])
    hit2 = meet_plane(hit1, leg1, point2, normal2)
    lost |= bool(np.linalg.norm(hit2 - mount2.center) > radius_mm)
    leg2 = reflect(leg1, normal2)

    h_axis, v_axis = self.sensor_axes()
    errors = self.scenario.errors
    center = mount2.center + lx_mm * self.x_travel + 1e-3 * (errors.sensor_h_um * h_axis + errors.sensor_v_um * v_axis)
    hit3 = meet_plane(hit2, leg2, center, self.x_travel)
    offset = hit3 - center
    beam = self.scenario.beam
    layout = self.scenario.layout
    beyond_a_mm = (ly_mm + lx_mm) - (layout.ly_near_mm + layout.lx_near_mm)
    return Trace(
      points=np.array([self.tube_origin, hit1, hit2, hit3]),
      spot_um=(float(np.dot(offset, h_axis) * 1e3), float(np.dot(offset, v_axis) * 1e3)),
      radius_um=beam.radius_um + beam.divergence_urad * beyond_a_mm * 1e-3,
      lost=lost
    )

  def sensor_axes(self) -> tuple[np.ndarray, np.ndarray]:
    """The sensor's h and v: across the X travel horizontally, and up."""
    h_axis = unit(np.cross(UP, self.x_travel))
    return h_axis, np.cross(self.x_travel, h_axis)

  def traces(self, seats: dict[str, float] | None = None) -> list[Trace]:
    return [self.trace(ly_mm, lx_mm, seats) for ly_mm, lx_mm in self.scenario.layout.points()]

  def quadrants(self, trace: Trace, noisy: bool) -> list[float]:
    """The sensor's four quadrants, a, b, c, d, for the beam of `trace`."""
    beam = self.scenario.beam
    if trace.lost:
      fractions = [0.0] * 4
    else:
      left, right = _halves(trace.spot_um[0], trace.radius_um, beam.aperture_half_um)
      bottom, top = _halves(trace.spot_um[1], trace.radius_um, beam.aperture_half_um)
      fractions = [right * top, left * top, left * bottom, right * bottom]
    # One sigma on a quadrant, for the asked sigma of a normalized coordinate: `Beam::with_position_noise`.
    sigma = beam.noise_norm * beam.sum_ref / 2.0 if noisy else 0.0
    noise = [sigma * self.rng.standard_normal() if sigma > 0.0 else 0.0 for _ in fractions]
    return [beam.sum_ref * fraction + error for fraction, error in zip(fractions, noise, strict=True)]

  # --- what the aligner works toward ---

  def readings(self, seats: dict[str, float] | None = None) -> list[tuple[float, float, float]]:
    """The noise-free normalized readings at A, B, and C: h, v, and the sum."""
    result = []
    for trace in self.traces(seats):
      a, b, c, d = self.quadrants(trace, noisy=False)
      total = a + b + c + d
      if total <= 0.0:
        result.append((0.0, 0.0, 0.0))
      else:
        result.append((((a + d) - (b + c)) / total, ((a + b) - (c + d)) / total, total))
    return result

  def residual(self, seats: dict[str, float] | None = None) -> np.ndarray:
    """The three-position difference vector `m = [hB - hA, vB - vA, hC - hB, vC - vB]`, noise-free."""
    (ha, va, _), (hb, vb, _), (hc, vc, _) = self.readings(seats)
    return np.array([hb - ha, vb - va, hc - hb, vc - vb])

  def jacobian(self, seats: dict[str, float] | None = None, step: float = 2.0) -> np.ndarray:
    """How `m` changes per step of each screw's seat, by central differences: the truth the aligner's poking
    estimates."""
    seats = self.seats() if seats is None else seats
    columns = []
    for axis in AXES:
      plus = {**seats, axis: seats[axis] + step}
      minus = {**seats, axis: seats[axis] - step}
      columns.append((self.residual(plus) - self.residual(minus)) / (2.0 * step))
    return np.array(columns).T

  def aligned_seats(self, start: dict[str, float] | None = None) -> dict[str, float]:
    """The seat positions, in steps, at which `m` is zero: solved on this model by Newton's method."""
    seats = dict(self.seats() if start is None else start)
    for _ in range(20):
      residual = self.residual(seats)
      if np.max(np.abs(residual)) < 1e-9:
        break
      correction = np.linalg.solve(self.jacobian(seats), -residual)
      seats = {axis: seats[axis] + float(delta) for axis, delta in zip(AXES, correction, strict=True)}
    return seats

  def leg_angles_urad(self) -> dict[str, float]:
    """How far each leg of the beam points from the travel it should follow: leg 1 from the Y travel, leg 2 from the
    X travel. All zero is aligned, whatever the head offset."""
    traces = self.traces()
    points = traces[0].points
    leg1 = unit(points[2] - points[1])
    trace_c = traces[2].points
    leg2 = unit(trace_c[3] - trace_c[2])
    leg1_h, leg1_v = angle_between_urad(leg1, self.y_travel)
    leg2_h, leg2_v = angle_between_urad(leg2, self.x_travel)
    return {"leg1_h": leg1_h, "leg1_v": leg1_v, "leg2_h": leg2_h, "leg2_v": leg2_v}
