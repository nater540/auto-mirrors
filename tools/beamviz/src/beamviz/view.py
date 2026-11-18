"""The viewer: the run, one frame per move or measurement.

- Top view: the beam at A, B, and C, its sideways error exaggerated so that microradians show. Aligned, each path
  runs along its travel line, so the spot stays put as the gantry and the head move.
- Sensor: where the beam lands at A, B, and C. The aligner drives the three spots together; where they meet is the
  head offset, which no tilt removes.
- Side view: the beam's height along its path, in micrometers, for A, B, and C.
- Residual: the largest component of `m`, true and as measured, against the tolerance.
- Screws: each screw's count, against the positions that zero `m` exactly on this model.

The scales come from the machine as built: the pokes of the calibration swing the beam well past them.
"""

from dataclasses import dataclass

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import Slider

from beamviz.link import Event, Run
from beamviz.machine import AXES, POINTS
from beamviz.optics import UP, unit

COLORS = ("#1f77b4", "#d62728", "#2ca02c")
AXIS_COLORS = ("#1f77b4", "#ff7f0e", "#2ca02c", "#9467bd")
TOLERANCE_NORM = 0.02
"""`align_core::config::Solver::converge_tol_norm` at its starting value (hand-off 3.2)."""


@dataclass(frozen=True)
class Frame:
  """The fixed references the paths are drawn against, for one gantry position: the mirror centers as drawn and the
  head's beam line, on the travel lines."""

  mirror1: np.ndarray
  mirror2: np.ndarray
  head: np.ndarray

  def references(self) -> np.ndarray:
    return np.array([self.mirror1, self.mirror2, self.head])


def _round_down(value: float) -> float:
  """The largest of 1, 2, or 5 times a power of ten that is not above `value`."""
  magnitude = 10 ** np.floor(np.log10(value))
  return float(max(step for step in (1.0, 2.0, 5.0) if step * magnitude <= value) * magnitude)


class View:
  def __init__(self, run: Run, interactive: bool):
    self.run = run
    machine = run.machine
    self.frames = []
    for ly_mm, lx_mm in run.scenario.layout.points():
      mirror2 = ly_mm * machine.y_travel
      self.frames.append(Frame(np.zeros(3), mirror2, mirror2 + lx_mm * machine.x_travel))
    start = run.events[0]
    layout = run.scenario.layout
    sideways_mm = max(
      float(np.max(np.abs(path[1:] - frame.references())[:, :2]))
      for path, frame in zip(start.paths, self.frames, strict=True)
    )
    self.exaggeration = max(1.0, _round_down(0.08 * max(layout.ly_far_mm, layout.lx_far_mm) / max(sideways_mm, 1e-6)))

    self.figure = plt.figure(figsize=(16, 9.5))
    grid = self.figure.add_gridspec(
      2, 3, height_ratios=(1.25, 1), left=0.05, right=0.98, top=0.92, bottom=0.17, hspace=0.3, wspace=0.22
    )
    self.top = self.figure.add_subplot(grid[0, 0:2])
    self.sensor = self.figure.add_subplot(grid[0, 2])
    self.side = self.figure.add_subplot(grid[1, 0])
    self.residual = self.figure.add_subplot(grid[1, 1])
    self.screws = self.figure.add_subplot(grid[1, 2])
    self._draw_top()
    self._draw_sensor()
    self._draw_side()
    self._draw_residual()
    self._draw_screws()
    self.title = self.figure.suptitle("", fontsize=13)
    self.summary = self.figure.text(0.5, 0.075, "", ha="center", va="top", fontsize=10, family="monospace")
    self.slider = None
    if interactive:
      slider_axes = self.figure.add_axes((0.12, 0.015, 0.76, 0.025))
      self.slider = Slider(slider_axes, "event", 0, len(run.events) - 1, valinit=0, valstep=1)
      self.slider.on_changed(lambda value: self.draw(int(value)))
      self.figure.canvas.mpl_connect("key_press_event", self._on_key)
    self.draw(0)

  # --- geometry of the drawing ---

  def _exaggerated(self, path: np.ndarray, frame: Frame) -> np.ndarray:
    """The path in the top view: each point at its reference, plus `exaggeration` times its offset from it."""
    k = self.exaggeration
    references = frame.references()
    pairs = zip(path[1:], references, strict=True)
    points = [path[0], *(reference + k * (point - reference) for point, reference in pairs)]
    return np.array(points)[:, :2]

  def _heights_um(self, path: np.ndarray, frame: Frame) -> tuple[np.ndarray, np.ndarray]:
    """Distance along the path from mirror 1, in millimeters, and the beam's height off the travel lines, in
    micrometers, at mirror 1, mirror 2, and the sensor."""
    machine = self.run.machine
    height1 = unit(np.cross(machine.y_travel, unit(np.cross(UP, machine.y_travel))))
    _, height2 = machine.sensor_axes()
    ly_mm = np.linalg.norm(frame.mirror2)
    lx_mm = np.linalg.norm(frame.head - frame.mirror2)
    offsets = path[1:] - frame.references()
    heights = [np.dot(offsets[0], height1), np.dot(offsets[1], height2), np.dot(offsets[2], height2)]
    return np.array([0.0, ly_mm, ly_mm + lx_mm]), np.array(heights) * 1e3

  # --- the parts that do not change ---

  def _draw_top(self) -> None:
    run = self.run
    layout = run.scenario.layout
    machine = run.machine
    top = self.top
    far = layout.ly_far_mm * machine.y_travel
    top.plot([0.0, far[0]], [0.0, far[1]], color="0.88", lw=4, solid_capstyle="round", zorder=0)
    for ly_mm in (layout.ly_near_mm, layout.ly_far_mm):
      start = ly_mm * machine.y_travel
      end = start + (layout.lx_far_mm + 60.0) * machine.x_travel
      top.plot([start[0], end[0]], [start[1], end[1]], color="0.88", lw=4, solid_capstyle="round", zorder=0)
    tube_end = machine.tube_origin
    top.plot([tube_end[0], tube_end[0] + 120.0], [0.0, 0.0], color="0.3", lw=9, solid_capstyle="butt")
    top.annotate("tube", (tube_end[0] + 60.0, 0.0), xytext=(0, -16), textcoords="offset points", ha="center",
                 fontsize=9)
    top.plot(*self._mirror_segment(machine.mount1.center, machine.mount1.normal), color="0.2", lw=3)
    top.annotate("mirror 1", (0.0, 0.0), xytext=(-8, -16), textcoords="offset points", ha="right", fontsize=9)
    for ly_mm in (layout.ly_near_mm, layout.ly_far_mm):
      top.plot(*self._mirror_segment(ly_mm * machine.y_travel, machine.mount2_home.normal), color="0.2", lw=3)
    top.annotate("mirror 2, on the gantry at Y near and Y far", (0.0, layout.ly_far_mm), xytext=(-8, 14),
                 textcoords="offset points", ha="left", fontsize=9)
    for index, (name, (ly_mm, lx_mm)) in enumerate(zip(POINTS, layout.points(), strict=True)):
      head = ly_mm * machine.y_travel + lx_mm * machine.x_travel
      top.plot(*head[:2], "s", color=COLORS[index], ms=7, mfc="white", zorder=4)
      top.annotate(name, head[:2], xytext=(8, -16), textcoords="offset points", fontsize=11, weight="bold",
                   color=COLORS[index])
    top.set_aspect("equal")
    top.set_xlim(-140.0, max(layout.lx_far_mm, tube_end[0] + 120.0) + 80.0)
    top.set_ylim(-60.0, layout.ly_far_mm + 70.0)
    top.set_xlabel("X (mm)")
    top.set_ylabel("Y (mm)")
    top.set_title(f"Top view: the beam at A, B, C; sideways error drawn x{self.exaggeration:g}", fontsize=11)
    self.paths_top = [
      top.plot([], [], color=color, lw=2, label=f"beam at {name}", zorder=3)[0]
      for name, color in zip(POINTS, COLORS, strict=True)
    ]
    top.legend(loc="lower right", bbox_to_anchor=(1.0, 0.12), fontsize=9)
    self.angles_text = top.text(0.22, 0.62, "", transform=top.transAxes, va="top", family="monospace", fontsize=9,
                                bbox={"boxstyle": "round", "fc": "white", "ec": "0.8"})

  def _draw_sensor(self) -> None:
    run = self.run
    sensor = self.sensor
    start_spots = np.array(run.events[0].spots_um)
    span = max(2.0 * float(np.max(np.abs(start_spots))), 100.0)
    sensor.axhline(0.0, color="0.85", lw=1)
    sensor.axvline(0.0, color="0.85", lw=1)
    sensor.set_xlim(-span, span)
    sensor.set_ylim(-span, span)
    sensor.set_aspect("equal")
    sensor.set_xlabel("h (um)")
    sensor.set_ylabel("v (um)")
    sensor.set_title(f"Sensor: spot centers (beam radius {run.scenario.beam.radius_um / 1000:g} mm)", fontsize=11)
    self.trails = [sensor.plot([], [], "-", color=color, lw=0.8, alpha=0.4)[0] for color in COLORS]
    self.spot_marks = [
      sensor.plot([], [], "o", color=color, ms=9, label=name)[0] for name, color in zip(POINTS, COLORS, strict=True)
    ]
    self.spot_line = sensor.plot([], [], "-", color="0.4", lw=1)[0]
    sensor.legend(loc="upper right", fontsize=9, title="now; faint: at each\nmeasurement", title_fontsize=8)
    self.spot_text = sensor.text(0.02, 0.02, "", transform=sensor.transAxes, family="monospace", fontsize=9)

  def _draw_side(self) -> None:
    run = self.run
    layout = run.scenario.layout
    side = self.side
    pairs = zip(run.events[0].paths, self.frames, strict=True)
    start_heights = [self._heights_um(path, frame)[1] for path, frame in pairs]
    reach = max(2.0 * float(np.max(np.abs(start_heights))), 50.0)
    side.set_ylim(-reach, reach)
    side.set_xlim(-30.0, layout.ly_far_mm + layout.lx_far_mm + 30.0)
    side.axhline(0.0, color="0.85", lw=1)
    side.set_xlabel("path from mirror 1 (mm)")
    side.set_ylabel("height off the travel lines (um)")
    side.set_title("Side view: the beam's height along its path", fontsize=11)
    self.paths_side = [side.plot([], [], "-o", color=color, ms=3, lw=1.5)[0] for color in COLORS]

  def _draw_residual(self) -> None:
    events = self.run.events
    residual = self.residual
    index = np.arange(len(events))
    true_norms = np.array([np.max(np.abs(event.residual_true)) for event in events])
    residual.step(index, true_norms, where="post", color="0.3", lw=1.5, label="true (this model)")
    measured = [
      (i, np.max(np.abs(event.residual_measured)))
      for i, event in enumerate(events)
      if event.residual_measured is not None
    ]
    if measured:
      residual.plot(*zip(*measured, strict=True), "o", color="#d62728", ms=5, label="measured (what the aligner saw)")
    residual.axhline(TOLERANCE_NORM, color="#2ca02c", ls="--", lw=1, label="tolerance")
    residual.set_yscale("log")
    residual.set_xlabel("event")
    residual.set_ylabel("largest |m| (normalized)")
    residual.set_title("Residual: the three-position difference vector m", fontsize=11)
    residual.legend(fontsize=8, loc="lower left")
    self.residual_cursor = residual.axvline(0, color="0.6", lw=1)

  def _draw_screws(self) -> None:
    run = self.run
    events = run.events
    screws = self.screws
    index = np.arange(len(events))
    for axis, color in zip(AXES, AXIS_COLORS, strict=True):
      screws.step(index, [event.commanded[axis] for event in events], where="post", color=color, lw=1.5, label=axis)
      screws.axhline(run.aligned_seats[axis], color=color, ls=":", lw=1.2)
    screws.set_xlabel("event")
    screws.set_ylabel("screw count (steps)")
    screws.set_title("Screws; dotted: where m is exactly zero", fontsize=11)
    screws.legend(fontsize=9, ncols=4, loc="lower left")
    self.screws_cursor = screws.axvline(0, color="0.6", lw=1)

  def _mirror_segment(self, center: np.ndarray, normal: np.ndarray) -> tuple[list[float], list[float]]:
    across = np.cross(UP, normal) * 22.0
    return [center[0] - across[0], center[0] + across[0]], [center[1] - across[1], center[1] + across[1]]

  # --- one frame ---

  def draw(self, index: int) -> None:
    run = self.run
    event = run.events[index]
    for line, path, frame in zip(self.paths_top, event.paths, self.frames, strict=True):
      points = self._exaggerated(path, frame)
      line.set_data(points[:, 0], points[:, 1])
    for line, path, frame in zip(self.paths_side, event.paths, self.frames, strict=True):
      line.set_data(*self._heights_um(path, frame))
    seen = [past for past in run.events[: index + 1] if past.kind != "move"]
    for point, trail in enumerate(self.trails):
      history = np.array([past.spots_um[point] for past in seen])
      trail.set_data(history[:, 0], history[:, 1])
    for mark, spot in zip(self.spot_marks, event.spots_um, strict=True):
      mark.set_data([spot[0]], [spot[1]])
    spots = np.array(event.spots_um)
    self.spot_line.set_data(spots[:, 0], spots[:, 1])
    self.spot_text.set_text(
      f"A to B (Y travel): {np.hypot(*(spots[1] - spots[0])):6.1f} um\n"
      f"B to C (X travel): {np.hypot(*(spots[2] - spots[1])):6.1f} um"
    )
    self.angles_text.set_text(self._angles(event))
    self.residual_cursor.set_xdata([index, index])
    self.screws_cursor.set_xdata([index, index])
    self.title.set_text(f"{index + 1}/{len(run.events)}   {event.label}")
    self.summary.set_text(self._summary() if index == len(run.events) - 1 else "")
    self.figure.canvas.draw_idle()

  @staticmethod
  def _angles(event: Event) -> str:
    angles = event.angles_urad
    return (
      "beam off its travel (urad)       horizontal  vertical\n"
      f"leg 1, mirror 1 to 2, vs Y        {angles['leg1_h']:+9.1f} {angles['leg1_v']:+9.1f}\n"
      f"leg 2, mirror 2 to head, vs X     {angles['leg2_h']:+9.1f} {angles['leg2_v']:+9.1f}"
    )

  def _summary(self) -> str:
    run = self.run
    text = f"aligner: {run.report}"
    if run.calibration is not None and run.true_jacobian_start is not None:
      exact = run.true_jacobian_start
      errors = np.linalg.norm(run.calibration - exact, axis=0) / np.linalg.norm(exact, axis=0)
      text += "\nJacobian measured by poking, off the exact one by (per column): "
      text += "  ".join(f"{axis} {error:.0%}" for axis, error in zip(AXES, errors, strict=True))
    return text

  def _on_key(self, key_event) -> None:
    if self.slider is None:
      return
    step = {"right": 1, "left": -1}.get(key_event.key)
    if step is not None:
      self.slider.set_val(int(np.clip(self.slider.val + step, 0, len(self.run.events) - 1)))

  # --- output ---

  def save_animation(self, path: str, fps: float) -> None:
    animation = FuncAnimation(self.figure, self.draw, frames=len(self.run.events), interval=1000.0 / fps)
    animation.save(path, writer=PillowWriter(fps=fps))

  def save_frame(self, index: int, path: str) -> None:
    self.draw(index)
    self.figure.savefig(path, dpi=100)
