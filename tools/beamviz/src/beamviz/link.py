"""Runs the aligner of `align-core` against a `Machine`. `align-sim external` asks for every move and measurement on
its standard output, one line each (`align_sim::external`); this answers each from the ray-traced machine and
records the machine as it stands after each one, for the viewer."""

import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from beamviz.machine import AXES, Machine, Scenario

REPOSITORY = Path(__file__).resolve().parents[4]
ALIGN_SIM = REPOSITORY / "target" / "release" / "align-sim"


@dataclass(frozen=True)
class Event:
  """The machine after one thing the aligner asked for."""

  kind: str
  """`start`, `move`, or `measure`."""
  label: str
  commanded: dict[str, int]
  seats: dict[str, float]
  paths: list[np.ndarray]
  """The beam's path at A, B, and C: tube, mirror 1, mirror 2, sensor."""
  spots_um: list[tuple[float, float]]
  """Where the beam's center lands on the sensor at A, B, and C."""
  residual_true: np.ndarray
  angles_urad: dict[str, float]
  residual_measured: np.ndarray | None = None
  """What the aligner saw, from the noisy quadrants: measurements only."""


@dataclass
class Run:
  scenario: Scenario
  machine: Machine
  events: list[Event] = field(default_factory=list)
  report: str = ""
  calibration: np.ndarray | None = None
  """The Jacobian the aligner measured by poking, or `None` if it measured none."""
  true_jacobian_start: np.ndarray | None = None
  aligned_seats: dict[str, float] = field(default_factory=dict)
  """Where the seats must be for `m` to be zero, solved exactly on the model before the run."""

  @property
  def converged(self) -> bool:
    return self.report.startswith("converged")


def _normalized(quadrants: list[float]) -> tuple[float, float]:
  a, b, c, d = quadrants
  total = a + b + c + d
  if total <= 0.0:
    return 0.0, 0.0
  return ((a + d) - (b + c)) / total, ((a + b) - (c + d)) / total


def _snapshot(machine: Machine, kind: str, label: str, measured: np.ndarray | None = None) -> Event:
  traces = machine.traces()
  return Event(
    kind=kind,
    label=label,
    commanded={axis: screw.commanded for axis, screw in machine.screws.items()},
    seats=machine.seats(),
    paths=[trace.points for trace in traces],
    spots_um=[trace.spot_um for trace in traces],
    residual_true=machine.residual(),
    angles_urad=machine.leg_angles_urad(),
    residual_measured=measured
  )


def _measurement_label(number: int, before: dict[str, int], now: dict[str, int]) -> str:
  """What a measurement looks at: how the screws' counts changed since the last one."""
  changes = [f"{axis} {now[axis] - before[axis]:+d}" for axis in AXES if now[axis] != before[axis]]
  if number == 1:
    return "measurement 1: every screw engaged from below, the starting point"
  if not changes:
    return f"measurement {number}: nothing moved since the last"
  return f"measurement {number}: since the last, " + ", ".join(changes)


def run(scenario: Scenario, align_sim: Path = ALIGN_SIM) -> Run:
  """One alignment of a fresh machine in `scenario`, every step recorded."""
  if not align_sim.exists():
    raise FileNotFoundError(f"{align_sim} is not built: `just beamviz` builds it, as does `cargo build --release`")
  machine = Machine(scenario)
  result = Run(scenario=scenario, machine=machine)
  result.true_jacobian_start = machine.jacobian()
  result.aligned_seats = machine.aligned_seats()
  result.events.append(_snapshot(machine, "start", "start: the machine as built, every screw at zero"))
  command = [
    str(align_sim),
    "external",
    "--sum-ref",
    str(scenario.beam.sum_ref),
    "--beam-radius",
    str(scenario.beam.radius_um),
    "--backlash",
    str(scenario.screws.backlash_steps)
  ]
  with subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1) as process:
    assert process.stdin is not None and process.stdout is not None
    measurements = 0
    counts_at_last_measurement = {axis: 0 for axis in AXES}
    for line in process.stdout:
      words = line.split()
      match words:
        case ["move", axis, delta] if axis in AXES:
          count = machine.move(axis, int(delta))
          process.stdin.write(f"done {count}\n")
          process.stdin.flush()
          result.events.append(_snapshot(machine, "move", f"move {axis} {int(delta):+d} (count {count:+d})"))
        case ["measure"]:
          measurements += 1
          quadrants = [machine.quadrants(trace, noisy=True) for trace in machine.traces()]
          process.stdin.write("shots " + " ".join(f"{value:.9g}" for shot in quadrants for value in shot) + "\n")
          process.stdin.flush()
          (ha, va), (hb, vb), (hc, vc) = (_normalized(shot) for shot in quadrants)
          measured = np.array([hb - ha, vb - va, hc - hb, vc - vb])
          counts = {axis: screw.commanded for axis, screw in machine.screws.items()}
          label = _measurement_label(measurements, counts_at_last_measurement, counts)
          result.events.append(_snapshot(machine, "measure", label, measured))
          counts_at_last_measurement = counts
        case ["report", *rest]:
          result.report = " ".join(rest)
        case ["calibration", "none"]:
          result.calibration = None
        case ["calibration", *values] if len(values) == 16:
          result.calibration = np.array([float(value) for value in values]).reshape(4, 4)
        case ["end"]:
          break
        case _:
          raise RuntimeError(f"align-sim said something this does not understand: {line!r}")
    process.stdin.close()
    if process.wait() != 0:
      raise RuntimeError(f"align-sim ended with status {process.returncode}")
  return result
