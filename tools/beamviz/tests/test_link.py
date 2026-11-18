"""The aligner of `align-core` against the ray-traced machine, through `align-sim external`, and the viewer on its run.

The run needs the simulator built (`just beamviz` builds it); without it these tests are skipped, and say so.
"""

from pathlib import Path

import matplotlib
import numpy as np
import pytest

from beamviz.link import ALIGN_SIM, _measurement_label, run
from beamviz.machine import Scenario

matplotlib.use("Agg")

needs_simulator = pytest.mark.skipif(not ALIGN_SIM.exists(), reason=f"{ALIGN_SIM} is not built")


def test_a_measurement_is_labeled_by_what_moved_since_the_last():
  zero = dict.fromkeys(("m1h", "m1v", "m2h", "m2v"), 0)
  assert _measurement_label(1, zero, zero).startswith("measurement 1: every screw engaged")
  assert _measurement_label(4, zero, zero) == "measurement 4: nothing moved since the last"
  moved = {**zero, "m1h": -23, "m2v": 1}
  assert _measurement_label(7, zero, moved) == "measurement 7: since the last, m1h -23, m2v +1"


def test_a_missing_simulator_is_named():
  with pytest.raises(FileNotFoundError, match="is not built"):
    run(Scenario(), Path("/nonexistent/align-sim"))


@needs_simulator
def test_the_aligner_converges_on_the_ray_traced_machine():
  result = run(Scenario())
  assert result.converged, result.report
  first, last = result.events[0], result.events[-1]
  assert np.max(np.abs(first.residual_true)) > 0.05
  assert np.max(np.abs(last.residual_true)) < 0.03
  assert result.calibration is not None
  # Each screw ends within a few steps of the exact solution: the tolerance allows that much.
  for axis, count in last.commanded.items():
    assert abs(count - result.aligned_seats[axis]) < 8, axis
  kinds = {event.kind for event in result.events}
  assert kinds == {"start", "move", "measure"}


@needs_simulator
def test_the_viewer_draws_the_first_and_the_last_frame(tmp_path):
  from beamviz.view import View  # noqa: PLC0415 - after the backend is chosen

  result = run(Scenario())
  view = View(result, interactive=False)
  assert view.exaggeration >= 1.0
  for index in (0, len(result.events) - 1):
    path = tmp_path / f"frame-{index}.png"
    view.save_frame(index, str(path))
    assert path.stat().st_size > 10_000
