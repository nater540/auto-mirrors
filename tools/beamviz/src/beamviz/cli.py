"""`beamviz`: the aligner of `align-core` against a ray-traced machine, run and shown.

Without `--save` or `--frames` it opens a window: drag the slider, or use the left and right arrow keys.
"""

import argparse
import sys
from dataclasses import fields, replace
from pathlib import Path

import matplotlib
import numpy as np

from beamviz.link import ALIGN_SIM, Run, run
from beamviz.machine import AXES, Beam, FrameErrors, Layout, Scenario, ScrewModel


def _scenario(args: argparse.Namespace) -> Scenario:
  layout = Layout()
  layout = replace(layout, ly_far_mm=layout.ly_near_mm + args.y_span, lx_far_mm=layout.lx_near_mm + args.x_span)
  defaults = FrameErrors()
  errors = FrameErrors(**{error.name: getattr(defaults, error.name) * args.errors for error in fields(FrameErrors)})
  beam = replace(Beam(), radius_um=args.beam_radius, noise_norm=args.noise)
  return Scenario(layout=layout, errors=errors, beam=beam, screws=ScrewModel(args.backlash, args.stick), seed=args.seed)


def _report(result: Run) -> str:
  first, last = result.events[0], result.events[-1]
  lines = [f"aligner: {result.report}", f"moves: {sum(event.kind == 'move' for event in result.events)}, "
           f"measurements: {sum(event.kind == 'measure' for event in result.events)}"]
  for name, event in (("start", first), ("end", last)):
    angles = "  ".join(f"{key} {value:+7.1f}" for key, value in event.angles_urad.items())
    largest = np.max(np.abs(event.residual_true))
    lines.append(f"{name}: largest true |m| {largest:.4f}; beam off its travel (urad): {angles}")
  lines.append("screws, count at the end against the exact solution: " + "  ".join(
    f"{axis} {last.commanded[axis]:+d} ({result.aligned_seats[axis]:+.1f})" for axis in AXES))
  if result.calibration is not None and result.true_jacobian_start is not None:
    lines.append("measured Jacobian (1e-4 per step), then the exact one at the start:")
    for measured, exact in zip(result.calibration, result.true_jacobian_start, strict=True):
      lines.append("  " + " ".join(f"{value * 1e4:+8.2f}" for value in measured) + "   |" +
                   " ".join(f"{value * 1e4:+8.2f}" for value in exact))
  return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(prog="beamviz", description=__doc__.splitlines()[0])
  parser.add_argument("--seed", type=int, default=1, help="seed for the sensor noise (default 1)")
  parser.add_argument("--backlash", type=int, default=20, help="play between motor and screw, steps (default 20)")
  parser.add_argument("--stick", type=int, default=3, help="screw travel a seat resists before it slips (default 3)")
  parser.add_argument("--noise", type=float, default=0.002, help="sensor noise, normalized, one sigma (default 0.002)")
  parser.add_argument("--beam-radius", type=float, default=4000.0, help="1/e^2 radius at the head, um (default 4000)")
  parser.add_argument("--y-span", type=float, default=500.0, help="gantry travel from A to B, mm (default 500)")
  parser.add_argument("--x-span", type=float, default=600.0, help="head travel from B to C, mm (default 600)")
  parser.add_argument("--errors", type=float, default=1.0, help="scale every frame error by this (default 1)")
  parser.add_argument("--save", type=Path, help="write the animation (.gif) or the last frame (.png), no window")
  parser.add_argument("--frames", type=Path, help="write every frame as a PNG into this directory")
  parser.add_argument("--fps", type=float, default=3.0, help="frames per second of a saved animation (default 3)")
  parser.add_argument("--align-sim", type=Path, default=ALIGN_SIM, help=f"the simulator binary (default {ALIGN_SIM})")
  args = parser.parse_args(argv)

  headless = args.save is not None or args.frames is not None
  if headless:
    matplotlib.use("Agg")
  from beamviz.view import View  # noqa: PLC0415 - pyplot must load after the backend is chosen

  result = run(_scenario(args), args.align_sim)
  print(_report(result))
  view = View(result, interactive=not headless)
  if args.frames is not None:
    args.frames.mkdir(parents=True, exist_ok=True)
    for index in range(len(result.events)):
      view.save_frame(index, str(args.frames / f"frame-{index:03d}.png"))
  if args.save is not None:
    if args.save.suffix == ".gif":
      view.save_animation(str(args.save), args.fps)
    else:
      view.save_frame(len(result.events) - 1, str(args.save))
  if not headless:
    from matplotlib import pyplot as plt  # noqa: PLC0415

    plt.show()
  return 0 if result.converged else 1


if __name__ == "__main__":
  sys.exit(main())
