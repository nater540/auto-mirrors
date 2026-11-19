"""A printable paper target for the bench rig (`just bench-rig`): a millimeter grid at exact scale, numbered every 10 mm
from its center, h to the right and v up, the convention the rig's readings use.

The page is drawn in millimeters on an axes that fills the figure exactly, so one unit is one millimeter on paper as
long as the PDF is printed at actual size. The sheet says so, and how to check it with a ruler.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path

import matplotlib as mpl

mpl.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

MM_PER_INCH = 25.4


@dataclass(frozen=True)
class Paper:
  """A sheet's name and size, portrait, in millimeters."""

  name: str
  width_mm: float
  height_mm: float


LETTER = Paper("letter", 215.9, 279.4)
A4 = Paper("a4", 210.0, 297.0)

# The grid's half-width and half-height: inside both sheets, with room for the numbers and the notes.
H_HALF_MM = 90
V_HALF_MM = 110

# Light enough that the dot reads through them, dark enough that a gray laser printer keeps them.
FINE = ("#c4c4c4", 0.25)
MEDIUM = ("#8a8a8a", 0.5)
BOLD = ("#3c3c3c", 0.8)
AXIS = ("#000000", 1.2)


def _lines(positions, start, end, along_h):
  """Segments at each position, from `start` to `end` across them: vertical lines when `along_h`, else horizontal."""
  if along_h:
    return [[(p, start), (p, end)] for p in positions]
  return [[(start, p), (end, p)] for p in positions]


def _weight(mm):
  if mm == 0:
    return AXIS
  if mm % 10 == 0:
    return BOLD
  if mm % 5 == 0:
    return MEDIUM
  return FINE


def draw(paper):
  """The target on `paper`, as a figure whose data units are millimeters on the sheet."""
  figure = plt.figure(figsize=(paper.width_mm / MM_PER_INCH, paper.height_mm / MM_PER_INCH))
  axes = figure.add_axes((0.0, 0.0, 1.0, 1.0))
  axes.set_xlim(0.0, paper.width_mm)
  axes.set_ylim(0.0, paper.height_mm)
  axes.set_axis_off()
  cx = paper.width_mm / 2.0
  cy = paper.height_mm / 2.0 - 4.0

  # Fine lines first, so the heavier ones are drawn over them.
  for weight in (FINE, MEDIUM, BOLD, AXIS):
    columns = [cx + h for h in range(-H_HALF_MM, H_HALF_MM + 1) if _weight(h) == weight]
    rows = [cy + v for v in range(-V_HALF_MM, V_HALF_MM + 1) if _weight(v) == weight]
    segments = _lines(columns, cy - V_HALF_MM, cy + V_HALF_MM, True)
    segments += _lines(rows, cx - H_HALF_MM, cx + H_HALF_MM, False)
    color, width = weight
    axes.add_collection(LineCollection(segments, colors=color, linewidths=width, capstyle="butt"))

  numbers = {"fontsize": 6.5, "color": "#000000"}
  for h in range(-H_HALF_MM, H_HALF_MM + 1, 10):
    for y, va in ((cy - V_HALF_MM - 1.2, "top"), (cy + V_HALF_MM + 1.2, "bottom")):
      axes.text(cx + h, y, f"{h}", ha="center", va=va, **numbers)
    if 0 < abs(h) < H_HALF_MM:
      # Along the h axis too, just under it, so a dot near the center has a number close by; not at the ends, where
      # the edge numbers are.
      axes.text(cx + h + 0.4, cy - 0.5, f"{h}", ha="left", va="top", fontsize=5, color="#303030")
  for v in range(-V_HALF_MM, V_HALF_MM + 1, 10):
    axes.text(cx - H_HALF_MM - 1.5, cy + v, f"{v}", ha="right", va="center", **numbers)
    axes.text(cx + H_HALF_MM + 1.5, cy + v, f"{v}", ha="left", va="center", **numbers)
    if 0 < abs(v) < V_HALF_MM:
      axes.text(cx - 0.5, cy + v + 0.4, f"{v}", ha="right", va="bottom", fontsize=5, color="#303030")

  axes.text(cx + H_HALF_MM, cy + 1.5, "h →", ha="right", va="bottom", fontsize=8, weight="bold")
  axes.text(cx + 1.5, cy + V_HALF_MM - 1.0, "v ↑", ha="left", va="top", fontsize=8, weight="bold")

  top = cy + V_HALF_MM + 7.0
  axes.text(
    cx,
    top + 8.0,
    "Bench target: millimeters from the center, h to the right, v up, seen from the side the beam hits",
    ha="center",
    va="bottom",
    fontsize=9,
    weight="bold",
  )
  axes.text(
    cx,
    top + 2.5,
    "Type the dot's center as  h v  (for example  12.5 -3.25). Lines every 1 mm, darker every 5 mm and 10 mm.",
    ha="center",
    va="bottom",
    fontsize=7.5,
  )
  bottom = cy - V_HALF_MM - 7.0
  notes = (
    (f"Print at actual size (100%, not fit to page) on {paper.name.upper()} paper.", 7.5, "#000000"),
    ("Check with a ruler before use: the bold lines -50 and 50 must be 100 mm apart, across and down.", 7.5, "#000000"),
    (
      "A sheet that does not measure true still brackets backlash in steps, but its millimeters are off.",
      6.5,
      "#404040"
    ),
  )
  for index, (text, size, color) in enumerate(notes):
    axes.text(cx, bottom - 1.0 - 4.5 * index, text, ha="center", va="top", fontsize=size, color=color)
  return figure


def save(paper, path):
  """Writes the target on `paper` as a PDF at `path`."""
  mpl.rcParams["pdf.fonttype"] = 42
  figure = draw(paper)
  figure.savefig(path, format="pdf")
  plt.close(figure)


def main(argv=None):
  parser = argparse.ArgumentParser(
    description="Write the bench rig's paper target as PDFs at exact scale, on Letter and A4 paper."
  )
  parser.add_argument("--out", type=Path, default=Path("target"), help="directory for the PDFs [default: target]")
  args = parser.parse_args(argv)
  args.out.mkdir(parents=True, exist_ok=True)
  for paper in (LETTER, A4):
    path = args.out / f"bench-target-{paper.name}.pdf"
    save(paper, path)
    print(path)


if __name__ == "__main__":
  main()
