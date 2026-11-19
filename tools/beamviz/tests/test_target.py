"""The printable target is at exact scale: its page is the paper, and its grid's millimeters are the paper's."""

import re

import pytest

from beamviz.target import A4, LETTER, MM_PER_INCH, draw, save


@pytest.mark.parametrize("paper", [LETTER, A4])
def test_the_figure_is_the_sheet_and_one_unit_is_one_millimeter(paper):
  figure = draw(paper)
  width_in, height_in = figure.get_size_inches()
  assert width_in * MM_PER_INCH == pytest.approx(paper.width_mm)
  assert height_in * MM_PER_INCH == pytest.approx(paper.height_mm)
  axes = figure.axes[0]
  # Two points 100 mm apart in data land 100 mm apart on the page, across and down.
  to_inches = axes.transData + figure.dpi_scale_trans.inverted()
  (x0, y0), (x1, y1) = to_inches.transform([(20.0, 30.0), (120.0, 130.0)])
  assert (x1 - x0) * MM_PER_INCH == pytest.approx(100.0)
  assert (y1 - y0) * MM_PER_INCH == pytest.approx(100.0)


@pytest.mark.parametrize(("paper", "box"), [(LETTER, (612.0, 792.0)), (A4, (595.2756, 841.8898))])
def test_the_pdf_page_is_letter_or_a4_in_points(tmp_path, paper, box):
  path = tmp_path / "target.pdf"
  save(paper, path)
  media = re.search(rb"MediaBox \[ 0 0 ([0-9.]+) ([0-9.]+) \]", path.read_bytes())
  assert media is not None
  assert (float(media.group(1)), float(media.group(2))) == pytest.approx(box, abs=1e-3)
