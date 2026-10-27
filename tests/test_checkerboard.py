"""Tests for checkerboard.py. Offline only, no hardware, no camera."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import cv2
import numpy

from muse_qc.checkerboard import (
    CHECKER_INNER_CORNERS,
    CHECKER_SQUARES,
    CHECKER_SQUARE_MM,
    build_svg,
)

SVG_NS = "{http://www.w3.org/2000/svg}"


def _parse(text):
    return ElementTree.fromstring(text)


def _millimetres(value, name):
    assert value.endswith("mm"), name
    return float(value[:-2])


def test_svg_is_valid_xml_with_a4_landscape_size_in_mm():
    root = _parse(build_svg())
    assert root.tag == f"{SVG_NS}svg"
    assert _millimetres(root.attrib["width"], "width") == 297.0
    assert _millimetres(root.attrib["height"], "height") == 210.0
    assert root.attrib["viewBox"] == "0 0 297.0 210.0"


def test_svg_has_35_black_squares_of_exactly_25mm():
    assert CHECKER_SQUARE_MM == 25.0
    assert tuple(CHECKER_SQUARES) == (10, 7)
    assert tuple(CHECKER_INNER_CORNERS) == (9, 6)
    root = _parse(build_svg())
    black = [
        rect
        for rect in root.findall(f".//{SVG_NS}rect")
        if rect.attrib.get("fill") == "black"
        and float(rect.attrib["width"]) == 25.0
        and float(rect.attrib["height"]) == 25.0
    ]
    assert len(black) == 35
    first = min(black, key=lambda rect: (float(rect.attrib["y"]), float(rect.attrib["x"])))
    top = float(first.attrib["y"])
    left = float(first.attrib["x"])
    board_top = min(float(rect.attrib["y"]) for rect in black)
    board_left = min(float(rect.attrib["x"]) for rect in black)
    assert top == board_top
    assert left == board_left


def test_board_is_centred_with_margin():
    root = _parse(build_svg())
    black = [
        rect
        for rect in root.findall(f".//{SVG_NS}rect")
        if rect.attrib.get("fill") == "black"
    ]
    left = min(float(rect.attrib["x"]) for rect in black)
    top = min(float(rect.attrib["y"]) for rect in black)
    right = max(
        float(rect.attrib["x"]) + float(rect.attrib["width"]) for rect in black
    )
    bottom = max(
        float(rect.attrib["y"]) + float(rect.attrib["height"]) for rect in black
    )
    assert left >= 10.0
    assert top >= 10.0
    assert 297.0 - right >= 10.0
    assert 210.0 - bottom >= 10.0
    assert abs(left - (297.0 - right)) < 1.0
    assert abs(top - (210.0 - bottom)) < 1.0


def test_svg_has_100mm_scale_line_and_text():
    root = _parse(build_svg())
    line = root.find(f"{SVG_NS}line[@id='scale-line']")
    assert line is not None
    length = abs(float(line.attrib["x2"]) - float(line.attrib["x1"]))
    assert length == 100.0
    assert "25 mm squares, print at 100 % / actual size, glue flat on cardboard" in build_svg()


def _render_board(px_per_mm=8, quiet_mm=10.0):
    columns, rows = CHECKER_SQUARES
    quiet_px = int(round(quiet_mm * px_per_mm))
    cell_px = int(round(CHECKER_SQUARE_MM * px_per_mm))
    height = rows * cell_px + 2 * quiet_px
    width = columns * cell_px + 2 * quiet_px
    image = numpy.full((height, width), 255, dtype=numpy.uint8)
    for row in range(rows):
        for column in range(columns):
            if (column + row) % 2 != 0:
                continue
            image[
                quiet_px + row * cell_px : quiet_px + (row + 1) * cell_px,
                quiet_px + column * cell_px : quiet_px + (column + 1) * cell_px,
            ] = 0
    return image


def test_rendered_board_finds_inner_corners():
    image = _render_board()
    found, corners = cv2.findChessboardCorners(image, CHECKER_INNER_CORNERS)
    assert found
    assert corners.shape[0] == CHECKER_INNER_CORNERS[0] * CHECKER_INNER_CORNERS[1]


def test_only_squares_scale_line_and_text_are_black():
    root = _parse(build_svg())
    squares = {
        id(rect)
        for rect in root.findall(f".//{SVG_NS}rect")
        if rect.attrib.get("fill") == "black"
        and float(rect.attrib["width"]) == 25.0
        and float(rect.attrib["height"]) == 25.0
    }
    assert len(squares) == 35
    for element in root.iter():
        if id(element) in squares:
            continue
        if element.tag == f"{SVG_NS}text":
            continue
        if (
            element.tag == f"{SVG_NS}line"
            and element.attrib.get("id") == "scale-line"
        ):
            continue
        assert element.attrib.get("fill") != "black"
        assert element.attrib.get("stroke") != "black"
    texts = root.findall(f".//{SVG_NS}text")
    assert texts
    for text in texts:
        x = float(text.attrib["x"])
        y = float(text.attrib["y"])
        assert x >= 8.0
        assert y >= 8.0
        assert x <= 297.0 - 8.0
        assert y <= 210.0 - 8.0


def test_committed_svg_matches_generator():
    committed = (
        Path(__file__).resolve().parent.parent / "docs" / "checkerboard.svg"
    )
    hint = "run uv run python -m muse_qc.checkerboard and commit the file"
    assert committed.is_file(), hint
    assert committed.read_text(encoding="utf-8") == build_svg(), hint


def test_never_imports_arm_safety_or_sdk_code():
    path = os.path.join(
        os.path.dirname(__file__), "..", "muse_qc", "checkerboard.py"
    )
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "from muse_qc.arm" not in source
    assert "from muse_qc import arm" not in source
    assert "from muse_qc.safety" not in source
    assert "from muse_qc import safety" not in source
    assert "import arm" not in source
    assert "import safety" not in source
    assert "reBotArm" not in source
    assert "motorbridge" not in source
