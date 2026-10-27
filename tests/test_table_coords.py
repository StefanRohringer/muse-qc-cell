"""Tests for table_coords.py. Offline only, no camera, no SDK import."""

from __future__ import annotations

import math
import os

import cv2
import numpy
import pytest

import muse_qc.table_coords as table_coords
from muse_qc.table_coords import (
    draw_grid,
    marker_sides_mm,
    pixel_to_table_mm,
    table_homography,
    table_mm_to_pixel,
)
from muse_qc.table_markers import (
    TABLE_MARKER_IDS,
    TABLE_MARKER_LAYOUT_MM,
    TABLE_MARKER_SIZE_MM,
    get_dictionary,
)

# Known scale and offset: image y points down, same direction as table y.
SCALE_PX_PER_MM = 3.0
OFFSET_X = 100.0
OFFSET_Y = 80.0
CELL_PX = 20
SIDE_PX = 6 * CELL_PX
WIDTH = int(977.0 * SCALE_PX_PER_MM) + 2 * int(OFFSET_X)
HEIGHT = int(531.0 * SCALE_PX_PER_MM) + 2 * int(OFFSET_Y)

MARKER_CELLS_SIDE = 6
MEASURED_DIAGONAL_MM = 1110.0


def _cells(marker_id: int) -> numpy.ndarray:
    """Return the 6x6 cell pattern for a dictionary ID (1 is black)."""
    image = cv2.aruco.generateImageMarker(
        get_dictionary(), marker_id, MARKER_CELLS_SIDE
    )
    return (image == 0).astype(numpy.uint8)


def _draw_marker(frame: numpy.ndarray, marker_id: int, top_left: tuple) -> None:
    """Draw one marker with top-left pixel at top_left into frame."""
    cells = _cells(marker_id)
    x0, y0 = top_left
    for row in range(MARKER_CELLS_SIDE):
        for column in range(MARKER_CELLS_SIDE):
            if cells[row, column] == 1:
                frame[
                    y0 + row * CELL_PX : y0 + (row + 1) * CELL_PX,
                    x0 + column * CELL_PX : x0 + (column + 1) * CELL_PX,
                ] = 0


def _frame() -> numpy.ndarray:
    """Build a white BGR frame with markers at the layout positions."""
    frame = numpy.full((HEIGHT, WIDTH, 3), 255, dtype=numpy.uint8)
    for marker_id in TABLE_MARKER_IDS:
        x_mm, y_mm = TABLE_MARKER_LAYOUT_MM[marker_id]
        center_u = OFFSET_X + SCALE_PX_PER_MM * x_mm
        center_v = OFFSET_Y + SCALE_PX_PER_MM * y_mm
        top_left = (
            int(round(center_u - SIDE_PX / 2.0)),
            int(round(center_v - SIDE_PX / 2.0)),
        )
        _draw_marker(frame, marker_id, top_left)
    return frame


def _markers_and_homography():
    """Detect the synthetic frame and return (markers, homography)."""
    from muse_qc.detect_markers import detect_table_markers

    markers = detect_table_markers(_frame())
    return markers, table_homography(markers)


def test_centres_map_to_layout_within_1mm():
    markers, homography = _markers_and_homography()
    for marker_id in TABLE_MARKER_IDS:
        mapped = pixel_to_table_mm(homography, markers[marker_id]["center"])
        assert mapped == pytest.approx(
            TABLE_MARKER_LAYOUT_MM[marker_id], abs=1.0
        )


def test_round_trip_pixel_mm_pixel_within_half_px():
    markers, homography = _markers_and_homography()
    points = [markers[marker_id]["center"] for marker_id in TABLE_MARKER_IDS]
    points += [corner for marker_id in TABLE_MARKER_IDS for corner in markers[marker_id]["corners"]]
    points.append((WIDTH / 2.0, HEIGHT / 2.0))
    for point in points:
        back = table_mm_to_pixel(homography, pixel_to_table_mm(homography, point))
        assert back == pytest.approx(point, abs=0.5)


def test_marker_sides_about_40mm():
    markers, homography = _markers_and_homography()
    sides = marker_sides_mm(homography, markers)
    for marker_id in TABLE_MARKER_IDS:
        assert sides[marker_id] == pytest.approx(TABLE_MARKER_SIZE_MM, abs=1.5)


def test_layout_diagonal_matches_measurement():
    corner_1 = TABLE_MARKER_LAYOUT_MM[1]
    corner_2 = TABLE_MARKER_LAYOUT_MM[2]
    assert math.dist(corner_1, corner_2) == pytest.approx(
        MEASURED_DIAGONAL_MM, abs=5.0
    )


def test_draw_grid_returns_annotated_copy():
    markers, homography = _markers_and_homography()
    frame = _frame()
    grid = draw_grid(frame, homography)
    assert grid.shape == frame.shape
    assert grid is not frame
    assert numpy.any(grid != frame)


def test_main_writes_grid_png(tmp_path, monkeypatch, capsys):
    frame = _frame()
    monkeypatch.setattr(
        table_coords, "open_table_camera", lambda: (frame, {})
    )
    monkeypatch.chdir(tmp_path)
    table_coords.main()
    out = capsys.readouterr().out
    files = sorted((tmp_path / "recordings").glob("table_grid_*.png"))
    assert len(files) == 1
    assert files[0].name in out
    for marker_id in TABLE_MARKER_IDS:
        assert f"ID {marker_id}" in out
        assert "center_mm" in out
        assert "side=" in out
    assert "image center" in out
    saved = cv2.imread(str(files[0]))
    assert saved is not None
    assert (saved.shape[1], saved.shape[0]) == (WIDTH, HEIGHT)


def test_never_imports_arm_safety_or_sdk_code():
    path = os.path.join(
        os.path.dirname(__file__), "..", "muse_qc", "table_coords.py"
    )
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "from muse_qc.arm" not in source
    assert "from muse_qc import arm" not in source
    assert "import arm" not in source
    assert "from muse_qc.safety" not in source
    assert "import safety" not in source
    assert "reBotArm" not in source
    assert "motorbridge" not in source
