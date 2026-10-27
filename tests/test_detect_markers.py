"""Tests for detect_markers.py. Offline only, no camera, no SDK import."""

from __future__ import annotations

import os

import cv2
import numpy
import pytest

import muse_qc.detect_markers as detect_markers
from muse_qc.detect_markers import (
    image_corner_names,
    detect_table_markers,
    scale_px_per_mm,
)
from muse_qc.table_markers import (
    TABLE_MARKER_IDS,
    TABLE_MARKER_SIZE_MM,
    get_dictionary,
)

WIDTH = 1920
HEIGHT = 1080
CELL_PX = 25
SIDE_PX = 6 * CELL_PX
UNKNOWN_ID = 4

TOP_LEFTS = {
    0: (120, 100),
    1: (WIDTH - 120 - SIDE_PX, 100),
    2: (120, HEIGHT - 100 - SIDE_PX),
    3: (WIDTH - 120 - SIDE_PX, HEIGHT - 100 - SIDE_PX),
}
UNKNOWN_TOP_LEFT = (WIDTH // 2 - SIDE_PX // 2, HEIGHT // 2 - SIDE_PX // 2)

EXPECTED_CORNERS = {
    0: "top-left",
    1: "top-right",
    2: "bottom-left",
    3: "bottom-right",
}


MARKER_CELLS_SIDE = 6


def _cells(marker_id: int) -> numpy.ndarray:
    """Return the 6x6 cell pattern for any dictionary ID (1 is black)."""
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


def _frame(marker_ids=None, extra_markers=()) -> numpy.ndarray:
    """Build a white BGR frame with markers at the known positions."""
    if marker_ids is None:
        marker_ids = list(TABLE_MARKER_IDS)
    frame = numpy.full((HEIGHT, WIDTH, 3), 255, dtype=numpy.uint8)
    for marker_id in marker_ids:
        _draw_marker(frame, marker_id, TOP_LEFTS[marker_id])
    for marker_id, top_left in extra_markers:
        _draw_marker(frame, marker_id, top_left)
    return frame


def _frame_with_unknown() -> numpy.ndarray:
    """Build the standard frame plus one marker outside TABLE_MARKER_IDS."""
    return _frame(extra_markers=[(UNKNOWN_ID, UNKNOWN_TOP_LEFT)])


def test_all_four_found_with_accuracy():
    markers = detect_table_markers(_frame_with_unknown())
    assert sorted(
        key for key in markers if isinstance(key, int)
    ) == [0, 1, 2, 3]
    for marker_id in TABLE_MARKER_IDS:
        x0, y0 = TOP_LEFTS[marker_id]
        expected_center = (x0 + SIDE_PX / 2.0, y0 + SIDE_PX / 2.0)
        center = markers[marker_id]["center"]
        assert center == pytest.approx(expected_center, abs=2.0)
        assert markers[marker_id]["side_px"] == pytest.approx(
            SIDE_PX, abs=2.0
        )
        assert len(markers[marker_id]["corners"]) == 4
    assert markers["ignored_ids"] == [UNKNOWN_ID]


def test_corner_names_correct():
    markers = detect_table_markers(_frame_with_unknown())
    assert image_corner_names(markers, WIDTH, HEIGHT) == EXPECTED_CORNERS


def test_scale_correct():
    markers = detect_table_markers(_frame_with_unknown())
    assert scale_px_per_mm(markers) == pytest.approx(
        SIDE_PX / TABLE_MARKER_SIZE_MM, abs=0.05
    )


def test_missing_marker_raises_naming_id():
    frame = _frame(marker_ids=[0, 1, 2])
    with pytest.raises(RuntimeError, match="missing"):
        detect_table_markers(frame)
    with pytest.raises(RuntimeError, match="3"):
        detect_table_markers(frame)


def test_duplicated_marker_raises():
    frame = _frame(extra_markers=[(0, UNKNOWN_TOP_LEFT)])
    with pytest.raises(RuntimeError, match="duplicat"):
        detect_table_markers(frame)
    with pytest.raises(RuntimeError, match="0"):
        detect_table_markers(frame)


def test_markers_in_same_image_corner_raise():
    markers = detect_table_markers(_frame_with_unknown())
    with pytest.raises(RuntimeError, match="same image corner"):
        image_corner_names(markers, WIDTH * 4, HEIGHT * 4)


def test_main_writes_annotated_png(tmp_path, monkeypatch, capsys):
    frame = _frame_with_unknown()
    monkeypatch.setattr(
        detect_markers, "open_table_camera", lambda: (frame, {})
    )
    monkeypatch.chdir(tmp_path)
    detect_markers.main()
    out = capsys.readouterr().out
    files = sorted((tmp_path / "recordings").glob("table_markers_*.png"))
    assert len(files) == 1
    assert files[0].name in out
    for marker_id in TABLE_MARKER_IDS:
        assert f"ID {marker_id}" in out
        assert EXPECTED_CORNERS[marker_id] in out
    assert "px/mm" in out
    saved = cv2.imread(str(files[0]))
    assert saved is not None
    assert (saved.shape[1], saved.shape[0]) == (WIDTH, HEIGHT)


def test_never_imports_arm_safety_or_sdk_code():
    path = os.path.join(
        os.path.dirname(__file__), "..", "muse_qc", "detect_markers.py"
    )
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "from muse_qc.arm" not in source
    assert "from muse_qc import arm" not in source
    assert "import arm" not in source
    assert "safety" not in source
    assert "reBotArm" not in source
    assert "motorbridge" not in source
