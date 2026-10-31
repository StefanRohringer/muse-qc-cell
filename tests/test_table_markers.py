"""Tests for table_markers.py. Offline only, no hardware, no SDK import."""

from __future__ import annotations

import os

import cv2
import numpy
import pytest

from muse_qc.vision.table_markers import (
    TABLE_MARKER_DICT,
    TABLE_MARKER_IDS,
    TABLE_MARKER_SIZE_MM,
    get_dictionary,
    marker_cells,
)


def test_constants():
    assert TABLE_MARKER_DICT == cv2.aruco.DICT_4X4_50
    assert tuple(TABLE_MARKER_IDS) == (0, 1, 2, 3)
    assert TABLE_MARKER_SIZE_MM == 40.0


def test_cells_are_6x6_binary():
    for marker_id in TABLE_MARKER_IDS:
        cells = marker_cells(marker_id)
        assert cells.shape == (6, 6)
        assert set(numpy.unique(cells).tolist()) <= {0, 1}


def test_outer_ring_is_black():
    for marker_id in TABLE_MARKER_IDS:
        cells = marker_cells(marker_id)
        assert cells[0, :].tolist() == [1] * 6
        assert cells[5, :].tolist() == [1] * 6
        assert cells[:, 0].tolist() == [1] * 6
        assert cells[:, 5].tolist() == [1] * 6


def test_ids_have_different_patterns():
    patterns = [marker_cells(marker_id).tobytes() for marker_id in TABLE_MARKER_IDS]
    assert len(set(patterns)) == 4


def test_unknown_id_raises():
    with pytest.raises(ValueError, match="unknown table marker ID"):
        marker_cells(4)


def _render(cells, cell_px=20, quiet_px=20):
    """Render cells into a white grayscale image with a quiet zone."""
    rows, columns = cells.shape
    image = numpy.full(
        (rows * cell_px + 2 * quiet_px, columns * cell_px + 2 * quiet_px),
        255,
        dtype=numpy.uint8,
    )
    for row in range(rows):
        for column in range(columns):
            if cells[row, column] == 1:
                image[
                    quiet_px + row * cell_px : quiet_px + (row + 1) * cell_px,
                    quiet_px + column * cell_px : quiet_px
                    + (column + 1) * cell_px,
                ] = 0
    return image


def test_round_trip_detects_same_id():
    detector = cv2.aruco.ArucoDetector(get_dictionary())
    for marker_id in TABLE_MARKER_IDS:
        image = _render(marker_cells(marker_id))
        corners, ids, _ = detector.detectMarkers(image)
        assert ids is not None
        assert ids.flatten().tolist() == [marker_id]


def test_never_imports_arm_or_sdk_code():
    for name in ("table_markers.py", "make_markers.py"):
        path = os.path.join(os.path.dirname(__file__), "..", "muse_qc", "vision", name)
        with open(path, encoding="utf-8") as handle:
            source = handle.read()
        assert "from muse_qc.arm.arm" not in source
        assert "from muse_qc import arm" not in source
        assert "import arm" not in source
        assert "reBotArm" not in source
        assert "motorbridge" not in source
