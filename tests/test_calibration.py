"""Tests for capture_calibration.py and calibrate_camera.py.

Offline only: synthetic boards are drawn with OpenCV or projected
with cv2.projectPoints from a known camera. No camera is opened and
no arm, safety or SDK code is imported.
"""

from __future__ import annotations

import ast
from pathlib import Path

import cv2
import numpy
import pytest

import muse_qc.calibrate_camera as calibrate_camera
import muse_qc.capture_calibration as capture_calibration
from muse_qc.calibrate_camera import (
    board_object_points,
    calibrate_from_files,
    calibrate_from_points,
    load_calibration,
    save_calibration,
    undistort,
)
from muse_qc.capture_calibration import capture_session, find_board
from muse_qc.checkerboard import CHECKER_SQUARES

TRUE_FX = 800.0
TRUE_FY = 800.0
TRUE_CX = 320.0
TRUE_CY = 240.0
TRUE_DIST = numpy.array([[-0.2, 0.05, 0.0, 0.0, 0.0]])
TRUE_SIZE = (640, 640 - 160)


def true_camera_matrix() -> numpy.ndarray:
    """Return the known synthetic camera matrix."""
    return numpy.array(
        [
            [TRUE_FX, 0.0, TRUE_CX],
            [0.0, TRUE_FY, TRUE_CY],
            [0.0, 0.0, 1.0],
        ]
    )


def synthetic_views(count: int) -> tuple[list, list]:
    """Project the board with the known camera at varied poses.

    Returns (object_points, image_points) rendered with
    cv2.projectPoints, one pair per view.
    """
    rng = numpy.random.default_rng(7)
    camera_matrix = true_camera_matrix()
    template = board_object_points()
    object_points: list = []
    image_points: list = []
    for _ in range(count):
        rotation = numpy.array(
            [
                numpy.deg2rad(rng.uniform(-25.0, 25.0)),
                numpy.deg2rad(rng.uniform(-25.0, 25.0)),
                numpy.deg2rad(rng.uniform(-10.0, 10.0)),
            ]
        )
        shift = numpy.array(
            [
                rng.uniform(-100.0, 100.0),
                rng.uniform(-60.0, 60.0),
                rng.uniform(450.0, 650.0),
            ]
        )
        projected, _ = cv2.projectPoints(
            template, rotation, shift, camera_matrix, TRUE_DIST
        )
        object_points.append(template)
        image_points.append(projected.astype(numpy.float32))
    return object_points, image_points


def draw_board(width: int, height: int, margin: int = 40) -> numpy.ndarray:
    """Draw a centred synthetic checkerboard as a BGR image."""
    columns, rows = CHECKER_SQUARES
    square = min((width - 2 * margin) // columns, (height - 2 * margin) // rows)
    board_width = columns * square
    board_height = rows * square
    left = (width - board_width) // 2
    top = (height - board_height) // 2
    image = numpy.full((height, width, 3), 255, dtype=numpy.uint8)
    for row in range(rows):
        for column in range(columns):
            if (row + column) % 2 == 0:
                image[
                    top + row * square : top + (row + 1) * square,
                    left + column * square : left + (column + 1) * square,
                ] = 0
    return image


def write_boards(target_dir: Path, count: int, width: int = 640, height: int = 480) -> None:
    """Write count detectable board PNGs into target_dir."""
    for index in range(count):
        path = target_dir / f"calib_test_{index:02d}.png"
        assert cv2.imwrite(str(path), draw_board(width, height))


def test_recover_known_camera() -> None:
    """Calibrate recovers focal length within 2 percent and k1 sign."""
    object_points, image_points = synthetic_views(12)
    rms, camera_matrix, dist = calibrate_from_points(object_points, image_points, TRUE_SIZE)
    assert abs(camera_matrix[0, 0] - TRUE_FX) / TRUE_FX < 0.02
    assert abs(camera_matrix[1, 1] - TRUE_FY) / TRUE_FY < 0.02
    assert float(dist[0, 0]) < 0.0
    assert rms < 0.1


def test_fewer_than_10_boards_refused(tmp_path: Path) -> None:
    """Nine found boards are refused with a clear error."""
    write_boards(tmp_path, 9)
    with pytest.raises(RuntimeError, match="fewer than 10"):
        calibrate_from_files(sorted(tmp_path.glob("*.png")), target_dir=tmp_path)


def test_mixed_image_sizes_refused(tmp_path: Path) -> None:
    """Images with different sizes are refused with a clear error."""
    assert cv2.imwrite(str(tmp_path / "a.png"), draw_board(640, 480))
    assert cv2.imwrite(str(tmp_path / "b.png"), draw_board(800, 600))
    with pytest.raises(RuntimeError, match="differ"):
        calibrate_from_files(sorted(tmp_path.glob("*.png")), target_dir=tmp_path)


def test_json_round_trip_and_undistort(tmp_path: Path) -> None:
    """Save/load keeps matrices and undistort keeps the frame shape."""
    object_points, image_points = synthetic_views(12)
    rms, camera_matrix, dist = calibrate_from_points(object_points, image_points, TRUE_SIZE)
    record = {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist.tolist(),
        "image_size": [TRUE_SIZE[0], TRUE_SIZE[1]],
        "rms_px": rms,
        "images_used": ["calib_test_00.png"],
        "created": "2026-10-28",
    }
    path = tmp_path / "table_camera.json"
    assert save_calibration(record, path) == path
    loaded = load_calibration(path)
    numpy.testing.assert_allclose(loaded["camera_matrix"], camera_matrix)
    numpy.testing.assert_allclose(loaded["dist_coeffs"], dist)
    assert loaded["image_size"] == (TRUE_SIZE[0], TRUE_SIZE[1])
    frame = draw_board(*TRUE_SIZE)
    assert undistort(frame, loaded).shape == frame.shape
    assert undistort(frame, record).shape == frame.shape


def test_modules_import_no_hardware_code() -> None:
    """Neither module imports arm, safety or SDK code."""
    forbidden = ("arm", "safety", "sdk", "rebotarm", "motorbridge")
    for module_path in (
        Path(capture_calibration.__file__),
        Path(calibrate_camera.__file__),
    ):
        tree = ast.parse(module_path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            for name in names:
                assert all(word not in name.lower() for word in forbidden), name


def test_capture_loop_saves_found_and_skips_missing(
    tmp_path: Path, capsys: pytest.CaptureFixture
) -> None:
    """Fake camera: a found board is saved, a blank frame is skipped."""
    board = draw_board(640, 480)
    blank = numpy.full((480, 640, 3), 255, dtype=numpy.uint8)
    frames = [board, blank]
    answers = iter(["", "", "q"])
    saved = capture_session(
        open_fn=lambda: frames.pop(0),
        read_input=lambda _prompt: next(answers),
        target_dir=tmp_path,
    )
    assert saved == 1
    assert len(list(tmp_path.glob("calib_*.png"))) == 1
    output = capsys.readouterr().out
    assert "board found" in output
    assert "board NOT found" in output
    assert "(count: 1)" in output


def test_capture_loop_quit_saves_nothing(tmp_path: Path) -> None:
    """Answering q immediately saves no photo."""
    calls: list = []

    def _open() -> numpy.ndarray:
        calls.append(1)
        return draw_board(640, 480)

    saved = capture_session(
        open_fn=_open,
        read_input=lambda _prompt: "q",
        target_dir=tmp_path,
    )
    assert saved == 0
    assert calls == []
    assert list(tmp_path.glob("*.png")) == []


def test_find_board_rejects_blank() -> None:
    """A blank frame reports no board."""
    blank = numpy.full((480, 640, 3), 255, dtype=numpy.uint8)
    found, corners = find_board(blank)
    assert found is False
    assert corners is None
