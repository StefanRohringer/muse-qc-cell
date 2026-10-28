"""Convert table camera pixels into table millimetres.

The four printed table markers (see muse_qc.table_markers) span a known
quadrilateral on the table. Their detected centres give a homography from
pixel coordinates to the table frame defined by TABLE_MARKER_LAYOUT_MM
(origin at the centre of marker 0, +x towards marker 1, +y towards
marker 2 away from the arm, units mm). When a calibration record is
passed, the raw marker points are undistorted first and every mapping
converts between raw pixels and undistorted pixels point by point;
the frame itself is never undistorted as a whole.

Run by hand only (it opens the real table camera):

    uv run python -m muse_qc.table_coords
"""

from __future__ import annotations

import math
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.calibrate_camera import (
    OUTPUT_PATH as CALIBRATION_PATH,
)
from muse_qc.calibrate_camera import (
    distort_points,
    load_calibration,
    undistort_points,
)
from muse_qc.camera_top import open_table_camera
from muse_qc.detect_markers import detect_table_markers
from muse_qc.table_markers import (
    TABLE_MARKER_IDS,
    TABLE_MARKER_LAYOUT_MM,
    TABLE_MARKER_SIZE_MM,
)

# Order around the quadrilateral, so source and destination points match.
CORNER_ORDER = (0, 1, 3, 2)


def load_table_calibration(path: str | Path = CALIBRATION_PATH) -> dict | None:
    """Load the table calibration, or return None when the file is missing.

    A corrupt file still raises; only a missing file falls back to raw
    pixels.
    """
    try:
        return load_calibration(path)
    except OSError:
        return None


def table_homography(markers: dict, calibration: dict | None = None) -> numpy.ndarray:
    """Return the 3x3 homography from pixels to table millimetres.

    The source points are the detected marker centres (output of
    detect_table_markers), undistorted first when a calibration is
    passed; the destination points are the matching entries of
    TABLE_MARKER_LAYOUT_MM. Raises RuntimeError naming the missing
    IDs when a table marker has no entry.
    """
    missing = [marker_id for marker_id in TABLE_MARKER_IDS if marker_id not in markers]
    if missing:
        raise RuntimeError(f"missing table markers: {missing}")
    centers = [markers[marker_id]["center"] for marker_id in CORNER_ORDER]
    if calibration is not None:
        centers = undistort_points(centers, calibration)
    source = numpy.array(centers, dtype=numpy.float32)
    target = numpy.array(
        [TABLE_MARKER_LAYOUT_MM[marker_id] for marker_id in CORNER_ORDER],
        dtype=numpy.float32,
    )
    return cv2.getPerspectiveTransform(source, target)


def _apply(homography: numpy.ndarray, point: tuple[float, float]) -> tuple[float, float]:
    """Map one 2D point through a 3x3 homography."""
    mapped = homography @ numpy.array([point[0], point[1], 1.0], dtype=float)
    return (float(mapped[0] / mapped[2]), float(mapped[1] / mapped[2]))


def pixel_to_table_mm(
    homography: numpy.ndarray,
    pixel: tuple[float, float],
    calibration: dict | None = None,
) -> tuple[float, float]:
    """Map one raw pixel (u, v) to table coordinates (x_mm, y_mm).

    With a calibration the raw pixel is undistorted first, so the
    homography always works on undistorted pixels.
    """
    if calibration is not None:
        (pixel,) = undistort_points([pixel], calibration)
    return _apply(homography, pixel)


def table_mm_to_pixel(
    homography: numpy.ndarray,
    point_mm: tuple[float, float],
    calibration: dict | None = None,
) -> tuple[float, float]:
    """Map one table point (x_mm, y_mm) back to raw pixels (u, v).

    With a calibration the undistorted pixel is distorted back, so
    the result lands on the raw frame.
    """
    undistorted = _apply(numpy.linalg.inv(homography), point_mm)
    if calibration is not None:
        (undistorted,) = distort_points([undistorted], calibration)
    return undistorted


def boxes_to_table_mm(
    boxes: list[dict],
    frame: numpy.ndarray,
    load_calibration: Callable[[], dict | None] = load_table_calibration,
    detect_markers: Callable[[numpy.ndarray], dict] = detect_table_markers,
) -> tuple[str, list[tuple[float, float] | None]]:
    """Map each box centre to table millimetres on the same frame.

    Loads the calibration (None means raw pixels), detects the table
    markers on frame and maps every box["center"] with pixel_to_table_mm.
    Returns the status word for the "table mapping:" line ("calibrated",
    "raw pixels", or "unavailable (<reason>)") plus one (x_mm, y_mm)
    point per box (None when the mapping is unavailable).
    """
    try:
        calibration = load_calibration()
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        return f"unavailable ({exc})", [None] * len(boxes)
    try:
        markers = detect_markers(frame)
        homography = table_homography(markers, calibration)
        points = [
            pixel_to_table_mm(homography, box["center"], calibration)
            for box in boxes
        ]
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        return f"unavailable ({exc})", [None] * len(boxes)
    if calibration is None:
        return "raw pixels", points
    return "calibrated", points


def marker_sides_mm(
    homography: numpy.ndarray, markers: dict, calibration: dict | None = None
) -> dict:
    """Return the mean side length in mm of each marker.

    Every marker corner is mapped into the table frame and the four
    edge lengths are averaged. Each value is expected to be about
    TABLE_MARKER_SIZE_MM; a clearly different value means the
    homography (or the marker placement) is wrong.
    """
    sides: dict = {}
    for marker_id in TABLE_MARKER_IDS:
        corners = list(markers[marker_id]["corners"])
        if calibration is not None:
            corners = undistort_points(corners, calibration)
        mapped = [pixel_to_table_mm(homography, corner) for corner in corners]
        edges = [math.dist(mapped[i], mapped[(i + 1) % 4]) for i in range(4)]
        sides[marker_id] = sum(edges) / 4.0
    return sides


def draw_grid(
    frame: numpy.ndarray,
    homography: numpy.ndarray,
    step_mm: int = 100,
    calibration: dict | None = None,
) -> numpy.ndarray:
    """Return a copy of frame with a 100 mm table grid drawn over it.

    Grid lines run every step_mm over the marker quadrilateral, the
    quadrilateral border is drawn, and the origin plus the +x and +y
    axes are labelled. With a calibration the grid is distorted back onto
    the raw frame; the frame itself is never undistorted.
    """
    grid = frame.copy()
    max_x = max(x for x, _ in TABLE_MARKER_LAYOUT_MM.values())
    max_y = max(y for _, y in TABLE_MARKER_LAYOUT_MM.values())
    green = (0, 255, 0)
    red = (0, 0, 255)

    def to_px(point_mm: tuple[float, float]) -> tuple[int, int]:
        u, v = table_mm_to_pixel(homography, point_mm, calibration)
        return (int(round(u)), int(round(v)))

    x = 0.0
    while x <= max_x:
        cv2.line(grid, to_px((x, 0.0)), to_px((x, max_y)), green, 1)
        x += step_mm
    y = 0.0
    while y <= max_y:
        cv2.line(grid, to_px((0.0, y)), to_px((max_x, y)), green, 1)
        y += step_mm
    border = [to_px(TABLE_MARKER_LAYOUT_MM[marker_id]) for marker_id in CORNER_ORDER]
    for i in range(4):
        cv2.line(grid, border[i], border[(i + 1) % 4], red, 2)
    origin = to_px((0.0, 0.0))
    cv2.circle(grid, origin, 8, red, 2)
    cv2.putText(grid, "origin (0, 0)", (origin[0] + 12, origin[1] + 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, red, 2)
    tip_x = to_px((min(150.0, max_x), 0.0))
    cv2.putText(grid, "+x", (tip_x[0] + 8, tip_x[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, red, 2)
    tip_y = to_px((0.0, min(150.0, max_y)))
    cv2.putText(grid, "+y", (tip_y[0] + 8, tip_y[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, red, 2)
    return grid


def save_grid(
    frame: numpy.ndarray, recordings_dir: str | Path = "recordings"
) -> Path:
    """Save a grid frame as PNG, return the file path."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target_dir / f"table_grid_{stamp}.png"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"failed to write image to {path}")
    return path


def main(calibration_path: str | Path = CALIBRATION_PATH) -> None:
    """Take one frame, print table coordinates, save the grid image.

    Loads the calibration from calibration_path when the file exists
    (undistorted mapping) and falls back to raw pixels otherwise.
    """
    frame, _ = open_table_camera()
    height, width = frame.shape[0], frame.shape[1]
    calibration = load_table_calibration(calibration_path)
    if calibration is None:
        print("calibration: none (raw pixels)")
    else:
        print(
            f"calibration: {calibration_path}"
            f" (rms {calibration['rms_px']:.3f} px)"
        )
    markers = detect_table_markers(frame)
    homography = table_homography(markers, calibration)
    sides = marker_sides_mm(homography, markers, calibration)
    for marker_id in TABLE_MARKER_IDS:
        center_x, center_y = markers[marker_id]["center"]
        x_mm, y_mm = pixel_to_table_mm(
            homography, (center_x, center_y), calibration
        )
        print(
            f"ID {marker_id} center_px=({center_x:.1f}, {center_y:.1f})"
            f" center_mm=({x_mm:.1f}, {y_mm:.1f})"
            f" side={sides[marker_id]:.1f} mm"
            f" (expected {TABLE_MARKER_SIZE_MM:.0f} mm)"
        )
    image_u, image_v = width / 2.0, height / 2.0
    image_x, image_y = pixel_to_table_mm(
        homography, (image_u, image_v), calibration
    )
    print(
        f"image center px=({image_u:.1f}, {image_v:.1f})"
        f" mm=({image_x:.1f}, {image_y:.1f})"
    )
    path = save_grid(draw_grid(frame, homography, calibration=calibration))
    print(path)


if __name__ == "__main__":
    main()
