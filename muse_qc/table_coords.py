"""Convert table camera pixels into table millimetres.

The four printed table markers (see muse_qc.table_markers) span a known
rectangle on the table. Their detected centres give a homography from
pixel coordinates to the table frame defined by TABLE_MARKER_LAYOUT_MM
(origin at the centre of marker 0, +x towards marker 1, +y towards
marker 2 away from the arm, units mm).

Run by hand only (it opens the real table camera):

    uv run python -m muse_qc.table_coords
"""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.camera_top import open_table_camera
from muse_qc.detect_markers import detect_table_markers
from muse_qc.table_markers import (
    TABLE_MARKER_IDS,
    TABLE_MARKER_LAYOUT_MM,
    TABLE_MARKER_SIZE_MM,
)

# Order around the rectangle, so source and destination points match.
RECTANGLE_ORDER = (0, 1, 3, 2)


def table_homography(markers: dict) -> numpy.ndarray:
    """Return the 3x3 homography from pixels to table millimetres.

    The source points are the detected marker centres (output of
    detect_table_markers), the destination points are the matching
    entries of TABLE_MARKER_LAYOUT_MM. Raises RuntimeError naming the
    missing IDs when a table marker has no entry.
    """
    missing = [marker_id for marker_id in TABLE_MARKER_IDS if marker_id not in markers]
    if missing:
        raise RuntimeError(f"missing table markers: {missing}")
    source = numpy.array(
        [markers[marker_id]["center"] for marker_id in RECTANGLE_ORDER],
        dtype=numpy.float32,
    )
    target = numpy.array(
        [TABLE_MARKER_LAYOUT_MM[marker_id] for marker_id in RECTANGLE_ORDER],
        dtype=numpy.float32,
    )
    return cv2.getPerspectiveTransform(source, target)


def _apply(homography: numpy.ndarray, point: tuple[float, float]) -> tuple[float, float]:
    """Map one 2D point through a 3x3 homography."""
    mapped = homography @ numpy.array([point[0], point[1], 1.0], dtype=float)
    return (float(mapped[0] / mapped[2]), float(mapped[1] / mapped[2]))


def pixel_to_table_mm(
    homography: numpy.ndarray, pixel: tuple[float, float]
) -> tuple[float, float]:
    """Map one pixel (u, v) to table coordinates (x_mm, y_mm)."""
    return _apply(homography, pixel)


def table_mm_to_pixel(
    homography: numpy.ndarray, point_mm: tuple[float, float]
) -> tuple[float, float]:
    """Map one table point (x_mm, y_mm) back to pixels (u, v)."""
    return _apply(numpy.linalg.inv(homography), point_mm)


def marker_sides_mm(homography: numpy.ndarray, markers: dict) -> dict:
    """Return the mean side length in mm of each marker.

    Every marker corner is mapped into the table frame and the four
    edge lengths are averaged. Each value is expected to be about
    TABLE_MARKER_SIZE_MM; a clearly different value means the
    homography (or the marker placement) is wrong.
    """
    sides: dict = {}
    for marker_id in TABLE_MARKER_IDS:
        corners = [
            pixel_to_table_mm(homography, corner)
            for corner in markers[marker_id]["corners"]
        ]
        edges = [math.dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
        sides[marker_id] = sum(edges) / 4.0
    return sides


def draw_grid(
    frame: numpy.ndarray, homography: numpy.ndarray, step_mm: int = 100
) -> numpy.ndarray:
    """Return a copy of frame with a 100 mm table grid drawn over it.

    Grid lines run every step_mm over the marker rectangle, the
    rectangle border is drawn, and the origin plus the +x and +y axes
    are labelled.
    """
    grid = frame.copy()
    max_x = max(x for x, _ in TABLE_MARKER_LAYOUT_MM.values())
    max_y = max(y for _, y in TABLE_MARKER_LAYOUT_MM.values())
    green = (0, 255, 0)
    red = (0, 0, 255)

    def to_px(point_mm: tuple[float, float]) -> tuple[int, int]:
        u, v = table_mm_to_pixel(homography, point_mm)
        return (int(round(u)), int(round(v)))

    x = 0.0
    while x <= max_x:
        cv2.line(grid, to_px((x, 0.0)), to_px((x, max_y)), green, 1)
        x += step_mm
    y = 0.0
    while y <= max_y:
        cv2.line(grid, to_px((0.0, y)), to_px((max_x, y)), green, 1)
        y += step_mm
    border = [to_px(TABLE_MARKER_LAYOUT_MM[marker_id]) for marker_id in RECTANGLE_ORDER]
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


def main() -> None:
    """Take one frame, print table coordinates, save the grid image."""
    frame, _ = open_table_camera()
    height, width = frame.shape[0], frame.shape[1]
    markers = detect_table_markers(frame)
    homography = table_homography(markers)
    sides = marker_sides_mm(homography, markers)
    for marker_id in TABLE_MARKER_IDS:
        center_x, center_y = markers[marker_id]["center"]
        x_mm, y_mm = pixel_to_table_mm(homography, (center_x, center_y))
        print(
            f"ID {marker_id} center_px=({center_x:.1f}, {center_y:.1f})"
            f" center_mm=({x_mm:.1f}, {y_mm:.1f})"
            f" side={sides[marker_id]:.1f} mm"
            f" (expected {TABLE_MARKER_SIZE_MM:.0f} mm)"
        )
    image_u, image_v = width / 2.0, height / 2.0
    image_x, image_y = pixel_to_table_mm(homography, (image_u, image_v))
    print(
        f"image center px=({image_u:.1f}, {image_v:.1f})"
        f" mm=({image_x:.1f}, {image_y:.1f})"
    )
    path = save_grid(draw_grid(frame, homography))
    print(path)


if __name__ == "__main__":
    main()
