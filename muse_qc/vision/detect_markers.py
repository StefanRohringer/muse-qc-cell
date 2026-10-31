"""Detect the four printed table markers in a table camera frame.

Each marker ID from muse_qc.vision.table_markers is located with an OpenCV
ArUco detector. The result maps every marker ID to its pixel corners,
its center and its mean edge length, plus the ignored non-table IDs
under the key "ignored_ids".

Run by hand only (it opens the real table camera):

    uv run python -m muse_qc.vision.detect_markers
"""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.vision.camera_top import open_table_camera
from muse_qc.vision.table_markers import (
    TABLE_MARKER_IDS,
    TABLE_MARKER_SIZE_MM,
    get_dictionary,
)

IGNORED_IDS_KEY = "ignored_ids"


def _detector() -> cv2.aruco.ArucoDetector:
    """Return an ArUco detector for the shared table marker dictionary."""
    return cv2.aruco.ArucoDetector(get_dictionary())


def detect_table_markers(frame: numpy.ndarray) -> dict:
    """Find the four table markers in frame.

    Returns a dict mapping each ID in TABLE_MARKER_IDS to
    {"corners": four (x, y) pixel points, "center": (x, y),
    "side_px": mean edge length in pixels}, plus a sorted list of
    detected IDs outside TABLE_MARKER_IDS under "ignored_ids".

    Raises RuntimeError naming the duplicated IDs when a table marker
    is found more than once, or naming the missing IDs when a table
    marker is not found exactly once.
    """
    corners, ids, _ = _detector().detectMarkers(frame)
    found: dict[int, list[numpy.ndarray]] = {}
    ignored: set[int] = set()
    if ids is not None:
        for index, raw_id in enumerate(ids.flatten().tolist()):
            marker_id = int(raw_id)
            if marker_id in TABLE_MARKER_IDS:
                found.setdefault(marker_id, []).append(corners[index][0])
            else:
                ignored.add(marker_id)
    duplicated = sorted(
        marker_id for marker_id, hits in found.items() if len(hits) > 1
    )
    if duplicated:
        raise RuntimeError(
            f"duplicated table markers detected more than once: {duplicated}"
        )
    missing = sorted(set(TABLE_MARKER_IDS) - set(found))
    if missing:
        raise RuntimeError(f"missing table markers: {missing}")
    markers: dict = {}
    for marker_id in TABLE_MARKER_IDS:
        points = [(float(x), float(y)) for x, y in found[marker_id][0]]
        center = (
            sum(x for x, _ in points) / 4.0,
            sum(y for _, y in points) / 4.0,
        )
        edges = [
            math.dist(points[i], points[(i + 1) % 4]) for i in range(4)
        ]
        markers[marker_id] = {
            "corners": points,
            "center": center,
            "side_px": sum(edges) / 4.0,
        }
    markers[IGNORED_IDS_KEY] = sorted(ignored)
    return markers


def image_corner_names(markers: dict, width: int, height: int) -> dict:
    """Name the image corner of each marker from its center.

    Returns a dict mapping each marker ID to "top-left", "top-right",
    "bottom-left" or "bottom-right", decided by which half of the
    image the center falls into. Raises RuntimeError when two markers
    fall into the same image corner.
    """
    names: dict = {}
    used: dict[str, int] = {}
    for marker_id, info in markers.items():
        if marker_id == IGNORED_IDS_KEY:
            continue
        center_x, center_y = info["center"]
        vertical = "top" if center_y < height / 2.0 else "bottom"
        horizontal = "left" if center_x < width / 2.0 else "right"
        name = f"{vertical}-{horizontal}"
        if name in used:
            raise RuntimeError(
                "two table markers fall into the same image corner"
                f" {name}: IDs {used[name]} and {marker_id}"
            )
        used[name] = marker_id
        names[marker_id] = name
    return names


def scale_px_per_mm(markers: dict) -> float:
    """Return the mean marker side length in pixels per millimetre."""
    sides = [
        info["side_px"]
        for marker_id, info in markers.items()
        if marker_id != IGNORED_IDS_KEY
    ]
    return (sum(sides) / len(sides)) / TABLE_MARKER_SIZE_MM


def annotate(
    frame: numpy.ndarray, markers: dict, names: dict
) -> numpy.ndarray:
    """Return a copy of frame with marker outlines and IDs drawn."""
    annotated = frame.copy()
    ids = [marker_id for marker_id in markers if marker_id != IGNORED_IDS_KEY]
    corners = [
        numpy.array(markers[marker_id]["corners"], dtype=numpy.float32).reshape(
            1, 4, 2
        )
        for marker_id in ids
    ]
    cv2.aruco.drawDetectedMarkers(annotated, corners, numpy.array(ids))
    for marker_id in ids:
        center_x, center_y = markers[marker_id]["center"]
        cv2.putText(
            annotated,
            f"{marker_id} {names[marker_id]}",
            (int(center_x), int(center_y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            2,
        )
    return annotated


def save_annotated(
    frame: numpy.ndarray, recordings_dir: str | Path = "recordings"
) -> Path:
    """Save an annotated frame as PNG, return the file path."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target_dir / f"table_markers_{stamp}.png"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"failed to write image to {path}")
    return path


def main() -> None:
    """Take one table camera frame, print and save the marker detection."""
    frame, _ = open_table_camera()
    height, width = frame.shape[0], frame.shape[1]
    markers = detect_table_markers(frame)
    names = image_corner_names(markers, width, height)
    for marker_id in TABLE_MARKER_IDS:
        center_x, center_y = markers[marker_id]["center"]
        side_px = markers[marker_id]["side_px"]
        print(
            f"ID {marker_id} {names[marker_id]}"
            f" center=({center_x:.1f}, {center_y:.1f})"
            f" side={side_px:.1f} px"
        )
    print(f"scale: {scale_px_per_mm(markers):.3f} px/mm")
    path = save_annotated(annotate(frame, markers, names))
    print(path)


if __name__ == "__main__":
    main()
