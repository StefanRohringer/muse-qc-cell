"""Zoomed part search window for the arm camera image. Pure, sends nothing.

The part held between the fingertips is always at the same image
position (EXPECTED_PART_PX in muse_qc.count_clips) but only about
40 x 55 px in the 640 x 480 frame, so the full-frame part request
misses it. Cutting a 160 x 160 px window around the expected
position and enlarging it 4 times gives SAM a 640 x 640 px image
to find the part box on; the box is then mapped back into full
frame pixels for the same expected_px check as today.
"""

from __future__ import annotations

import cv2
import numpy

PART_ZOOM_WINDOW_PX = 160
PART_ZOOM_SCALE = 4


def part_zoom_window(frame: numpy.ndarray, expected_px,
                     size: int = PART_ZOOM_WINDOW_PX) -> tuple[int, int, int]:
    """Return the square search window (x0, y0, side) around expected_px.

    The window is centred on expected_px and shifted to stay
    inside the frame; the side is clamped to the frame size.
    """
    height, width = frame.shape[:2]
    side = min(int(size), width, height)
    center_x, center_y = float(expected_px[0]), float(expected_px[1])
    x0 = min(max(int(round(center_x - side / 2.0)), 0), width - side)
    y0 = min(max(int(round(center_y - side / 2.0)), 0), height - side)
    return (x0, y0, side)


def zoom_part_window(frame: numpy.ndarray, expected_px,
                     scale: int = PART_ZOOM_SCALE) -> tuple:
    """Cut the search window and enlarge it; return (zoomed, window, scale).

    The resize uses cv2.INTER_CUBIC, the same as crop_and_enlarge
    in muse_qc.count_clips. Window is (x0, y0, side) in full frame
    pixels: (x0, y0) is the corner for the back mapping, side the
    drawn square on the annotated full image.
    """
    x0, y0, side = part_zoom_window(frame, expected_px)
    window = frame[y0:y0 + side, x0:x0 + side]
    zoomed = cv2.resize(
        window,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_CUBIC,
    )
    return zoomed, (x0, y0, side), scale


def map_zoom_box_to_frame(box: dict, origin, scale: int = PART_ZOOM_SCALE) -> dict:
    """Map one SAM box from zoomed window pixels to full frame pixels."""
    x0, y0 = float(origin[0]), float(origin[1])
    scale = float(scale)
    x1 = x0 + float(box["x1"]) / scale
    y1 = y0 + float(box["y1"]) / scale
    x2 = x0 + float(box["x2"]) / scale
    y2 = y0 + float(box["y2"]) / scale
    return {
        "id": box["id"],
        "x1": x1,
        "y1": y1,
        "x2": x2,
        "y2": y2,
        "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
        "confidence": box["confidence"],
    }
