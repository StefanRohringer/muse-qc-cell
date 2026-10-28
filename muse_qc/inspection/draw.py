"""Draw and save annotated lever-count images (used by count_clips)."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.vision_sam import draw_boxes


def draw_levers(frame: numpy.ndarray, boxes: list[dict], count: int) -> numpy.ndarray:
    """Return a copy of frame with boxes and the lever count drawn."""
    annotated = draw_boxes(frame, boxes)
    cv2.putText(
        annotated,
        f"levers: {count}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
    )
    return annotated


def draw_part(frame: numpy.ndarray, box: dict, count: int) -> numpy.ndarray:
    """Return a copy of frame with the part box and count drawn."""
    return draw_levers(frame, [box], count)


def save_pair(
    full: numpy.ndarray,
    crop: numpy.ndarray,
    recordings_dir: str | Path = "recordings",
) -> tuple[Path, Path]:
    """Save full and crop under one timestamp, return both paths."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    full_path = target_dir / f"count_{stamp}.png"
    crop_path = target_dir / f"count_{stamp}_crop.png"
    ok_full = cv2.imwrite(str(full_path), full)
    ok_crop = cv2.imwrite(str(crop_path), crop)
    if not ok_full or not ok_crop:
        raise RuntimeError(f"failed to write images to {target_dir}")
    return full_path, crop_path
