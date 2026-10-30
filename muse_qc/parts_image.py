"""Annotated part images for find_parts (rectangles, ids, table mm)."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2


def annotate(frame, results):
    """Return a copy of frame with blob rectangles, ids and table mm.

    Skipped blobs are grey with the skip reason below the box.
    """
    out = frame.copy()
    for index, item in enumerate(results):
        x, y, width, height = item["bbox"]
        if item.get("skipped") is not None:
            color = (128, 128, 128)
        elif item["confirmed"]:
            color = (0, 255, 0)
        else:
            color = (0, 255, 255)
        cv2.rectangle(out, (x, y), (x + width, y + height), color, 2)
        table_x, table_y = item["table_mm"]
        cv2.putText(
            out, f"{index} ({table_x:.0f}, {table_y:.0f})",
            (x, max(0, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
        )
        if item.get("skipped") is not None:
            cv2.putText(
                out, str(item["skipped"]),
                (x, y + height + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                color, 2,
            )
    return out


def save_annotated(frame, recordings_dir="recordings"):
    """Save the annotated frame as PNG, return the file path."""
    target = Path(recordings_dir)
    target.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target / f"parts_{stamp}.png"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"failed to write image to {path}")
    return path
