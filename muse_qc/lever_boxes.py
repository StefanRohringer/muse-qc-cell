"""Lever box filtering for the SAM lever count. Pure, sends nothing.

Keeps boxes at or above a confidence, merges overlapping
duplicates (IoU above the threshold keeps the higher
confidence) and drops too-short remnants: a bar must reach
both the part-height fraction (of the enlarged part height
without the crop margin) and the tallest-bar fraction.
"""

from __future__ import annotations


def _iou(first: dict, second: dict) -> float:
    """Return the intersection-over-union of two box dicts."""
    x1 = max(first["x1"], second["x1"])
    y1 = max(first["y1"], second["y1"])
    x2 = min(first["x2"], second["x2"])
    y2 = min(first["y2"], second["y2"])
    width = x2 - x1
    height = y2 - y1
    if width <= 0 or height <= 0:
        return 0.0
    inter = width * height
    area_first = (first["x2"] - first["x1"]) * (first["y2"] - first["y1"])
    area_second = (second["x2"] - second["x1"]) * (second["y2"] - second["y1"])
    union = area_first + area_second - inter
    if union <= 0:
        return 0.0
    return inter / union


def count_levers(boxes: list[dict], min_confidence: float,
                 iou_threshold: float) -> list[dict]:
    """Filter and de-duplicate boxes, return the kept lever boxes.

    Drops boxes without confidence or below min_confidence, then
    merges overlapping duplicates: boxes are checked from high to
    low confidence and a box is dropped when its IoU with any kept
    box is above iou_threshold. The kept boxes keep their input
    order by confidence (highest first).
    """
    confident = [
        box
        for box in boxes
        if isinstance(box.get("confidence"), (int, float))
        and box["confidence"] >= min_confidence
    ]
    confident.sort(key=lambda box: box["confidence"], reverse=True)
    kept: list[dict] = []
    for box in confident:
        if all(_iou(box, other) <= iou_threshold for other in kept):
            kept.append(box)
    return kept


def drop_short_bars(
    boxes: list[dict], part_height_px: int, crop_scale: int,
    min_bar_part_frac: float, min_bar_tallest_frac: float,
) -> tuple[list[dict], list[dict]]:
    """Split lever boxes into bars and too-short remnants.

    A bar must satisfy both rules: height at least
    min_bar_part_frac of the enlarged part height
    (part_height_px * crop_scale, without the crop margin) and
    height at least min_bar_tallest_frac of the tallest box in
    the input. A box of exactly either limit is kept. An empty
    input stays empty.
    """
    if not boxes:
        return [], []
    part_limit = min_bar_part_frac * part_height_px * crop_scale
    tallest = max(box["y2"] - box["y1"] for box in boxes)
    tallest_limit = min_bar_tallest_frac * tallest
    bars = []
    short = []
    for box in boxes:
        height = box["y2"] - box["y1"]
        if height >= part_limit and height >= tallest_limit:
            bars.append(box)
        else:
            short.append(box)
    return bars, short
