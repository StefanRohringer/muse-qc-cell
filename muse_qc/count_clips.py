"""Count orange levers on an arm camera image with SAM 3.1 (hand-run).

Two steps: SAM 3.1 first finds the part on the full image with
PART_PROMPT, then SAM 3.1 counts the levers on the cropped and
enlarged part with LEVER_PROMPT. Boxes at or above MIN_CONFIDENCE
are kept, overlapping duplicates are merged (IoU above
IOU_THRESHOLD keeps the higher confidence). Prints the part box,
one line per kept lever box plus "levers: N" and
"verdict: good|bad", and saves two annotated copies:
recordings/count_<timestamp>.png (full image with the part box)
and recordings/count_<timestamp>_crop.png (enlarged crop with the
lever boxes). A good part has exactly EXPECTED_LEVERS levers.

Run by the human only (it contacts the Meta Model API):

    uv run python -m muse_qc.count_clips --image recordings/armcam_test.png
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.model_check import KEY_NAME, read_api_key
from muse_qc.vision_sam import (
    _default_client_factory,
    draw_boxes,
    fetch_output_text,
    image_to_data_url,
    load_image,
    parse_output_text,
)

EXPECTED_LEVERS = 3
MIN_CONFIDENCE = 0.5
IOU_THRESHOLD = 0.5
PART_PROMPT = "small part"
LEVER_PROMPT = "orange bar"
GRIPPER_ZONE_Y = 380
CROP_MARGIN_PX = 15
CROP_SCALE = 8


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


def count_levers(boxes: list[dict]) -> list[dict]:
    """Filter and de-duplicate boxes, return the kept lever boxes.

    Drops boxes without confidence or below MIN_CONFIDENCE, then
    merges overlapping duplicates: boxes are checked from high to
    low confidence and a box is dropped when its IoU with any kept
    box is above IOU_THRESHOLD. The kept boxes keep their input
    order by confidence (highest first).
    """
    confident = [
        box
        for box in boxes
        if isinstance(box.get("confidence"), (int, float))
        and box["confidence"] >= MIN_CONFIDENCE
    ]
    confident.sort(key=lambda box: box["confidence"], reverse=True)
    kept: list[dict] = []
    for box in confident:
        if all(_iou(box, other) <= IOU_THRESHOLD for other in kept):
            kept.append(box)
    return kept


def verdict(count: int) -> str:
    """Return "good" for exactly EXPECTED_LEVERS levers, else "bad"."""
    return "good" if count == EXPECTED_LEVERS else "bad"


def find_part(boxes: list[dict]) -> dict | None:
    """Return the part box above the gripper zone, or None.

    Ignores boxes without confidence, below MIN_CONFIDENCE, or
    inside the gripper zone (center y at or above GRIPPER_ZONE_Y).
    Picks the remaining box with the highest confidence.
    """
    candidates = [
        box
        for box in boxes
        if isinstance(box.get("confidence"), (int, float))
        and box["confidence"] >= MIN_CONFIDENCE
        and box["center"][1] < GRIPPER_ZONE_Y
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda box: box["confidence"])


def crop_and_enlarge(frame: numpy.ndarray, box: dict) -> numpy.ndarray:
    """Crop the box plus margin and enlarge it.

    Adds CROP_MARGIN_PX on each side (clamped to the image) and
    scales the crop by CROP_SCALE with cv2.INTER_CUBIC.
    """
    height, width = frame.shape[:2]
    x1 = max(0, int(box["x1"]) - CROP_MARGIN_PX)
    y1 = max(0, int(box["y1"]) - CROP_MARGIN_PX)
    x2 = min(width, int(box["x2"]) + CROP_MARGIN_PX)
    y2 = min(height, int(box["y2"]) + CROP_MARGIN_PX)
    if x2 <= x1 or y2 <= y1:
        raise RuntimeError("part box is outside the image")
    crop = frame[y1:y2, x1:x2]
    return cv2.resize(
        crop,
        None,
        fx=CROP_SCALE,
        fy=CROP_SCALE,
        interpolation=cv2.INTER_CUBIC,
    )


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
    annotated = draw_boxes(frame, [box])
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


def save_annotated(
    frame: numpy.ndarray,
    boxes: list[dict],
    count: int,
    recordings_dir: str | Path = "recordings",
) -> Path:
    """Save the frame with boxes and count drawn, return the path."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target_dir / f"count_{stamp}.png"
    if not cv2.imwrite(str(path), draw_levers(frame, boxes, count)):
        raise RuntimeError(f"failed to write image to {path}")
    return path


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


def run_count(
    api_key: str | None,
    frame: numpy.ndarray,
    prompt: str,
    make_client: Callable[[str], object],
    recordings_dir: str | Path = "recordings",
) -> int:
    """Find the part, count its levers; return the exit code.

    Prints "part: box=(...) confidence=...", one line per kept
    lever box (id, box, confidence), then "levers: N",
    "verdict: good|bad" and both annotated image paths. Returns 0
    on success, 3 when no part is found, 2 when the key is missing
    or an API request fails, 1 when the crop or an annotated image
    cannot be written. Errors print a short message without a
    stack trace and never include the key.
    """
    if not api_key or not api_key.strip():
        print(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    try:
        client = make_client(api_key)
        part_text = fetch_output_text(
            client, PART_PROMPT, image_to_data_url(frame)
        )
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        print(f"request failed: {exc}")
        return 2
    part = find_part(parse_output_text(part_text))
    if part is None:
        print("part: not found")
        return 3
    print(
        f"part: box=({part['x1']}, {part['y1']},"
        f" {part['x2']}, {part['y2']})"
        f" confidence={part['confidence']:.2f}"
    )
    try:
        crop = crop_and_enlarge(frame, part)
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        print(f"request failed: {exc}")
        return 1
    try:
        lever_text = fetch_output_text(
            client, prompt, image_to_data_url(crop)
        )
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        print(f"request failed: {exc}")
        return 2
    kept = count_levers(parse_output_text(lever_text))
    for box in kept:
        center_x, center_y = box["center"]
        print(
            f"id {box['id']}"
            f" box=({box['x1']}, {box['y1']}, {box['x2']}, {box['y2']})"
            f" center=({center_x:.1f}, {center_y:.1f})"
            f" confidence={box['confidence']:.2f}"
        )
    print(f"levers: {len(kept)}")
    print(f"verdict: {verdict(len(kept))}")
    try:
        full_path, crop_path = save_pair(
            draw_part(frame, part, len(kept)),
            draw_levers(crop, kept, len(kept)),
            recordings_dir,
        )
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    print(full_path)
    print(crop_path)
    return 0


def main(argv: list[str] | None = None) -> int:
    """Load an arm camera image and count orange levers with SAM 3.1."""
    parser = argparse.ArgumentParser(
        description="Count orange levers on an arm camera image."
    )
    parser.add_argument("--image", required=True, help="saved image file")
    parser.add_argument("--prompt", default=LEVER_PROMPT)
    args = parser.parse_args(argv)
    try:
        frame = load_image(args.image)
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    return run_count(
        read_api_key(), frame, args.prompt, _default_client_factory
    )


if __name__ == "__main__":
    sys.exit(main())
