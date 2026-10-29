"""Count brown levers on an arm camera image with SAM 3.1 (hand-run).

Two steps: SAM 3.1 first finds the part on the full image with
PART_PROMPT, then SAM 3.1 counts the levers on the cropped and
enlarged part with LEVER_PROMPT ("brown bar": the orange levers
look brown under the evening lamp). Boxes at or above MIN_CONFIDENCE
are kept, overlapping duplicates are merged (IoU above
IOU_THRESHOLD keeps the higher confidence), and boxes shorter than
MIN_BAR_PART_FRAC of the enlarged part height or shorter than
MIN_BAR_TALLEST_FRAC of the tallest confident bar are dropped as
remnants.
Prints the part box, one dropped line per remnant, one line per
kept lever box plus "levers: N" and
"verdict: good|bad", and saves two annotated copies:
recordings/count_<timestamp>.png (full image with the part box)
and recordings/count_<timestamp>_crop.png (enlarged crop with the
lever boxes), plus recordings/count_<timestamp>.json with the
inspection record. A good part has exactly EXPECTED_LEVERS levers.

Run by the human only (it contacts the Meta Model API):

    uv run python -m muse_qc.count_clips --image recordings/armcam_test.png
"""

from __future__ import annotations

import argparse
import math
import sys
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy

from muse_qc.inspection.draw import draw_levers, draw_part, save_pair
from muse_qc.inspection.record import build_record, write_record
from muse_qc.model_check import KEY_NAME, read_api_key
from muse_qc.vision_sam import (
    _default_client_factory,
    fetch_output_text,
    image_to_data_url,
    load_image,
    parse_output_text,
)

EXPECTED_LEVERS = 3
MIN_CONFIDENCE = 0.5
IOU_THRESHOLD = 0.5
PART_PROMPT = "small part"
LEVER_PROMPT = "brown bar"
GRIPPER_ZONE_Y = 380
CROP_MARGIN_PX = 15
CROP_SCALE = 8
MIN_BAR_PART_FRAC = 0.35
MIN_BAR_TALLEST_FRAC = 0.6
EXPECTED_PART_PX = (340, 201)
MAX_PART_OFFSET_PX = 60


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


def drop_short_bars(
    boxes: list[dict], part_height_px: int
) -> tuple[list[dict], list[dict]]:
    """Split lever boxes into bars and too-short remnants.

    A bar must satisfy both rules: height at least
    MIN_BAR_PART_FRAC of the enlarged part height
    (part_height_px * CROP_SCALE, without the crop margin) and
    height at least MIN_BAR_TALLEST_FRAC of the tallest box in
    the input. A box of exactly either limit is kept. An empty
    input stays empty.
    """
    if not boxes:
        return [], []
    part_limit = MIN_BAR_PART_FRAC * part_height_px * CROP_SCALE
    tallest = max(box["y2"] - box["y1"] for box in boxes)
    tallest_limit = MIN_BAR_TALLEST_FRAC * tallest
    bars = []
    short = []
    for box in boxes:
        height = box["y2"] - box["y1"]
        if height >= part_limit and height >= tallest_limit:
            bars.append(box)
        else:
            short.append(box)
    return bars, short


def verdict(count: int) -> str:
    """Return "good" for exactly EXPECTED_LEVERS levers, else "bad"."""
    return "good" if count == EXPECTED_LEVERS else "bad"


def find_part(boxes: list[dict], expected_px=None) -> dict | None:
    """Return the part box above the gripper zone, or None.

    Ignores boxes without confidence, below MIN_CONFIDENCE, or
    inside the gripper zone (center y at or above GRIPPER_ZONE_Y).
    Without expected_px picks the remaining box with the highest
    confidence. With expected_px (the target part position in the
    arm camera image at the hover pose) picks the nearest centre
    instead, or None when even that is farther than
    MAX_PART_OFFSET_PX away.
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
    if expected_px is None:
        return max(candidates, key=lambda box: box["confidence"])
    target = (float(expected_px[0]), float(expected_px[1]))
    nearest = min(candidates,
                  key=lambda box: math.dist(box["center"], target))
    if math.dist(nearest["center"], target) > MAX_PART_OFFSET_PX:
        return None
    return nearest


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


def _fail(reason: str, stage: str) -> dict:
    """Return the count_frame result dict for a failed inspection."""
    return {
        "ok": False,
        "reason": reason,
        "stage": stage,
        "levers": None,
        "verdict": None,
        "full_image": "",
        "crop_image": "",
        "record": "",
        "part": None,
        "kept": [],
        "dropped": [],
    }


def count_frame(
    client: object,
    frame: numpy.ndarray,
    prompt: str,
    recordings_dir: str | Path = "recordings",
    source_image: str | Path = "",
    expected_px=None,
) -> dict:
    """Find the part on one frame and count its levers; return a dict.

    Runs the same two SAM calls as run_count (part, then levers on
    the enlarged crop) and saves the two annotated images plus the
    JSON record exactly as run_count does. With expected_px only
    the target part near that image position is accepted. Returns
    {"ok", "reason", "levers", "verdict", "full_image",
    "crop_image", "record"} plus "stage" (which step failed),
    "part", "kept" and "dropped" for printing. Prints nothing and
    never includes any key.
    """
    try:
        part_text = fetch_output_text(
            client, PART_PROMPT, image_to_data_url(frame)
        )
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        return _fail(f"request failed: {exc}", "part-request")
    part = find_part(parse_output_text(part_text), expected_px)
    if part is None:
        if expected_px is not None:
            return _fail("part: target not in view", "no-part")
        return _fail("part: not found", "no-part")
    try:
        crop = crop_and_enlarge(frame, part)
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        return _fail(f"request failed: {exc}", "crop")
    try:
        lever_text = fetch_output_text(
            client, prompt, image_to_data_url(crop)
        )
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        return _fail(f"request failed: {exc}", "lever-request")
    kept, dropped = drop_short_bars(
        count_levers(parse_output_text(lever_text)),
        int(part["y2"]) - int(part["y1"]),
    )
    try:
        full_path, crop_path = save_pair(
            draw_part(frame, part, len(kept)),
            draw_levers(crop, kept, len(kept)),
            recordings_dir,
        )
    except (OSError, RuntimeError) as exc:
        failed = _fail(f"request failed: {exc}", "images")
        failed.update({"levers": len(kept), "verdict": verdict(len(kept)),
                       "part": part, "kept": kept, "dropped": dropped})
        return failed
    offset = None
    if expected_px is not None:
        offset = math.dist(
            part["center"],
            (float(expected_px[0]), float(expected_px[1])))
    try:
        record_path = write_record(
            build_record(
                source_image=str(source_image),
                full_image=str(full_path),
                crop_image=str(crop_path),
                part=part,
                lever_prompt=prompt,
                kept=kept,
                dropped=dropped,
                expected_levers=EXPECTED_LEVERS,
                part_offset_px=offset,
            ),
            recordings_dir,
        )
    except (OSError, RuntimeError) as exc:
        failed = _fail(f"request failed: {exc}", "record")
        failed.update({"levers": len(kept), "verdict": verdict(len(kept)),
                       "part": part, "kept": kept, "dropped": dropped})
        return failed
    return {
        "ok": True,
        "reason": "",
        "stage": "done",
        "levers": len(kept),
        "verdict": verdict(len(kept)),
        "full_image": str(full_path),
        "crop_image": str(crop_path),
        "record": str(record_path),
        "part": part,
        "kept": kept,
        "dropped": dropped,
    }


def run_count(
    api_key: str | None,
    frame: numpy.ndarray,
    prompt: str,
    make_client: Callable[[str], object],
    recordings_dir: str | Path = "recordings",
    source_image: str | Path = "",
    expected_px=None,
) -> int:
    """Find the part, count its levers; return the exit code.

    Prints "part: box=(...) confidence=...", one dropped line per
    too-short remnant, one line per kept lever box (id, box,
    confidence), then "levers: N", "verdict: good|bad", both
    annotated image paths and the JSON record path. Returns 0
    on success, 3 when no part is found, 2 when the key is missing
    or an API request fails, 1 when the crop, an annotated image
    or the JSON record cannot be written. Errors print a short
    message without a stack trace and never include the key.
    """
    if not api_key or not api_key.strip():
        print(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    try:
        client = make_client(api_key)
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        print(f"request failed: {exc}")
        return 2
    result = count_frame(client, frame, prompt, recordings_dir,
                         source_image, expected_px)
    if result["part"] is not None:
        part, kept, dropped = result["part"], result["kept"], result["dropped"]
        print(
            f"part: box=({part['x1']}, {part['y1']},"
            f" {part['x2']}, {part['y2']})"
            f" confidence={part['confidence']:.2f}"
        )
        for box in dropped:
            print(
                f"dropped (too short): id {box['id']}"
                f" box=({box['x1']}, {box['y1']}, {box['x2']}, {box['y2']})"
                f" confidence={box['confidence']:.2f}"
            )
        for box in kept:
            center_x, center_y = box["center"]
            print(
                f"id {box['id']}"
                f" box=({box['x1']}, {box['y1']}, {box['x2']}, {box['y2']})"
                f" center=({center_x:.1f}, {center_y:.1f})"
                f" confidence={box['confidence']:.2f}"
            )
        print(f"levers: {result['levers']}")
        print(f"verdict: {result['verdict']}")
    if not result["ok"]:
        print(result["reason"])
        if result["stage"] == "no-part":
            return 3
        if result["stage"] in ("part-request", "lever-request"):
            return 2
        return 1
    print(result["full_image"])
    print(result["crop_image"])
    print(result["record"])
    return 0


def main(argv: list[str] | None = None) -> int:
    """Load an arm camera image and count orange levers with SAM 3.1."""
    parser = argparse.ArgumentParser(
        description="Count orange levers on an arm camera image."
    )
    parser.add_argument("--image", required=True, help="saved image file")
    parser.add_argument("--prompt", default=LEVER_PROMPT)
    parser.add_argument("--expect-target", action="store_true",
                        help="only accept the part near EXPECTED_PART_PX,"
                        " as the hover inspection does")
    args = parser.parse_args(argv)
    try:
        frame = load_image(args.image)
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    return run_count(
        read_api_key(),
        frame,
        args.prompt,
        _default_client_factory,
        source_image=args.image,
        expected_px=EXPECTED_PART_PX if args.expect_target else None,
    )


if __name__ == "__main__":
    sys.exit(main())
