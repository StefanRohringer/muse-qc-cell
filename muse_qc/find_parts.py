"""Find connector parts: empty-table diff blobs, confirmed by SAM crops.

SAM 3.1 on the full frame misses the small grey/orange parts (low
contrast on light wood), so new blobs against an empty-table reference
become candidates and each one is confirmed on a zoomed crop.
Hand-run only (real camera and API); tests use synthetic images:

    uv run python -m muse_qc.find_parts --save-empty
    uv run python -m muse_qc.find_parts --no-sam
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy

from muse_qc.camera_top import open_table_camera
from muse_qc.detect_markers import detect_table_markers, scale_px_per_mm
from muse_qc.drop_plan import BOXES_TABLE_MM
from muse_qc.part_angle import lever_angle_deg
from muse_qc.parts_image import annotate, save_annotated
from muse_qc.target_pose import ARM_BASE_IN_TABLE_MM
from muse_qc.model_check import (
    KEY_NAME,
    _is_auth_error,
    _is_connection_error,
    read_api_key,
)
from muse_qc.table_coords import (
    load_table_calibration,
    pixel_to_table_mm,
    table_homography,
)
from muse_qc.vision_sam import (
    _default_client_factory,
    fetch_output_text,
    image_to_data_url,
    parse_output_text,
)

DIFF_THRESHOLD = 30
PART_AREA_MIN_MM2 = 150
PART_AREA_MAX_MM2 = 3000
MAX_CANDIDATES = 12
CROP_SIZE_PX = 640
DEFAULT_PHRASE = "grey object"
EMPTY_PATH = "recordings/empty_table.png"
TABLE_X_MAX_MM = 990.0
TABLE_Y_MAX_MM = 540.0
GROW_PX = 10
# Skip zones for diff blobs, table mm. Parts already sorted sit in the
# 100 mm boxes (square keep-out around the box centre), the arm with
# cables and gripper rests in ARM_ZONE_TABLE_MM, and anything beyond
# REACH_MAX_MM from the joint 1 axis cannot be picked anyway.
BOX_KEEP_OUT_MM = 70.0
ARM_ZONE_TABLE_MM = ((380.0, 610.0, 0.0, 200.0),  # base/arm
                       (430.0, 590.0, 200.0, 300.0))  # resting gripper
REACH_MAX_MM = 345.0


class TooManyChanges(RuntimeError):
    """Unskipped blob limit tripped; carries results and image path."""

    def __init__(self, results, path):
        super().__init__("too many changes: light or arm moved? take a new"
                         " empty reference with --save-empty")
        self.results, self.path = results, path


def classify_skip(table_mm):
    """Return the skip reason for a candidate table position, or None.

    "in box good"/"in box bad" is within BOX_KEEP_OUT_MM (max of
    |dx|, |dy|) of the drop_plan box centre, "arm zone" is inside
    ARM_ZONE_TABLE_MM, "out of reach" is farther than REACH_MAX_MM
    from the joint 1 axis. None (no mapping) is never skipped.
    """
    if table_mm is None:
        return None
    table_x, table_y = float(table_mm[0]), float(table_mm[1])
    for name in ("good", "bad"):
        box = BOXES_TABLE_MM.get(name)
        if box is None:
            continue
        if max(abs(table_x - box[0]), abs(table_y - box[1])) \
                <= BOX_KEEP_OUT_MM:
            return f"in box {name}"
    for zone in ARM_ZONE_TABLE_MM:
        x_min, x_max, y_min, y_max = zone
        if x_min <= table_x <= x_max and y_min <= table_y <= y_max:
            return "arm zone"
    base_x, base_y = ARM_BASE_IN_TABLE_MM
    if math.dist((table_x, table_y), (base_x, base_y)) > REACH_MAX_MM:
        return "out of reach"
    return None


def detect_changes(frame, empty, scale, to_table_mm=None):
    """Pure LAB diff of frame vs empty: blur, max-channel diff,
    threshold, open, close, external contours. Keeps blobs with an
    area in mm2 inside the part range whose centre maps onto the
    table (with to_table_mm). Returns bbox/center/area_mm2/contour
    dicts (contour is the diff-mask blob contour, used for the
    lever direction and never printed or saved)."""
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    blurred_ref = cv2.GaussianBlur(empty, (5, 5), 0)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    lab_ref = cv2.cvtColor(blurred_ref, cv2.COLOR_BGR2LAB)
    gray = numpy.max(cv2.absdiff(lab, lab_ref), axis=2).astype(numpy.uint8)
    _, mask = cv2.threshold(gray, DIFF_THRESHOLD, 255, cv2.THRESH_BINARY)
    kernel = numpy.ones((3, 3), numpy.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for contour in contours:
        area_mm2 = cv2.contourArea(contour) / (scale * scale)
        if not PART_AREA_MIN_MM2 <= area_mm2 <= PART_AREA_MAX_MM2:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        center = (x + width / 2.0, y + height / 2.0)
        if to_table_mm is not None:
            table_x, table_y = to_table_mm(center)
            if not 0.0 <= table_x <= TABLE_X_MAX_MM:
                continue
            if not 0.0 <= table_y <= TABLE_Y_MAX_MM:
                continue
        blobs.append({"bbox": (x, y, width, height), "center": center,
                      "area_mm2": area_mm2, "contour": contour})
    return blobs


def crop_for_blob(frame_size, center, size):
    """Square SAM crop (x0, y0, side): side max(3x blob side, 160 px),
    centred on the blob, clipped to the (width, height) frame."""
    width, height = frame_size
    side = max(3 * max(size[0], size[1]), 160)
    side = min(int(round(side)), width, height)
    x0 = min(max(int(round(center[0] - side / 2.0)), 0), width - side)
    y0 = min(max(int(round(center[1] - side / 2.0)), 0), height - side)
    return (x0, y0, side)


def crop_box_to_frame(origin, scale, box):
    """Map a SAM box centre from 640 px crop space to frame pixels."""
    center_x = (box["x1"] + box["x2"]) / 2.0
    center_y = (box["y1"] + box["y2"]) / 2.0
    return (origin[0] + center_x / scale, origin[1] + center_y / scale)


def confirm_candidate(frame, candidate, phrase, client):
    """One SAM call on the zoomed crop; (confirmed, centre, confidence).
    Confirmed when a SAM box centre lands in the blob box grown by
    GROW_PX, taking the best confidence."""
    height, width = frame.shape[0], frame.shape[1]
    center_x, center_y = candidate["center"]
    _, _, blob_w, blob_h = candidate["bbox"]
    x0, y0, side = crop_for_blob((width, height), (center_x, center_y),
                                 (blob_w, blob_h))
    crop = frame[y0 : y0 + side, x0 : x0 + side]
    big = cv2.resize(crop, (CROP_SIZE_PX, CROP_SIZE_PX),
                     interpolation=cv2.INTER_CUBIC)
    scale = CROP_SIZE_PX / side
    text = fetch_output_text(client, phrase, image_to_data_url(big))
    blob_x, blob_y, blob_w, blob_h = candidate["bbox"]
    best = None
    for box in parse_output_text(text):
        frame_x, frame_y = crop_box_to_frame((x0, y0), scale, box)
        if not blob_x - GROW_PX <= frame_x <= blob_x + blob_w + GROW_PX:
            continue
        if not blob_y - GROW_PX <= frame_y <= blob_y + blob_h + GROW_PX:
            continue
        confidence = box["confidence"]
        score = confidence if confidence is not None else 0.0
        if best is None or score > best[0]:
            best = (score, (frame_x, frame_y), confidence)
    if best is None:
        return (False, None, None)
    return (True, best[1], best[2])


def _request_error(exc):
    """Map a SAM request failure to a short message (like run_detection)."""
    status = getattr(exc, "status_code", None)
    if status == 400:
        detail = f": {exc}" if str(exc) else ""
        return f"prompt rejected{detail}"
    if _is_auth_error(exc):
        return "key rejected"
    if _is_connection_error(exc):
        return f"network error: {exc}"
    return f"request failed: {exc}"


def save_empty():
    """Take one empty-table frame and store it as the reference."""
    try:
        frame, _ = open_table_camera()
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    path = Path(EMPTY_PATH)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), frame):
        print(f"request failed: failed to write image to {path}")
        return 1
    print(path)
    return 0


def scan_table(frame, empty, phrase=DEFAULT_PHRASE, client=None,
               recordings_dir="recordings"):
    """Scan one frame for parts, return (results, annotated image path).

    Runs the same steps as main: table mapping, empty-table diff,
    skip zones, one SAM confirm per unskipped candidate, then the
    annotated image save. Each result holds bbox, pixel, area_mm2,
    table_mm, confirmed, confidence, skipped (the skip reason or
    None), contour (the diff-mask blob contour, never printed or
    saved), lever_deg (table-frame lever direction in deg, None
    when unclear or skipped) and lever_source ("levers" or
    "outline", None when unclear or skipped) plus lever_reason
    (the unclear reason, None otherwise). Raises RuntimeError
    with the user-facing message (the
    same text main prints) instead of returning an exit code.
    """
    try:
        markers = detect_table_markers(frame)
        calibration = load_table_calibration()
        homography = table_homography(markers, calibration)
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        raise RuntimeError(
            f"table mapping: unavailable ({exc})") from exc
    scale = scale_px_per_mm(markers)
    to_table = lambda point: pixel_to_table_mm(  # noqa: E731
        homography, point, calibration)
    blobs = detect_changes(frame, empty, scale, to_table)
    results = []
    for blob in blobs:
        blob_table = to_table(blob["center"])
        results.append({
            "bbox": blob["bbox"], "pixel": blob["center"],
            "area_mm2": blob["area_mm2"], "table_mm": blob_table,
            "confirmed": False, "confidence": None,
            "skipped": classify_skip(blob_table),
            "contour": blob["contour"], "lever_deg": None,
            "lever_source": None, "lever_reason": None,
        })
    unskipped = [item for item in results if item["skipped"] is None]
    if len(unskipped) > MAX_CANDIDATES:
        results.sort(key=lambda item: item["table_mm"][0])
        try:
            path = save_annotated(annotate(frame, results), recordings_dir)
        except (OSError, RuntimeError) as exc:
            raise RuntimeError(f"request failed: {exc}") from exc
        raise TooManyChanges(results, path)
    for item in unskipped:
        confirmed, sam_center, confidence = False, None, None
        if client is not None:
            try:
                confirmed, sam_center, confidence = confirm_candidate(
                    frame, {"center": item["pixel"], "bbox": item["bbox"]},
                    phrase, client)
            except Exception as exc:  # noqa: BLE001 - mapped, no key
                raise RuntimeError(_request_error(exc)) from exc
        pixel = sam_center if confirmed else item["pixel"]
        item.update(pixel=pixel, table_mm=to_table(pixel),
                    confirmed=confirmed, confidence=confidence)
        angle = lever_angle_deg(frame, item["bbox"], item["contour"],
                                to_table)
        if angle["ok"]:
            item["lever_deg"] = angle["lever_deg"]
            item["lever_source"] = angle["source"]
        else:
            item["lever_reason"] = angle["reason"]
    results.sort(key=lambda item: item["table_mm"][0])
    try:
        path = save_annotated(annotate(frame, results), recordings_dir)
    except (OSError, RuntimeError) as exc:
        raise RuntimeError(f"request failed: {exc}") from exc
    return results, path


def main(argv=None):
    """Find table parts against the empty reference, confirm with SAM."""
    parser = argparse.ArgumentParser(
        description="Find connector parts via empty-table diff plus SAM."
    )
    parser.add_argument("--save-empty", action="store_true")
    parser.add_argument("--phrase", default=DEFAULT_PHRASE)
    parser.add_argument("--no-sam", action="store_true")
    parser.add_argument("--image", default=None)
    parser.add_argument("--empty", default=EMPTY_PATH)
    args = parser.parse_args(argv)
    if args.save_empty:
        return save_empty()
    if args.image is not None:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"cannot read image {args.image}")
            return 1
    else:
        try:
            frame, _ = open_table_camera()
        except (OSError, RuntimeError) as exc:
            print(f"request failed: {exc}")
            return 1
    empty = cv2.imread(args.empty)
    if empty is None:
        print(f"missing empty reference {args.empty}:"
              " take one with --save-empty")
        return 2
    if empty.shape != frame.shape:
        print("size mismatch: take a new empty reference with --save-empty")
        return 2
    client = None
    if not args.no_sam:
        api_key = read_api_key()
        if not api_key or not api_key.strip():
            print(f"{KEY_NAME} is missing: set it in .env or the environment")
            return 2
        try:
            client = _default_client_factory(api_key)
        except Exception as exc:  # noqa: BLE001 - mapped, key never printed
            print(_request_error(exc))
            return 1
    try:
        results, path = scan_table(frame, empty, args.phrase, client)
    except TooManyChanges as exc:
        skipped = sum(1 for item in exc.results if item["skipped"] is not None)
        print(f"blobs: {len(exc.results)} total, {skipped} skipped,"
              f" {len(exc.results) - skipped} unskipped")
        print(exc.path)
        print(exc)
        return 1
    except RuntimeError as exc:
        print(exc)
        return 1
    for index, item in enumerate(results):
        pixel_x, pixel_y = item["pixel"]
        table_x, table_y = item["table_mm"]
        if item["skipped"] is not None:
            print(f"part {index} center=({pixel_x:.0f}, {pixel_y:.0f})"
                  f" area_mm2={item['area_mm2']:.0f}"
                  f" table_mm=({table_x:.1f}, {table_y:.1f})"
                  f" skipped: {item['skipped']}")
            continue
        if client is None:
            sam_text = "skipped"
        elif not item["confirmed"]:
            sam_text = "no box"
        elif item["confidence"] is None:
            sam_text = "confirmed n/a"
        else:
            sam_text = f"confirmed {item['confidence']:.2f}"
        if item["lever_deg"] is None:
            lever_text = f"lever_deg=? ({item['lever_reason']})"
        else:
            shown = float(item["lever_deg"])
            if round(shown, 1) >= 180.0:
                shown = 0.0
            lever_text = f"lever_deg={shown:.1f} ({item['lever_source']})"
        print(f"part {index} center=({pixel_x:.0f}, {pixel_y:.0f})"
              f" area_mm2={item['area_mm2']:.0f}"
              f" table_mm=({table_x:.1f}, {table_y:.1f}) sam={sam_text}"
              f" {lever_text}")
    skipped = sum(1 for item in results if item["skipped"] is not None)
    skipped_text = f" ({skipped} skipped)" if skipped else ""
    if client is None:
        print(f"candidates: {len(results)}{skipped_text}")
    else:
        done = sum(1 for item in results if item["confirmed"])
        print(f"parts: {done} confirmed of {len(results)}"
              f" candidates{skipped_text}")
    print(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
