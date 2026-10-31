"""Find connector parts: empty-table diff blobs, confirmed by SAM crops.

SAM 3.1 on the full frame misses the small grey/orange parts (low
contrast on light wood), so new blobs against an empty-table reference
become candidates and each one is confirmed on a zoomed crop.
Hand-run only (real camera and API); tests use synthetic images:

    uv run python -m muse_qc.vision.find_parts --save-empty
    uv run python -m muse_qc.vision.find_parts --no-sam
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy

from muse_qc.vision import table_mapping
from muse_qc.vision.camera_top import open_table_camera
from muse_qc.vision.detect_markers import detect_table_markers, scale_px_per_mm
from muse_qc.vision.find_skip import classify_skip
from muse_qc.vision.part_angle import lever_angle_deg
from muse_qc.vision.parts_image import annotate, crop_to_markers, save_annotated
from muse_qc.vision.known_match import match_known
from muse_qc.checks.model_check import (
    KEY_NAME,
    _is_auth_error,
    _is_connection_error,
    read_api_key,
)
from muse_qc.vision.table_coords import (
    load_table_calibration,
    pixel_to_table_mm,
    table_homography,
)
from muse_qc.vision.vision_sam import (
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
# Second SAM try for real-looking candidates: SAM with DEFAULT_PHRASE
# misses real parts on some crops, so a "no box" blob with an area in
# RETRY_AREA_MM2 gets one more confirm with RETRY_PHRASE.
RETRY_AREA_MM2 = (350.0, 900.0)
RETRY_PHRASE = "small part"


class TooManyChanges(RuntimeError):
    """Unskipped blob limit tripped; carries results and image path."""

    def __init__(self, results, path):
        super().__init__("too many changes: light or arm moved? take a new"
                         " empty reference with --save-empty")
        self.results, self.path = results, path


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


def _save_cropped(frame, results, markers, recordings_dir):
    """Annotate and save, cropped to the table markers.

    Only the saved image is cropped; the frame used for detection,
    calibration and all table_mm numbers stays untouched. A failed
    crop never raises: the uncropped image is saved instead.
    """
    annotated = annotate(frame, results)
    try:
        annotated = crop_to_markers(annotated, markers)
    except Exception:  # noqa: BLE001 - uncropped fallback, never raised
        pass
    return save_annotated(annotated, recordings_dir)


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
               recordings_dir="recordings", known=None):
    """Scan one frame for parts, return (results, annotated image path).

    Runs the same steps as main: table mapping, empty-table diff,
    skip zones, one SAM confirm per unskipped candidate plus one
    retry with RETRY_PHRASE when "no box" meets RETRY_AREA_MM2,
    then the annotated image save. With known (confirmed
    table_mm points of the previous scan) a candidate within
    KNOWN_MATCH_MM is confirmed without any SAM request
    (confidence copied from that known part). Each result holds
    bbox, pixel, area_mm2, table_mm, confirmed, confidence,
    skipped (the skip reason or None), retried (True only for a
    retry confirmation), known (True only for a known match),
    contour (the diff-mask blob contour, never printed or
    saved), lever_deg (table-frame lever direction in deg, None
    when unclear or skipped) and lever_source ("levers", "slot" or
    "outline", None when unclear or skipped) plus lever_reason
    (the unclear reason, None otherwise). Raises RuntimeError
    with the user-facing message (the
    same text main prints) instead of returning an exit code.
    The table mapping comes from all 4 markers and is saved for
    later; when markers are covered (the arm at the inspection
    station hides marker 0) the last mapping at most
    MAPPING_MAX_AGE_H hours old is reused with one printed line.
    """
    mapping_path = str(Path(recordings_dir) / "table_mapping_last.json")
    try:
        markers = detect_table_markers(frame)
        calibration = load_table_calibration()
        homography = table_homography(markers, calibration)
    except (RuntimeError, OSError, ValueError, KeyError) as exc:
        loaded = table_mapping.load_last_mapping(mapping_path)
        if loaded is None:
            raise RuntimeError(
                f"table mapping: unavailable ({exc})") from exc
        homography, scale, stamp = loaded
        try:
            calibration = load_table_calibration()
        except (OSError, ValueError, KeyError):
            calibration = None
        markers = None
        print(f"table mapping: from last full scan {stamp}")
    else:
        scale = scale_px_per_mm(markers)
        try:
            table_mapping.save_last_mapping(homography, markers, scale,
                                            mapping_path)
        except Exception as exc:  # noqa: BLE001 - never break a scan
            reason = str(exc).splitlines()
            short = reason[0][:160] if reason and reason[0] \
                else type(exc).__name__
            print(f"table mapping: not saved ({short})")
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
            "retried": False, "known": False,
        })
    unskipped = [item for item in results if item["skipped"] is None]
    if len(unskipped) > MAX_CANDIDATES:
        results.sort(key=lambda item: item["table_mm"][0])
        try:
            path = _save_cropped(frame, results, markers, recordings_dir)
        except (OSError, RuntimeError) as exc:
            raise RuntimeError(f"request failed: {exc}") from exc
        raise TooManyChanges(results, path)
    for item in unskipped:
        confirmed, sam_center, confidence = False, None, None
        retried = False
        hit, known_confidence, known_point = match_known(
            known, item["table_mm"])
        if hit:
            confirmed, confidence = True, known_confidence
            item["known"] = True
        elif client is not None:
            try:
                confirmed, sam_center, confidence = confirm_candidate(
                    frame, {"center": item["pixel"], "bbox": item["bbox"]},
                    phrase, client)
                if not confirmed and RETRY_AREA_MM2[0] <= item[
                        "area_mm2"] <= RETRY_AREA_MM2[1]:
                    second = confirm_candidate(
                        frame, {"center": item["pixel"],
                                "bbox": item["bbox"]},
                        RETRY_PHRASE, client)
                    confirmed, sam_center, confidence = second
                    retried = confirmed
            except Exception as exc:  # noqa: BLE001 - mapped, no key
                raise RuntimeError(_request_error(exc)) from exc
        pixel = (sam_center if confirmed and sam_center is not None
                 else item["pixel"])
        item.update(pixel=pixel, table_mm=to_table(pixel),
                    confirmed=confirmed, confidence=confidence,
                    retried=retried)
        if hit:
            item["table_mm"] = [float(known_point[0]),
                                float(known_point[1])]
        angle = lever_angle_deg(frame, item["bbox"], item["contour"],
                                to_table)
        if angle["ok"]:
            item["lever_deg"] = angle["lever_deg"]
            item["lever_source"] = angle["source"]
        else:
            item["lever_reason"] = angle["reason"]
    results.sort(key=lambda item: item["table_mm"][0])
    try:
        path = _save_cropped(frame, results, markers, recordings_dir)
    except (OSError, RuntimeError) as exc:
        raise RuntimeError(f"request failed: {exc}") from exc
    return results, path


def main(argv=None):
    """Find table parts against the empty reference, confirm with SAM.

    Moved to muse_qc.vision.parts_cli to keep this module within
    the 400 line limit; kept here so python -m and the tests keep
    calling find_parts.main.
    """
    from muse_qc.vision.parts_cli import main as cli_main

    return cli_main(argv)


if __name__ == "__main__":
    sys.exit(main())
