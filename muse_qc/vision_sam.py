"""Find parts on the table with SAM 3.1 visual grounding (hand-run).

Takes one frame from the table camera (or a saved image), sends it
with a noun phrase to SAM 3.1 through the openai SDK and draws the
returned boxes onto a copy of the image. The key is read with
muse_qc.model_check (never printed). Masks are not used yet.

Run by the human only (it opens the real table camera and contacts
the Meta Model API over the network):

    uv run python -m muse_qc.vision_sam "grey connector"
    uv run python -m muse_qc.vision_sam "grey connector" --image path.png
"""

from __future__ import annotations

import argparse
import base64
import re
import sys
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.camera_top import open_table_camera
from muse_qc.detect_markers import detect_table_markers
from muse_qc.table_coords import (
    boxes_to_table_mm,
    load_table_calibration,
)
from muse_qc.model_check import (
    BASE_URL,
    KEY_NAME,
    _is_auth_error,
    _is_connection_error,
    read_api_key,
)

MODEL = "sam-3.1"
JPEG_QUALITY = 90

_BOX_RE = re.compile(r"(\d+)\s*<\|\s*box\s*;([^|]*)\|>")


def _number(token: str) -> int | float:
    """Parse an int or float box field."""
    token = token.strip()
    try:
        return int(token)
    except ValueError:
        return float(token)


def parse_output_text(text: str) -> list[dict]:
    """Parse SAM 3.1 box records into a list of dicts.

    Records look like id<|box;x1=..;y1=..;x2=..;y2=..;w=..;h=..;c=..|>,
    optionally followed by <|mask;...|>, separated by commas, with an
    optional <0f> header. Fields are read by key, ids may be sparse,
    mask segments are ignored. Empty output means zero matches. Each
    dict holds id, x1, y1, x2, y2, center (pixels) and confidence
    (or None without c=).
    """
    boxes: list[dict] = []
    if not text or not text.strip():
        return boxes
    for match in _BOX_RE.finditer(text):
        fields: dict[str, str] = {}
        for part in match.group(2).split(";"):
            name, sep, value = part.partition("=")
            if sep and name.strip():
                fields[name.strip()] = value.strip()
        try:
            x1 = _number(fields["x1"])
            y1 = _number(fields["y1"])
            x2 = _number(fields["x2"])
            y2 = _number(fields["y2"])
        except (KeyError, ValueError):
            continue
        confidence = None
        if "c" in fields:
            try:
                confidence = float(fields["c"])
            except ValueError:
                confidence = None
        boxes.append(
            {
                "id": int(match.group(1)),
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                "confidence": confidence,
            }
        )
    return boxes


def image_to_data_url(frame: numpy.ndarray) -> str:
    """Encode a frame as a JPEG base64 data URL."""
    ok, encoded = cv2.imencode(
        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    )
    if not ok:
        raise RuntimeError("failed to encode frame as JPEG")
    raw = base64.b64encode(bytes(encoded)).decode("ascii")
    return f"data:image/jpeg;base64,{raw}"


def fetch_output_text(client: object, noun_phrase: str, data_url: str) -> str:
    """Send one POST /v1/responses call and return its output text."""
    create = getattr(getattr(client, "responses"), "create")
    response = create(
        model=MODEL,
        input=[
            {
                "role": "user",
                "content": [
                    {"type": "input_text", "text": noun_phrase},
                    {"type": "input_image", "image_url": data_url},
                ],
            }
        ],
        metadata={"include_confidence": "true"},
    )
    return getattr(response, "output_text", "") or ""


def draw_boxes(frame: numpy.ndarray, boxes: list[dict]) -> numpy.ndarray:
    """Return a copy of frame with boxes and ids drawn."""
    annotated = frame.copy()
    for box in boxes:
        x1, y1, x2, y2 = (
            int(box["x1"]),
            int(box["y1"]),
            int(box["x2"]),
            int(box["y2"]),
        )
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            annotated,
            f"id {box['id']}",
            (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
    return annotated


def save_annotated(
    frame: numpy.ndarray,
    boxes: list[dict],
    recordings_dir: str | Path = "recordings",
) -> Path:
    """Save the frame with boxes drawn, return the file path."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target_dir / f"sam_{stamp}.png"
    if not cv2.imwrite(str(path), draw_boxes(frame, boxes)):
        raise RuntimeError(f"failed to write image to {path}")
    return path


def load_image(path: str | Path) -> numpy.ndarray:
    """Read a saved image, raising a clear error when it cannot be read."""
    frame = cv2.imread(str(path))
    if frame is None:
        raise RuntimeError(f"cannot read image {path}")
    return frame


def _default_client_factory(api_key: str) -> object:
    """Build the real client (only called by main, never by tests)."""
    from openai import OpenAI

    return OpenAI(api_key=api_key, base_url=BASE_URL)


def run_detection(
    api_key: str | None,
    noun_phrase: str,
    frame: numpy.ndarray,
    make_client: Callable[[str], object],
    recordings_dir: str | Path = "recordings",
    load_calibration: Callable[[], dict | None] = load_table_calibration,
    detect_markers: Callable[[numpy.ndarray], dict] = detect_table_markers,
    map_table: bool = True,
) -> int:
    """Run one detection on frame; return the exit code.

    Prints the table mapping status, one line per box (id, box,
    center px, confidence, table_mm when the mapping works), the
    count and the annotated image path. With map_table=False the
    table mapping is skipped (saved image files) and the status
    line reads "table mapping: skipped (image file)". Returns 0
    on success, 2 when the key is missing, 1 otherwise. Errors
    print a short message without a stack trace and never include
    the key.
    """
    if not api_key or not api_key.strip():
        print(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    try:
        client = make_client(api_key)
        text = fetch_output_text(client, noun_phrase, image_to_data_url(frame))
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        status = getattr(exc, "status_code", None)
        if status == 400:
            detail = f": {exc}" if str(exc) else ""
            print(f"prompt rejected{detail}")
            return 1
        if _is_auth_error(exc):
            print("key rejected")
            return 1
        if _is_connection_error(exc):
            print(f"network error: {exc}")
            return 1
        print(f"request failed: {exc}")
        return 1
    boxes = parse_output_text(text)
    if map_table:
        status, table_points = boxes_to_table_mm(
            boxes, frame, load_calibration, detect_markers
        )
        print(f"table mapping: {status}")
    else:
        table_points = [None] * len(boxes)
        print("table mapping: skipped (image file)")
    for box, point_mm in zip(boxes, table_points):
        confidence = (
            f"{box['confidence']:.2f}"
            if box["confidence"] is not None
            else "n/a"
        )
        center_x, center_y = box["center"]
        table_suffix = (
            f" table_mm=({point_mm[0]:.1f}, {point_mm[1]:.1f})"
            if point_mm is not None
            else ""
        )
        print(
            f"id {box['id']}"
            f" box=({box['x1']}, {box['y1']}, {box['x2']}, {box['y2']})"
            f" center=({center_x:.1f}, {center_y:.1f})"
            f" confidence={confidence}"
            f"{table_suffix}"
        )
    print(f"boxes: {len(boxes)}")
    try:
        path = save_annotated(frame, boxes, recordings_dir)
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    print(path)
    return 0


def main(argv: list[str] | None = None) -> int:
    """Take a frame, detect the noun phrase with SAM 3.1, save boxes."""
    parser = argparse.ArgumentParser(
        description="Detect a noun phrase on the table with SAM 3.1."
    )
    parser.add_argument("phrase", help='noun phrase, e.g. "grey connector"')
    parser.add_argument(
        "--image",
        default=None,
        help="use a saved image instead of the table camera",
    )
    args = parser.parse_args(argv)
    try:
        if args.image is not None:
            frame = load_image(args.image)
        else:
            frame, _ = open_table_camera()
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    return run_detection(
        read_api_key(),
        args.phrase,
        frame,
        _default_client_factory,
        map_table=(args.image is None),
    )


if __name__ == "__main__":
    sys.exit(main())
