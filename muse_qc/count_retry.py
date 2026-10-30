"""Retry the lever count when SAM misses the 3 levers of a good part.

SAM is not fully deterministic: the same image gave 0 levers
three times, then 1. So every first count other than exactly 3
is retried, and the part is good only when two retry counts each
give exactly 3. The first retry is a fresh camera frame (the arm
may still have moved slightly during the photo), counted with
the normal prompt, scale and margin and saved as a new
armcam_<timestamp>.png; only then come the crop and prompt
variants in RETRY_VARIANTS. Without two 3s the first result
stands (verdict bad), so an unconfirmed single 3 never turns
into good. A failed first count (no part, request error) is
returned unchanged, never retried.

Run by the human only (it contacts the Meta Model API):

    uv run python -m muse_qc.count_retry --image recordings/armcam_test.png
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc import count_clips
from muse_qc.model_check import KEY_NAME, read_api_key
from muse_qc.vision_sam import _default_client_factory, load_image

RETRY_VARIANTS = (
    ("brown bar", 6, 15),
    ("brown bar", 10, 25),
    ("orange bar", 8, 15),
    ("orange bar", 6, 15),
)


def _save_fresh_frame(
    frame: numpy.ndarray, recordings_dir: str | Path,
) -> Path:
    """Save a retry frame as armcam_<timestamp>.png, return its path."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    path = target_dir / f"armcam_{datetime.now():%Y%m%d_%H%M%S}.png"
    try:
        cv2.imwrite(str(path), frame)
    except Exception:  # noqa: BLE001 - the frame is still counted
        pass
    return path


def count_with_retry(
    client: object,
    frame: numpy.ndarray,
    prompt: str,
    recordings_dir: str | Path = "recordings",
    source_image: str | Path = "",
    expected_px=None,
    count_fn: Callable = count_clips.count_frame,
    grab_frame: Callable[[], numpy.ndarray | None] | None = None,
) -> dict:
    """Count levers, retrying any first count other than exactly 3.

    Runs the normal count first: a failed count is returned
    unchanged, and exactly 3 levers are returned as good, both
    without retry. Otherwise retries follow in order: first a
    fresh frame from grab_frame (when given) with the normal
    prompt, scale and margin, then RETRY_VARIANTS on the
    original frame. The part is good only when two retry counts
    each give exactly 3 levers; returns that second 3 on
    arrival, and stops early when the remaining retries can no
    longer reach two 3s. Otherwise the first result stands
    (verdict bad), so an unconfirmed single 3 never turns into
    good. Failed retry counts simply count as not 3. The
    returned dict gains "attempts", one (prompt, scale, margin,
    levers) tuple per executed count, starting with the normal
    count. Prints nothing and never includes any key.
    """
    first = count_fn(
        client, frame, prompt, recordings_dir, source_image,
        expected_px=expected_px,
    )
    attempts = [(
        prompt,
        count_clips.CROP_SCALE,
        count_clips.CROP_MARGIN_PX,
        first.get("levers"),
    )]
    if not first.get("ok") or first.get("levers") == 3:
        if first.get("ok"):
            first["attempts"] = attempts
        return first
    specs: list[tuple] = []
    if grab_frame is not None:
        try:
            fresh = grab_frame()
        except Exception:  # noqa: BLE001 - fall through to variants
            fresh = None
        if fresh is not None:
            specs.append((
                prompt,
                count_clips.CROP_SCALE,
                count_clips.CROP_MARGIN_PX,
                fresh,
                _save_fresh_frame(fresh, recordings_dir),
            ))
    for variant_prompt, scale, margin in RETRY_VARIANTS:
        specs.append(
            (variant_prompt, scale, margin, frame, source_image))
    threes = 0
    for index, (retry_prompt, scale, margin, retry_frame,
                retry_source) in enumerate(specs):
        result = count_fn(
            client, retry_frame, retry_prompt, recordings_dir,
            retry_source, expected_px=expected_px,
            crop_scale=scale, crop_margin_px=margin,
        )
        attempts.append(
            (retry_prompt, scale, margin, result.get("levers")))
        if result.get("ok") and result.get("levers") == 3:
            threes += 1
            if threes == 2:
                result["attempts"] = attempts
                return result
        if threes + len(specs) - index - 1 < 2:
            break
    first["attempts"] = attempts
    return first


def main(argv: list[str] | None = None) -> int:
    """Count levers on a saved image with the retry rule.

    Always uses --expect-target behaviour (the held-part position)
    and never grabs a fresh frame. Prints one line per attempt
    ("attempt N: prompt P scale S margin M levers L"), then
    "levers: N" and "verdict: ...". Returns 0 on a successful
    count, 1 otherwise. Errors print a short message without a
    stack trace and never include the key.
    """
    parser = argparse.ArgumentParser(
        description="Count levers on a saved image with the retry rule."
    )
    parser.add_argument("--image", required=True, help="saved image file")
    args = parser.parse_args(argv)
    try:
        frame = load_image(args.image)
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    api_key = read_api_key()
    if not api_key or not api_key.strip():
        print(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    try:
        client = _default_client_factory(api_key)
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        print(f"request failed: {exc}")
        return 2
    result = count_with_retry(
        client,
        frame,
        count_clips.LEVER_PROMPT,
        source_image=args.image,
        expected_px=count_clips.EXPECTED_PART_PX,
    )
    attempts = result.get("attempts")
    if not attempts:
        print(result.get("reason", "count failed"))
        return 1
    for number, (retry_prompt, scale, margin, levers) in enumerate(
        attempts, start=1
    ):
        print(
            f"attempt {number}: prompt {retry_prompt}"
            f" scale {scale} margin {margin} levers {levers}"
        )
    print(f"levers: {result['levers']}")
    print(f"verdict: {result['verdict']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
