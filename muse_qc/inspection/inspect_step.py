"""Inspect the part in the gripper after the lift, decide the box. Main thread only.

Used by muse_qc.pick.pick_test --inspect while the control loop holds
the last lift waypoint with the part held: grab one frame with the
arm camera, save it as recordings/armcam_<timestamp>.png, count
the levers with count_clips.count_frame and map the verdict to a
box ("good" or "bad"). The camera sits on the gripper, so a held
part always appears at the same image position
(count_clips.EXPECTED_PART_PX). The control loop thread never
touches this module: it only holds the pose and takes the verdict
through PickLoop.set_verdict.
"""

from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path

import cv2

from muse_qc.vision import camera_arm
from muse_qc.inspection import count_clips
from muse_qc.inspection import count_retry
from muse_qc.checks.model_check import read_api_key as _read_api_key
from muse_qc.pick.pick_plan import check_pick_plan
from muse_qc.vision.vision_sam import _default_client_factory as _make_client


def check_prereqs(out=print, find_node=None, read_key=None):
    """Check the arm camera node and the API key; return the node or None.

    Prints "inspect: camera <node>" and "inspect: api key present"
    (never the key); on a missing camera or key prints a short
    message and returns None.
    """
    node_of = find_node or camera_arm.find_arm_color_node
    key_of = read_key or _read_api_key
    node = node_of()
    if node is None:
        out("inspect: arm camera not found")
        return None
    out(f"inspect: camera {node}")
    if not (key_of() or "").strip():
        out("inspect: MODEL_API_KEY is missing")
        return None
    out("inspect: api key present")
    return node


def check_inspect_plan(built, start_deg, check_fn=None, tip_fn=None):
    """Check an inspect plan branch by branch. Pure, moves nothing.

    The shared hover/lower/lift plus the good branch are chained
    from the start pose as in check_pick_plan; the bad branch is
    chained from the last lift waypoint, the pose the loop really
    holds when it takes the bad verdict (chaining it after the
    good return at rest would refuse every plan on the step
    limits). Any refusal fails the whole plan; the index is
    absolute in built["plan"]. Without ranges_by_box the whole
    plan is checked chained from the start pose.
    """
    plan = built["plan"]
    try:
        bad = built["ranges_by_box"]["bad"]
        shared = built["shared_ranges"]
        lift_end = shared["lift"][1]
        good_end = built["ranges"]["return"][1]
    except (KeyError, TypeError):
        return check_pick_plan(plan, start_deg, check_fn, tip_fn)
    ok, at, res = check_pick_plan(plan[:good_end], start_deg,
                                  check_fn, tip_fn)
    if not ok:
        return False, at, res
    bad_low = min(low for low, _ in bad.values())
    bad_high = max(high for _, high in bad.values())
    hold_end = shared.get("to-station", (None, lift_end))[1]
    seed = plan[hold_end - 1][1]
    ok, at, res = check_pick_plan(plan[bad_low:bad_high], seed,
                                  check_fn, tip_fn)
    if not ok:
        return False, bad_low + at, res
    return True, -1, None


def run_inspection(
    out=print,
    find_node=None,
    open_camera=None,
    read_key=None,
    make_client=None,
    count_frame_fn=None,
    recordings_dir: str | Path = "recordings",
) -> str | None:
    """Grab one arm camera frame, count levers, return the box or None.

    Prints "inspect: levers N verdict good|bad" plus the two
    annotated image paths and the record path on success. A
    0-lever count on a found part is retried through
    count_retry.count_with_retry (fresh frame first, then other
    crop sizes and prompts), printing "inspect retry: N attempts"
    before the levers line; 1, 2 or 3 levers are never retried.
    Any failure (no camera node, no frame, missing key, API
    error, no part found) prints "inspect failed: ..." and
    returns None. The key is never printed. All hardware and API
    access is injected (fakes in tests), the defaults are the
    real helpers.
    """
    node_of = find_node or camera_arm.find_arm_color_node
    camera_of = open_camera or camera_arm.open_arm_camera
    key_of = read_key or _read_api_key
    client_of = make_client or _make_client
    use_retry = count_frame_fn is None
    counter = count_frame_fn or count_retry.count_with_retry
    node = node_of()
    if node is None:
        out("inspect failed: arm camera not found")
        return None
    try:
        frame = camera_of(node)
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        out(f"inspect failed: {exc}")
        return None
    if frame is None:
        out(f"inspect failed: no frame from {node}")
        return None
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    source = target_dir / f"armcam_{datetime.now():%Y%m%d_%H%M%S}.png"
    if not cv2.imwrite(str(source), frame):
        out(f"inspect failed: could not save {source}")
        return None
    key = key_of()
    if not (key or "").strip():
        out("inspect failed: MODEL_API_KEY is missing")
        return None
    try:
        if use_retry:
            result = counter(
                client_of(key), frame, count_clips.LEVER_PROMPT,
                recordings_dir, source,
                expected_px=count_clips.EXPECTED_PART_PX,
                grab_frame=lambda: camera_of(node))
        else:
            result = counter(
                client_of(key), frame, count_clips.LEVER_PROMPT,
                recordings_dir, source,
                expected_px=count_clips.EXPECTED_PART_PX)
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        out(f"inspect failed: {exc}")
        return None
    if not result.get("ok"):
        out(f"inspect failed: {result.get('reason')}")
        return None
    attempts = result.get("attempts") or []
    if len(attempts) > 1:
        out(f"inspect retry: {len(attempts)} attempts")
    out(f"inspect: levers {result['levers']} verdict {result['verdict']}")
    out(str(result["full_image"]))
    out(str(result["crop_image"]))
    out(str(result["record"]))
    return result["verdict"]


def wait_and_inspect(
    stream,
    out=print,
    ready_timeout_s: float = 135.0,
    on_ready=None,
    **kwargs,
) -> str | None:
    """Wait for the loop lift hold, inspect, return the box or None.

    Polls in short steps until inspect_ready is set (the lift hold
    settled with the part held), the loop is done, or the loop
    reports inspect_skipped (missed grip, lost part, contact stop
    or another abort before the lift ended). On done or skip no
    frame is taken and None is returned, so the caller hands
    set_verdict(None) to the loop. A ready timeout also returns
    None. on_ready is called exactly once right after inspect_ready
    is set and before the inspection runs (never on skip, done or
    ready timeout); pick_test starts the station table shot there.
    """
    ready = getattr(stream, "inspect_ready", None)
    done = getattr(stream, "done", None)
    deadline = time.monotonic() + ready_timeout_s
    while True:
        if getattr(stream, "inspect_skipped", False):
            out("inspect skipped: no part held")
            return None
        if done is not None and done.is_set():
            out("inspect skipped: no part held")
            return None
        if ready is not None and ready.is_set():
            break
        if time.monotonic() >= deadline:
            out("inspect failed: hold never settled")
            return None
        time.sleep(0.1)
    if on_ready is not None:
        on_ready()
    return run_inspection(out, **kwargs)
