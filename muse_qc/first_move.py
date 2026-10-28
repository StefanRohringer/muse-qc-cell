"""First arm motion for the reBot B601-DM arm. The human runs it by hand.

Default is a dry run: connect through the SDK, read the six joints,
validate every planned waypoint through safety.check_target and print
one line per waypoint, then disconnect. The dry run never enables the
motors and never sends anything.

With --execute the same validation runs first and the human confirms
with "yes". Only then the arm ramps to home, moves joint1 to +5 deg
and back, holds, and settles to the clamped rest reading, at 10 %
of the SDK speed caps with a fresh safety check in every control
cycle.
"""

from __future__ import annotations

import argparse
import json
import math
import sys

from muse_qc import safety
from muse_qc.cycle_timing import summarize_ms
from muse_qc.waypoint_stream import WaypointStream
from muse_qc.first_move_plan import (
    LOOP_RATE_HZ,
    build_plan,
    read_current_deg,
    read_sdk_vlims,
    real_tip_m,
    scale_vlim,
    validate_and_print,
)

CONFIRM_WORD = "yes"


def _group(sdk: object) -> object:
    """Return the arm joint group of a connected SDK object."""
    return sdk.groups["arm"]


def _create_sdk() -> object:
    """Create the real SDK object. Called only by main(), never by tests."""
    from reBotArm_control_py.actuator import RebotArm

    return RebotArm()


def dry_run(
    sdk: object,
    check_fn: object = None,
    tip_fn: object = None,
    joint_limits: object = None,
    out: object = print,
    plan_fn: object = None,
) -> int:
    """Validate the plan without enabling or sending anything."""
    sdk.connect()
    try:
        start_deg = read_current_deg(_group(sdk))
        out(f"rest joints deg: {[f'{v:+.2f}' for v in start_deg]}")
        old_ref = safety.J6_REF
        safety.set_j6_ref(start_deg[5])
        try:
            if plan_fn is None:
                plan = build_plan(start_deg, joint_limits)
            else:
                try:
                    plan = plan_fn(start_deg)
                except Exception as exc:
                    out(f"plan failed: {exc}")
                    return 1
            out(f"plan: {len(plan)} waypoints at {LOOP_RATE_HZ:.0f} Hz")
            ok, _ = validate_and_print(
                _group(sdk), plan,
                check_fn=check_fn, tip_fn=tip_fn, out=out,
            )
            out("dry run: all waypoints pass" if ok else "dry run: plan refused")
            return 0 if ok else 1
        finally:
            safety.set_j6_ref(old_ref)
    finally:
        sdk.disconnect()


def execute(
    sdk: object,
    input_fn: object = input,
    check_fn: object = None,
    tip_fn: object = None,
    vlims: object = None,
    joint_limits: object = None,
    out: object = print,
    plan_fn: object = None,
) -> int:
    """Validate the plan, ask for confirmation, then move at 10 % speed."""
    sdk.connect()
    try:
        group = _group(sdk)
        start_deg = read_current_deg(group)
        out(f"rest joints deg: {[f'{v:+.2f}' for v in start_deg]}")
        old_ref = safety.J6_REF
        safety.set_j6_ref(start_deg[5])
        try:
            if plan_fn is None:
                plan = build_plan(start_deg, joint_limits)
            else:
                try:
                    plan = plan_fn(start_deg)
                except Exception as exc:
                    out(f"plan failed: {exc}")
                    return 1
            out(f"plan: {len(plan)} waypoints at {LOOP_RATE_HZ:.0f} Hz")
            ok, target_tips = validate_and_print(
                group, plan,
                check_fn=check_fn, tip_fn=tip_fn, out=out,
            )
            if not ok:
                out("plan refused, not moving")
                return 1
            vlims = read_sdk_vlims() if vlims is None else list(vlims)
            if vlims is None:
                out("SDK speed caps unreadable, not moving")
                return 1
            slow = scale_vlim(vlims)
            out(f"SDK vlim: {[f'{v:.1f}' for v in vlims]} -> 10 %: "
                f"{[f'{v:.2f}' for v in slow]}")
            answer = input_fn("Type yes to move the arm: ").strip()
            if answer != CONFIRM_WORD:
                out("confirmation missing, nothing was enabled")
                return 2
            group.mode_pos_vel(vlim=slow)
            first_rad = [math.radians(v) for v in read_current_deg(group)]
            group.send_pos_vel(first_rad, vlim=slow)
            group.enable()
            waypoints = [point for _, point in plan]
            snapshot_index: int | None = None
            for pos, (segment, _point) in enumerate(plan):
                if segment == "hold":
                    snapshot_index = pos
            stream = WaypointStream(group, waypoints, target_tips, slow,
                                    first_rad, check_fn, tip_fn,
                                    snapshot_index=snapshot_index)
            sdk.start_control_loop(stream, rate=LOOP_RATE_HZ)
            timeout_s = len(waypoints) / LOOP_RATE_HZ + 120.0
            try:
                stream.done.wait(timeout=timeout_s)
            except KeyboardInterrupt:
                out("interrupted by human, holding position")
                stream.halt = True
            if not stream.done.is_set():
                stream.halt = True
                stream.done.wait(timeout=30.0)
            if stream.work_ms:
                out(f"cycle work: {summarize_ms(stream.work_ms)}"
                    f" over {len(stream.work_ms)} cycles")
            else:
                out("cycle work: no samples")
            target_ms = 1000.0 / LOOP_RATE_HZ
            if stream.period_ms:
                out(f"cycle period: {summarize_ms(stream.period_ms)}"
                    f" (target {target_ms:.2f} ms)")
            else:
                out(f"cycle period: no samples (target {target_ms:.2f} ms)")
            if stream.refusal is not None:
                out(f"REFUSED {json.dumps(stream.refusal)}")
                return 1
            if stream.snapshot_deg is not None and snapshot_index is not None:
                commanded = waypoints[snapshot_index]
                out("hold commanded deg: "
                    + " ".join(f"{v:+.2f}" for v in commanded))
                out("hold measured deg: "
                    + " ".join(f"{v:+.2f}" for v in stream.snapshot_deg))
                tip_of = real_tip_m if tip_fn is None else tip_fn
                tip_m = tip_of(list(stream.snapshot_deg))
                out(f"hold measured tip mm: x={tip_m[0] * 1000.0:+.1f} "
                    f"y={tip_m[1] * 1000.0:+.1f} "
                    f"z={tip_m[2] * 1000.0:+.1f}")
            out("motion finished, arm holds the settle pose")
            return 0
        finally:
            safety.set_j6_ref(old_ref)
    finally:
        out("stopping: the motors will be disabled now")
        try:
            sdk.stop_control_loop()
        except Exception:
            pass
        sdk.disconnect()


def main(argv: object = None, sdk_factory: object = None, **kwargs: object) -> int:
    """Entry point for the human: dry run by default, --execute moves."""
    parser = argparse.ArgumentParser(description="First arm motion (human-run only).")
    parser.add_argument("--execute", action="store_true",
                        help="move the arm after confirmation")
    args = parser.parse_args(argv)
    sdk = sdk_factory() if sdk_factory is not None else _create_sdk()
    if args.execute:
        return execute(sdk, **kwargs)
    return dry_run(sdk, **kwargs)


if __name__ == "__main__":
    sys.exit(main())
