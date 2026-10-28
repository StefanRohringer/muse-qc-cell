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
import threading
import time

from muse_qc import safety
from muse_qc.cycle_timing import summarize_ms
from muse_qc.first_move_plan import (
    LOOP_RATE_HZ,
    SETTLE_CYCLES,
    SPEED_FRACTION,
    build_plan,
    read_current_deg,
    read_sdk_vlims,
    real_tip_m,
    scale_vlim,
    validate_and_print,
)

CONFIRM_WORD = "yes"


class WaypointStream:
    """Control-loop callback: stream waypoints with a per-cycle check."""

    def __init__(self, group, waypoints, target_tips, vlim, initial_rad,
                 check_fn, tip_fn):
        self.group = group
        self.waypoints = waypoints
        self.target_tips = target_tips
        self.vlim = [float(v) for v in vlim]
        self.last_commanded_rad = [float(v) for v in initial_rad]
        self.check = safety.check_target if check_fn is None else check_fn
        self.tip_of = real_tip_m if tip_fn is None else tip_fn
        self.index = 0
        self.refusal: dict | None = None
        self.halt = False
        self.settle = 0
        self.done = threading.Event()
        self.work_ms: list[float] = []
        self.period_ms: list[float] = []
        self._prev_start: float | None = None

    def _send(self, joint_rad: list[float]) -> None:
        try:
            self.group.send_pos_vel([float(v) for v in joint_rad],
                                    vlim=self.vlim)
        except Exception:
            pass

    def _hold(self) -> None:
        # Hold the last commanded position, never a live reading: the
        # SDK reports 0.0 for missing feedback, which must never be sent.
        if self.settle <= 0:
            self.settle = SETTLE_CYCLES
        self._send(self.last_commanded_rad)
        self.settle -= 1
        if self.settle <= 0:
            self.done.set()

    def _refuse(self, reason: str, value: object, limit: object) -> None:
        self.refusal = {"ok": False, "reason": reason,
                        "value": value, "limit": limit}
        self._hold()

    def __call__(self, _sdk: object, _dt: float) -> None:
        start = time.perf_counter()
        try:
            if self._prev_start is not None:
                self.period_ms.append((start - self._prev_start) * 1000.0)
        except Exception:
            pass
        self._prev_start = start
        try:
            self._step()
        finally:
            try:
                self.work_ms.append((time.perf_counter() - start) * 1000.0)
            except Exception:
                pass

    def _step(self) -> None:
        read_start = time.monotonic()
        try:
            current_rad = [float(v) for v in self.group.get_positions()]
            if len(current_rad) != 6:
                raise ValueError(f"got {len(current_rad)} positions")
        except Exception as exc:
            self._refuse("read-failed", str(exc), "six joint angles")
            return
        age_ms = (time.monotonic() - read_start) * 1000.0
        current_deg = [math.degrees(v) for v in current_rad]
        if self.refusal is not None or self.halt or self.index >= len(self.waypoints):
            if self.refusal is None and self.halt and self.index < len(self.waypoints):
                self._refuse("user-halt", self.index, len(self.waypoints))
                return
            self._hold()
            return
        waypoint = self.waypoints[self.index]
        target_tip = self.target_tips[self.index]
        try:
            current_tip = self.tip_of(current_deg)
        except Exception as exc:
            self._refuse("read-failed", str(exc), "tool tip")
            return
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=tuple(current_deg), reading_age_ms=age_ms,
        )
        result = self.check(target, current)
        if not result.get("ok"):
            self.refusal = dict(result)
            self._hold()
            return
        commanded = [math.radians(v) for v in waypoint]
        self._send(commanded)
        self.last_commanded_rad = commanded
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done.set()


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
) -> int:
    """Validate the plan without enabling or sending anything."""
    sdk.connect()
    try:
        start_deg = read_current_deg(_group(sdk))
        out(f"rest joints deg: {[f'{v:+.2f}' for v in start_deg]}")
        old_ref = safety.J6_REF
        safety.set_j6_ref(start_deg[5])
        try:
            plan = build_plan(start_deg, joint_limits)
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
            plan = build_plan(start_deg, joint_limits)
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
            stream = WaypointStream(group, waypoints, target_tips, slow,
                                    first_rad, check_fn, tip_fn)
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
