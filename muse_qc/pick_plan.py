"""Arm waypoint plan for the first pick. Pure, moves nothing.

Builds the labelled waypoint list for muse_qc/pick_test.py: ramp
from the clamped rest reading to the 100 mm hover joints, lower the
tip straight down to the grip height, lift back up, put down again
2 mm higher, raise, then ramp back to the rest reading and settle.
The free ramps fly at FREE_SPEED_DEG_S with an ACCEL_DEG_S2
trapezoid plus a short hover settle; the ladder solves the hover
IK at every 1 mm of height and descends fast (FAST_MM_S) above the
slow zone, slow (LOWER_MM_S) near the part. Every waypoint carries
its segment label and whether the grasp z floor applies.
"""

from __future__ import annotations

import math

from muse_qc import safety
from muse_qc.drop_plan import SEG_RELEASE, SEG_TOBOX, build_drop_segments
from muse_qc.first_move_plan import (
    LOOP_RATE_HZ,
    SETTLE_HOLD_CYCLES,
    SIM_ZERO_DEG,
    SPEED_FRACTION,
    clamp_to_limits,
    ramp_waypoints,
    real_tip_m,
)
from muse_qc.target_pose import base_to_table_mm, solve_hover, table_to_base_mm
from muse_qc.xy_correction import correct_base_target_mm

HOVER_MM = 100.0
LOWER_MM_S = 20.0
FAST_MM_S = 40.0
SLOW_ZONE_MM = 25.0
LIFT_SLOW_MM = 10.0
FREE_SPEED_DEG_S = 15.0
ACCEL_DEG_S2 = 30.0
SETTLE_AT_HOVER_S = 0.5
SETTLE_AT_HOVER_CYCLES = int(SETTLE_AT_HOVER_S * LOOP_RATE_HZ)
FREE_STEP_DEG = FREE_SPEED_DEG_S / LOOP_RATE_HZ  # 0.15 deg per command
CYCLES_PER_MM = int(LOOP_RATE_HZ / LOWER_MM_S)  # 10 cycles per 1 mm
FAST_CYCLES_PER_MM = LOOP_RATE_HZ / FAST_MM_S  # 2.5 cycles per 1 mm
PUT_DOWN_LIFT_MM = 2.0
SEG_HOVER = "hover"
SEG_LOWER = "lower"
SEG_LIFT = "lift"
SEG_PUTDOWN = "put-down"
SEG_RAISE = "raise"
SEG_RETURN = "return"
SEG_TOSTATION = "to-station"
SEGMENTS = (SEG_HOVER, SEG_LOWER, SEG_LIFT, SEG_PUTDOWN, SEG_RAISE, SEG_RETURN)
SEGMENTS_DROP = (SEG_HOVER, SEG_LOWER, SEG_LIFT, SEG_TOBOX, SEG_RELEASE, SEG_RETURN)
SEGMENTS_ALL = (SEG_HOVER, SEG_LOWER, SEG_LIFT, SEG_TOSTATION, SEG_PUTDOWN, SEG_RAISE,
                SEG_TOBOX, SEG_RELEASE, SEG_RETURN)
GRASP_SEGMENTS = frozenset({SEG_LOWER, SEG_LIFT, SEG_PUTDOWN, SEG_RAISE})
READING_AGE_MS = 1000.0 / LOOP_RATE_HZ


def _heights(top_mm: float, bottom_mm: float) -> list[float]:
    """1 mm height rungs from top down to bottom, bottom always last."""
    rungs = [float(top_mm)]
    while rungs[-1] - 1.0 > float(bottom_mm):
        rungs.append(rungs[-1] - 1.0)
    if rungs[-1] != float(bottom_mm):
        rungs.append(float(bottom_mm))
    return rungs


def _steps_for(rungs: list[float]) -> list[int]:
    """Cycle count per rung step so each 1 mm takes CYCLES_PER_MM."""
    return [max(1, int(round(abs(b - a) * CYCLES_PER_MM)))
            for a, b in zip(rungs, rungs[1:])]


def _rung_steps(upper: tuple[float, ...], lower: tuple[float, ...],
                mm_dist: float, fast: bool) -> int:
    """Cycle count for one ladder rung step at the rung speed.

    Fast rungs take FAST_CYCLES_PER_MM per mm, slow rungs CYCLES_PER_MM
    per mm; either way no joint moves more than FREE_STEP_DEG per
    cycle, adding cycles when a rung needs more.
    """
    base_mm = FAST_CYCLES_PER_MM if fast else CYCLES_PER_MM
    steps = max(1, int(round(abs(mm_dist) * base_mm)))
    peak = max(abs(e - s) for s, e in zip(upper, lower))
    need = int(math.ceil(peak / FREE_STEP_DEG)) if peak > 0.0 else 1
    return max(steps, need)


def _interpolate(start: tuple[float, ...], end: tuple[float, ...],
                 steps: int) -> list[tuple[float, ...]]:
    """Linear joint interpolation, excluding start, including end."""
    return [tuple(s + (e - s) * i / steps for s, e in zip(start, end))
            for i in range(1, steps + 1)]


def _solve_ladder(solve_table: tuple[float, float], rungs: list[float],
                  seed: object, solve: object) -> dict:
    """Solve every rung height, each seeded with the previous solution."""
    joints: list[tuple[float, ...]] = []
    previous = seed
    for height in rungs:
        solved = solve(solve_table, height, previous)
        if not solved.get("ok"):
            return {"ok": False,
                    "reason": f"ladder unreachable at {height:.1f} mm: "
                              f"{solved.get('reason', 'unreachable')}"}
        previous = tuple(float(v) for v in solved["joints_deg"])
        joints.append(previous)
    return {"ok": True, "joints": joints}


def build_pick_plan(start_deg: object, table_xy: object, grip_z_mm: float,
                    joint_limits: object = None, solve_fn: object = None,
                    hover_mm: float = HOVER_MM,
                    use_xy_correction: bool = True,
                    free_speed_deg_s: float = FREE_SPEED_DEG_S,
                    accel_deg_s2: float = ACCEL_DEG_S2,
                    drop_joints: object = None,
                    tip_fn: object = None) -> dict:
    """Build the full labelled pick plan. Pure, moves nothing.

    Returns {"ok": True, "plan", "ranges", ...} or {"ok": False,
    "reason"}. plan holds (segment, joints_deg, grasp_floor) triples
    at 100 Hz; ranges maps each segment to its [start, end) slice.
    With use_xy_correction the base target is shifted through
    xy_correction.correct_base_target_mm first, so the real tip
    lands on the part; False flies the uncorrected target. With
    drop_joints the plan is hover, lower, lift, to-box, release,
    return (no put-down, no raise); the lift still ends at the
    hover height.
    """
    try:
        start = tuple(float(v) for v in start_deg)
        table = (float(table_xy[0]), float(table_xy[1]))
        grip_z = float(grip_z_mm)
        hover = float(hover_mm)
    except (TypeError, ValueError, IndexError):
        return {"ok": False, "reason": "bad pick target"}
    if len(start) != 6 or not all(math.isfinite(v) for v in start):
        return {"ok": False, "reason": "bad pick target"}
    if not all(math.isfinite(v) for v in table + (grip_z, hover)):
        return {"ok": False, "reason": "bad pick target"}
    if not 0.0 < grip_z < hover:
        return {"ok": False, "reason": f"bad grip z {grip_z:.1f} mm"}
    if grip_z + PUT_DOWN_LIFT_MM > hover:
        return {"ok": False, "reason": f"bad grip z {grip_z:.1f} mm"}
    limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
    if limits is None:
        return {"ok": False, "reason": "joint limits unavailable"}
    solve = solve_hover if solve_fn is None else solve_fn
    rest = clamp_to_limits(start, limits)
    base_x, base_y = table_to_base_mm(table[0], table[1])
    if use_xy_correction:
        corr_x, corr_y = correct_base_target_mm(base_x, base_y)
    else:
        corr_x, corr_y = base_x, base_y
    solve_table = base_to_table_mm(corr_x, corr_y)
    hover_solved = solve(solve_table, hover, list(start))
    if not hover_solved.get("ok"):
        return {"ok": False, "reason": str(hover_solved.get("reason"))}
    hover_joints = tuple(float(v) for v in hover_solved["joints_deg"])
    lower_rungs = _heights(hover, grip_z)
    lowered = _solve_ladder(solve_table, lower_rungs[1:], list(hover_joints), solve)
    if not lowered.get("ok"):
        return {"ok": False, "reason": str(lowered.get("reason"))}
    lower_joints = [hover_joints] + list(lowered["joints"])
    put_rungs = _heights(hover, grip_z + PUT_DOWN_LIFT_MM)
    put = _solve_ladder(solve_table, put_rungs[1:], list(hover_joints), solve)
    if not put.get("ok"):
        return {"ok": False, "reason": str(put.get("reason"))}
    put_joints = [hover_joints] + list(put["joints"])
    plan: list[tuple[str, tuple[float, ...], bool]] = []
    ranges: dict[str, tuple[int, int]] = {}

    def _append(segment: str, points: list[tuple[float, ...]]) -> None:
        start_index = len(plan)
        plan.extend((segment, point, segment in GRASP_SEGMENTS) for point in points)
        ranges[segment] = (start_index, len(plan))

    hover_pts = list(ramp_waypoints(rest, hover_joints,
                                      speed_deg_s=free_speed_deg_s,
                                      accel_deg_s2=accel_deg_s2))
    hover_pts[-1] = hover_joints
    _append(SEG_HOVER, hover_pts + [hover_joints] * SETTLE_AT_HOVER_CYCLES)
    lower_heights = list(zip(lower_rungs, lower_rungs[1:]))
    lower_pts: list[tuple[float, ...]] = []
    for upper, lower, (top, bottom) in zip(lower_joints, lower_joints[1:],
                                          lower_heights):
        fast = bottom >= grip_z + SLOW_ZONE_MM
        steps = _rung_steps(upper, lower, top - bottom, fast)
        lower_pts += _interpolate(upper, lower, steps)
    lower_pts[-1] = lower_joints[-1]
    _append(SEG_LOWER, lower_pts)
    lift_pts: list[tuple[float, ...]] = []
    for upper, lower, (top, bottom) in zip(lower_joints, lower_joints[1:],
                                          lower_heights):
        fast = bottom >= grip_z + LIFT_SLOW_MM
        steps = _rung_steps(lower, upper, top - bottom, fast)
        lift_pts = _interpolate(lower, upper, steps) + lift_pts
    lift_pts[-1] = hover_joints
    _append(SEG_LIFT, lift_pts)
    drop: tuple[float, ...] | None = None
    if drop_joints is None:
        put_z = grip_z + PUT_DOWN_LIFT_MM
        put_heights = list(zip(put_rungs, put_rungs[1:]))
        put_pts: list[tuple[float, ...]] = []
        for upper, lower, (top, bottom) in zip(put_joints, put_joints[1:],
                                              put_heights):
            fast = bottom >= put_z + SLOW_ZONE_MM
            steps = _rung_steps(upper, lower, top - bottom, fast)
            put_pts += _interpolate(upper, lower, steps)
        put_pts[-1] = put_joints[-1]
        _append(SEG_PUTDOWN, put_pts)
        raise_pts: list[tuple[float, ...]] = []
        for upper, lower, (top, bottom) in zip(put_joints, put_joints[1:],
                                              put_heights):
            steps = _rung_steps(lower, upper, top - bottom, True)
            raise_pts = _interpolate(lower, upper, steps) + raise_pts
        raise_pts[-1] = hover_joints
        _append(SEG_RAISE, raise_pts)
        return_pts = list(ramp_waypoints(hover_joints, rest,
                                         speed_deg_s=free_speed_deg_s,
                                         accel_deg_s2=accel_deg_s2))
        return_pts[-1] = rest
        _append(SEG_RETURN, return_pts + [rest] * SETTLE_HOLD_CYCLES)
    else:
        try:
            drop = tuple(float(v) for v in drop_joints)
        except (TypeError, ValueError):
            return {"ok": False, "reason": "bad drop joints"}
        if len(drop) != 6 or not all(math.isfinite(v) for v in drop):
            return {"ok": False, "reason": "bad drop joints"}
        segs = build_drop_segments(hover_joints, drop, rest,
                                   free_speed_deg_s=free_speed_deg_s,
                                   accel_deg_s2=accel_deg_s2,
                                   tip_fn=tip_fn)
        if not segs.get("ok"):
            return {"ok": False, "reason": str(segs.get("reason"))}
        offset = len(plan)
        plan.extend(segs["plan"])
        for segment, (low, high) in segs["ranges"].items():
            ranges[segment] = (low + offset, high + offset)
    return {
        "ok": True,
        "plan": plan,
        "ranges": ranges,
        "hover_joints": hover_joints,
        "grip_joints": lower_joints[-1],
        "putdown_joints": put_joints[-1],
        "drop_joints": drop,
        "rest_joints": rest,
        "corrected_base_mm": (corr_x, corr_y),
        "solve_table_mm": solve_table,
        "use_xy_correction": bool(use_xy_correction),
        "hover_mm": hover,
        "grip_z_mm": grip_z,
        "free_speed_deg_s": free_speed_deg_s,
        "accel_deg_s2": accel_deg_s2,
        "hover_tip_mm": tuple(hover_solved.get("tip_table_mm", (0.0, 0.0, hover))),
        "grip_tip_mm": (0.0, 0.0, grip_z),
    }


def build_inspect_plan(start_deg: object, table_xy: object, grip_z_mm: float,
                       joint_limits: object = None, solve_fn: object = None,
                       hover_mm: float = HOVER_MM,
                       use_xy_correction: bool = True,
                       free_speed_deg_s: float = FREE_SPEED_DEG_S,
                       accel_deg_s2: float = ACCEL_DEG_S2,
                       drop_joints_by_box: object = None,
                       tip_fn: object = None) -> dict:
    """Build one plan with both drop branches. Pure, moves nothing.

    Shared hover, lower and lift exactly as build_pick_plan, then
    the to-box/release/return waypoints of the good branch followed
    by the bad branch. Returns the build_pick_plan dict (ranges
    holds the good branch, the default) plus ranges_by_box, the
    shared hover/lower/lift ranges and drop_joints_by_box.
    check_pick_plan over the whole plan checks every waypoint of
    both branches before anything moves.
    """
    try:
        by_box = {box: tuple(float(v) for v in drop_joints_by_box[box])
                  for box in ("good", "bad")}
    except (TypeError, ValueError, KeyError, IndexError):
        return {"ok": False, "reason": "bad drop joints"}
    first = build_pick_plan(start_deg, table_xy, grip_z_mm, joint_limits,
                            solve_fn, hover_mm, use_xy_correction,
                            free_speed_deg_s, accel_deg_s2,
                            by_box["good"], tip_fn)
    if not first.get("ok"):
        return first
    second = build_drop_segments(first["hover_joints"], by_box["bad"],
                                 first["rest_joints"], free_speed_deg_s,
                                 accel_deg_s2, tip_fn)
    if not second.get("ok"):
        return {"ok": False, "reason": str(second.get("reason"))}
    offset = len(first["plan"])
    first["plan"] = list(first["plan"]) + list(second["plan"])
    first["ranges_by_box"] = {
        "good": {seg: first["ranges"][seg]
                 for seg in (SEG_TOBOX, SEG_RELEASE, SEG_RETURN)},
        "bad": {seg: (low + offset, high + offset)
                for seg, (low, high) in second["ranges"].items()},
    }
    first["shared_ranges"] = {seg: first["ranges"][seg]
                              for seg in (SEG_HOVER, SEG_LOWER, SEG_LIFT)}
    first["drop_joints_by_box"] = by_box
    return first


def abort_detour(plan: list[tuple[str, tuple[float, ...], bool]],
                   tips: list[tuple[float, float, float]],
                   ranges: dict[str, tuple[int, int]], hover_z: float,
                   pos: int | None) -> list[int]:
    """Abort path: hold, climb the lift ladder, then return. Pure.

    From the last commanded waypoint index pos (None before the first
    send): when below the hover height, walk the lift segment up from
    the first rung at or above the current tip, then the return
    segment; when at or above it, walk the rest of the return segment.
    Every index is a planned waypoint. The gripper is untouched by
    this path, so it never opens above the put-down height.
    """
    low, high = ranges[SEG_RETURN]
    if pos is None:
        return list(range(low, high))
    cur_z = tips[pos][2]
    if cur_z < hover_z - 1e-9:
        up, end = ranges[SEG_LIFT]
        rung = next((i for i in range(up, end) if tips[i][2] >= cur_z - 1e-9), end - 1)
        return [pos] + list(range(rung, end)) + list(range(low, high))
    if low <= pos < high:
        return list(range(pos, high))
    return list(range(low, high))


def check_pick_plan(plan: list[tuple[str, tuple[float, ...], bool]],
                    start_deg: object, check_fn: object = None,
                    tip_fn: object = None) -> tuple[bool, int, dict | None]:
    """Check every waypoint chained, grasp floor only where flagged. Pure.

    Exact 0.0 in a chained waypoint is a command (the clamped rest),
    not missing feedback, so it is nudged to SIM_ZERO_DEG for the
    current-state validity rule that rejects 0.0 live reads; commanded
    waypoints are never changed.
    """
    check = safety.check_target if check_fn is None else check_fn
    tip_of = real_tip_m if tip_fn is None else tip_fn
    previous = tuple(float(v) for v in start_deg)
    for index, (_, waypoint, grasp) in enumerate(plan):
        target_tip = tip_of(list(waypoint))
        extra = {"z_min_m": safety.GRASP_Z_MIN_M} if grasp else {}
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current_tip = tip_of(list(previous))
        current_joints = tuple(
            SIM_ZERO_DEG if value == 0.0 else value for value in previous
        )
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=current_joints, reading_age_ms=READING_AGE_MS,
        )
        result = check(target, current, **extra)
        if not result.get("ok"):
            return False, index, result
        previous = tuple(waypoint)
    return True, -1, None


def plan_summary(plan: list[tuple[str, tuple[float, ...], bool]],
                 info: dict) -> list[str]:
    """Human-readable plan lines: segment counts, tip heights, target."""
    names = [segment for segment in SEGMENTS_ALL
             if segment in info.get("ranges", {})] or list(SEGMENTS)
    counts = {segment: sum(1 for item in plan if item[0] == segment)
              for segment in names}
    order = " ".join(f"{segment} {counts[segment]}" for segment in names)
    corr = info["corrected_base_mm"]
    hover_tip = info["hover_tip_mm"]
    state = "on" if info.get("use_xy_correction", True) else "off"
    return [
        f"segments at 100 Hz: {order} (total {len(plan)})",
        f"speeds: free {info.get('free_speed_deg_s', FREE_SPEED_DEG_S):g} deg/s,"
        f" accel {info.get('accel_deg_s2', ACCEL_DEG_S2):g} deg/s2,"
        f" vertical {FAST_MM_S:g}/{LOWER_MM_S:g} mm/s",
        f"hover tip z: {hover_tip[2]:.1f} mm,"
        f" grip tip z: {info['grip_z_mm']:.1f} mm",
        f"corrected base target: x={corr[0]:+.2f} y={corr[1]:+.2f} mm",
        f"xy correction: {state}",
    ]
