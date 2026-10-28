"""Arm waypoint plan for the first pick. Pure, moves nothing.

Builds the labelled waypoint list for muse_qc/pick_test.py: ramp
from the clamped rest reading to the 100 mm hover joints, lower the
tip straight down to the grip height, lift back up, put down again
2 mm higher, raise, then ramp back to the rest reading and settle.
The lower ladder solves the hover IK at every 1 mm of height, so
the tip moves straight down at LOWER_MM_S. Every waypoint carries
its segment label and whether the grasp z floor applies.
"""

from __future__ import annotations

import math

from muse_qc import safety
from muse_qc.first_move_plan import (
    LOOP_RATE_HZ,
    SETTLE_HOLD_CYCLES,
    SPEED_FRACTION,
    clamp_to_limits,
    ramp_waypoints,
    real_tip_m,
)
from muse_qc.target_pose import base_to_table_mm, solve_hover, table_to_base_mm
from muse_qc.xy_correction import correct_base_target_mm

HOVER_MM = 100.0
LOWER_MM_S = 10.0
PUT_DOWN_LIFT_MM = 2.0
CYCLES_PER_MM = int(LOOP_RATE_HZ / LOWER_MM_S)  # 10 cycles per 1 mm
SEG_HOVER = "hover"
SEG_LOWER = "lower"
SEG_LIFT = "lift"
SEG_PUTDOWN = "put-down"
SEG_RAISE = "raise"
SEG_RETURN = "return"
SEGMENTS = (SEG_HOVER, SEG_LOWER, SEG_LIFT, SEG_PUTDOWN, SEG_RAISE, SEG_RETURN)
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
                    use_xy_correction: bool = True) -> dict:
    """Build the full labelled pick plan. Pure, moves nothing.

    Returns {"ok": True, "plan", "ranges", ...} or {"ok": False,
    "reason"}. plan holds (segment, joints_deg, grasp_floor) triples
    at 100 Hz; ranges maps each segment to its [start, end) slice.
    With use_xy_correction the base target is shifted through
    xy_correction.correct_base_target_mm first, so the real tip
    lands on the part; False flies the uncorrected target.
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

    hover_pts = list(ramp_waypoints(rest, hover_joints))
    hover_pts[-1] = hover_joints
    _append(SEG_HOVER, hover_pts)
    lower_steps = _steps_for(lower_rungs)
    lower_pts: list[tuple[float, ...]] = []
    for upper, lower, steps in zip(lower_joints, lower_joints[1:], lower_steps):
        lower_pts += _interpolate(upper, lower, steps)
    lower_pts[-1] = lower_joints[-1]
    _append(SEG_LOWER, lower_pts)
    lift_pts: list[tuple[float, ...]] = []
    for upper, lower, steps in zip(lower_joints, lower_joints[1:], lower_steps):
        lift_pts = _interpolate(lower, upper, steps) + lift_pts
    lift_pts[-1] = hover_joints
    _append(SEG_LIFT, lift_pts)
    put_steps = _steps_for(put_rungs)
    put_pts: list[tuple[float, ...]] = []
    for upper, lower, steps in zip(put_joints, put_joints[1:], put_steps):
        put_pts += _interpolate(upper, lower, steps)
    put_pts[-1] = put_joints[-1]
    _append(SEG_PUTDOWN, put_pts)
    raise_pts: list[tuple[float, ...]] = []
    for upper, lower, steps in zip(put_joints, put_joints[1:], put_steps):
        raise_pts = _interpolate(lower, upper, steps) + raise_pts
    raise_pts[-1] = hover_joints
    _append(SEG_RAISE, raise_pts)
    return_pts = list(ramp_waypoints(hover_joints, rest))
    return_pts[-1] = rest
    _append(SEG_RETURN, return_pts + [rest] * SETTLE_HOLD_CYCLES)
    return {
        "ok": True,
        "plan": plan,
        "ranges": ranges,
        "hover_joints": hover_joints,
        "grip_joints": lower_joints[-1],
        "putdown_joints": put_joints[-1],
        "rest_joints": rest,
        "corrected_base_mm": (corr_x, corr_y),
        "solve_table_mm": solve_table,
        "use_xy_correction": bool(use_xy_correction),
        "hover_mm": hover,
        "grip_z_mm": grip_z,
        "hover_tip_mm": tuple(hover_solved.get("tip_table_mm", (0.0, 0.0, hover))),
        "grip_tip_mm": (0.0, 0.0, grip_z),
    }


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
    """Check every waypoint chained, grasp floor only where flagged. Pure."""
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
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=previous, reading_age_ms=READING_AGE_MS,
        )
        result = check(target, current, **extra)
        if not result.get("ok"):
            return False, index, result
        previous = tuple(waypoint)
    return True, -1, None


def plan_summary(plan: list[tuple[str, tuple[float, ...], bool]],
                 info: dict) -> list[str]:
    """Human-readable plan lines: segment counts, tip heights, target."""
    counts = {segment: sum(1 for item in plan if item[0] == segment)
              for segment in SEGMENTS}
    order = " ".join(f"{segment} {counts[segment]}" for segment in SEGMENTS)
    corr = info["corrected_base_mm"]
    hover_tip = info["hover_tip_mm"]
    state = "on" if info.get("use_xy_correction", True) else "off"
    return [
        f"segments at 100 Hz: {order} (total {len(plan)})",
        f"hover tip z: {hover_tip[2]:.1f} mm,"
        f" grip tip z: {info['grip_z_mm']:.1f} mm",
        f"corrected base target: x={corr[0]:+.2f} y={corr[1]:+.2f} mm",
        f"xy correction: {state}",
    ]
