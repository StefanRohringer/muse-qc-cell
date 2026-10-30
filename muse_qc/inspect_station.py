"""Fixed inspection station left of the table. Pure, moves nothing.

After the lift the arm carries the held part to one fixed table
point, turns joint 6 to one fixed angle and holds there while the
main thread inspects (same light and background for every part);
both drop branches then start from the station joints. The
station tip height equals the hover height (HOVER_MM from
muse_qc.pick_plan).

 solve_station_joints: same IK/solve_fn and xy correction as the
 hover (gripper pointing straight down), then joint 6 set to
 STATION_J6_DEG; refuses with "station unreachable" or
 "joint 6 limit".
 build_to_station_segments: trapezoid ramp from the top of the
 lift to the station joints (same free speed/accel as the other
 ramps); every waypoint keeps the corrected tip at or above
 TRANSIT_FLOOR_MM.
 build_inspect_plan_with_station: build_inspect_plan with the
 shared part hover, lower, lift, to-station and both branches
 built from the station joints. With use_station off the result
 is exactly build_inspect_plan (byte-for-byte the same plan).
"""

from __future__ import annotations

import math

from muse_qc import safety
from muse_qc.drop_plan import TRANSIT_FLOOR_MM, build_drop_segments
from muse_qc.first_move_plan import ramp_waypoints, real_tip_m
from muse_qc.pick_plan import (
    HOVER_MM,
    SEG_TOSTATION,
    build_inspect_plan,
)
from muse_qc.target_pose import base_to_table_mm, solve_hover, table_to_base_mm
from muse_qc.xy_correction import correct_base_target_mm

# Human chosen 2026-10-30, about 290 mm from the joint 1 axis.
STATION_TABLE_MM = (235.0, 150.0)
STATION_TIP_Z_MM = HOVER_MM
STATION_J6_DEG = 0.0
USE_STATION_DEFAULT = False


def solve_station_joints(start_deg: object, joint_limits: object = None,
                         solve_fn: object = None,
                         use_xy_correction: bool = True) -> dict:
    """Station joints for the fixed inspection point. Pure, moves nothing.

    Same solve path as the hover: xy correction (unless off),
    solve_fn at STATION_TIP_Z_MM seeded with the start joints,
    gripper pointing straight down. Joint 6 is then set to the
    absolute STATION_J6_DEG; the human tunes that angle later.
    Refuses with "station unreachable" (bad target, no limits,
    IK failure) or "joint 6 limit" (STATION_J6_DEG outside the
    joint 6 limits).
    """
    try:
        start = tuple(float(v) for v in start_deg)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "station unreachable: bad start joints"}
    if len(start) != 6 or not all(math.isfinite(v) for v in start):
        return {"ok": False, "reason": "station unreachable: bad start joints"}
    limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
    if limits is None:
        return {"ok": False, "reason": "station unreachable: joint limits unavailable"}
    try:
        low, high = limits["joint6"]
    except (KeyError, TypeError):
        return {"ok": False, "reason": "station unreachable: joint limits unavailable"}
    if not (float(low) <= STATION_J6_DEG <= float(high)):
        return {"ok": False,
                "reason": f"joint 6 limit: station joint 6 {STATION_J6_DEG:.1f} deg"
                          f" outside [{float(low):.1f}, {float(high):.1f}]"}
    solve = solve_hover if solve_fn is None else solve_fn
    base_x, base_y = table_to_base_mm(STATION_TABLE_MM[0], STATION_TABLE_MM[1])
    if use_xy_correction:
        corr_x, corr_y = correct_base_target_mm(base_x, base_y)
    else:
        corr_x, corr_y = base_x, base_y
    solve_table = base_to_table_mm(corr_x, corr_y)
    solved = solve(solve_table, STATION_TIP_Z_MM, list(start))
    if not solved.get("ok"):
        return {"ok": False,
                "reason": f"station unreachable: {solved.get('reason', 'unreachable')}"}
    joints = tuple(float(v) for v in solved["joints_deg"])
    if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
        return {"ok": False, "reason": "station unreachable: bad IK joints"}
    joints = joints[:5] + (float(STATION_J6_DEG),)
    return {
        "ok": True,
        "joints_deg": joints,
        "corrected_base_mm": (corr_x, corr_y),
        "solve_table_mm": solve_table,
    }


def build_to_station_segments(lift_top_deg: object, station_deg: object,
                              free_speed_deg_s: float = 15.0,
                              accel_deg_s2: float = 30.0,
                              tip_fn: object = None) -> dict:
    """Ramp from the top of the lift to the station joints. Pure.

    Trapezoid ramp with the same free speed/accel as the other
    free ramps; every waypoint must keep the corrected tip at or
    above TRANSIT_FLOOR_MM, else the build fails with
    "transit too low" (no detours are tried).
    """
    tip_of = real_tip_m if tip_fn is None else tip_fn
    try:
        top = tuple(float(v) for v in lift_top_deg)
        station = tuple(float(v) for v in station_deg)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "bad station joints"}
    if len(top) != 6 or len(station) != 6:
        return {"ok": False, "reason": "bad station joints"}
    if not all(math.isfinite(v) for v in top + station):
        return {"ok": False, "reason": "bad station joints"}
    floor_m = TRANSIT_FLOOR_MM / 1000.0
    lowest_mm = math.inf
    try:
        carry = list(ramp_waypoints(top, station,
                                    speed_deg_s=free_speed_deg_s,
                                    accel_deg_s2=accel_deg_s2))
    except (ValueError, TypeError, AttributeError) as exc:
        return {"ok": False, "reason": f"bad station joints ({exc})"}
    for index, point in enumerate(carry):
        try:
            tip_z = tip_of(list(point))[2]
        except (ValueError, TypeError, AttributeError) as exc:
            return {"ok": False, "reason": f"bad station joints ({exc})"}
        lowest_mm = min(lowest_mm, tip_z * 1000.0)
        if tip_z < floor_m:
            return {"ok": False,
                    "reason": f"transit too low: tip z {tip_z * 1000.0:.1f} mm"
                              f" below {TRANSIT_FLOOR_MM:.0f} mm at to-station"
                              f" waypoint {index + 1}"}
    plan = [(SEG_TOSTATION, point, False) for point in carry]
    return {
        "ok": True,
        "plan": plan,
        "ranges": {SEG_TOSTATION: (0, len(plan))},
        "station_joints": station,
        "lowest_tip_z_mm": lowest_mm,
    }


def build_inspect_plan_with_station(
        start_deg: object, table_xy: object, grip_z_mm: float,
        joint_limits: object = None, solve_fn: object = None,
        hover_mm: float = HOVER_MM, use_xy_correction: bool = True,
        free_speed_deg_s: float = 15.0, accel_deg_s2: float = 30.0,
        drop_joints_by_box: object = None, tip_fn: object = None,
        use_station: bool = USE_STATION_DEFAULT) -> dict:
    """One inspect plan with both drop branches, station optional. Pure.

    With use_station off this is exactly build_inspect_plan.
    With use_station on the shared part is hover, lower, lift,
    to-station and both drop branches are built with
    build_drop_segments starting from the station joints instead
    of the lift top. A refused station ("station unreachable",
    "joint 6 limit") or a low to-station ramp fails the whole
    plan before anything moves.
    """
    if not use_station:
        return build_inspect_plan(
            start_deg, table_xy, grip_z_mm, joint_limits, solve_fn,
            hover_mm, use_xy_correction, free_speed_deg_s, accel_deg_s2,
            drop_joints_by_box, tip_fn)
    try:
        by_box = {box: tuple(float(v) for v in drop_joints_by_box[box])
                  for box in ("good", "bad")}
    except (TypeError, ValueError, KeyError, IndexError):
        return {"ok": False, "reason": "bad drop joints"}
    base = build_inspect_plan(
        start_deg, table_xy, grip_z_mm, joint_limits, solve_fn,
        hover_mm, use_xy_correction, free_speed_deg_s, accel_deg_s2,
        by_box, tip_fn)
    if not base.get("ok"):
        return base
    station = solve_station_joints(start_deg, joint_limits, solve_fn,
                                   use_xy_correction)
    if not station.get("ok"):
        return {"ok": False, "reason": str(station.get("reason"))}
    station_joints = station["joints_deg"]
    to_station = build_to_station_segments(
        base["hover_joints"], station_joints, free_speed_deg_s,
        accel_deg_s2, tip_fn)
    if not to_station.get("ok"):
        return {"ok": False, "reason": str(to_station.get("reason"))}
    tip_of = real_tip_m if tip_fn is None else tip_fn
    branches = {}
    for box in ("good", "bad"):
        segs = build_drop_segments(station_joints, by_box[box],
                                    base["rest_joints"], free_speed_deg_s,
                                    accel_deg_s2, tip_of)
        if not segs.get("ok"):
            return {"ok": False, "reason": str(segs.get("reason"))}
        branches[box] = segs
    lift_end = base["shared_ranges"]["lift"][1]
    plan = list(base["plan"][:lift_end]) + list(to_station["plan"])
    offset = len(plan)
    ranges_by_box = {}
    for box in ("good", "bad"):
        segs = branches[box]
        ranges_by_box[box] = {
            segment: (low + offset, high + offset)
            for segment, (low, high) in segs["ranges"].items()
        }
        plan = plan + list(segs["plan"])
        offset = len(plan)
    shared_ranges = dict(base["shared_ranges"])
    seg_low, seg_high = to_station["ranges"][SEG_TOSTATION]
    shared_ranges[SEG_TOSTATION] = (lift_end + seg_low, lift_end + seg_high)
    base["plan"] = plan
    base["ranges"] = {**shared_ranges, **ranges_by_box["good"]}
    base["ranges_by_box"] = ranges_by_box
    base["shared_ranges"] = shared_ranges
    base["station_joints"] = station_joints
    base["use_station"] = True
    return base
