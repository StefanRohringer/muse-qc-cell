"""Box-exit transit for the arm session. Pure, moves nothing.

build_box_transit ramps from the box exit joints (where the arm
holds the released part pose) to the next part hover joints with
ramp_waypoints at the same free speed and accel as the to-box
segment. Every waypoint keeps the corrected tip at or above
TRANSIT_FLOOR_MM (the same floor the to-box ramp keeps); below
it the build fails with "transit too low" and the session
returns to rest (no detours are tried). splice_hover swaps the
hover prefix of a _plan_and_check plan with the transit, so the
chained check and the loop see one plan starting at the box.
"""

from __future__ import annotations

import math

from muse_qc.arm import safety
from muse_qc.bringup.first_move_plan import (
    SETTLE_HOLD_CYCLES,
    clamp_to_limits,
    ramp_waypoints,
    real_tip_m,
)
from muse_qc.pick.drop_plan import TRANSIT_FLOOR_MM
from muse_qc.pick.pick_plan import SEG_HOVER, SEG_RETURN, SETTLE_AT_HOVER_CYCLES


def build_box_transit(box_exit_deg: object, hover_joints_deg: object,
                      free_speed_deg_s: float = 15.0,
                      accel_deg_s2: float = 30.0,
                      tip_fn: object = None) -> dict:
    """Ramp from the box exit joints to the next hover joints. Pure.

    Returns {"ok": True, "plan", "ranges", "lowest_tip_z_mm"} with
    every waypoint labelled SEG_HOVER and the grasp floor off, or
    {"ok": False, "reason"}. The plan ends with the same hover
    settle the part plan keeps, so the arm settles at hover
    before the next lower starts.
    """
    tip_of = real_tip_m if tip_fn is None else tip_fn
    try:
        box_exit = tuple(float(v) for v in box_exit_deg)
        hover = tuple(float(v) for v in hover_joints_deg)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "bad transit joints"}
    if len(box_exit) != 6 or len(hover) != 6:
        return {"ok": False, "reason": "bad transit joints"}
    if not all(math.isfinite(v) for v in box_exit + hover):
        return {"ok": False, "reason": "bad transit joints"}
    floor_m = TRANSIT_FLOOR_MM / 1000.0
    try:
        carry = list(ramp_waypoints(
            box_exit, hover, speed_deg_s=free_speed_deg_s,
            accel_deg_s2=accel_deg_s2))
    except (ValueError, TypeError, AttributeError) as exc:
        return {"ok": False, "reason": f"bad transit joints ({exc})"}
    points = carry + [hover] * SETTLE_AT_HOVER_CYCLES
    lowest_mm = math.inf
    for index, point in enumerate(points):
        try:
            tip_z = tip_of(list(point))[2]
        except (ValueError, TypeError, AttributeError) as exc:
            return {"ok": False, "reason": f"bad transit joints ({exc})"}
        lowest_mm = min(lowest_mm, tip_z * 1000.0)
        if tip_z < floor_m:
            return {"ok": False,
                    "reason": f"transit too low: tip z "
                              f"{tip_z * 1000.0:.1f} mm below "
                              f"{TRANSIT_FLOOR_MM:.0f} mm at transit "
                              f"waypoint {index + 1}"}
    plan = [(SEG_HOVER, point, False) for point in points]
    return {
        "ok": True,
        "plan": plan,
        "ranges": {SEG_HOVER: (0, len(plan))},
        "lowest_tip_z_mm": lowest_mm,
    }


def _shifted(ranges: dict, delta: int) -> dict:
    """Shift every range in ranges by delta waypoints."""
    return {seg: (low + delta, high + delta)
            for seg, (low, high) in ranges.items()}


def retarget_return(built: dict, rest_joints: object,
                    free_speed_deg_s: float = 15.0,
                    accel_deg_s2: float = 30.0,
                    joint_limits: object = None) -> dict:
    """Point every return segment at the session rest pose. Pure.

    A part plan built with start = the box exit would return to
    the box (rest = clamp(start) inside the builders); the
    session must return to the measured rest once at its end.
    The rest pose is clamped into the limits with the builders'
    rule first: a measured rest just outside a limit (joint 2
    reads above 0.0) would otherwise rebuild a return that the
    chained check must refuse. Rebuilds each branch return as
    ramp_waypoints from the release joints to rest plus the rest
    settle, keeping the to-box and release waypoints bit for
    bit, and re-indexes ranges, shared_ranges and ranges_by_box.
    Returns a copy; the input dict is unchanged.
    """
    limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
    if limits is None:
        return {"ok": False, "reason": "joint limits unavailable"}
    try:
        rest = tuple(clamp_to_limits(rest_joints, limits))
    except (TypeError, ValueError, KeyError) as exc:
        return {"ok": False, "reason": f"bad rest joints ({exc})"}
    plan = list(built["plan"])
    by_box = built.get("ranges_by_box")
    if not by_box:
        branches = [("return", built["ranges"]["return"],
                     built["ranges"]["release"])]
    else:
        branches = [(box, by_box[box]["return"], by_box[box]["release"])
                    for box in ("good", "bad")
                    if box in by_box and "return" in by_box[box]]
    rebuilt = dict(built)
    new_plan = list(plan)
    new_by_box = {box: dict(branch)
                  for box, branch in (by_box or {}).items()}
    growth = 0
    for box, (low, high), (rel_low, rel_high) in branches:
        low, high = low + growth, high + growth
        rel_low, rel_high = rel_low + growth, rel_high + growth
        drop = tuple(new_plan[rel_high - 1][1])
        try:
            back = list(ramp_waypoints(drop, rest,
                                       speed_deg_s=free_speed_deg_s,
                                       accel_deg_s2=accel_deg_s2))
        except (ValueError, TypeError, AttributeError) as exc:
            return {"ok": False, "reason": f"bad return joints ({exc})"}
        back[-1] = rest
        segment = [(SEG_RETURN, point, False)
                   for point in back + [rest] * SETTLE_HOLD_CYCLES]
        new_plan[low:high] = segment
        gap = len(segment) - (high - low)
        span = (low, low + len(segment))
        if new_by_box and box in new_by_box:
            # The rebuilt return changes length: shift this branch
            # with the growth so far, or the loop would fly the bad
            # branch (and the check would slice it) at stale indices.
            # Later branches are still original here; their turn adds
            # the full growth including this gap.
            shifted = {}
            for seg, (slo, shi) in new_by_box[box].items():
                if seg == "return":
                    shifted[seg] = span
                elif slo >= high - growth:
                    shifted[seg] = (slo + growth + gap,
                                    shi + growth + gap)
                else:
                    shifted[seg] = (slo + growth, shi + growth)
            new_by_box[box] = shifted
            if box == "good" and SEG_RETURN in rebuilt.get("ranges", {}):
                ranges = dict(rebuilt["ranges"])
                ranges["return"] = span
                rebuilt["ranges"] = ranges
        elif SEG_RETURN in rebuilt.get("ranges", {}):
            ranges = dict(rebuilt["ranges"])
            ranges["return"] = span
            rebuilt["ranges"] = ranges
        growth += gap
    if new_by_box:
        rebuilt["ranges_by_box"] = new_by_box
    rebuilt["plan"] = new_plan
    rebuilt["rest_joints"] = rest
    return {"ok": True, "built": rebuilt}


def splice_hover(built: dict, transit: dict) -> dict:
    """Swap the hover prefix of built with the transit plan. Pure.

    built is a _plan_and_check plan (hover first, then lower,
    lift and the drop branches); transit is a build_box_transit
    result. Returns a copy with plan, ranges, shared_ranges and
    ranges_by_box re-indexed; the input dict is unchanged.
    """
    plan = list(built["plan"])
    _, hover_end = built["ranges"][SEG_HOVER]
    assert built["ranges"][SEG_HOVER][0] == 0
    transit_plan = list(transit["plan"])
    new_plan = transit_plan + plan[hover_end:]
    delta = len(transit_plan) - hover_end
    ranges = {SEG_HOVER: (0, len(transit_plan))}
    ranges.update({seg: (low + delta, high + delta)
                   for seg, (low, high) in built["ranges"].items()
                   if seg != SEG_HOVER})
    out = dict(built)
    out["plan"] = new_plan
    out["ranges"] = ranges
    if built.get("shared_ranges"):
        shared = {SEG_HOVER: (0, len(transit_plan))}
        shared.update({seg: (low + delta, high + delta)
                       for seg, (low, high)
                       in built["shared_ranges"].items()
                       if seg != SEG_HOVER})
        out["shared_ranges"] = shared
    if built.get("ranges_by_box"):
        out["ranges_by_box"] = {
            box: _shifted(branch, delta)
            for box, branch in built["ranges_by_box"].items()
        }
    return out
