"""Plan-and-check glue for the pick entry point. Pure, moves nothing.

Moved out of muse_qc.pick.pick_test so that module stays within the
400 line limit. Builds the pick/inspect plan through
muse_qc.pick.pick_plan and checks every waypoint chained.
"""

from __future__ import annotations

import math

from muse_qc.inspection import inspect_step
from muse_qc.arm import safety
from muse_qc.pick.drop_plan import (
    BOXES_TABLE_MM,
    RELEASE_OPEN_MM,
    solve_drop,
)
from muse_qc.pick.inspect_station import (
    STATION_J6_DEG,
    STATION_TABLE_MM,
    STATION_TIP_Z_MM,
    USE_STATION_DEFAULT,
    build_inspect_plan_with_station,
)
from muse_qc.bringup.first_move_plan import real_tip_m
from muse_qc.gripper.gripper_grip import validate_targets, width_to_deg
from muse_qc.pick.pick_args import ACCEL_DEG_S2, SPEED_DEG_S
from muse_qc.pick.pick_plan import (
    build_inspect_plan,
    build_pick_plan,
    check_pick_plan,
    plan_summary,
)
from muse_qc.arm.target_pose import table_to_base_mm


def _plan_and_check(arm_start_deg, grip_start_deg, table_xy, grip_z_mm, open_mm,
                    out, joint_limits, solve_fn, check_fn, tip_fn,
                    use_xy_correction=True, speed_deg_s=SPEED_DEG_S[2],
                    accel_deg_s2=ACCEL_DEG_S2[2], drop=None,
                    solve_drop_fn=None, inspect=False, use_station=None,
                    station_j6_deg=None):
    if use_station is None:
        # The station only inspects, so the default applies to
        # inspect runs; plain picks behave exactly as before.
        use_station = USE_STATION_DEFAULT and inspect
    if use_station and not inspect:
        out("station needs --inspect")
        return None, None
    open_deg = width_to_deg(open_mm)
    refused = validate_targets(grip_start_deg, open_deg)
    boxes = ("good", "bad") if inspect else ([drop] if drop is not None else [])
    release_open_deg = None
    if boxes:
        release_open_deg = width_to_deg(RELEASE_OPEN_MM)
        refused = refused or validate_targets(grip_start_deg, release_open_deg)
    if refused is not None:
        out(refused)
        return None, None
    out(f"open width: {open_mm:.0f} mm -> gripper {open_deg:+.2f} deg")
    if drop is not None:
        out(f"release width: {RELEASE_OPEN_MM:.0f} mm"
            f" -> gripper {release_open_deg:+.2f} deg")
    check = safety.check_target if check_fn is None else check_fn
    tip_of = real_tip_m if tip_fn is None else tip_fn
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    try:
        solved = None
        if inspect:
            solver = solve_drop if solve_drop_fn is None else solve_drop_fn
            solved = {}
            for box in boxes:
                one = solver(BOXES_TABLE_MM[box], arm_start_deg)
                if not one.get("ok"):
                    out(f"drop solve failed: {one.get('reason')}")
                    return None, None
                solved[box] = one
            j6 = STATION_J6_DEG if station_j6_deg is None \
                else float(station_j6_deg)
            built = build_inspect_plan_with_station(
                arm_start_deg, table_xy, grip_z_mm, joint_limits,
                solve_fn=solve_fn, use_xy_correction=use_xy_correction,
                free_speed_deg_s=speed_deg_s, accel_deg_s2=accel_deg_s2,
                drop_joints_by_box={box: one["joints_deg"]
                                    for box, one in solved.items()},
                tip_fn=tip_of, use_station=use_station,
                station_j6_deg=j6)
        else:
            drop_joints = None
            if drop is not None:
                solver = solve_drop if solve_drop_fn is None else solve_drop_fn
                solved = solver(BOXES_TABLE_MM[drop], arm_start_deg)
                if not solved.get("ok"):
                    out(f"drop solve failed: {solved.get('reason')}")
                    return None, None
                drop_joints = solved["joints_deg"]
            built = build_pick_plan(arm_start_deg, table_xy, grip_z_mm,
                                    joint_limits, solve_fn=solve_fn,
                                    use_xy_correction=use_xy_correction,
                                    free_speed_deg_s=speed_deg_s, accel_deg_s2=accel_deg_s2,
                                    drop_joints=drop_joints, tip_fn=tip_of)
        if not built.get("ok"):
            out(f"plan failed: {built.get('reason')}")
            return None, None
        for line in plan_summary(built["plan"], built):
            out(line)
        if use_station and built.get("use_station"):
            shown = STATION_J6_DEG if station_j6_deg is None \
                else float(station_j6_deg)
            out(f"station: table mm ({STATION_TABLE_MM[0]:.1f},"
                f" {STATION_TABLE_MM[1]:.1f}), tip z {STATION_TIP_Z_MM:.1f} mm,"
                f" joint 6 {shown:+.1f} deg")
        for box in boxes:
            one = solved[box] if inspect else solved
            box_xy = BOXES_TABLE_MM[box]
            base_x, base_y = table_to_base_mm(box_xy[0], box_xy[1])
            radius = math.hypot(base_x, base_y)
            joints = one["joints_deg"]
            tip = one["tip_base_mm"]
            out(f"drop box: {box} at table mm ({box_xy[0]:+.1f},"
                f" {box_xy[1]:+.1f}), r={radius:.1f} mm")
            out("drop joints deg: " + " ".join(f"{v:+.2f}" for v in joints))
            out(f"drop tilt: {one['tilt_deg']:.1f} deg,"
                f" fk tip base mm ({tip[0]:+.1f}, {tip[1]:+.1f}, {tip[2]:+.1f}),"
                f" real tip z {one['real_tip_z_mm']:.1f} mm")
            branch = built.get("ranges_by_box", {}).get(box, built["ranges"])
            low, high = branch["to-box"]
            lowest = min(tip_of(list(wp))[2]
                         for _, wp, _ in built["plan"][low:high]) * 1000.0
            out(f"to-box lowest tip z: {lowest:.1f} mm")
        if inspect:
            ok, bad, res = inspect_step.check_inspect_plan(
                built, arm_start_deg, check, tip_of)
        else:
            ok, bad, res = check_pick_plan(
                built["plan"], arm_start_deg, check, tip_of)
    finally:
        safety.set_j6_ref(old_ref)
    if not ok:
        out(f"waypoint {bad + 1}/{len(built['plan'])} REFUSED {res.get('reason')}")
        return None, None
    out("all waypoints ok")
    return open_deg, built
