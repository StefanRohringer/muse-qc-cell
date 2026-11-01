"""Per-part planning and flying for the arm session. Moves nothing itself.

plan_part plans one part through the same _plan_and_check path
as pick_test (every IK solve re-seeded with the joints the arm
holds now: rest for part 1, the box exit for later parts) and
splices the box-exit transit in front; the full spliced plan
additionally goes through the same branch-chained check as
_plan_and_check. run_parts enables
once, starts the control loop once, and flies every part with
the SessionLoop holding the box pose between parts.
"""

from __future__ import annotations

import math
import threading
import time

from muse_qc.arm import safety
from muse_qc.pick import pick_mit
from muse_qc.pick.box_transit import (
    build_box_transit,
    retarget_return,
    splice_hover,
)
from muse_qc.pick.drop_plan import RELEASE_OPEN_MM
from muse_qc.pick.pick_plan_check import _plan_and_check
from muse_qc.pick.pick_test import _lever_solve
from muse_qc.pick.session_loop import SessionLoop, join_done, wait_next_line, wait_part
from muse_qc.pick.session_protocol import _EOF, parse_session_line
from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ
from muse_qc.bringup.first_move_plan import HOME_DEG
from muse_qc.bringup.first_move_plan import clamp_to_limits, read_sdk_vlims, real_tip_m, scale_vlim
from muse_qc.gripper.gripper_grip import width_to_deg
from muse_qc.gripper.gripper_motor import GRIPPER_CONFIG_VLIM
from muse_qc.gripper.grip_speed import GRIPPER_VLIM_FRACTION
from muse_qc.gripper.gripper_stream import open_touch_sensor
from muse_qc.inspection import inspect_step, station_shot


REST_TOL_DEG = 10.0


def check_start_pose(start_deg, joint_limits, out, expected_rest_deg=None):
    """Compare the read start joints with the session rest reference.

    The reference is first_move's HOME_DEG clamped to the limits
    (the folded support pose every session returns to); tests pass
    their fake start as expected_rest_deg. Any joint further than
    REST_TOL_DEG refuses with the hand-move line, so a session
    never enables away from rest. True means enabling is allowed.
    """
    if expected_rest_deg is None:
        limits = (safety.JOINT_LIMITS_DEG if joint_limits is None
                  else joint_limits)
        expected = (tuple(float(v) for v in HOME_DEG) if limits is None
                    else tuple(clamp_to_limits(HOME_DEG, limits)))
    else:
        expected = tuple(float(v) for v in expected_rest_deg)
    read = [float(v) for v in start_deg]
    line = ("arm is not at rest, move it to rest by hand"
            " with the power supply off")
    if len(read) != 6 or len(expected) != 6:
        out(line)
        return False
    if any(abs(got - want) > REST_TOL_DEG
           for got, want in zip(read, expected)):
        out(line)
        return False
    return True


def box_exit_joints(built, box):
    """Planned box exit joints of the sorted box (the held pose)."""
    exits = built.get("exits")
    if exits is None:
        exits = box_exits(built)
    return list(exits[box])


def hold_matches_box(cmd_arm_rad, box_exit_deg, tol_deg=2.0) -> bool:
    """Whether the commanded hold pose matches the planned box exit.

    Compares per joint in degrees; a deviation above tol_deg
    means the arm is not where the transit assumes it is. The
    tolerance covers the planner's own IK drift: the release
    waypoint the loop holds can differ from the nominal drop
    joints by about a degree (elbow joints), so a sub-degree
    tolerance would end every session after the first part.
    """
    held_deg = [math.degrees(v) for v in cmd_arm_rad]
    return len(held_deg) == len(box_exit_deg) and all(
        abs(held - planned) <= tol_deg
        for held, planned in zip(held_deg, box_exit_deg))


def _refusal_text(bad, total, segment, res) -> str:
    """Chained-check refusal naming segment, joint, value and limit."""
    reason = res.get("reason")
    value, limit = res.get("value"), res.get("limit")
    if isinstance(value, dict) and isinstance(limit, dict):
        bits = []
        for joint, val in value.items():
            span = limit.get(joint)
            if isinstance(span, (list, tuple)) and len(span) == 2:
                bits.append(f"{joint} {val:+.2f} deg not in "
                            f"[{span[0]:+.2f}, {span[1]:+.2f}]")
            else:
                bits.append(f"{joint} value {val}, limit {span}")
        detail = "; ".join(bits) if bits else f"value {value}, limit {limit}"
    else:
        detail = f"value {value}, limit {limit}"
    return (f"plan refused: waypoint {bad + 1}/{total} {segment} "
            f"{reason} ({detail})")


def _tips_of(plan, tip_fn):
    """Tip per waypoint with the session tip function."""
    tip_of = real_tip_m if tip_fn is None else tip_fn
    return [tip_of(list(wp)) for _, wp, _ in plan]


def plan_part(start_joints, table_xy, lever_deg, box_exit, out, deps,
              rest_joints=None, transit_from=None):
    """Plan one session part; (open_deg, bundle) or (None, reason).

    Same _plan_and_check as pick_test, then the box-exit
    transit spliced in front when box_exit holds the previous
    box pose. The transit starts at transit_from (the joints the
    arm actually holds) when given, else at the planned box
    exit. With rest_joints (parts 2+, built from the box
    exit) every return segment is retargeted at the measured
    session rest instead of the box. The full plan is checked
    with the same branch chaining as _plan_and_check.
    """
    solve_fn = _lever_solve(table_xy, lever_deg, deps["solve_fn"], out)
    try:
        open_deg, built = _plan_and_check(
            start_joints, deps["grip_start_deg"], table_xy,
            deps["grip_z_mm"], deps["open_mm"], out,
            deps["joint_limits"], solve_fn, deps["check_fn"],
            deps["tip_fn"], deps["use_xy_correction"],
            speed_deg_s=deps["speed_deg_s"],
            accel_deg_s2=deps["accel_deg_s2"], drop=None,
            solve_drop_fn=deps["solve_drop_fn"], inspect=True,
            use_station=deps["use_station"],
            station_j6_deg=deps["station_j6_deg"])
    except Exception as exc:  # noqa: BLE001 - session must end, not crash
        return None, f"plan failed: {exc}"
    if built is None:
        return None, "plan refused"
    if box_exit is not None:
        transit = build_box_transit(
            transit_from if transit_from is not None else box_exit,
            built["hover_joints"],
            free_speed_deg_s=deps["speed_deg_s"],
            accel_deg_s2=deps["accel_deg_s2"], tip_fn=deps["tip_fn"])
        if not transit.get("ok"):
            return None, f"plan refused: {transit.get('reason')}"
        built = splice_hover(built, transit)
        out(f"transit: {len(transit['plan'])} waypoints,"
            f" lowest tip z {transit['lowest_tip_z_mm']:.1f} mm")
    if rest_joints is not None:
        retargeted = retarget_return(
            built, rest_joints, free_speed_deg_s=deps["speed_deg_s"],
            accel_deg_s2=deps["accel_deg_s2"],
            joint_limits=deps["joint_limits"])
        if not retargeted.get("ok"):
            return None, f"plan refused: {retargeted.get('reason')}"
        built = retargeted["built"]
        out("return: retargeted at the session rest pose")
    if box_exit is not None or rest_joints is not None:
        check = deps["check_fn"] or safety.check_target
        if box_exit is not None:
            chained_from = (transit_from if transit_from is not None
                            else box_exit)
        else:
            chained_from = start_joints
        # Same branch chaining as _plan_and_check (shared plus good
        # from the hold pose, bad from the lift seed): chaining the
        # bad branch after the good return at rest would refuse
        # every plan on the step limits.
        ok, bad, res = inspect_step.check_inspect_plan(
            built, chained_from, check, deps["tip_fn"] or real_tip_m)
        if not ok:
            return None, _refusal_text(
                bad, len(built["plan"]), built["plan"][bad][0], res)
        out("session plan waypoints ok")
    tips = _tips_of(built["plan"], deps["tip_fn"])
    return open_deg, {
        "plan": built["plan"], "tips": tips, "ranges": built["ranges"],
        "hover_z": tips[built["ranges"]["hover"][1] - 1][2],
        "open_deg": open_deg,
        "solve_fn": solve_fn,
        "release_open_deg": width_to_deg(RELEASE_OPEN_MM),
        "ranges_by_box": built.get("ranges_by_box"),
        "shared_ranges": built.get("shared_ranges"),
        "hold_cycles": round(deps["hold_s"] * LOOP_RATE_HZ),
        "inspect": True,
        "exits": box_exits(built),
        "hover_joints": built["hover_joints"],
        "drop_joints": built.get("drop_joints"),
        "drop_joints_by_box": built.get("drop_joints_by_box"),
    }


def part_total(num_waypoints, hold_s) -> float:
    """Per-part completion timeout in seconds (same margin as pick_test)."""
    total = num_waypoints + 2 * round(hold_s * LOOP_RATE_HZ) + 500
    return total / LOOP_RATE_HZ + 120.0


def fly_one(loop, out, total_s, hw, clock=None) -> str:
    """Inspect one part and wait for its hold: "part", "end" or "timeout"."""
    shot_holder: dict = {}

    def _start_station_shot() -> None:
        shot_holder["shot"] = station_shot.start_station_table_shot(
            out, open_camera=hw["open_table_camera"],
            recordings_dir=hw["inspect_recordings_dir"])

    box = inspect_step.wait_and_inspect(
        loop, out, ready_timeout_s=total_s,
        on_ready=(_start_station_shot if hw["station_table_shot"] else None),
        find_node=hw["find_arm_node"], open_camera=hw["open_arm_camera"],
        read_key=hw["read_key"], make_client=hw["make_client"],
        count_frame_fn=hw["count_frame"],
        recordings_dir=hw["inspect_recordings_dir"])
    shot = shot_holder.get("shot")
    if shot is not None:
        remaining = (station_shot.STATION_SHOT_TIMEOUT_S
                     - (time.monotonic() - shot.started))
        shot.join(max(remaining, 0.0))
    loop.set_verdict(box)
    return wait_part(loop, total_s, clock)


def spin_loop(sdk, loop, out) -> None:
    """Start the control loop once; never let a bus error hang the session."""
    try:
        sdk.start_control_loop(loop, rate=LOOP_RATE_HZ)
    except Exception as exc:  # noqa: BLE001 - main must still disable
        out(f"motion failed: {exc}")
        if loop.end_note is None:
            loop.end_note = str(exc)
        loop.done.set()


def next_target(lines):
    """Next session target: ("part", ...), ("end",), ("bad", raw) or None (EOF)."""
    item = lines.get()
    if item is _EOF:
        return None
    parsed = parse_session_line(item)
    if parsed is None:
        return ("bad", item)
    return parsed


def box_exits(built) -> dict:
    """Planned box exit joints per box (the held poses after release)."""
    if built.get("drop_joints_by_box"):
        return {box: list(joints)
                for box, joints in built["drop_joints_by_box"].items()}
    return {"good": list(built["drop_joints"]),
            "bad": list(built["drop_joints"])}


def run_parts(sdk, lines, out, deps, hw, arm_start_deg, first_open_deg,
              first_built, owned, clock=None) -> str:
    """Enable once, fly every part, return the session end reason.

    The control loop starts once; the SessionLoop holds the box
    pose between parts. owned collects stream/sensor/arm_sender
    for the caller finally (single stop/disable there).
    """
    limits = safety.JOINT_LIMITS_DEG if deps["joint_limits"] is None \
        else deps["joint_limits"]
    if limits is None:
        return "joint limits unavailable"
    vlims = read_sdk_vlims() if hw["arm_vlims"] is None else list(hw["arm_vlims"])
    if vlims is None:
        return "SDK speed caps unreadable"
    slow = scale_vlim(vlims)
    gripper_slow = [GRIPPER_CONFIG_VLIM * GRIPPER_VLIM_FRACTION]
    out(f"arm vlim 10 %: {[f'{v:.2f}' for v in slow]}"
        f" gripper vlim 25 %: {gripper_slow[0]:.2f}")
    sensor = open_touch_sensor(hw["find_port"], hw["sensor_factory"], out)
    if sensor is None:
        return "no touch sensor"
    owned["sensor"] = sensor
    arm_group, gripper_group = sdk.groups["arm"], sdk.groups["gripper"]
    hold_deg = list(clamp_to_limits(arm_start_deg, limits))
    arm_start_rad = [math.radians(v) for v in hold_deg]
    arm_sender = None
    guard = None
    if hw["arm_mode"] == "mit-ff":
        arm_sender = pick_mit.MitArmSender(
            arm_group, tau_scale=hw["tau_scale"], gravity_fn=hw["gravity_fn"])
        arm_sender.begin(arm_start_rad)
        guard = pick_mit.TrackingGuard()
    else:
        arm_group.mode_pos_vel(vlim=slow)
        arm_group.send_pos_vel(arm_start_rad, vlim=slow)
    owned["arm_sender"] = arm_sender
    gripper_group.mode_pos_vel(vlim=gripper_slow)
    gripper_group.send_pos_vel(
        [math.radians(deps["grip_start_deg"])], vlim=gripper_slow)
    sdk.enable_all()
    check = safety.check_target if deps["check_fn"] is None else deps["check_fn"]
    release_open_deg = width_to_deg(RELEASE_OPEN_MM)
    built, open_deg = first_built, first_open_deg

    def _bundle(open_deg, built):
        tips = _tips_of(built["plan"], deps["tip_fn"])
        return {
            "plan": built["plan"], "tips": tips, "ranges": built["ranges"],
            "hover_z": tips[built["ranges"]["hover"][1] - 1][2],
            "open_deg": open_deg, "release_open_deg": release_open_deg,
            "solve_fn": built.get("solve_fn"),
            "ranges_by_box": built.get("ranges_by_box"),
            "shared_ranges": built.get("shared_ranges"),
            "hold_cycles": round(deps["hold_s"] * LOOP_RATE_HZ),
            "inspect": True,
            "exits": built.get("exits"),
        }

    first = _bundle(open_deg, built)
    loop = SessionLoop(
        arm_group, gripper_group, slow, gripper_slow, arm_start_rad,
        first["plan"], first["tips"], first["ranges"], first["hover_z"],
        first["open_deg"], deps["grip_start_deg"], first["hold_cycles"],
        False, sensor, out, check, deps["tip_fn"] or real_tip_m, None,
        first["release_open_deg"], inspect=True,
        ranges_by_box=first["ranges_by_box"],
        shared_ranges=first["shared_ranges"], arm_sender=arm_sender,
        tracking_guard=guard, hold_wait_s=hw["hold_wait_s"],
        rest_deg=hold_deg, home_speed_deg_s=deps["speed_deg_s"],
        home_accel_deg_s2=deps["accel_deg_s2"],
        home_solve_fn=first.get("solve_fn"))
    owned["stream"] = loop
    # The spin thread only registers the loop callback and returns;
    # aliveness says nothing (a stored callback means a finished
    # thread). Startup failures surface through loop.done.
    threading.Thread(target=spin_loop, args=(sdk, loop, out),
                     daemon=True).start()
    while True:
        outcome = fly_one(loop, out, part_total(len(loop.plan), deps["hold_s"]),
                        hw, clock)
        if outcome == "timeout":
            loop.request_end("part timeout")
            join_done(loop, clock=clock)
            return "part timeout"
        if outcome == "end":
            join_done(loop, clock=clock)
            return loop.end_reason or loop.end_note or "error"
        box_exit = box_exit_joints(built, loop.drop_box)
        held_deg = [math.degrees(v) for v in loop.cmd_arm_rad]
        if not hold_matches_box(loop.cmd_arm_rad, box_exit):
            reason = "held pose differs from planned box exit"
            loop.request_end(reason)
            join_done(loop, clock=clock)
            return reason
        kind, payload = wait_next_line(lines, loop, hw["hold_wait_s"], clock)
        if kind == "eof":
            loop.request_end("eof")
            join_done(loop, clock=clock)
            return "eof"
        if kind == "gone":
            join_done(loop, clock=clock)
            return loop.end_reason or loop.end_note or "hold timeout"
        parsed = parse_session_line(payload)
        if parsed is None:
            loop.request_end(f"bad line {payload!r}")
            join_done(loop, clock=clock)
            return f"bad line {payload!r}"
        if parsed[0] != "part":
            loop.request_end("end")
            join_done(loop, clock=clock)
            return "end"
        _, x, y, lever_deg = parsed
        table_xy = (x, y)
        open_deg, built = plan_part(
            box_exit, table_xy, lever_deg, box_exit, out, deps,
            rest_joints=arm_start_deg, transit_from=held_deg)
        if open_deg is None:
            loop.request_end(str(built))
            join_done(loop, clock=clock)
            return str(built)
        if not loop.load_next(_bundle(open_deg, built)):
            join_done(loop, clock=clock)
            return loop.end_reason or loop.end_note or "hold timeout"
