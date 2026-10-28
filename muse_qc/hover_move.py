"""Hover move for the reBot B601-DM arm. The human runs it by hand.

Given a part position in table millimetres (table frame of
muse_qc.table_coords: origin at the centre of marker 0, +x towards
marker 1, +y towards marker 2 away from the arm), this module
moves the gripper tip from the current rest pose to the hover
pose above the part (the same pose target_pose.solve_hover solves
and checks), holds it for 2 s, ramps back to the rest reading
clamped into its URDF limits, holds it for 1 s, and only then
lets first_move disable the motors. The arm is never disabled
while it is away from the rest pose.

Targets below 80 mm hover are refused in this module: the real
tip sits about 13 mm lower than the FK model predicts, so a
lower FK hover could touch the part or the table.

Default is a dry run: connect through the SDK, read the six
joints, solve the hover pose, check every waypoint through
safety.check_target and print one line per waypoint, then
disconnect. The dry run never enables the motors and never sends
anything. With --execute the same validation runs first and the
human confirms with "yes"; only then the arm moves at 10 % of
the SDK speed caps with a fresh safety check in every control
cycle.
"""

from __future__ import annotations

import argparse
import sys

from muse_qc import safety
from muse_qc.first_move import dry_run as first_dry_run
from muse_qc.first_move import execute as first_execute
from muse_qc.first_move_plan import (
    HOLD_CYCLES,
    LOOP_RATE_HZ,
    SETTLE_HOLD_CYCLES,
    clamp_to_limits,
    ramp_waypoints,
)
from muse_qc.target_pose import (
    HOVER_MM,
    base_to_table_mm,
    solve_hover,
    table_to_base_mm,
)

MIN_HOVER_MM = 80.0
# NOTE: the arm_calibration correction was verified by the human on
# 2026-10-28 at two 100 mm hovers (docs/DEVLOG.md); heights below
# 100 mm are not verified yet. The 80 mm floor above stays.

SEG_HOVER = "hover"  # ramp from the rest reading to the hover joints
SEG_HOLD = "hold"  # hold the hover joints (--hold-s, default 2 s)
SEG_RETURN = "return"  # ramp from the hover joints to the clamped rest
SEG_SETTLE = "settle"  # hold the clamped rest for 1 s before disable


def build_hover_return_plan(
    start_deg: object,
    goal_deg: object,
    joint_limits: object = None,
    hold_cycles: int = HOLD_CYCLES,
) -> list[tuple[str, tuple[float, ...]]]:
    """Ramp rest -> hover, hold, ramp back, hold 1 s. Pure, no hardware.

    Returns (segment, waypoint) pairs. The return ramp ends exactly
    on the rest reading clamped into its URDF limits, and the final
    settle segment holds that clamped pose, so first_move disables
    the motors only at the rest pose.
    """
    limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
    if limits is None:
        raise RuntimeError("Joint limits unavailable, cannot build the plan.")
    start = tuple(float(value) for value in start_deg)
    goal = tuple(float(value) for value in goal_deg)
    if len(start) != 6 or len(goal) != 6:
        raise ValueError("Need six joint angles for start and goal.")
    rest = clamp_to_limits(start, limits)
    hover_pts = ramp_waypoints(start, goal)
    hover_pts[-1] = goal  # exact end: s + (e - s) is not exact in floats
    return_pts = ramp_waypoints(goal, rest)
    return_pts[-1] = rest  # exact end, so disable happens at the rest pose
    plan = [(SEG_HOVER, point) for point in hover_pts]
    plan += [(SEG_HOLD, goal)] * int(hold_cycles)
    plan += [(SEG_RETURN, point) for point in return_pts]
    plan += [(SEG_SETTLE, rest)] * SETTLE_HOLD_CYCLES
    return plan


def make_hover_return_plan_fn(
    table_xy: tuple[float, float],
    hover_mm: float,
    joint_limits: object = None,
    out: object = print,
    hold_cycles: int = HOLD_CYCLES,
):
    """Return plan_fn(start_deg) for first_move over a table target.

    Solves the hover joints with target_pose.solve_hover using the
    start joints as the seed (first_move sets J6_REF first, so the
    call runs with the right joint 6 reference), prints the same
    lines as target_pose.run, then builds the hover-return plan. A
    failed solve or a hover below 80 mm raises RuntimeError with
    the reason, so first_move prints it and moves nothing.
    """
    table_x, table_y = float(table_xy[0]), float(table_xy[1])
    hover = float(hover_mm)

    def plan_fn(start_deg: object) -> list[tuple[str, tuple[float, ...]]]:
        start = [float(value) for value in start_deg]
        base_x, base_y = table_to_base_mm(table_x, table_y)
        out(f"table target: x={table_x:+.2f} y={table_y:+.2f} mm")
        out(f"base target: x={base_x:+.2f} y={base_y:+.2f} mm")
        out(f"hover: {hover:+.1f} mm")
        out("start joints deg: " + " ".join(f"{v:+.2f}" for v in start))
        if hover < MIN_HOVER_MM:
            raise RuntimeError(
                f"hover too low: {hover:.1f} mm below {MIN_HOVER_MM:.0f} mm "
                "(real tip sits about 13 mm lower than FK)"
            )
        solved = solve_hover((table_x, table_y), hover, start)
        if not solved.get("ok"):
            raise RuntimeError(str(solved.get("reason", "unreachable")))
        joints = solved["joints_deg"]
        assert joints is not None
        out("ik joints deg: " + " ".join(f"{v:+.2f}" for v in joints))
        tip = solved["tip_base_mm"]
        assert tip is not None
        table_tip = base_to_table_mm(tip[0], tip[1])
        out(f"fk tip base mm: x={tip[0]:+.2f} y={tip[1]:+.2f} z={tip[2]:+.2f}")
        out(f"fk tip table mm: x={table_tip[0]:+.2f} "
            f"y={table_tip[1]:+.2f} z={tip[2]:+.2f}")
        out(f"error to target: {solved['error_mm']:.2f} mm")
        corr = solved["tip_table_mm"]
        corr_table = base_to_table_mm(corr[0], corr[1])
        out(f"corrected tip table mm: x={corr_table[0]:+.2f} "
            f"y={corr_table[1]:+.2f} z={corr[2]:+.2f} "
            f"(iterations {solved['iterations']})")
        return build_hover_return_plan(start, joints, joint_limits, hold_cycles)

    return plan_fn


def _create_sdk() -> object:
    """Create the real SDK object. Called only by main(), never by tests."""
    from reBotArm_control_py.actuator import RebotArm

    return RebotArm()


def main(argv: object = None, sdk_factory: object = None, **kwargs: object) -> int:
    """Entry point for the human: dry run by default, --execute moves."""
    parser = argparse.ArgumentParser(description="Hover move (human-run only).")
    parser.add_argument("--table-mm", nargs=2, type=float, required=True,
                        metavar=("X", "Y"), help="part position in table mm")
    parser.add_argument("--hover", type=float, default=HOVER_MM,
                        help="tip height above the table in mm")
    parser.add_argument("--execute", action="store_true",
                        help="move the arm after confirmation")
    parser.add_argument("--hold-s", type=float, default=2.0,
                        help="hover hold time in seconds (0.5 to 30.0)")
    args = parser.parse_args(argv)
    if not 0.5 <= args.hold_s <= 30.0:
        parser.error("--hold-s must be within 0.5 to 30.0 seconds")
    out = kwargs.get("out", print)
    joint_limits = kwargs.get("joint_limits", None)
    plan_fn = make_hover_return_plan_fn(
        (args.table_mm[0], args.table_mm[1]), args.hover,
        joint_limits, out, round(args.hold_s * LOOP_RATE_HZ),
    )
    kwargs = dict(kwargs)
    kwargs["plan_fn"] = plan_fn
    sdk = sdk_factory() if sdk_factory is not None else _create_sdk()
    if args.execute:
        return first_execute(sdk, **kwargs)
    return first_dry_run(sdk, **kwargs)


if __name__ == "__main__":
    sys.exit(main())
