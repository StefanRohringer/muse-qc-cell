"""Target pose dry run for the reBot B601-DM arm. The human runs it by hand.

Given a part position in table millimetres (table frame of
muse_qc.table_coords: origin at the centre of marker 0, +x towards
marker 1, +y towards marker 2 away from the arm), this module
computes the arm pose with the gripper tip straight above the part
at a hover height, plans the joint ramp from the current joints to
that pose, and checks every waypoint with safety.check_target.
Dry run only: nothing is enabled, nothing is sent, and this
module offers no motion flag at all.
"""

from __future__ import annotations

import argparse
import math

from muse_qc import fk_check, safety
from muse_qc.first_move_plan import (
    HOLD_CYCLES,
    LOOP_RATE_HZ,
    SPEED_FRACTION,
    ramp_waypoints,
    real_tip_m,
)

# Joint 1 axis in table mm, from human tape measurements (2026-10-28,
# arm power supply off): 495 mm to marker 0, 485 mm to marker 1,
# 705 mm to marker 2; least-squares fit with TABLE_MARKER_LAYOUT_MM
# gives (495.9, 22.7) mm with residuals of at most 1.6 mm.
ARM_BASE_IN_TABLE_MM = (495.9, 22.7)

HOVER_MM = 100.0
TIP_TOL_MM = 2.0
AXIS_TOL_DEG = 2.0
# Rotation with the tool axis (+x of end_link at home) pointing down.
TOOL_DOWN_ROT = ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (-1.0, 0.0, 0.0))
READING_AGE_MS = 1000.0 / LOOP_RATE_HZ
SEG_MOVE = "move"
SEG_HOLD = "hold"


def table_to_base_mm(x: float, y: float) -> tuple[float, float]:
    """Convert table mm to arm base mm (base x forward, y left, z up)."""
    base_x = float(y) - ARM_BASE_IN_TABLE_MM[1]
    base_y = -(float(x) - ARM_BASE_IN_TABLE_MM[0])
    return (base_x, base_y)


def base_to_table_mm(base_x: float, base_y: float) -> tuple[float, float]:
    """Convert arm base mm back to table mm (inverse of table_to_base_mm)."""
    table_x = ARM_BASE_IN_TABLE_MM[0] - float(base_y)
    table_y = float(base_x) + ARM_BASE_IN_TABLE_MM[1]
    return (table_x, table_y)


def tool_axis(pose: dict) -> tuple[float, float, float]:
    """Unit vector from the link6 origin to the end_link origin."""
    axis = (
        pose["x_mm"] - pose["link6_x_mm"],
        pose["y_mm"] - pose["link6_y_mm"],
        pose["z_mm"] - pose["link6_z_mm"],
    )
    norm = math.sqrt(sum(v * v for v in axis))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Pose has no usable tool axis.")
    return (axis[0] / norm, axis[1] / norm, axis[2] / norm)


def solve_hover(
    table_xy: tuple[float, float],
    hover_mm: float,
    seed_joints_deg: object,
) -> dict:
    """Solve the hover joints for a table target. Pure SDK math, no bus.

    Places end_link so the tip (TIP_OFFSET_MM further along the tool
    axis, pointing straight down) sits at the target, calls the SDK
    IK (compute_ik) with the seed joints, and verifies with FK that
    the tip is within TIP_TOL_MM of the target. The local solver
    starts from the seed, so the solution stays near the seed,
    including joint 6 (gripper cable).
    """
    import numpy as np

    try:
        from reBotArm_control_py.kinematics import compute_ik
    except ImportError:
        return {"ok": False, "reason": "unreachable: SDK kinematics missing"}
    table_x, table_y = float(table_xy[0]), float(table_xy[1])
    hover = float(hover_mm)
    seed = [float(value) for value in seed_joints_deg]
    if len(seed) != 6 or not all(math.isfinite(v) for v in seed):
        return {"ok": False, "reason": "unreachable: bad seed joints"}
    if not all(math.isfinite(v) for v in (table_x, table_y, hover)):
        return {"ok": False, "reason": "unreachable: bad table target"}
    base_x, base_y = table_to_base_mm(table_x, table_y)
    tip_m = (base_x / 1000.0, base_y / 1000.0, hover / 1000.0)
    end_m = (
        tip_m[0],
        tip_m[1],
        (hover + fk_check.TIP_OFFSET_MM) / 1000.0,
    )
    # Turn the tool-down orientation about the vertical axis by the
    # joint 1 direction, so side targets need no wrist twist and
    # joint 6 stays near the seed (gripper cable).
    yaw = math.atan2(base_y, base_x)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    turn = (
        (cos_yaw, -sin_yaw, 0.0),
        (sin_yaw, cos_yaw, 0.0),
        (0.0, 0.0, 1.0),
    )
    target_rot = tuple(
        tuple(
            sum(turn[i][k] * TOOL_DOWN_ROT[k][j] for k in range(3))
            for j in range(3)
        )
        for i in range(3)
    )
    result = compute_ik(
        np.radians(np.array(seed)),
        np.array(end_m),
        np.array(target_rot),
    )
    if not result.success:
        return {"ok": False, "reason": "unreachable: IK did not converge"}
    joints_deg = tuple(float(v) for v in np.degrees(result.q[:6]))
    pose = fk_check.pose_mm([math.radians(v) for v in joints_deg])
    tip = fk_check.tip_mm(pose)
    error_mm = math.dist(
        (tip["x_mm"], tip["y_mm"], tip["z_mm"]),
        (base_x, base_y, hover),
    )
    if error_mm > TIP_TOL_MM:
        return {"ok": False, "reason": "unreachable: FK tip off target"}
    axis = tool_axis(pose)
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))
    if tilt_deg > AXIS_TOL_DEG:
        return {"ok": False, "reason": "unreachable: tool axis not down"}
    return {
        "ok": True,
        "joints_deg": joints_deg,
        "tip_base_mm": (tip["x_mm"], tip["y_mm"], tip["z_mm"]),
        "error_mm": error_mm,
        "tilt_deg": tilt_deg,
    }


def build_hover_plan(
    start_deg: object, goal_deg: object
) -> list[tuple[str, tuple[float, ...]]]:
    """Ramp from the start joints to the goal joints, then a 2 s hold."""
    waypoints = [(SEG_MOVE, point) for point in ramp_waypoints(start_deg, goal_deg)]
    goal = tuple(float(value) for value in goal_deg)
    waypoints += [(SEG_HOLD, goal)] * HOLD_CYCLES
    return waypoints


def check_plan(
    plan: list[tuple[str, tuple[float, ...]]],
) -> tuple[bool, int, dict | None]:
    """Check every waypoint with safety.check_target. Pure, moves nothing.

    Each waypoint is checked as its own fresh reading (one loop
    period old), so the check verifies limits, workspace, tool
    height, cable and speed for every pose; step continuity between
    consecutive waypoints is measured directly against the same
    MAX_STEP limits the check enforces.
    """
    previous_tip: tuple[float, float, float] | None = None
    previous_joints: tuple[float, ...] | None = None
    for index, (_, waypoint) in enumerate(plan):
        target_tip = real_tip_m(list(waypoint))
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current = safety.CurrentState(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), reading_age_ms=READING_AGE_MS,
        )
        result = safety.check_target(target, current)
        if not result.get("ok"):
            return False, index, result
        if previous_tip is not None:
            assert previous_joints is not None
            step_m = math.dist(previous_tip, target_tip)
            if step_m > safety.MAX_STEP_M:
                return False, index, {
                    "ok": False, "reason": "step-cartesian",
                    "value": step_m, "limit": safety.MAX_STEP_M,
                }
            peak = max(
                abs(w - p) for w, p in zip(waypoint, previous_joints)
            )
            if peak > safety.MAX_STEP_JOINT_DEG:
                return False, index, {
                    "ok": False, "reason": "step-joint",
                    "value": peak, "limit": safety.MAX_STEP_JOINT_DEG,
                }
        previous_tip = target_tip
        previous_joints = tuple(waypoint)
    return True, -1, None


def read_arm_once() -> list[float]:
    """Read the six joints once through the read-only Arm wrapper."""
    from muse_qc.arm import Arm

    arm = Arm()
    arm.connect()
    try:
        return [joint.degrees for joint in arm.read_joints()[:6]]
    finally:
        arm.disconnect()


def run(
    table_xy: tuple[float, float],
    hover_mm: float,
    start_deg: list[float],
    out: object = print,
) -> int:
    """Solve, plan and check one hover target. Returns the exit code."""
    table_x, table_y = float(table_xy[0]), float(table_xy[1])
    base_x, base_y = table_to_base_mm(table_x, table_y)
    out(f"table target: x={table_x:+.2f} y={table_y:+.2f} mm")
    out(f"base target: x={base_x:+.2f} y={base_y:+.2f} mm")
    out(f"hover: {float(hover_mm):+.1f} mm")
    old_ref = safety.J6_REF
    safety.set_j6_ref(float(start_deg[5]))
    try:
        solved = solve_hover((table_x, table_y), hover_mm, start_deg)
        if not solved.get("ok"):
            out(solved.get("reason", "unreachable"))
            out("dry run: no motion")
            return 1
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
        plan = build_hover_plan(start_deg, joints)
        out(f"waypoints: {len(plan)}")
        ok, bad_index, result = check_plan(plan)
        if ok:
            out("all waypoints ok")
        else:
            assert result is not None
            out(f"waypoint {bad_index + 1}/{len(plan)} "
                f"REFUSED {result.get('reason')}")
            out("dry run: no motion")
            return 1
        out("dry run: no motion")
        return 0
    finally:
        safety.set_j6_ref(old_ref)


def main(argv: object = None) -> int:
    """Entry point for the human: dry run only, never moves the arm."""
    parser = argparse.ArgumentParser(description="Target pose dry run (no motion).")
    parser.add_argument("--table-mm", nargs=2, type=float, required=True,
                        metavar=("X", "Y"), help="part position in table mm")
    parser.add_argument("--hover", type=float, default=HOVER_MM,
                        help="tip height above the table in mm")
    parser.add_argument("--start", choices=("home", "arm"), default="arm",
                        help="home needs no hardware, arm reads joints once")
    args = parser.parse_args(argv)
    if args.start == "home":
        start_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    else:
        try:
            start_deg = read_arm_once()
        except Exception as exc:
            print(f"arm read failed: {exc}")
            return 2
    return run((args.table_mm[0], args.table_mm[1]), args.hover, start_deg)


if __name__ == "__main__":
    raise SystemExit(main())
