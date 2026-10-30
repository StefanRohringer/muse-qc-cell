"""Target pose dry run for the reBot B601-DM arm. The human runs it by hand.

Given a part position in table millimetres (table frame of
muse_qc.table_coords), this module solves the hover joints with the
corrected tip above the part, plans the joint ramp, and checks every
waypoint with safety.check_target. Dry run only, no motion flag.
"""

from __future__ import annotations

import argparse
import math

from muse_qc import arm_calibration, fk_check, safety
from muse_qc.first_move_plan import (
    HOLD_CYCLES,
    LOOP_RATE_HZ,
    SPEED_FRACTION,
    ramp_waypoints,
    real_tip_m,
)

# Joint 1 axis in table mm, from human tape measurements (2026-10-28):
# least-squares fit gives (495.9, 22.7) mm, residuals at most 1.6 mm.
ARM_BASE_IN_TABLE_MM = (495.9, 22.7)

HOVER_MM = 100.0
TIP_TOL_MM = 2.0
AXIS_TOL_DEG = 2.0
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


def _solve_nominal(
    table_xy: tuple[float, float],
    hover_mm: float,
    seed_joints_deg: object,
    *,
    yaw_offset_deg: float = 0.0,
) -> dict:
    """Nominal (uncorrected) hover joints via the SDK IK. No bus.

    Used by arm_calibration.solve_corrected for each iteration.
    yaw_offset_deg turns the tool about z past the radial yaw
    (grip yaw from the lever direction); 0 keeps the old heading.
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
    try:
        offset = float(yaw_offset_deg)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "unreachable: bad yaw offset"}
    if not all(math.isfinite(v) for v in (table_x, table_y, hover, offset)):
        return {"ok": False, "reason": "unreachable: bad table target"}
    base_x, base_y = table_to_base_mm(table_x, table_y)
    tip_m = (base_x / 1000.0, base_y / 1000.0, hover / 1000.0)
    end_m = (
        tip_m[0],
        tip_m[1],
        (hover + fk_check.TIP_OFFSET_MM) / 1000.0,
    )
    # Turn tool-down about z by the joint 1 direction (joint 6 cable).
    yaw = math.atan2(base_y, base_x) + math.radians(offset)
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


def solve_hover(
    table_xy: tuple[float, float],
    hover_mm: float,
    seed_joints_deg: object,
    *,
    yaw_offset_deg: float = 0.0,
) -> dict:
    """Hover joints so the corrected tip lands on the target.

    Iterates the nominal IK (arm_calibration.solve_corrected).
    tip_base_mm stays the nominal FK tip; tip_table_mm is the
    corrected tip (base x/y, table z), plus the iteration count.
    yaw_offset_deg turns the tool about z past the radial yaw.
    """
    try:
        table = (float(table_xy[0]), float(table_xy[1]))
    except (TypeError, ValueError, IndexError):
        return {"ok": False, "reason": "unreachable: bad table target"}
    base_x, base_y = table_to_base_mm(table[0], table[1])
    solved = arm_calibration.solve_corrected(
        (base_x, base_y), hover_mm, seed_joints_deg,
        yaw_offset_deg=yaw_offset_deg)
    if not solved.get("ok"):
        return {"ok": False, "reason": str(solved.get("reason"))}
    joints = solved["joints_deg"]
    pose = fk_check.pose_mm([math.radians(v) for v in joints])
    tip = fk_check.tip_mm(pose)
    axis = tool_axis(pose)
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, -axis[2]))))
    if tilt_deg > AXIS_TOL_DEG:
        return {"ok": False, "reason": "unreachable: tool axis not down"}
    return {
        "ok": True,
        "joints_deg": joints,
        "tip_base_mm": (tip["x_mm"], tip["y_mm"], tip["z_mm"]),
        "tip_table_mm": solved["tip_table_mm"],
        "error_mm": solved["error_mm"],
        "tilt_deg": tilt_deg,
        "iterations": solved["iterations"],
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
    start_deg: object,
) -> tuple[bool, int, dict | None]:
    """Check every waypoint with safety.check_target (chained). Pure."""
    seed = tuple(float(value) for value in start_deg)
    if len(seed) != 6 or not all(math.isfinite(v) and v != 0.0 for v in seed):
        seed = None
    previous: tuple[float, ...] | None = seed
    for index, (_, waypoint) in enumerate(plan):
        target_tip = real_tip_m(list(waypoint))
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current_joints = tuple(waypoint) if previous is None else previous
        current_tip = target_tip if previous is None else real_tip_m(list(previous))
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=current_joints, reading_age_ms=READING_AGE_MS,
        )
        result = safety.check_target(target, current)
        if not result.get("ok"):
            return False, index, result
        previous = tuple(waypoint)
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
    out("start joints deg: " + " ".join(f"{v:+.2f}" for v in start_deg))
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
        corr = solved["tip_table_mm"]
        corr_table = base_to_table_mm(corr[0], corr[1])
        out(f"corrected tip table mm: x={corr_table[0]:+.2f} "
            f"y={corr_table[1]:+.2f} z={corr[2]:+.2f} "
            f"(iterations {solved['iterations']})")
        plan = build_hover_plan(start_deg, joints)
        out(f"waypoints: {len(plan)}")
        ok, bad_index, result = check_plan(plan, start_deg)
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
