"""Drop plan for sorting parts into the good and bad box. Pure, moves nothing.

After a successful pick the arm carries the part to one of two open
boxes and opens the gripper over it. The boxes were measured by the
human with ``python -m muse_qc.vision.vision_sam "box"`` (calibrated). The
IK aims the nominal tip 170 mm above the table with the tool tilted
30 deg, so the SDK IK still reaches both boxes; a vertical tool does
not reach that far at this height. The arm_calibration correction
puts the real tip about 40 mm lower, about 130 mm above the table,
i.e. about 60 mm above the box rim. The x/y correction is NOT
applied to the drop pose (it was fitted only for r 312.8 to 345 mm);
the wide box opening forgives 2 to 3 cm.
"""

from __future__ import annotations

import math

from muse_qc.bringup import fk_check
from muse_qc.bringup.first_move_plan import (
    LOOP_RATE_HZ,
    SETTLE_HOLD_CYCLES,
    ramp_waypoints,
    real_tip_m,
)
from muse_qc.arm.target_pose import (
    TOOL_DOWN_ROT,
    base_to_table_mm,
    table_to_base_mm,
    tool_axis,
)

# Box centres of the top rim in the camera view, table mm, from the
# human run of python -m muse_qc.vision.vision_sam "box" on 2026-10-29
# (calibrated): left box = good, right box = bad.
BOXES_TABLE_MM = {"good": (376.4, 423.8), "bad": (675.4, 420.3)}
BOX_HEIGHT_MM = 70.0
# Nominal IK tip height above the table. The arm_calibration
# correction puts the real tip about 40 mm lower, about 130 mm,
# i.e. about 60 mm above the 70 mm high rim.
DROP_NOMINAL_Z_MM = 170.0
# Lowest allowed corrected (real) tip z at the drop joints, 40 mm
# above the rim.
DROP_MIN_REAL_Z_MM = 110.0
DROP_TILT_DEG = 30.0  # Tool tilt from vertical so the IK reaches the boxes.
TRANSIT_FLOOR_MM = 90.0  # Corrected tip z floor for the whole to-box ramp.
TIP_TOL_MM = 5.0  # FK tip must land this close to the drop target.
AXIS_TOL_DEG = 2.0  # FK tool tilt must match DROP_TILT_DEG this closely.
DROP_SETTLE_S = 0.5  # Settle at the drop pose before the gripper opens.
DROP_SETTLE_CYCLES = int(DROP_SETTLE_S * LOOP_RATE_HZ)
DROP_HOLD_S = 1.0  # Hold open over the box before the single sensor verdict.
DROP_HOLD_CYCLES = int(DROP_HOLD_S * LOOP_RATE_HZ)
RELEASE_OPEN_MM = 40.0  # Jaw opening for the release over the box.
SEG_TOBOX = "to-box"
SEG_RELEASE = "release"


def _mat_mul(left: object, right: object) -> tuple:
    """Multiply two 3x3 rotation matrices given as row tuples."""
    return tuple(
        tuple(sum(left[i][k] * right[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _tilt_outward(yaw_rad: float, tilt_deg: float) -> tuple:
    """Base-frame rotation tilting "down" outward by tilt_deg.

    Rotates about the tangential axis (-sin yaw, cos yaw, 0) so the
    tool axis tips away from the joint 1 axis in the radial
    direction (finger tips point outward, away from the arm).
    """
    tilt = math.radians(float(tilt_deg))
    tx, ty = -math.sin(yaw_rad), math.cos(yaw_rad)
    cos_t, sin_t = math.cos(-tilt), math.sin(-tilt)
    outer = 1.0 - cos_t
    return (
        (cos_t + tx * tx * outer, tx * ty * outer, ty * sin_t),
        (tx * ty * outer, cos_t + ty * ty * outer, -tx * sin_t),
        (-ty * sin_t, tx * sin_t, cos_t),
    )


def solve_drop(box_xy_table_mm: object, seed_deg: object) -> dict:
    """Nominal tilted drop joints over one box. No bus, SDK IK math only.

    Same machinery as target_pose._solve_nominal (table_to_base_mm,
    end_link = tip minus the tip offset along the tool axis, joint 6
    turned by the joint 1 direction), but the tool axis is tilted
    DROP_TILT_DEG away from the joint 1 axis in the radial
    direction. Verified with FK: the tip must land within
    TIP_TOL_MM of the target and the real tool tilt within
    AXIS_TOL_DEG of DROP_TILT_DEG. The corrected tip (real_tip_m)
    at the drop joints must also clear DROP_MIN_REAL_Z_MM. No xy
    correction is applied.
    """
    try:
        table = (float(box_xy_table_mm[0]), float(box_xy_table_mm[1]))
        seed = [float(value) for value in seed_deg]
    except (TypeError, ValueError, IndexError):
        return {"ok": False, "reason": "unreachable: bad box target"}
    if len(seed) != 6 or not all(math.isfinite(v) for v in seed + list(table)):
        return {"ok": False, "reason": "unreachable: bad box target"}
    try:
        from reBotArm_control_py.kinematics import compute_ik
    except ImportError:
        return {"ok": False, "reason": "unreachable: SDK kinematics missing"}
    import numpy as np

    base_x, base_y = table_to_base_mm(table[0], table[1])
    tip_z = float(DROP_NOMINAL_Z_MM)
    yaw = math.atan2(base_y, base_x)
    tilt_rad = math.radians(DROP_TILT_DEG)
    axis = (
        math.sin(tilt_rad) * math.cos(yaw),
        math.sin(tilt_rad) * math.sin(yaw),
        -math.cos(tilt_rad),
    )
    off_m = fk_check.TIP_OFFSET_MM / 1000.0
    tip_m = (base_x / 1000.0, base_y / 1000.0, tip_z / 1000.0)
    end_m = (tip_m[0] - axis[0] * off_m,
             tip_m[1] - axis[1] * off_m,
             tip_m[2] - axis[2] * off_m)
    # Turn tool-down about z by the joint 1 direction (joint 6 cable),
    # then tilt outward in the base frame.
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    turn = (
        (cos_yaw, -sin_yaw, 0.0),
        (sin_yaw, cos_yaw, 0.0),
        (0.0, 0.0, 1.0),
    )
    target_rot = _mat_mul(
        _tilt_outward(yaw, DROP_TILT_DEG), _mat_mul(turn, TOOL_DOWN_ROT))
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
        (tip["x_mm"], tip["y_mm"], tip["z_mm"]), (base_x, base_y, tip_z))
    if error_mm > TIP_TOL_MM:
        return {"ok": False, "reason": "unreachable: FK tip off target"}
    axis_now = tool_axis(pose)
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, -axis_now[2]))))
    if abs(tilt_deg - DROP_TILT_DEG) > AXIS_TOL_DEG:
        return {"ok": False, "reason": "unreachable: tool tilt off target"}
    real_z_mm = real_tip_m(list(joints_deg))[2] * 1000.0
    if real_z_mm < DROP_MIN_REAL_Z_MM:
        return {"ok": False,
                "reason": f"unreachable: drop tip too low "
                          f"(real z {real_z_mm:.1f} mm)"}
    table_tip = base_to_table_mm(tip["x_mm"], tip["y_mm"])
    return {
        "ok": True,
        "joints_deg": joints_deg,
        "tip_base_mm": (tip["x_mm"], tip["y_mm"], tip["z_mm"]),
        "tip_table_mm": (table_tip[0], table_tip[1], tip["z_mm"]),
        "tilt_deg": tilt_deg,
        "error_mm": error_mm,
        "real_tip_z_mm": real_z_mm,
    }


def build_drop_segments(
    lift_top_deg: object,
    drop_deg: object,
    rest_deg: object,
    free_speed_deg_s: float = 15.0,
    accel_deg_s2: float = 30.0,
    tip_fn: object = None,
) -> dict:
    """Carry, release and return segments after the pick lift. Pure.

    to-box is a trapezoid ramp (same speed/accel as the pick free
    ramps) from the joints at the top of the lift to the drop
    joints plus a short settle; release holds the drop joints
    while the loop opens the gripper (the loop decides the length,
    one waypoint marks the range); return ramps back to the
    rest/settle pose like today. Every to-box waypoint must keep
    the corrected tip at or above TRANSIT_FLOOR_MM, else the plan
    fails with "transit too low" (no detours are tried).
    """
    tip_of = real_tip_m if tip_fn is None else tip_fn
    try:
        top = tuple(float(v) for v in lift_top_deg)
        drop = tuple(float(v) for v in drop_deg)
        rest = tuple(float(v) for v in rest_deg)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "bad drop joints"}
    if not (len(top) == len(drop) == len(rest) == 6):
        return {"ok": False, "reason": "bad drop joints"}
    if not all(math.isfinite(v) for v in top + drop + rest):
        return {"ok": False, "reason": "bad drop joints"}
    floor_m = TRANSIT_FLOOR_MM / 1000.0
    lowest_mm = math.inf
    try:
        carry = list(ramp_waypoints(top, drop,
                                    speed_deg_s=free_speed_deg_s,
                                    accel_deg_s2=accel_deg_s2))
        for index, point in enumerate(carry + [drop] * DROP_SETTLE_CYCLES):
            tip_z = tip_of(list(point))[2]
            lowest_mm = min(lowest_mm, tip_z * 1000.0)
            if tip_z < floor_m:
                return {"ok": False,
                        "reason": f"transit too low: tip z "
                                  f"{tip_z * 1000.0:.1f} mm below "
                                  f"{TRANSIT_FLOOR_MM:.0f} mm at to-box "
                                  f"waypoint {index + 1}"}
        back = list(ramp_waypoints(drop, rest,
                                   speed_deg_s=free_speed_deg_s,
                                   accel_deg_s2=accel_deg_s2))
    except (ValueError, TypeError, AttributeError) as exc:
        return {"ok": False, "reason": f"bad drop joints ({exc})"}
    plan: list[tuple[str, tuple[float, ...], bool]] = []
    ranges: dict[str, tuple[int, int]] = {}

    def _append(segment: str, points: list[tuple[float, ...]]) -> None:
        start = len(plan)
        plan.extend((segment, point, False) for point in points)
        ranges[segment] = (start, len(plan))

    _append(SEG_TOBOX, carry + [drop] * DROP_SETTLE_CYCLES)
    _append(SEG_RELEASE, [drop])
    _append("return", back + [rest] * SETTLE_HOLD_CYCLES)
    return {
        "ok": True,
        "plan": plan,
        "ranges": ranges,
        "drop_joints": drop,
        "lowest_tip_z_mm": lowest_mm,
    }
