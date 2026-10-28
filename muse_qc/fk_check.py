"""Read-only forward kinematics check for the reBot B601-DM arm.

Pure computation through the official Seeed SDK (Pinocchio model
only): this module never opens the bus and never moves anything.
The hand-run main() only reads joint angles via muse_qc.arm.Arm
and prints the end_link pose plus an estimated finger-tip position.
"""

from __future__ import annotations

import math

# Estimate from a hand measurement, verify.
TIP_OFFSET_MM = 30.0


def pose_mm(joint_radians: object) -> dict:
    """Return the end_link pose for six arm joint angles in radians.

    Pure SDK computation (no bus): x, y, z in mm and roll, pitch,
    yaw in degrees, plus the rotation matrix and the link6 origin
    in mm. The link6 origin defines the tool axis together with
    the end_link origin (the fixed end_joint offset runs along
    the link6 z axis, not along the end_link z axis).
    """
    import numpy as np
    import pinocchio as pin
    from reBotArm_control_py.kinematics import (
        compute_fk,
        get_arm_joint_count,
        load_robot_model,
        pad_q_for_model,
    )

    values = [float(value) for value in joint_radians]
    if len(values) != 6:
        raise ValueError(f"Need six joint angles in radians, got {len(values)}.")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Joint angles must be finite numbers.")
    model = load_robot_model()
    full_q = pad_q_for_model(
        model, np.array(values), controlled_joints=get_arm_joint_count(model)
    )
    position, rotation, _ = compute_fk(model, full_q)
    link6_position, _, _ = compute_fk(model, full_q, frame_name="link6")
    euler = pin.rpy.matrixToRpy(rotation)
    pos_mm = position * 1000.0
    link6_mm = link6_position * 1000.0
    return {
        "x_mm": float(pos_mm[0]),
        "y_mm": float(pos_mm[1]),
        "z_mm": float(pos_mm[2]),
        "roll_deg": float(math.degrees(euler[0])),
        "pitch_deg": float(math.degrees(euler[1])),
        "yaw_deg": float(math.degrees(euler[2])),
        "rotation": tuple(tuple(float(v) for v in row) for row in rotation),
        "link6_x_mm": float(link6_mm[0]),
        "link6_y_mm": float(link6_mm[1]),
        "link6_z_mm": float(link6_mm[2]),
    }


def tip_mm(pose: dict) -> dict:
    """Project the finger tip from an end_link pose along the tool axis.

    Moves TIP_OFFSET_MM along the tool axis, the unit vector from
    the link6 origin to the end_link origin (both from SDK forward
    kinematics on the same joint angles), never from a guessed
    direction.
    """
    offset = TIP_OFFSET_MM
    end = (pose["x_mm"], pose["y_mm"], pose["z_mm"])
    base = (pose["link6_x_mm"], pose["link6_y_mm"], pose["link6_z_mm"])
    axis = [e - b for e, b in zip(end, base)]
    norm = math.sqrt(sum(v * v for v in axis))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Pose has no usable tool axis.")
    unit = [v / norm for v in axis]
    return {
        "x_mm": float(end[0] + offset * unit[0]),
        "y_mm": float(end[1] + offset * unit[1]),
        "z_mm": float(end[2] + offset * unit[2]),
    }


def main() -> None:
    """Read joints once and print end_link pose and tip. Hand-run only."""
    from muse_qc.arm import Arm

    arm = Arm()
    arm.connect()
    try:
        joints = arm.read_joints()[:6]
        for joint in joints:
            print(f"{joint.name}: {joint.degrees:+.2f} deg")
        pose = pose_mm([joint.radians for joint in joints])
        print(
            f"end_link: x={pose['x_mm']:+.1f} y={pose['y_mm']:+.1f} "
            f"z={pose['z_mm']:+.1f} mm "
            f"roll={pose['roll_deg']:+.2f} pitch={pose['pitch_deg']:+.2f} "
            f"yaw={pose['yaw_deg']:+.2f} deg"
        )
        tip = tip_mm(pose)
        print(
            f"tip: x={tip['x_mm']:+.1f} y={tip['y_mm']:+.1f} "
            f"z={tip['z_mm']:+.1f} mm "
            f"(offset {TIP_OFFSET_MM:.0f} mm along tool axis)"
        )
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
