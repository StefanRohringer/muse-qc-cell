"""Grip yaw from the lever direction. Pure, no SDK, no motion.

find_parts reports lever_deg in the table frame (0 along table +x,
90 along table +y, modulo 180). The hover IK in target_pose turns
the tool down about z by the radial yaw atan2(base_y, base_x), which
grips correctly only when the levers run along that radial
direction. This module computes the extra joint 6 turn so every
part is gripped like a straight part.

Cable limit: the joint 6 camera cable allows +/- MAX_GRIP_YAW_DEG
70; the offset is clamped into that range and the clamp is
reported. safety.check_target still checks every waypoint against
the 75 deg J6 limit.
"""

from __future__ import annotations

import math

MAX_GRIP_YAW_DEG = 70.0


def grip_yaw_offset_deg(
    table_xy: object,
    lever_deg: float,
) -> tuple[float, bool]:
    """Extra tool turn about z in deg, plus whether it was clamped.

    The lever direction is mapped into the base frame with
    table_to_base_mm on the part centre and centre plus the unit
    lever direction, its angle is compared with the radial angle
    atan2(base_y, base_x) of the part, and the difference is wrapped
    modulo 180 into [-90, 90) (a 180 deg turn of the part does not
    matter). Positive means counterclockwise seen from above: from
    base +x toward base +y, i.e. the tool-down rotation grows by
    that angle. The result is clamped to +/- MAX_GRIP_YAW_DEG.
    """
    from muse_qc.arm.target_pose import table_to_base_mm

    try:
        center = (float(table_xy[0]), float(table_xy[1]))
        lever = float(lever_deg) % 180.0
    except (TypeError, ValueError, IndexError):
        raise ValueError("Need a table [x, y] position and lever deg.")
    if not all(math.isfinite(v) for v in center + (lever,)):
        raise ValueError("Need a table [x, y] position and lever deg.")
    direction = (math.cos(math.radians(lever)),
                 math.sin(math.radians(lever)))
    base = table_to_base_mm(center[0], center[1])
    ahead = table_to_base_mm(center[0] + direction[0],
                             center[1] + direction[1])
    lever_angle = math.degrees(math.atan2(ahead[1] - base[1],
                                          ahead[0] - base[0]))
    radial = math.degrees(math.atan2(base[1], base[0]))
    offset = (lever_angle - radial + 90.0) % 180.0 - 90.0
    if offset > MAX_GRIP_YAW_DEG:
        return (MAX_GRIP_YAW_DEG, True)
    if offset < -MAX_GRIP_YAW_DEG:
        return (-MAX_GRIP_YAW_DEG, True)
    return (offset, False)
