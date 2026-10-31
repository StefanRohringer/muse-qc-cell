"""Skip zones for find_parts diff blobs (no hardware, pure check)."""

from __future__ import annotations

import math

from muse_qc.pick.drop_plan import BOXES_TABLE_MM
from muse_qc.arm.target_pose import ARM_BASE_IN_TABLE_MM

BOX_KEEP_OUT_MM = 70.0
ARM_ZONE_TABLE_MM = ((380.0, 610.0, 0.0, 200.0),
                     (430.0, 590.0, 200.0, 300.0))
REACH_MAX_MM = 345.0


def classify_skip(table_mm):
    """Return the skip reason for a candidate table position, or None.

    "in box good"/"in box bad" is within BOX_KEEP_OUT_MM (max of
    |dx|, |dy|) of the drop_plan box centre, "arm zone" is inside
    ARM_ZONE_TABLE_MM, "out of reach" is farther than REACH_MAX_MM
    from the joint 1 axis. None (no mapping) is never skipped.
    """
    if table_mm is None:
        return None
    table_x, table_y = float(table_mm[0]), float(table_mm[1])
    for name in ("good", "bad"):
        box = BOXES_TABLE_MM.get(name)
        if box is None:
            continue
        if max(abs(table_x - box[0]), abs(table_y - box[1])) \
                <= BOX_KEEP_OUT_MM:
            return f"in box {name}"
    for zone in ARM_ZONE_TABLE_MM:
        x_min, x_max, y_min, y_max = zone
        if x_min <= table_x <= x_max and y_min <= table_y <= y_max:
            return "arm zone"
    base_x, base_y = ARM_BASE_IN_TABLE_MM
    if math.dist((table_x, table_y), (base_x, base_y)) > REACH_MAX_MM:
        return "out of reach"
    return None
