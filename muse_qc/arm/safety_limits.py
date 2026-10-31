"""Joint-limit loader for the safety checks. Read-only, moves nothing.

Locates the installed reBotArm_control_py package without importing
it, follows config/rebotarm.yaml to the hardware config, then reads
the joint limits from the URDF file named there. Lives in its own
module so muse_qc/safety.py stays at or below 300 lines.
"""

from __future__ import annotations

import importlib.util
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

# Same six joints as muse_qc.arm.safety.JOINT_NAMES.
_SDK_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _load_joint_limits_from_sdk() -> dict[str, tuple[float, float]] | None:
    """Read joint1..joint6 limits from the Seeed SDK config, in degrees.

    Returns None when any step fails; check_target() then refuses
    every move (fail closed).
    """
    try:
        spec = importlib.util.find_spec("reBotArm_control_py")
        if spec is None or spec.origin is None:
            return None
        repo = Path(spec.origin).resolve().parent.parent
        hardware_name = "rebotarm_dm.yaml"
        main_text = (repo / "config" / "rebotarm.yaml").read_text(encoding="utf-8")
        match = re.search(r'hardware_yaml\s*:\s*["\']?([\w.\-]+\.yaml)["\']?', main_text)
        if match is not None:
            hardware_name = match.group(1)
        hardware_text = (repo / "config" / hardware_name).read_text(encoding="utf-8")
        match = re.search(r'urdf_path\s*:\s*["\']?([\w./\-]+\.urdf)["\']?', hardware_text)
        if match is None:
            return None
        root = ET.parse(repo / match.group(1)).getroot()
        limits: dict[str, tuple[float, float]] = {}
        for joint in root.iter("joint"):
            name = joint.get("name")
            if name not in _SDK_JOINT_NAMES:
                continue
            limit = joint.find("limit")
            if limit is None:
                return None
            limits[name] = (math.degrees(float(limit.get("lower"))), math.degrees(float(limit.get("upper"))))
        if len(limits) != len(_SDK_JOINT_NAMES):
            return None
        return limits
    except (OSError, ValueError, ET.ParseError):
        return None


# Joint limits in degrees, read from the Seeed SDK configuration at
# import. None means the limits could not be read, and then every move
# is refused (fail closed).
JOINT_LIMITS_DEG: dict[str, tuple[float, float]] | None = _load_joint_limits_from_sdk()
