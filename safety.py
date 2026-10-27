"""Pre-move safety checks for the reBot B601-DM arm.

Pure checks only: this module never opens a serial port or camera and
never commands motion. Tools must pass every planned target through
check_target() and every gripper reading through check_gripper()
before sending anything to the arm.

Base frame: origin at the center of the arm base on the table, z up.
The Seeed SDK (reBotArm_control_py) reports end-effector positions in
the URDF base_link frame in meters with z up (joint1 spins about +z;
SDK example 6_ik_test.py reads "<x> <y> <z>" in meters). The SDK never
says which horizontal axis faces the front of the cell, so this module
defines the cell convention on top of the SDK frame: +x forward toward
the front table edge, +y left, right-handed with z up.
"""

from __future__ import annotations

import importlib.util
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

# All limits live here, in meters and degrees.
TABLE_FRONT_M = 0.545  # Front table edge from base center (+x).
TABLE_LEFT_M = 0.525  # Left table edge from base center (+y).
TABLE_RIGHT_M = 0.525  # Right table edge from base center (-y).
TABLE_BACK_M = 0.055  # Back table edge from base center (-x).
EDGE_MARGIN_M = 0.05  # Keep-out margin from every table edge.
WORK_X_MIN_M = 0.10  # Nothing behind the base: 10 cm forward minimum.
WORK_X_MAX_M = TABLE_FRONT_M - EDGE_MARGIN_M  # Front reach (0.495).
WORK_Y_MAX_M = TABLE_LEFT_M - EDGE_MARGIN_M  # Left/right reach (0.475).
TOOL_Z_MIN_M = 0.03  # Tool height above the table, minimum.
TOOL_Z_MAX_M = 0.45  # Tool height above the table, maximum.
J6_CABLE_LIMIT_DEG = 75.0  # Joint 6 hard limit around J6_REF (camera cable).
SPEED_DEFAULT = 0.10  # Default speed fraction (10 %).
SPEED_MAX = 0.30  # Hard speed cap (30 %).
MAX_STEP_M = 0.05  # Max Cartesian step per command.
MAX_STEP_JOINT_DEG = 10.0  # Max step per joint per command.
STATE_MAX_AGE_MS = 200.0  # Arm readings older than this are stale.
GRIP_MAX_RETRIES = 2  # Missed-grip retries before STOP.

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIP_SENSORS = frozenset({"L1", "L2", "R1", "R2"})
GRIP_LEFT = frozenset({"L1", "L2"})
GRIP_RIGHT = frozenset({"R1", "R2"})
GRIP_PHASES = ("open", "open-moving", "closed", "holding-moving", "holding-lowering")

# Camera-cable reference for joint 6, in degrees. None until the user
# sets it via set_j6_ref(); while None, every move that changes joint 6
# is refused.
J6_REF: float | None = None

_STOP_REASON: str | None = None
_GRIP_RETRIES = 0
_HOLDING_CONTACTS: frozenset[str] | None = None


def _load_joint_limits_from_sdk() -> dict[str, tuple[float, float]] | None:
    """Read joint1..joint6 limits from the Seeed SDK config, in degrees.

    Locates the installed reBotArm_control_py package without importing
    it, follows config/rebotarm.yaml to the hardware config, then reads
    the joint limits from the URDF file named there. Returns None when
    any step fails; check_target() then refuses every move.
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
            if name not in JOINT_NAMES:
                continue
            limit = joint.find("limit")
            if limit is None:
                return None
            limits[name] = (math.degrees(float(limit.get("lower"))), math.degrees(float(limit.get("upper"))))
        if len(limits) != len(JOINT_NAMES):
            return None
        return limits
    except (OSError, ValueError, ET.ParseError):
        return None


# Joint limits in degrees, read from the Seeed SDK configuration at
# import. None means the limits could not be read, and then every move
# is refused (fail closed).
JOINT_LIMITS_DEG: dict[str, tuple[float, float]] | None = _load_joint_limits_from_sdk()


@dataclass(frozen=True)
class Target:
    """Planned move. Full x/y/z in meters plus all six joints in degrees are required; speed 0..1."""

    x: float | None = None
    y: float | None = None
    z: float | None = None
    joints_deg: tuple[float, ...] | None = None  # joint1..joint6
    speed: float | None = None  # None means SPEED_DEFAULT


@dataclass(frozen=True)
class CurrentState:
    """Latest arm reading. Joints may hold None/NaN/0.0 from failed reads."""

    x: float | None = None
    y: float | None = None
    z: float | None = None
    joints_deg: tuple[float | None, ...] | None = None  # joint1..joint6
    reading_age_ms: float | None = None


def _refuse(reason: str, value: object, limit: object) -> dict:
    """Build the refusal result every blocked move returns."""
    return {"ok": False, "reason": reason, "value": value, "limit": limit}


def set_stop(reason: str) -> dict:
    """Latch the STOP flag; every move is refused until the user clears it."""
    global _STOP_REASON
    if _STOP_REASON is None:
        _STOP_REASON = str(reason)
    return _refuse("stop-latched", _STOP_REASON, None)


def clear_stop(*, confirmed_by_user: bool = False) -> dict:
    """Clear STOP only with confirmed_by_user=True, so the agent cannot.

    Called without user confirmation this refuses and keeps the flag.
    A user-confirmed clear also resets grip retry and hold tracking.
    """
    global _STOP_REASON, _GRIP_RETRIES, _HOLDING_CONTACTS
    if not confirmed_by_user:
        return _refuse("stop-clear-needs-user", _STOP_REASON, True)
    _STOP_REASON = None
    _GRIP_RETRIES = 0
    _HOLDING_CONTACTS = None
    return {"ok": True, "detail": "stop cleared by user"}


def is_stopped() -> bool:
    """Whether the STOP flag currently blocks all moves."""
    return _STOP_REASON is not None


def set_j6_ref(value_deg: float | None) -> None:
    """Set the joint 6 cable reference in degrees (None unsets it)."""
    global J6_REF
    if value_deg is not None and not math.isfinite(value_deg):
        raise ValueError(f"J6_REF must be finite or None, got {value_deg!r}.")
    J6_REF = value_deg


def _check_current(state: CurrentState) -> dict | None:
    """Refuse on stale readings or failed-read values, else None."""
    if state.reading_age_ms is None or not math.isfinite(state.reading_age_ms):
        return _refuse("stale-state", state.reading_age_ms, STATE_MAX_AGE_MS)
    if state.reading_age_ms > STATE_MAX_AGE_MS:
        return _refuse("stale-state", state.reading_age_ms, STATE_MAX_AGE_MS)
    if state.joints_deg is None or len(state.joints_deg) != len(JOINT_NAMES):
        return _refuse("invalid-current-reading", state.joints_deg, "six finite nonzero angles")
    for name, value in zip(JOINT_NAMES, state.joints_deg):
        if value is None or not math.isfinite(value) or value == 0.0:
            return _refuse("invalid-current-reading", {name: value}, "finite and nonzero")
    if (
        state.x is None
        or state.y is None
        or state.z is None
        or not math.isfinite(state.x)
        or not math.isfinite(state.y)
        or not math.isfinite(state.z)
    ):
        return _refuse("invalid-current-position", (state.x, state.y, state.z), "finite numbers")
    return None


def check_target(target: Target, current_state: CurrentState) -> dict:
    """Check a planned move against every safety limit. Pure, moves nothing."""
    if is_stopped():
        return _refuse("stop-latched", _STOP_REASON, None)
    if JOINT_LIMITS_DEG is None:
        return _refuse("joint-limits-unavailable", None, "readable Seeed SDK joint limits")
    if target.x is None or target.y is None or target.z is None or target.joints_deg is None:
        return _refuse("incomplete-target", (target.x, target.y, target.z, target.joints_deg is None), "full x/y/z and six joints")
    assert target.joints_deg is not None
    if any(not math.isfinite(v) for v in (target.x, target.y, target.z)):
        return _refuse("bad-target", (target.x, target.y, target.z), "finite numbers")
    if len(target.joints_deg) != len(JOINT_NAMES):
        return _refuse("bad-target", target.joints_deg, "six joint angles")
    if any(not math.isfinite(v) for v in target.joints_deg):
        return _refuse("bad-target", target.joints_deg, "finite numbers")
    failed = _check_current(current_state)
    if failed is not None:
        return failed
    assert current_state.joints_deg is not None
    assert target.x is not None and target.y is not None and target.z is not None
    if not WORK_X_MIN_M <= target.x <= WORK_X_MAX_M:
        return _refuse("workspace-x", target.x, [WORK_X_MIN_M, WORK_X_MAX_M])
    if abs(target.y) > WORK_Y_MAX_M:
        return _refuse("workspace-y", target.y, [-WORK_Y_MAX_M, WORK_Y_MAX_M])
    if not TOOL_Z_MIN_M <= target.z <= TOOL_Z_MAX_M:
        return _refuse("tool-z", target.z, [TOOL_Z_MIN_M, TOOL_Z_MAX_M])
    for name, value in zip(JOINT_NAMES, target.joints_deg):
        low, high = JOINT_LIMITS_DEG[name]
        if not low <= value <= high:
            return _refuse("joint-limit", {name: value}, {name: [low, high]})
    target_j6 = target.joints_deg[5]
    current_j6 = current_state.joints_deg[5]
    assert current_j6 is not None
    if J6_REF is None:
        return _refuse("joint6-ref-unset", target_j6, "user-set J6_REF")
    if abs(target_j6 - J6_REF) > J6_CABLE_LIMIT_DEG:
        return _refuse("joint6-cable", target_j6, [J6_REF - J6_CABLE_LIMIT_DEG, J6_REF + J6_CABLE_LIMIT_DEG])
    speed = SPEED_DEFAULT if target.speed is None else target.speed
    if not math.isfinite(speed) or not 0.0 < speed <= SPEED_MAX:
        return _refuse("speed-cap", target.speed, SPEED_MAX)
    assert current_state.x is not None and current_state.y is not None and current_state.z is not None
    step = math.dist((current_state.x, current_state.y, current_state.z), (target.x, target.y, target.z))
    if step > MAX_STEP_M:
        return _refuse("step-cartesian", step, MAX_STEP_M)
    peak = max(abs(t - c) for t, c in zip(target.joints_deg, current_state.joints_deg) if c is not None)
    if peak > MAX_STEP_JOINT_DEG:
        return _refuse("step-joint", peak, MAX_STEP_JOINT_DEG)
    return {"ok": True, "speed": speed, "detail": "target passes all safety checks"}


def check_gripper(phase: str, contacts: set[str] | frozenset[str] | list[str] | tuple[str, ...]) -> dict:
    """Check 4-sensor gripper contact state. Latches STOP on danger rows.

    Phases: "open", "open-moving", "closed", "holding-moving",
    "holding-lowering". contacts holds the sensor names with contact
    (subset of L1, L2 on the left finger and R1, R2 on the right).
    """
    global _GRIP_RETRIES, _HOLDING_CONTACTS
    if phase not in GRIP_PHASES:
        return _refuse("unknown-phase", phase, sorted(GRIP_PHASES))
    seen = set(contacts)
    unknown = seen - GRIP_SENSORS
    if unknown:
        return _refuse("unknown-sensor", sorted(unknown), sorted(GRIP_SENSORS))
    if phase == "open":
        if not seen:
            return {"ok": True, "grip": "open", "detail": "gripper open, no contact"}
        return _refuse("unexpected-contact", sorted(seen), "no contact while open")
    if phase == "open-moving":
        if not seen:
            return {"ok": True, "grip": "open", "detail": "gripper open, no contact"}
        set_stop("gripper contact while moving with open gripper")
        return _refuse("contact-while-moving-open", sorted(seen), "no contact while moving")
    if phase == "closed":
        left = bool(seen & GRIP_LEFT)
        right = bool(seen & GRIP_RIGHT)
        if left and right:
            _GRIP_RETRIES = 0
            _HOLDING_CONTACTS = frozenset(seen)
            return {"ok": True, "grip": "held", "detail": "part held by both fingers"}
        _GRIP_RETRIES += 1
        if _GRIP_RETRIES > GRIP_MAX_RETRIES:
            set_stop(f"missed grip {_GRIP_RETRIES - 1} retries exhausted")
            return _refuse("missed-grip-retries-exhausted", sorted(seen), "contact on left and right finger")
        return {
            "ok": False,
            "reason": "missed-grip",
            "value": sorted(seen),
            "limit": "contact on left and right finger",
            "action": "retry",
            "attempt": _GRIP_RETRIES,
        }
    if _HOLDING_CONTACTS is None:
        return _refuse("no-hold-baseline", phase, "a confirmed held grasp first")
    # NOTE: the holding-lowering rule below (any contact change means the
    # part was placed) is written from the sensor spec only and must be
    # verified on the real gripper before it sorts parts.
    if seen != _HOLDING_CONTACTS:
        if phase == "holding-moving":
            set_stop("gripper contact changed while holding and moving")
            return _refuse("contact-change-while-holding", sorted(seen), sorted(_HOLDING_CONTACTS))
        _HOLDING_CONTACTS = None
        return {"ok": True, "grip": "placed", "action": "open-gripper", "detail": "part placed, open gripper"}
    return {"ok": True, "grip": "holding", "detail": "grasp unchanged"}
