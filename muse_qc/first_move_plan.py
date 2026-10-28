"""Joint-space plan for the first arm motion. Pure, moves nothing.

Builds the waypoint list for muse_qc/first_move.py: ramp from the
measured rest position to home (all joints zero), joint1 to +5 deg,
joint1 back to zero, then a hold. Ramps are linear in joint space so
every step stays within the per-cycle speed budget. Also reads the
per-joint speed caps from the Seeed SDK config (values only, the SDK
package itself is located but never imported).
"""

from __future__ import annotations

import importlib.util
import math
import re
import threading
import time
from pathlib import Path

from muse_qc import fk_check, safety

LOOP_RATE_HZ = 50.0
JOINT_SPEED_DEG_S = 5.0
STEP_DEG = JOINT_SPEED_DEG_S / LOOP_RATE_HZ  # 0.1 deg per command.
J1_TEST_DEG = 5.0
HOLD_S = 2.0
HOLD_CYCLES = int(HOLD_S * LOOP_RATE_HZ)
SETTLE_CYCLES = HOLD_CYCLES // 2
SPEED_FRACTION = 0.1  # Plan speed as a fraction for safety.check_target.
VLIM_FRACTION = 0.1  # Motor speed caps run at 10 % of the SDK values.
HOME_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
ZERO_RETRIES = 5
ZERO_DELAY_S = 0.2

_PACKAGE_NAME = "reBotArm_control_py"
_GLOBAL_CONFIG_NAME = "rebotarm.yaml"


def ramp_waypoints(
    start_deg: object, end_deg: object
) -> list[tuple[float, ...]]:
    """Linear joint-space ramp, excluding start, including end.

    Step count follows the largest joint move so no joint travels
    more than STEP_DEG per step.
    """
    start = [float(value) for value in start_deg]
    end = [float(value) for value in end_deg]
    if len(start) != 6 or len(end) != 6:
        raise ValueError("Need six joint angles for start and end.")
    if not all(math.isfinite(v) for v in start + end):
        raise ValueError("Joint angles must be finite numbers.")
    peak = max(abs(e - s) for s, e in zip(start, end))
    steps = max(1, int(math.ceil(peak / STEP_DEG)))
    return [
        tuple(s + (e - s) * i / steps for s, e in zip(start, end))
        for i in range(1, steps + 1)
    ]


def build_plan(start_deg: object) -> list[tuple[float, ...]]:
    """Full waypoint list: to home, joint1 +5 deg, back, hold 2 s."""
    start = tuple(float(value) for value in start_deg)
    if len(start) != 6:
        raise ValueError("Need six joint angles for the start.")
    middle = (J1_TEST_DEG,) + HOME_DEG[1:]
    waypoints = ramp_waypoints(start, HOME_DEG)
    waypoints += ramp_waypoints(HOME_DEG, middle)
    waypoints += ramp_waypoints(middle, HOME_DEG)
    waypoints += [HOME_DEG] * HOLD_CYCLES
    return waypoints


def scale_vlim(vlims: object, fraction: float = VLIM_FRACTION) -> list[float]:
    """Scale SDK speed caps to the given fraction (default 10 %)."""
    values = [float(value) for value in vlims]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Need six finite SDK speed caps.")
    if not 0.0 < fraction <= 1.0:
        raise ValueError("Fraction must be within (0, 1].")
    return [value * fraction for value in values]


def _config_dir(config_dir: object = None) -> Path | None:
    """Return the SDK config folder (found, never imported)."""
    if config_dir is not None:
        base = Path(config_dir)
        return base if base.is_dir() else None
    try:
        spec = importlib.util.find_spec(_PACKAGE_NAME)
    except (ImportError, AttributeError, ValueError):
        return None
    if spec is None:
        return None
    package_dir: Path | None = None
    locations = spec.submodule_search_locations
    if locations:
        package_dir = Path(next(iter(locations)))
    elif spec.origin:
        package_dir = Path(spec.origin).parent
    if package_dir is None:
        return None
    for candidate in (package_dir.parent / "config", package_dir / "config"):
        try:
            if candidate.is_dir():
                return candidate
        except OSError:
            continue
    return None


def _scalar(path: Path, key: str) -> str | None:
    """Return a "key: value" scalar without a YAML parser."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        name, _, value = stripped.partition(":")
        if name.strip() != key:
            continue
        cleaned = value.split("#", 1)[0].strip().strip("\"'")
        return cleaned or None


def read_sdk_vlims(config_dir: object = None) -> list[float] | None:
    """Return the six arm speed caps from the SDK config, or None.

    Follows config/rebotarm.yaml to the hardware config, reads the
    arm group joint order and each joint's POS_VEL vlim. Any failure
    returns None so the caller refuses to move (fail closed).
    """
    base = _config_dir(config_dir)
    if base is None:
        return None
    hardware_name = _scalar(base / _GLOBAL_CONFIG_NAME, "hardware_yaml")
    if not hardware_name:
        return None
    hardware_path = (
        Path(hardware_name)
        if Path(hardware_name).is_absolute()
        else base / hardware_name
    )
    try:
        text = hardware_path.read_text(encoding="utf-8")
    except OSError:
        return None
    arm_match = re.search(r"arm:\s*\n\s*joints:\s*\[([^\]]+)\]", text)
    if arm_match is None:
        return None
    arm_names = [name.strip().strip("\"'") for name in arm_match.group(1).split(",")]
    if len(arm_names) != 6:
        return None
    vlims: dict[str, float] = {}
    current: str | None = None
    for line in text.splitlines():
        stripped = line.strip()
        name_match = re.match(r"-\s*name:\s*(\S+)", stripped)
        if name_match is not None:
            current = name_match.group(1).strip("\"'")
            continue
        vlim_match = re.match(r"vlim:\s*([0-9.]+)", stripped)
        if vlim_match is not None and current is not None:
            try:
                vlims[current] = float(vlim_match.group(1))
            except ValueError:
                return None
    try:
        return [vlims[name] for name in arm_names]
    except KeyError:
        return None


def read_current_deg(group: object) -> list[float]:
    """Read six joint angles in degrees, retrying empty 0.0 feedback."""
    positions: list[float] = []
    for _ in range(ZERO_RETRIES):
        positions = [float(value) for value in group.get_positions()]
        if len(positions) == 6 and not any(v == 0.0 for v in positions):
            break
        time.sleep(ZERO_DELAY_S)
    if len(positions) != 6:
        raise RuntimeError(f"Expected six joint angles, got {len(positions)}.")
    if any(v == 0.0 for v in positions):
        raise RuntimeError("No joint feedback: readings stay at 0.0.")
    return [math.degrees(value) for value in positions]


def real_tip_m(joints_deg: object) -> tuple[float, float, float]:
    """Tool tip in metres for six joint angles in degrees via fk_check."""
    pose = fk_check.pose_mm([math.radians(float(v)) for v in joints_deg])
    tip = fk_check.tip_mm(pose)
    return (tip["x_mm"] / 1000.0, tip["y_mm"] / 1000.0, tip["z_mm"] / 1000.0)


def validate_and_print(
    group: object,
    start_deg: list[float],
    waypoints: list[tuple[float, ...]],
    check_fn: object = None,
    tip_fn: object = None,
    out: object = print,
) -> tuple[bool, list[tuple[float, float, float]]]:
    """Check every waypoint against a fresh reading; print one line each."""
    check = safety.check_target if check_fn is None else check_fn
    tip_of = real_tip_m if tip_fn is None else tip_fn
    target_tips: list[tuple[float, float, float]] = []
    all_ok = True
    for index, waypoint in enumerate(waypoints):
        read_start = time.monotonic()
        current_deg = read_current_deg(group)
        age_ms = (time.monotonic() - read_start) * 1000.0
        current_tip = tip_of(current_deg)
        target_tip = tip_of(list(waypoint))
        target_tips.append(target_tip)
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=tuple(current_deg), reading_age_ms=age_ms,
        )
        result = check(target, current)
        if not result.get("ok"):
            all_ok = False
        tip_mm = (target_tip[0] * 1000.0, target_tip[1] * 1000.0,
                  target_tip[2] * 1000.0)
        out(format_line(index, len(waypoints), waypoint, tip_mm, result))
    return all_ok, target_tips


class WaypointStream:
    """Control-loop callback: stream waypoints with a per-cycle check."""

    def __init__(self, group, waypoints, target_tips, vlim, initial_rad,
                 check_fn, tip_fn):
        self.group = group
        self.waypoints = waypoints
        self.target_tips = target_tips
        self.vlim = [float(v) for v in vlim]
        self.last_commanded_rad = [float(v) for v in initial_rad]
        self.check = safety.check_target if check_fn is None else check_fn
        self.tip_of = real_tip_m if tip_fn is None else tip_fn
        self.index = 0
        self.refusal: dict | None = None
        self.halt = False
        self.settle = 0
        self.done = threading.Event()

    def _send(self, joint_rad: list[float]) -> None:
        try:
            self.group.send_pos_vel([float(v) for v in joint_rad],
                                    vlim=self.vlim)
        except Exception:
            pass

    def _hold(self) -> None:
        # Hold the last commanded position, never a live reading: the
        # SDK reports 0.0 for missing feedback, which must never be sent.
        if self.settle <= 0:
            self.settle = SETTLE_CYCLES
        self._send(self.last_commanded_rad)
        self.settle -= 1
        if self.settle <= 0:
            self.done.set()

    def _refuse(self, reason: str, value: object, limit: object) -> None:
        self.refusal = {"ok": False, "reason": reason,
                        "value": value, "limit": limit}
        self._hold()

    def __call__(self, _sdk: object, _dt: float) -> None:
        read_start = time.monotonic()
        try:
            current_rad = [float(v) for v in self.group.get_positions()]
            if len(current_rad) != 6:
                raise ValueError(f"got {len(current_rad)} positions")
        except Exception as exc:
            self._refuse("read-failed", str(exc), "six joint angles")
            return
        age_ms = (time.monotonic() - read_start) * 1000.0
        current_deg = [math.degrees(v) for v in current_rad]
        if self.refusal is not None or self.halt or self.index >= len(self.waypoints):
            if self.refusal is None and self.halt and self.index < len(self.waypoints):
                self._refuse("user-halt", self.index, len(self.waypoints))
                return
            self._hold()
            return
        waypoint = self.waypoints[self.index]
        target_tip = self.target_tips[self.index]
        try:
            current_tip = self.tip_of(current_deg)
        except Exception as exc:
            self._refuse("read-failed", str(exc), "tool tip")
            return
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=tuple(current_deg), reading_age_ms=age_ms,
        )
        result = self.check(target, current)
        if not result.get("ok"):
            self.refusal = dict(result)
            self._hold()
            return
        commanded = [math.radians(v) for v in waypoint]
        self._send(commanded)
        self.last_commanded_rad = commanded
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done.set()


def format_line(
    index: int,
    count: int,
    joints_deg: tuple[float, ...] | list[float],
    tip_mm: tuple[float, float, float] | None,
    result: dict,
) -> str:
    """One printable line per waypoint for the human log."""
    joints = " ".join(f"{v:+.2f}" for v in joints_deg)
    tip = "unknown" if tip_mm is None else f"{tip_mm[0]:+.1f} {tip_mm[1]:+.1f} {tip_mm[2]:+.1f}"
    if result.get("ok"):
        verdict = "ok"
    else:
        verdict = f"REFUSED {result.get('reason')}"
    return f"wp {index + 1:03d}/{count:03d} joints=[{joints}] tip=[{tip}] {verdict}"
