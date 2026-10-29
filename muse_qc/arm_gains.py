"""Softer position-loop gains for joints 4 and 5 (runtime yaml copy).

While the arm holds a pose, joints 4 and 5 visibly swing a little:
a limit cycle of the motor position loop (integral term plus
friction/backlash on the small 4310 motors), not a planning error.
The "soft" profile kills the position integral (pos_ki 0.0) and
lowers pos_kp by 30 % for joints 4 and 5 only; the velocity loop
is unchanged. Overrides are written into a copy of the SDK hardware
yaml in a temporary directory (never into the repo or the SDK
package) and passed to RebotArm(hw_yaml=...). The SDK file itself
stays untouched, and a normal run restores the default gains
because mode_pos_vel rewrites the registers on every call.
"""

from __future__ import annotations

import copy
import tempfile
from pathlib import Path

import yaml

from muse_qc.sdk_config import _default_config_dir, _scalar

GLOBAL_CONFIG_NAME = "rebotarm.yaml"
HARDWARE_KEY = "hardware_yaml"
SOFT_POS_KP_FACTOR = 0.7
GAIN_PROFILES = {
    "default": {},
    "soft": {
        4: {"pos_kp_factor": SOFT_POS_KP_FACTOR, "pos_ki": 0.0},
        5: {"pos_kp_factor": SOFT_POS_KP_FACTOR, "pos_ki": 0.0},
    },
}


def _hardware_yaml_path(config_dir=None) -> Path:
    """Return the SDK hardware yaml path (absolute or via rebotarm.yaml)."""
    base = Path(config_dir) if config_dir is not None else None
    if base is None:
        base = _default_config_dir()
    if base is None:
        raise RuntimeError("SDK config folder not found")
    name = _scalar(base / GLOBAL_CONFIG_NAME, HARDWARE_KEY)
    if not name:
        raise RuntimeError(f"{HARDWARE_KEY} not set in {GLOBAL_CONFIG_NAME}")
    if Path(name).is_absolute():
        return Path(name)
    return base / name


def _read_hardware_yaml(config_dir=None):
    """Return (path, parsed dict) of the SDK hardware yaml."""
    path = _hardware_yaml_path(config_dir)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"cannot read hardware config: {exc}")
    if not isinstance(data, dict):
        raise RuntimeError(f"bad hardware config in {path}")
    return path, data


def _apply_profile(data: dict, profile: str):
    """Return (patched copy, describe lines) with the profile overrides."""
    if profile not in GAIN_PROFILES:
        raise ValueError(
            f"unknown gains profile {profile!r},"
            f" expected one of {sorted(GAIN_PROFILES)}"
        )
    patched = copy.deepcopy(data)
    entries = {entry.get("name"): entry for entry in patched.get("joints", [])}
    lines = []
    for joint, change in GAIN_PROFILES[profile].items():
        entry = entries.get(f"joint{joint}")
        if entry is None:
            raise RuntimeError(f"joint{joint} missing in hardware config")
        loop = entry.setdefault("POS_VEL", {})
        old_kp = float(loop.get("pos_kp", 0.0))
        old_ki = float(loop.get("pos_ki", 0.0))
        new_kp = round(old_kp * change["pos_kp_factor"], 6)
        new_ki = float(change["pos_ki"])
        loop["pos_kp"], loop["pos_ki"] = new_kp, new_ki
        lines.append(f"joint {joint}: pos_kp {old_kp:g} -> {new_kp:g},"
                     f" pos_ki {old_ki:.1f} -> {new_ki:.1f}")
    return patched, lines


def write_runtime_yaml(profile: str, out_dir) -> Path | None:
    """Write the hardware yaml with profile overrides into out_dir.

    Returns the absolute path of the copy, or None for "default"
    (the SDK file is used as is).
    """
    if profile == "default":
        return None
    _, data = _read_hardware_yaml()
    patched, _ = _apply_profile(data, profile)
    target = Path(out_dir) / f"rebotarm_{profile}.yaml"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(patched), encoding="utf-8")
    return target.absolute()


def make_arm(profile: str, factory=None):
    """Build the SDK arm object, via a runtime yaml copy when needed."""
    if profile == "default":
        return (factory or _real_arm_factory())()
    runtime = write_runtime_yaml(
        profile, tempfile.mkdtemp(prefix="arm_gains_")
    )
    return (factory or _real_arm_factory())(hw_yaml=str(runtime))


def _real_arm_factory():
    """Return the real RebotArm class. Called only by hand runs."""
    from reBotArm_control_py.actuator import RebotArm

    return RebotArm


def describe(profile: str) -> list[str]:
    """Return one "joint N: pos_kp a -> b, pos_ki c -> d" line per change."""
    if profile == "default":
        return []
    _, data = _read_hardware_yaml()
    _, lines = _apply_profile(data, profile)
    return lines
