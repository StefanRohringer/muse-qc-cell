"""Gain profiles for joints 4 and 5 (runtime yaml copy).

While the arm holds a pose, joints 4 and 5 visibly swing a little.
The "soft" profile (pos_ki 0.0, pos_kp 30 % lower for joints 4
and 5 only) was tried in an A/B on 2026-10-29 and was slightly
worse than default, so default stays. A hand push test the same
day (about 5 mm sideways on the finger tips during the hold)
showed the wrist moving along with the push while the encoders
saw it (joint 5 p2p 0.15 deg without push, 1.77 deg with push),
so the swing is mostly servo compliance of joints 4/5 (too soft
position/velocity loop: vel_kp 0.0008 on joints 4/5 versus
0.0125 on joints 1 to 3), not only play behind the encoder.
The "stiff" profile raises pos_kp by 30 % and vel_kp by 50 %
for joints 4 and 5 only; the integral terms are unchanged.
Stiff (pos_kp x1.3, vel_kp x1.5) was worse in the 2026-10-29 push
test, so "vel2" tests the velocity loop alone (vel_kp x2.0 for
joints 4 and 5 only, pos_kp, pos_ki and vel_ki unchanged).
Vel2 without push was clearly calmer than default on 2026-10-29
(joint 4 p2p 0.55 vs 1.07 deg, no swing frequency any more), so
"vel3" is the next A/B step (vel_kp x3.0 for joints 4 and 5 only,
nothing else changed).
Overrides are written into a copy of the SDK hardware
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
STIFF_POS_KP_FACTOR = 1.3
STIFF_VEL_KP_FACTOR = 1.5
VEL2_VEL_KP_FACTOR = 2.0
VEL3_VEL_KP_FACTOR = 3.0
GAIN_PROFILES = {
    "default": {},
    "soft": {
        4: {"pos_kp_factor": SOFT_POS_KP_FACTOR, "pos_ki": 0.0},
        5: {"pos_kp_factor": SOFT_POS_KP_FACTOR, "pos_ki": 0.0},
    },
    "stiff": {
        4: {"pos_kp_factor": STIFF_POS_KP_FACTOR,
            "vel_kp_factor": STIFF_VEL_KP_FACTOR},
        5: {"pos_kp_factor": STIFF_POS_KP_FACTOR,
            "vel_kp_factor": STIFF_VEL_KP_FACTOR},
    },
    "vel2": {
        4: {"vel_kp_factor": VEL2_VEL_KP_FACTOR},
        5: {"vel_kp_factor": VEL2_VEL_KP_FACTOR},
    },
    "vel3": {
        4: {"vel_kp_factor": VEL3_VEL_KP_FACTOR},
        5: {"vel_kp_factor": VEL3_VEL_KP_FACTOR},
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
        parts = []
        if "pos_kp_factor" in change:
            old_kp = float(loop.get("pos_kp", 0.0))
            new_kp = round(old_kp * change["pos_kp_factor"], 6)
            loop["pos_kp"] = new_kp
            parts.append(f"pos_kp {old_kp:g} -> {new_kp:g}")
        if "vel_kp_factor" in change:
            old_vkp = float(loop.get("vel_kp", 0.0))
            new_vkp = round(old_vkp * change["vel_kp_factor"], 6)
            loop["vel_kp"] = new_vkp
            parts.append(f"vel_kp {old_vkp:g} -> {new_vkp:g}")
        if "pos_ki" in change:
            old_ki = float(loop.get("pos_ki", 0.0))
            new_ki = float(change["pos_ki"])
            loop["pos_ki"] = new_ki
            parts.append(f"pos_ki {old_ki:.1f} -> {new_ki:.1f}")
        if "vel_ki" in change:
            old_vki = float(loop.get("vel_ki", 0.0))
            new_vki = float(change["vel_ki"])
            loop["vel_ki"] = new_vki
            parts.append(f"vel_ki {old_vki:.1f} -> {new_vki:.1f}")
        lines.append(f"joint {joint}: " + ", ".join(parts))
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
    """Return one "joint N: ..." line per changed joint."""
    if profile == "default":
        return []
    _, data = _read_hardware_yaml()
    _, lines = _apply_profile(data, profile)
    return lines
