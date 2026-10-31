"""Shared CLI ranges and SDK helpers for the pick entry points.

Moved out of muse_qc.pick.pick_test so that module stays within the
400 line limit. Pure argument validation plus the read-only arm
reader and the SDK factory (both move nothing by themselves).
"""

from __future__ import annotations

import argparse

GRIP_Z_MM = (5.0, 60.0, 8.0)
OPEN_MM = (25.0, 50.0, 28.0)
HOLD_S = (0.5, 30.0, 1.0)
SPEED_DEG_S = (2.0, 80.0, 50.0)
ACCEL_DEG_S2 = (15.0, 160.0, 100.0)
LEVER_DEG = (0.0, 180.0)
POSVEL_SPEED_MAX_DEG_S = 15.0


def _ranged(text, name, low, high, unit):
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{name} must be a number, got {text!r}.")
    if not low <= value <= high:
        raise argparse.ArgumentTypeError(
            f"{name} must be between {low:g} and {high:g} {unit}, got {text!r}.")
    return value


def _grip_z_type(text): return _ranged(text, "--grip-z-mm", *GRIP_Z_MM[:2], "mm")


def _open_mm_type(text): return _ranged(text, "--open-mm", *OPEN_MM[:2], "mm")


def _hold_s_type(text): return _ranged(text, "--hold-s", *HOLD_S[:2], "s")


def _speed_type(text): return _ranged(text, "--speed-deg-s", *SPEED_DEG_S[:2], "deg/s")


def _accel_type(text): return _ranged(text, "--accel-deg-s2", *ACCEL_DEG_S2[:2], "deg/s2")


def _lever_deg_type(text):
    return _ranged(text, "--lever-deg", *LEVER_DEG, "deg")


def _read_start(sdk):
    from muse_qc.arm.arm import Arm

    reader = Arm(sdk_factory=lambda: sdk)
    reader.connect()
    joints = reader.read_joints()
    return reader, [j.degrees for j in joints], reader.read_gripper().degrees


def _create_sdk():
    from reBotArm_control_py.actuator import RebotArm

    return RebotArm()
