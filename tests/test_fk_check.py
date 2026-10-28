"""Tests for fk_check.py. Offline only: SDK model math, never the bus."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

FK_PATH = Path(__file__).resolve().parent.parent / "muse_qc" / "fk_check.py"

FORBIDDEN_CALLS = [
    ".enable",
    "set_zero",
    ".send",
    "send_mit",
    "send_pos_vel",
    "send_vel",
    "mode_mit",
    "mode_pos_vel",
    "mode_vel",
    "start_control_loop",
    "estop",
    "reconnect",
]


def _need_sdk_kinematics():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed", allow_module_level=False)


def test_pose_mm_zero_is_finite_and_tip_is_offset_away():
    _need_sdk_kinematics()
    from muse_qc.fk_check import TIP_OFFSET_MM, pose_mm, tip_mm

    pose = pose_mm([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for key in ("x_mm", "y_mm", "z_mm", "roll_deg", "pitch_deg", "yaw_deg"):
        assert math.isfinite(pose[key]), key
    tip = tip_mm(pose)
    dist = math.dist(
        (pose["x_mm"], pose["y_mm"], pose["z_mm"]),
        (tip["x_mm"], tip["y_mm"], tip["z_mm"]),
    )
    assert dist == pytest.approx(TIP_OFFSET_MM, abs=1e-6)
    assert TIP_OFFSET_MM == pytest.approx(30.0)


def test_tip_stays_offset_away_from_end_link_off_home():
    _need_sdk_kinematics()
    from muse_qc.fk_check import TIP_OFFSET_MM, pose_mm, tip_mm

    pose = pose_mm(
        [math.radians(v) for v in (10.0, -20.0, 15.0, 5.0, -10.0, 30.0)]
    )
    tip = tip_mm(pose)
    dist = math.dist(
        (pose["x_mm"], pose["y_mm"], pose["z_mm"]),
        (tip["x_mm"], tip["y_mm"], tip["z_mm"]),
    )
    assert dist == pytest.approx(TIP_OFFSET_MM, abs=1e-6)


def test_pose_mm_rejects_bad_input():
    _need_sdk_kinematics()
    from muse_qc.fk_check import pose_mm

    with pytest.raises(ValueError):
        pose_mm([0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        pose_mm([0.0] * 5 + [float("nan")])


def test_fk_check_never_calls_motion_methods():
    source = FK_PATH.read_text(encoding="utf-8")
    for token in FORBIDDEN_CALLS:
        assert token not in source, token
    assert "muse_qc.arm" in source
    assert "compute_fk" in source or "joint_to_pose" in source
