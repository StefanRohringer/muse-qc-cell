"""Tests for target_pose.py. Offline only: SDK math, never the bus."""

from __future__ import annotations

from pathlib import Path

import pytest

TARGET_PATH = Path(__file__).resolve().parent.parent / "muse_qc" / "target_pose.py"

FORBIDDEN_IMPORTS = [
    "camera",
    "vision_sam",
    "model_check",
    "count_clips",
    "MODEL_API",
]

FORBIDDEN_CALLS = [
    ".enable",
    "send_pos_vel",
    "send_mit",
    "send_vel",
    "mode_pos_vel",
    "start_control_loop",
    "--execute",
]


def _need_sdk_kinematics():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed", allow_module_level=False)


def test_table_to_base_mm_of_base_point():
    from muse_qc.target_pose import table_to_base_mm

    assert table_to_base_mm(495.9, 22.7) == pytest.approx((0.0, 0.0))


def test_table_to_base_mm_straight_ahead():
    from muse_qc.target_pose import table_to_base_mm

    assert table_to_base_mm(495.9, 322.7) == pytest.approx((300.0, 0.0))


def test_table_to_base_mm_to_the_left():
    from muse_qc.target_pose import table_to_base_mm

    assert table_to_base_mm(395.9, 322.7) == pytest.approx((300.0, 100.0))


def test_hover_home_plan_ok(capsys):
    _need_sdk_kinematics()
    from muse_qc.target_pose import main, solve_hover

    solved = solve_hover((495.9, 322.7), 100.0, [0.0] * 6)
    assert solved["ok"] is True
    assert solved["error_mm"] == pytest.approx(0.0, abs=2.0)
    code = main(["--table-mm", "495.9", "322.7", "--start", "home"])
    assert code == 0
    shown = capsys.readouterr().out
    assert "all waypoints ok" in shown
    assert "dry run: no motion" in shown


def test_hover_side_target_keeps_joint6_near_seed():
    _need_sdk_kinematics()
    from muse_qc.target_pose import solve_hover

    solved = solve_hover((395.9, 322.7), 100.0, [0.0] * 6)
    assert solved["ok"] is True
    assert abs(solved["joints_deg"][5]) <= 10.0


def test_far_target_is_unreachable(capsys):
    _need_sdk_kinematics()
    from muse_qc.target_pose import main

    code = main(["--table-mm", "495.9", "2022.7", "--start", "home"])
    assert code == 1
    assert "unreachable" in capsys.readouterr().out


def test_module_imports_no_camera_or_model_client():
    source = TARGET_PATH.read_text(encoding="utf-8")
    for token in FORBIDDEN_IMPORTS + FORBIDDEN_CALLS:
        assert token not in source, token
