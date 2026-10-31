"""Tests for arm_calibration.py. Offline only: SDK model math, no bus."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

CAL_PATH = Path(__file__).resolve().parent.parent / "muse_qc" / "arm" / "arm_calibration.py"

FORBIDDEN_CALLS = [
    ".enable",
    "send_pos_vel",
    "send_mit",
    "send_vel",
    "start_control_loop",
    "--execute",
    ".env",
    "MODEL_API",
]

HOVER_A = [-0.01, -97.45, -82.52, 75.07, -0.01, 0.05]


def _need_sdk_kinematics():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed", allow_module_level=False)


def test_fit_reproduces_stored_constants():
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    k, rms_k = cal.fit_ratio()
    assert k == pytest.approx(cal.RATIO_K, abs=0.05)
    assert rms_k == pytest.approx(5.45, abs=0.05)
    g, rms_g = cal.fit_load()
    assert g == pytest.approx(cal.LOAD_G, abs=0.1)
    assert rms_g == pytest.approx(6.76, abs=0.05)
    assert cal.CHOSEN_FORM == "ratio"
    assert rms_k < rms_g


def test_fit_report_lists_every_point_with_both_residuals(capsys):
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    lines = cal.fit_report_lines()
    assert len(lines) == len(cal.DATA_POINTS) + 2
    for line, (name, _, _, measured) in zip(lines, cal.DATA_POINTS):
        assert line.startswith(f"{name}: measured={measured:.1f}")
        assert "ratio z=" in line and "load z=" in line
    assert "A1: measured=75.0" in lines[0]
    assert "r=-3.3" in lines[0] and "r=-1.6" in lines[0]
    assert lines[-2].startswith("ratio: k=")
    assert lines[-1].startswith("load: g=")


def test_true_joints_applies_ratio_correction():
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    true = cal.true_joints_deg(HOVER_A)
    assert true[:2] + true[3:] == pytest.approx(tuple(HOVER_A[:2] + HOVER_A[3:]))
    assert true[2] == pytest.approx(HOVER_A[2] + cal.RATIO_K * HOVER_A[2])
    assert true[2] == pytest.approx(-77.93, abs=0.01)
    rest = [-1.83, 0.00, -1.02, 1.67, 1.45, -0.47]
    assert cal.true_joints_deg(rest)[2] == pytest.approx(-1.02, abs=0.1)


def test_tip_table_mm_matches_hand_fit_point():
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    assert cal.link_span_mm() == pytest.approx(155.39, abs=0.1)
    assert cal.tip_beyond_end_mm() == pytest.approx(34.61, abs=0.1)
    tip = cal.tip_table_mm(HOVER_A)
    assert tip == pytest.approx((274.74, 0.0, 71.65), abs=0.1)


def test_zeroed_correction_matches_old_fk_tip(monkeypatch):
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal
    from muse_qc.bringup import fk_check

    monkeypatch.setattr(cal, "RATIO_K", 0.0)
    monkeypatch.setattr(cal, "LOAD_G", 0.0)
    monkeypatch.setattr(cal, "BASE_Z_MM", 0.0)
    monkeypatch.setattr(cal, "TOOL_LENGTH_MM", 185.0)
    poses = (
        HOVER_A,
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [-0.01, -80.01, -65.49, 75.66, -0.01, -0.01],
    )
    for pose in poses:
        assert cal.true_joints_deg(pose) == pytest.approx(tuple(pose))
        tip = cal.tip_table_mm(pose)
        old_pose = fk_check.pose_mm([math.radians(v) for v in pose])
        old = fk_check.tip_mm(old_pose)
        assert math.dist(tip, (old["x_mm"], old["y_mm"], old["z_mm"])) < 0.5


def test_solve_corrected_converges_at_100mm():
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    for base_xy, height in (((300.0, 0.0), 100.0), ((220.0, 0.0), 100.0)):
        solved = cal.solve_corrected(base_xy, height, [0.0] * 6)
        assert solved["ok"] is True
        assert solved["error_mm"] < 0.5
        assert solved["iterations"] <= 10
        tip = solved["tip_table_mm"]
        assert math.dist(tip, (base_xy[0], base_xy[1], height)) < 0.5
        assert len(solved["joints_deg"]) == 6


def test_solve_corrected_150mm_fails_cleanly():
    # The corrected 150 mm hover at base x 300 is outside the tool-down
    # workspace: it needs the nominal tip near (325, 0, 176), but the
    # nominal IK stops at z 162 there (joint 4 hits its 89.95 deg limit;
    # 0/500 random-seed solves above it). The real arm agrees: the human
    # measured 130 mm for the nominal 150 mm hover on 2026-10-28.
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    solved = cal.solve_corrected((300.0, 0.0), 150.0, [0.0] * 6)
    assert solved["ok"] is False
    assert "unreachable" in solved["reason"]


def test_solve_corrected_rejects_bad_input():
    _need_sdk_kinematics()
    from muse_qc.arm import arm_calibration as cal

    assert cal.solve_corrected((300.0, 0.0), 100.0, [0.0] * 5)["ok"] is False
    far = cal.solve_corrected((300.0, 0.0), 2000.0, [0.0] * 6)
    assert far["ok"] is False


def test_solve_hover_puts_corrected_tip_on_target(capsys):
    _need_sdk_kinematics()
    from muse_qc.arm.target_pose import base_to_table_mm, run, solve_hover

    solved = solve_hover((495.9, 322.7), 100.0, [0.0] * 6)
    assert solved["ok"] is True
    assert solved["error_mm"] < 0.5
    assert solved["tilt_deg"] <= 2.0
    table = base_to_table_mm(*solved["tip_table_mm"][:2])
    assert math.dist((table[0], table[1], solved["tip_table_mm"][2]),
                     (495.9, 322.7, 100.0)) < 0.5
    code = run((495.9, 322.7), 100.0, [0.0] * 6)
    assert code == 0
    assert "corrected tip table mm:" in capsys.readouterr().out


def test_arm_calibration_never_calls_motion():
    source = CAL_PATH.read_text(encoding="utf-8")
    for token in FORBIDDEN_CALLS:
        assert token not in source, token
