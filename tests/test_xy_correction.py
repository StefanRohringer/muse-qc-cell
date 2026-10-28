"""Tests for xy_correction.py and the hover_move wiring. Offline only."""

from __future__ import annotations

import math

from muse_qc import hover_move, xy_correction
from muse_qc.target_pose import base_to_table_mm, table_to_base_mm

REVIEWER_YAW_DEG = (2.4, 2.4, 1.35)
REVIEWER_RADIAL_MM = (18.0, 23.0, 26.0)


def _base_points():
    """(commanded base A, measured base B) per measurement point."""
    points = []
    for (ax, ay), (bx, by) in xy_correction.MEASUREMENTS:
        points.append((table_to_base_mm(ax, ay), table_to_base_mm(bx, by)))
    return points


def test_fit_reproduces_reviewer_yaw():
    fit = xy_correction._fit()
    assert fit["yaw_deg"] > 0.0
    mean_reviewer = sum(REVIEWER_YAW_DEG) / len(REVIEWER_YAW_DEG)
    assert abs(fit["yaw_deg"] - mean_reviewer) < 0.3
    for found, expected in zip(fit["yaw_errs"], REVIEWER_YAW_DEG):
        assert abs(found - expected) < 0.3


def test_fit_reproduces_reviewer_radial_errors():
    fit = xy_correction._fit()
    for cmd_r, expected in zip(fit["cmd_radii"], REVIEWER_RADIAL_MM):
        fitted = fit["slope"] * cmd_r + fit["intercept"]
        assert abs(fitted - expected) < 2.0


def test_correct_round_trips_within_0p1mm():
    targets = [(300.0, 0.0), (250.0, 80.0), (350.0, -100.0), (500.0, 0.0)]
    for (ax, ay), _ in xy_correction.MEASUREMENTS:
        targets.append(table_to_base_mm(ax, ay))
    for x, y in targets:
        cmd = xy_correction.correct_base_target_mm(x, y)
        back = xy_correction.predict_real_mm(*cmd)
        assert math.dist(back, (x, y)) < 0.1


def test_predicted_real_tip_within_5mm_of_measured():
    for cmd, real in _base_points():
        predicted = xy_correction.predict_real_mm(*cmd)
        assert math.dist(predicted, real) < 5.0


def test_axis_target_moves_right_and_closer():
    cmd = xy_correction.correct_base_target_mm(300.0, 0.0)
    assert cmd[1] < 0.0
    assert cmd[0] < 300.0


def test_radial_error_clamped_outside_range():
    fit = xy_correction._fit()
    lo_err = fit["slope"] * fit["r_min"] + fit["intercept"]
    hi_err = fit["slope"] * fit["r_max"] + fit["intercept"]
    assert xy_correction._r_err(0.0, fit) == lo_err
    assert xy_correction._r_err(fit["r_min"] - 50.0, fit) == lo_err
    assert xy_correction._r_err(1000.0, fit) == hi_err
    assert xy_correction._r_err(fit["r_max"] + 50.0, fit) == hi_err


def test_fit_report_lists_yaw_line_and_residuals(capsys):
    assert xy_correction.main([]) == 0
    out = capsys.readouterr().out
    assert "YAW_DEG=" in out
    assert "R_ERR(cmd_r)" in out
    for index in (1, 2, 3):
        assert f"P{index}:" in out and "residual=" in out


TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}
REST_DEG = [-3.33, 0.69, -1.45, 2.72, 0.91, -3.97]


def _recording_solve(seen):
    def fake(table_xy, hover_mm, seed):
        seen.append(tuple(float(v) for v in table_xy))
        return {
            "ok": True,
            "joints_deg": (-5.0, -97.0, -82.0, 75.0, 1.0, -4.0),
            "tip_base_mm": (300.0, 0.0, float(hover_mm)),
            "tip_table_mm": (300.0, 0.0, float(hover_mm)),
            "error_mm": 0.01,
            "tilt_deg": 0.0,
            "iterations": 3,
        }

    return fake


class _FakeGroup:
    """Fake SDK arm group: reads return the rest pose, never a bus."""

    def __init__(self):
        self.calls = []
        self._rad = [math.radians(v) for v in REST_DEG]

    def get_positions(self):
        return list(self._rad)


class _FakeArm:
    """Fake SDK object for dry runs: connect/disconnect only."""

    def __init__(self):
        self.groups = {"arm": _FakeGroup()}

    def connect(self):
        pass

    def disconnect(self):
        pass


def _always_ok(target, current):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _fake_tip(joints_deg):
    return (0.2865, -0.0201, 0.1988)


def test_hover_move_prints_corrected_line(monkeypatch):
    seen = []
    monkeypatch.setattr(hover_move, "solve_hover", _recording_solve(seen))
    lines = []
    code = hover_move.main(
        ["--table-mm", "505.0", "348.7"],
        sdk_factory=_FakeArm,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    corrected = [line for line in lines if line.startswith("corrected base target:")]
    assert len(corrected) == 1
    assert "(xy correction)" in corrected[0]
    assert any(line.startswith("base target:") for line in lines)
    base = table_to_base_mm(505.0, 348.7)
    cmd = xy_correction.correct_base_target_mm(*base)
    assert math.dist(seen[0], base_to_table_mm(*cmd)) < 0.01
    assert math.dist(seen[0], (505.0, 348.7)) > 1.0


def test_hover_move_no_xy_correction_skips_line(monkeypatch):
    seen = []
    monkeypatch.setattr(hover_move, "solve_hover", _recording_solve(seen))
    lines = []
    code = hover_move.main(
        ["--table-mm", "505.0", "348.7", "--no-xy-correction"],
        sdk_factory=_FakeArm,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    assert not any(line.startswith("corrected base target:") for line in lines)
    assert seen[0] == (505.0, 348.7)
