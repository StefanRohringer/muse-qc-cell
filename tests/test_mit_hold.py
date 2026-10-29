"""Tests for the mit-ff hold mode. Offline only: fakes, no SDK bus."""

from __future__ import annotations

import math
import re

import pytest

from muse_qc import hold_test, hover_move, mit_hold
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.mit_hold import (
    MIT_KD,
    MIT_KP,
    TAU_SCALE,
    MitHoldStream,
    gravity_lines,
    gravity_torque_Nm,
)

REST_DEG = [-3.33, 0.69, -1.45, 2.72, 0.91, -3.97]
GOAL_DEG = [-5.0, -97.0, -82.0, 75.0, 1.0, -4.0]
VLIMS = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}


class FakeMitGroup:
    """Fake SDK group: fixed or following readings, MIT and POS_VEL."""

    def __init__(self, joints_deg, follow=True):
        self._rad = [math.radians(v) for v in joints_deg]
        self._follow = follow
        self.calls = []

    def get_positions(self):
        self.calls.append("get")
        return list(self._rad)

    def mode_pos_vel(self, vlim=None):
        self.calls.append(("mode_posvel", [float(v) for v in vlim]))

    def mode_mit(self, kp=None, kd=None):
        self.calls.append("mode_mit")

    def enable(self):
        self.calls.append("enable")

    def disable(self):
        self.calls.append("disable")

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send_posvel", values, [float(v) for v in vlim]))
        if self._follow:
            self._rad = list(values)

    def send_mit(self, pos, vel=None, kp=None, kd=None, tau=None):
        values = [float(v) for v in pos]
        self.calls.append((
            "send_mit", values,
            [float(v) for v in (vel if vel is not None else [])],
            [float(v) for v in (kp if kp is not None else [])],
            [float(v) for v in (kd if kd is not None else [])],
            [float(v) for v in (tau if tau is not None else [])],
        ))
        if self._follow:
            self._rad = list(values)


class FakeMitArm:
    """Fake SDK object: control loop runs inline, never touches a bus."""

    def __init__(self, joints_deg, follow=True, with_gripper=False):
        self.groups = {"arm": FakeMitGroup(joints_deg, follow)}
        if with_gripper:
            self.groups["gripper"] = FakeMitGroup([2.8], follow)
        self.calls = []

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def start_control_loop(self, fn, rate=None):
        self.calls.append("loop")
        for _ in range(60000):
            fn(self, 0.01)
            if fn.done.is_set():
                break

    def stop_control_loop(self):
        self.calls.append("stop")


def _always_ok(target, current):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _fake_tip(joints_deg):
    return (0.2865, -0.0201, 0.1988)


def _fake_solve_ok(table_xy, hover_mm, seed):
    return {
        "ok": True,
        "joints_deg": tuple(GOAL_DEG),
        "tip_base_mm": (300.0, 0.0, float(hover_mm)),
        "tip_table_mm": (300.0, 0.0, float(hover_mm)),
        "error_mm": 0.01,
        "tilt_deg": 0.0,
        "iterations": 3,
    }


def _q_gravity(joint_rad):
    """Fake gravity: torque follows the first commanded joint."""
    return [2.0 * float(joint_rad[0])] + [0.0] * 5


def _sends(group, name):
    return [call for call in group.calls if call[0] == name]


def test_mit_gains_match_dm_config():
    assert tuple(MIT_KP) == (120.0, 120.0, 120.0, 18.0, 18.0, 18.0)
    assert tuple(MIT_KD) == (8.0, 8.0, 8.0, 2.0, 2.0, 2.0)
    assert TAU_SCALE == 1.0


def test_mit_execute_sends_only_mit_with_gains_and_tau(monkeypatch, tmp_path):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    monkeypatch.chdir(tmp_path)
    arm = FakeMitArm(REST_DEG, follow=True)
    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-s", "5",
         "--hold-mode", "mit-ff", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS,
        gravity_fn=_q_gravity, out=lines.append,
    )
    assert code == 0
    group = arm.groups["arm"]
    assert group.calls.index("mode_mit") < group.calls.index("enable")
    assert _sends(group, "mode_posvel") == []
    assert _sends(group, "send_posvel") == []
    sends = _sends(group, "send_mit")
    assert sends, "arm commands must go through send_mit"
    for _, _pos, _vel, kp, kd, tau in sends:
        assert kp == pytest.approx(list(MIT_KP))
        assert kd == pytest.approx(list(MIT_KD))
    for _, pos, _vel, _kp, _kd, tau in sends:
        assert tau[0] == pytest.approx(TAU_SCALE * 2.0 * pos[0])
        assert tau[1:] == pytest.approx([0.0] * 5)
    assert len({round(call[5][0], 9) for call in sends}) > 1
    text = "\n".join(str(line) for line in lines)
    assert "hold mode: mit-ff" in text
    assert "joint 4: mean err" in text
    files = sorted((tmp_path / "recordings").glob("hold_mit-ff_*.csv"))
    assert len(files) == 1
    assert files[0].name in text


def test_mit_first_command_is_measured_start_pose(monkeypatch, tmp_path):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    monkeypatch.chdir(tmp_path)
    arm = FakeMitArm(REST_DEG, follow=True)
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-s", "5",
         "--hold-mode", "mit-ff", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS,
        gravity_fn=_q_gravity, out=lambda line: None,
    )
    assert code == 0
    first = _sends(arm.groups["arm"], "send_mit")[0]
    assert first[1] == pytest.approx([math.radians(v) for v in REST_DEG])


def test_posvel_default_never_touches_mit(monkeypatch, tmp_path):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    monkeypatch.chdir(tmp_path)
    arm = FakeMitArm(REST_DEG, follow=True)
    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-s", "5", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    group = arm.groups["arm"]
    assert "mode_mit" not in group.calls
    assert _sends(group, "send_mit") == []
    assert _sends(group, "send_posvel"), "posvel must keep sending"
    text = "\n".join(str(line) for line in lines)
    assert "hold mode: posvel" in text
    assert len(list((tmp_path / "recordings").glob("hold_posvel_*.csv"))) == 1


def _direct_stream(follow_group, gravity_fn, errors):
    waypoints = [tuple(GOAL_DEG)] * 30
    tips = [(0.3, 0.0, 0.2)] * 30
    stream = MitHoldStream(
        follow_group, waypoints, tips, VLIMS,
        [math.radians(v) for v in REST_DEG],
        _always_ok, _fake_tip,
        segments=[hover_move.SEG_HOVER] * 30,
        gravity_fn=gravity_fn, on_tracking_error=errors.append,
    )
    return stream


def test_tracking_guard_needs_ten_bad_cycles_in_a_row():
    group = FakeMitGroup(REST_DEG, follow=False)
    errors = []
    stream = _direct_stream(group, _q_gravity, errors)
    bad = [GOAL_DEG[0] + 10.0] + GOAL_DEG[1:]
    good = list(GOAL_DEG)
    for _ in range(9):
        stream.on_sample(bad)
    assert stream.refusal is None
    assert stream.tracking_error is None
    stream.on_sample(good)  # one good cycle resets the count
    for _ in range(9):
        stream.on_sample(bad)
    assert stream.refusal is None
    assert stream.tracking_error is None
    stream.on_sample(bad)  # tenth bad cycle in a row trips the guard
    assert stream.tracking_error == {"joint": 1}
    assert stream.refusal["reason"] == "tracking-error"
    assert errors == [1]


def test_tracking_guard_holds_and_disables_only_in_finally(
        monkeypatch, tmp_path):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    monkeypatch.chdir(tmp_path)
    arm = FakeMitArm(REST_DEG, follow=False)  # readings never follow
    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-s", "5",
         "--hold-mode", "mit-ff", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS,
        gravity_fn=_q_gravity, out=lines.append,
    )
    assert code == 1
    group = arm.groups["arm"]
    text = "\n".join(str(line) for line in lines)
    assert re.search(r"tracking error on joint \d+, holding", text)
    assert "tracking-error" in text
    assert "hold: no samples" in text
    assert "disable" not in group.calls
    assert "disconnect" in arm.calls
    assert _sends(group, "send_posvel") == []
    last = _sends(group, "send_mit")[-1]
    assert last[1] == pytest.approx([math.radians(v) for v in REST_DEG])


def test_mit_dry_run_prints_gains_and_gravity(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeMitArm(REST_DEG, follow=False)
    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-mode", "mit-ff"],
        sdk_factory=lambda: arm,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS,
        gravity_fn=lambda q: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        out=lines.append,
    )
    assert code == 0
    text = "\n".join(str(line) for line in lines)
    assert "hold mode: mit-ff" in text
    assert "mit kp: 120 120 120 18 18 18" in text
    assert "mit kd: 8 8 8 2 2 2" in text
    assert "tau scale: 1.0" in text
    for joint, value in enumerate([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], start=1):
        assert f"joint {joint} gravity torque: {value:+.3f} N*m" in text
    assert "enable" not in arm.groups["arm"].calls
    assert _sends(arm.groups["arm"], "send_mit") == []


def test_gripper_group_untouched_in_mit_mode(monkeypatch, tmp_path):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    monkeypatch.chdir(tmp_path)
    arm = FakeMitArm(REST_DEG, follow=True, with_gripper=True)
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-s", "5",
         "--hold-mode", "mit-ff", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS,
        gravity_fn=_q_gravity, out=lambda line: None,
    )
    assert code == 0
    assert arm.groups["gripper"].calls == []


def test_gravity_torque_rejects_bad_input_without_sdk():
    with pytest.raises(ValueError):
        gravity_torque_Nm([0.0] * 5)
    with pytest.raises(ValueError):
        gravity_torque_Nm([float("inf")] * 6)


def test_gravity_lines_format_with_fake_function():
    lines = gravity_lines(GOAL_DEG, gravity_fn=lambda q: [0.5] * 6)
    assert len(lines) == 6
    assert lines[0] == "joint 1 gravity torque: +0.500 N*m"
    assert lines[5] == "joint 6 gravity torque: +0.500 N*m"
