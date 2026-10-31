"""Tests for hold_test.py. Offline only: fakes, no SDK bus."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import pytest

from muse_qc.bringup import hold_test, hover_move
from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ
from muse_qc.bringup.hold_test import HoldStream, hold_lines, swing_hz

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


class FakeGroup:
    """Fake SDK arm group: reads return last commanded position."""

    def __init__(self, joints_deg):
        self._rad = [math.radians(v) for v in joints_deg]
        self.calls = []

    def get_positions(self):
        self.calls.append("get")
        return list(self._rad)

    def mode_pos_vel(self, vlim=None):
        self.calls.append(("mode", [float(v) for v in vlim]))

    def enable(self):
        self.calls.append("enable")

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send", values, [float(v) for v in vlim]))
        self._rad = list(values)


class FakeArm:
    """Fake SDK object: control loop runs inline, never touches a bus."""

    def __init__(self, joints_deg):
        self.groups = {"arm": FakeGroup(joints_deg)}
        self.calls = []
        self.loop_rate = None

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def start_control_loop(self, fn, rate=None):
        self.calls.append("loop")
        self.loop_rate = rate
        for _ in range(60000):
            fn(self, 0.01)
            if getattr(fn, "done", None) is not None and fn.done.is_set():
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


def _touched(group):
    return any(call in ("enable", "loop") or call[0] in ("mode", "send")
               for call in group.calls)


def _sine(errors_hz=2.0, amplitude=0.3, seconds=20.0):
    count = int(seconds * LOOP_RATE_HZ)
    return [
        amplitude * math.sin(2.0 * math.pi * errors_hz * index / LOOP_RATE_HZ)
        for index in range(count)
    ]


def test_swing_hz_finds_2hz_sine():
    assert swing_hz(_sine(), LOOP_RATE_HZ) == pytest.approx(2.0, abs=0.05)


def test_swing_hz_none_for_constant():
    assert swing_hz([0.05] * 2000, LOOP_RATE_HZ) is None
    assert swing_hz([0.05], LOOP_RATE_HZ) is None


def test_hold_lines_sine_and_constant():
    goal = [0.0] * 6
    samples = [[value] * 6 for value in _sine()]
    lines = hold_lines(samples, goal)
    assert "joint 1:" in lines[0] and "swing" not in lines[0]
    assert "p2p 0.60 deg" in lines[0]
    assert "swing 2.0 Hz" in lines[3]
    assert "swing 2.0 Hz" in lines[4]
    assert "swing" not in lines[5]
    flat = hold_lines([[0.05] * 6] * 2000, goal)
    assert "swing n/a" in flat[3]
    assert "swing n/a" in flat[4]


def test_dry_run_never_enables_or_sends(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7"],
        sdk_factory=lambda: arm,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    assert any(line.startswith("table target:") for line in lines)
    assert any(line.startswith("plan:") for line in lines)
    assert not _touched(arm.groups["arm"])


@pytest.mark.parametrize("hold_s", ["4.9", "60.1", "abc"])
def test_hold_s_out_of_range_refused_without_connect(hold_s):
    def factory():
        raise AssertionError("must not connect on a bad --hold-s")

    with pytest.raises(SystemExit) as exc:
        hold_test.main(
            ["--table-mm", "495.9", "322.7", "--hold-s", hold_s],
            sdk_factory=factory,
            out=lambda line: None,
        )
    assert exc.value.code == 2


def test_execute_reports_hold_stats_and_saves_csv(monkeypatch, tmp_path):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    monkeypatch.chdir(tmp_path)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--hold-s", "5", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    text = "\n".join(str(line) for line in lines)
    assert "joint 4: mean err" in text
    assert "p2p 0.00 deg" in text
    assert "swing n/a" in text
    files = sorted((tmp_path / "recordings").glob("hold_*.csv"))
    assert len(files) == 1
    assert files[0].name in text
    with open(files[0], newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    assert rows[0] == ["t_s"] + [f"joint{joint}_deg" for joint in range(1, 7)]
    assert len(rows) == 1 + 5 * int(LOOP_RATE_HZ)
    assert all(len(row) == 7 for row in rows[1:])


def test_gains_stiff_passes_hw_yaml_to_factory(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return arm

    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--gains", "stiff"],
        sdk_factory=factory,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    (kwargs,) = seen
    runtime = Path(kwargs["hw_yaml"])
    assert runtime.is_file()
    text = "\n".join(str(line) for line in lines)
    assert "joint 4: pos_kp" in text
    assert "vel_kp" in text


def test_gains_vel2_passes_hw_yaml_to_factory(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return arm

    lines = []
    code = hold_test.main(
        ["--table-mm", "495.9", "322.7", "--gains", "vel2"],
        sdk_factory=factory,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    (kwargs,) = seen
    runtime = Path(kwargs["hw_yaml"])
    assert runtime.is_file()
    text = "\n".join(str(line) for line in lines)
    assert "joint 4: vel_kp 0.0008 -> 0.0016" in text
    assert "joint 5: vel_kp 0.0008 -> 0.0016" in text


def test_hold_stream_keeps_only_hold_samples():
    group = FakeGroup(REST_DEG)
    waypoints = [tuple(REST_DEG), tuple(GOAL_DEG), tuple(GOAL_DEG)]
    tips = [(0.3, 0.0, 0.2)] * 3
    stream = HoldStream(
        group, waypoints, tips, VLIMS, [math.radians(v) for v in REST_DEG],
        _always_ok, _fake_tip,
        segments=[hover_move.SEG_HOVER, hover_move.SEG_HOLD, hover_move.SEG_HOLD],
    )
    stream(group, 0.01)
    stream(group, 0.01)
    stream(group, 0.01)
    assert len(stream.hold_samples) == 2
    assert stream.hold_goal == list(GOAL_DEG)
