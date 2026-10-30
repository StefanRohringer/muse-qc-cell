"""Tests for the grip yaw from the lever direction. Offline, fakes only.

Pure math in muse_qc.grip_yaw, the --lever-deg wiring in
muse_qc.pick_test (fake solver, stubbed arm read and sensor) and
the --lever-deg flag in agent_tools.real_sort (fake child). No bus,
no camera, no network.
"""

from __future__ import annotations

import sys

import pytest

from muse_qc import agent_tools, pick_test
from muse_qc.grip_yaw import MAX_GRIP_YAW_DEG, grip_yaw_offset_deg

STRAIGHT = (495.9, 322.7)  # base (300, 0): radial yaw 0, radial is table +y
HUMAN_STRAIGHT = (506.1, 346.3)  # human 2026-10-30 straight part
REST = [-3.33, -99.0, -79.0, 74.0, 0.91, -3.97]
REST_GRIP = -4.97
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}


def test_max_grip_yaw_is_70():
    assert MAX_GRIP_YAW_DEG == 70.0


def test_straight_part_lever_along_radial_is_zero():
    offset, clamped = grip_yaw_offset_deg(STRAIGHT, 90.0)
    assert offset == pytest.approx(0.0, abs=1e-9)
    assert clamped is False


def test_lever_90_deg_off_clamps_to_cable_limit():
    for lever in (0.0, 180.0):
        offset, clamped = grip_yaw_offset_deg(STRAIGHT, lever)
        assert offset == pytest.approx(-70.0)
        assert clamped is True
    offset, clamped = grip_yaw_offset_deg(STRAIGHT, 90.0 + 90.0)
    assert clamped is True
    assert abs(offset) == pytest.approx(70.0)


def test_lever_30_deg_off_matches_left_right_sign():
    offset, clamped = grip_yaw_offset_deg(STRAIGHT, 120.0)
    assert offset == pytest.approx(30.0)
    assert clamped is False
    mirror, mirror_clamped = grip_yaw_offset_deg(STRAIGHT, 60.0)
    assert mirror == pytest.approx(-30.0)
    assert mirror_clamped is False


def test_180_deg_turned_lever_gives_same_offset():
    first = grip_yaw_offset_deg(HUMAN_STRAIGHT, 31.9)
    second = grip_yaw_offset_deg(HUMAN_STRAIGHT, 31.9 + 180.0)
    assert first == second


def test_human_straight_part_near_zero():
    offset, clamped = grip_yaw_offset_deg(HUMAN_STRAIGHT, 89.8)
    assert abs(offset) < 5.0
    assert clamped is False


def test_bad_inputs_raise_value_error():
    with pytest.raises(ValueError):
        grip_yaw_offset_deg(("x", 1.0), 90.0)
    with pytest.raises(ValueError):
        grip_yaw_offset_deg(STRAIGHT, float("nan"))


def _need_sdk_kinematics():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed")


def test_nominal_offset_zero_matches_default():
    _need_sdk_kinematics()
    from muse_qc.target_pose import _solve_nominal

    seed = [0.0] * 6
    default = _solve_nominal(STRAIGHT, 100.0, seed)
    explicit = _solve_nominal(STRAIGHT, 100.0, seed, yaw_offset_deg=0.0)
    assert default["ok"] is True and explicit["ok"] is True
    assert default["joints_deg"] == pytest.approx(explicit["joints_deg"])


def test_nominal_offset_turns_the_tool():
    _need_sdk_kinematics()
    from muse_qc.target_pose import _solve_nominal

    seed = [0.0] * 6
    plain = _solve_nominal(STRAIGHT, 100.0, seed)
    turned = _solve_nominal(STRAIGHT, 100.0, seed, yaw_offset_deg=30.0)
    assert plain["ok"] is True and turned["ok"] is True
    peak = max(abs(a - b) for a, b in
               zip(plain["joints_deg"], turned["joints_deg"]))
    assert peak > 0.5


class _StubReader:
    def disconnect(self):
        pass


class _StubSensor:
    def close(self):
        pass


def _run_cli(monkeypatch, extra_args, solve_fn):
    monkeypatch.setattr(pick_test, "_read_start",
                        lambda sdk: (_StubReader(), list(REST), REST_GRIP))
    monkeypatch.setattr(pick_test, "open_touch_sensor",
                        lambda *args, **kwargs: _StubSensor())
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel"]
        + extra_args,
        sdk_factory=lambda **kwargs: object(),
        check_fn=lambda target, current, **kw: {"ok": True},
        tip_fn=lambda joints: (0.30, 0.0, 0.05),
        solve_fn=solve_fn,
        joint_limits=TEST_LIMITS,
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: _StubSensor(),
        out=lines.append,
    )
    return code, "\n".join(lines)


def _recording_solve(seen):
    def solve(table_xy, height_mm, seed, yaw_offset_deg=0.0):
        seen.append(float(yaw_offset_deg))
        height = float(height_mm)
        return {
            "ok": True,
            "joints_deg": (-3.0, -90.0 - 0.1 * height, -80.0,
                           75.0, 1.0, -4.0),
            "tip_base_mm": (300.0, 0.0, height),
            "tip_table_mm": (300.0, 0.0, height),
            "error_mm": 0.0,
            "tilt_deg": 0.0,
            "iterations": 1,
        }

    return solve


def test_lever_deg_prints_grip_yaw_and_passes_offset(monkeypatch):
    seen = []
    code, text = _run_cli(monkeypatch, ["--lever-deg", "120"],
                          _recording_solve(seen))
    assert code == 0
    assert "grip yaw: lever 120.0 deg, offset +30.0 deg" in text
    assert "(clamped)" not in text
    assert seen and all(value == pytest.approx(30.0) for value in seen)
    assert "segments at 100 Hz:" in text


def test_lever_deg_clamped_prints_clamped(monkeypatch):
    seen = []
    code, text = _run_cli(monkeypatch, ["--lever-deg", "0"],
                          _recording_solve(seen))
    assert code == 0
    assert "grip yaw: lever 0.0 deg, offset -70.0 deg (clamped)" in text
    assert seen and all(value == pytest.approx(-70.0) for value in seen)


def test_without_lever_deg_nothing_changes(monkeypatch):
    calls = []

    def solve(table_xy, height_mm, seed):
        calls.append(True)
        return _recording_solve([])(table_xy, height_mm, seed)

    code, text = _run_cli(monkeypatch, [], solve)
    assert code == 0
    assert "grip yaw:" not in text
    assert calls


def test_lever_deg_range_rejected(monkeypatch):
    with pytest.raises(SystemExit) as info:
        pick_test.main(["--table-mm", "495.9", "322.7",
                        "--lever-deg", "180.1"],
                       sdk_factory=lambda **kwargs: object())
    assert info.value.code == 2


class _FakeStdin:
    def __init__(self):
        self.written = []

    def write(self, text):
        self.written.append(text)

    def flush(self):
        pass

    def close(self):
        pass


class _FakeProc:
    def __init__(self, lines):
        self._lines = list(lines)
        self.stdin = _FakeStdin()
        self.stdout = self
        self.cmds = []

    def readline(self):
        if self._lines:
            return self._lines.pop(0) + "\n"
        return ""

    def __iter__(self):
        while True:
            line = self.readline()
            if line == "":
                return
            yield line

    def wait(self):
        return 0


def _child_result(lever):
    made = []

    def factory(cmd):
        made.append(cmd)
        return _FakeProc([
            "inspect: levers 3 verdict good",
            "dropped into the good box",
            "recordings/count_child_000001.json",
        ])

    result = agent_tools.real_sort([478.7, 344.7], lever,
                                   popen_factory=factory)
    assert result["outcome"] == "sorted"
    return made[0]


def test_real_sort_adds_lever_deg_only_when_not_none():
    with_lever = _child_result(31.9)
    assert "--lever-deg" in with_lever
    assert with_lever[with_lever.index("--lever-deg") + 1] == "31.9"
    without = _child_result(None)
    assert "--lever-deg" not in without
    assert without[:3] == [sys.executable, "-m", "muse_qc.pick_test"]


def test_run_state_keeps_lever_deg_per_id():
    from muse_qc.agent_tools import RunState

    state = RunState()
    state.new_scan([[500.0, 360.0], [540.0, 330.0]], [89.8, None])
    assert state.lookup(0) == [500.0, 360.0]
    assert state.lever_of(0) == pytest.approx(89.8)
    assert state.lever_of(1) is None
    assert state.lever_of(99) is None
    state.remove_sorted(0)
    assert state.lookup(0) is None
    assert state.lever_of(0) is None
    assert state.lookup(1) == [540.0, 330.0]
    assert state.lever_of(1) is None
