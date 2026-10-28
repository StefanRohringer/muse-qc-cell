"""Tests for first_move.py. Offline only: fake arm, no SDK bus."""

from __future__ import annotations

import math

import pytest

from muse_qc import first_move, safety
from muse_qc.first_move_plan import (
    STEP_DEG,
    build_plan,
    ramp_waypoints,
    read_sdk_vlims,
    scale_vlim,
)

REST_DEG = [-3.33, 1.58, -1.45, 2.72, 0.91, -3.97]
REST_TIP_M = (0.2865, -0.0201, 0.1988)
VLIMS = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]


class FakeGroup:
    """Fake SDK arm group: reads return last commanded position."""

    def __init__(self, joints_deg):
        self._rad = [math.radians(v) for v in joints_deg]
        self.calls = []

    @property
    def joint_names(self):
        return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

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


class FlakyGroup(FakeGroup):
    """After a few sends one joint reads exactly 0.0 (missing feedback)."""

    CORRUPT_AFTER = 5

    def __init__(self, joints_deg):
        super().__init__(joints_deg)
        self.sent_count = 0

    def send_pos_vel(self, pos, vlim):
        super().send_pos_vel(pos, vlim)
        self.sent_count += 1

    def get_positions(self):
        values = super().get_positions()
        if self.sent_count >= self.CORRUPT_AFTER:
            values = list(values)
            values[2] = 0.0
        return values


class FakeArm:
    """Fake SDK object: control loop runs inline, never touches a bus."""

    def __init__(self, joints_deg, group_cls=FakeGroup):
        self.groups = {"arm": group_cls(joints_deg)}
        self.calls = []
        self.loop_rate = None

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def start_control_loop(self, fn, rate=None):
        self.calls.append("loop")
        self.loop_rate = rate
        for _ in range(10000):
            fn(self, 0.02)
            if getattr(fn, "done", None) is not None and fn.done.is_set():
                break

    def stop_control_loop(self):
        self.calls.append("stop")


def _always_ok(target, current):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _fake_tip(joints_deg):
    return REST_TIP_M


def _sends(group):
    return [call[1] for call in group.calls if call[0] == "send"]


def test_dry_run_never_modes_enables_loops_or_sends():
    arm = FakeArm(REST_DEG)
    lines = []
    before = safety.J6_REF
    assert first_move.dry_run(arm, check_fn=_always_ok, tip_fn=_fake_tip,
                              out=lines.append) == 0
    assert "connect" in arm.calls and "disconnect" in arm.calls
    assert not any(call in ("enable", "loop") or call[0] in ("mode", "send")
                   for call in arm.groups["arm"].calls)
    assert len(lines) > 200  # one line per waypoint
    assert safety.J6_REF == before


def test_dry_run_passes_real_safety():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    rest_ok = [-3.33, -1.58, -1.45, 2.72, 0.91, -3.97]
    arm = FakeArm(rest_ok)
    assert first_move.dry_run(arm, tip_fn=_fake_tip, out=lambda line: None) == 0


def test_dry_run_passes_from_real_rest_position():
    # joint2 rests at +1.58 deg, just above the URDF upper limit 0.0;
    # the recovery rule allows moving back toward the range.
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    arm = FakeArm(REST_DEG)
    lines = []
    assert first_move.dry_run(arm, tip_fn=_fake_tip, out=lines.append) == 0
    assert all("REFUSED" not in line for line in lines)


def test_loop_runs_at_planned_rate_with_caps_on_every_send():
    from muse_qc.first_move_plan import LOOP_RATE_HZ

    arm = FakeArm(REST_DEG)
    result = first_move.execute(arm, input_fn=lambda prompt: "yes",
                                check_fn=_always_ok, tip_fn=_fake_tip,
                                vlims=VLIMS, out=lambda line: None)
    assert result == 0
    assert arm.loop_rate == LOOP_RATE_HZ == 50.0
    slow = [v * 0.1 for v in VLIMS]
    group = arm.groups["arm"]
    assert ("mode", slow) in group.calls
    sends = [call for call in group.calls if call[0] == "send"]
    assert len(sends) > 200  # first command plus every waypoint plus hold
    for _, _, caps in sends:
        assert caps == pytest.approx(slow)


def test_bad_reading_refuses_and_holds_last_command():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    arm = FakeArm(REST_DEG, group_cls=FlakyGroup)
    lines = []
    result = first_move.execute(arm, input_fn=lambda prompt: "yes",
                                check_fn=None, tip_fn=_fake_tip,
                                vlims=VLIMS, out=lines.append)
    assert result == 1
    sends = [call[1] for call in arm.groups["arm"].calls if call[0] == "send"]
    assert all(abs(values[2]) > 0.0 for values in sends)
    assert sends[-10:] == [sends[-1]] * 10  # hold repeats the last command
    assert any(line.startswith("REFUSED ") for line in lines)


def test_execute_without_yes_never_enables():
    arm = FakeArm(REST_DEG)
    lines = []
    result = first_move.execute(arm, input_fn=lambda prompt: "no",
                                check_fn=_always_ok, tip_fn=_fake_tip,
                                vlims=VLIMS, out=lines.append)
    assert result == 2
    assert not any(call in ("enable", "loop") or call[0] in ("mode", "send")
                   for call in arm.groups["arm"].calls)
    assert "disconnect" in arm.calls


def test_first_streamed_command_is_current_reading():
    arm = FakeArm(REST_DEG)
    expected = [math.radians(v) for v in REST_DEG]
    result = first_move.execute(arm, input_fn=lambda prompt: "yes",
                                check_fn=_always_ok, tip_fn=_fake_tip,
                                vlims=VLIMS, out=lambda line: None)
    assert result == 0
    group = arm.groups["arm"]
    assert ("mode", [v * 0.1 for v in VLIMS]) in group.calls
    sends = _sends(group)
    assert sends, "expected streamed commands"
    assert sends[0] == pytest.approx(expected)
    assert "loop" in arm.calls and "stop" in arm.calls


def test_ramp_steps_stay_within_speed_budget():
    waypoints = build_plan(REST_DEG)
    previous = list(REST_DEG)
    for waypoint in waypoints:
        assert len(waypoint) == 6
        peak = max(abs(w - p) for w, p in zip(waypoint, previous))
        assert peak <= STEP_DEG + 1e-9
        previous = list(waypoint)
    assert waypoints[-1] == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert any(abs(w[0] - 5.0) < 1e-9 for w in waypoints)
    assert ramp_waypoints([1.0] * 6, [1.0] * 6)[-1] == (1.0,) * 6


def test_refusal_stops_ramp_and_holds_last_reading():
    def refuse_once_moved(target, current):
        # Dynamic cause: validation sees the arm at rest and passes,
        # the motion sees the moved arm and refuses.
        if current.joints_deg[0] > 1.0:
            return {"ok": False, "reason": "test-refusal",
                    "value": current.joints_deg[0], "limit": 1.0}
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    arm = FakeArm(REST_DEG)
    lines = []
    result = first_move.execute(arm, input_fn=lambda prompt: "yes",
                                check_fn=refuse_once_moved,
                                tip_fn=_fake_tip, vlims=VLIMS, out=lines.append)
    assert result == 1
    sends = _sends(arm.groups["arm"])
    streamed_j1 = [math.degrees(values[0]) for values in sends[1:]]
    assert streamed_j1
    assert max(streamed_j1) <= 1.0 + STEP_DEG + 1e-9
    assert any(line.startswith("REFUSED ") for line in lines)
    held = sends[-1]
    assert sends[-10:] == [held] * 10  # hold repeats the last reading


def test_scale_vlim_reads_config_not_hardcoded(tmp_path):
    hardware = (
        "groups:\n  arm:\n    joints: [joint1, joint2, joint3, joint4, joint5, joint6]\n"
        "joints:\n"
    )
    for name, cap in zip(
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], VLIMS):
        hardware += f'  - name: {name}\n    POS_VEL:\n      vlim: {cap}\n'
    (tmp_path / "rebotarm.yaml").write_text('hardware_yaml: "hw.yaml"\n',
                                            encoding="utf-8")
    (tmp_path / "hw.yaml").write_text(hardware, encoding="utf-8")
    assert read_sdk_vlims(tmp_path) == VLIMS
    assert scale_vlim(VLIMS) == [v * 0.1 for v in VLIMS]
    assert read_sdk_vlims(tmp_path / "missing") is None


def test_source_avoids_motion_primitives():
    from pathlib import Path

    for name in ("muse_qc/first_move.py", "muse_qc/first_move_plan.py"):
        text = Path(name).read_text(encoding="utf-8")
        for forbidden in ("set_zero", "safe_home", "estop", "RebotArmEndPose"):
            assert forbidden not in text
