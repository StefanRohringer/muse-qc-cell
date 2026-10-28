"""Tests for gripper_motor.py. Offline only: fake SDK, no bus."""

from __future__ import annotations

import math

import pytest

from muse_qc import gripper_motor, safety
from muse_qc.first_move_plan import (
    HOLD_CYCLES,
    LOOP_RATE_HZ,
    SEG_BACK,
    SEG_HOLD,
    SEG_OUT,
    SEG_SETTLE,
    SETTLE_HOLD_CYCLES,
)
from muse_qc.gripper_motor import build_gripper_plan
from muse_qc.first_move_plan import clamp_to_limits

REST_DEG = [-3.33, -1.58, -1.45, 2.72, 0.91, -3.97]
REST_TIP_M = (0.2865, -0.0201, 0.1988)
REST_GRIP_DEG = -167.13
VLIMS = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}
REAL_REST_DEG = [-2.74, 1.08, -1.34, 2.09, 1.48, -0.47]


class FakeArmGroup:
    """Fake SDK arm group: reads return the last commanded position."""

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

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send", values, [float(v) for v in vlim]))
        self._rad = list(values)


class FakeGripperGroup:
    """Fake SDK gripper group: one joint, reads return last command."""

    def __init__(self, joint_deg):
        self._rad = [math.radians(joint_deg)]
        self.calls = []

    @property
    def joint_names(self):
        return ["gripper"]

    def get_positions(self):
        self.calls.append("get")
        return list(self._rad)

    def mode_pos_vel(self, vlim=None):
        self.calls.append(("mode", [float(v) for v in vlim]))

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send", values, [float(v) for v in vlim]))
        self._rad = list(values)


class FakeSdk:
    """Fake SDK object: control loop runs inline, never touches a bus."""

    def __init__(self, joints_deg=REST_DEG, grip_deg=REST_GRIP_DEG):
        self.groups = {
            "arm": FakeArmGroup(joints_deg),
            "gripper": FakeGripperGroup(grip_deg),
        }
        self.calls = []
        self.loop_rate = None

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def enable_all(self):
        self.calls.append("enable_all")

    def disable_all(self):
        self.calls.append("disable_all")

    def start_control_loop(self, fn, rate=None):
        self.calls.append("loop")
        self.loop_rate = rate
        for _ in range(30000):
            fn(self, 1.0 / rate if rate else 0.01)
            if getattr(fn, "done", None) is not None and fn.done.is_set():
                break

    def stop_control_loop(self):
        self.calls.append("stop")


class FailingLoopSdk(FakeSdk):
    """The bus fails when the control loop should start."""

    def start_control_loop(self, fn, rate=None):
        raise RuntimeError("bus down")


def _always_ok(target, current):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _fake_tip(joints_deg):
    return REST_TIP_M


def _refuse(target, current):
    return {"ok": False, "reason": "test-refusal", "value": 0.0, "limit": 1.0}


def _arm_sends(sdk):
    return [call[1] for call in sdk.groups["arm"].calls if call[0] == "send"]


def _grip_sends(sdk):
    return [call[1][0] for call in sdk.groups["gripper"].calls if call[0] == "send"]


@pytest.mark.parametrize("value", ["0", "0.0", "-0.0", "20.1", "-20.1", "abc"])
def test_delta_range_exits_2_without_connect(value):
    def factory():
        raise AssertionError("nothing may connect on a bad --delta-deg")

    with pytest.raises(SystemExit) as info:
        gripper_motor.main(["--delta-deg", value], sdk_factory=factory)
    assert info.value.code == 2


def test_delta_missing_exits_2_without_connect():
    def factory():
        raise AssertionError("nothing may connect without --delta-deg")

    with pytest.raises(SystemExit) as info:
        gripper_motor.main([], sdk_factory=factory)
    assert info.value.code == 2


@pytest.mark.parametrize("value", ["20", "-20", "10", "-0.5"])
def test_delta_boundaries_dry_run(value):
    sdk = FakeSdk()
    lines = []
    assert gripper_motor.main(["--delta-deg", value], sdk_factory=lambda: sdk,
                              out=lines.append) == 0
    assert any("expectation:" in line for line in lines)


def test_dry_run_never_modes_enables_loops_or_sends():
    sdk = FakeSdk()
    lines = []
    assert gripper_motor.dry_run(sdk, 10.0, out=lines.append) == 0
    assert "connect" in sdk.calls and "disconnect" in sdk.calls
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    for group in sdk.groups.values():
        assert not any(call[0] in ("mode", "send") for call in group.calls
                       if isinstance(call, tuple))
    text = "\n".join(lines)
    assert "rest joints deg:" in text
    assert "gripper rest deg: -167.13" in text
    assert "+10.0 deg should close the jaws by about 3.2 mm" in text
    assert "moves nothing" in text


def test_plan_lengths_and_bounds():
    plan = build_gripper_plan(REST_GRIP_DEG, 10.0)
    out_len = math.ceil(10.0 / 0.05)
    assert [segment for segment, _ in plan].count(SEG_OUT) == out_len
    assert [segment for segment, _ in plan].count(SEG_HOLD) == HOLD_CYCLES
    assert [segment for segment, _ in plan].count(SEG_BACK) == out_len
    assert [segment for segment, _ in plan].count(SEG_SETTLE) == SETTLE_HOLD_CYCLES
    points = [point for _, point in plan]
    assert min(points) == pytest.approx(REST_GRIP_DEG)
    assert max(points) == pytest.approx(REST_GRIP_DEG + 10.0)
    assert points[out_len - 1] == pytest.approx(REST_GRIP_DEG + 10.0)
    assert points[out_len + HOLD_CYCLES - 1] == pytest.approx(REST_GRIP_DEG + 10.0)
    assert plan[-1] == (SEG_SETTLE, pytest.approx(REST_GRIP_DEG))


def test_plan_negative_delta_bounds():
    plan = build_gripper_plan(REST_GRIP_DEG, -7.5)
    points = [point for _, point in plan]
    assert min(points) == pytest.approx(REST_GRIP_DEG - 7.5)
    assert max(points) == pytest.approx(REST_GRIP_DEG)


def test_plan_rejects_zero_delta():
    with pytest.raises(ValueError):
        build_gripper_plan(REST_GRIP_DEG, 0.0)


def test_execute_sends_arm_start_every_cycle():
    sdk = FakeSdk()
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=_always_ok,
        tip_fn=_fake_tip, arm_vlims=VLIMS, joint_limits=TEST_LIMITS,
        out=lines.append)
    assert result == 0
    assert sdk.loop_rate == LOOP_RATE_HZ == 100.0
    clamped = list(clamp_to_limits(REST_DEG, TEST_LIMITS))
    assert clamped == pytest.approx(REST_DEG)
    expected_rad = [math.radians(v) for v in clamped]
    sends = _arm_sends(sdk)
    assert sends, "expected arm hold commands"
    for values in sends:
        assert values == pytest.approx(expected_rad)
    grip_deg = [math.degrees(v) for v in _grip_sends(sdk)]
    assert max(grip_deg) == pytest.approx(REST_GRIP_DEG + 10.0)
    assert grip_deg[-1] == pytest.approx(REST_GRIP_DEG)
    text = "\n".join(lines)
    assert "gripper commanded deg: -157.13" in text
    assert "gripper measured deg: -157.13" in text
    assert "gripper back deg: -167.13" in text
    assert "stop" in sdk.calls and "disable_all" in sdk.calls
    assert "disconnect" in sdk.calls


def test_execute_uses_ten_percent_vlims():
    sdk = FakeSdk()
    gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=_always_ok,
        tip_fn=_fake_tip, arm_vlims=VLIMS, out=lambda line: None)
    slow = [v * 0.1 for v in VLIMS]
    assert ("mode", slow) in sdk.groups["arm"].calls
    grip_modes = [call[1] for call in sdk.groups["gripper"].calls
                  if call[0] == "mode"]
    assert grip_modes and grip_modes[0] == pytest.approx([3.0 * 0.1])
    for _, _, caps in (call for call in sdk.groups["arm"].calls
                       if call[0] == "send"):
        assert caps == pytest.approx(slow)
    for _, _, caps in (call for call in sdk.groups["gripper"].calls
                       if call[0] == "send"):
        assert caps == pytest.approx([3.0 * 0.1])


def test_execute_reads_arm_vlims_from_config():
    sdk = FakeSdk()
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=_always_ok,
        tip_fn=_fake_tip, arm_vlims=None, out=lines.append)
    assert result == 0
    assert ("mode", [v * 0.1 for v in VLIMS]) in sdk.groups["arm"].calls


def test_execute_refused_hold_moves_nothing():
    sdk = FakeSdk()
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=_refuse,
        tip_fn=_fake_tip, arm_vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append)
    assert result == 1
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert not _arm_sends(sdk) and not _grip_sends(sdk)
    assert "disconnect" in sdk.calls
    assert any("refused" in line for line in lines)


def test_execute_without_yes_never_enables():
    sdk = FakeSdk()
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "no", check_fn=_always_ok,
        tip_fn=_fake_tip, arm_vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append)
    assert result == 2
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert not _arm_sends(sdk) and not _grip_sends(sdk)
    assert "disconnect" in sdk.calls


def test_loop_exception_still_disables():
    sdk = FailingLoopSdk()
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=_always_ok,
        tip_fn=_fake_tip, arm_vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append)
    assert result == 1
    assert "disable_all" in sdk.calls
    assert "stop" in sdk.calls
    assert "disconnect" in sdk.calls
    assert any("motion failed: bus down" in line for line in lines)


def test_hold_check_uses_real_safety():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    sdk = FakeSdk()
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=None,
        tip_fn=_fake_tip, arm_vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append)
    assert result == 0
    assert any("arm hold: ok" in line for line in lines)


def test_hold_clamps_real_rest_above_limit():
    """Joint 2 rests above its URDF limit: hold the clamped pose.

    Bug repro: checking target == rest reading is refused with
    "joint-limit" because LIMIT_RECOVERY needs motion back toward the
    range. The clamped hold (joint 2 at the limit) is accepted and
    commanded every cycle.
    """
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    sdk = FakeSdk(joints_deg=REAL_REST_DEG, grip_deg=REST_GRIP_DEG)
    lines = []
    result = gripper_motor.execute(
        sdk, 10.0, input_fn=lambda prompt: "yes", check_fn=None,
        tip_fn=_fake_tip, arm_vlims=VLIMS, out=lines.append)
    assert result == 0
    text = "\n".join(lines)
    assert "arm hold: ok" in text
    assert "arm hold deg:" in text
    clamped = list(clamp_to_limits(REAL_REST_DEG, safety.JOINT_LIMITS_DEG))
    assert clamped[1] == pytest.approx(safety.JOINT_LIMITS_DEG["joint2"][1])
    assert clamped[1] == pytest.approx(0.0)
    assert clamped[:1] + clamped[2:] == pytest.approx(REAL_REST_DEG[:1] + REAL_REST_DEG[2:])
    expected_rad = [math.radians(v) for v in clamped]
    sends = _arm_sends(sdk)
    assert sends, "expected arm hold commands"
    for values in sends:
        assert values == pytest.approx(expected_rad)
    assert all(values[1] == pytest.approx(0.0) for values in sends)


def test_source_avoids_homing_primitives():
    from pathlib import Path

    text = Path("muse_qc/gripper_motor.py").read_text(encoding="utf-8")
    # The docstring names RebotArmEndPose only to document why its
    # open/close direction must be confirmed on the real arm; the code
    # itself must never home, zero or stop the arm this way.
    for forbidden in ("set_zero", "safe_home", "estop", "RebotArmEndPose("):
        assert forbidden not in text
