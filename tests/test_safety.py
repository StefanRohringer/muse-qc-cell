"""Tests for safety.py using fake values only. No hardware is touched."""

from __future__ import annotations

import importlib.util

import pytest

from muse_qc.arm import safety
from muse_qc.arm.safety import CurrentState, Target

CURRENT_JOINTS = (10.0, -90.0, -45.0, 20.0, 30.0, 5.0)

FAKE_JOINT_LIMITS_DEG = {
    "joint1": (-170.0, 170.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-170.0, 170.0),
    "joint4": (-170.0, 170.0),
    "joint5": (-170.0, 170.0),
    "joint6": (-170.0, 170.0),
}


@pytest.fixture(autouse=True)
def _clean_safety_state(monkeypatch):
    monkeypatch.setattr(safety, "JOINT_LIMITS_DEG", dict(FAKE_JOINT_LIMITS_DEG))
    safety.clear_stop(confirmed_by_user=True)
    safety.set_j6_ref(0.0)
    yield
    safety.clear_stop(confirmed_by_user=True)
    safety.set_j6_ref(0.0)


def make_state(**overrides) -> CurrentState:
    values: dict = {
        "x": 0.30,
        "y": 0.0,
        "z": 0.20,
        "joints_deg": CURRENT_JOINTS,
        "reading_age_ms": 12.0,
    }
    values.update(overrides)
    return CurrentState(**values)


def make_target(**overrides) -> Target:
    values: dict = {
        "x": 0.32,
        "y": 0.01,
        "z": 0.21,
        "joints_deg": CURRENT_JOINTS,
        "speed": None,
    }
    values.update(overrides)
    return Target(**values)


def assert_refusal(result: dict) -> dict:
    assert result["ok"] is False
    assert {"ok", "reason", "value", "limit"} <= set(result)
    assert isinstance(result["reason"], str) and result["reason"]
    return result


def test_joint_limits_loaded_from_sdk():
    if importlib.util.find_spec("reBotArm_control_py") is None:
        pytest.skip("Seeed SDK not installed")
    real_limits = safety._load_joint_limits_from_sdk()
    if real_limits is None:
        pytest.skip("SDK joint limits unavailable")
    assert set(real_limits) == set(safety.JOINT_NAMES)
    for low, high in real_limits.values():
        assert low < high


def test_valid_cartesian_nudge_passes_with_default_speed():
    result = safety.check_target(make_target(), make_state())
    assert result == {"ok": True, "speed": 0.10, "detail": "target passes all safety checks"}


def test_valid_joint_nudge_with_explicit_speed_passes():
    joints = (12.0, -92.0, -45.0, 20.0, 30.0, 8.0)
    result = safety.check_target(make_target(joints_deg=joints, speed=0.2), make_state())
    assert result["ok"] is True
    assert result["speed"] == pytest.approx(0.2)


def test_workspace_corner_at_limits_passes():
    state = make_state(x=0.48, y=-0.46, z=0.44)
    target = make_target(x=0.495, y=-0.475, z=0.45)
    assert safety.check_target(target, state)["ok"] is True


def test_joints_only_target_refused():
    target = Target(x=None, y=None, z=None, joints_deg=CURRENT_JOINTS)
    result = assert_refusal(safety.check_target(target, make_state()))
    assert result["reason"] == "incomplete-target"


def test_position_only_target_refused():
    result = assert_refusal(safety.check_target(make_target(joints_deg=None), make_state()))
    assert result["reason"] == "incomplete-target"


def test_fresh_state_at_age_limit_passes():
    assert safety.check_target(make_target(), make_state(reading_age_ms=200.0))["ok"] is True


def test_joint6_at_cable_boundary_passes():
    state = make_state(joints_deg=(10.0, -90.0, -45.0, 20.0, 30.0, 70.0))
    target = make_target(joints_deg=(10.0, -90.0, -45.0, 20.0, 30.0, 75.0))
    assert safety.check_target(target, state)["ok"] is True


@pytest.mark.parametrize(
    "position",
    [
        (0.555, 0.0, 0.20),  # 1 cm past the front table edge
        (0.30, 0.535, 0.20),  # 1 cm past the left table edge
        (0.30, -0.535, 0.20),  # 1 cm past the right table edge
        (0.045, 0.0, 0.20),  # 1 cm past the back table edge
        (-0.02, 0.0, 0.20),  # behind the base
        (0.30, 0.0, 0.02),  # below tool z minimum
    ],
)
def test_out_of_workspace_targets_refused(position):
    x, y, z = position
    result = assert_refusal(safety.check_target(make_target(x=x, y=y, z=z), make_state()))
    assert result["reason"] in ("workspace-x", "workspace-y", "tool-z")


def test_joint6_at_76_deg_refused():
    state = make_state(joints_deg=(10.0, -90.0, -45.0, 20.0, 30.0, 70.0))
    target = make_target(joints_deg=(10.0, -90.0, -45.0, 20.0, 30.0, 76.0))
    result = assert_refusal(safety.check_target(target, state))
    assert result["reason"] == "joint6-cable"


def test_joint6_move_with_unset_ref_refused():
    safety.set_j6_ref(None)
    target = make_target(joints_deg=(10.0, -90.0, -45.0, 20.0, 30.0, 10.0))
    result = assert_refusal(safety.check_target(target, make_state()))
    assert result["reason"] == "joint6-ref-unset"


def test_joint_move_outside_sdk_limits_refused():
    target = make_target(joints_deg=(10.0, 5.0, -45.0, 20.0, 30.0, 5.0))
    result = assert_refusal(safety.check_target(target, make_state()))
    assert result["reason"] == "joint-limit"


def test_speed_above_cap_refused():
    result = assert_refusal(safety.check_target(make_target(speed=0.5), make_state()))
    assert result["reason"] == "speed-cap"
    assert result["limit"] == safety.SPEED_MAX


def test_cartesian_step_of_6cm_refused():
    target = make_target(x=0.36)
    result = assert_refusal(safety.check_target(target, make_state()))
    assert result["reason"] == "step-cartesian"


def test_joint_step_above_10_deg_refused():
    target = make_target(joints_deg=(21.0, -90.0, -45.0, 20.0, 30.0, 5.0))
    result = assert_refusal(safety.check_target(target, make_state()))
    assert result["reason"] == "step-joint"


def test_stop_blocks_valid_target():
    safety.set_stop("probe")
    result = assert_refusal(safety.check_target(make_target(), make_state()))
    assert result["reason"] == "stop-latched"


def test_clear_stop_needs_user_confirmation():
    safety.set_stop("probe")
    assert_refusal(safety.clear_stop())
    assert safety.is_stopped() is True
    assert safety.clear_stop(confirmed_by_user=True)["ok"] is True
    assert safety.is_stopped() is False
    assert safety.check_target(make_target(), make_state())["ok"] is True


def test_stale_state_refused():
    result = assert_refusal(safety.check_target(make_target(), make_state(reading_age_ms=250.0)))
    assert result["reason"] == "stale-state"


@pytest.mark.parametrize("bad", [None, float("nan"), 0.0])
def test_failed_current_readings_refused(bad):
    joints = list(CURRENT_JOINTS)
    joints[0] = bad
    result = assert_refusal(safety.check_target(make_target(), make_state(joints_deg=tuple(joints))))
    assert result["reason"] == "invalid-current-reading"


def test_missing_joint_limits_refuse_every_move(monkeypatch):
    monkeypatch.setattr(safety, "JOINT_LIMITS_DEG", None)
    result = assert_refusal(safety.check_target(make_target(), make_state()))
    assert result["reason"] == "joint-limits-unavailable"


def test_open_gripper_without_contact_ok():
    assert safety.check_gripper("open", set())["ok"] is True


def test_single_sensor_while_moving_open_stops():
    result = assert_refusal(safety.check_gripper("open-moving", {"L1"}))
    assert result["reason"] == "contact-while-moving-open"
    assert safety.is_stopped() is True


def test_closed_with_only_left_contact_is_not_held():
    result = assert_refusal(safety.check_gripper("closed", {"L1", "L2"}))
    assert result["reason"] == "missed-grip"
    assert result["action"] == "retry"


def test_closed_with_left_and_right_contact_is_held():
    result = safety.check_gripper("closed", {"L1", "R2"})
    assert result == {"ok": True, "grip": "held", "detail": "part held by both fingers"}


def test_missed_grip_retries_then_stops():
    first = assert_refusal(safety.check_gripper("closed", set()))
    second = assert_refusal(safety.check_gripper("closed", set()))
    assert (first["action"], second["action"]) == ("retry", "retry")
    exhausted = assert_refusal(safety.check_gripper("closed", set()))
    assert exhausted["reason"] == "missed-grip-retries-exhausted"
    assert safety.is_stopped() is True


def test_contact_change_while_holding_and_moving_stops():
    safety.check_gripper("closed", {"L1", "R2"})
    assert safety.check_gripper("holding-moving", {"L1", "R2"})["ok"] is True
    result = assert_refusal(safety.check_gripper("holding-moving", {"L1"}))
    assert result["reason"] == "contact-change-while-holding"
    assert safety.is_stopped() is True


def test_contact_change_while_lowering_means_placed():
    safety.check_gripper("closed", {"L1", "R2"})
    assert safety.check_gripper("holding-lowering", {"L1", "R2"})["grip"] == "holding"
    result = safety.check_gripper("holding-lowering", set())
    assert result == {"ok": True, "grip": "placed", "action": "open-gripper", "detail": "part placed, open gripper"}


def test_unknown_phase_and_sensor_refused():
    assert_refusal(safety.check_gripper("half-open", set()))
    assert_refusal(safety.check_gripper("open", {"L9"}))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf"), None])
def test_non_finite_current_position_refused(bad):
    result = assert_refusal(safety.check_target(make_target(), make_state(x=bad)))
    assert result["reason"] == "invalid-current-position"


@pytest.mark.parametrize("speed", [0.0, -0.1])
def test_non_positive_speed_refused(speed):
    result = assert_refusal(safety.check_target(make_target(speed=speed), make_state()))
    assert result["reason"] == "speed-cap"


def test_work_limits_derive_from_table_edges():
    assert safety.EDGE_MARGIN_M == pytest.approx(0.05)
    assert safety.WORK_X_MAX_M == pytest.approx(safety.TABLE_FRONT_M - safety.EDGE_MARGIN_M)
    assert safety.WORK_Y_MAX_M == pytest.approx(safety.TABLE_LEFT_M - safety.EDGE_MARGIN_M)
    assert safety.WORK_Y_MIN_M == pytest.approx(-(safety.TABLE_RIGHT_M - safety.EDGE_MARGIN_M))
    assert safety.WORK_X_MAX_M == pytest.approx(0.495)
    assert safety.WORK_Y_MAX_M == pytest.approx(0.475)
    assert safety.WORK_Y_MIN_M == pytest.approx(-0.475)


@pytest.mark.parametrize(
    ("state_y", "target_y", "passes"),
    [
        (0.474, 0.474, True),  # just inside on the left (+y)
        (-0.474, -0.474, True),  # just inside on the right (-y)
        (0.470, 0.476, False),  # just outside on the left (+y)
        (-0.470, -0.476, False),  # just outside on the right (-y)
    ],
)
def test_workspace_y_limits_both_sides(state_y, target_y, passes):
    state = make_state(x=0.32, y=state_y, z=0.21)
    target = make_target(x=0.32, y=target_y, z=0.21)
    result = safety.check_target(target, state)
    if passes:
        assert result["ok"] is True
    else:
        assert_refusal(result)
        assert result["reason"] == "workspace-y"


def _recovery_state(current_j2):
    joints = (10.0, current_j2, -45.0, 20.0, 30.0, 5.0)
    return make_state(joints_deg=joints)


def _recovery_target(target_j2):
    joints = (10.0, target_j2, -45.0, 20.0, 30.0, 5.0)
    return make_target(joints_deg=joints)


def test_limit_recovery_allows_move_back_into_range():
    result = safety.check_target(_recovery_target(1.48), _recovery_state(1.58))
    assert result["ok"] is True


def test_limit_recovery_refuses_move_further_out():
    result = safety.check_target(_recovery_target(1.68), _recovery_state(1.58))
    assert_refusal(result)
    assert result["reason"] == "joint-limit"


def test_limit_recovery_refuses_hold_outside_range():
    result = safety.check_target(_recovery_target(1.58), _recovery_state(1.58))
    assert_refusal(result)
    assert result["reason"] == "joint-limit"


def test_limit_recovery_refuses_when_too_far_out():
    result = safety.check_target(_recovery_target(3.4), _recovery_state(3.5))
    assert_refusal(result)
    assert result["reason"] == "joint-limit"


def test_limit_recovery_allows_move_within_range():
    result = safety.check_target(_recovery_target(-0.5), _recovery_state(-1.0))
    assert result["ok"] is True
