"""Tests for hover_move.py. Offline only: fakes, no SDK bus."""

from __future__ import annotations

import math

import pytest

from muse_qc import first_move, hover_move, safety
from muse_qc.first_move_plan import (
    HOLD_CYCLES,
    SETTLE_HOLD_CYCLES,
    STEP_DEG,
    clamp_to_limits,
    validate_and_print,
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
CLAMPED_REST = (-3.33, 0.0, -1.45, 2.72, 0.91, -3.97)


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
        for _ in range(30000):
            fn(self, 0.01)
            if getattr(fn, "done", None) is not None and fn.done.is_set():
                break

    def stop_control_loop(self):
        self.calls.append("stop")


def _always_ok(target, current):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _fake_tip(joints_deg):
    return (0.2865, -0.0201, 0.1988)


def _tracking_tip(joints_deg):
    """Tip that follows the joints, so step-cartesian is exercised."""
    vals = [float(v) for v in joints_deg]
    return (0.300 + vals[0] * 0.0001, vals[1] * 0.0001, 0.200 + vals[2] * 0.0001)


def _fake_solve_ok(table_xy, hover_mm, seed):
    return {
        "ok": True,
        "joints_deg": tuple(GOAL_DEG),
        "tip_base_mm": (300.0, 0.0, float(hover_mm)),
        "error_mm": 0.01,
        "tilt_deg": 0.0,
    }


def _fake_solve_fail(table_xy, hover_mm, seed):
    return {"ok": False, "reason": "unreachable: IK did not converge"}


def _touched(group):
    return any(call in ("enable", "loop") or call[0] in ("mode", "send")
               for call in group.calls)


def test_plan_segments_in_order_ends_clamped_step_budget():
    plan = hover_move.build_hover_return_plan(REST_DEG, GOAL_DEG, TEST_LIMITS)
    segments = [segment for segment, _ in plan]
    assert segments[0] == "hover"
    assert set(segments) == {"hover", "hold", "return", "settle"}
    first_hold = segments.index("hold")
    first_return = segments.index("return")
    first_settle = segments.index("settle")
    assert first_hold > 0 and first_return > first_hold
    assert first_settle > first_return
    assert segments[first_settle:] == ["settle"] * len(segments[first_settle:])
    holds = [point for segment, point in plan if segment == "hold"]
    assert len(holds) == HOLD_CYCLES
    assert all(point == tuple(GOAL_DEG) for point in holds)
    settles = [point for segment, point in plan if segment == "settle"]
    assert len(settles) == SETTLE_HOLD_CYCLES
    assert plan[-1] == ("settle", CLAMPED_REST)
    previous = list(REST_DEG)
    for _, waypoint in plan:
        peak = max(abs(w - p) for w, p in zip(waypoint, previous))
        assert peak <= STEP_DEG + 1e-9
        previous = list(waypoint)


def test_return_ramp_ends_exactly_on_clamped_start():
    assert clamp_to_limits(REST_DEG, TEST_LIMITS) == CLAMPED_REST
    plan = hover_move.build_hover_return_plan(REST_DEG, GOAL_DEG, TEST_LIMITS)
    returns = [point for segment, point in plan if segment == "return"]
    assert returns
    assert returns[-1] == CLAMPED_REST


def test_dry_run_never_enables_or_sends(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hover_move.main(
        ["--table-mm", "495.9", "322.7"],
        sdk_factory=lambda: arm,
        check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 0
    assert any(line.startswith("table target:") for line in lines)
    assert any(line.startswith("ik joints deg:") for line in lines)
    assert not _touched(arm.groups["arm"])


def test_dry_run_real_safety_chained_passes(monkeypatch):
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hover_move.main(
        ["--table-mm", "495.9", "322.7"],
        sdk_factory=lambda: arm,
        out=lines.append,
    )
    assert code == 0
    assert not _touched(arm.groups["arm"])
    wp_lines = [line for line in lines if line.startswith("wp ")]
    assert wp_lines
    assert all(line.endswith(" ok") for line in wp_lines)
    assert not any("REFUSED" in line for line in lines)


def test_single_20deg_jump_refused_step_joint():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    start = [-3.33, -1.58, -1.45, 2.72, 0.91, -3.97]
    jumped = list(start)
    jumped[0] += 20.0
    arm = FakeArm(start)
    lines = []
    old_ref = safety.J6_REF
    safety.set_j6_ref(start[5])
    try:
        ok, _ = validate_and_print(
            arm.groups["arm"], [("hover", tuple(jumped))],
            tip_fn=_tracking_tip, out=lines.append,
        )
    finally:
        safety.set_j6_ref(old_ref)
    assert ok is False
    assert any("REFUSED step-joint" in line for line in lines)


def test_execute_without_yes_enables_nothing(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hover_move.main(
        ["--table-mm", "495.9", "322.7", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "no",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code == 2
    assert not _touched(arm.groups["arm"])


def test_solve_failure_enables_nothing(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_fail)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hover_move.main(
        ["--table-mm", "495.9", "322.7", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code != 0
    assert not _touched(arm.groups["arm"])
    assert any("plan failed" in line for line in lines)


def test_hover_below_80mm_enables_nothing(monkeypatch):
    monkeypatch.setattr(hover_move, "solve_hover", _fake_solve_ok)
    arm = FakeArm(REST_DEG)
    lines = []
    code = hover_move.main(
        ["--table-mm", "495.9", "322.7", "--hover", "50", "--execute"],
        sdk_factory=lambda: arm,
        input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS, out=lines.append,
    )
    assert code != 0
    assert not _touched(arm.groups["arm"])


def test_first_move_plan_exception_prints_and_enables_nothing():
    def boom(start_deg):
        raise RuntimeError("unreachable: test boom")

    arm = FakeArm(REST_DEG)
    lines = []
    code = first_move.execute(
        arm, input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip,
        vlims=VLIMS, joint_limits=TEST_LIMITS,
        out=lines.append, plan_fn=boom,
    )
    assert code == 1
    assert not _touched(arm.groups["arm"])
    assert any("plan failed" in line for line in lines)
    arm2 = FakeArm(REST_DEG)
    lines2 = []
    assert first_move.dry_run(
        arm2, check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines2.append,
        plan_fn=boom,
    ) == 1
    assert not _touched(arm2.groups["arm"])


def test_first_move_plan_fn_none_still_builds_old_plan():
    arm = FakeArm(REST_DEG)
    lines = []
    assert first_move.dry_run(
        arm, check_fn=_always_ok, tip_fn=_fake_tip,
        joint_limits=TEST_LIMITS, out=lines.append,
        plan_fn=None,
    ) == 0
    assert not _touched(arm.groups["arm"])
    assert any(line.startswith("plan:") for line in lines)
