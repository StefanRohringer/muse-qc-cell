"""Tests for the first pick: pick_plan, pick_test, pick_loop.

Offline only: fake SDK, fake TouchSensor, fake solve_hover, real or
stubbed safety. No bus, no serial port, no camera, no network.
"""

from __future__ import annotations

import math
import time

import pytest

from muse_qc import pick_loop, pick_plan, pick_test, safety
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.gripper_grip import CLOSED_DEG, width_to_deg

TABLE = (495.9, 322.7)
REST = [-3.33, -99.0, -79.0, 74.0, 0.91, -3.97]
REST_GRIP = -4.97
VLIMS = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}
OPEN_DEG = width_to_deg(40.0)
GRIP_STEP = 15.0 / LOOP_RATE_HZ


def _fake_solve(table_xy, height_mm, seed):
    """Straight line in z: joints linear in the height, tip at the height."""
    height = float(height_mm)
    return {
        "ok": True,
        "joints_deg": (-3.0, -90.0 - 0.1 * height, -80.0, 75.0, 1.0, -4.0),
        "tip_base_mm": (300.0, 0.0, height),
        "tip_table_mm": (300.0, 0.0, height),
        "error_mm": 0.0,
        "tilt_deg": 0.0,
        "iterations": 1,
    }


def _fake_solve_fail_below_50(table_xy, height_mm, seed):
    if float(height_mm) <= 50.0:
        return {"ok": False, "reason": "unreachable: IK did not converge"}
    return _fake_solve(table_xy, height_mm, seed)


def _fake_tip(joints_deg):
    """Tip z from joint 2, matching _fake_solve: z = height in metres."""
    height = -(float(joints_deg[1]) + 90.0) * 10.0
    return (0.30, 0.0, height / 1000.0)


def _always_ok(target, current, z_min_m=None):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


class FakeArmGroup:
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


class JumpGripperGroup(FakeGripperGroup):
    def __init__(self, joint_deg, jump_at, jump_deg=50.0):
        super().__init__(joint_deg)
        self.jump_at = jump_at
        self.jump_rad = math.radians(jump_deg)
        self.reads = 0

    def get_positions(self):
        self.calls.append("get")
        self.reads += 1
        if self.reads == self.jump_at:
            return [self._rad[0] + self.jump_rad]
        return list(self._rad)


class InterruptArmGroup(FakeArmGroup):
    def __init__(self, joints_deg, interrupt_at):
        super().__init__(joints_deg)
        self.interrupt_at = interrupt_at
        self.reads = 0

    def get_positions(self):
        self.reads += 1
        if self.reads == self.interrupt_at:
            raise KeyboardInterrupt()
        return super().get_positions()


class FakeSdk:
    def __init__(self, joints_deg=REST, grip_deg=REST_GRIP, grip_group=None,
                 arm_group=None):
        self.groups = {
            "arm": arm_group if arm_group is not None else FakeArmGroup(joints_deg),
            "gripper": grip_group if grip_group is not None else FakeGripperGroup(grip_deg),
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
    def start_control_loop(self, fn, rate=None):
        raise RuntimeError("bus down")


class FakeReading:
    def __init__(self, contacts):
        if isinstance(contacts, str):
            contacts = [] if contacts.strip() in ("", "-") else contacts.split()
        self.contacts = frozenset(contacts)


def _dline(number, contacts):
    return ("D,%d," % number + ",".join(["1.0", "2.0", "3.0", "10.0"] * 4)
            + ",%s\r\n" % contacts)


class FakeSensor:
    """TouchSensor stand-in: scripted contacts per loop cycle."""

    def __init__(self, script):
        self.script = script
        self._reads = 0
        self._lines = 0
        self.closed = False

    def open(self, port):
        self.port = port

    def send(self, cmd):
        pass

    def read_line(self, timeout):
        self._lines += 1
        if self._lines == 1:
            return "# firmware 2"
        if self._lines <= 201:
            return _dline(self._lines, "-")
        return None

    def read(self, timeout):
        self._reads += 1
        if self._reads == 1:
            return None  # setup drain
        return FakeReading(self.script(self._reads - 1))

    def close(self):
        self.closed = True


@pytest.fixture(autouse=True)
def _restore_safety(monkeypatch):
    monkeypatch.setattr(time, "sleep", lambda s: None)
    stop = (safety._STOP_REASON, safety._GRIP_RETRIES,
            safety._HOLDING_CONTACTS, safety.J6_REF)
    yield
    (safety._STOP_REASON, safety._GRIP_RETRIES,
     safety._HOLDING_CONTACTS, safety.J6_REF) = stop


def _built(grip_z=8.0):
    built = pick_plan.build_pick_plan(REST, TABLE, grip_z, TEST_LIMITS,
                                      solve_fn=_fake_solve)
    assert built["ok"] is True
    return built


def _cycle_counts(built):
    ranges = built["ranges"]
    counts = {seg: ranges[seg][1] - ranges[seg][0] for seg in ranges}
    hover = counts["hover"]
    ramp = math.ceil(abs(OPEN_DEG - REST_GRIP) / GRIP_STEP)
    return hover, ramp, counts


def _run(sdk, script, **kwargs):
    kwargs.setdefault("input_fn", lambda prompt: "yes")
    kwargs.setdefault("tip_fn", _fake_tip)
    kwargs.setdefault("solve_fn", _fake_solve)
    kwargs.setdefault("arm_vlims", VLIMS)
    kwargs.setdefault("joint_limits", TEST_LIMITS)
    kwargs.setdefault("find_port", lambda: "/dev/ttyFAKE")
    sensor_holder = {}

    def factory():
        sensor = FakeSensor(script)
        sensor_holder["sensor"] = sensor
        return sensor

    kwargs.setdefault("sensor_factory", factory)
    kwargs.setdefault("hold_s", 0.5)
    lines = []
    kwargs.setdefault("out", lines.append)
    result = pick_test.execute(sdk, TABLE, 8.0, 40.0, False, **kwargs)
    return result, "\n".join(lines), sensor_holder.get("sensor")


def _arm_sends(sdk):
    return [tuple(call[1]) for call in sdk.groups["arm"].calls if call[0] == "send"]


def _grip_sends_deg(sdk):
    return [math.degrees(call[1][0])
            for call in sdk.groups["gripper"].calls if call[0] == "send"]


def test_safety_default_floor_still_refuses_002():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    joints = (10.0, -90.0, -45.0, 20.0, 30.0, 5.0)
    safety.set_j6_ref(5.0)
    target = safety.Target(x=0.30, y=0.0, z=0.02, joints_deg=joints, speed=0.1)
    state = safety.CurrentState(x=0.30, y=0.0, z=0.02, joints_deg=joints,
                                reading_age_ms=0.0)
    refused = safety.check_target(target, state)
    assert refused["ok"] is False and refused["reason"] == "tool-z"
    assert safety.check_target(target, state, z_min_m=safety.GRASP_Z_MIN_M)["ok"] is True
    low = safety.Target(x=0.30, y=0.0, z=0.003, joints_deg=joints, speed=0.1)
    denied = safety.check_target(low, state, z_min_m=safety.GRASP_Z_MIN_M)
    assert denied["ok"] is False and denied["reason"] == "tool-z"


def test_grasp_floor_only_on_flagged_waypoints():
    seen = []

    def spy(target, current, z_min_m=None):
        seen.append(z_min_m)
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    def tip(joints_deg):
        return (0.30, 0.0, 0.02)

    plan = [("hover", tuple(REST), False), ("lower", tuple(REST), True)]
    ok, bad, _ = pick_plan.check_pick_plan(plan, REST, check_fn=spy, tip_fn=tip)
    assert (ok, bad) == (True, -1)
    assert seen == [None, safety.GRASP_Z_MIN_M]


def test_grasp_floor_missing_on_low_waypoint_refused():
    def tip(joints_deg):
        return (0.30, 0.0, 0.02)

    old_ref = safety.J6_REF
    safety.set_j6_ref(REST[5])
    try:
        plan = [("hover", tuple(REST), False)]
        state_tip = tip(REST)
        assert state_tip[2] == pytest.approx(0.02)
        ok, bad, res = pick_plan.check_pick_plan(plan, REST, tip_fn=tip)
    finally:
        safety.set_j6_ref(old_ref)
    assert ok is False and bad == 0 and res["reason"] == "tool-z"


def test_pick_plan_segments_counts_and_flags():
    built = _built()
    plan, ranges = built["plan"], built["ranges"]
    assert [seg for seg in ranges] == ["hover", "lower", "lift", "put-down",
                                       "raise", "return"]
    counts = {seg: ranges[seg][1] - ranges[seg][0] for seg in ranges}
    assert counts["lower"] == 92 * 10
    assert counts["lift"] == counts["lower"]
    assert counts["put-down"] == 90 * 10
    assert counts["raise"] == counts["put-down"]
    index = 0
    for seg in ["hover", "lower", "lift", "put-down", "raise", "return"]:
        low, high = ranges[seg]
        assert (low, high) == (index, index + counts[seg])
        index = high
    assert index == len(plan)
    for seg, joints, grasp in plan:
        assert grasp == (seg in ("lower", "lift", "put-down", "raise"))
    assert built["grip_z_mm"] == pytest.approx(8.0)
    assert built["hover_mm"] == pytest.approx(100.0)
    assert built["grip_joints"][1] == pytest.approx(-90.8)
    assert built["putdown_joints"][1] == pytest.approx(-91.0)


def test_pick_plan_lift_mirrors_lower_and_return_settles():
    from muse_qc.first_move_plan import SETTLE_HOLD_CYCLES, clamp_to_limits
    built = _built()
    plan, ranges = built["plan"], built["ranges"]
    lower = [wp for seg, wp, _ in plan[ranges["lower"][0]:ranges["lower"][1]]]
    lift = [wp for seg, wp, _ in plan[ranges["lift"][0]:ranges["lift"][1]]]
    assert lower[-1] == built["grip_joints"]  # exact rung end
    assert lift[-1] == built["hover_joints"]  # exact rung end
    for got, want in zip(lift, lower[::-1]):
        assert got == pytest.approx(want, abs=0.02)  # same ladder, one step offset
    rest = clamp_to_limits(REST, TEST_LIMITS)
    assert plan[-1] == ("return", rest, False)
    assert sum(1 for seg, wp, _ in plan[ranges["return"][0]:]
               if wp == rest) >= SETTLE_HOLD_CYCLES


def test_pick_plan_ladder_failure_names_height():
    built = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                      solve_fn=_fake_solve_fail_below_50)
    assert built["ok"] is False
    assert "50.0 mm" in built["reason"]


def test_pick_plan_abort_detour_pure():
    built = _built()
    plan, ranges = built["plan"], built["ranges"]
    tips = [_fake_tip(list(wp)) for _, wp, _ in plan]
    hover_z = tips[ranges["hover"][1] - 1][2]
    bottom = ranges["lower"][1] - 1
    detour = pick_plan.abort_detour(plan, tips, ranges, hover_z, bottom)
    assert detour[0] == bottom
    assert detour[-len(range(*ranges["return"])): ] == list(range(*ranges["return"]))
    assert all(tips[i][2] >= tips[bottom][2] - 1e-9 for i in detour)
    top = ranges["hover"][1] - 1
    assert pick_plan.abort_detour(plan, tips, ranges, hover_z, top) == \
        list(range(*ranges["return"]))
    assert pick_plan.abort_detour(plan, tips, ranges, hover_z, None) == \
        list(range(*ranges["return"]))


def test_pick_plan_real_solve_hover_ladder():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed")
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    start = [-3.33, 1.58, -1.45, 2.72, 0.91, -3.97]
    built = pick_plan.build_pick_plan(start, TABLE, 90.0, None)
    assert built["ok"] is True
    assert set(built["ranges"]) == {"hover", "lower", "lift", "put-down",
                                    "raise", "return"}


def test_plan_summary_lines():
    built = _built()
    lines = pick_plan.plan_summary(built["plan"], built)
    text = "\n".join(lines)
    assert "lower 920" in text and "lift 920" in text
    assert "hover tip z: 100.0 mm" in text and "grip tip z: 8.0 mm" in text
    assert "corrected base target:" in text
    assert "xy correction: on" in text


def test_no_xy_correction_skips_base_shift():
    seen = []

    def recording_solve(table_xy, height_mm, seed):
        seen.append((float(table_xy[0]), float(table_xy[1])))
        return _fake_solve(table_xy, height_mm, seed)

    on = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                   solve_fn=recording_solve)
    assert on["ok"] is True and on["use_xy_correction"] is True
    assert seen and all(math.dist(pt, TABLE) > 0.5 for pt in seen)
    seen.clear()
    off = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                    solve_fn=recording_solve,
                                    use_xy_correction=False)
    assert off["ok"] is True and off["use_xy_correction"] is False
    assert seen and all(math.dist(pt, TABLE) < 1e-6 for pt in seen)
    off_text = "\n".join(pick_plan.plan_summary(off["plan"], off))
    assert "xy correction: off" in off_text


def test_cli_no_xy_correction_dry_run():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--no-xy-correction"],
        sdk_factory=lambda: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0
    assert "xy correction: off" in "\n".join(lines)


@pytest.mark.parametrize("flag,value", [("--grip-z-mm", "4.9"), ("--grip-z-mm", "60.1"),
                                        ("--grip-z-mm", "abc"),
                                        ("--open-mm", "24.9"), ("--open-mm", "50.1"),
                                        ("--hold-s", "0.49"), ("--hold-s", "30.1")])
def test_arg_ranges_exit_2_before_connect(flag, value):
    def factory():
        raise AssertionError("nothing may connect on a bad argument")

    with pytest.raises(SystemExit) as info:
        pick_test.main(["--table-mm", "495.9", "322.7", flag, value],
                       sdk_factory=factory)
    assert info.value.code == 2


def test_missing_table_mm_exits_2_before_connect():
    def factory():
        raise AssertionError("nothing may connect without a table target")

    with pytest.raises(SystemExit) as info:
        pick_test.main([], sdk_factory=factory)
    assert info.value.code == 2


def test_dry_run_never_enables_or_sends():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7"],
        sdk_factory=lambda: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0
    text = "\n".join(lines)
    assert "segments at 100 Hz:" in text
    assert "hover tip z: 100.0 mm" in text and "grip tip z: 8.0 mm" in text
    assert "corrected base target:" in text
    assert "xy correction: on" in text
    assert "dry run: moves nothing, motors never enabled" in text
    assert "connect" in sdk.calls and "disconnect" in sdk.calls
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    for group in sdk.groups.values():
        assert not any(isinstance(call, tuple) and call[0] in ("mode", "send")
                       for call in group.calls)


def test_dry_run_real_safety_chained_passes():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7"],
        sdk_factory=lambda: sdk,
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0


def test_execute_without_yes_enables_nothing():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--execute"],
        sdk_factory=lambda: sdk,
        input_fn=lambda prompt: "no",
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 2
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert "disconnect" in sdk.calls


def test_execute_wrapped_gripper_start_refused():
    sdk = FakeSdk(grip_deg=33.21)
    code, _, _ = _run(sdk, lambda n: "-")
    assert code == 1
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert not _grip_sends_deg(sdk)


def test_no_grip_holds_at_bottom_and_returns():
    sdk = FakeSdk()
    lines = []
    code = pick_test.execute(
        sdk, TABLE, 8.0, 40.0, True, 0.5, input_fn=lambda prompt: "yes",
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"), out=lines.append)
    text = "\n".join(lines)
    assert code == 0
    assert "measure the tip height now" in text
    assert "holding at tip z 8.0 mm" in text
    built = _built()
    low, high = built["ranges"]["lower"]
    bottom_rad = tuple(math.radians(v) for v in built["plan"][high - 1][1])
    sends = _arm_sends(sdk)
    assert sum(1 for item in sends if item == bottom_rad) >= 50
    rest_rad = tuple(math.radians(v) for v in REST)
    assert sends[-1] == rest_rad
    grips = _grip_sends_deg(sdk)
    assert grips[-1] == pytest.approx(OPEN_DEG)
    assert "end readings:" in text
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls


def test_contact_during_lower_stops_within_one_cycle():
    built = _built()
    hover, ramp, _ = _cycle_counts(built)
    stop_at = hover + ramp + 5
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: ("L1 R1" if n >= stop_at else "-"))
    assert code == 1
    assert "contact while lowering:" in text
    assert "at tip z about" in text
    ranges = built["ranges"]
    lower_set = {tuple(math.radians(v) for v in wp)
                 for _, wp, _ in built["plan"][ranges["lower"][0]:ranges["lower"][1]]}
    sends = _arm_sends(sdk)
    assert len({item for item in sends if item in lower_set}) <= 5
    stop_z = _fake_tip(list(built["plan"][ranges["lower"][0] + 3][1]))[2]
    rest_rad = tuple(math.radians(v) for v in REST)
    assert sends[-1] == rest_rad
    hover_rad = tuple(math.radians(v) for v in built["hover_joints"])
    first_lower = next(i for i, item in enumerate(sends) if item in lower_set)
    hover_back = next(i for i in range(first_lower, len(sends))
                      if sends[i] == hover_rad)
    for item in sends[first_lower:hover_back + 1]:
        deg = [math.degrees(v) for v in item]
        assert _fake_tip(deg)[2] >= stop_z - 1e-9
    assert "disable_all" in sdk.calls


def test_bilateral_contact_closes_squeezes_lifts():
    built = _built()
    hover, ramp, _ = _cycle_counts(built)
    close_start = hover + ramp + (ranges_len(built, "lower")) + 1
    touch_at = close_start + 14
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: ("L1 R1" if n >= touch_at else "-"))
    assert code == 0
    assert "part lifted, holding" in text
    contact_cmd = OPEN_DEG + (touch_at - close_start) * GRIP_STEP
    squeeze = min(contact_cmd + 2.0, CLOSED_DEG)
    grips = _grip_sends_deg(sdk)
    assert any(abs(v - squeeze) < 1e-9 for v in grips)
    assert max(grips) < CLOSED_DEG - 1.0  # stopped early, never drove to closed
    assert grips[-1] == pytest.approx(OPEN_DEG)
    assert not safety.is_stopped()
    assert "disable_all" in sdk.calls


def ranges_len(built, seg):
    low, high = built["ranges"][seg]
    return high - low


def test_one_sided_contact_missed_grip_opens_before_going_up():
    built = _built()
    hover, ramp, _ = _cycle_counts(built)
    close_start = hover + ramp + ranges_len(built, "lower") + 1
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: ("L1" if n >= close_start + 14 else "-"))
    assert code == 0
    assert "missed grip" in text
    assert max(_grip_sends_deg(sdk)) == pytest.approx(CLOSED_DEG)
    arm_items, grip_items = _arm_sends(sdk), _grip_sends_deg(sdk)
    reached_closed = next(i for i, value in enumerate(grip_items)
                          if abs(value - CLOSED_DEG) < 1e-9)
    first_reopen = next(i for i in range(reached_closed, len(grip_items))
                        if abs(grip_items[i] - OPEN_DEG) < 1e-9)
    reopen_tip_z = _fake_tip([math.degrees(v) for v in arm_items[first_reopen]])[2]
    assert reopen_tip_z <= 0.008 + 1e-6  # still at the bottom while opening
    later_z = [_fake_tip([math.degrees(v) for v in item])[2]
               for item in arm_items[first_reopen + 1:]]
    assert max(later_z) > 0.05  # climbs back up only afterwards
    assert "disable_all" in sdk.calls


def test_no_contact_missed_grip():
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: "-")
    assert code == 0
    assert "missed grip" in text
    assert max(_grip_sends_deg(sdk)) == pytest.approx(CLOSED_DEG)


def test_part_lost_printed_and_no_open_above_putdown():
    built = _built()
    hover, ramp, _ = _cycle_counts(built)
    lower_len = ranges_len(built, "lower")
    close_start = hover + ramp + lower_len + 1
    touch_at = close_start + 14
    drop_at = touch_at + 300

    def script(n):
        if n < touch_at:
            return set()
        if n < drop_at:
            return {"L1", "R1"}
        return {"L1"}

    sdk = FakeSdk()
    code, text, _ = _run(sdk, script)
    assert code == 0
    assert "part lost while lifting" in text
    contact_cmd = OPEN_DEG + (touch_at - close_start) * GRIP_STEP
    squeeze = min(contact_cmd + 2.0, CLOSED_DEG)
    arm_items, grip_items = _arm_sends(sdk), _grip_sends_deg(sdk)
    reached = next(i for i, value in enumerate(grip_items)
                   if abs(value - squeeze) < 1e-6)
    first_opener = next(i for i in range(reached, len(grip_items))
                        if grip_items[i] < squeeze - 1.0)
    open_tip_z = _fake_tip([math.degrees(v) for v in arm_items[first_opener]])[2]
    assert open_tip_z <= 0.010 + 1e-9  # opens only at/below the put-down height
    assert "disable_all" in sdk.calls


def test_gripper_jump_fault_goes_up_then_returns():
    built = _built()
    hover, ramp, _ = _cycle_counts(built)
    jump_at = hover + ramp + ranges_len(built, "lower") + 30
    group = JumpGripperGroup(REST_GRIP, jump_at=jump_at)
    sdk = FakeSdk(grip_group=group)
    code, text, _ = _run(sdk, lambda n: "-")
    assert code == 1
    assert "jumped" in text and "holding position" in text
    rest_rad = tuple(math.radians(v) for v in REST)
    assert _arm_sends(sdk)[-1] == rest_rad
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls


def test_ctrl_c_goes_up_then_returns():
    built = _built()
    hover, ramp, _ = _cycle_counts(built)
    interrupt_at = hover + ramp + 10
    arm_group = InterruptArmGroup(REST, interrupt_at=interrupt_at)
    sdk = FakeSdk(arm_group=arm_group)
    code, text, _ = _run(sdk, lambda n: "-")
    assert code == 1
    assert "interrupted by human, holding position" in text
    rest_rad = tuple(math.radians(v) for v in REST)
    assert _arm_sends(sdk)[-1] == rest_rad
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls


def test_failing_loop_disables():
    sdk = FailingLoopSdk()
    lines = []
    code = pick_test.execute(
        sdk, TABLE, 8.0, 40.0, False, 0.5, input_fn=lambda prompt: "yes",
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"), out=lines.append)
    assert code == 1
    assert "motion failed: bus down" in "\n".join(lines)
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls


def test_every_arm_send_is_planned_or_held():
    sdk = FakeSdk()
    code, _, _ = _run(sdk, lambda n: "-")
    assert code == 0
    built = _built()
    planned = {tuple(math.radians(v) for v in wp) for _, wp, _ in built["plan"]}
    previous = None
    for item in _arm_sends(sdk):
        assert item in planned or item == previous
        previous = item


def test_vlims_run_at_ten_percent():
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: "-")
    assert code == 0
    slow = [v * 0.1 for v in VLIMS]
    assert ("mode", slow) in sdk.groups["arm"].calls
    grip_modes = [call[1] for call in sdk.groups["gripper"].calls if call[0] == "mode"]
    assert grip_modes and grip_modes[0] == pytest.approx([3.0 * 0.1])
    assert sdk.loop_rate == LOOP_RATE_HZ == 100.0

