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
GRIP_OPEN_STEP = 40.0 / LOOP_RATE_HZ
GRIP_FAST_CLOSE_STEP = 40.0 / LOOP_RATE_HZ
GRIP_SLOW_CLOSE_STEP = 15.0 / LOOP_RATE_HZ


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


def _cycle_counts(built, open_deg=OPEN_DEG):
    ranges = built["ranges"]
    counts = {seg: ranges[seg][1] - ranges[seg][0] for seg in ranges}
    hover = counts["hover"]
    ramp = math.ceil(abs(open_deg - REST_GRIP) / GRIP_OPEN_STEP)
    catchup = max(0, ramp - hover)  # gripper opens during hover first
    return hover, catchup, counts


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
    kwargs.setdefault("speed_deg_s", 15.0)
    kwargs.setdefault("accel_deg_s2", 30.0)
    kwargs.setdefault("arm_mode", "posvel")
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
    assert counts["hover"] == 37 + 50  # trapezoid ramp plus 0.5 s settle
    assert counts["lower"] == 67 * 2 + 25 * 5  # fast above grip + 25, slow below
    assert counts["lift"] == 82 * 2 + 10 * 5  # 10 mm slow watch, fast above
    assert counts["put-down"] == 65 * 2 + 25 * 5
    assert counts["raise"] == 90 * 2  # empty gripper, fast all the way
    assert counts["return"] == 37 + 100  # trapezoid ramp plus 1 s settle
    assert sum(counts.values()) == len(plan) == 1132
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
    lower_z = [_fake_tip(list(wp))[2] for wp in lower]
    lift_z = [_fake_tip(list(wp))[2] for wp in lift]
    assert all(b <= a + 1e-9 for a, b in zip(lower_z, lower_z[1:]))  # descends
    assert all(b >= a - 1e-9 for a, b in zip(lift_z, lift_z[1:]))  # climbs back
    assert lift_z[0] >= lower_z[-1] - 1e-9  # same ladder bottom
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
    assert "lower 259" in text and "lift 214" in text
    assert "free 15 deg/s, accel 30 deg/s2, vertical 40/20 mm/s" in text
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
        ["--table-mm", "495.9", "322.7", "--no-xy-correction",
         "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
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
                                        ("--hold-s", "0.49"), ("--hold-s", "30.1"),
                                        ("--speed-deg-s", "1.9"),
                                        ("--speed-deg-s", "81"),
                                        ("--accel-deg-s2", "14.9"),
                                        ("--accel-deg-s2", "161"),
                                        ("--accel-deg-s2", "abc")])
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
        ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
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
        ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
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
        ["--table-mm", "495.9", "322.7", "--execute",
         "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
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
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE", arm_mode="posvel",
        speed_deg_s=15.0, accel_deg_s2=30.0,
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
    hover, catchup, _ = _cycle_counts(built)
    stop_at = hover + catchup + 5
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
    hover, catchup, _ = _cycle_counts(built)
    close_start = hover + catchup + (ranges_len(built, "lower")) + 1
    touch_at = close_start + 14
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: ("L1 R1" if n >= touch_at else "-"))
    assert code == 0
    assert "part lifted, holding" in text
    contact_cmd = OPEN_DEG + (touch_at - close_start) * GRIP_FAST_CLOSE_STEP
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
    hover, catchup, _ = _cycle_counts(built)
    close_start = hover + catchup + ranges_len(built, "lower") + 1
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
    hover, catchup, _ = _cycle_counts(built)
    lower_len = ranges_len(built, "lower")
    close_start = hover + catchup + lower_len + 1
    touch_at = close_start + 14
    drop_at = touch_at + 150  # still lifting: lift is 214 cycles now

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
    assert "no part held, holding" in text
    assert "part lifted, holding" not in text
    contact_cmd = OPEN_DEG + (touch_at - close_start) * GRIP_FAST_CLOSE_STEP
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
    hover, catchup, _ = _cycle_counts(built)
    jump_at = hover + catchup + ranges_len(built, "lower") + 30
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
    hover, catchup, _ = _cycle_counts(built)
    interrupt_at = hover + catchup + 10
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
        sensor_factory=lambda: FakeSensor(lambda n: "-"), out=lines.append,
        arm_mode="posvel", speed_deg_s=15.0, accel_deg_s2=30.0)
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


def test_vlims_arm_ten_percent_gripper_quarter():
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: "-")
    assert code == 0
    slow = [v * 0.1 for v in VLIMS]
    assert ("mode", slow) in sdk.groups["arm"].calls
    grip_modes = [call[1] for call in sdk.groups["gripper"].calls if call[0] == "mode"]
    assert grip_modes and grip_modes[0] == pytest.approx([3.0 * 0.25])
    assert "gripper vlim 25 %: 0.75" in text
    assert sdk.loop_rate == LOOP_RATE_HZ == 100.0



def test_gains_soft_passes_hw_yaml_to_factory():
    from pathlib import Path

    sdk = FakeSdk()
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return sdk

    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--gains", "soft",
         "--arm-mode", "posvel"],
        sdk_factory=factory,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0
    (kwargs,) = seen
    runtime = Path(kwargs["hw_yaml"])
    assert runtime.is_file()
    text = "\n".join(lines)
    assert "joint 4: pos_kp" in text
    assert "joint 5: pos_kp" in text


def test_gains_default_is_vel2():
    from pathlib import Path

    sdk = FakeSdk()
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return sdk

    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel"],
        sdk_factory=factory,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0
    (kwargs,) = seen
    runtime = Path(kwargs["hw_yaml"])
    assert runtime.is_file()
    text = "\n".join(lines)
    assert "joint 4: vel_kp 0.0008 -> 0.0016" in text
    assert "joint 5: vel_kp 0.0008 -> 0.0016" in text


def test_gains_explicit_default_calls_factory_without_args():
    sdk = FakeSdk()
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return sdk

    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--gains", "default",
         "--arm-mode", "posvel"],
        sdk_factory=factory,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0
    assert seen == [{}]
    assert "joint 4:" not in "\n".join(lines)


def test_hover_ramp_smooth_start_stop_and_capped():
    from muse_qc.pick_plan import FREE_STEP_DEG, SETTLE_AT_HOVER_CYCLES
    built = _built()
    plan, ranges = built["plan"], built["ranges"]
    low, high = ranges["hover"]
    ramp = [wp for seg, wp, _ in plan[low:high - SETTLE_AT_HOVER_CYCLES]]
    assert ramp and ramp[-1] == built["hover_joints"]
    previous = list(REST)
    peak = 0.0
    for wp in ramp:
        peak = max(peak, max(abs(w - p) for w, p in zip(wp, previous)))
        previous = list(wp)
    assert peak <= FREE_STEP_DEG + 1e-9
    first = max(abs(w - p) for w, p in zip(ramp[0], REST))
    last = max(abs(w - p) for w, p in zip(ramp[-1], ramp[-2]))
    assert first <= 0.01 and last <= 0.01  # smooth start and stop
    assert first < peak and last < peak


def test_lower_fast_above_slow_zone_and_slow_below():
    from muse_qc.pick_plan import SLOW_ZONE_MM
    built = _built()
    plan, ranges = built["plan"], built["ranges"]
    low, high = ranges["lower"]
    points = [built["hover_joints"]] + [wp for _, wp, _ in plan[low:high]]
    zones = [z for z in (_fake_tip(list(wp))[2] * 1000.0 for wp in points)]
    grip_z = 8.0
    fast = slow = 0
    for top, bottom in zip(zones, zones[1:]):
        drop = top - bottom
        assert drop >= -1e-9
        if min(top, bottom) >= grip_z + SLOW_ZONE_MM - 1e-9:
            fast += 1
            assert drop == pytest.approx(0.5)  # 40 mm/s at 100 Hz
        else:
            slow += 1
            assert drop == pytest.approx(0.2)  # 20 mm/s at 100 Hz
    assert (fast, slow) == (67 * 2, 25 * 5)  # entry step included in fast


def test_lift_slow_watch_then_fast():
    from muse_qc.pick_plan import LIFT_SLOW_MM
    built = _built()
    plan, ranges = built["plan"], built["ranges"]
    low, high = ranges["lift"]
    waypoints = [built["grip_joints"]] + [wp for _, wp, _ in plan[low:high]]
    zones = [_fake_tip(list(wp))[2] * 1000.0 for wp in waypoints]
    grip_z = 8.0
    slow = sum(1 for top, bottom in zip(zones, zones[1:])
               if min(top, bottom) < grip_z + LIFT_SLOW_MM - 1e-9)
    assert slow == 10 * 5  # first 10 mm at 20 mm/s for the part-lost watch
    assert all(bottom >= top - 1e-9 for top, bottom in zip(zones, zones[1:]))


def test_contact_in_fast_lower_stops_within_one_cycle_and_climbs():
    from muse_qc.pick_plan import SLOW_ZONE_MM
    built = _built()
    hover, catchup, _ = _cycle_counts(built)
    stop_at = hover + catchup + 5  # still in the fast part of lower
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: ("L1 R1" if n >= stop_at else "-"))
    assert code == 1
    assert "contact while lowering:" in text
    ranges = built["ranges"]
    lower_set = {tuple(math.radians(v) for v in wp)
                 for _, wp, _ in built["plan"][ranges["lower"][0]:ranges["lower"][1]]}
    sends = _arm_sends(sdk)
    assert len({item for item in sends if item in lower_set}) <= 5
    stop_z = _fake_tip(list(built["plan"][ranges["lower"][0] + 3][1]))[2]
    assert stop_z > (8.0 + SLOW_ZONE_MM) / 1000.0  # stopped in the fast zone
    hover_rad = tuple(math.radians(v) for v in built["hover_joints"])
    first_lower = next(i for i, item in enumerate(sends) if item in lower_set)
    hover_back = next(i for i in range(first_lower, len(sends))
                      if sends[i] == hover_rad)
    for item in sends[first_lower:hover_back + 1]:
        deg = [math.degrees(v) for v in item]
        assert _fake_tip(deg)[2] >= stop_z - 1e-9  # climbs, never descends


def test_gripper_fully_open_before_lower_starts():
    sdk = FakeSdk()
    code, _, _ = _run(sdk, lambda n: "-")
    assert code == 0
    built = _built()
    ranges = built["ranges"]
    lower_set = {tuple(math.radians(v) for v in wp)
                 for _, wp, _ in built["plan"][ranges["lower"][0]:ranges["lower"][1]]}
    arm_items, grip_items = _arm_sends(sdk), _grip_sends_deg(sdk)
    assert len(arm_items) == len(grip_items)  # one gripper send per arm cycle
    first_lower = next(i for i, item in enumerate(arm_items) if item in lower_set)
    assert any(abs(value - OPEN_DEG) < 1e-9
               for value in grip_items[:first_lower + 1])
    hover_set = {tuple(math.radians(v) for v in wp)
                 for _, wp, _ in built["plan"][ranges["hover"][0]:ranges["hover"][1]]}
    opened_at = next(i for i, value in enumerate(grip_items)
                     if abs(value - OPEN_DEG) < 1e-9)
    assert opened_at <= first_lower
    assert all(item in hover_set or item == arm_items[0]
               for item in arm_items[:opened_at + 1])  # held hover until open


def test_reference_plan_less_than_half_of_today():
    ref = pick_plan.build_pick_plan(REST, (495.9, 330.0), 8.0, TEST_LIMITS,
                                    solve_fn=_fake_solve)
    assert ref["ok"] is True
    old_total = 20 + 92 * 10 + 92 * 10 + 90 * 10 + 90 * 10 + 120
    assert len(ref["plan"]) * 2 < old_total  # 1132 planned vs 3780 today


def test_speed_deg_s_scales_free_ramps():
    slow = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                     solve_fn=_fake_solve,
                                     free_speed_deg_s=2.0)
    assert slow["ok"] is True
    assert (slow["ranges"]["hover"][1] - slow["ranges"]["hover"][0]) == 57 + 50
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--speed-deg-s", "2",
         "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    assert code == 0
    assert "hover 107" in "\n".join(lines)


def _dry_run_main(extra_args):
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7"] + extra_args,
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        out=lines.append,
    )
    return code, "\n".join(lines)


def test_cli_defaults_mitff_50_100():
    code, text = _dry_run_main([])
    assert code == 0
    assert "arm mode: mit-ff" in text
    assert "speeds: free 50 deg/s, accel 100 deg/s2" in text


def test_cli_posvel_without_speed_gets_15_30():
    code, text = _dry_run_main(["--arm-mode", "posvel"])
    assert code == 0
    assert "speeds: free 15 deg/s, accel 30 deg/s2" in text


def test_cli_posvel_explicit_20_refused_before_connect():
    def factory():
        raise AssertionError("nothing may connect on a refused speed")

    with pytest.raises(SystemExit) as info:
        pick_test.main(["--table-mm", "495.9", "322.7",
                        "--arm-mode", "posvel", "--speed-deg-s", "20"],
                       sdk_factory=factory)
    assert info.value.code == 2


def test_cli_80_160_accepted():
    code, text = _dry_run_main(["--speed-deg-s", "80",
                                "--accel-deg-s2", "160"])
    assert code == 0
    assert "speeds: free 80 deg/s, accel 160 deg/s2" in text
    assert "all waypoints ok" in text


def test_80_160_passes_real_chained_safety():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    safety.set_j6_ref(REST[5])  # as _plan_and_check does before checking
    built = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                      solve_fn=_fake_solve,
                                      free_speed_deg_s=80.0,
                                      accel_deg_s2=160.0, tip_fn=_fake_tip)
    assert built["ok"] is True
    peak = 0.0
    previous = tuple(REST)
    for _, waypoint, _ in built["plan"]:
        peak = max(peak, max(abs(a - b)
                             for a, b in zip(waypoint, previous)))
        previous = tuple(waypoint)
    assert peak <= 10.0  # MAX_STEP_JOINT_DEG per command at 100 Hz
    ok, at, _ = pick_plan.check_pick_plan(
        built["plan"], REST, tip_fn=_fake_tip)
    assert (ok, at) == (True, -1)


def test_grip_speed_profile():
    from muse_qc import grip_speed
    assert grip_speed.OPEN_STEP_DEG == pytest.approx(0.4)
    assert grip_speed.FAST_CLOSE_STEP_DEG == pytest.approx(0.4)
    assert grip_speed.SLOW_CLOSE_STEP_DEG == pytest.approx(0.15)
    assert grip_speed.FAST_CLOSE_UNTIL_MM == pytest.approx(22.0)
    assert grip_speed.GRIPPER_VLIM_FRACTION == pytest.approx(0.25)
    nxt, done = grip_speed.step_toward(0.0, 1.0, 0.4)
    assert nxt == pytest.approx(0.4) and done is False
    assert grip_speed.step_toward(0.9, 1.0, 0.4) == (pytest.approx(1.0), True)
    nxt, done = grip_speed.close_step(-100.0, CLOSED_DEG, 30.0)
    assert nxt == pytest.approx(-99.6) and done is False
    nxt, done = grip_speed.close_step(-68.0, CLOSED_DEG, 21.9)
    assert nxt == pytest.approx(-67.85) and done is False


def _dedup(values):
    """Collapse consecutive duplicate sends (hold plus step send twice)."""
    out = []
    for value in values:
        if not out or abs(value - out[-1]) > 1e-12:
            out.append(value)
    return out


def test_gripper_opens_at_fast_rate():
    sdk = FakeSdk()
    code, _, _ = _run(sdk, lambda n: "-")
    assert code == 0
    grips = _dedup(_grip_sends_deg(sdk))
    opened_at = next(i for i, v in enumerate(grips)
                     if abs(v - OPEN_DEG) < 1e-9)
    diffs = [b - a for a, b in zip(grips[:opened_at], grips[1:opened_at + 1])]
    assert diffs and all(d < 0 for d in diffs)  # open runs toward negative
    assert all(-d <= GRIP_OPEN_STEP + 1e-9 for d in diffs)
    assert all(abs(-d - GRIP_OPEN_STEP) < 1e-9 for d in diffs[:-1])


def test_closing_fast_above_22mm_then_slow():
    from muse_qc.gripper_grip import deg_to_width
    sdk = FakeSdk()
    code, _, _ = _run(sdk, lambda n: "-")  # missed grip: closes fully
    assert code == 0
    grips = _grip_sends_deg(sdk)
    closed_at = next(i for i, v in enumerate(grips)
                     if abs(v - CLOSED_DEG) < 1e-9)
    grips = _dedup(grips)
    closed_at = next(i for i, v in enumerate(grips)
                     if abs(v - CLOSED_DEG) < 1e-9)
    start = max(i for i, v in enumerate(grips[:closed_at + 1])
                if abs(v - OPEN_DEG) < 1e-9)
    cmds = grips[start:closed_at + 1]
    assert len(cmds) > 10
    fast = slow = 0
    for a, b in zip(cmds, cmds[1:]):
        expect = (GRIP_FAST_CLOSE_STEP if deg_to_width(a) > 22.0
                  else GRIP_SLOW_CLOSE_STEP)
        assert 0 < b - a <= expect + 1e-9
        if abs(CLOSED_DEG - b) > expect:
            assert abs((b - a) - expect) < 1e-9
        if deg_to_width(a) > 22.0:
            fast += 1
        else:
            slow += 1
    assert fast > 0 and slow > 0


def test_contact_during_fast_close_stops_and_squeezes():
    built = _built()
    hover, catchup, _ = _cycle_counts(built)
    close_start = hover + catchup + ranges_len(built, "lower") + 1
    touch_at = close_start + 2  # jaw still wider than 22 mm
    sdk = FakeSdk()
    code, text, _ = _run(sdk, lambda n: ("L1 R1" if n >= touch_at else "-"))
    assert code == 0
    assert "contact: gripper commanded deg" in text
    assert "part lifted, holding" in text
    contact_cmd = OPEN_DEG + (touch_at - close_start) * GRIP_FAST_CLOSE_STEP
    squeeze = min(contact_cmd + 2.0, CLOSED_DEG)
    assert any(abs(v - squeeze) < 1e-9 for v in _grip_sends_deg(sdk))


def test_lower_shorter_than_before():
    built = _built()
    low, high = built["ranges"]["lower"]
    assert high - low == 259  # was 384 at 10 mm/s, now 20 mm/s
    assert high - low < 384
