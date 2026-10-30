"""Tests for pick_test --arm-mode mit-ff (muse_qc.pick_mit).

Offline only: fake SDK groups, fake TouchSensor, fake solve_hover,
fake gravity. No bus, no serial port, no camera, no network.
"""

from __future__ import annotations

import math

import pytest

from muse_qc import pick_mit, pick_test
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.gripper_grip import width_to_deg
from muse_qc.mit_hold import MIT_KD, MIT_KP, TAU_SCALE

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


def _fake_drop_solve(box_xy, seed_deg):
    """Fixed drop joints, independent of the box.

    Joint 2 at -100 deg keeps the fake tip at 100 mm, above the
    to-box transit floor that build_drop_segments enforces.
    """
    return {
        "ok": True,
        "joints_deg": (-30.0, -100.0, -60.0, 50.0, 10.0, -4.0),
        "tip_base_mm": (300.0, 0.0, 120.0),
        "tip_table_mm": (300.0, 0.0, 120.0),
        "error_mm": 0.0,
        "tilt_deg": 0.0,
        "real_tip_z_mm": 100.0,
        "iterations": 1,
    }


def _fake_tip(joints_deg):
    """Tip z from joint 2, matching _fake_solve: z = height in metres."""
    height = -(float(joints_deg[1]) + 90.0) * 10.0
    return (0.30, 0.0, height / 1000.0)


def _always_ok(target, current, z_min_m=None):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _q_gravity(joint_rad):
    """Fake gravity: torque follows the first commanded joint."""
    return [2.0 * float(joint_rad[0])] + [0.0] * 5


class FakeArmGroup:
    """Fake arm group speaking both POS_VEL and MIT, following sends."""

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
        self.calls.append(("mode_posvel", [float(v) for v in vlim]))

    def mode_mit(self, kp=None, kd=None):
        self.calls.append("mode_mit")

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send_posvel", values, [float(v) for v in vlim]))
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
        self._rad = list(values)


class LagArmGroup(FakeArmGroup):
    """Fake arm group whose readings lag the command on one joint.

    The first read is exact, so the plan builds from the true start
    pose; every later reading trails the last command by lag_deg on
    one joint and the tracking guard trips mid-walk.
    """

    def __init__(self, joints_deg, lag_joint=1, lag_deg=6.0, exact_reads=1):
        super().__init__(joints_deg)
        self._lag = [0.0] * 6
        self._lag[lag_joint] = math.radians(lag_deg)
        self._reads = 0
        self._exact_reads = exact_reads

    def get_positions(self):
        self.calls.append("get")
        self._reads += 1
        if self._reads <= self._exact_reads:
            return list(self._rad)
        return [c - lag for c, lag in zip(self._rad, self._lag)]


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


class FakeSdk:
    def __init__(self, joints_deg=REST, grip_deg=REST_GRIP, arm_group=None):
        self.groups = {
            "arm": arm_group if arm_group is not None
            else FakeArmGroup(joints_deg),
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


def _run(sdk, script, **kwargs):
    kwargs.setdefault("input_fn", lambda prompt: "yes")
    kwargs.setdefault("check_fn", _always_ok)
    kwargs.setdefault("tip_fn", _fake_tip)
    kwargs.setdefault("solve_fn", _fake_solve)
    kwargs.setdefault("arm_vlims", VLIMS)
    kwargs.setdefault("joint_limits", TEST_LIMITS)
    kwargs.setdefault("find_port", lambda: "/dev/ttyFAKE")
    kwargs.setdefault("sensor_factory", lambda: FakeSensor(script))
    kwargs.setdefault("hold_s", 0.5)
    kwargs.setdefault("gravity_fn", _q_gravity)
    lines = []
    kwargs.setdefault("out", lines.append)
    result = pick_test.execute(sdk, TABLE, 8.0, 40.0, False, **kwargs)
    return result, "\n".join(str(line) for line in lines)


def _arm_sends(sdk, name):
    return [call for call in sdk.groups["arm"].calls if call[0] == name]


def _command_step_check(arm, limit=1.0):
    """Refuse a command further than limit from the last command.

    Before the first send (the chained pre-run plan check) compare
    against the chained current waypoint instead; during the loop the
    lagging arm still receives every command, so the check sees the
    plan continuity (not the lagged reading): micro-steps pass, a 6
    deg jump after the hold is refused and needs the abort ramp.
    """

    def check(target, current, z_min_m=None):
        if any(call[0] in ("send_mit", "send_posvel") for call in arm.calls):
            last = [math.degrees(v) for v in arm._rad]
        else:
            last = [float(v) for v in current.joints_deg]
        peak = max(abs(t - c) for t, c in zip(target.joints_deg, last))
        if peak > limit:
            return {"ok": False, "reason": "step-joint",
                    "value": peak, "limit": limit}
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    return check


def test_posvel_explicit_sends_and_output_unchanged():
    sdk = FakeSdk()
    code, text = _run(sdk, lambda n: "-", arm_mode="posvel")
    assert code == 0
    assert text.splitlines()[0] == "arm mode: posvel"
    assert "mode_mit" not in sdk.groups["arm"].calls
    assert _arm_sends(sdk, "send_mit") == []
    assert _arm_sends(sdk, "send_posvel"), "posvel must keep sending"
    grips = sdk.groups["gripper"].calls
    assert any(call[0] == "mode" for call in grips)
    assert any(call[0] == "send" for call in grips)


def test_defaults_are_mitff_and_grip_z_8():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7"],
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        solve_drop_fn=_fake_drop_solve, joint_limits=TEST_LIMITS,
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        gravity_fn=_q_gravity, out=lines.append,
    )
    assert code == 0
    text = "\n".join(str(line) for line in lines)
    assert "arm mode: mit-ff" in text
    assert "grip tip z: 8.0 mm" in text


def test_mitff_sends_only_mit_with_scaled_tau():
    sdk = FakeSdk()
    code, text = _run(sdk, lambda n: "-", arm_mode="mit-ff")
    assert code == 0
    assert text.splitlines()[0] == "arm mode: mit-ff"
    group = sdk.groups["arm"]
    assert "mode_mit" in group.calls
    assert group.calls.index("mode_mit") < group.calls.index(
        next(call for call in group.calls if call[0] == "send_mit"))
    assert [call for call in group.calls if call[0] == "mode_posvel"] == []
    assert [call for call in group.calls if call[0] == "send_posvel"] == []
    sends = _arm_sends(sdk, "send_mit")
    assert sends, "arm commands must go through send_mit"
    assert sends[0][2] == pytest.approx([0.0] * 6)  # first send
    previous = sends[0][1]
    for _, pos, vel, kp, kd, _tau in sends[1:]:
        assert vel == pytest.approx(
            [(c - p) * LOOP_RATE_HZ for c, p in zip(pos, previous)])
        assert kp == pytest.approx(list(MIT_KP))
        assert kd == pytest.approx(list(MIT_KD))
        previous = pos
    for _, pos, _vel, _kp, _kd, tau in sends:
        assert tau[0] == pytest.approx(TAU_SCALE[0] * 2.0 * pos[0])
        assert tau[1:] == pytest.approx([0.0] * 5)
    grips = sdk.groups["gripper"].calls
    assert any(call[0] == "mode" for call in grips)
    assert any(call[0] == "send" for call in grips)
    assert "enable_all" in sdk.calls and "disable_all" in sdk.calls


def test_mitff_first_command_is_measured_start_pose():
    sdk = FakeSdk()
    code, _ = _run(sdk, lambda n: "-", arm_mode="mit-ff")
    assert code == 0
    first = _arm_sends(sdk, "send_mit")[0]
    assert first[1] == pytest.approx([math.radians(v) for v in REST])


def test_tracking_trip_holds_measured_pose_and_returns():
    sdk = FakeSdk(arm_group=LagArmGroup(REST, lag_joint=1, lag_deg=6.0))
    code, text = _run(sdk, lambda n: "-", arm_mode="mit-ff")
    assert code == 1
    assert "tracking error on joint 2, holding" in text
    group = sdk.groups["arm"]
    assert [call for call in group.calls if call[0] == "send_posvel"] == []
    assert _arm_sends(sdk, "send_mit"), "the hold keeps sending with feed-forward"
    rest_rad = tuple(math.radians(v) for v in REST)
    last = _arm_sends(sdk, "send_mit")[-1][1]
    # Within the trim frozen at the trip (under 1 deg on joint 2).
    assert tuple(last) == pytest.approx(rest_rad, abs=math.radians(1.0))
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls


def test_abort_ramp_bridges_the_lag_without_a_jump():
    sdk = FakeSdk(arm_group=LagArmGroup(REST, lag_joint=1, lag_deg=6.0))
    check = _command_step_check(sdk.groups["arm"])
    code, text = _run(sdk, lambda n: "-", arm_mode="mit-ff", check_fn=check)
    assert code == 1
    assert "tracking error on joint 2, holding" in text
    sends = [call[1] for call in _arm_sends(sdk, "send_mit")]
    assert len(sends) > 100  # walk, 0.5 s hold, ramp, detour, return

    def peak_deg(before, after):
        return (max(abs(b - a) for a, b in zip(before, after))
                * 180.0 / math.pi)

    big = [i for i, (before, after) in enumerate(zip(sends, sends[1:]))
           if peak_deg(before, after) >= 2.0]
    # Only the trip hold-back commands a jump (back to the measured
    # pose, where the arm actually is); without the ramp the first
    # abort step would add a second 6 deg jump or refuse outright.
    assert len(big) == 1
    tail = sends[big[0] + 1:]
    assert all(peak_deg(before, after) < 2.0
               for before, after in zip(tail, tail[1:]))
    rest_rad = [math.radians(v) for v in REST]
    # Within the trim frozen at the trip (under 1 deg on joint 2).
    assert sends[-1] == pytest.approx(rest_rad, abs=math.radians(1.0))


def test_guard_trips_once_after_ten_bad_cycles_in_a_row():
    guard = pick_mit.TrackingGuard()
    bad = [v + (6.0 if i == 1 else 0.0) for i, v in enumerate(REST)]
    for _ in range(9):
        assert guard.feed(bad, REST) is None
    assert guard.feed(REST, REST) is None  # one good cycle resets
    for _ in range(9):
        assert guard.feed(bad, REST) is None
    assert guard.feed(bad, REST) == 2
    assert guard.feed(bad, REST) is None  # trips once only


def test_begin_switches_to_mit_before_enable():
    group = FakeArmGroup(REST)
    sender = pick_mit.MitArmSender(group, gravity_fn=_q_gravity)
    start_rad = [math.radians(v) for v in REST]
    assert sender.begin(start_rad) == pytest.approx(start_rad)
    order = [call if isinstance(call, str) else call[0] for call in group.calls]
    assert order[0] == "mode_mit"
    assert order[1] == "send_mit"
    first = [call for call in group.calls if call[0] == "send_mit"][0]
    assert first[1] == pytest.approx(start_rad)


def test_bad_tau_scale_raises_before_anything_is_enabled():
    group = FakeArmGroup(REST)
    with pytest.raises(ValueError):
        pick_mit.MitArmSender(group, gravity_fn=_q_gravity,
                              tau_scale=(1.0,) * 5)
    with pytest.raises(ValueError):
        pick_mit.MitArmSender(group, gravity_fn=_q_gravity,
                              tau_scale=(1.0, 1.0, 1.0, 2.5, 1.0, 1.0))
    assert group.calls == []


def test_tau_scale_with_posvel_exits_2_before_connect():
    def factory():
        raise AssertionError("must not connect on --tau-scale with posvel")

    with pytest.raises(SystemExit) as info:
        pick_test.main(
            ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel",
             "--tau-scale", "1.0", "1.0", "1.4", "1.0", "1.0", "1.0"],
            sdk_factory=factory,
            out=lambda line: None,
        )
    assert info.value.code == 2


def test_dry_run_prints_three_gravity_blocks():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--arm-mode", "mit-ff"],
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        solve_drop_fn=_fake_drop_solve, joint_limits=TEST_LIMITS,
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        gravity_fn=_q_gravity, out=lines.append,
    )
    assert code == 0
    text = "\n".join(str(line) for line in lines)
    assert "arm mode: mit-ff" in text
    assert "mit kp: 120 120 120 18 18 18" in text
    assert "tau scale: 1.00 1.05 1.45 1.00 1.00 1.00" in text
    assert "hover pose gravity:" in text
    assert "grip pose gravity:" in text
    assert "drop (good) pose gravity:" in text
    assert text.count("gravity torque:") == 18
    assert "enable_all" not in sdk.calls
    assert _arm_sends(sdk, "send_mit") == []


def test_dry_run_drop_box_selects_the_pose():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--arm-mode", "mit-ff",
         "--drop", "bad"],
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        solve_drop_fn=_fake_drop_solve, joint_limits=TEST_LIMITS,
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        gravity_fn=_q_gravity, out=lines.append,
    )
    assert code == 0
    text = "\n".join(str(line) for line in lines)
    assert "drop (bad) pose gravity:" in text
    assert text.count("gravity torque:") == 18


class HangArmGroup(FakeArmGroup):
    """Readings hang a constant offset below the last command."""

    def __init__(self, joints_deg, hang_joint=3, hang_deg=3.0):
        super().__init__(joints_deg)
        self._hang = [0.0] * 6
        self._hang[hang_joint] = math.radians(hang_deg)

    def get_positions(self):
        self.calls.append("get")
        return [c - hang for c, hang in zip(self._rad, self._hang)]


def _trim_values(sender):
    return [float(v) for v in sender.trim_line().split(":")[1].split()]


def _drive(sender, group, target_deg, cycles):
    target_rad = [math.radians(v) for v in target_deg]
    for _ in range(cycles):
        measured = [math.degrees(v) for v in group.get_positions()]
        sender.observe(measured, list(target_deg))
        sender.send(target_rad)


def test_trim_converges_to_hang_offset():
    group = HangArmGroup(REST, hang_joint=3, hang_deg=3.0)
    sender = pick_mit.MitArmSender(group, gravity_fn=lambda q: [0.0] * 6)
    _drive(sender, group, REST, 300)
    commanded = [call[1] for call in group.calls if call[0] == "send_mit"][-1]
    assert commanded[3] == pytest.approx(
        math.radians(REST[3] + 3.0), abs=math.radians(0.2))
    assert all(abs(v) <= pick_mit.TRIM_LIMIT_DEG + 1e-9
               for v in _trim_values(sender))


def test_trim_clamps_at_limit():
    group = HangArmGroup(REST, hang_joint=3, hang_deg=10.0)
    sender = pick_mit.MitArmSender(group, gravity_fn=lambda q: [0.0] * 6)
    _drive(sender, group, REST, 600)
    assert _trim_values(sender)[3] == pytest.approx(pick_mit.TRIM_LIMIT_DEG)
    commanded = [call[1] for call in group.calls if call[0] == "send_mit"][-1]
    assert commanded[3] == pytest.approx(
        math.radians(REST[3]) + math.radians(pick_mit.TRIM_LIMIT_DEG))


def test_tau_uses_target_pose_not_trimmed_pose():
    group = HangArmGroup(REST, hang_joint=0, hang_deg=2.0)
    sender = pick_mit.MitArmSender(group, gravity_fn=_q_gravity)
    _drive(sender, group, REST, 100)
    assert _trim_values(sender)[0] != pytest.approx(0.0)  # trim is active
    target_rad = [math.radians(v) for v in REST]
    last = [call for call in group.calls if call[0] == "send_mit"][-1]
    assert last[5][0] == pytest.approx(
        TAU_SCALE[0] * 2.0 * target_rad[0])


class LagRateArmGroup(FakeArmGroup):
    """Reading trails the command by kd*v/kp of the command velocity."""

    def __init__(self, joints_deg, kp=18.0, kd=2.0, lag_joint=3):
        super().__init__(joints_deg)
        self._prev = list(self._rad)
        self._kp, self._kd, self._joint = kp, kd, lag_joint

    def send_mit(self, pos, vel=None, kp=None, kd=None, tau=None):
        self._prev = list(self._rad)
        super().send_mit(pos, vel=vel, kp=kp, kd=kd, tau=tau)

    def get_positions(self):
        self.calls.append("get")
        reading = list(self._rad)
        vel = (self._rad[self._joint] - self._prev[self._joint]) * LOOP_RATE_HZ
        reading[self._joint] -= self._kd * vel / self._kp
        return reading


def test_fast_ramp_leaves_trim_unchanged():
    group = LagRateArmGroup(REST)
    sender = pick_mit.MitArmSender(group, gravity_fn=lambda q: [0.0] * 6)
    target = [math.radians(v) for v in REST]
    sender.begin(target)
    stepped = list(target)
    stepped[3] += math.radians(0.15)  # one 15 deg/s step on joint 4
    sender.send(stepped)
    reading = group.get_positions()
    assert (stepped[3] - reading[3]) == pytest.approx(
        2.0 * math.radians(15.0) / 18.0)  # the kd*v/kp lag is real here
    target = stepped
    for _ in range(200):
        stepped = list(target)
        stepped[3] += math.radians(0.15)
        measured = [math.degrees(v) for v in group.get_positions()]
        sender.observe(measured, [math.degrees(v) for v in target])
        sender.send(stepped)
        target = stepped
    assert sender.trim_line() == "mit trim deg: " + "+0.00 " * 5 + "+0.00"
    last = [call for call in group.calls if call[0] == "send_mit"][-1]
    assert last[1][3] == pytest.approx(target[3])  # no trim wound up


def test_vel_feedforward_matches_target_step_without_trim():
    group = FakeArmGroup(REST)
    sender = pick_mit.MitArmSender(group, gravity_fn=lambda q: [0.0] * 6)
    base = [math.radians(v) for v in REST]
    sender.begin(base)
    assert [call for call in group.calls
            if call[0] == "send_mit"][0][2] == pytest.approx([0.0] * 6)
    sender.observe(
        [v - (2.0 if i == 3 else 0.0) for i, v in enumerate(REST)], REST)
    assert _trim_values(sender)[3] == pytest.approx(0.02)  # trim is active
    stepped = list(base)
    stepped[3] += math.radians(0.15)
    sender.send(stepped)
    last = [call for call in group.calls if call[0] == "send_mit"][-1]
    assert last[2][3] == pytest.approx(math.radians(0.15) * LOOP_RATE_HZ)
    assert last[2][:3] + last[2][4:] == pytest.approx([0.0] * 5)
    assert last[1][3] == pytest.approx(
        stepped[3] + math.radians(0.02))  # position carries the trim
    sender.send(stepped)  # holding
    held = [call for call in group.calls if call[0] == "send_mit"][-1]
    assert held[2] == pytest.approx([0.0] * 6)


def test_freeze_stops_trim_updates():
    group = HangArmGroup(REST, hang_joint=3, hang_deg=3.0)
    sender = pick_mit.MitArmSender(group, gravity_fn=lambda q: [0.0] * 6)
    _drive(sender, group, REST, 50)
    sender.freeze()
    before = sender.trim_line()
    _drive(sender, group, REST, 50)
    assert sender.trim_line() == before


def test_observe_ignores_non_finite_readings():
    group = FakeArmGroup(REST)
    sender = pick_mit.MitArmSender(group, gravity_fn=lambda q: [0.0] * 6)
    bad = list(REST)
    bad[2] = float("nan")
    sender.observe(bad, REST)
    worse = list(REST)
    worse[4] = float("inf")
    sender.observe(REST, worse)
    assert sender.trim_line() == "mit trim deg: " + "+0.00 " * 5 + "+0.00"


def test_trim_line_printed_only_in_mitff():
    sdk = FakeSdk()
    _, text = _run(sdk, lambda n: "-", arm_mode="posvel")
    assert "mit trim" not in text
    sdk = FakeSdk()
    _, text = _run(sdk, lambda n: "-", arm_mode="mit-ff")
    assert "mit trim deg:" in text
    assert text.index("mit trim deg:") < text.index("stopping:")


def test_maybe_ramp_inserts_checked_micro_steps():
    from muse_qc.pick_loop import PickLoop
    from muse_qc.pick_plan import FREE_STEP_DEG

    plan = [("hover", tuple(REST), False)] * 4
    tips = [(0.30, 0.0, 0.100)] * 4
    ranges = {"hover": (0, 4), "return": (0, 4)}
    arm = FakeArmGroup(REST)
    loop = PickLoop(
        arm, FakeGripperGroup(REST_GRIP), VLIMS, [0.3],
        [math.radians(v) for v in REST], plan, tips, ranges, 0.100,
        OPEN_DEG, REST_GRIP, 50, True, FakeSensor(lambda n: "-"),
        out=lambda line: None, check_fn=_command_step_check(arm),
        tip_fn=_fake_tip)
    loop.pos = 3
    loop.detour = [3]
    loop.cmd_arm_rad = [math.radians(v) for v in REST]
    measured = [v - (6.0 if i == 1 else 0.0) for i, v in enumerate(REST)]
    loop._abort_from = list(measured)
    arm._rad = [math.radians(v) for v in measured]  # fault hold end state
    pick_mit.maybe_ramp(loop)
    assert len(loop.detour) > 1  # the 6 deg jump is refused, ramp first
    assert loop.detour[-1] == 3
    assert len(loop.plan) == len(loop.tips)
    previous = list(measured)
    for index in loop.detour:
        _, waypoint, _ = loop.plan[index]
        peak = max(abs(w - p) for w, p in zip(waypoint, previous))
        assert peak <= FREE_STEP_DEG + 1e-9
        previous = list(waypoint)
    for index in loop.detour:  # every ramp step passes the check live
        assert loop._walk_send(index) is True
