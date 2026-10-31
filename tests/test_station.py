"""Tests for the fixed inspection station: inspect_station, loop, CLI.

Offline only: fake SDK groups, fake touch sensor, fake solve, stub
safety. No bus, no serial port, no camera, no network. Never run
against real devices.
"""

from __future__ import annotations

import math

import pytest

from muse_qc.pick import inspect_station, pick_loop, pick_plan, pick_test
from muse_qc.pick.drop_plan import BOXES_TABLE_MM, DROP_TILT_DEG, RELEASE_OPEN_MM
from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ
from muse_qc.gripper.gripper_grip import width_to_deg

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
OPEN28 = width_to_deg(28.0)
DROP_GOOD = (-3.5, -99.03, -80.0, 75.0, 1.0, -4.0)
DROP_BAD = (-5.0, -99.47, -81.0, 75.0, 1.5, -4.5)
SECRET = "[REDACTED]"


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


def _fake_drop_solve(box_xy, seed):
    """Drop solve stand-in: distinct fixed joints per box."""
    if tuple(float(v) for v in box_xy) == BOXES_TABLE_MM["bad"]:
        joints, table = DROP_BAD, (675.4, 420.3)
    else:
        joints, table = DROP_GOOD, (376.4, 423.8)
    return {
        "ok": True,
        "joints_deg": joints,
        "tip_base_mm": (401.1, 119.5, 170.0),
        "tip_table_mm": (table[0], table[1], 170.0),
        "tilt_deg": DROP_TILT_DEG,
        "error_mm": 0.0,
        "real_tip_z_mm": 130.0,
    }


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

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send", values, [float(v) for v in vlim]))
        self._rad = list(values)


class FakeSdk:
    def __init__(self, joints_deg=REST, grip_deg=REST_GRIP):
        self.groups = {
            "arm": FakeArmGroup(joints_deg),
            "gripper": FakeGripperGroup(grip_deg),
        }
        self.calls = []

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def enable_all(self):
        self.calls.append("enable_all")

    def disable_all(self):
        self.calls.append("disable_all")


class FakeReading:
    def __init__(self, contacts):
        self.contacts = frozenset(contacts)


class FakeSensor:
    """TouchSensor stand-in: scripted contacts per loop cycle."""

    def __init__(self, script):
        self.script = script
        self._reads = 0
        self.closed = False

    def open(self, port):
        self.port = port

    def send(self, cmd):
        pass

    def read_line(self, timeout):
        return "# firmware 2"

    def read(self, timeout):
        self._reads += 1
        if self._reads == 1:
            return None  # setup drain
        return FakeReading(self.script(self._reads - 1))

    def close(self):
        self.closed = True


def _station_kwargs():
    return {
        "drop_joints_by_box": {"good": DROP_GOOD, "bad": DROP_BAD},
        "tip_fn": _fake_tip,
    }


def test_station_joints_use_fixed_j6():
    solved = inspect_station.solve_station_joints(
        REST, TEST_LIMITS, solve_fn=_fake_solve)
    assert solved["ok"] is True
    assert solved["joints_deg"][5] == inspect_station.STATION_J6_DEG == 0.0


def test_station_j6_outside_limits_refuses():
    limits = dict(TEST_LIMITS, joint6=(10.0, 180.0))
    solved = inspect_station.solve_station_joints(
        REST, limits, solve_fn=_fake_solve)
    assert solved["ok"] is False
    assert "joint 6 limit" in solved["reason"]


def _fail_at_station(table_xy, height_mm, seed):
    """Fake solve that only refuses near the station table point."""
    dist = math.dist((float(table_xy[0]), float(table_xy[1])),
                     inspect_station.STATION_TABLE_MM)
    if dist < 60.0:
        return {"ok": False, "reason": "unreachable: IK did not converge"}
    return _fake_solve(table_xy, height_mm, seed)


def test_unreachable_station_refuses_plan():
    built = inspect_station.build_inspect_plan_with_station(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fail_at_station,
        use_station=True, **_station_kwargs())
    assert built["ok"] is False
    assert "station unreachable" in built["reason"]


def test_station_on_branches_start_at_station_and_all_checked():
    built = inspect_station.build_inspect_plan_with_station(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        use_station=True, **_station_kwargs())
    assert built["ok"] is True
    assert list(built["shared_ranges"]) == ["hover", "lower", "lift",
                                            "to-station"]
    station = built["station_joints"]
    assert station[5] == inspect_station.STATION_J6_DEG
    seg_low, seg_high = built["shared_ranges"]["to-station"]
    assert built["plan"][seg_high - 1][1] == station
    for box in ("good", "bad"):
        branch = built["ranges_by_box"][box]
        assert list(branch) == ["to-box", "release", "return"]
        low, high = branch["to-box"]
        assert low == seg_high or low > seg_high
    seen = []

    def recording(target, current, z_min_m=None):
        seen.append(tuple(target.joints_deg))
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    ok, at, _ = pick_plan.check_pick_plan(built["plan"], REST,
                                          check_fn=recording, tip_fn=_fake_tip)
    assert (ok, at) == (True, -1)
    assert len(seen) == len(built["plan"])

    def refuse_station(target, current, z_min_m=None):
        if tuple(target.joints_deg) == station:
            return {"ok": False, "reason": "nope",
                    "value": "station", "limit": "refused"}
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    ok, at, res = pick_plan.check_pick_plan(built["plan"], REST,
                                            check_fn=refuse_station,
                                            tip_fn=_fake_tip)
    assert ok is False and res["reason"] == "nope"
    assert seg_low <= at < seg_high


def test_station_off_plan_equals_today():
    with_station = inspect_station.build_inspect_plan_with_station(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        use_station=False, **_station_kwargs())
    today = pick_plan.build_inspect_plan(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        drop_joints_by_box={"good": DROP_GOOD, "bad": DROP_BAD},
        tip_fn=_fake_tip)
    assert with_station == today


def _station_loop(script, lines, **overrides):
    """Build the station plan and loop; the script may read holder["loop"]."""
    built = inspect_station.build_inspect_plan_with_station(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        use_station=True, **_station_kwargs())
    assert built["ok"] is True
    arm, grip = FakeArmGroup(REST), FakeGripperGroup(REST_GRIP)
    tips = [_fake_tip(list(wp)) for _, wp, _ in built["plan"]]
    ranges = dict(built["shared_ranges"])
    ranges.update(built["ranges_by_box"]["good"])
    release_open = width_to_deg(RELEASE_OPEN_MM)
    loop = pick_loop.PickLoop(
        arm, grip, VLIMS, [0.3], [math.radians(v) for v in REST],
        built["plan"], tips, ranges, tips[ranges["hover"][1] - 1][2],
        OPEN28, REST_GRIP, 50, False, FakeSensor(script), lines.append,
        _always_ok, _fake_tip, None, release_open, True,
        built["ranges_by_box"], built["shared_ranges"])
    for key, value in overrides.items():
        setattr(loop, key, value)
    return built, arm, grip, loop


def _step_until(loop, condition, bound=30000):
    for _ in range(bound):
        loop(None, 1.0 / LOOP_RATE_HZ)
        if condition():
            break


def _grip_script(holder):
    def script(n):
        loop = holder.get("loop")
        phase = loop.phase if loop is not None else "hover"
        if phase in ("close", "squeeze", "squeeze_hold", "lift",
                     "top_hold", "to-station", "to-box"):
            return {"L1", "R1"}
        return set()

    return script


def _arm_sends(arm):
    return [tuple(call[1]) for call in arm.calls if call[0] == "send"]


def test_loop_walks_to_station_before_inspect_hold():
    holder, lines = {}, []
    built, arm, grip, loop = _station_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    assert loop.phase == "inspect_hold"
    station = built["station_joints"]
    assert loop.pos == built["shared_ranges"]["to-station"][1] - 1
    sends = _arm_sends(arm)
    station_rad = tuple(math.radians(v) for v in station)
    assert station_rad in sends
    assert "to-station" in "\n".join(
        f"{seg}" for seg, _, _ in built["plan"])


def test_loop_verdict_good_sorts_from_station():
    holder, lines = {}, []
    built, arm, grip, loop = _station_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    station_rad = tuple(math.radians(v) for v in built["station_joints"])
    assert station_rad in _arm_sends(arm)
    loop.set_verdict("good")
    _step_until(loop, lambda: loop.done.is_set())
    assert loop.exit_code == 0
    assert "dropped into the good box" in "\n".join(lines)


def test_abort_at_station_ends_at_rest_gripper_unchanged():
    holder, lines = {}, []
    built, arm, grip, loop = _station_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    assert loop.phase == "inspect_hold"
    held_grip = loop.grip_cmd
    grip.calls.clear()
    loop.abort()
    _step_until(loop, lambda: loop.done.is_set())
    assert loop.done.is_set()
    assert loop.exit_code == 1
    rest_rad = tuple(math.radians(v) for v in built["rest_joints"])
    sends = _arm_sends(arm)
    assert sends and sends[-1] == rest_rad
    grip_sends = [call[1][0] for call in grip.calls
                  if call[0] == "send"]
    assert grip_sends, "gripper must keep holding during the abort"
    assert all(value == math.radians(held_grip) for value in grip_sends)


def _station_main_args(extra):
    return ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel"] + extra


def _station_dry_kwargs(lines, **overrides):
    kwargs = {
        "check_fn": _always_ok,
        "tip_fn": _fake_tip,
        "solve_fn": _fake_solve,
        "joint_limits": TEST_LIMITS,
        "find_port": lambda: "/dev/ttyFAKE",
        "sensor_factory": lambda: FakeSensor(lambda n: "-"),
        "solve_drop_fn": _fake_drop_solve,
        "find_arm_node": lambda: "/dev/videoFAKE",
        "read_key": lambda: SECRET,
        "out": lines.append,
    }
    kwargs.update(overrides)
    return kwargs


def test_dry_run_station_prints_line_and_segment():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        _station_main_args(["--inspect", "--station"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(lines),
    )
    assert code == 0
    text = "\n".join(lines)
    assert ("station: table mm (235.0, 150.0), tip z 100.0 mm,"
            " joint 6 +0.0 deg") in text
    seg_line = next(line for line in lines
                    if line.startswith("segments at 100 Hz:"))
    assert "to-station" in seg_line
    assert "all waypoints ok" in text
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_dry_run_station_off_has_no_station_lines():
    sdk = FakeSdk()
    off = []
    assert pick_test.main(
        _station_main_args(["--inspect", "--no-station"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(off)) == 0
    assert not any(line.startswith("station:") for line in off)
    assert not any("to-station" in line for line in off
                   if line.startswith("segments at 100 Hz:"))


def test_dry_run_default_plans_with_station():
    default, explicit = [], []
    sdk = FakeSdk()
    assert pick_test.main(
        _station_main_args(["--inspect"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(default)) == 0
    assert pick_test.main(
        _station_main_args(["--inspect", "--station"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(explicit)) == 0
    assert default == explicit
    text = "\n".join(default)
    assert ("station: table mm (235.0, 150.0), tip z 100.0 mm,"
            " joint 6 +0.0 deg") in text
    seg_line = next(line for line in default
                    if line.startswith("segments at 100 Hz:"))
    assert "to-station" in seg_line
    assert "all waypoints ok" in text
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_dry_run_unreachable_station_refuses_before_moving():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        _station_main_args(["--inspect", "--station"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(lines, solve_fn=_fail_at_station),
    )
    assert code == 1
    assert "station unreachable" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_station_j6_option_sets_joint_and_prints():
    solved = inspect_station.solve_station_joints(
        REST, TEST_LIMITS, solve_fn=_fake_solve, station_j6_deg=15.0)
    assert solved["ok"] is True
    assert solved["joints_deg"][5] == 15.0
    built = inspect_station.build_inspect_plan_with_station(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        use_station=True, station_j6_deg=15.0, **_station_kwargs())
    assert built["ok"] is True
    assert built["station_joints"][5] == 15.0
    sdk, lines = FakeSdk(), []
    code = pick_test.main(
        _station_main_args(["--inspect", "--station", "--station-j6", "15"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(lines),
    )
    assert code == 0
    assert "joint 6 +15.0 deg" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_station_j6_outside_limits_refuses_before_moving():
    sdk, lines = FakeSdk(), []
    code = pick_test.main(
        _station_main_args(["--inspect", "--station", "--station-j6", "200"]),
        sdk_factory=lambda **kwargs: sdk,
        **_station_dry_kwargs(lines),
    )
    assert code == 1
    assert "joint 6 limit" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_station_j6_without_station_is_refused():
    sdk, lines = FakeSdk(), []
    with pytest.raises(SystemExit) as exc:
        pick_test.main(
            _station_main_args(["--inspect", "--station-j6", "15"]),
            sdk_factory=lambda **kwargs: sdk,
            **_station_dry_kwargs(lines),
        )
    assert exc.value.code == 2
