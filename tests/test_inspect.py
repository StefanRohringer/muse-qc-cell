"""Tests for gripper inspection and sorting: --inspect, inspect_step, count_frame.

Offline only: fake SDK groups, fake touch sensor, fake arm camera,
fake SAM client, fake API key. No bus, no serial port, no camera,
no network. Never run against real devices.
"""

from __future__ import annotations

import ast
import math
import threading
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy
import pytest

from muse_qc import (
    count_clips,
    inspect_step,
    pick_loop,
    pick_plan,
    pick_test,
    safety,
)
from muse_qc.drop_plan import (
    BOXES_TABLE_MM,
    DROP_TILT_DEG,
    RELEASE_OPEN_MM,
    solve_drop,
)
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.gripper_grip import width_to_deg

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
DROP_GOOD = (-3.5, -99.03, -80.0, 75.0, 1.0, -4.0)  # fake tip z 90 mm
DROP_BAD = (-5.0, -99.47, -81.0, 75.0, 1.5, -4.5)  # fake tip z 95 mm
SECRET = "[REDACTED]"

# Held-part fixture near EXPECTED_PART_PX (342, 290): the good part box
# from the human 2026-10-30 gripper test, plus a gripper-zone box that
# the zone filter must ignore.
PART_AND_GRIPPER = (
    "8<|box;x1=423;y1=402;x2=456;y2=429;w=33;h=27;c=0.76|>,"
    "7<|box;x1=323;y1=268;x2=362;y2=312;w=39;h=44;c=0.75|>"
)
THREE_BARS = (
    "1<|box;x1=158;y1=206;x2=210;y2=457;w=52;h=251;c=0.70|>,"
    "2<|box;x1=237;y1=206;x2=289;y2=457;w=52;h=251;c=0.70|>,"
    "0<|box;x1=313;y1=213;x2=363;y2=464;w=50;h=251;c=0.68|>"
)
TWO_BARS = (
    "2<|box;x1=241;y1=201;x2=290;y2=457;w=49;h=256;c=0.77|>,"
    "0<|box;x1=162;y1=205;x2=212;y2=458;w=50;h=253;c=0.75|>"
)


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


class FakeSdk:
    def __init__(self, joints_deg=REST, grip_deg=REST_GRIP):
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


class ThreadedFakeSdk(FakeSdk):
    """Fake SDK with a background control loop thread, like the real bus."""

    def __init__(self, joints_deg=REST, grip_deg=REST_GRIP, holder=None):
        super().__init__(joints_deg, grip_deg)
        self._holder = holder
        self._thread = None

    def start_control_loop(self, fn, rate=None):
        self.calls.append("loop")
        self.loop_rate = rate
        if self._holder is not None:
            self._holder["loop"] = fn

        def _spin():
            for _ in range(30000):
                fn(self, 1.0 / rate if rate else 0.01)
                if fn.done.is_set():
                    break

        self._thread = threading.Thread(target=_spin, daemon=True)
        self._thread.start()

    def stop_control_loop(self):
        self.calls.append("stop")
        if self._thread is not None:
            self._thread.join(timeout=60)


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


class _SeqClient:
    """Fake SAM client with one answer text per call."""

    def __init__(self, texts, error=None):
        self._texts = list(texts)
        self._error = error
        self.calls = 0
        self.responses = self

    def create(self, **kwargs):
        self.calls += 1
        if self._error is not None:
            raise self._error
        index = min(self.calls - 1, len(self._texts) - 1)
        return SimpleNamespace(output_text=self._texts[index])


def _arm_frame():
    """640x480 frame like the real arm camera image."""
    frame = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
    frame[187:233, 317:352] = (0, 140, 255)
    return frame


def _inspect_main_args(extra):
    return ["--table-mm", "495.9", "322.7", "--arm-mode", "posvel"] + extra


def _inspect_dry_kwargs(lines, **overrides):
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


def test_inspect_and_drop_together_exit_2():
    def factory(**kwargs):
        raise AssertionError("nothing may connect on a bad argument")

    with pytest.raises(SystemExit) as info:
        pick_test.main(
            _inspect_main_args(["--drop", "good", "--inspect"]),
            sdk_factory=factory,
        )
    assert info.value.code == 2


def test_dry_run_inspect_prints_both_boxes_and_never_enables():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        _inspect_main_args(["--inspect"]),
        sdk_factory=lambda **kwargs: sdk,
        **_inspect_dry_kwargs(lines),
    )
    assert code == 0
    text = "\n".join(lines)
    assert "inspect: camera /dev/videoFAKE" in text
    assert "inspect: api key present" in text
    assert SECRET not in text
    assert "drop box: good at table mm (+376.4, +423.8)" in text
    assert "drop box: bad at table mm (+675.4, +420.3)" in text
    assert "to-box lowest tip z:" in text
    assert "all waypoints ok" in text
    assert "dry run: moves nothing, motors never enabled" in text
    assert "connect" in sdk.calls and "disconnect" in sdk.calls
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    for group in sdk.groups.values():
        assert not any(isinstance(call, tuple) and call[0] in ("mode", "send")
                       for call in group.calls)


def test_dry_run_inspect_missing_camera_exits_2_without_enabling():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        _inspect_main_args(["--inspect"]),
        sdk_factory=lambda **kwargs: sdk,
        **_inspect_dry_kwargs(lines, find_arm_node=lambda: None),
    )
    assert code == 2
    assert "arm camera not found" in "\n".join(lines)
    assert "connect" not in sdk.calls
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_dry_run_inspect_missing_key_exits_2_without_enabling():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        _inspect_main_args(["--inspect"]),
        sdk_factory=lambda **kwargs: sdk,
        **_inspect_dry_kwargs(lines, read_key=lambda: None),
    )
    assert code == 2
    assert "inspect: camera /dev/videoFAKE" in "\n".join(lines)
    assert "MODEL_API_KEY" in "\n".join(lines)
    assert SECRET not in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_inspect_plan_holds_both_branches_and_shared_ranges():
    built = pick_plan.build_inspect_plan(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        drop_joints_by_box={"good": DROP_GOOD, "bad": DROP_BAD},
        tip_fn=_fake_tip)
    assert built["ok"] is True
    assert list(built["ranges"]) == ["hover", "lower", "lift",
                                    "to-box", "release", "return"]
    assert set(built["ranges_by_box"]) == {"good", "bad"}
    for box in ("good", "bad"):
        assert list(built["ranges_by_box"][box]) == ["to-box", "release",
                                                    "return"]
    assert list(built["shared_ranges"]) == ["hover", "lower", "lift"]
    plan = built["plan"]
    good = built["ranges_by_box"]["good"]
    bad = built["ranges_by_box"]["bad"]
    assert plan[good["to-box"][1] - 1][1] == DROP_GOOD
    assert plan[bad["to-box"][1] - 1][1] == DROP_BAD
    assert plan[bad["release"][0]] == ("release", DROP_BAD, False)
    ok, at, _ = pick_plan.check_pick_plan(plan, REST, check_fn=_always_ok,
                                          tip_fn=_fake_tip)
    assert (ok, at) == (True, -1)


def test_bad_branch_refusal_refuses_whole_plan():
    def refusing(target, current, z_min_m=None):
        if tuple(target.joints_deg) == DROP_BAD:
            return {"ok": False, "reason": "nope",
                    "value": "bad branch", "limit": "refused"}
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    built = pick_plan.build_inspect_plan(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        drop_joints_by_box={"good": DROP_GOOD, "bad": DROP_BAD},
        tip_fn=_fake_tip)
    assert built["ok"] is True
    ok, at, res = pick_plan.check_pick_plan(built["plan"], REST,
                                            check_fn=refusing, tip_fn=_fake_tip)
    assert ok is False and res["reason"] == "nope"
    low, high = built["ranges_by_box"]["bad"]["to-box"]
    assert low <= at < high
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        _inspect_main_args(["--inspect"]),
        sdk_factory=lambda **kwargs: sdk,
        **_inspect_dry_kwargs(lines, check_fn=refusing),
    )
    assert code == 1
    assert "REFUSED" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def _inspect_loop(script, lines, **overrides):
    """Build the inspect plan and loop; the script may read holder["loop"]."""
    built = pick_plan.build_inspect_plan(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        drop_joints_by_box={"good": DROP_GOOD, "bad": DROP_BAD},
        tip_fn=_fake_tip)
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
                     "top_hold", "to-box"):
            return {"L1", "R1"}
        return set()

    return script


def _arm_sends(arm):
    return [tuple(call[1]) for call in arm.calls if call[0] == "send"]


def test_loop_does_not_hold_hover_for_inspection():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.phase == "lower")
    assert not loop.inspect_ready.is_set()
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    assert loop.phase == "inspect_hold"
    assert loop.pos == built["shared_ranges"]["lift"][1] - 1


def test_loop_verdict_good_sorts_into_good_box():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    assert loop.phase == "inspect_hold"
    assert loop.exit_code == 0
    loop.set_verdict("good")
    _step_until(loop, lambda: loop.done.is_set())
    text = "\n".join(lines)
    assert loop.exit_code == 0
    assert "dropped into the good box" in text
    sends = _arm_sends(arm)
    good_rad = tuple(math.radians(v) for v in DROP_GOOD)
    bad_rad = tuple(math.radians(v) for v in DROP_BAD)
    assert good_rad in sends
    assert bad_rad not in sends


def test_loop_verdict_bad_sorts_into_bad_box():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    loop.set_verdict("bad")
    _step_until(loop, lambda: loop.done.is_set())
    text = "\n".join(lines)
    assert loop.exit_code == 0
    assert "dropped into the bad box" in text
    sends = _arm_sends(arm)
    good_rad = tuple(math.radians(v) for v in DROP_GOOD)
    bad_rad = tuple(math.radians(v) for v in DROP_BAD)
    assert bad_rad in sends
    assert good_rad not in sends


def _assert_unverified_into_bad_box(loop, arm, lines):
    text = "\n".join(lines)
    assert loop.exit_code == 1
    assert "dropped into the bad box" in text
    sends = _arm_sends(arm)
    good_rad = tuple(math.radians(v) for v in DROP_GOOD)
    bad_rad = tuple(math.radians(v) for v in DROP_BAD)
    assert bad_rad in sends
    assert good_rad not in sends
    return text


def test_loop_verdict_none_sorts_unverified_into_bad_box():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    arm.calls.clear()
    loop.set_verdict(None)
    _step_until(loop, lambda: loop.done.is_set())
    text = _assert_unverified_into_bad_box(loop, arm, lines)
    assert "inspect failed, sorting into the bad box" in text
    assert "(unverified part)" in text


def test_loop_inspect_timeout_sorts_unverified_into_bad_box(monkeypatch):
    monkeypatch.setattr(pick_loop, "INSPECT_TIMEOUT_S", 0.05)
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    arm.calls.clear()
    _step_until(loop, lambda: loop.done.is_set())
    text = _assert_unverified_into_bad_box(loop, arm, lines)
    assert "inspect timeout, sorting into the bad box" in text
    assert "(unverified part)" in text


def _losing_script(holder):
    """Contacts for the grip, then nothing: the part is lost in the lift."""

    def script(n):
        loop = holder.get("loop")
        phase = loop.phase if loop is not None else "hover"
        if phase in ("close", "squeeze", "squeeze_hold"):
            return {"L1", "R1"}
        return set()

    return script


def test_loop_lost_while_lifting_skips_inspection():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_losing_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.done.is_set())
    text = "\n".join(lines)
    assert "part lost while lifting" in text
    assert not loop.inspect_ready.is_set()
    assert loop.inspect_skipped is True
    assert "dropped into" not in text
    assert loop.exit_code == 0
    rest_rad = tuple(math.radians(v) for v in REST)
    assert _arm_sends(arm)[-1] == rest_rad


def test_missed_grip_skips_inspection_quickly():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(lambda n: set(), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.done.is_set())
    assert "missed grip" in "\n".join(lines)
    assert not loop.inspect_ready.is_set()
    assert loop.inspect_skipped is True

    def _no_camera(node):
        raise AssertionError("no frame without a part held")

    out = []
    box = inspect_step.wait_and_inspect(
        loop, out.append, ready_timeout_s=5.0,
        find_node=lambda: (_ for _ in ()).throw(
            AssertionError("no camera without a part held")),
        open_camera=_no_camera)
    assert box is None
    assert "inspect skipped: no part held" in "\n".join(out)


def test_loop_hold_sends_last_commanded_pose():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    assert loop.pos is not None
    expected = tuple(math.radians(v) for v in built["plan"][loop.pos][1])
    arm.calls.clear()
    for _ in range(30):
        loop(None, 1.0 / LOOP_RATE_HZ)
    held = _arm_sends(arm)
    assert held
    assert all(item == expected for item in held)
    assert not loop.done.is_set()


def test_inspect_plan_real_solve_and_real_safety_pass():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed")
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    rest = [-2.79, 1.13, -2.68, 4.64, 2.57, -0.45]
    table = (513.3, 340.7)
    by_box = {}
    for box, xy in BOXES_TABLE_MM.items():
        solved = solve_drop(xy, rest)
        assert solved["ok"] is True, (box, solved.get("reason"))
        by_box[box] = solved["joints_deg"]
    built = pick_plan.build_inspect_plan(rest, table, 5.0, None,
                                        drop_joints_by_box=by_box)
    assert built["ok"] is True, built.get("reason")
    naive_ok, _, _ = pick_plan.check_pick_plan(built["plan"], rest)
    assert naive_ok is False  # rest chained into the bad branch refuses
    old_ref = safety.J6_REF
    safety.set_j6_ref(rest[5])
    try:
        ok, at, res = inspect_step.check_inspect_plan(built, rest)
    finally:
        safety.set_j6_ref(old_ref)
    assert (ok, at) == (True, -1), res


def test_loop_takes_bad_branch_directly_after_lift():
    holder, lines = {}, []
    built, arm, grip, loop = _inspect_loop(_grip_script(holder), lines)
    holder["loop"] = loop
    _step_until(loop, lambda: loop.inspect_ready.is_set())
    arm.calls.clear()
    loop.set_verdict("bad")
    _step_until(loop, lambda: loop.done.is_set())
    assert loop.exit_code == 0
    sends = _arm_sends(arm)
    bad_low, _ = built["ranges_by_box"]["bad"]["to-box"]
    bad_start = tuple(math.radians(v) for v in built["plan"][bad_low][1])
    lift_end = built["shared_ranges"]["lift"][1] - 1
    lift_pose = tuple(math.radians(v) for v in built["plan"][lift_end][1])
    first = next(i for i, item in enumerate(sends) if item == bad_start)
    assert sends[first - 1] == lift_pose  # no step through the good branch


def test_set_verdict_rejects_other_boxes():
    holder, lines = {}, []
    _, _, _, loop = _inspect_loop(_grip_script(holder), lines)
    with pytest.raises(ValueError):
        loop.set_verdict("middle")


def test_count_frame_three_levers_is_good(tmp_path, capsys):
    result = count_clips.count_frame(
        _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        _arm_frame(), "orange bar", recordings_dir=tmp_path,
        source_image="armcam_test.png")
    assert result["ok"] is True
    assert result["levers"] == 3
    assert result["verdict"] == "good"
    assert {"ok", "reason", "levers", "verdict", "full_image",
            "crop_image", "record"} <= set(result)
    assert SECRET not in str(result)
    assert capsys.readouterr().out == ""
    assert Path(result["full_image"]).is_file()
    assert Path(result["crop_image"]).is_file()
    assert Path(result["record"]).is_file()


def test_count_frame_two_levers_is_bad(tmp_path, capsys):
    result = count_clips.count_frame(
        _SeqClient([PART_AND_GRIPPER, TWO_BARS]),
        _arm_frame(), "orange bar", recordings_dir=tmp_path)
    assert result["ok"] is True
    assert result["levers"] == 2
    assert result["verdict"] == "bad"
    assert capsys.readouterr().out == ""


def test_count_frame_no_part_found(tmp_path):
    result = count_clips.count_frame(
        _SeqClient(["", THREE_BARS]),
        _arm_frame(), "orange bar", recordings_dir=tmp_path)
    assert result["ok"] is False
    assert result["reason"] == "part: not found"
    assert result["levers"] is None and result["verdict"] is None


def test_run_count_prints_paths_after_verdict(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path, source_image="armcam_test.png")
    assert code == 0
    out = capsys.readouterr().out
    assert SECRET not in out
    order = ["part: box=(323, 268, 362, 312) confidence=0.75",
             "levers: 3", "verdict: good"]
    positions = [out.index(line) for line in order]
    assert positions == sorted(positions)
    tail = out.split("verdict: good")[1].splitlines()
    tail = [line for line in tail if line.strip()]
    assert len(tail) == 3
    assert tail[0].endswith(".png") and not tail[0].endswith("_crop.png")
    assert tail[1].endswith("_crop.png")
    assert tail[2].endswith(".json")
    assert Path(tail[2]).is_file()


def _inspection_kwargs(tmp_path, texts, **overrides):
    kwargs = {
        "find_node": lambda: "/dev/videoFAKE",
        "open_camera": lambda node: _arm_frame(),
        "read_key": lambda: SECRET,
        "make_client": lambda key: _SeqClient(texts),
        "count_frame_fn": count_clips.count_frame,
        "recordings_dir": tmp_path,
    }
    kwargs.update(overrides)
    return kwargs


def test_run_inspection_returns_good_box(tmp_path, capsys):
    lines = []
    box = inspect_step.run_inspection(
        lines.append,
        **_inspection_kwargs(tmp_path, [PART_AND_GRIPPER, THREE_BARS]))
    assert box == "good"
    text = "\n".join(lines)
    assert "inspect: levers 3 verdict good" in text
    assert SECRET not in text
    assert len(list(Path(tmp_path).glob("armcam_*.png"))) == 1
    assert len(list(Path(tmp_path).glob("count_*.png"))) == 2
    assert len(list(Path(tmp_path).glob("count_*.json"))) == 1


def test_run_inspection_returns_bad_box(tmp_path, capsys):
    lines = []
    box = inspect_step.run_inspection(
        lines.append,
        **_inspection_kwargs(tmp_path, [PART_AND_GRIPPER, TWO_BARS]))
    assert box == "bad"
    assert "inspect: levers 2 verdict bad" in "\n".join(lines)


def test_run_inspection_no_frame_fails(capsys):
    lines = []
    box = inspect_step.run_inspection(
        lines.append,
        find_node=lambda: "/dev/videoFAKE",
        open_camera=lambda node: None,
        read_key=lambda: SECRET,
        make_client=lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]))
    assert box is None
    assert "inspect failed: no frame" in "\n".join(lines)


def test_run_inspection_api_error_fails():
    lines = []
    box = inspect_step.run_inspection(
        lines.append,
        find_node=lambda: "/dev/videoFAKE",
        open_camera=lambda node: _arm_frame(),
        read_key=lambda: SECRET,
        make_client=lambda key: _SeqClient(
            [PART_AND_GRIPPER, THREE_BARS], error=RuntimeError("boom")))
    assert box is None
    assert "inspect failed: " in "\n".join(lines)


def test_run_inspection_no_part_fails(tmp_path):
    lines = []
    box = inspect_step.run_inspection(
        lines.append, **_inspection_kwargs(tmp_path, ["", THREE_BARS]))
    assert box is None
    assert "inspect failed: part: target not in view" in "\n".join(lines)


def test_run_inspection_sends_brown_bar_to_target_part(tmp_path):
    phrases: list[str] = []
    texts = [PART_AND_GRIPPER, THREE_BARS]

    def make_client(key):
        client = SimpleNamespace()

        def create(**kwargs):
            phrases.append(kwargs["input"][0]["content"][0]["text"])
            return SimpleNamespace(output_text=texts[len(phrases) - 1])

        client.responses = SimpleNamespace(create=create)
        return client

    lines = []
    box = inspect_step.run_inspection(
        lines.append,
        **_inspection_kwargs(tmp_path, [], make_client=make_client))
    assert box == "good"
    assert phrases == ["small part", "brown bar"]
    assert "inspect: levers 3 verdict good" in "\n".join(lines)


def test_run_inspection_missing_node_fails():
    lines = []
    box = inspect_step.run_inspection(lines.append, find_node=lambda: None)
    assert box is None
    assert "inspect failed: arm camera not found" in "\n".join(lines)


def test_wait_and_inspect_times_out_without_ready():
    lines = []
    box = inspect_step.wait_and_inspect(
        SimpleNamespace(), lines.append, ready_timeout_s=0.01)
    assert box is None
    assert "inspect failed: hold never settled" in "\n".join(lines)


def test_wait_and_inspect_returns_on_skip_without_frame():
    skipped = SimpleNamespace(inspect_skipped=True)

    def _no_camera(node):
        raise AssertionError("no frame on skip")

    lines = []
    box = inspect_step.wait_and_inspect(
        skipped, lines.append, ready_timeout_s=5.0,
        open_camera=_no_camera)
    assert box is None
    assert "inspect skipped: no part held" in "\n".join(lines)


def test_wait_and_inspect_returns_on_done_without_frame():
    done = threading.Event()
    done.set()

    def _no_camera(node):
        raise AssertionError("no frame when done")

    lines = []
    box = inspect_step.wait_and_inspect(
        SimpleNamespace(done=done), lines.append, ready_timeout_s=5.0,
        open_camera=_no_camera)
    assert box is None
    assert "inspect skipped: no part held" in "\n".join(lines)


def test_loop_never_opens_camera_or_calls_api():
    tree = ast.parse(Path(pick_loop.__file__).read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module or "")
    names = " ".join(imported).lower()
    for word in ("camera", "vision", "model_check", "count_clips",
                 "inspect_step", "cv2", "openai"):
        assert word not in names, word
    source = Path(pick_loop.__file__).read_text(encoding="utf-8")
    assert "open_arm_camera" not in source
    assert "fetch_output_text" not in source
    assert "read_api_key" not in source


def _run_execute_inspect(tmp_path, texts, **overrides):
    """Run pick_test.execute with --inspect on a background-loop fake SDK."""
    holder, lines = {}, []
    sdk = ThreadedFakeSdk(holder=holder)
    kwargs = {
        "input_fn": lambda prompt: "yes",
        "tip_fn": _fake_tip,
        "solve_fn": _fake_solve,
        "joint_limits": TEST_LIMITS,
        "arm_vlims": VLIMS,
        "find_port": lambda: "/dev/ttyFAKE",
        "sensor_factory": lambda: FakeSensor(_grip_script(holder)),
        "solve_drop_fn": _fake_drop_solve,
        "inspect": True,
        "out": lines.append,
        "find_arm_node": lambda: "/dev/videoFAKE",
        "read_key": lambda: SECRET,
        "open_arm_camera": lambda node: _arm_frame(),
        "make_client": lambda key: _SeqClient(texts),
        "count_frame": count_clips.count_frame,
        "inspect_recordings_dir": tmp_path,
        "arm_mode": "posvel",
    }
    kwargs.update(overrides)
    old_timeout = pick_loop.INSPECT_TIMEOUT_S
    pick_loop.INSPECT_TIMEOUT_S = 600.0
    try:
        code = pick_test.execute(sdk, TABLE, 8.0, 28.0, False, 0.5, **kwargs)
    finally:
        pick_loop.INSPECT_TIMEOUT_S = old_timeout
    return sdk, code, "\n".join(lines)


def test_execute_inspect_sorts_good_end_to_end(tmp_path):
    sdk, code, text = _run_execute_inspect(
        tmp_path, [PART_AND_GRIPPER, THREE_BARS])
    assert code == 0
    assert "inspect: levers 3 verdict good" in text
    assert SECRET not in text
    assert "dropped into the good box" in text
    assert len(list(Path(tmp_path).glob("armcam_*.png"))) == 1
    assert len(list(Path(tmp_path).glob("count_*.json"))) == 1
    good_rad = tuple(math.radians(v) for v in DROP_GOOD)
    bad_rad = tuple(math.radians(v) for v in DROP_BAD)
    sends = _arm_sends(sdk.groups["arm"])
    assert good_rad in sends
    assert bad_rad not in sends
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls


def test_execute_inspect_sorts_bad_end_to_end(tmp_path):
    sdk, code, text = _run_execute_inspect(
        tmp_path, [PART_AND_GRIPPER, TWO_BARS])
    assert code == 0
    assert "inspect: levers 2 verdict bad" in text
    assert "dropped into the bad box" in text
    good_rad = tuple(math.radians(v) for v in DROP_GOOD)
    bad_rad = tuple(math.radians(v) for v in DROP_BAD)
    sends = _arm_sends(sdk.groups["arm"])
    assert bad_rad in sends
    assert good_rad not in sends


def test_execute_inspect_checks_camera_and_key_before_prompt(tmp_path):
    lines = []
    sdk = ThreadedFakeSdk()
    prompted = []

    def never_prompt(prompt):
        prompted.append(prompt)
        raise AssertionError("no prompt without camera and key")

    code = pick_test.execute(
        sdk, TABLE, 8.0, 28.0, False, 0.5, input_fn=never_prompt,
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE", arm_mode="posvel",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=_fake_drop_solve, inspect=True, out=lines.append,
        find_arm_node=lambda: None, read_key=lambda: SECRET)
    assert code == 2
    assert prompted == []
    assert "arm camera not found" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_execute_inspect_missing_key_exits_2_without_enabling(tmp_path):
    sdk = ThreadedFakeSdk()
    lines = []
    code = pick_test.execute(
        sdk, TABLE, 8.0, 28.0, False, 0.5,
        input_fn=lambda prompt: "yes",
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE", arm_mode="posvel",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=_fake_drop_solve, inspect=True, out=lines.append,
        find_arm_node=lambda: "/dev/videoFAKE", read_key=lambda: "  ")
    assert code == 2
    assert "MODEL_API_KEY" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert "disconnect" in sdk.calls


def test_execute_inspect_failed_sorts_unverified_into_bad_box(tmp_path):
    sdk, code, text = _run_execute_inspect(
        tmp_path, [PART_AND_GRIPPER, THREE_BARS],
        open_arm_camera=lambda node: None)
    assert code == 1
    assert "inspect failed: no frame" in text
    assert "inspect failed, sorting into the bad box" in text
    assert "(unverified part)" in text
    assert "dropped into the bad box" in text
    good_rad = tuple(math.radians(v) for v in DROP_GOOD)
    bad_rad = tuple(math.radians(v) for v in DROP_BAD)
    sends = _arm_sends(sdk.groups["arm"])
    assert bad_rad in sends
    assert good_rad not in sends
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls
