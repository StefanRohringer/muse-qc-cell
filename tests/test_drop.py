"""Tests for sorting into the good and bad box: drop_plan and --drop.

Offline only: fake SDK, fake TouchSensor, fake solve/drop solves, real
or stubbed safety. No bus, no serial port, no camera, no network. One
test uses the real SDK kinematics to solve both boxes.
"""

from __future__ import annotations

import math
import time

import pytest

from muse_qc import drop_plan, pick_loop, pick_plan, pick_test, safety
from muse_qc.drop_plan import (
    BOXES_TABLE_MM,
    DROP_TILT_DEG,
    RELEASE_OPEN_MM,
    TRANSIT_FLOOR_MM,
)
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.gripper_grip import CLOSED_DEG, width_to_deg
from muse_qc.target_pose import table_to_base_mm

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
RELEASE40 = width_to_deg(RELEASE_OPEN_MM)
GRIP_OPEN_STEP = 40.0 / LOOP_RATE_HZ
DROP_OK = (-3.0, -99.0, -80.0, 75.0, 1.0, -4.0)  # fake tip z 90 mm
DROP_LOW = (-3.0, -98.0, -80.0, 75.0, 1.0, -4.0)  # fake tip z 80 mm


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
    """Drop solve stand-in: fixed joints, tilt exactly on target."""
    return {
        "ok": True,
        "joints_deg": DROP_OK,
        "tip_base_mm": (401.1, 119.5, 170.0),
        "tip_table_mm": (376.4, 423.8, 170.0),
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


class FakeReading:
    def __init__(self, contacts):
        if isinstance(contacts, str):
            contacts = [] if contacts.strip() in ("", "-") else contacts.split()
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


def _built_drop(grip_z=8.0):
    built = pick_plan.build_pick_plan(REST, TABLE, grip_z, TEST_LIMITS,
                                      solve_fn=_fake_solve,
                                      drop_joints=DROP_OK, tip_fn=_fake_tip)
    assert built["ok"] is True
    return built


def _run_drop(sdk, script, open_mm=28.0, drop="good"):
    lines = []
    code = pick_test.execute(
        sdk, TABLE, 8.0, open_mm, False, 0.5, input_fn=lambda prompt: "yes",
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE", arm_mode="posvel",
        speed_deg_s=15.0, accel_deg_s2=30.0,
        sensor_factory=lambda: FakeSensor(script),
        solve_drop_fn=_fake_drop_solve, drop=drop, out=lines.append)
    return code, "\n".join(lines)


def _arm_sends(sdk):
    return [tuple(call[1]) for call in sdk.groups["arm"].calls if call[0] == "send"]


def _grip_sends_deg(sdk):
    return [math.degrees(call[1][0])
            for call in sdk.groups["gripper"].calls if call[0] == "send"]


def _cycle_plan(built, open_deg):
    """Cycle where each phase ends: hover/catchup/lower/close14/squeeze/holds."""
    ranges = built["ranges"]
    counts = {seg: ranges[seg][1] - ranges[seg][0] for seg in ranges}
    ramp = math.ceil(abs(open_deg - REST_GRIP) / GRIP_OPEN_STEP)
    catchup = max(0, ramp - counts["hover"])
    close_start = counts["hover"] + catchup + counts["lower"] + 1
    touch = close_start + 14
    squeeze_end = touch + 14
    squeeze_hold_end = squeeze_end + 50
    lift_end = squeeze_hold_end + counts["lift"]
    top_hold_end = lift_end + 50
    tobox_end = top_hold_end + counts["to-box"]
    return {
        "counts": counts, "close_start": close_start, "touch": touch,
        "lift_end": lift_end, "top_hold_end": top_hold_end,
        "tobox_end": tobox_end,
    }


def test_box_table_positions():
    assert BOXES_TABLE_MM == {"good": (376.4, 423.8), "bad": (675.4, 420.3)}
    assert drop_plan.BOX_HEIGHT_MM == pytest.approx(70.0)
    assert drop_plan.DROP_NOMINAL_Z_MM == pytest.approx(170.0)
    assert drop_plan.DROP_MIN_REAL_Z_MM == pytest.approx(110.0)
    assert DROP_TILT_DEG == pytest.approx(30.0)
    assert TRANSIT_FLOOR_MM == pytest.approx(90.0)


def test_drop_segment_order_and_shapes():
    built = _built_drop()
    assert list(built["ranges"]) == ["hover", "lower", "lift",
                                    "to-box", "release", "return"]
    assert built["drop_joints"] == DROP_OK
    plan, ranges = built["plan"], built["ranges"]
    assert plan[ranges["lift"][1] - 1][1] == built["hover_joints"]
    low, high = ranges["to-box"]
    assert plan[high - 1][1] == DROP_OK  # settle ends exactly at drop
    low_r, high_r = ranges["release"]
    assert high_r - low_r == 1
    assert plan[low_r] == ("release", DROP_OK, False)
    assert all(grasp is False for seg, _, grasp in plan[low:high_r])
    assert plan[-1][1] == built["rest_joints"]
    assert plan[-1][0] == "return"


def test_plan_without_drop_is_unchanged():
    built = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                      solve_fn=_fake_solve)
    assert built["ok"] is True
    assert list(built["ranges"]) == ["hover", "lower", "lift",
                                    "put-down", "raise", "return"]
    assert built["drop_joints"] is None
    lines = pick_plan.plan_summary(built["plan"], built)
    assert "put-down" in lines[0] and "to-box" not in lines[0]


def test_plan_summary_lists_drop_segments():
    built = _built_drop()
    text = "\n".join(pick_plan.plan_summary(built["plan"], built))
    assert "to-box 87" in text and "release 1" in text
    assert "put-down" not in text and "raise" not in text


def test_transit_floor_failure():
    built = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                      solve_fn=_fake_solve,
                                      drop_joints=DROP_LOW, tip_fn=_fake_tip)
    assert built["ok"] is False
    assert "transit too low" in built["reason"]
    direct = drop_plan.build_drop_segments(built.get("hover_joints", REST),
                                           DROP_LOW, REST, tip_fn=_fake_tip)
    assert direct["ok"] is False
    assert "transit too low" in direct["reason"]


def test_solve_drop_rejects_wrong_axis_tilt(monkeypatch):
    import numpy as np

    import reBotArm_control_py.kinematics as kin

    seed = [-3.33, -99.0, -79.0, 74.0, 0.91, -3.97]
    answered = np.radians(np.array(seed))

    class _Result:
        success = True
        q = answered

    monkeypatch.setattr(kin, "compute_ik", lambda *args, **kwargs: _Result())
    base = table_to_base_mm(*BOXES_TABLE_MM["good"])

    def fake_pose(joint_radians):
        return {"x_mm": base[0], "y_mm": base[1], "z_mm": 200.0,
                "link6_x_mm": base[0], "link6_y_mm": base[1],
                "link6_z_mm": 230.0}

    monkeypatch.setattr("muse_qc.fk_check.pose_mm", fake_pose)
    solved = drop_plan.solve_drop(BOXES_TABLE_MM["good"], seed)
    assert solved["ok"] is False
    assert "tilt" in solved["reason"]


def test_solve_drop_both_boxes_real_ik():
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed")
    seed = [-3.33, -99.0, -79.0, 74.0, 0.91, -3.97]
    for box, table_xy in BOXES_TABLE_MM.items():
        solved = drop_plan.solve_drop(table_xy, seed)
        assert solved["ok"] is True, (box, solved.get("reason"))
        assert abs(solved["tilt_deg"] - DROP_TILT_DEG) <= 2.0, box
        assert solved["error_mm"] <= 5.0, box
        assert solved["real_tip_z_mm"] >= drop_plan.DROP_MIN_REAL_Z_MM, box


def test_drop_arg_choices_exit_2_before_connect():
    def factory(**kwargs):
        raise AssertionError("nothing may connect on a bad argument")

    with pytest.raises(SystemExit) as info:
        pick_test.main(["--table-mm", "495.9", "322.7", "--drop", "middle"],
                       sdk_factory=factory)
    assert info.value.code == 2


def test_dry_run_drop_prints_pose_and_checks_all():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--drop", "good",
         "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=_fake_drop_solve, out=lines.append,
    )
    assert code == 0
    text = "\n".join(lines)
    assert "drop box: good at table mm (+376.4, +423.8), r=418.5 mm" in text
    assert "drop joints deg:" in text and "drop tilt: 30.0 deg" in text
    assert "fk tip base mm (+401.1, +119.5, +170.0), real tip z 130.0 mm" in text
    assert "to-box lowest tip z: 90.0 mm" in text
    assert "to-box 87" in text and "release 1" in text
    assert "dry run: moves nothing, motors never enabled" in text
    assert "connect" in sdk.calls and "disconnect" in sdk.calls
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    for group in sdk.groups.values():
        assert not any(isinstance(call, tuple) and call[0] in ("mode", "send")
                       for call in group.calls)


def test_dry_run_drop_real_safety_passes():
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--drop", "bad",
         "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
        tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=_fake_drop_solve, out=lines.append,
    )
    assert code == 0
    assert "drop box: bad" in "\n".join(lines)


def test_drop_waypoints_use_plain_safety_chain():
    seen = []

    def spy(target, current, z_min_m=None):
        seen.append(z_min_m)
        return {"ok": True, "speed": 0.1, "detail": "stub"}

    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--drop", "good",
         "--arm-mode", "posvel"],
        sdk_factory=lambda **kwargs: sdk,
        check_fn=spy, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=_fake_drop_solve, out=lines.append,
    )
    assert code == 0
    built = _built_drop(8.0)  # main() defaults to --grip-z-mm 8
    assert len(seen) == len(built["plan"])
    for seg in ("to-box", "release", "return"):
        low, high = built["ranges"][seg]
        assert seen[low:high] == [None] * (high - low), seg
    for seg in ("lower", "lift"):
        low, high = built["ranges"][seg]
        assert all(v == safety.GRASP_Z_MIN_M for v in seen[low:high]), seg


def test_drop_solve_failure_plans_nothing():
    def failing(box_xy, seed):
        return {"ok": False, "reason": "unreachable: IK did not converge"}

    sdk = FakeSdk()
    sanity, _ = _run_drop(sdk, lambda n: "-")
    assert sanity == 0  # sanity: the fake solve works
    sdk = FakeSdk()
    lines = []
    code = pick_test.execute(
        sdk, TABLE, 8.0, 28.0, False, 0.5, input_fn=lambda prompt: "yes",
        tip_fn=_fake_tip, solve_fn=_fake_solve, joint_limits=TEST_LIMITS,
        arm_vlims=VLIMS, find_port=lambda: "/dev/ttyFAKE", arm_mode="posvel",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=failing, drop="good", out=lines.append)
    assert code == 1
    assert "drop solve failed" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_release_reports_dropped_into_box():
    built = _built_drop()
    plan = _cycle_plan(built, OPEN28)
    cut = plan["tobox_end"] + 60  # contacts vanish mid release hold

    def script(n):
        if n < plan["touch"]:
            return set()
        if n < cut:
            return {"L1", "R1"}
        return set()

    sdk = FakeSdk()
    code, text = _run_drop(sdk, script)
    assert code == 0
    assert "part lifted, holding" in text
    assert "part lost while carrying" not in text
    assert "dropped into the good box" in text
    assert "part still held after release" not in text
    grips = _grip_sends_deg(sdk)
    assert any(v == pytest.approx(RELEASE40) for v in grips)
    assert grips[-1] == pytest.approx(RELEASE40)  # stays open over the box
    assert "disable_all" in sdk.calls


def test_release_reports_part_still_held():
    built = _built_drop()
    plan = _cycle_plan(built, OPEN28)

    def script(n):
        if n < plan["touch"]:
            return set()
        return {"L1", "R1"}

    sdk = FakeSdk()
    code, text = _run_drop(sdk, script)
    assert code == 0
    assert "dropped into the" not in text
    assert "part still held after release" in text
    assert "disable_all" in sdk.calls


def test_part_lost_during_tobox_returns_open():
    built = _built_drop()
    plan = _cycle_plan(built, OPEN28)
    lost_at = plan["top_hold_end"] + 10

    def script(n):
        if n < plan["touch"]:
            return set()
        if n < lost_at:
            return {"L1", "R1"}
        return {"L1"}

    sdk = FakeSdk()
    code, text = _run_drop(sdk, script)
    assert code == 0
    assert "part lost while carrying" in text
    assert "dropped into the" not in text
    assert "part still held after release" not in text
    assert _grip_sends_deg(sdk)[-1] == pytest.approx(OPEN28)
    rest_rad = tuple(math.radians(v) for v in REST)
    assert _arm_sends(sdk)[-1] == rest_rad
    assert "disable_all" in sdk.calls


def test_drop_loop_still_uses_pick_loop_phases():
    assert pick_loop.DROP_WALKS == ("to-box",)
    assert "to-box" in pick_loop._WALKS


def test_solve_drop_rejects_low_real_tip(monkeypatch):
    try:
        import reBotArm_control_py.kinematics  # noqa: F401
    except ImportError:
        pytest.skip("Seeed SDK kinematics not installed")
    import numpy as np

    import reBotArm_control_py.kinematics as kin

    seed = [-3.33, -99.0, -79.0, 74.0, 0.91, -3.97]
    answered = np.radians(np.array(seed))

    class _Result:
        success = True
        q = answered

    monkeypatch.setattr(kin, "compute_ik", lambda *args, **kwargs: _Result())
    base = table_to_base_mm(*BOXES_TABLE_MM["good"])
    yaw = math.atan2(base[1], base[0])
    tilt = math.radians(DROP_TILT_DEG)
    axis = (math.sin(tilt) * math.cos(yaw), math.sin(tilt) * math.sin(yaw),
            -math.cos(tilt))
    end = (base[0] - axis[0] * 30.0, base[1] - axis[1] * 30.0,
           170.0 - axis[2] * 30.0)
    link6 = (end[0] - axis[0] * 155.39, end[1] - axis[1] * 155.39,
             end[2] - axis[2] * 155.39)

    def fake_pose(joint_radians):
        return {"x_mm": end[0], "y_mm": end[1], "z_mm": end[2],
                "link6_x_mm": link6[0], "link6_y_mm": link6[1],
                "link6_z_mm": link6[2]}

    monkeypatch.setattr("muse_qc.fk_check.pose_mm", fake_pose)
    monkeypatch.setattr("muse_qc.drop_plan.real_tip_m",
                        lambda joints_deg: (0.40, 0.12, 0.05))
    solved = drop_plan.solve_drop(BOXES_TABLE_MM["good"], seed)
    assert solved == {"ok": False,
                      "reason": "unreachable: drop tip too low (real z 50.0 mm)"}


def _abort_loop(mid_phase, mid_pos):
    """PickLoop parked mid-drop with a held part, for abort tests."""
    from muse_qc.pick_loop import PickLoop

    built = _built_drop()
    plan, ranges = built["plan"], built["ranges"]
    tips = [_fake_tip(list(wp)) for _, wp, _ in plan]
    arm = FakeArmGroup(REST)
    grip = FakeGripperGroup(REST_GRIP)
    lines = []
    loop = PickLoop(arm, grip, list(VLIMS), [0.3],
                    [math.radians(v) for v in REST],
                    plan, tips, ranges, tips[ranges["hover"][1] - 1][2],
                    OPEN28, REST_GRIP, 50, False,
                    FakeSensor(lambda n: {"L1", "R1"}),
                    lines.append, _always_ok, _fake_tip,
                    drop_box="good", release_open_deg=RELEASE40)
    squeeze = OPEN28 + 4.1
    loop.pos = mid_pos
    loop.phase = mid_phase
    loop.k, loop.end = ranges["to-box"] if mid_phase == "to-box" \
        else ranges["release"]
    loop.cmd_arm_rad = [math.radians(v) for v in plan[mid_pos][1]]
    loop.grip_cmd = loop.measured = squeeze
    loop.contacts = loop._prev = frozenset({"L1", "R1"})
    arm.calls.clear()
    grip.calls.clear()
    return loop, arm, grip, built, squeeze


def _step_until_done(loop, limit=30000):
    for _ in range(limit):
        loop(None, 1.0 / LOOP_RATE_HZ)
        if loop.done.is_set():
            break
    assert loop.done.is_set()


def test_abort_mid_tobox_detour_is_contiguous():
    built = _built_drop()
    plan, ranges = built["plan"], built["ranges"]
    mid = ranges["to-box"][0] + 5
    loop, arm, grip, _, squeeze = _abort_loop("to-box", mid)
    loop.abort()
    _step_until_done(loop)
    assert loop.exit_code == 1
    low, high = ranges["return"]
    assert loop.detour == (list(range(mid, ranges["to-box"][1]))
                           + list(range(low, high)))
    sends = [tuple(call[1]) for call in arm.calls if call[0] == "send"]
    want = [tuple(math.radians(v) for v in plan[i][1]) for i in loop.detour]
    assert len(sends) == len(want)
    for got, expected in zip(sends, want):
        assert got == pytest.approx(expected)
    grips = [math.degrees(call[1][0]) for call in grip.calls
             if call[0] == "send"]
    assert grips and all(v == pytest.approx(squeeze) for v in grips)


def test_abort_in_release_goes_straight_to_return():
    built = _built_drop()
    plan, ranges = built["plan"], built["ranges"]
    at = ranges["to-box"][1] - 1
    loop, arm, grip, _, squeeze = _abort_loop("release", at)
    loop.grip_cmd = loop.measured = RELEASE40
    arm.calls.clear()
    grip.calls.clear()
    loop.abort()
    _step_until_done(loop)
    assert loop.exit_code == 1
    low, high = ranges["return"]
    assert loop.detour == [at] + list(range(low, high))
    sends = [tuple(call[1]) for call in arm.calls if call[0] == "send"]
    want = [tuple(math.radians(v) for v in plan[i][1]) for i in loop.detour]
    assert len(sends) == len(want)
    for got, expected in zip(sends, want):
        assert got == pytest.approx(expected)
    grips = [math.degrees(call[1][0]) for call in grip.calls
             if call[0] == "send"]
    assert grips and all(v == pytest.approx(RELEASE40) for v in grips)


def _zero_gravity(joint_rad):
    return [0.0] * 6


def _dry_run_drop(extra_args):
    sdk = FakeSdk()
    lines = []
    code = pick_test.main(
        ["--table-mm", "495.9", "322.7", "--drop", "good"] + extra_args,
        sdk_factory=lambda **kwargs: sdk,
        check_fn=_always_ok, tip_fn=_fake_tip, solve_fn=_fake_solve,
        joint_limits=TEST_LIMITS, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: FakeSensor(lambda n: "-"),
        solve_drop_fn=_fake_drop_solve, gravity_fn=_zero_gravity,
        out=lines.append,
    )
    assert code == 0
    return "\n".join(lines)


def _segment_line_counts(text):
    line = next(item for item in text.splitlines()
                if item.startswith("segments at 100 Hz:"))
    counts = {}
    bits = line.split(":")[1].split("(")[0].split()
    for name, value in zip(bits[::2], bits[1::2]):
        counts[name] = int(value)
    return counts


def test_fast_free_moves_shorten_drop_segments():
    slow = _dry_run_drop(["--speed-deg-s", "15", "--accel-deg-s2", "30"])
    mid = _dry_run_drop(["--speed-deg-s", "30", "--accel-deg-s2", "60"])
    fast = _dry_run_drop(["--speed-deg-s", "50", "--accel-deg-s2", "100"])
    assert "speeds: free 15 deg/s, accel 30 deg/s2" in slow
    assert "speeds: free 30 deg/s, accel 60 deg/s2" in mid
    assert "speeds: free 50 deg/s, accel 100 deg/s2" in fast
    for text in (slow, mid, fast):
        assert "arm mode: mit-ff" in text
        assert "all waypoints ok" in text
    slow_counts = _segment_line_counts(slow)
    mid_counts = _segment_line_counts(mid)
    fast_counts = _segment_line_counts(fast)
    for seg in ("hover", "to-box", "return"):
        assert mid_counts[seg] < slow_counts[seg], seg
        assert fast_counts[seg] < mid_counts[seg], seg


def test_accel_reaches_the_drop_segments():
    soft = _dry_run_drop(["--speed-deg-s", "50", "--accel-deg-s2", "30"])
    hard = _dry_run_drop(["--speed-deg-s", "50", "--accel-deg-s2", "100"])
    assert (_segment_line_counts(hard)["to-box"]
            < _segment_line_counts(soft)["to-box"])


def test_fast_plan_passes_real_safety():
    built = pick_plan.build_pick_plan(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        free_speed_deg_s=50.0, accel_deg_s2=100.0,
        drop_joints=DROP_OK, tip_fn=_fake_tip)
    assert built["ok"] is True
    peak = 0.0
    previous = tuple(REST)
    for _, waypoint, _ in built["plan"]:
        peak = max(peak, max(abs(a - b)
                             for a, b in zip(waypoint, previous)))
        previous = tuple(waypoint)
    assert peak <= 10.0  # MAX_STEP_JOINT_DEG per command at 100 Hz
    safety.set_j6_ref(REST[5])  # as _plan_and_check does before checking
    ok, at, _ = pick_plan.check_pick_plan(
        built["plan"], REST, tip_fn=_fake_tip)
    assert (ok, at) == (True, -1)


def test_posvel_refuses_speed_above_15_before_connect():
    def factory(**kwargs):
        raise AssertionError("must not connect on a refused speed")

    with pytest.raises(SystemExit) as info:
        pick_test.main(
            ["--table-mm", "495.9", "322.7", "--drop", "good",
             "--arm-mode", "posvel", "--speed-deg-s", "20"],
            sdk_factory=factory,
            out=lambda line: None,
        )
    assert info.value.code == 2


def test_posvel_accepts_speed_at_15():
    text = _dry_run_drop(["--arm-mode", "posvel", "--speed-deg-s", "15"])
    assert "arm mode: posvel" in text
    assert "all waypoints ok" in text


def test_drop_defaults_mitff_50_100():
    text = _dry_run_drop([])
    assert "arm mode: mit-ff" in text
    assert "speeds: free 50 deg/s, accel 100 deg/s2" in text


def test_drop_80_160_accepted():
    text = _dry_run_drop(["--speed-deg-s", "80", "--accel-deg-s2", "160"])
    assert "speeds: free 80 deg/s, accel 160 deg/s2" in text
    assert "all waypoints ok" in text


@pytest.mark.parametrize("flag,value", [("--speed-deg-s", "81"),
                                        ("--accel-deg-s2", "161")])
def test_drop_81_161_refused_before_connect(flag, value):
    def factory(**kwargs):
        raise AssertionError("must not connect on a refused range")

    with pytest.raises(SystemExit) as info:
        pick_test.main(
            ["--table-mm", "495.9", "322.7", "--drop", "good", flag, value],
            sdk_factory=factory,
            out=lambda line: None,
        )
    assert info.value.code == 2
