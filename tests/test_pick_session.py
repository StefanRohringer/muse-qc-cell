"""Tests for the arm session (pick_session). Offline, fakes only.

Deterministic by construction, no wall-clock dependence:
the control loop never free-runs. The injected SDK stores the
loop callback and the test pumps it cycle by cycle
(StoredFakeSdk.pump); virtual time (FakeClock) advances only
with pumped cycles, so every main-thread deadline binds at a
deterministic pump count. The test pumps until a loop signal
appears, lets the main thread work with the loop frozen
(budgets cannot burn), then resumes. Stdin lines are pre-fed
in order (StringIO) or fed at exact pump points (PumpStdin).
The only real-time waits are never-binding stuck-guards and
the product's own inspection poll; no sleeps shape the fake.

Fake SDK with phase-based touch contacts, fake arm camera
plus scripted answers, fake tip/solve with the real dict
shapes. No bus, no camera, no network.
"""

from __future__ import annotations

import io
import math
import queue
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy
import pytest

from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ, SETTLE_HOLD_CYCLES
from muse_qc.inspection import count_clips
from muse_qc.pick import box_transit, pick_session, session_driver
from muse_qc.pick.session_protocol import FakeClock, parse_session_line

TABLE = (495.9, 322.7)
TABLE2 = (500.0, 330.0)
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
SECRET = "[REDACTED]"
PART_AND_GRIPPER = (
    "8<|box;x1=423;y1=402;x2=456;y2=429;w=33;h=27;c=0.76|>,"
    "7<|box;x1=323;y1=268;x2=362;y2=312;w=39;h=44;c=0.75|>"
)
THREE_BARS = (
    "1<|box;x1=158;y1=206;x2=210;y2=457;w=52;h=251;c=0.70|>,"
    "2<|box;x1=237;y1=206;x2=289;y2=457;w=52;h=251;c=0.70|>,"
    "0<|box;x1=313;y1=213;x2=363;y2=464;w=50;h=251;c=0.68|>"
)
DROP_GOOD = (-3.5, -99.03, -80.0, 75.0, 1.0, -4.0)
DROP_BAD = (-5.0, -99.47, -81.0, 75.0, 1.5, -4.5)
HELD_PHASES = frozenset({
    "close", "squeeze", "squeeze_hold", "lift", "top_hold",
    "to-station", "inspect_hold", "to-box",
})


@pytest.fixture(autouse=True)
def _no_hardware_settles(monkeypatch):
    """Skip the touch-sensor settle delay: the fake needs none.

    Patches only the module attribute the product reads;
    protocol and messages are unchanged.
    """
    monkeypatch.setattr(
        "muse_qc.gripper.gripper_stream.ZERO_SETTLE_S", 0.0)


def _fake_solve(table_xy, height_mm, seed, yaw_offset_deg=0.0):
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
    from muse_qc.pick.drop_plan import BOXES_TABLE_MM

    if tuple(float(v) for v in box_xy) == BOXES_TABLE_MM["bad"]:
        joints, table = DROP_BAD, (675.4, 420.3)
    else:
        joints, table = DROP_GOOD, (376.4, 423.8)
    return {
        "ok": True,
        "joints_deg": joints,
        "tip_base_mm": (401.1, 119.5, 170.0),
        "tip_table_mm": (table[0], table[1], 170.0),
        "tilt_deg": 30.0,
        "error_mm": 0.0,
        "real_tip_z_mm": 130.0,
    }


def _fake_tip(joints_deg):
    height = -(float(joints_deg[1]) + 90.0) * 10.0
    return (0.30, 0.0, height / 1000.0)


class FakeArmGroup:
    def __init__(self, joints_deg):
        self._rad = [math.radians(v) for v in joints_deg]
        self.calls = []

    @property
    def joint_names(self):
        return ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    def get_positions(self):
        self.calls.append("get")
        # Hardware never reads back exactly 0.0 (encoder noise);
        # without this the station joint 6 (0.0 deg) would trip
        # the live invalid-reading check no real arm ever trips.
        return [v if v != 0.0 else 1e-9 for v in self._rad]

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


class StoredFakeSdk:
    """Fake SDK storing the loop: the test pumps it, nothing free-runs."""

    def __init__(self, joints_deg=REST, grip_deg=REST_GRIP):
        self.groups = {
            "arm": FakeArmGroup(joints_deg),
            "gripper": FakeGripperGroup(grip_deg),
        }
        self.calls = []
        self.loop_fn = None

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
        self.loop_fn = fn

    def stop_control_loop(self):
        self.calls.append("stop")

    def pump(self, clock, count):
        """Run count loop cycles; virtual time tracks pumped cycles."""
        for _ in range(count):
            self.loop_fn(self, 0.01)
            clock.advance(0.01)


class FakeReading:
    def __init__(self, contacts):
        self.contacts = frozenset(contacts)


def _dline(number, contacts):
    return ("D,%d," % number + ",".join(["1.0", "2.0", "3.0", "10.0"] * 4)
            + ",%s\r\n" % contacts)


class PhaseSensor:
    """Contacts follow the loop phase; empty before the loop exists."""

    def __init__(self, sdk, held_phases=HELD_PHASES):
        self._sdk = sdk
        self._held = held_phases
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
        loop = self._sdk.loop_fn
        phase = loop.phase if loop is not None else "hover"
        if phase in self._held:
            return FakeReading({"L1", "R1"})
        return FakeReading(set())

    def close(self):
        self.closed = True


class PumpStdin:
    """Stdin fed at exact pump points; None feeds EOF for reader cleanup."""

    def __init__(self):
        self._items: queue.Queue = queue.Queue()

    def feed(self, line):
        """Queue one full line (with newline) or None for EOF."""
        self._items.put(line)

    def readline(self):
        item = self._items.get()
        return "" if item is None else item


class _SeqClient:
    def __init__(self, texts):
        self._texts = list(texts)
        self.calls = 0
        self.responses = self

    def create(self, **kwargs):
        self.calls += 1
        index = min(self.calls - 1, len(self._texts) - 1)
        return SimpleNamespace(output_text=self._texts[index])


class _TickDatetime:
    """datetime stand-in stamping one more second per call (unique records)."""

    def __init__(self):
        import datetime as _dt

        self._dt = _dt
        self._calls = 0

    def now(self, tz=None):
        self._calls += 1
        moment = self._dt.datetime(2026, 11, 1, 12, 0, 0)
        if tz is not None:
            moment = moment.replace(tzinfo=tz)
        return moment + self._dt.timedelta(seconds=self._calls)


def _arm_frame():
    frame = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
    frame[187:233, 317:352] = (0, 140, 255)
    return frame


def _session_kwargs(sdk, sensor_holder, clock, lines, tmp_path, **overrides):
    sensor = PhaseSensor(sdk)
    sensor_holder.append(sensor)
    kwargs = {
        "input_fn": lambda prompt: "yes",
        "tip_fn": _fake_tip,
        "solve_fn": _fake_solve,
        "solve_drop_fn": _fake_drop_solve,
        "joint_limits": TEST_LIMITS,
        "arm_vlims": VLIMS,
        "find_port": lambda: "/dev/ttyFAKE",
        "sensor_factory": lambda: sensor,
        "use_station": False,
        "out": lines.append,
        "find_arm_node": lambda: "/dev/videoFAKE",
        "read_key": lambda: SECRET,
        "open_arm_camera": lambda node: _arm_frame(),
        "make_client": lambda key: _SeqClient(
            [PART_AND_GRIPPER, THREE_BARS] * 4),
        "count_frame": count_clips.count_frame,
        "inspect_recordings_dir": tmp_path,
        "arm_mode": "posvel",
        "speed_deg_s": 15.0,
        "accel_deg_s2": 30.0,
        "hold_wait_s": 3.0,
        "clock": clock,
    }
    kwargs.update(overrides)
    return kwargs


def _start_main(target, holder):
    """Run the session main in a daemon thread; the test pumps the loop."""
    thread = threading.Thread(
        target=lambda: holder.update(code=target()), daemon=True)
    thread.start()
    return thread


def _wait_main(cond, what, timeout_s=60.0):
    """Wait for main-thread progress; the guard never fires on correct code."""
    deadline = time.monotonic() + timeout_s
    while not cond():
        if time.monotonic() >= deadline:
            raise AssertionError(f"main thread never {what}")
        time.sleep(0.005)


def _pump_until(sdk, clock, cond, what, max_cycles=200000):
    """Pump one cycle at a time until cond holds; precise, no overshoot."""
    for _ in range(max_cycles):
        if cond():
            return
        sdk.pump(clock, 1)
    raise AssertionError(f"loop never {what}")


def _fly_part(sdk, clock, loop):
    """Pump one part flight: hold for inspect, verdict, release or end."""
    _pump_until(sdk, clock,
                lambda: loop.inspect_ready.is_set() or loop.done.is_set(),
                "held for inspect")
    if loop.done.is_set():
        return "end"
    _wait_main(lambda: loop._verdict != "pending" or loop.done.is_set(),
               "inspected the part")
    _pump_until(sdk, clock,
                lambda: loop.part_done.is_set() or loop.done.is_set(),
                "released the part")
    return "part" if loop.part_done.is_set() else "end"


def _begin_session(targets, tmp_path, stdin=None, **overrides):
    """Start a session under test control; return (sdk, sensors, clock,
    thread, holder, lines). The caller pumps, then joins the thread."""
    clock: FakeClock = FakeClock()
    lines: list = []
    sensors: list = []
    sdk = StoredFakeSdk()
    if stdin is None:
        stdin = io.StringIO("".join(line + "\n" for line in targets))
    holder: dict = {}
    thread = _start_main(
        lambda: pick_session.execute_session(
            sdk, stdin,
            **_session_kwargs(sdk, sensors, clock, lines, tmp_path,
                             **overrides)),
        holder)
    _wait_main(lambda: sdk.loop_fn is not None, "created the loop")
    return sdk, sensors, clock, thread, holder, lines


def _finish_session(thread, holder, timeout_s=60.0):
    """Join the session main; fail (never hang) when it never finishes."""
    thread.join(timeout=timeout_s)
    assert not thread.is_alive(), "session main never finished"
    return holder.get("code")


def test_two_targets_plus_end_sorts_both_with_one_enable_disable(tmp_path):
    sdk, sensors, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90",
         f"part {TABLE2[0]} {TABLE2[1]}",
         "end"],
        tmp_path)  # fake drop solves with the real dict shapes
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop._pending is not None
               or loop.end_reason is not None or loop.done.is_set(),
               "planned part 2")
    assert loop.end_reason is None
    sdk.pump(clock, 1)  # install
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 0
    text = "\n".join(lines)
    assert text.count("session: part done") == 2
    assert text.count("dropped into the good box") == 2
    assert "session: end (end)" in text
    assert sdk.calls.count("enable_all") == 1
    assert sdk.calls.count("disable_all") == 1
    assert sdk.calls.count("disconnect") == 1
    assert sdk.calls.count("loop") == 1
    assert text.count("transit: ") == 1
    assert "return: retargeted at the session rest pose" in text
    assert "session plan waypoints ok" in text
    assert len(_arm_rest_sends(sdk)) == SETTLE_HOLD_CYCLES + 2
    assert ("end readings: arm [-3.33 -99.00 -79.00"
            " +74.00 +0.91 -3.97]") in text
    assert sensors and sensors[0].closed


def test_two_targets_with_station_sorts_both(tmp_path):
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90",
         f"part {TABLE2[0]} {TABLE2[1]} 30",
         "end"],
        tmp_path, use_station=True)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop._pending is not None
               or loop.end_reason is not None or loop.done.is_set(),
               "planned part 2")
    assert loop.end_reason is None
    sdk.pump(clock, 1)  # install
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 0
    text = "\n".join(lines)
    assert text.count("session: part done") == 2
    assert text.count("dropped into the good box") == 2
    assert "session: end (end)" in text
    assert sdk.calls.count("enable_all") == 1
    assert sdk.calls.count("disable_all") == 1


def test_eof_after_first_part_returns_to_rest(tmp_path):
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90"], tmp_path)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    assert loop.end_reason == "eof"
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 1
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "session: end (eof)" in text
    assert sdk.calls.count("enable_all") == 1
    assert sdk.calls.count("disable_all") == 1
    # One init send plus the return ramp endpoint plus the rest settle.
    assert len(_arm_rest_sends(sdk)) == SETTLE_HOLD_CYCLES + 2


def test_bad_line_ends_session_with_return(tmp_path):
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90", "bogus"], tmp_path)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    assert loop.end_reason == "bad line 'bogus'"
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 1
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "session: end (bad line 'bogus')" in text
    assert sdk.calls.count("disable_all") == 1
    # One init send plus the return ramp endpoint plus the rest settle.
    assert len(_arm_rest_sends(sdk)) == SETTLE_HOLD_CYCLES + 2


def test_hold_timeout_returns_to_rest(tmp_path):
    stdin = PumpStdin()
    stdin.feed(f"part {TABLE[0]} {TABLE[1]} 90\n")
    sdk, _, clock, thread, holder, lines = _begin_session(
        [], tmp_path, stdin=stdin, hold_wait_s=0.3)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    # No more lines: pump exactly through the 0.3 s hold budget.
    _pump_until(sdk, clock, loop.done.is_set, "timed out the hold")
    code = _finish_session(thread, holder)
    stdin.feed(None)  # reader cleanup only after main is done
    assert code == 1
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "session: end (hold timeout)" in text
    assert sdk.calls.count("disable_all") == 1


def test_transit_floor_refusal_ends_session(tmp_path, monkeypatch):
    monkeypatch.setattr(box_transit, "TRANSIT_FLOOR_MM", 10000.0)
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90",
         f"part {TABLE2[0]} {TABLE2[1]}",
         "end"],
        tmp_path)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "refused the transit")
    assert (loop.end_reason or "").startswith("plan refused")
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 1
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "plan refused: transit too low" in text
    assert "session: end (plan refused: transit too low" in text
    assert sdk.calls.count("disable_all") == 1


def test_missed_grip_never_transits(tmp_path):
    lines: list = []
    sensors: list = []
    clock: FakeClock = FakeClock()
    sdk = StoredFakeSdk()
    stdin = io.StringIO(f"part {TABLE[0]} {TABLE[1]} 90\nend\n")
    holder: dict = {}
    thread = _start_main(
        lambda: pick_session.execute_session(
            sdk, stdin,
            **_session_kwargs(
                sdk, sensors, clock, lines, tmp_path,
                sensor_factory=lambda: PhaseSensor(
                    sdk, held_phases=frozenset()))),
        holder)
    _wait_main(lambda: sdk.loop_fn is not None, "created the loop")
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "end"
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 1
    text = "\n".join(lines)
    assert "missed grip" in text
    assert "session: part done" not in text
    assert "session: end (missed grip)" in text
    assert sdk.calls.count("disable_all") == 1
    # One init send plus the return ramp endpoint plus the rest settle.
    assert len(_arm_rest_sends(sdk)) == SETTLE_HOLD_CYCLES + 2


def test_second_part_plan_exception_ends_session(tmp_path):
    seen_tables: list = []

    def _fail_second_table(table_xy, height_mm, seed, yaw_offset_deg=0.0):
        key = (round(float(table_xy[0]), 1), round(float(table_xy[1]), 1))
        if key not in seen_tables:
            seen_tables.append(key)
        if len(seen_tables) > 1:
            raise RuntimeError("boom")
        return _fake_solve(table_xy, height_mm, seed)

    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90",
         f"part {TABLE2[0]} {TABLE2[1]}",
         "end"],
        tmp_path, solve_fn=_fail_second_table)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "failed the plan")
    assert loop.end_reason == "plan failed: boom"
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 1
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "session: end (plan failed: boom)" in text
    assert sdk.calls.count("disable_all") == 1


def test_dry_run_checks_both_parts_and_sends_nothing(tmp_path):
    lines: list = []
    sdk = StoredFakeSdk()
    stdin = io.StringIO(f"part {TABLE[0]} {TABLE[1]} 90\n"
                        f"part {TABLE2[0]} {TABLE2[1]}\nend\n")
    code = pick_session.dry_run_session(
        sdk, stdin, out=lines.append, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: PhaseSensor(sdk),
        joint_limits=TEST_LIMITS, tip_fn=_fake_tip, solve_fn=_fake_solve,
        solve_drop_fn=_fake_drop_solve, find_arm_node=lambda: "/dev/videoFAKE",
        read_key=lambda: SECRET, arm_mode="posvel", use_station=False,
        speed_deg_s=15.0, accel_deg_s2=30.0)
    text = "\n".join(lines)
    assert code == 0
    assert text.count("segments at 100 Hz:") == 2
    assert text.count("transit: ") == 1
    assert "session: dry run ok" in text
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    for group in sdk.groups.values():
        assert not any(isinstance(call, tuple) and call[0] in ("mode", "send")
                       for call in group.calls)


def test_dry_run_bad_line_reports_and_moves_nothing(tmp_path):
    lines: list = []
    sdk = StoredFakeSdk()
    stdin = io.StringIO("part 1 2 3\nbogus\n")
    code = pick_session.dry_run_session(
        sdk, stdin, out=lines.append, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: PhaseSensor(sdk),
        joint_limits=TEST_LIMITS, tip_fn=_fake_tip, solve_fn=_fake_solve,
        solve_drop_fn=_fake_drop_solve, find_arm_node=lambda: "/dev/videoFAKE",
        read_key=lambda: SECRET, arm_mode="posvel", use_station=False,
        speed_deg_s=15.0, accel_deg_s2=30.0)
    assert code == 1
    assert "bad session line 'bogus'" in "\n".join(lines)
    assert "enable_all" not in sdk.calls


def _tty_yes(*args, **kwargs):
    """Fake /dev/tty opener answering yes (never touches the terminal)."""
    return io.StringIO("yes\n")


def _tty_missing(*args, **kwargs):
    raise OSError("no tty")


def test_confirm_stdin_sorts_with_leading_yes(tmp_path):
    sdk, _, clock, thread, holder, lines = _begin_session(
        ["yes", f"part {TABLE[0]} {TABLE[1]} 90", "end"],
        tmp_path, input_fn=None, confirm_stdin=True)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 0
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "bad line" not in text
    assert "session: end (end)" in text


def test_confirm_stdin_wrong_first_line_enables_nothing(tmp_path):
    lines: list = []
    sdk = StoredFakeSdk()
    stdin = io.StringIO("no\n")
    code = pick_session.execute_session(
        sdk, stdin, out=lines.append, input_fn=None, confirm_stdin=True)
    assert code == 2
    assert "confirmation missing" in "\n".join(lines)
    assert sdk.calls == []


def test_tty_confirm_leaves_leading_stdin_yes_as_target(tmp_path):
    lines: list = []
    sdk = StoredFakeSdk()
    stdin = io.StringIO(f"yes\npart {TABLE[0]} {TABLE[1]} 90\nend\n")
    code = pick_session.execute_session(
        sdk, stdin, out=lines.append, input_fn=None,
        tty_opener=_tty_yes, find_arm_node=lambda: "/dev/videoFAKE",
        read_key=lambda: SECRET, joint_limits=TEST_LIMITS)
    assert code == 1
    assert "bad line 'yes'" in "\n".join(lines)
    assert "enable_all" not in sdk.calls


def test_tty_confirm_sorts_without_stdin_yes(tmp_path):
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90", "end"],
        tmp_path, input_fn=None, tty_opener=_tty_yes)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 0
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "session: end (end)" in text


def test_tty_confirm_without_tty_enables_nothing(tmp_path):
    lines: list = []
    sdk = StoredFakeSdk()
    stdin = io.StringIO(f"part {TABLE[0]} {TABLE[1]} 90\n")
    code = pick_session.execute_session(
        sdk, stdin, out=lines.append, input_fn=None,
        tty_opener=_tty_missing)
    assert code == 2
    assert "confirmation missing" in "\n".join(lines)
    assert sdk.calls == []


def test_hold_matches_box_boundary():
    import math as _math

    rad = [_math.radians(v) for v in DROP_GOOD]
    assert session_driver.hold_matches_box(rad, list(DROP_GOOD)) is True
    # Planner IK drift (about a degree on the elbow joints) must
    # pass; only a gross deviation ends the session.
    shifted = [_math.radians(v + 1.99) for v in DROP_GOOD]
    assert session_driver.hold_matches_box(shifted, list(DROP_GOOD)) is True
    over = ([_math.radians(DROP_GOOD[0] + 2.01)]
            + [_math.radians(v) for v in DROP_GOOD[1:]])
    assert session_driver.hold_matches_box(over, list(DROP_GOOD)) is False
    assert session_driver.hold_matches_box([0.0] * 5, list(DROP_GOOD)) is False


def test_held_pose_mismatch_returns_to_rest(tmp_path, monkeypatch):
    real_exits = session_driver.box_exits

    def _shifted(built):
        return {box: [joints[0] + 5.0] + list(joints[1:])
                for box, joints in real_exits(built).items()}

    monkeypatch.setattr(session_driver, "box_exits", _shifted)
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90", "end"], tmp_path)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "detected the mismatch")
    assert loop.end_reason == "held pose differs from planned box exit"
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 1
    text = "\n".join(lines)
    assert text.count("session: part done") == 1
    assert "session: end (held pose differs from planned box exit)" in text
    assert "session: part done\n" in text  # released before the check
    assert sdk.calls.count("disable_all") == 1
    # One init send plus the return ramp endpoint plus the rest settle.
    assert len(_arm_rest_sends(sdk)) == SETTLE_HOLD_CYCLES + 2


def test_main_requires_inspect_without_touching_sdk():
    def factory(**kwargs):
        raise AssertionError("no SDK without --inspect")

    with pytest.raises(SystemExit) as info:
        pick_session.main(["--execute"], sdk_factory=factory)
    assert info.value.code == 2


def test_confirmation_missing_enables_nothing(tmp_path):
    lines: list = []
    sdk = StoredFakeSdk()
    stdin = io.StringIO(f"part {TABLE[0]} {TABLE[1]} 90\n")
    code = pick_session.execute_session(
        sdk, stdin, input_fn=lambda prompt: "no",
        out=lines.append, joint_limits=TEST_LIMITS)
    assert code == 2
    assert "confirmation missing" in "\n".join(lines)
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_parse_session_line_shapes():
    assert parse_session_line("part 500.0 360.0 89.8") == ("part", 500.0, 360.0, 89.8)
    assert parse_session_line("part 500.0 360.0") == ("part", 500.0, 360.0, None)
    assert parse_session_line("end") == ("end",)
    assert parse_session_line("bogus") is None
    assert parse_session_line("part 1 2 999") is None
    assert parse_session_line("part x y 10") is None
    assert session_driver.next_target(_queue_of([])) is None


def _queue_of(items):
    from muse_qc.pick.session_protocol import _EOF

    out = queue.Queue()
    for item in items:
        out.put(item)
    out.put(_EOF)
    return out


def test_install_resets_per_part_state():
    from muse_qc.pick.session_loop import SessionLoop

    arm, grip = FakeArmGroup(REST), FakeGripperGroup(REST_GRIP)
    plan = [("hover", tuple(REST), False), ("return", tuple(REST), False)]
    tips = [(0.3, 0.0, 0.1)] * 2
    loop = SessionLoop(
        arm, grip, VLIMS, [1.0], [math.radians(v) for v in REST],
        plan, tips, {"hover": (0, 1), "return": (1, 2)}, 0.1,
        -87.0, REST_GRIP, 50, False, object(), lambda text: None,
        lambda *args: {"ok": True}, _fake_tip, None, -40.0,
        inspect=True, ranges_by_box=None, shared_ranges=None,
        arm_sender=None, tracking_guard=None, hold_wait_s=3.0)
    loop.part_done.set()
    loop.drop_box = "good"
    loop.confirmed = True
    loop.phase = "hold_wait"
    plan2 = [("hover", tuple(REST), False)] * 2
    bundle = {"plan": plan2, "tips": [(0.3, 0.0, 0.1)] * 2,
              "ranges": {"hover": (0, 2)}, "hover_z": 0.1,
              "open_deg": -87.0, "release_open_deg": -40.0,
              "ranges_by_box": None, "shared_ranges": None,
              "hold_cycles": 50, "inspect": True}
    assert loop.load_next(bundle) is True
    loop._install(bundle)
    assert loop.phase == "hover"
    assert (loop.k, loop.end) == (0, 2)
    assert loop.part_done.is_set() is False
    assert loop.drop_box is None
    assert loop.confirmed is False


def test_plan_part_reports_refusal_without_moving():
    lines: list = []
    ok, reason = session_driver.plan_part(
        REST, TABLE, None, None, lines.append,
        {"grip_start_deg": REST_GRIP, "grip_z_mm": 8.0, "open_mm": 28.0,
         "hold_s": 0.5, "joint_limits": TEST_LIMITS,
         "solve_fn": lambda *a: {"ok": False, "reason": "no IK"},
         "check_fn": None, "tip_fn": _fake_tip, "use_xy_correction": True,
         "speed_deg_s": 15.0, "accel_deg_s2": 30.0, "solve_drop_fn": None,
         "use_station": False, "station_j6_deg": None})
    assert ok is None
    assert reason == "plan refused"


def test_count_record_saved_per_part(tmp_path, monkeypatch):
    # Record names resolve to seconds: tick the clock instead of
    # waiting (hardware parts are minutes apart).
    tick = _TickDatetime()
    monkeypatch.setattr("muse_qc.inspection.draw.datetime", tick)
    sdk, _, clock, thread, holder, lines = _begin_session(
        [f"part {TABLE[0]} {TABLE[1]} 90",
         f"part {TABLE2[0]} {TABLE2[1]}",
         "end"],
        tmp_path)
    loop = sdk.loop_fn
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop._pending is not None
               or loop.end_reason is not None or loop.done.is_set(),
               "planned part 2")
    sdk.pump(clock, 1)  # install
    assert _fly_part(sdk, clock, loop) == "part"
    _wait_main(lambda: loop.end_reason is not None or loop.done.is_set(),
               "ended the session")
    _pump_until(sdk, clock, loop.done.is_set, "returned to rest")
    assert _finish_session(thread, holder) == 0
    assert len(list(Path(tmp_path).glob("count_*.json"))) == 2


def _arm_rest_sends(sdk):
    rest = tuple(math.radians(v) for v in REST)
    return [call for call in sdk.groups["arm"].calls
            if call[0] == "send" and tuple(call[1]) == rest]
