"""Tests for gripper_grip.py. Offline only: fake SDK, fake serial, no bus."""

from __future__ import annotations

import math

import pytest

import muse_qc.gripper_grip as gripper_grip
from muse_qc import safety
from muse_qc.first_move_plan import LOOP_RATE_HZ, clamp_to_limits
from muse_qc.gripper import LINE_TIMEOUT_S, ZERO_SETTLE_S, TouchSensor
from muse_qc.gripper_grip import deg_to_width, validate_targets, width_to_deg
from muse_qc.gripper_stream import open_touch_sensor

REST_DEG = [-3.33, -1.58, -1.45, 2.72, 0.91, -3.97]
REST_TIP_M = (0.2865, -0.0201, 0.1988)
REST_GRIP_DEG = -4.97
WRAPPED_GRIP_DEG = 33.21  # fully open by hand: wrapped, not closed
VLIMS = [5.0, 5.0, 5.0, 3.0, 3.0, 3.0]
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}
OPEN_DEG_40 = width_to_deg(40.0)  # about -126.23
# Setup lines before the loop starts: firmware (1) + deviation probe (200) + drain (1).
SETUP_LINES = 202
OPEN_RAMP_40 = math.ceil(abs(OPEN_DEG_40 - REST_GRIP_DEG) / 0.05)
CLOSE_START_N = SETUP_LINES + OPEN_RAMP_40 + 300


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
        self.reads = 0

    @property
    def joint_names(self):
        return ["gripper"]

    def get_positions(self):
        self.calls.append("get")
        self.reads += 1
        return list(self._rad)

    def mode_pos_vel(self, vlim=None):
        self.calls.append(("mode", [float(v) for v in vlim]))

    def send_pos_vel(self, pos, vlim):
        values = [float(v) for v in pos]
        self.calls.append(("send", values, [float(v) for v in vlim]))
        self._rad = list(values)


class JumpGripperGroup(FakeGripperGroup):
    """One mid-run reading jump of jump_deg at the given read count."""

    def __init__(self, joint_deg, jump_at, jump_deg=50.0):
        super().__init__(joint_deg)
        self.jump_at = jump_at
        self.jump_rad = math.radians(jump_deg)

    def get_positions(self):
        self.calls.append("get")
        self.reads += 1
        if self.reads == self.jump_at:
            return [self._rad[0] + self.jump_rad]
        return list(self._rad)


class FakeSdk:
    """Fake SDK object: control loop runs inline, never touches a bus."""

    def __init__(self, joints_deg=REST_DEG, grip_deg=REST_GRIP_DEG, grip_group=None):
        self.groups = {
            "arm": FakeArmGroup(joints_deg),
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
    """The bus fails when the control loop should start."""

    def start_control_loop(self, fn, rate=None):
        raise RuntimeError("bus down")


class FakeSerial:
    """Minimal pyserial stand-in with a line generator and timeout log."""

    def __init__(self, *args, **kwargs):
        self.port = args[0] if args else None
        self.written: list[bytes] = []
        self.generator = kwargs.pop("generator", None)
        self._timeout = kwargs.pop("timeout", 0.1)
        self.timeout_sets: list[float] = []
        self.closed = False

    @property
    def in_waiting(self) -> int:
        return 0  # one line per readline, like an idle-paced port

    @property
    def timeout(self) -> float:
        return self._timeout

    @timeout.setter
    def timeout(self, value: float) -> None:
        self._timeout = value
        self.timeout_sets.append(float(value))

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def readline(self) -> bytes:
        if self.generator is not None:
            return self.generator()
        return b""

    def close(self) -> None:
        self.closed = True


def _dline(number: int, contacts: str) -> bytes:
    """One valid fast-mode data line with small deviation magnitudes."""
    return ((f"D,{number}," + ",".join(["1.0", "2.0", "3.0", "10.0"] * 4)
             + f",{contacts}\r\n").encode("ascii"))


def _serial_factory(contacts_fn, serials):
    """TouchSensor factory: firmware 2 first, then contacts_fn(n) lines."""
    state = {"n": 0}

    def generator():
        state["n"] += 1
        if state["n"] == 1:
            return b"# firmware 2\r\n"
        return _dline(state["n"], contacts_fn(state["n"]))

    def make():
        serial = FakeSerial(generator=generator)
        serials.append(serial)
        sensor = TouchSensor(serial_factory=lambda *a, **k: serial)
        return sensor

    return make


def _contacts_after(first_n, contacts):
    return lambda n: contacts if n >= first_n else "-"


def _always_ok(target, current):
    return {"ok": True, "speed": 0.1, "detail": "stub"}


def _fake_tip(joints_deg):
    return REST_TIP_M


def _refuse(target, current):
    return {"ok": False, "reason": "test-refusal", "value": 0.0, "limit": 1.0}


def _arm_sends(sdk):
    return [call[1] for call in sdk.groups["arm"].calls if call[0] == "send"]


def _grip_sends_deg(sdk):
    return [math.degrees(call[1][0])
            for call in sdk.groups["gripper"].calls if call[0] == "send"]


def _run_execute(sdk, contacts_fn, serials, **kwargs):
    kwargs.setdefault("input_fn", lambda prompt: "yes")
    kwargs.setdefault("check_fn", _always_ok)
    kwargs.setdefault("tip_fn", _fake_tip)
    kwargs.setdefault("arm_vlims", VLIMS)
    kwargs.setdefault("joint_limits", TEST_LIMITS)
    kwargs.setdefault("find_port", lambda: "/dev/ttyFAKE")
    kwargs.setdefault("sensor_factory", _serial_factory(contacts_fn, serials))
    lines = []
    kwargs.setdefault("out", lines.append)
    result = gripper_grip.execute(sdk, 40.0, 2.0, **kwargs)
    return result, "\n".join(lines)


@pytest.fixture(autouse=True)
def _restore_safety():
    stop, retries, holding = safety._STOP_REASON, safety._GRIP_RETRIES, safety._HOLDING_CONTACTS
    yield
    safety._STOP_REASON, safety._GRIP_RETRIES, safety._HOLDING_CONTACTS = stop, retries, holding


def test_conversions_match_human_ruler_readings():
    assert width_to_deg(0.0) == pytest.approx(2.8)
    assert deg_to_width(2.8) == pytest.approx(0.0)
    assert width_to_deg(40.0) == pytest.approx(-126.23, abs=0.01)
    # 32 mm and 53 mm points sit near the fitted line.
    assert width_to_deg(32.0) == pytest.approx(-102.63, abs=3.0)
    assert width_to_deg(53.0) == pytest.approx(-167.13, abs=1.5)
    # The 5 mm point sits off the line (soft Fin-Ray tips): the fit says 2.4.
    assert deg_to_width(-4.97) == pytest.approx(2.4, abs=0.1)
    # Negative = open: a wider gap needs a smaller (more negative) angle.
    assert width_to_deg(50.0) < width_to_deg(20.0)
    assert deg_to_width(width_to_deg(17.0)) == pytest.approx(17.0)


def test_validate_targets():
    assert validate_targets(-4.97, OPEN_DEG_40) is None
    assert validate_targets(WRAPPED_GRIP_DEG, OPEN_DEG_40) == gripper_grip.OUT_OF_RANGE_MSG
    assert "about 30 mm" in validate_targets(WRAPPED_GRIP_DEG, OPEN_DEG_40)
    refused = validate_targets(-4.97, -179.0)
    assert refused is not None and "outside the safe range" in refused
    assert validate_targets(float("nan"), OPEN_DEG_40) == gripper_grip.OUT_OF_RANGE_MSG


@pytest.mark.parametrize("flag", ["--open-mm", "--squeeze-deg"])
@pytest.mark.parametrize("value", ["abc"])
def test_non_numeric_args_exit_2_without_connect(flag, value):
    def factory():
        raise AssertionError("nothing may connect on a bad argument")

    with pytest.raises(SystemExit) as info:
        gripper_grip.main([flag, value], sdk_factory=factory)
    assert info.value.code == 2


@pytest.mark.parametrize("value", ["19.9", "50.1", "0", "-5"])
def test_open_mm_range_exits_2_without_connect(value):
    def factory():
        raise AssertionError("nothing may connect on a bad --open-mm")

    with pytest.raises(SystemExit) as info:
        gripper_grip.main(["--open-mm", value], sdk_factory=factory)
    assert info.value.code == 2


@pytest.mark.parametrize("value", ["-0.1", "5.1"])
def test_squeeze_range_exits_2_without_connect(value):
    def factory():
        raise AssertionError("nothing may connect on a bad --squeeze-deg")

    with pytest.raises(SystemExit) as info:
        gripper_grip.main(["--squeeze-deg", value], sdk_factory=factory)
    assert info.value.code == 2


def test_dry_run_never_enables_modes_loops_or_sends():
    sdk = FakeSdk()
    serials = []
    lines = []
    result = gripper_grip.dry_run(
        sdk, 40.0, 2.0, out=lines.append, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=_serial_factory(lambda n: "-", serials))
    assert result == 0
    assert "connect" in sdk.calls and "disconnect" in sdk.calls
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    for group in sdk.groups.values():
        assert not any(isinstance(call, tuple) and call[0] in ("mode", "send")
                       for call in group.calls)
    text = "\n".join(lines)
    assert "rest joints deg:" in text
    assert "gripper rest deg: -4.97" in text
    assert "open width: 40 mm -> gripper -126.23 deg" in text
    assert "squeeze: 2.0 deg past first bilateral contact" in text
    assert "dry run: moves nothing, motors never enabled" in text
    serial = serials[0]
    assert b"n" in serial.written and b"f" in serial.written
    assert b"a" not in serial.written  # already in deviation mode
    assert serial.closed
    assert "firmware: 2" in text and "output mode: deviation" in text


def test_dry_run_without_pico_returns_2():
    sdk = FakeSdk()
    lines = []
    result = gripper_grip.dry_run(
        sdk, 40.0, 2.0, out=lines.append, find_port=lambda: None,
        sensor_factory=lambda: (_ for _ in ()).throw(AssertionError("no sensor")))
    assert result == 2
    assert "enable_all" not in sdk.calls
    assert any("not found" in line for line in lines)


def test_dry_run_wrapped_start_refused():
    sdk = FakeSdk(grip_deg=WRAPPED_GRIP_DEG)
    lines = []
    result = gripper_grip.dry_run(
        sdk, 40.0, 2.0, out=lines.append, find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=lambda: (_ for _ in ()).throw(AssertionError("no sensor")))
    assert result == 1
    assert any("outside the safe range" in line for line in lines)
    assert "enable_all" not in sdk.calls


def test_execute_bilateral_contact_stops_closing_and_adds_squeeze():
    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(sdk, _contacts_after(CLOSE_START_N + 22, "L1 R1"), serials)
    assert result == 0
    assert sdk.loop_rate == LOOP_RATE_HZ == 100.0
    assert "place the part between the jaws now" in text
    contact_line = next(line for line in text.splitlines() if line.startswith("contact:"))
    contact_deg = float(contact_line.split("commanded deg")[1].split()[0])
    assert contact_deg < -100.0  # stopped early in the close ramp, far from closed
    assert "grip held at deg" in text
    assert "hold end: gripper commanded deg" in text
    assert "gripper final deg:" in text
    assert "missed grip" not in text
    assert "motion finished, gripper back at the open width" in text
    hold_line = next(line for line in text.splitlines() if line.startswith("grip held at deg"))
    hold_deg = float(hold_line.split("grip held at deg")[1].split()[0])
    assert hold_deg == pytest.approx(contact_deg + 2.0, abs=0.1)  # squeeze added
    assert hold_deg < -100.0  # held far from closed, never drove further
    sends = _grip_sends_deg(sdk)
    assert sends[-1] == pytest.approx(OPEN_DEG_40)  # reopened to the open width
    assert not safety.is_stopped()  # single safety confirmation burns no retries
    assert "disable_all" in sdk.calls and "disconnect" in sdk.calls
    assert serials and serials[0].closed


def test_execute_open_phase_contacts_only_printed_never_stop():
    def contacts(n):
        if 500 <= n <= 510:
            return "L1 R2"  # hand touches a finger while placing
        if n >= CLOSE_START_N + 22:
            return "L1 R1"
        return "-"

    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(sdk, contacts, serials)
    assert result == 0
    assert "contacts L1 R2 (placing phase, ignored)" in text
    assert "grip held at deg" in text
    assert "missed grip" not in text


@pytest.mark.parametrize("contacts", ["L1", "R1 R2"])
def test_execute_one_sided_contact_keeps_closing(contacts):
    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(sdk, _contacts_after(CLOSE_START_N + 22, contacts), serials)
    assert result == 0
    assert "missed grip" in text
    assert "grip held at deg" not in text
    sends = _grip_sends_deg(sdk)
    assert max(sends) == pytest.approx(2.8)  # reached CLOSED_DEG
    assert sends[-1] == pytest.approx(OPEN_DEG_40)


def test_execute_no_contact_reaches_closed_and_prints_missed_grip():
    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(sdk, lambda n: "-", serials)
    assert result == 0
    assert "missed grip" in text
    assert "motion finished without a held grip" in text
    assert max(_grip_sends_deg(sdk)) == pytest.approx(2.8)


def test_execute_wrapped_start_refused_before_enabling():
    sdk = FakeSdk(grip_deg=WRAPPED_GRIP_DEG)
    serials = []
    result, text = _run_execute(
        sdk, lambda n: "-", serials,
        input_fn=lambda prompt: (_ for _ in ()).throw(AssertionError("never ask")),
        sensor_factory=lambda: (_ for _ in ()).throw(AssertionError("never open")))
    assert result == 1
    assert gripper_grip.OUT_OF_RANGE_MSG in text
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert not _grip_sends_deg(sdk)
    assert "disconnect" in sdk.calls


def test_execute_plan_target_outside_range_refused(monkeypatch):
    monkeypatch.setattr(gripper_grip, "width_to_deg", lambda mm: -179.0)
    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(
        sdk, lambda n: "-", serials,
        input_fn=lambda prompt: (_ for _ in ()).throw(AssertionError("never ask")))
    assert result == 1
    assert "outside the safe range" in text
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls


def test_execute_reading_jump_stops_closing_holds_and_disables():
    group = JumpGripperGroup(REST_GRIP_DEG, jump_at=1 + OPEN_RAMP_40 + 300 + 50)
    sdk = FakeSdk(grip_group=group)
    serials = []
    result, text = _run_execute(sdk, lambda n: "-", serials)
    assert result == 1
    assert "jumped" in text and "holding position" in text
    assert "disable_all" in sdk.calls and "stop" in sdk.calls
    assert "disconnect" in sdk.calls
    sends = _grip_sends_deg(sdk)
    assert len(sends) < 20000  # stopped mid-run instead of running on
    assert sends[-1] < -50.0  # held far from closed, never reached it


def test_execute_sends_arm_hold_every_cycle_at_ten_percent():
    sdk = FakeSdk()
    serials = []
    result, _ = _run_execute(sdk, lambda n: "-", serials)
    assert result == 0
    clamped = list(clamp_to_limits(REST_DEG, TEST_LIMITS))
    expected_rad = [math.radians(v) for v in clamped]
    sends = _arm_sends(sdk)
    assert sends, "expected arm hold commands"
    for values in sends:
        assert values == pytest.approx(expected_rad)
    slow = [v * 0.1 for v in VLIMS]
    assert ("mode", slow) in sdk.groups["arm"].calls
    grip_modes = [call[1] for call in sdk.groups["gripper"].calls if call[0] == "mode"]
    assert grip_modes and grip_modes[0] == pytest.approx([3.0 * 0.1])


def test_execute_without_yes_never_enables():
    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(sdk, lambda n: "-", serials,
                                input_fn=lambda prompt: "no")
    assert result == 2
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert not _arm_sends(sdk) and not _grip_sends_deg(sdk)
    assert "disconnect" in sdk.calls


def test_execute_refused_hold_moves_nothing():
    sdk = FakeSdk()
    serials = []
    result, text = _run_execute(sdk, lambda n: "-", serials, check_fn=_refuse)
    assert result == 1
    assert "enable_all" not in sdk.calls and "loop" not in sdk.calls
    assert any("refused" in line for line in text.splitlines())


def test_loop_exception_still_disables():
    sdk = FailingLoopSdk()
    serials = []
    result, text = _run_execute(sdk, lambda n: "-", serials)
    assert result == 1
    assert "disable_all" in sdk.calls and "stop" in sdk.calls
    assert "disconnect" in sdk.calls
    assert any("motion failed: bus down" in line for line in text.splitlines())


def test_sensor_read_uses_short_timeout_every_cycle():
    sdk = FakeSdk()
    serials = []
    result, _ = _run_execute(sdk, lambda n: "-", serials)
    assert result == 0
    sets = serials[0].timeout_sets
    assert len(sets) > 2000  # one bounded read per control cycle
    # Loop-phase reads use the ~2 ms budget; read_line restores the idle
    # port default (LINE_TIMEOUT_S) after each read, which never blocks.
    tail = sets[-4000:]  # each read sets the budget, then restores the default
    assert all(value <= 0.005 or value == pytest.approx(LINE_TIMEOUT_S) for value in tail)
    assert sum(1 for value in tail if value <= 0.005) > 1500


def test_execute_main_entry_point_with_defaults():
    sdk = FakeSdk()
    serials = []
    lines = []
    result = gripper_grip.main(
        ["--open-mm", "40", "--execute"], sdk_factory=lambda: sdk,
        input_fn=lambda prompt: "yes", check_fn=_always_ok, tip_fn=_fake_tip,
        arm_vlims=VLIMS, joint_limits=TEST_LIMITS,
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=_serial_factory(lambda n: "-", serials), out=lines.append)
    assert result == 0
    assert any("missed grip" in line for line in lines)


def test_open_sensor_warns_and_settles_before_zero(monkeypatch):
    import time

    assert ZERO_SETTLE_S == 1.0  # same settle as the --watch CLI
    serials = []
    lines = []
    n_already_sent = []
    sleeps = []

    def fake_sleep(seconds):
        sleeps.append(seconds)
        n_already_sent.append(b"n" in serials[0].written)

    monkeypatch.setattr(time, "sleep", fake_sleep)
    sensor = open_touch_sensor(
        find_port=lambda: "/dev/ttyFAKE",
        sensor_factory=_serial_factory(lambda n: "-", serials), out=lines.append)
    assert sensor is not None
    sensor.close()
    assert "do not touch the fingers" in lines
    assert lines.index("firmware: 2") < lines.index("do not touch the fingers")
    assert sleeps == [ZERO_SETTLE_S]  # exactly one 1 s settle
    assert n_already_sent == [False]  # the settle happened before "n" was sent
    assert serials[0].written.index(b"n") < serials[0].written.index(b"f")


def test_source_avoids_homing_primitives_and_foreign_code():
    from pathlib import Path

    text = Path("muse_qc/gripper_grip.py").read_text(encoding="utf-8")
    for forbidden in ("set_zero", "safe_home", "estop", "RebotArmEndPose("):
        assert forbidden not in text
    assert "gripper_stream" in text  # loop code lives in the shared module
