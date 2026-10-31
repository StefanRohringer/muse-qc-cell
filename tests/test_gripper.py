"""Tests for gripper.py. No hardware, no serial port; serial is faked."""

from __future__ import annotations

import re
import time
from pathlib import Path

import pytest

import muse_qc.gripper.gripper as gripper
import muse_qc.gripper.gripper_watch as gripper_watch
from muse_qc.gripper.gripper import (
    COMMANDS,
    PICO_USB_VID,
    RAW_MODE_UT,
    Reading,
    TouchSensor,
    ensure_deviation_mode,
    find_pico_port,
    parse_line,
)
from muse_qc.gripper.gripper_watch import (
    _firmware_version,
    main,
    watch,
)

GOOD = (
    "D,1234,1.0,2.0,3.0,10.0,4.0,5.0,6.0,20.0,"
    "7.0,8.0,9.0,30.0,10.0,11.0,12.0,40.0,L1 R1"
)
ZERO = "D,5,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,-"


class FakeSerial:
    """Minimal pyserial stand-in: queued input lines, recorded writes.

    With generator set, readline never runs dry (gapless stream like the
    real Pico) and in_waiting stays positive; sleep_per_line simulates
    the streaming pace.
    """

    def __init__(self, *args, **kwargs):
        self.port = args[0] if args else None
        self.written: list[bytes] = []
        self.queued: list[bytes] = list(kwargs.pop("queued", []))
        self.generator = kwargs.pop("generator", None)
        self.sleep_per_line = kwargs.pop("sleep_per_line", 0.0)
        self.timeout = kwargs.pop("timeout", 0.1)
        self.closed = False

    @property
    def in_waiting(self) -> int:
        if self.queued:
            return sum(len(line) for line in self.queued)
        return 64 if self.generator is not None else 0

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def readline(self) -> bytes:
        if self.sleep_per_line:
            time.sleep(self.sleep_per_line)
        if self.queued:
            return self.queued.pop(0)
        if self.generator is not None:
            return self.generator()
        return b""

    def close(self) -> None:
        self.closed = True


def _stream_line(ms: int, contacts: str) -> str:
    """One valid fast-mode data line (magnitudes around 20 uT)."""
    return (
        f"D,{ms},1.0,2.0,3.0,10.0,4.0,5.0,6.0,20.0,"
        f"7.0,8.0,9.0,30.0,10.0,11.0,12.0,40.0,{contacts}"
    )


def _sensor(*lines: str) -> TouchSensor:
    sensor = TouchSensor(serial_factory=FakeSerial)
    sensor.open("/dev/ttyFAKE")
    sensor._serial.queued = [line.encode("ascii") + b"\r\n" for line in lines]
    return sensor


def _make_tty(root: Path, name: str, vid: str | None, pid: str | None) -> None:
    device = root / name / "device"
    device.mkdir(parents=True, exist_ok=True)
    if vid is not None:
        (device / "idVendor").write_text(vid + "\n", encoding="utf-8")
    if pid is not None:
        (device / "idProduct").write_text(pid + "\n", encoding="utf-8")


def test_parse_good_line():
    reading = parse_line(GOOD)
    assert isinstance(reading, Reading)
    assert reading.ms == 1234
    assert reading.values == (
        (1.0, 2.0, 3.0, 10.0),
        (4.0, 5.0, 6.0, 20.0),
        (7.0, 8.0, 9.0, 30.0),
        (10.0, 11.0, 12.0, 40.0),
    )
    assert reading.contacts == frozenset({"L1", "R1"})
    hash(reading)


def test_parse_dash_contacts_is_empty():
    assert parse_line(ZERO).contacts == frozenset()


def test_parse_crlf_and_spaces():
    reading = parse_line("  D, 7 , 0,0,0,2, 0,0,0,2,0,0,0,2,0,0,0,2 , R2 \r\n")
    assert reading.ms == 7
    assert reading.contacts == frozenset({"R2"})


@pytest.mark.parametrize(
    "line",
    ["", "garbage", "K,dx,dy,dz,mag", "# firmware 2", "X,1,2,3", "d,1,2,3"],
)
def test_parse_ignores_non_data_lines(line):
    assert parse_line(line) is None


@pytest.mark.parametrize(
    "line",
    [
        "D,1,2",
        "D,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,-",
        GOOD + ",extra",
        "D,,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,-",
        "D,-3,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,-",
        "D,abc,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,-",
        "D,5,0,0,X,1,0,0,0,1,0,0,0,1,0,0,0,1,-",
        "D,5,0,0,nan,1,0,0,0,1,0,0,0,1,0,0,0,1,-",
        "D,5,0,0,inf,1,0,0,0,1,0,0,0,1,0,0,0,1,-",
        "D,5,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,",
        "D,5,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,L3",
        "D,5,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,l1",
    ],
)
def test_parse_rejects_bad_shapes(line):
    assert parse_line(line) is None


def test_send_writes_single_bytes():
    sensor = _sensor()
    sensor.send("f")
    sensor.send("n")
    assert sensor._serial.written == [b"f", b"n"]


def test_send_refuses_anything_else():
    sensor = _sensor()
    for bad in ["x", "F", "ff", "", "zero", None, 102]:
        with pytest.raises(ValueError):
            sensor.send(bad)
    assert sensor._serial.written == []


def test_send_and_read_need_open():
    sensor = TouchSensor(serial_factory=FakeSerial)
    with pytest.raises(RuntimeError):
        sensor.send("f")
    with pytest.raises(RuntimeError):
        sensor.read(0.1)
    sensor.close()


def test_read_returns_newest_reading():
    sensor = _sensor(ZERO, GOOD)
    reading = sensor.read(1.0)
    assert reading.ms == 1234
    assert reading.contacts == frozenset({"L1", "R1"})


def test_read_skips_garbage_and_returns_none_without_data():
    sensor = _sensor("K,hdr", "# info", "junk", "")
    assert sensor.read(0.5) is None
    assert sensor.lines_read == 4


def test_context_manager_closes():
    sensor = TouchSensor(serial_factory=FakeSerial)
    sensor.open("/dev/ttyFAKE")
    with sensor:
        assert sensor._serial is not None
    assert sensor._serial is None


def test_find_pico_port_matches_by_id_not_number(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e8a", "000a")
    _make_tty(tmp_path, "ttyACM1", "2e88", "4603")
    assert find_pico_port(tmp_path) == "/dev/ttyACM0"


def test_find_pico_port_none_without_pico(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e88", "4603")
    assert find_pico_port(tmp_path) is None
    assert find_pico_port(tmp_path / "no-such-dir") is None


def test_main_no_pico_exits_2(monkeypatch, capsys):
    monkeypatch.setattr(gripper_watch, "find_pico_port", lambda *a, **k: None)
    assert main(["--watch", "1"]) == 2
    assert "not found" in capsys.readouterr().out


@pytest.mark.parametrize("value", ["0", "-5", "0.5", "121", "abc"])
def test_main_watch_range_exits_2(value):
    with pytest.raises(SystemExit) as info:
        main(["--watch", value])
    assert info.value.code == 2


def test_watch_stats_and_change_lines(capsys):
    lines = [
        ZERO,
        ZERO.replace(",-", ",L1"),
        GOOD,
        GOOD.replace(",L1 R1", ",-"),
        "junk",
    ]
    sensor = _sensor(*lines)
    stats = watch(sensor, 0.2)
    assert stats["data"] == 4
    assert stats["ignored"] == 1
    assert stats["changes"] == 3
    assert stats["events"] == {"L1": 1, "L2": 0, "R1": 1, "R2": 0}
    assert stats["max_mag"] == {"L1": 10.0, "L2": 20.0, "R1": 30.0, "R2": 40.0}
    out = capsys.readouterr().out
    assert "contacts: L1" in out
    assert "safety open: ok" in out
    assert "summary:" in out
    assert "max magnitude uT (as sent by the firmware):" in out


def test_watch_empty_contacts_refused_by_safety(capsys):
    sensor = _sensor(ZERO.replace(",-", ",L1"))
    stats = watch(sensor, 0.2)
    assert stats["changes"] == 1
    assert "unexpected-contact" in capsys.readouterr().out


def test_no_hardware_strings_in_module():
    text = (Path(__file__).parent.parent / "muse_qc" / "gripper" / "gripper.py").read_text(
        encoding="utf-8"
    )
    for banned in ("send_pos_vel", "start_control_loop", ".enable", "--execute", ".env"):
        assert banned not in text, banned


def test_allowed_commands():
    assert COMMANDS == frozenset({"n", "a", "f", "l", "k", "v"})
    assert PICO_USB_VID == "2e8a"


def test_read_line_returns_decoded_stripped_line():
    sensor = _sensor("  " + ZERO + "  ")
    assert sensor.read_line(1.0) == ZERO
    assert sensor.lines_read == 1


def test_read_line_none_on_timeout_blank_and_closed():
    sensor = _sensor("   ")
    assert sensor.read_line(0.5) is None
    assert sensor.lines_read == 1
    assert sensor._serial.timeout == 0.1
    assert _sensor().read_line(0.5) is None
    with pytest.raises(RuntimeError):
        TouchSensor(serial_factory=FakeSerial).read_line(0.1)


def test_watch_reports_changes_live_on_endless_stream(capsys):
    """Bug 1 repro: a gapless stream must print changes live, not at the end."""
    names = ["L1", "L2", "R1", "R2"]
    state = {"ms": 0}

    def generate():
        state["ms"] += 1
        line = _stream_line(state["ms"], names[state["ms"] % 4])
        return (line + "\r\n").encode("ascii")

    sensor = TouchSensor(
        serial_factory=lambda *a, **k: FakeSerial(
            *a, sleep_per_line=0.02, generator=generate, **k
        )
    )
    sensor.open("/dev/ttyFAKE")
    stats = watch(sensor, 0.4)
    assert stats["changes"] > 3
    stamps = [
        float(match)
        for match in re.findall(r"^\+([0-9.]+)s contacts:", capsys.readouterr().out, re.M)
    ]
    assert len(stamps) == stats["changes"]
    assert stamps[0] < 0.3
    assert stamps[-1] > stamps[0]
    assert all(earlier <= later for earlier, later in zip(stamps, stamps[1:]))


def test_read_returns_newest_within_timeout_on_endless_stream():
    state = {"ms": 0}

    def generate():
        state["ms"] += 1
        return (_stream_line(state["ms"], "-") + "\r\n").encode("ascii")

    sensor = TouchSensor(
        serial_factory=lambda *a, **k: FakeSerial(*a, generator=generate, **k)
    )
    sensor.open("/dev/ttyFAKE")
    start = time.monotonic()
    reading = sensor.read(0.2)
    assert time.monotonic() - start < 0.7
    assert reading is not None and reading.ms > 10


@pytest.mark.parametrize(
    ("queued", "expected"),
    [
        (["# firmware 2"], 2),
        (["junk", GOOD, "# firmware 3"], 3),
        (["# Ausgabe: Rohwerte"], None),
        ([GOOD.replace("D,1234", "D,2")], None),
        ([], None),
    ],
)
def test_firmware_version_parsing(monkeypatch, queued, expected):
    """Bug 2: only "# firmware <N>" counts, never a 2 inside a data line."""
    monkeypatch.setattr(gripper_watch, "VERSION_WAIT_S", 0.05)
    assert _firmware_version(_sensor(*queued)) == expected


def test_main_watch_ok_with_fakes(monkeypatch, capsys):
    monkeypatch.setattr(gripper_watch, "find_pico_port", lambda *a, **k: "/dev/ttyACM1")
    cycle = [ZERO, ZERO.replace(",-", ",L1")]
    state = {"i": 0}

    def generate():
        line = cycle[state["i"] % 2]
        state["i"] += 1
        return (line + "\r\n").encode("ascii")

    created: list[FakeSerial] = []

    def fake_serial(*args, **kwargs):
        serial_obj = FakeSerial(
            *args,
            queued=[b"# firmware 2\r\n"],
            generator=generate,
            sleep_per_line=0.02,
            **kwargs,
        )
        created.append(serial_obj)
        return serial_obj

    monkeypatch.setattr(gripper_watch.serial, "Serial", fake_serial)
    monkeypatch.setattr(gripper_watch.time, "sleep", lambda s: None)
    assert main(["--watch", "1"]) == 0
    out = capsys.readouterr().out
    assert "pico: /dev/ttyACM1" in out
    assert "firmware: 2" in out
    assert "do not touch the fingers" in out
    assert "output mode: deviation" in out
    assert "contacts: L1" in out
    assert "summary:" in out
    assert created[0].written == [b"v", b"n", b"f"]
    assert created[0].closed


def _data_line(ms: int, mags: tuple[float, float, float, float],
               contacts: str = "-") -> str:
    """One data line with the given per-sensor magnitudes."""
    vals = ",".join(f"0,0,0,{m}" for m in mags)
    return f"D,{ms},{vals},{contacts}"


DEV_MAGS = (10.0, 20.0, 30.0, 40.0)
RAW_MAGS = (7640.0, 9775.0, 7950.0, 7228.0)


class SwitchingSerial(FakeSerial):
    """Raw stream until 'a' is sent, deviation after (like the firmware)."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._ms = 0

    def readline(self) -> bytes:
        self._ms += 1
        mags = DEV_MAGS if b"a" in self.written else RAW_MAGS
        return (_data_line(self._ms, mags) + "\r\n").encode("ascii")


def test_raw_mode_threshold():
    assert RAW_MODE_UT == 1000.0


def test_output_mode_already_deviation_sends_no_a(capsys):
    sensor = _sensor(*[_data_line(ms, DEV_MAGS) for ms in range(1, 8)])
    assert ensure_deviation_mode(sensor, retry_s=0.0) == "deviation"
    assert sensor._serial.written == []
    assert "output mode: deviation" in capsys.readouterr().out


def test_output_mode_raw_then_deviation_sends_a_once(capsys):
    sensor = TouchSensor(
        serial_factory=lambda *a, **k: SwitchingSerial(*a, **k)
    )
    sensor.open("/dev/ttyFAKE")
    assert ensure_deviation_mode(sensor, retry_s=0.0) == "deviation"
    assert sensor._serial.written == [b"a"]
    assert "output mode: deviation" in capsys.readouterr().out


def test_output_mode_raw_stays_raw_sends_a_once_only(capsys):
    state = {"ms": 0}

    def generate():
        state["ms"] += 1
        return (_data_line(state["ms"], RAW_MAGS) + "\r\n").encode("ascii")

    sensor = TouchSensor(
        serial_factory=lambda *a, **k: FakeSerial(*a, generator=generate, **k)
    )
    sensor.open("/dev/ttyFAKE")
    assert ensure_deviation_mode(sensor, retry_s=0.0) == "raw (a sent, still raw)"
    assert sensor._serial.written == [b"a"]
    assert "output mode: raw (a sent, still raw)" in capsys.readouterr().out
