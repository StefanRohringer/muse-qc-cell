"""Tests for gripper.py. No hardware, no serial port; serial is faked."""

from __future__ import annotations

from pathlib import Path

import pytest

import muse_qc.gripper as gripper
from muse_qc.gripper import (
    COMMANDS,
    PICO_USB_VID,
    Reading,
    TouchSensor,
    find_pico_port,
    main,
    parse_line,
    watch,
)

GOOD = (
    "D,1234,1.0,2.0,3.0,10.0,4.0,5.0,6.0,20.0,"
    "7.0,8.0,9.0,30.0,10.0,11.0,12.0,40.0,L1 R1"
)
ZERO = "D,5,0,0,0,1,0,0,0,1,0,0,0,1,0,0,0,1,-"


class FakeSerial:
    """Minimal pyserial stand-in: queued input lines, recorded writes."""

    def __init__(self, *args, **kwargs):
        self.port = args[0] if args else None
        self.written: list[bytes] = []
        self.queued: list[bytes] = list(kwargs.pop("queued", []))
        self.timeout = kwargs.pop("timeout", 0.1)
        self.closed = False

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def readline(self) -> bytes:
        if self.queued:
            return self.queued.pop(0)
        return b""

    def close(self) -> None:
        self.closed = True


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
    monkeypatch.setattr(gripper, "find_pico_port", lambda *a, **k: None)
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


def test_watch_empty_contacts_refused_by_safety(capsys):
    sensor = _sensor(ZERO.replace(",-", ",L1"))
    stats = watch(sensor, 0.2)
    assert stats["changes"] == 1
    assert "unexpected-contact" in capsys.readouterr().out


def test_no_hardware_strings_in_module():
    text = (Path(__file__).parent.parent / "muse_qc" / "gripper.py").read_text(
        encoding="utf-8"
    )
    for banned in ("send_pos_vel", "start_control_loop", ".enable", "--execute", ".env"):
        assert banned not in text, banned


def test_allowed_commands():
    assert COMMANDS == frozenset({"n", "a", "f", "l", "k", "v"})
    assert PICO_USB_VID == "2e8a"
