"""Gripper touch sensor reader over USB serial (moves nothing).

Pre-existing hardware (not built here): Fin-Ray fingers with four
magnet sensors (L1, L2 left, R1, R2 right), read by a Pico (USB VID
2e8a, 115200 baud) streaming one text reading per line. Only this
reader is new. Protocol: data "D,<ms>,<16 dx,dy,dz,mag>,<contacts>"
("L1 R1" or "-"); "K," headers, "#" info, rest ignored. Commands: n
zero, a deviation, f fast (~50/s), l slow, k header, v version (2).
Port matched by USB VID, never by fixed number. Human-run only.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

from muse_qc import safety
from muse_qc.usb_devices import find_serial_ports

PICO_USB_VID = "2e8a"
BAUDRATE = 115200
LINE_TIMEOUT_S = 0.1
VERSION_WAIT_S = 1.0
ZERO_SETTLE_S = 1.0
FAST_DRAIN_S = 0.3
EXPECTED_FIRMWARE_VERSION = 2
SENSOR_NAMES = ("L1", "L2", "R1", "R2")
SENSOR_SET = frozenset(SENSOR_NAMES)
COMMANDS = frozenset({"n", "a", "f", "l", "k", "v"})
DATA_FIELDS = 19  # "D" + ms + 16 values + contacts
WATCH_DEFAULT_S = 10.0
WATCH_MIN_S = 1.0
WATCH_MAX_S = 120.0


@dataclass(frozen=True)
class Reading:
    """One data line: (dx, dy, dz, magnitude) per SENSOR_NAMES."""

    ms: int
    values: tuple[tuple[float, float, float, float], ...]
    contacts: frozenset[str]


def parse_line(text: str) -> Reading | None:
    """One line to Reading, else None. Pure: headers, partial lines and
    garbage never yield a reading (19 fields, finite numbers, known names)."""
    if not isinstance(text, str):
        return None
    fields = text.strip().split(",")
    if len(fields) != DATA_FIELDS or fields[0] != "D":
        return None
    try:
        ms = int(fields[1])
    except ValueError:
        return None
    if ms < 0:
        return None
    flat: list[float] = []
    for raw in fields[2:18]:
        try:
            value = float(raw)
        except ValueError:
            return None
        if not math.isfinite(value):
            return None
        flat.append(value)
    contacts_raw = fields[18].strip()
    if not contacts_raw:
        return None
    if contacts_raw == "-":
        contacts = frozenset()
    else:
        parts = contacts_raw.split()
        if not parts or any(p not in SENSOR_SET for p in parts):
            return None
        contacts = frozenset(parts)
    grouped = tuple((flat[i], flat[i + 1], flat[i + 2], flat[i + 3])
                    for i in range(0, 16, 4))
    return Reading(ms=ms, values=grouped, contacts=contacts)


def _present_pids(sysfs_root: str | os.PathLike[str]) -> list[str]:
    """Distinct idProduct values on serial tty entries (enum only)."""
    try:
        entries = sorted(Path(sysfs_root).iterdir())
    except OSError:
        return []
    pids: list[str] = []
    for entry in entries:
        if not entry.name.startswith(("ttyACM", "ttyUSB")):
            continue
        device = entry / "device"
        if not os.path.lexists(device):
            continue
        try:
            node = device.resolve()
        except OSError:
            continue
        for _ in range(4):
            try:
                pid = (node / "idProduct").read_text(encoding="utf-8").strip().lower()
            except OSError:
                pid = None
            if pid is not None:
                if pid not in pids:
                    pids.append(pid)
                break
            if node.parent == node:
                break
            node = node.parent
    return pids


def find_pico_port(sysfs_root="/sys/class/tty") -> str | None:
    """Pico /dev path (USB VID 2e8a), or None when absent."""
    for pid in _present_pids(sysfs_root):
        ports = find_serial_ports(PICO_USB_VID, pid, sysfs_root)
        if ports:
            return sorted(ports)[0]
    return None


class TouchSensor:
    """USB serial touch sensor reader (moves nothing)."""

    def __init__(self, serial_factory=None) -> None:
        self._serial_factory = serial_factory
        self._serial = None
        self.lines_read = 0

    def open(self, port: str) -> None:
        """Open the Pico port at 115200 baud. Raises on failure."""
        factory = self._serial_factory or serial.Serial
        self._serial = factory(str(port), BAUDRATE, timeout=LINE_TIMEOUT_S)

    def close(self) -> None:
        """Close the port. Safe when already closed."""
        serial_obj, self._serial = self._serial, None
        if serial_obj is not None:
            serial_obj.close()

    def __enter__(self) -> TouchSensor:
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def _require_open(self) -> None:
        if self._serial is None:
            raise RuntimeError("TouchSensor is not open; call open(port) first.")

    def send(self, cmd: str) -> None:
        """Write one firmware command char; refuse anything else."""
        if cmd not in COMMANDS:
            raise ValueError(f"unknown command {cmd!r}; allowed: {sorted(COMMANDS)}.")
        self._require_open()
        self._serial.write(cmd.encode("ascii"))

    def _waiting(self) -> int:
        """Buffered input bytes (0 when the serial object has no in_waiting)."""
        return getattr(self._serial, "in_waiting", 0) or 0

    def read_line(self, timeout_s: float) -> str | None:
        """One decoded stripped line; None on timeout or blank line."""
        self._require_open()
        before = self._serial.timeout
        self._serial.timeout = max(0.0, timeout_s)
        try:
            raw = self._serial.readline()
        finally:
            self._serial.timeout = before
        if not raw:
            return None
        self.lines_read += 1
        return raw.decode("ascii", errors="replace").strip() or None

    def read(self, timeout_s: float) -> Reading | None:
        """Newest reading available now; back within about timeout_s."""
        self._require_open()
        deadline = time.monotonic() + max(0.0, timeout_s)
        latest = None
        while self._waiting() > 0 and time.monotonic() < deadline:
            reading = parse_line(self.read_line(max(0.0, deadline - time.monotonic())))
            if reading is not None:
                latest = reading
        if latest is None:
            latest = parse_line(self.read_line(max(0.0, deadline - time.monotonic())))
        return latest


def _parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Watch gripper sensors (moves nothing).")
    parser.add_argument("--watch", metavar="SECONDS", type=float, default=WATCH_DEFAULT_S,
                        help=f"watch time, {WATCH_MIN_S:g}-{WATCH_MAX_S:g}s (default {WATCH_DEFAULT_S:g})")
    args = parser.parse_args(argv)
    if not WATCH_MIN_S <= args.watch <= WATCH_MAX_S:
        parser.error(f"--watch must be {WATCH_MIN_S:g} to {WATCH_MAX_S:g}s, got {args.watch}.")
    return args


def _show(contacts: frozenset[str]) -> str:
    return "-" if not contacts else " ".join(sorted(contacts))


def _firmware_version(sensor: TouchSensor) -> int | None:
    """Send v; the N in "# firmware <N>" within VERSION_WAIT_S, else None."""
    sensor.send("v")
    deadline = time.monotonic() + VERSION_WAIT_S
    while time.monotonic() < deadline:
        parts = (sensor.read_line(max(0.0, deadline - time.monotonic())) or "").split()
        if parts[:2] == ["#", "firmware"] and len(parts) == 3 and parts[2].isdigit():
            return int(parts[2])
    return None


def watch(sensor: TouchSensor, duration_s: float) -> dict:
    """One line per contact change plus a summary; return stats."""
    start = time.monotonic()
    deadline = start + duration_s
    previous: frozenset[str] = frozenset()
    base_lines = sensor.lines_read
    data = 0
    changes = 0
    events = dict.fromkeys(SENSOR_NAMES, 0)
    max_mag = dict.fromkeys(SENSOR_NAMES, 0.0)
    print(f"start: contacts {_show(previous)}")
    try:
        while time.monotonic() < deadline:
            reading = parse_line(sensor.read_line(max(0.0, deadline - time.monotonic())))
            if reading is None:
                continue
            data += 1
            for i, name in enumerate(SENSOR_NAMES):
                max_mag[name] = max(max_mag[name], reading.values[i][3])
            if reading.contacts != previous:
                for name in reading.contacts - previous:
                    events[name] += 1
                previous = reading.contacts
                changes += 1
                verdict = safety.check_gripper("open", reading.contacts)
                note = "safety open: ok" if verdict.get("ok") else \
                    f"safety open: {verdict.get('reason')}"
                print(f"+{time.monotonic() - start:.1f}s contacts: {_show(previous)} ({note})")
    except KeyboardInterrupt:
        print("interrupted; partial summary below")
    lines = sensor.lines_read - base_lines
    print(f"summary: lines={lines} data={data} ignored={lines - data} changes={changes}")
    print("events: " + " ".join(f"{n}={events[n]}" for n in SENSOR_NAMES))
    print("max magnitude uT (as sent by the firmware): " + " ".join(f"{n}={max_mag[n]:.1f}" for n in SENSOR_NAMES))
    return {"lines": lines, "data": data, "ignored": lines - data, "changes": changes,
            "events": events, "max_mag": max_mag}


def main(argv=None) -> int:
    """Open the Pico, zero the sensors, watch contact changes."""
    args = _parse_args(argv)
    port = find_pico_port()
    if port is None:
        print(f"gripper Pico not found (USB VID {PICO_USB_VID}); ports match by USB"
              " ID, never by fixed number (/dev/ttyACM0 can be the arm).")
        return 2
    print(f"pico: {port}")
    sensor = TouchSensor()
    try:
        sensor.open(port)
    except Exception as exc:
        print(f"cannot open {port}: {exc}")
        return 1
    with sensor:
        version = _firmware_version(sensor)
        if version is None:
            print("firmware: no reply")
        else:
            print(f"firmware: {version}")
            if version != EXPECTED_FIRMWARE_VERSION:
                print(f"warning: expected firmware {EXPECTED_FIRMWARE_VERSION}; continuing")
        print("do not touch the fingers")
        time.sleep(ZERO_SETTLE_S)
        sensor.send("n")
        sensor.send("f")
        sensor.read(FAST_DRAIN_S)  # drop stale pre-zero lines
        watch(sensor, args.watch)
    if safety.is_stopped():
        # Phase "open" never latches STOP; only the user can clear it.
        print("safety STOP is latched; only the user can clear it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
