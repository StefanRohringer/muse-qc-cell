"""Gripper touch sensor reader over USB serial (moves nothing).

Pre-existing hardware (not built here): Fin-Ray fingers with four
magnet sensors (L1, L2 left, R1, R2 right), read by a Pico (USB VID
2e8a, 115200 baud) streaming one text reading per line. Only this
reader is new. Protocol: data "D,<ms>,<16 dx,dy,dz,mag>,<contacts>"
("L1 R1" or "-"); "K," headers, "#" info, rest ignored. Commands: n
zero, a deviation, f fast (~50/s), l slow, k header, v version (2).
Normal output is deviation mode (magnitudes near 0 at rest, rising on
contact); ensure_deviation_mode probes the stream after "n" and "f"
and sends "a" at most once when raw values are seen. The watch CLI
lives in muse_qc.gripper.gripper_watch. Port matched by USB VID, never by
fixed number. Human-run only.
"""

from __future__ import annotations

import math
import os
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import serial

from muse_qc.checks.usb_devices import find_serial_ports

PICO_USB_VID = "2e8a"
BAUDRATE = 115200
LINE_TIMEOUT_S = 0.1
VERSION_WAIT_S = 1.0
ZERO_SETTLE_S = 1.0
FAST_DRAIN_S = 0.3
EXPECTED_FIRMWARE_VERSION = 2
RAW_MODE_UT = 1000.0
MODE_PROBE_S = 0.5
MODE_RETRY_S = 0.3
PROBE_MAX_READINGS = 200
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


def _median_magnitude(sensor: TouchSensor, budget_s: float) -> float | None:
    """Median magnitude over all four sensors from the next live lines.

    Reads for up to budget_s, stopping early when no line arrives at
    once (idle port or spent queued input), so offline fakes return
    fast while the live gapless stream fills the whole budget. Returns
    None when no reading arrived.
    """
    mags: list[float] = []
    count = 0
    deadline = time.monotonic() + max(0.0, budget_s)
    while time.monotonic() < deadline and count < PROBE_MAX_READINGS:
        line = sensor.read_line(max(0.0, deadline - time.monotonic()))
        if line is None:
            break
        reading = parse_line(line)
        if reading is None:
            continue
        count += 1
        for value in reading.values:
            mags.append(value[3])
    if not mags:
        return None
    return statistics.median(mags)


def ensure_deviation_mode(sensor: TouchSensor, out=print,
                          probe_s: float = MODE_PROBE_S,
                          retry_s: float = MODE_RETRY_S) -> str:
    """Send "a" at most once when the stream still looks like raw values.

    Call after "n" and "f": probe the next readings for up to probe_s;
    when the median magnitude over all four sensors is above RAW_MODE_UT
    the firmware streams raw values, so send "a" once ("a" may be a
    toggle, so it is never sent twice), wait retry_s and probe again.
    Prints and returns "deviation" or "raw (a sent, still raw)".
    """
    median = _median_magnitude(sensor, probe_s)
    if median is not None and median > RAW_MODE_UT:
        sensor.send("a")
        time.sleep(retry_s)
        again = _median_magnitude(sensor, probe_s)
        if again is not None and again > RAW_MODE_UT:
            out("output mode: raw (a sent, still raw)")
            return "raw (a sent, still raw)"
    out("output mode: deviation")
    return "deviation"


def main(argv=None) -> int:
    """Thin entry point: the watch CLI lives in muse_qc.gripper.gripper_watch."""
    from muse_qc.gripper.gripper_watch import main as watch_main

    return watch_main(argv)


if __name__ == "__main__":
    sys.exit(main())
