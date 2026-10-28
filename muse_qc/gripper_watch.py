"""Gripper touch sensor watch CLI (moves nothing). Human-run only.

Moved out of muse_qc.gripper so that module stays at or below 300
lines; python -m muse_qc.gripper still works through its thin entry
point. The sensor protocol stays in muse_qc.gripper.
"""

from __future__ import annotations

import argparse
import sys
import time

import serial

from muse_qc import safety
from muse_qc.gripper import (
    EXPECTED_FIRMWARE_VERSION,
    FAST_DRAIN_S,
    PICO_USB_VID,
    SENSOR_NAMES,
    VERSION_WAIT_S,
    WATCH_DEFAULT_S,
    WATCH_MAX_S,
    WATCH_MIN_S,
    ZERO_SETTLE_S,
    TouchSensor,
    ensure_deviation_mode,
    find_pico_port,
    parse_line,
)


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
    """Open the Pico, zero the sensors, fix the output mode, watch."""
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
        ensure_deviation_mode(sensor)
        watch(sensor, args.watch)
    if safety.is_stopped():
        # Phase "open" never latches STOP; only the user can clear it.
        print("safety STOP is latched; only the user can clear it.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
