"""Shared control-loop streaming helpers for the gripper tools.

GripperStream is the hold-the-arm loop moved over from
muse_qc.gripper_motor; GripLoop is the touch-feedback grip loop of
muse_qc.gripper_grip. Nothing here enables hardware on its own.
"""

from __future__ import annotations

import math
import threading
import time

from muse_qc import safety
from muse_qc.first_move_plan import (
    LOOP_RATE_HZ, SPEED_FRACTION, STEP_DEG, clamp_to_limits, real_tip_m)
from muse_qc.gripper import (
    EXPECTED_FIRMWARE_VERSION, FAST_DRAIN_S, PICO_USB_VID,
    VERSION_WAIT_S, ZERO_SETTLE_S, TouchSensor,
    ensure_deviation_mode, find_pico_port)

WAIT_CYCLES = int(3.0 * LOOP_RATE_HZ)  # "place the part" window
HOLD_CYCLES = int(3.0 * LOOP_RATE_HZ)  # held-grip window
SETTLE_CYCLES = int(1.0 * LOOP_RATE_HZ)  # rest at the open width
FAULT_CYCLES = int(0.5 * LOOP_RATE_HZ)  # hold after a reading fault
JUMP_DEG = 20.0  # mid-run reading jump that aborts the grip
SENSOR_TIMEOUT_S = 0.002  # touch read budget per control cycle


class GripperStream:
    """Control-loop callback: hold the arm start pose, stream one plan."""

    def __init__(self, arm_group: object, gripper_group: object,
                 arm_start_rad: list[float], arm_vlim: list[float],
                 start_rad: float, waypoints_rad: list[float],
                 gripper_vlim: list[float], snapshot_index: int | None) -> None:
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.arm_start_rad = [float(v) for v in arm_start_rad]
        self.arm_vlim = [float(v) for v in arm_vlim]
        self.last_rad = float(start_rad)
        self.waypoints = [float(v) for v in waypoints_rad]
        self.gripper_vlim = [float(v) for v in gripper_vlim]
        self.snapshot_index = snapshot_index
        self.snapshot_deg: float | None = None
        self.index = 0
        self.halt = False
        self.done = threading.Event()
    def _send(self, grip_rad: float) -> None:
        self.gripper_group.send_pos_vel([grip_rad], vlim=self.gripper_vlim)
        self.arm_group.send_pos_vel(list(self.arm_start_rad), vlim=self.arm_vlim)
    def _hold(self) -> None:
        self._send(self.last_rad)
        self.done.set()
    def __call__(self, _sdk: object, _dt: float) -> None:
        if self.halt or self.index >= len(self.waypoints):
            self._hold()
            return
        if self.index == self.snapshot_index:
            try:
                positions = self.gripper_group.get_positions()
                self.snapshot_deg = math.degrees(float(positions[0]))
            except Exception:
                pass
        commanded = self.waypoints[self.index]
        self._send(commanded)
        self.last_rad = commanded
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done.set()


class GripLoop:
    """100 Hz callback: hold the arm, open/wait/close/hold/reopen the gripper.

    Open/wait contacts are only printed. Bilateral close contact is
    confirmed once via safety.check_gripper("closed", ...) (every cycle
    would burn the missed-grip retries and latch STOP). Reading faults
    hold the last commanded position for 0.5 s, then done.
    """

    def __init__(self, arm_group: object, gripper_group: object,
                 arm_start_rad: list[float], arm_vlim: list[float],
                 gripper_vlim: list[float], start_deg: float, open_deg: float,
                 squeeze_deg: float, closed_deg: float, safe_min_deg: float,
                 safe_max_deg: float, jump_deg: float, sensor: object,
                 to_width: object, out: object = print,
                 step_deg: float = STEP_DEG) -> None:
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.arm_start_rad = [float(v) for v in arm_start_rad]
        self.arm_vlim = [float(v) for v in arm_vlim]
        self.gripper_vlim = [float(v) for v in gripper_vlim]
        self.open_deg = float(open_deg)
        self.squeeze_deg = float(squeeze_deg)
        self.closed_deg = float(closed_deg)
        self.safe_min = float(safe_min_deg)
        self.safe_max = float(safe_max_deg)
        self.jump_deg = float(jump_deg)
        self.commanded = float(start_deg)
        self.measured = float(start_deg)
        self._sensor = sensor
        self._to_width = to_width
        self._out = out
        self.grip_step = float(step_deg)
        self.contacts: frozenset[str] = frozenset()
        self._noted: frozenset[str] = frozenset()
        self.phase = "open"
        self.left = 0
        self.squeeze_target: float | None = None
        self.missed = False
        self.fault: str | None = None
        self.done = threading.Event()
    def abort(self) -> None:
        """Stop closing and open back to the open width if still running."""
        if not self.done.is_set():
            self.phase = "reopen"
    def _send(self, grip_deg: float) -> None:
        self.gripper_group.send_pos_vel([math.radians(grip_deg)], vlim=self.gripper_vlim)
        self.arm_group.send_pos_vel(list(self.arm_start_rad), vlim=self.arm_vlim)
        self.commanded = grip_deg
    def _step(self, target: float) -> bool:
        """Move one gripper step toward target; True when reached."""
        diff = target - self.commanded
        if abs(diff) <= self.grip_step:
            self._send(target)
            return True
        self._send(self.commanded + math.copysign(self.grip_step, diff))
        return False
    def _poll(self) -> None:
        """Refresh contacts from the newest sensor reading, if any."""
        try:
            reading = self._sensor.read(SENSOR_TIMEOUT_S)
        except Exception:
            return
        if reading is not None:
            self.contacts = frozenset(reading.contacts)
    def _enter_fault(self, message: str) -> None:
        if self.fault is None:
            self.fault = message
            self._out(message)
        self.phase = "fault"
        self.left = FAULT_CYCLES

    def _check_reading(self) -> bool:
        """Update the measured reading; False when a fault was entered."""
        try:
            value = math.degrees(float(self.gripper_group.get_positions()[0]))
        except Exception as exc:
            self._enter_fault(f"gripper reading failed ({exc}), holding position")
            return False
        if not (math.isfinite(value) and self.safe_min <= value <= self.safe_max):
            self._enter_fault(
                f"gripper reading {value:+.2f} deg left the safe range, holding position")
            return False
        if abs(value - self.measured) > self.jump_deg:
            self._enter_fault(
                f"gripper reading jumped {self.measured:+.2f} to {value:+.2f} deg,"
                " holding position")
            return False
        self.measured = value
        return True

    def _held_now(self) -> bool:
        """Both fingers in contact, confirmed once via safety. Pure query."""
        if not (self.contacts & safety.GRIP_LEFT and self.contacts & safety.GRIP_RIGHT):
            return False
        return safety.check_gripper("closed", self.contacts).get("grip") == "held"

    def __call__(self, _sdk: object, _dt: float) -> None:
        if self.done.is_set() or self.phase == "fault":
            self._send(self.commanded)
            if self.phase == "fault":
                self.left -= 1
                if self.left <= 0:
                    self.done.set()
            return
        if not self._check_reading():
            self._send(self.commanded)
            return
        self._poll()
        if self.phase in ("open", "wait") and self.contacts != self._noted:
            self._noted = set(self.contacts)
            self._out(f"contacts {' '.join(sorted(self.contacts))} (placing phase, ignored)")
        if self.phase == "open":
            if self._step(self.open_deg):
                self.phase = "wait"
                self.left = WAIT_CYCLES
                self._out("place the part between the jaws now")
        elif self.phase == "wait":
            self._send(self.open_deg)
            self.left -= 1
            if self.left <= 0:
                self.phase = "close"
        elif self.phase == "close":
            if self._held_now():
                self._out(f"contact: gripper commanded deg {self.commanded:+.2f}"
                          f" measured deg {self.measured:+.2f}"
                          f" contacts {' '.join(sorted(self.contacts))}")
                self.squeeze_target = min(self.commanded + self.squeeze_deg, self.closed_deg)
                self.phase = "squeeze"
            elif self._step(self.closed_deg):
                self.missed = True
                self._out("missed grip")
                self.phase = "reopen"
        elif self.phase == "squeeze":
            assert self.squeeze_target is not None
            if self._step(self.squeeze_target):
                self.phase = "hold"
                self.left = HOLD_CYCLES
        elif self.phase == "hold":
            assert self.squeeze_target is not None
            self._send(self.squeeze_target)
            self.left -= 1
            if self.left <= 0:
                self._out(f"grip held at deg {self.commanded:+.2f}"
                          f" width about {self._to_width(self.commanded):.1f} mm"
                          f" contacts {' '.join(sorted(self.contacts))}")
                self._out(f"hold end: gripper commanded deg {self.commanded:+.2f}"
                          f" measured deg {self.measured:+.2f}")
                self.phase = "reopen"
        elif self.phase == "reopen":
            if self._step(self.open_deg):
                self.phase = "settle"
                self.left = SETTLE_CYCLES
        elif self.phase == "settle":
            self._send(self.open_deg)
            self.left -= 1
            if self.left <= 0:
                self.done.set()


def check_arm_hold(arm_start_deg: list[float], joint_limits: object,
                   check_fn: object = None, tip_fn: object = None,
                   out: object = print) -> list[float] | None:
    """Clamp the rest reading into its URDF limits and check it once.

    Returns the hold pose, or None after printing the refusal.
    """
    limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
    if limits is None:
        out("Joint limits unavailable, not moving")
        return None
    hold_deg = list(clamp_to_limits(arm_start_deg, limits))
    out(f"arm hold deg: {[f'{v:+.2f}' for v in hold_deg]}")
    check = safety.check_target if check_fn is None else check_fn
    tip_of = real_tip_m if tip_fn is None else tip_fn
    hold_tip, rest_tip = tip_of(hold_deg), tip_of(list(arm_start_deg))
    target = safety.Target(x=hold_tip[0], y=hold_tip[1], z=hold_tip[2],
                           joints_deg=tuple(hold_deg), speed=SPEED_FRACTION)
    current = safety.CurrentState(x=rest_tip[0], y=rest_tip[1], z=rest_tip[2],
                                  joints_deg=tuple(arm_start_deg),
                                  reading_age_ms=0.0)
    held = check(target, current)
    if not held.get("ok"):
        out(f"arm hold refused: {held.get('reason')}, not moving")
        return None
    out("arm hold: ok")
    return hold_deg


def open_touch_sensor(find_port: object = None, sensor_factory: object = None,
                      out: object = print) -> object | None:
    """Open the Pico, check firmware, zero, fast mode, deviation mode."""
    find = find_pico_port if find_port is None else find_port
    port = find()
    if port is None:
        out(f"gripper Pico not found (USB VID {PICO_USB_VID}); ports match by USB"
            " ID, never by fixed number.")
        return None
    out(f"pico: {port}")
    make = TouchSensor if sensor_factory is None else sensor_factory
    sensor = make()
    try:
        sensor.open(port)
    except Exception as exc:
        out(f"cannot open {port}: {exc}")
        return None
    sensor.send("v")
    version = None
    deadline = time.monotonic() + VERSION_WAIT_S
    while time.monotonic() < deadline:
        parts = (sensor.read_line(max(0.0, deadline - time.monotonic())) or "").split()
        if parts[:2] == ["#", "firmware"] and len(parts) == 3 and parts[2].isdigit():
            version = int(parts[2])
            break
    if version is None:
        out("firmware: no reply")
    else:
        out(f"firmware: {version}")
        if version != EXPECTED_FIRMWARE_VERSION:
            out(f"warning: expected firmware {EXPECTED_FIRMWARE_VERSION}; continuing")
    out("do not touch the fingers")
    time.sleep(ZERO_SETTLE_S)
    sensor.send("n")
    sensor.send("f")
    sensor.read(FAST_DRAIN_S)  # drop stale pre-zero lines
    ensure_deviation_mode(sensor, out)
    return sensor
