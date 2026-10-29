"""Pick control loop for the first pick. Human-run only, no hardware opened here.

PickLoop is the 100 Hz callback used by muse_qc.pick_test: it walks
the arm waypoints of muse_qc.pick_plan with a fresh safety check per
cycle, opens the gripper during the hover walk (holding at hover
until fully open), commands the gripper every cycle, watches the
touch sensor and climbs back to the hover height before returning
after any fault, contact stop or abort. On a drop run an abort
during to-box walks the remaining carry waypoints to the drop pose
and then returns, during release it returns straight away; the
gripper is never opened on an abort. Protocol use only, no
gripper-project code.
"""
from __future__ import annotations
import math
import threading
import time
from muse_qc import safety
from muse_qc.drop_plan import DROP_HOLD_CYCLES
from muse_qc.first_move_plan import LOOP_RATE_HZ, SPEED_FRACTION, real_tip_m
from muse_qc.gripper_grip import CLOSED_DEG, SAFE_MAX_DEG, SAFE_MIN_DEG
from muse_qc.gripper_stream import JUMP_DEG, SENSOR_TIMEOUT_S
from muse_qc.pick_plan import abort_detour
GRIP_STEP_DEG = 15.0 / LOOP_RATE_HZ
SQUEEZE_DEG = 2.0
FAULT_HOLD_CYCLES = int(0.5 * LOOP_RATE_HZ)
DROP_WALKS = ("to-box",)
_WALKS = ("hover", "lower", "lift", "put-down", "raise", "return") + DROP_WALKS
INSPECT_SETTLE_S = 1.0
INSPECT_TIMEOUT_S = 30.0
class PickLoop:
    """100 Hz pick callback: walk the plan with touch-aware phases."""
    def __init__(self, arm_group, gripper_group, arm_vlim, gripper_vlim,
                 arm_start_rad, plan, tips, ranges, hover_z, open_deg,
                 start_grip_deg, hold_cycles, no_grip, sensor, out=print,
                 check_fn=None, tip_fn=None, drop_box=None,
                 release_open_deg=None, inspect=False,
                 ranges_by_box=None, shared_ranges=None):
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.arm_vlim = [float(v) for v in arm_vlim]
        self.gripper_vlim = [float(v) for v in gripper_vlim]
        self.plan, self.tips, self.ranges = plan, tips, ranges
        self.hover_z, self.open_deg = float(hover_z), float(open_deg)
        self.grip_cmd = self.measured = float(start_grip_deg)
        self.cmd_arm_rad = [float(v) for v in arm_start_rad]
        self.hold_cycles, self.no_grip = int(hold_cycles), bool(no_grip)
        self.drop_box = drop_box
        self.release_open_deg = (None if release_open_deg is None
                                 else float(release_open_deg))
        self._sensor, self._out = sensor, out
        self.check = safety.check_target if check_fn is None else check_fn
        self.tip_of = real_tip_m if tip_fn is None else tip_fn
        self.contacts: frozenset[str] = frozenset()
        self._prev: frozenset[str] = frozenset()
        self.phase, self.k, self.end = "hover", *ranges["hover"]
        self.left, self.di, self.pos = 0, 0, None
        self.detour: list[int] = []
        self.squeeze_target: float | None = None
        self.confirmed = self.lost = self.abort_requested = False
        self.exit_code = 0
        self.done = threading.Event()
        self.inspect = bool(inspect)
        self.ranges_by_box, self.shared_ranges = ranges_by_box, shared_ranges
        self.inspect_ready = threading.Event()
        self._verdict, self._inspected = "pending", False
        self._settle_left = int(INSPECT_SETTLE_S * LOOP_RATE_HZ)
    def abort(self):
        if not self.done.is_set():
            self.abort_requested = True
    def set_verdict(self, box):
        """Take the main-thread inspection verdict ("good", "bad", None)."""
        if box not in ("good", "bad", None):
            raise ValueError(f"bad verdict {box!r}")
        self._verdict = box
    def _send(self, arm_rad, grip_deg):
        self.arm_group.send_pos_vel([float(v) for v in arm_rad], vlim=self.arm_vlim)
        self.gripper_group.send_pos_vel([math.radians(grip_deg)], vlim=self.gripper_vlim)
        self.cmd_arm_rad = [float(v) for v in arm_rad]
        self.grip_cmd = float(grip_deg)
    def _refuse(self, reason, value, limit):
        if self.exit_code == 0:
            self._out(f"REFUSED {reason} ({value} vs {limit}), holding position")
        self.exit_code = 1
        if self.phase != "abort":
            self._send(self.cmd_arm_rad, self.grip_cmd)
            self._start_abort()
        else:
            self._send(self.cmd_arm_rad, self.grip_cmd)
            self.phase = "finish"
            self.done.set()
        return False
    def _fault(self, message):
        if self.exit_code == 0:
            self._out(message)
        self.exit_code = 1
        self.phase = "fault_hold"
        self.left = FAULT_HOLD_CYCLES
        return False
    def _check_gripper(self):
        try:
            value = math.degrees(float(self.gripper_group.get_positions()[0]))
        except Exception as exc:
            return self._fault(f"gripper reading failed ({exc}), holding position")
        if not (math.isfinite(value) and SAFE_MIN_DEG <= value <= SAFE_MAX_DEG):
            return self._fault(f"gripper reading {value:+.2f} deg left the safe range")
        if abs(value - self.measured) > JUMP_DEG:
            return self._fault(f"gripper reading jumped {self.measured:+.2f}"
                               f" to {value:+.2f} deg, holding position")
        self.measured = value
        return True
    def _poll(self):
        try:
            reading = self._sensor.read(SENSOR_TIMEOUT_S)
            if reading is not None:
                self.contacts = frozenset(reading.contacts)
        except Exception:
            pass
    def _walk_send(self, index):
        read_start = time.monotonic()
        try:
            current_rad = [float(v) for v in self.arm_group.get_positions()]
            if len(current_rad) != 6:
                raise ValueError(f"got {len(current_rad)} positions")
        except Exception as exc:
            return self._refuse("read-failed", str(exc), "six joint angles")
        age_ms = (time.monotonic() - read_start) * 1000.0
        current_deg = [math.degrees(v) for v in current_rad]
        _, waypoint, grasp = self.plan[index]
        try:
            current_tip = self.tip_of(current_deg)
        except Exception as exc:
            return self._refuse("read-failed", str(exc), "tool tip")
        tip = self.tips[index]
        target = safety.Target(x=tip[0], y=tip[1], z=tip[2], joints_deg=tuple(waypoint),
                               speed=SPEED_FRACTION)
        current = safety.CurrentState(x=current_tip[0], y=current_tip[1], z=current_tip[2],
                                      joints_deg=tuple(current_deg), reading_age_ms=age_ms)
        extra = {"z_min_m": safety.GRASP_Z_MIN_M} if grasp else {}
        result = self.check(target, current, **extra)
        if not result.get("ok"):
            return self._refuse(result.get("reason"), result.get("value"), result.get("limit"))
        self._send([math.radians(v) for v in waypoint], self.grip_cmd)
        self.pos = index
        return True
    def _hold(self):
        assert self.pos is not None
        return self._walk_send(self.pos)
    def _step_grip(self, target_deg):
        diff = target_deg - self.grip_cmd
        if abs(diff) <= GRIP_STEP_DEG:
            self._send(self.cmd_arm_rad, target_deg)
            return True
        self._send(self.cmd_arm_rad, self.grip_cmd + math.copysign(GRIP_STEP_DEG, diff))
        return False
    def _start_abort(self):
        if self.drop_box is not None and self._in_drop_segment("to-box"):
            first, end = self.ranges["to-box"]
            low, high = self.ranges["return"]
            at = first if self.pos is None else max(first, self.pos)
            self.detour = list(range(at, end)) + list(range(low, high))
        elif self.drop_box is not None and self._in_drop_segment("release"):
            low, high = self.ranges["return"]
            hold = [] if self.pos is None else [self.pos]
            self.detour = hold + list(range(low, high))
        else:
            self.detour = abort_detour(self.plan, self.tips, self.ranges,
                                       self.hover_z, self.pos)
        self.di = 0
        self.abort_requested = False
        self.phase = "abort"

    def _in_drop_segment(self, segment: str) -> bool:
        """Whether the loop is inside one drop segment right now."""
        if segment == "to-box":
            if self.phase == "to-box":
                return True
        elif self.phase in ("release", "release_hold"):
            return True
        return self.pos is not None and self.plan[self.pos][0] == segment
    def _contact_stop(self):
        assert self.pos is not None
        shown = " ".join(sorted(self.contacts)) or "-"
        self._out(f"contact while lowering: {shown}"
                  f" at tip z about {self.tips[self.pos][2] * 1000.0:.1f} mm")
        self.exit_code = 1
        self._send(self.cmd_arm_rad, self.grip_cmd)
        self.phase = "fault_hold"
        self.left = FAULT_HOLD_CYCLES
    def _ramp(self, target_deg, after):
        if not self._hold():
            return
        if self._step_grip(target_deg):
            after()
    def _hold_then(self, after):
        if not self._hold():
            return
        self.left -= 1
        if self.left <= 0:
            after()
    def __call__(self, _sdk, _dt):
        if self.done.is_set():
            self._send(self.cmd_arm_rad, self.grip_cmd)
            return
        if self.abort_requested and self.phase != "abort":
            self.exit_code = 1
            self._start_abort()
        if self.phase == "fault_hold":
            self._send(self.cmd_arm_rad, self.grip_cmd)
            self.left -= 1
            if self.left <= 0:
                self._start_abort()
            return
        if self.phase != "abort" and not self._check_gripper():
            self._send(self.cmd_arm_rad, self.grip_cmd)
            return
        self._poll()
        if self.phase in _WALKS:
            self._do_walk()
        elif self.phase == "close":
            self._do_close()
        elif self.phase == "squeeze":
            assert self.squeeze_target is not None
            self._ramp(self.squeeze_target, self._to_squeeze_hold)
        elif self.phase == "inspect_hold":
            self._do_inspect_hold()
        elif self.phase in ("open", "miss_open", "reopen", "drop_open"):
            self._ramp(self.open_deg, {"open": self._to_after_hover,
                                       "miss_open": self._start_abort,
                                       "reopen": self._to_raise,
                                       "drop_open": self._to_return}[self.phase])
        elif self.phase == "release":
            self._do_release()
        elif self.phase in ("bottom_hold", "squeeze_hold", "top_hold",
                            "release_hold"):
            self._hold_then({"bottom_hold": self._to_raise_detour,
                             "squeeze_hold": self._to_lift,
                             "top_hold": self._to_after_lift,
                             "release_hold": self._to_released}[self.phase])
        elif self.phase == "abort":
            if self.di >= len(self.detour):
                self.phase = "finish"
                self.done.set()
            elif self._walk_send(self.detour[self.di]):
                self.di += 1
    def _walk_seg(self, segment):
        self.k, self.end = self.ranges[segment]
        self.phase = segment
    def _to_after_hover(self):
        if self.inspect and not self._inspected and self.ranges_by_box:
            self.phase = "inspect_hold"
        else:
            self._to_lower()
    def _do_inspect_hold(self):
        if not self._hold():
            return
        if self._settle_left > 0:
            self._settle_left -= 1
            if self._settle_left > 0:
                return
            self.inspect_ready.set()
            self.left = int(INSPECT_TIMEOUT_S * LOOP_RATE_HZ)
            return
        if self._verdict == "pending":
            self.left -= 1
            if self.left <= 0:
                self._out("inspect timeout")
                self.exit_code = 1
                self._start_abort()
            return
        box = self._verdict
        self._inspected = True
        if box is None:
            self.exit_code = 1
            self._start_abort()
            return
        self.drop_box = box
        self.ranges = {**self.shared_ranges, **self.ranges_by_box[box]}
        self._to_lower()
    def _to_lower(self): self._walk_seg("lower")
    def _to_lift(self): self._walk_seg("lift")
    def _to_raise(self): self._walk_seg("raise")
    def _to_return(self): self._walk_seg("return")
    def _to_after_lift(self):
        if self.drop_box is None:
            self._walk_seg("put-down")
        else:
            self._walk_seg("to-box")
    def _to_putdown(self): self._walk_seg("put-down")
    def _to_released(self):
        if self.contacts:
            self._out("part still held after release")
        else:
            self._out(f"dropped into the {self.drop_box} box")
        self._walk_seg("return")
    def _do_release(self):
        assert self.release_open_deg is not None
        if not self._hold():
            return
        if self._step_grip(self.release_open_deg):
            self.left = DROP_HOLD_CYCLES
            self.phase = "release_hold"
    def _to_squeeze_hold(self):
        self.left = FAULT_HOLD_CYCLES
        self.phase = "squeeze_hold"
    def _to_raise_detour(self):
        first, last = self.ranges["return"]
        if "raise" in self.ranges:
            low, high = self.ranges["raise"]
            self.detour = list(range(low, high)) + list(range(first, last))
        else:
            self.detour = list(range(first, last))
        self.di = 0
        self.phase = "abort"
    def _grip_at_open(self):
        return abs(self.grip_cmd - self.open_deg) <= 1e-9

    def _nudge_grip_open(self):
        diff = self.open_deg - self.grip_cmd
        if abs(diff) <= GRIP_STEP_DEG:
            self.grip_cmd = self.open_deg
        else:
            self.grip_cmd = self.grip_cmd + math.copysign(GRIP_STEP_DEG, diff)

    def _do_walk(self):
        if self.phase == "lower" and self.contacts:
            self._contact_stop()
            return
        if self.phase == "hover":
            self._nudge_grip_open()
        if not self._walk_send(self.k):
            return
        self.k += 1
        finished = self.k >= self.end
        if self.phase == "lift":
            if self.contacts != self._prev:
                self._prev = frozenset(self.contacts)
                self._out(f"contacts {' '.join(sorted(self.contacts)) or '-'} (lifting)")
            if not self.lost and (not (self.contacts & safety.GRIP_LEFT)
                                  or not (self.contacts & safety.GRIP_RIGHT)):
                self.lost = True
                self._out("part lost while lifting")
        if self.phase == "to-box":
            if self.contacts != self._prev:
                self._prev = frozenset(self.contacts)
                self._out(f"contacts {' '.join(sorted(self.contacts)) or '-'} (carrying)")
            if not self.lost and (not (self.contacts & safety.GRIP_LEFT)
                                  or not (self.contacts & safety.GRIP_RIGHT)):
                self.lost = True
                self._out("part lost while carrying")
        if not finished or self.phase not in _WALKS:
            return
        if self.phase == "hover":
            if self._grip_at_open():
                self._to_after_hover()
            else:
                self.phase = "open"  # hold hover until fully open
        elif self.phase == "lower":
            if self.no_grip:
                self.left = self.hold_cycles
                self._out(f"holding at tip z {self.tips[self.pos][2] * 1000.0:.1f} mm,"
                          " measure the tip height now")
                self.phase = "bottom_hold"
            else:
                self.phase = "close"
        elif self.phase == "lift":
            self.left = self.hold_cycles
            self._out("no part held, holding" if self.lost else "part lifted, holding")
            self.phase = "top_hold"
        elif self.phase == "to-box":
            if self.lost:
                self._out("no part held, holding")
                self.phase = "drop_open"
            else:
                self.phase = "release"
        elif self.phase == "put-down":
            self.phase = "reopen"
        elif self.phase == "raise":
            self._walk_seg("return")
        else:
            self.phase = "finish"
            self.done.set()
    def _do_close(self):
        if not self._hold():
            return
        both = self.contacts & safety.GRIP_LEFT and self.contacts & safety.GRIP_RIGHT
        if both and not self.confirmed:
            self.confirmed = True
            held = safety.check_gripper("closed", self.contacts).get("grip") == "held"
            if held:
                self._out(f"contact: gripper commanded deg {self.grip_cmd:+.2f}"
                          f" measured deg {self.measured:+.2f}"
                          f" contacts {' '.join(sorted(self.contacts))}")
                self.squeeze_target = min(self.grip_cmd + SQUEEZE_DEG, CLOSED_DEG)
                self.phase = "squeeze"
                return
        if self._step_grip(CLOSED_DEG):
            self._out("missed grip")
            self.phase = "miss_open"
