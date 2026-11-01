"""Session control loop: hold the box pose between parts.

SessionLoop is a PickLoop that parks in "hold_wait" after the
release hold (same _hold mechanism and tracking guard as every
other hold) instead of walking "return". The main thread
installs the next part plan with load_next (False when the
loop already left the hold); request_end from the hold walks
the existing return segment. Both follow the same
main-thread flag pattern as PickLoop.abort. pick_loop.py
itself is unchanged. Any other abnormal end (missed grip,
refusal, tracking error, timeout) never walks a planned index:
after the gripper opens on a missed grip the loop ramps home
to the session rest from the joints actually commanded at that
moment, settles and ends.
"""

from __future__ import annotations

import math
import queue
import threading
import time

from muse_qc.arm import safety
from muse_qc.arm.target_pose import base_to_table_mm
from muse_qc.bringup.first_move_plan import (
    LOOP_RATE_HZ,
    SETTLE_HOLD_CYCLES,
    SPEED_FRACTION,
    ramp_waypoints,
)
from muse_qc.pick import pick_mit, pick_plan
from muse_qc.pick.pick_loop import INSPECT_SETTLE_S, PickLoop
from muse_qc.pick.session_protocol import (
    PART_DONE_LINE,
    SessionClock,
    _EOF,
)

RETURN_HOME_PHASE = "return_home"
RETURN_REFUSED_LINE = ("return refused, hold the arm and switch off "
                       "the power supply")


def build_home_return(cmd_deg, rest_deg, speed_deg_s, accel_deg_s2,
                      settle_cycles, tip_fn, solve_fn, hover_z_m):
    """Ramp home with a vertical lift first when starting low. Pure.

    When the commanded tip sits below the current part hover height,
    an ascent is solved fresh from the commanded joints (the same 1
    mm rung ladder and rung speeds as the pick lift, seeded with the
    commanded pose, so joint 1 and joint 6 stay put), then the joint
    ramp to the session rest plus settle follows. Every step still
    passes the per-cycle safety check live. Returns None when the
    ascent cannot be built (no solver or an unreachable rung): the
    caller must hold, never joint-ramp from low.
    """
    cmd = tuple(float(v) for v in cmd_deg)
    rest = tuple(float(v) for v in rest_deg)
    tip = tip_fn(list(cmd))
    top = cmd
    points: list = []
    if tip[2] < float(hover_z_m) - 1e-9:
        if solve_fn is None:
            return None
        table = base_to_table_mm(tip[0] * 1000.0, tip[1] * 1000.0)
        low_mm = tip[2] * 1000.0
        rungs = pick_plan._heights(float(hover_z_m) * 1000.0, low_mm)
        up = rungs[::-1]
        ladder = pick_plan._solve_ladder(table, up[1:], cmd, solve_fn)
        if not ladder.get("ok"):
            return None
        joints = [cmd] + [tuple(float(v) for v in rung)
                          for rung in ladder["joints"]]
        heights = [float(height) for height in up]
        for low, high, bottom_h, top_h in zip(joints, joints[1:],
                                              heights, heights[1:]):
            fast = bottom_h >= low_mm + pick_plan.LIFT_SLOW_MM
            steps = pick_plan._rung_steps(low, high, top_h - bottom_h,
                                          fast)
            points += pick_plan._interpolate(low, high, steps)
        points[-1] = joints[-1]
        top = joints[-1]
    ramp = list(ramp_waypoints(top, rest, speed_deg_s=speed_deg_s,
                               accel_deg_s2=accel_deg_s2))
    ramp[-1] = rest
    return points + ramp + [rest] * int(settle_cycles)


class SessionLoop(PickLoop):
    """PickLoop holding the box exit pose between session parts."""

    def __init__(self, *args, hold_wait_s, rest_deg, home_speed_deg_s,
                 home_accel_deg_s2, home_solve_fn=None, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = threading.Lock()
        self.hold_wait_cycles = int(float(hold_wait_s) * LOOP_RATE_HZ)
        self.wait_left = 0
        self.part_done = threading.Event()
        self.end_requested = False
        self.end_reason: str | None = None
        self.end_note: str | None = None
        self._pending: dict | None = None
        self._rest_deg = tuple(float(v) for v in rest_deg)
        self._home_speed = float(home_speed_deg_s)
        self._home_accel = float(home_accel_deg_s2)
        self._home_solve_fn = home_solve_fn
        self._home_points: list = []
        self._home_i = 0

    def _refuse(self, reason, value, limit):
        if self.end_note is None:
            self.end_note = f"{reason} ({value} vs {limit})"
        return super()._refuse(reason, value, limit)

    def _fault(self, message):
        if self.end_note is None:
            self.end_note = str(message)
        return super()._fault(message)

    def _start_abort(self):
        """Lift straight up, then ramp home; never a planned index.

        Keeps the missed-grip end note, then walks build_home_return
        from the joints actually commanded at that moment (open
        gripper after miss_open, untouched gripper otherwise): a
        fresh vertical ascent to the part hover height when starting
        low, then the joint ramp to rest. The planned detour is
        skipped on purpose: past the lift its return starts at the
        box release, which the per-cycle step check refuses from a
        table pose.
        """
        if self.end_note is None and not self.confirmed \
                and self.phase in ("open", "miss_open"):
            self.end_note = "missed grip"
        self._abort_from = None
        cmd_deg = [math.degrees(v) for v in self.cmd_arm_rad]
        try:
            self._home_points = build_home_return(
                cmd_deg, self._rest_deg, self._home_speed,
                self._home_accel, SETTLE_HOLD_CYCLES,
                self.tip_of, self._home_solve_fn, self.hover_z)
        except (ValueError, TypeError, AttributeError):
            self._home_hold()
            return
        if not self._home_points:
            self._home_points = []
            self._home_hold()
            return
        self._home_i = 0
        self.detour = []
        self.di = 0
        self.abort_requested = False
        self.phase = RETURN_HOME_PHASE

    def _home_hold(self):
        """Hold the commanded pose and end: the home ramp is refused."""
        self._out(RETURN_REFUSED_LINE)
        self.exit_code = 1
        self._send(self.cmd_arm_rad, self.grip_cmd)
        self.phase = "finish"
        self.done.set()
        return False

    def _walk_home_step(self, waypoint_deg):
        """Command one fresh home waypoint with a live safety check."""
        read_start = time.monotonic()
        try:
            current_rad = [float(v) for v in self.arm_group.get_positions()]
            if len(current_rad) != 6:
                raise ValueError(
                    f"got {len(current_rad)} positions")
        except Exception:  # noqa: BLE001 - unverified ramp must hold
            return self._home_hold()
        age_ms = (time.monotonic() - read_start) * 1000.0
        current_deg = [math.degrees(v) for v in current_rad]
        if pick_mit.feed_guard(self, current_deg):
            return False
        try:
            current_tip = self.tip_of(current_deg)
        except Exception:  # noqa: BLE001 - unverified ramp must hold
            return self._home_hold()
        tip = self.tip_of([float(v) for v in waypoint_deg])
        # The ramp starts at the low grip pose, so it uses the pick
        # grasp floor (not the 30 mm transit floor) for every step.
        target = safety.Target(x=tip[0], y=tip[1], z=tip[2],
                               joints_deg=tuple(float(v)
                                                for v in waypoint_deg),
                               speed=SPEED_FRACTION)
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=tuple(current_deg), reading_age_ms=age_ms)
        if not self.check(target, current,
                          z_min_m=safety.GRASP_Z_MIN_M).get("ok"):
            return self._home_hold()
        self._send([math.radians(v) for v in waypoint_deg], self.grip_cmd)
        return True

    def _do_return_home(self):
        """Walk one home-return cycle; the flags only end what is ending."""
        self.abort_requested = False
        self.end_requested = False
        if self._home_i >= len(self._home_points):
            self.phase = "finish"
            self.done.set()
            return
        if self._walk_home_step(self._home_points[self._home_i]):
            self._home_i += 1

    def _to_released(self):
        if self.contacts:
            if self.end_note is None:
                self.end_note = "part still held after release"
            super()._to_released()
            return
        self._out(f"dropped into the {self.drop_box} box")
        self.part_done.set()
        self._out(PART_DONE_LINE)
        self.wait_left = int(self.hold_wait_cycles)
        self.phase = "hold_wait"

    def load_next(self, bundle: dict) -> bool:
        """Queue the next part plan; False when the hold already ended.

        Also drops the previous part's inspection and completion
        state atomically, so the main thread's wait_and_inspect
        and part wait right after can never see stale events,
        whenever the loop thread installs the plan.
        """
        with self._lock:
            if self.phase != "hold_wait" or self.done.is_set():
                return False
            self._pending = bundle
            self._verdict, self._inspected = "pending", False
            self.inspect_skipped = False
            self.inspect_ready.clear()
            self.part_done.clear()
            return True

    def request_end(self, reason: str) -> None:
        """End the session: return to rest from any phase."""
        with self._lock:
            self.end_reason = str(reason)
            self._pending = None
            if self.done.is_set():
                return
            if self.phase == "hold_wait":
                self.end_requested = True
            else:
                self.abort_requested = True

    def _install(self, bundle: dict) -> None:
        """Install a queued plan on the loop thread; reset per-part state."""
        self.plan, self.tips, self.ranges = \
            bundle["plan"], bundle["tips"], bundle["ranges"]
        self.hover_z, self.open_deg = bundle["hover_z"], bundle["open_deg"]
        self.release_open_deg = bundle["release_open_deg"]
        self.ranges_by_box, self.shared_ranges = \
            bundle["ranges_by_box"], bundle["shared_ranges"]
        self.hold_cycles, self.inspect = bundle["hold_cycles"], bundle["inspect"]
        if bundle.get("solve_fn") is not None:
            self._home_solve_fn = bundle["solve_fn"]
        self.drop_box = None
        self.confirmed = self.lost = self.abort_requested = False
        self._prev = frozenset()
        self.pos = None
        self.detour, self.di = [], 0
        self.squeeze_target = None
        self._verdict, self._inspected = "pending", False
        self.inspect_skipped = False
        self.inspect_ready.clear()
        self._settle_left = int(INSPECT_SETTLE_S * LOOP_RATE_HZ)
        self.exit_code = 0
        self.left = 0
        self.part_done.clear()
        self._home_points = []
        self._home_i = 0
        self.k, self.end = self.ranges["hover"]
        self.phase = "hover"

    def __call__(self, sdk, dt):
        if self.phase == RETURN_HOME_PHASE:
            if self.done.is_set():
                self._send(self.cmd_arm_rad, self.grip_cmd)
                return
            self._do_return_home()
            return
        if self.phase != "hold_wait":
            super().__call__(sdk, dt)
            return
        if self.done.is_set():
            self._send(self.cmd_arm_rad, self.grip_cmd)
            return
        if self.abort_requested or self.end_requested:
            self.abort_requested = False
            self.end_requested = False
            self._walk_seg("return")
            return
        if not self._check_gripper():
            self._send(self.cmd_arm_rad, self.grip_cmd)
            return
        self._poll()
        with self._lock:
            pending, self._pending = self._pending, None
        if pending is not None:
            try:
                self._install(pending)
            except Exception as exc:  # noqa: BLE001 - never fly half a plan
                if self.end_note is None:
                    self.end_note = str(exc)
                self._walk_seg("return")
            return
        if not self._hold():
            return
        self.wait_left -= 1
        if self.wait_left <= 0:
            if self.end_reason is None:
                self.end_reason = "hold timeout"
            self._walk_seg("return")


def _clock_or_real(clock):
    """Return the given clock or the real-time session clock."""
    return clock if clock is not None else SessionClock()


def wait_part(loop, timeout_s, clock=None) -> str:
    """Wait for the part hold or the session end: "part", "end" or "timeout"."""
    watch = _clock_or_real(clock)
    deadline = watch.now() + timeout_s
    while True:
        if loop.part_done.is_set():
            return "part"
        if loop.done.is_set():
            return "end"
        if watch.now() >= deadline:
            return "timeout"
        watch.sleep(0.01)


def wait_next_line(lines: queue.Queue, loop, hold_wait_s, clock=None):
    """Wait for the next stdin line while the arm holds the box pose.

    Returns ("line", text), ("eof", None), or ("gone", None)
    when the loop already left the hold (timeout, abort); the
    caller then joins the loop instead of loading a plan.
    """
    watch = _clock_or_real(clock)
    deadline = watch.now() + hold_wait_s + 2.0
    while True:
        try:
            item = lines.get_nowait()
        except queue.Empty:
            item = None
        if item is not None:
            return ("eof", None) if item is _EOF else ("line", item)
        if loop.done.is_set() or loop.phase != "hold_wait":
            return ("gone", None)
        if watch.now() >= deadline:
            return ("gone", None)
        watch.sleep(0.01)


def join_done(loop, timeout_s=180.0, clock=None) -> None:
    """Wait for the loop, bounded; the caller finally still disables."""
    watch = _clock_or_real(clock)
    deadline = watch.now() + timeout_s
    while not loop.done.is_set():
        if watch.now() >= deadline:
            return
        watch.sleep(0.01)
