"""Session control loop: hold the box pose between parts.

SessionLoop is a PickLoop that parks in "hold_wait" after the
release hold (same _hold mechanism and tracking guard as every
other hold) instead of walking "return". The main thread
installs the next part plan with load_next (False when the
loop already left the hold); request_end walks the existing
return segment from any phase. Both follow the same
main-thread flag pattern as PickLoop.abort. pick_loop.py
itself is unchanged. A missed grip or contacts still held at
the release never reach the hold: they walk return as today.
"""

from __future__ import annotations

import queue
import threading

from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ
from muse_qc.pick.pick_loop import INSPECT_SETTLE_S, PickLoop
from muse_qc.pick.session_protocol import (
    PART_DONE_LINE,
    SessionClock,
    _EOF,
)


class SessionLoop(PickLoop):
    """PickLoop holding the box exit pose between session parts."""

    def __init__(self, *args, hold_wait_s, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = threading.Lock()
        self.hold_wait_cycles = int(float(hold_wait_s) * LOOP_RATE_HZ)
        self.wait_left = 0
        self.part_done = threading.Event()
        self.end_requested = False
        self.end_reason: str | None = None
        self.end_note: str | None = None
        self._pending: dict | None = None

    def _refuse(self, reason, value, limit):
        if self.end_note is None:
            self.end_note = f"{reason} ({value} vs {limit})"
        return super()._refuse(reason, value, limit)

    def _fault(self, message):
        if self.end_note is None:
            self.end_note = str(message)
        return super()._fault(message)

    def _start_abort(self):
        if self.end_note is None and not self.confirmed \
                and self.phase in ("open", "miss_open"):
            self.end_note = "missed grip"
        super()._start_abort()

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
        self.k, self.end = self.ranges["hover"]
        self.phase = "hover"

    def __call__(self, sdk, dt):
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
