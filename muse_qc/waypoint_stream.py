"""Waypoint stream for the reBot B601-DM arm control loop. Pure callback.

Streams planned joint waypoints with a fresh safety check in every
control cycle. On a refusal or halt it holds the last commanded
position, never a live reading.
"""

from __future__ import annotations

import math
import threading
import time

from muse_qc import safety
from muse_qc.first_move_plan import SETTLE_CYCLES, SPEED_FRACTION, real_tip_m


class WaypointStream:
    """Control-loop callback: stream waypoints with a per-cycle check."""

    def __init__(self, group, waypoints, target_tips, vlim, initial_rad,
                 check_fn, tip_fn, snapshot_index=None):
        self.group = group
        self.waypoints = waypoints
        self.target_tips = target_tips
        self.vlim = [float(v) for v in vlim]
        self.last_commanded_rad = [float(v) for v in initial_rad]
        self.check = safety.check_target if check_fn is None else check_fn
        self.tip_of = real_tip_m if tip_fn is None else tip_fn
        self.index = 0
        self.refusal: dict | None = None
        self.halt = False
        self.settle = 0
        self.done = threading.Event()
        self.work_ms: list[float] = []
        self.period_ms: list[float] = []
        self._prev_start: float | None = None
        self.snapshot_index = snapshot_index
        self.snapshot_deg: list[float] | None = None

    def on_sample(self, current_deg: list[float]) -> None:
        """Hook with every cycle's joint reading; no-op by default."""

    def _send(self, joint_rad: list[float]) -> None:
        try:
            self.group.send_pos_vel([float(v) for v in joint_rad],
                                    vlim=self.vlim)
        except Exception:
            pass

    def _hold(self) -> None:
        # Hold the last commanded position, never a live reading: the
        # SDK reports 0.0 for missing feedback, which must never be sent.
        if self.settle <= 0:
            self.settle = SETTLE_CYCLES
        self._send(self.last_commanded_rad)
        self.settle -= 1
        if self.settle <= 0:
            self.done.set()

    def _refuse(self, reason: str, value: object, limit: object) -> None:
        self.refusal = {"ok": False, "reason": reason,
                        "value": value, "limit": limit}
        self._hold()

    def __call__(self, _sdk: object, _dt: float) -> None:
        start = time.perf_counter()
        try:
            if self._prev_start is not None:
                self.period_ms.append((start - self._prev_start) * 1000.0)
        except Exception:
            pass
        self._prev_start = start
        try:
            self._step()
        finally:
            try:
                self.work_ms.append((time.perf_counter() - start) * 1000.0)
            except Exception:
                pass

    def _step(self) -> None:
        read_start = time.monotonic()
        try:
            current_rad = [float(v) for v in self.group.get_positions()]
            if len(current_rad) != 6:
                raise ValueError(f"got {len(current_rad)} positions")
        except Exception as exc:
            self._refuse("read-failed", str(exc), "six joint angles")
            return
        age_ms = (time.monotonic() - read_start) * 1000.0
        current_deg = [math.degrees(v) for v in current_rad]
        try:
            self.on_sample(list(current_deg))
        except Exception:
            pass
        if (self.snapshot_deg is None and self.snapshot_index is not None
                and self.index == self.snapshot_index):
            self.snapshot_deg = list(current_deg)
        if self.refusal is not None or self.halt or self.index >= len(self.waypoints):
            if self.refusal is None and self.halt and self.index < len(self.waypoints):
                self._refuse("user-halt", self.index, len(self.waypoints))
                return
            self._hold()
            return
        waypoint = self.waypoints[self.index]
        target_tip = self.target_tips[self.index]
        try:
            current_tip = self.tip_of(current_deg)
        except Exception as exc:
            self._refuse("read-failed", str(exc), "tool tip")
            return
        target = safety.Target(
            x=target_tip[0], y=target_tip[1], z=target_tip[2],
            joints_deg=tuple(waypoint), speed=SPEED_FRACTION,
        )
        current = safety.CurrentState(
            x=current_tip[0], y=current_tip[1], z=current_tip[2],
            joints_deg=tuple(current_deg), reading_age_ms=age_ms,
        )
        result = self.check(target, current)
        if not result.get("ok"):
            self.refusal = dict(result)
            self._hold()
            return
        commanded = [math.radians(v) for v in waypoint]
        self._send(commanded)
        self.last_commanded_rad = commanded
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done.set()
