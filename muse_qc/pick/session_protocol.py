"""Stdin protocol of the arm session. Pure, moves nothing.

Targets come on stdin, one per line ("part X Y LEVER", "part
X Y" without a lever, or "end"). Only full lines count: a
trailing partial line without a newline is dropped. Blank
lines are ignored. feed_lines reads them into a queue on a
thread, then an _EOF marker, so the main thread can wait for
the next target with a timeout while the arm holds its pose.
"""

from __future__ import annotations

import math
import queue
import threading
import time

HOLD_WAIT_S = 30.0
PART_DONE_LINE = "session: part done"
END_PREFIX = "session: end"
_EOF = object()


class SessionClock:
    """Time source for the session main-thread waits (real by default)."""

    def now(self) -> float:
        """Return the current time in seconds."""
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        """Wait out seconds in real time."""
        time.sleep(seconds)


class FakeClock(SessionClock):
    """Virtual clock for deterministic tests: only pumping advances time.

    sleep only yields the thread; virtual time moves exclusively
    through advance, which the test calls once per pumped loop
    cycle. Main-thread deadlines therefore bind at deterministic
    pump counts, never from wall-clock races.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._now = 0.0

    def now(self) -> float:
        """Return the virtual time in seconds."""
        with self._lock:
            return self._now

    def advance(self, seconds: float) -> None:
        """Move virtual time forward (called once per pumped cycle)."""
        with self._lock:
            self._now += max(0.0, float(seconds))

    def sleep(self, seconds: float) -> None:
        """Yield without advancing virtual time."""
        del seconds
        time.sleep(0)


def parse_session_line(line: str):
    """Parse one stdin line: ("part", x, y, lever|None), ("end",) or None."""
    words = str(line).strip().split()
    if words == ["end"]:
        return ("end",)
    if len(words) in (3, 4) and words[0] == "part":
        try:
            x, y = float(words[1]), float(words[2])
            lever = None if len(words) == 3 else float(words[3])
        except ValueError:
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        if lever is not None and not (
                math.isfinite(lever) and 0.0 <= lever <= 180.0):
            return None
        return ("part", x, y, lever)
    return None


def feed_lines(stdin, out_queue: queue.Queue) -> None:
    """Read full lines from stdin into the queue, then an _EOF marker."""
    try:
        if hasattr(stdin, "readline"):
            while True:
                raw = stdin.readline()
                if raw == "":
                    break
                if raw.endswith("\n") and raw.strip():
                    out_queue.put(raw.strip())
        else:
            for raw in stdin:
                if str(raw).strip():
                    out_queue.put(str(raw).strip())
    finally:
        out_queue.put(_EOF)
