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
import sys
import threading
import time

from muse_qc.bringup.first_move import CONFIRM_WORD
from muse_qc.gripper.gripper_motor import CONFIRM_PROMPT

HOLD_WAIT_S = 30.0
PART_DONE_LINE = "session: part done"
END_PREFIX = "session: end"
_EOF = object()
_CONFIRM_TIMEOUT_S = 60.0


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


def confirm_session(input_fn, lines, out, tty_opener=open,
                    from_stdin=False) -> bool:
    """Confirm --execute (see pick_session for the routing).

    With --confirm-stdin the first stdin line is the confirmation
    and must read exactly CONFIRM_WORD; it is never parsed as a
    target. Otherwise the confirmation comes only from /dev/tty
    (tty_opener is injectable for tests); stdin is never read
    for it.
    """
    if input_fn is not None:
        return input_fn(CONFIRM_PROMPT).strip() == CONFIRM_WORD
    if from_stdin:
        try:
            first = lines.get(timeout=_CONFIRM_TIMEOUT_S)
        except queue.Empty:
            return False
        return first is not _EOF and str(first).strip() == CONFIRM_WORD
    try:
        with tty_opener("/dev/tty", "r", encoding="utf-8") as handle:
            print(CONFIRM_PROMPT, end="", file=sys.stderr)
            return handle.readline().strip() == CONFIRM_WORD
    except OSError:
        return False


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
