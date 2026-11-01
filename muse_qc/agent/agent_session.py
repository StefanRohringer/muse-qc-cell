"""One pick_session child per sort_in_order plan (--arm-session).

ArmSession starts a single `python -m muse_qc.pick.pick_session`
child, keeps its stdin open, and feeds one "part X Y [LEVER]"
line per sort. The first stdin line is always "yes": the
session confirmation (same stdin pipe mechanism the agent
already uses to start pick_test children, but kept open). Each
sort reads child lines until "session: part done" and parses
that block with the existing parser, so the result dict has
exactly the real_sort shape plus "session_over". close sends
"end", closes stdin and drains the child through its own
return-to-rest plus disable ("session: end (...)"). A
replanned order opens a new session. Fakes only in tests.
"""

from __future__ import annotations

import queue
import signal
import threading
import time

from muse_qc.agent.agent_io import SORT_TIMEOUT_S
from muse_qc.agent.agent_parse import parse_pick_lines

PART_DONE_LINE = "session: part done"
END_PREFIX = "session: end"
_EOF = object()


def start_session_proc(cmd):
    """Start cmd with merged stderr, text pipes and an open stdin.

    Same shape as agent_io.default_popen, but stdin stays open:
    the caller writes "yes" first (the session confirmation),
    then one target per line, then "end".
    """
    import os
    import subprocess

    env = dict(os.environ)
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.Popen(
        cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, env=env)


def _drain(proc, out_queue) -> None:
    """Forward child stdout lines into the queue, then an _EOF marker."""
    try:
        while True:
            raw = proc.stdout.readline()
            if raw == "":
                break
            out_queue.put(raw.rstrip("\n"))
    finally:
        out_queue.put(_EOF)


class ArmSession:
    """One pick_session child feeding a whole sort_in_order plan."""

    def __init__(self, cmd, popen_factory=None, timeout_s=SORT_TIMEOUT_S,
                 clock=None, out=print, close_grace_s=180.0):
        self._out = out
        self._timeout_s = float(timeout_s)
        self._ticks = clock if clock is not None else time.monotonic
        self._grace_s = float(close_grace_s)
        factory = popen_factory if popen_factory is not None \
            else start_session_proc
        self._proc = factory(cmd)
        self._lines: queue.Queue = queue.Queue()
        self._over = False
        threading.Thread(target=_drain, args=(self._proc, self._lines),
                         daemon=True).start()
        try:
            self._proc.stdin.write("yes\n")
            self._proc.stdin.flush()
        except (OSError, ValueError):
            self._over = True

    @property
    def over(self) -> bool:
        """Whether the session ended (end line, EOF, timeout or error)."""
        return self._over

    def _error(self, reason) -> dict:
        """Session-level error result in the real_sort shape."""
        return {"outcome": "error", "levers": None, "verdict": None,
                "box": None, "record": None, "station_table_image": None,
                "reason": str(reason), "lines": [str(reason)],
                "exit_code": None, "session_over": True}

    def sort(self, table_mm, lever_deg=None) -> dict:
        """Sort one part through the session; same shape as real_sort."""
        if self._over:
            return self._error("arm session already over, replan")
        x, y = float(table_mm[0]), float(table_mm[1])
        line = f"part {x} {y}" if lever_deg is None \
            else f"part {x} {y} {float(lever_deg)}"
        try:
            self._proc.stdin.write(line + "\n")
            self._proc.stdin.flush()
        except (OSError, ValueError):
            self._over = True
            return self._error("arm session stdin broken")
        block: list[str] = []
        framed = ended = False
        deadline = self._ticks() + self._timeout_s
        while True:
            remaining = deadline - self._ticks()
            if remaining <= 0:
                return self._on_timeout(block)
            try:
                raw = self._lines.get(timeout=min(1.0, remaining))
            except queue.Empty:
                if self._proc.poll() is not None:
                    break
                continue
            if raw is _EOF:
                break
            self._out(raw)
            block.append(raw)
            if raw == PART_DONE_LINE:
                framed = True
                break
            if raw.startswith(END_PREFIX):
                ended = True
                break
        if not framed and not ended:
            self._over = True
            block.append("session exited before session: part done")
        if ended:
            self._over = True
        result = parse_pick_lines(block)
        result["lines"] = block[-12:]
        clean = result.get("outcome") == "sorted" \
            and result.get("reason") is None
        result["exit_code"] = 0 if clean else 1
        result["session_over"] = self._over
        return result

    def _on_timeout(self, block) -> dict:
        """SIGINT the child (it returns to rest itself), then parse."""
        note = f"sort timed out after {self._timeout_s:g} s"
        self._out(note)
        block.append(note)
        try:
            self._proc.send_signal(signal.SIGINT)
        except (OSError, ValueError, RuntimeError):
            pass
        self._drain_until_end()
        try:
            code = self._proc.wait(timeout=30.0)
        except Exception:  # noqa: BLE001 - last resort, never hang the agent
            try:
                self._proc.kill()
            except (OSError, ValueError):
                pass
            code = self._proc.wait()
        self._over = True
        result = parse_pick_lines(block)
        result["lines"] = block[-12:]
        result["exit_code"] = code
        result["session_over"] = True
        return result

    def _drain_until_end(self) -> None:
        """Drain lines until the session end line or EOF, bounded."""
        deadline = self._ticks() + self._grace_s
        while self._ticks() < deadline:
            try:
                raw = self._lines.get(timeout=1.0)
            except queue.Empty:
                if self._proc.poll() is not None:
                    return
                continue
            if raw is _EOF:
                return
            self._out(raw)
            if raw.startswith(END_PREFIX):
                return

    def close(self, reason="done") -> None:
        """Send "end", close stdin, drain the child through disable."""
        del reason  # the child prints its own end reason
        if not self._over:
            stdin = getattr(self._proc, "stdin", None)
            if stdin is not None:
                try:
                    stdin.write("end\n")
                    stdin.flush()
                    stdin.close()
                except (OSError, ValueError):
                    pass
        self._drain_until_end()
        try:
            self._proc.wait(timeout=30.0)
        except Exception:  # noqa: BLE001 - last resort, never hang the agent
            try:
                self._proc.kill()
            except (OSError, ValueError):
                pass
            try:
                self._proc.wait()
            except Exception:  # noqa: BLE001 - already leaving
                pass
        self._over = True
