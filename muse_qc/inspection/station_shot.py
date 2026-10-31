"""Table overview shot while the part is held at the inspection station.

Called by muse_qc.pick.pick_test --execute: start_station_table_shot
starts the shot in a background thread right before the lever
count, so the table camera opens while SAM counts; join with the
remaining time of STATION_SHOT_TIMEOUT_S after the count, before
the verdict reaches the loop. The extra hold at the station is at
most max(0, shot time - inspect time), normally zero. The frame is
only saved as recordings/station_table_<stamp>.png with exactly
one printed line; no SAM call happens here: the agent rescan reads
the saved file later.

Printing happens only in join: on a join timeout the failed line
is printed once and a late worker stays silent, so the hold, the
lever count, the drop and the return never wait longer than the
remaining budget. take_station_table_shot is the synchronous
wrapper (start plus join with the full timeout).
"""

from __future__ import annotations

import threading
import time
from datetime import datetime
from pathlib import Path

import cv2

STATION_SHOT_TIMEOUT_S = 6.0


def _short_reason(exc: Exception) -> str:
    reason = str(exc).splitlines()
    if reason and reason[0]:
        return reason[0][:160]
    return type(exc).__name__


class StationShot:
    """Background station overview shot; only join prints."""

    def __init__(self, out=print, open_camera=None,
                 recordings_dir="recordings"):
        from muse_qc.vision import camera_top

        self._out = out
        self._camera_of = open_camera or camera_top.open_table_camera
        self._recordings_dir = recordings_dir
        self.started = time.monotonic()
        self._done = threading.Event()
        self._result: dict = {}
        self._consumed = False
        self._lock = threading.Lock()
        thread = threading.Thread(target=self._work, daemon=True)
        thread.start()

    def _work(self) -> None:
        try:
            frame, _settings = self._camera_of()
        except Exception as exc:  # noqa: BLE001 - stored, printed in join
            self._result["error"] = _short_reason(exc)
        else:
            self._result["frame"] = frame
        finally:
            self._done.set()

    def join(self, timeout_s=STATION_SHOT_TIMEOUT_S) -> str | None:
        """Wait up to timeout_s, print exactly one line, return path/None.

        Prints "station table shot: <path>" on success and
        "station table shot failed: <short reason>" on any camera,
        save or timeout failure. A second join prints nothing and
        returns the saved path (or None); a worker finishing after
        a timed-out join stays silent.
        """
        with self._lock:
            if self._consumed:
                return self._result.get("path")
            self._consumed = True
        if not self._done.wait(timeout=max(float(timeout_s), 0.0)):
            self._out("station table shot failed:"
                      f" timed out after {timeout_s:g} s")
            return None
        if self._result.get("frame") is None:
            reason = self._result.get("error") or "no frame"
            self._out(f"station table shot failed: {reason}")
            return None
        target_dir = Path(self._recordings_dir)
        target_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = target_dir / f"station_table_{stamp}.png"
        if not cv2.imwrite(str(path), self._result["frame"]):
            self._out(f"station table shot failed: could not save {path}")
            return None
        self._out(f"station table shot: {path}")
        self._result["path"] = str(path)
        return str(path)


def start_station_table_shot(out=print, open_camera=None,
                             recordings_dir="recordings") -> StationShot:
    """Start the table shot in a background thread; join it later."""
    return StationShot(out, open_camera=open_camera,
                       recordings_dir=recordings_dir)


def take_station_table_shot(out=print, open_camera=None,
                            recordings_dir="recordings",
                            timeout_s=STATION_SHOT_TIMEOUT_S,
                            ) -> str | None:
    """Take one table frame during the station hold; return its path.

    Synchronous wrapper: start plus join with the full timeout.
    Prints exactly one line (see StationShot.join). All camera
    access is injected (fakes in tests), the default is the real
    table camera opener.
    """
    shot = start_station_table_shot(
        out, open_camera=open_camera, recordings_dir=recordings_dir)
    return shot.join(timeout_s)
