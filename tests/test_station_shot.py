"""Tests for the station table overview shot.

Offline only: fake camera opener, real cv2 imwrite into tmp
folders. No camera, no arm, no network. Never run against real
devices.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy

from muse_qc.inspection import station_shot


def _frame():
    return numpy.zeros((48, 64, 3), dtype=numpy.uint8)


def test_shot_saves_frame_and_prints_exact_line(tmp_path):
    lines = []
    path = station_shot.take_station_table_shot(
        lines.append, open_camera=lambda: (_frame(), {}),
        recordings_dir=str(tmp_path))
    assert path is not None
    assert Path(path).is_file()
    assert Path(path).name.startswith("station_table_")
    assert lines == [f"station table shot: {path}"]


def test_shot_camera_error_prints_failed_and_continues():
    def boom():
        raise RuntimeError("camera gone")

    lines = []
    assert station_shot.take_station_table_shot(
        lines.append, open_camera=boom,
        recordings_dir="recordings") is None
    assert lines == ["station table shot failed: camera gone"]


def test_shot_keeps_only_first_error_line():
    def boom():
        raise RuntimeError("first line\nsecond line")

    lines = []
    assert station_shot.take_station_table_shot(
        lines.append, open_camera=boom,
        recordings_dir="recordings") is None
    assert lines == ["station table shot failed: first line"]


def test_shot_missing_frame_prints_failed():
    lines = []
    assert station_shot.take_station_table_shot(
        lines.append, open_camera=lambda: (None, {}),
        recordings_dir="recordings") is None
    assert lines == ["station table shot failed: no frame"]


def test_shot_timeout_prints_failed_and_returns_fast(tmp_path):
    gate = threading.Event()

    def slow():
        gate.wait(30.0)
        return (_frame(), {})

    lines = []
    started = time.monotonic()
    assert station_shot.take_station_table_shot(
        lines.append, open_camera=slow,
        recordings_dir=str(tmp_path), timeout_s=0.05) is None
    assert time.monotonic() - started < 5.0
    assert lines == ["station table shot failed: timed out after 0.05 s"]


def test_parallel_shot_costs_max_not_sum(tmp_path):
    def slow_camera():
        time.sleep(0.5)
        return (_frame(), {})

    lines = []
    shot = station_shot.start_station_table_shot(
        lines.append, open_camera=slow_camera,
        recordings_dir=str(tmp_path))
    started = time.monotonic()
    time.sleep(0.5)  # the slow fake inspect (lever count)
    remaining = (station_shot.STATION_SHOT_TIMEOUT_S
                 - (time.monotonic() - shot.started))
    path = shot.join(max(remaining, 0.0))
    total = time.monotonic() - started
    assert path is not None
    assert total < 0.9
    assert lines == [f"station table shot: {path}"]
