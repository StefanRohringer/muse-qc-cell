"""Tests for camera_top.py. No real camera is opened; capture is faked."""

from __future__ import annotations

import os

import cv2
import numpy
import pytest

import muse_qc.vision.camera_top as camera_top
from muse_qc.vision.camera_top import (
    TABLE_CAM_AUTO_EXPOSURE,
    TABLE_CAM_AUTO_WB,
    TABLE_CAM_BRIGHTNESS,
    TABLE_CAM_CONTRAST,
    TABLE_CAM_EXPOSURE,
    TABLE_CAM_FOURCC,
    TABLE_CAM_GAIN,
    TABLE_CAM_HEIGHT,
    TABLE_CAM_ROTATE_180,
    TABLE_CAM_SETTINGS,
    TABLE_CAM_WARMUP_FRAMES,
    TABLE_CAM_WB_TEMPERATURE,
    TABLE_CAM_WIDTH,
)


def _full_frame(top_left=(10, 20, 30), bottom_right=(100, 110, 120)):
    """Return a full-size BGR frame with distinct corner pixels."""
    frame = numpy.zeros((TABLE_CAM_HEIGHT, TABLE_CAM_WIDTH, 3), dtype=numpy.uint8)
    frame[0, 0] = top_left
    frame[-1, -1] = bottom_right
    return frame


def _small_frame(width=640, height=480):
    """Return a frame with a wrong (too small) size."""
    return numpy.zeros((height, width, 3), dtype=numpy.uint8)


class FakeCapture:
    """Fake cv2.VideoCapture recording set calls, reads and release."""

    instances: list["FakeCapture"] = []

    def __init__(self, device, backend=None, frames=None, ok: bool = True):
        self.device = device
        self.backend = backend
        if frames is None:
            frames = [_full_frame()]
        self._frames = list(frames)
        self._last = self._frames[-1] if self._frames else None
        self._ok = ok
        self.set_calls: list[tuple[int, float]] = []
        self.events: list[tuple[str, int | None]] = []
        self.values: dict[int, float] = {}
        self.released = False
        FakeCapture.instances.append(self)

    def set(self, prop, value):
        self.set_calls.append((prop, value))
        self.events.append(("set", prop))
        self.values[prop] = value
        return True

    def get(self, prop):
        self.events.append(("get", prop))
        return self.values.get(prop, 0.0)

    def read(self):
        self.events.append(("read", None))
        if self._frames:
            self._last = self._frames.pop(0)
        return self._ok, self._last

    def release(self):
        self.released = True


def _patch_camera(monkeypatch, devices, frames=None, ok=True):
    FakeCapture.instances.clear()
    monkeypatch.setattr(camera_top, "find_video_devices", lambda *a, **k: devices)

    def _fake_capture(device, backend=None):
        return FakeCapture(device, backend=backend, frames=frames, ok=ok)

    monkeypatch.setattr(cv2, "VideoCapture", _fake_capture)


def _capture():
    assert len(FakeCapture.instances) == 1
    return FakeCapture.instances[0]


def test_one_frame_is_saved_rotated(tmp_path, monkeypatch, capsys):
    _patch_camera(monkeypatch, ["/dev/video0"])
    monkeypatch.chdir(tmp_path)
    camera_top.main()
    out = capsys.readouterr().out
    files = sorted((tmp_path / "recordings").glob("table_*.png"))
    assert len(files) == 1
    assert files[0].name in out
    assert "1920x1080" in out
    saved = cv2.imread(str(files[0]))
    assert saved is not None
    assert saved.shape[1] == 1920
    assert saved.shape[0] == 1080
    original = _full_frame()
    assert saved[-1, -1].tolist() == original[0, 0].tolist()
    assert saved[0, 0].tolist() == original[-1, -1].tolist()


def test_open_returns_rotated_frame(monkeypatch):
    original = _full_frame()
    _patch_camera(monkeypatch, ["/dev/video2"], frames=[original])
    frame, _ = camera_top.open_table_camera()
    assert frame[-1, -1].tolist() == original[0, 0].tolist()


def test_capture_uses_v4l2_backend(monkeypatch):
    _patch_camera(monkeypatch, ["/dev/video0"])
    camera_top.open_table_camera()
    assert _capture().backend == cv2.CAP_V4L2


def test_settings_are_set_before_first_read(monkeypatch):
    _patch_camera(monkeypatch, ["/dev/video0"])
    camera_top.open_table_camera()
    capture = _capture()
    first_read = capture.events.index(("read", None))
    assert capture.set_calls == [
        (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*TABLE_CAM_FOURCC)),
        (cv2.CAP_PROP_FRAME_WIDTH, TABLE_CAM_WIDTH),
        (cv2.CAP_PROP_FRAME_HEIGHT, TABLE_CAM_HEIGHT),
        (cv2.CAP_PROP_AUTO_EXPOSURE, TABLE_CAM_AUTO_EXPOSURE),
        (cv2.CAP_PROP_EXPOSURE, TABLE_CAM_EXPOSURE),
        (cv2.CAP_PROP_GAIN, TABLE_CAM_GAIN),
        (cv2.CAP_PROP_BRIGHTNESS, TABLE_CAM_BRIGHTNESS),
        (cv2.CAP_PROP_CONTRAST, TABLE_CAM_CONTRAST),
        (cv2.CAP_PROP_AUTO_WB, TABLE_CAM_AUTO_WB),
        (cv2.CAP_PROP_WB_TEMPERATURE, TABLE_CAM_WB_TEMPERATURE),
    ]
    assert all(kind == "set" for kind, _ in capture.events[:first_read])
    assert len(capture.events[:first_read]) == 10


def test_fixed_setting_values():
    assert TABLE_CAM_AUTO_EXPOSURE == 1
    assert TABLE_CAM_EXPOSURE == 450
    assert TABLE_CAM_GAIN == 0
    assert TABLE_CAM_BRIGHTNESS == 160
    assert TABLE_CAM_CONTRAST == 150
    assert TABLE_CAM_AUTO_WB == 0
    assert TABLE_CAM_WB_TEMPERATURE == 3200
    assert [name for name, _, _ in TABLE_CAM_SETTINGS] == [
        "auto_exposure",
        "exposure",
        "gain",
        "brightness",
        "contrast",
        "auto_wb",
        "wb_temperature",
    ]


def test_read_back_values_are_returned(monkeypatch):
    _patch_camera(monkeypatch, ["/dev/video0"])
    _, settings = camera_top.open_table_camera()
    assert settings == {
        "auto_exposure": TABLE_CAM_AUTO_EXPOSURE,
        "exposure": TABLE_CAM_EXPOSURE,
        "gain": TABLE_CAM_GAIN,
        "brightness": TABLE_CAM_BRIGHTNESS,
        "contrast": TABLE_CAM_CONTRAST,
        "auto_wb": TABLE_CAM_AUTO_WB,
        "wb_temperature": TABLE_CAM_WB_TEMPERATURE,
    }


def test_main_prints_read_back_values(tmp_path, monkeypatch, capsys):
    _patch_camera(monkeypatch, ["/dev/video0"])
    monkeypatch.chdir(tmp_path)
    camera_top.main()
    out = capsys.readouterr().out
    for text in (
        "auto_exposure=1",
        "exposure=450",
        "gain=0",
        "brightness=160",
        "contrast=150",
        "auto_wb=0",
        "wb_temperature=3200",
    ):
        assert text in out


def test_warmup_frames_are_discarded(monkeypatch):
    warmups = [numpy.zeros((2, 2, 3), dtype=numpy.uint8)] * TABLE_CAM_WARMUP_FRAMES
    final = _full_frame()
    _patch_camera(monkeypatch, ["/dev/video0"], frames=[*warmups, final])
    frame, _ = camera_top.open_table_camera()
    capture = _capture()
    reads = [event for event in capture.events if event[0] == "read"]
    assert len(reads) == TABLE_CAM_WARMUP_FRAMES + 1
    assert frame[-1, -1].tolist() == final[0, 0].tolist()


def test_wrong_frame_size_raises_and_releases(monkeypatch):
    _patch_camera(
        monkeypatch,
        ["/dev/video0"],
        frames=[_small_frame()] * (TABLE_CAM_WARMUP_FRAMES + 1),
    )
    with pytest.raises(RuntimeError, match="640x480"):
        camera_top.open_table_camera()
    assert _capture().released


def test_zero_devices_raises_and_opens_nothing(monkeypatch):
    _patch_camera(monkeypatch, [])
    with pytest.raises(RuntimeError, match="not found"):
        camera_top.open_table_camera()
    assert FakeCapture.instances == []


def test_several_devices_raise_and_open_nothing(monkeypatch):
    _patch_camera(monkeypatch, ["/dev/video0", "/dev/video1"])
    with pytest.raises(RuntimeError, match="several"):
        camera_top.open_table_camera()
    assert FakeCapture.instances == []


def test_camera_is_released_after_success(monkeypatch):
    _patch_camera(monkeypatch, ["/dev/video0"])
    camera_top.open_table_camera()
    assert _capture().released


def test_camera_is_released_on_read_error(monkeypatch):
    _patch_camera(monkeypatch, ["/dev/video0"], frames=[], ok=False)
    with pytest.raises(RuntimeError, match="failed to read"):
        camera_top.open_table_camera()
    assert _capture().released


def test_never_imports_arm_code():
    path = os.path.join(os.path.dirname(__file__), "..", "muse_qc", "vision", "camera_top.py")
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "from muse_qc.arm.arm" not in source
    assert "from muse_qc import arm" not in source
    assert "import arm" not in source
    assert "safety" not in source
    assert TABLE_CAM_ROTATE_180 is True
    assert "ROTATE_180" in source
