"""Tests for camera_arm.py. No real camera is opened; sysfs and capture are faked."""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy

import muse_qc.vision.camera_arm as camera_arm
from muse_qc.vision.camera_arm import (
    ARM_CAM_HEIGHT,
    ARM_CAM_WIDTH,
    find_arm_color_node,
)


def _arm_frame():
    """Return a 640x480 BGR frame."""
    frame = numpy.zeros((ARM_CAM_HEIGHT, ARM_CAM_WIDTH, 3), dtype=numpy.uint8)
    frame[0, 0] = (10, 20, 30)
    return frame


def _make_sysfs(root: Path, gemini_start=0, trust_names=("video10", "video11")):
    """Build a fake sysfs tree: 10 Gemini nodes plus the Trust camera.

    The Gemini nodes map like the real device: the first six on USB
    interface 1.0, the next two on 1.4, the last two on 1.6; even
    nodes carry index 0, odd nodes index 1. The Trust camera sits at
    1-4:1.0 with ids 145f:0203.
    """
    v4l = root / "class" / "video4linux"
    v4l.mkdir(parents=True)
    gemini = root / "usb" / "2-1"
    gemini.mkdir(parents=True)
    (gemini / "idVendor").write_text("2bc5\n")
    (gemini / "idProduct").write_text("0840\n")
    interfaces = {}
    for suffix, number in (("1.0", "00"), ("1.4", "04"), ("1.6", "06")):
        interface = gemini / f"2-1:{suffix}"
        interface.mkdir()
        (interface / "bInterfaceNumber").write_text(number + "\n")
        interfaces[suffix] = interface
    trust = root / "usb" / "1-4"
    trust.mkdir(parents=True)
    (trust / "idVendor").write_text("145f\n")
    (trust / "idProduct").write_text("0203\n")
    trust_interface = trust / "1-4:1.0"
    trust_interface.mkdir()
    (trust_interface / "bInterfaceNumber").write_text("00\n")

    def _node(name, interface, index):
        node = v4l / name
        node.mkdir()
        (node / "index").write_text(f"{index}\n")
        os.symlink(interface, node / "device")

    layout = ["1.0", "1.0", "1.0", "1.4", "1.6"]
    for pos in range(10):
        _node(
            f"video{gemini_start + pos}",
            interfaces[layout[pos // 2]],
            0 if pos % 2 == 0 else 1,
        )
    for pos, name in enumerate(trust_names):
        _node(name, trust_interface, pos)
    return v4l


def test_picks_video6(tmp_path):
    v4l = _make_sysfs(tmp_path)
    assert find_arm_color_node(v4l) == "/dev/video6"


def test_still_picked_when_numbers_shifted(tmp_path):
    v4l = _make_sysfs(tmp_path, gemini_start=2, trust_names=("video0", "video1"))
    assert find_arm_color_node(v4l) == "/dev/video8"


def test_none_when_camera_missing(tmp_path):
    v4l = tmp_path / "class" / "video4linux"
    v4l.mkdir(parents=True)
    trust = tmp_path / "usb" / "1-4"
    trust.mkdir(parents=True)
    (trust / "idVendor").write_text("145f\n")
    (trust / "idProduct").write_text("0203\n")
    interface = trust / "1-4:1.0"
    interface.mkdir()
    (interface / "bInterfaceNumber").write_text("00\n")
    node = v4l / "video10"
    node.mkdir()
    (node / "index").write_text("0\n")
    os.symlink(interface, node / "device")
    assert find_arm_color_node(v4l) is None


def test_none_when_no_index_zero_on_interface_4(tmp_path):
    v4l = _make_sysfs(tmp_path)
    for entry in sorted(v4l.iterdir()):
        if entry.name.startswith("video"):
            (entry / "index").write_text("1\n")
    assert find_arm_color_node(v4l) is None


class FakeCapture:
    """Fake cv2.VideoCapture with a scripted read outcome per instance."""

    instances: list["FakeCapture"] = []
    behaviours: list[bool] = []

    def __init__(self, device, backend=None):
        self.device = device
        self.backend = backend
        self.ok = FakeCapture.behaviours.pop(0)
        self.frame = _arm_frame() if self.ok else None
        self.set_calls: list[tuple[int, object]] = []
        self.reads = 0
        self.released = False
        FakeCapture.instances.append(self)

    def set(self, prop, value):
        self.set_calls.append((prop, value))
        return True

    def read(self):
        self.reads += 1
        return self.ok, self.frame

    def release(self):
        self.released = True


def _patch_capture(monkeypatch, behaviours):
    FakeCapture.instances.clear()
    FakeCapture.behaviours = list(behaviours)
    monkeypatch.setattr(cv2, "VideoCapture", FakeCapture)


def test_retry_after_failed_attempt(monkeypatch):
    _patch_capture(monkeypatch, [False, True])
    frame = camera_arm.open_arm_camera("/dev/video6")
    assert frame is not None
    assert frame.shape == (480, 640, 3)
    assert len(FakeCapture.instances) == 2
    assert all(capture.released for capture in FakeCapture.instances)


def test_release_is_always_called(monkeypatch):
    _patch_capture(monkeypatch, [False, False, False])
    assert camera_arm.open_arm_camera("/dev/video6") is None
    assert len(FakeCapture.instances) == 3
    assert all(capture.released for capture in FakeCapture.instances)


def test_failing_attempt_aborts_warmup_early(monkeypatch):
    _patch_capture(monkeypatch, [False, True])
    frame = camera_arm.open_arm_camera("/dev/video6")
    assert frame is not None
    assert (
        FakeCapture.instances[0].reads
        == camera_arm.ARM_CAM_MAX_CONSECUTIVE_WARMUP_FAILURES
    )
    assert FakeCapture.instances[1].reads == camera_arm.ARM_CAM_WARMUP_FRAMES + 1
    assert all(capture.released for capture in FakeCapture.instances)


def test_works_without_read_timeout_attr(monkeypatch):
    monkeypatch.delattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", raising=False)
    _patch_capture(monkeypatch, [True])
    assert camera_arm.open_arm_camera("/dev/video6") is not None


def test_capture_uses_v4l2_yuyv_640x480(monkeypatch):
    _patch_capture(monkeypatch, [True])
    frame = camera_arm.open_arm_camera("/dev/video6")
    assert frame is not None
    (capture,) = FakeCapture.instances
    assert capture.backend == cv2.CAP_V4L2
    assert capture.device == "/dev/video6"
    assert (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV")) in capture.set_calls
    assert (cv2.CAP_PROP_FRAME_WIDTH, 640) in capture.set_calls
    assert (cv2.CAP_PROP_FRAME_HEIGHT, 480) in capture.set_calls
    assert capture.reads == camera_arm.ARM_CAM_WARMUP_FRAMES + 1


def test_frame_is_not_rotated(monkeypatch):
    _patch_capture(monkeypatch, [True])
    frame = camera_arm.open_arm_camera("/dev/video6")
    assert frame is not None
    assert frame[0, 0].tolist() == [10, 20, 30]


def _patch_main(monkeypatch, tmp_path, node, frame, imwrite_result):
    monkeypatch.setattr(camera_arm, "find_arm_color_node", lambda *a, **k: node)
    monkeypatch.setattr(camera_arm, "open_arm_camera", lambda *a, **k: frame)
    monkeypatch.setattr(cv2, "imwrite", lambda *a, **k: imwrite_result)
    monkeypatch.chdir(tmp_path)


def test_main_camera_not_found_returns_1(monkeypatch, tmp_path, capsys):
    _patch_main(monkeypatch, tmp_path, None, None, True)
    assert camera_arm.main() == 1
    assert "not found" in capsys.readouterr().out


def test_main_no_frame_returns_2(monkeypatch, tmp_path, capsys):
    _patch_main(monkeypatch, tmp_path, "/dev/video6", None, True)
    assert camera_arm.main() == 2
    assert "no frame" in capsys.readouterr().out


def test_main_write_failure_returns_3(monkeypatch, tmp_path):
    _patch_main(monkeypatch, tmp_path, "/dev/video6", _arm_frame(), False)
    assert camera_arm.main() == 3


def test_source_has_no_fixed_device_path():
    source = Path("muse_qc/vision/camera_arm.py").read_text(encoding="utf-8")
    assert "2-1:1.4" not in source
    assert "/dev/video6" not in source
