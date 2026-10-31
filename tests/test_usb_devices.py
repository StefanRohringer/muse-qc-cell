"""Tests for usb_devices.py. No hardware is touched; sysfs is faked."""

from __future__ import annotations

import os
from pathlib import Path

from muse_qc.checks.usb_devices import (
    ARM_USB_PID,
    ARM_USB_VID,
    TABLE_CAM_USB_PID,
    TABLE_CAM_USB_VID,
    find_serial_ports,
    find_video_devices,
)


def _make_tty(root: Path, name: str, vid: str | None, pid: str | None) -> None:
    """Create a fake sysfs entry <root>/<name>/device with ID files."""
    device = root / name / "device"
    device.mkdir(parents=True, exist_ok=True)
    if vid is not None:
        (device / "idVendor").write_text(vid + "\n", encoding="utf-8")
    if pid is not None:
        (device / "idProduct").write_text(pid + "\n", encoding="utf-8")


def test_constants_name_the_arm_ids():
    assert ARM_USB_VID == "2e88"
    assert ARM_USB_PID == "4603"


def test_one_match(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e88", "4603")
    assert find_serial_ports("2e88", "4603", tmp_path) == ["/dev/ttyACM0"]


def test_no_match(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e88", "4603")
    assert find_serial_ports("2e88", "9999", tmp_path) == []


def test_two_matches(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e88", "4603")
    _make_tty(tmp_path, "ttyACM1", "2e88", "4603")
    assert find_serial_ports("2e88", "4603", tmp_path) == [
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    ]


def test_gripper_pico_vid_is_ignored(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e88", "4603")
    _make_tty(tmp_path, "ttyACM1", "2e8a", "4603")
    assert find_serial_ports("2e88", "4603", tmp_path) == ["/dev/ttyACM0"]


def test_uppercase_hex_matches(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2E88", "4603")
    _make_tty(tmp_path, "ttyUSB0", "2e88", "4603")
    assert find_serial_ports("2E88", "4603", tmp_path) == [
        "/dev/ttyACM0",
        "/dev/ttyUSB0",
    ]


def test_non_acm_usb_names_are_ignored(tmp_path):
    _make_tty(tmp_path, "ttyS0", "2e88", "4603")
    assert find_serial_ports("2e88", "4603", tmp_path) == []


def test_missing_sysfs_root_returns_empty(tmp_path):
    assert find_serial_ports("2e88", "4603", tmp_path / "no-such-dir") == []


def test_entry_without_device_link_is_skipped(tmp_path):
    (tmp_path / "ttyACM0").mkdir(parents=True, exist_ok=True)
    assert find_serial_ports("2e88", "4603", tmp_path) == []


def test_never_opens_a_serial_port(tmp_path):
    _make_tty(tmp_path, "ttyACM0", "2e88", "4603")
    path = os.path.join(os.path.dirname(__file__), "..", "muse_qc", "checks", "usb_devices.py")
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "import serial" not in source
    assert "from serial" not in source
    assert "open(\"/dev" not in source


def _make_video(
    root: Path, name: str, vid: str | None, pid: str | None, index: str | None = None
) -> None:
    """Create a fake sysfs entry <root>/<name>/device with ID files."""
    device = root / name / "device"
    device.mkdir(parents=True, exist_ok=True)
    if vid is not None:
        (device / "idVendor").write_text(vid + "\n", encoding="utf-8")
    if pid is not None:
        (device / "idProduct").write_text(pid + "\n", encoding="utf-8")
    if index is not None:
        (root / name / "index").write_text(index + "\n", encoding="utf-8")


def test_table_cam_constants():
    assert TABLE_CAM_USB_VID == "145f"
    assert TABLE_CAM_USB_PID == "0203"


def test_video_one_match(tmp_path):
    _make_video(tmp_path, "video0", "145f", "0203")
    assert find_video_devices("145f", "0203", tmp_path) == ["/dev/video0"]


def test_video_no_match(tmp_path):
    _make_video(tmp_path, "video0", "145f", "0203")
    assert find_video_devices("145f", "9999", tmp_path) == []


def test_video_orbbec_is_ignored(tmp_path):
    _make_video(tmp_path, "video0", "2bc5", "0840")
    _make_video(tmp_path, "video1", "145f", "0203")
    assert find_video_devices("145f", "0203", tmp_path) == ["/dev/video1"]


def test_video_metadata_node_is_ignored(tmp_path):
    _make_video(tmp_path, "video0", "145f", "0203", index="1")
    _make_video(tmp_path, "video1", "145f", "0203", index="0")
    assert find_video_devices("145f", "0203", tmp_path) == ["/dev/video1"]


def test_video_missing_index_is_kept(tmp_path):
    _make_video(tmp_path, "video0", "145f", "0203")
    assert find_video_devices("145f", "0203", tmp_path) == ["/dev/video0"]


def test_video_non_video_names_are_ignored(tmp_path):
    _make_video(tmp_path, "ttyACM0", "145f", "0203")
    assert find_video_devices("145f", "0203", tmp_path) == []


def test_video_missing_sysfs_root_returns_empty(tmp_path):
    assert find_video_devices("145f", "0203", tmp_path / "no-such-dir") == []
