"""Find serial ports by USB vendor and product ID (read-only).

Looks up /dev/ttyACM* and /dev/ttyUSB* entries through sysfs
(/sys/class/tty/<name>/device/.. idVendor, idProduct) without ever
opening a serial port. IDs are compared as lowercase hex strings.
"""

from __future__ import annotations

import os
from pathlib import Path

ARM_USB_VID = "2e88"
ARM_USB_PID = "4603"

TABLE_CAM_USB_VID = "145f"
TABLE_CAM_USB_PID = "0203"

_TTY_PREFIXES = ("ttyACM", "ttyUSB")


def _read_id(path: Path) -> str | None:
    """Return stripped lowercase file content, or None when unreadable."""
    try:
        return path.read_text(encoding="utf-8").strip().lower()
    except OSError:
        return None


def _ids_of(tty_dir: Path, device_dir: Path) -> tuple[str | None, str | None]:
    """Return (idVendor, idProduct) for one tty entry.

    Checks the resolved device directory and its parents (the USB
    device holding idVendor/idProduct sits above the tty device link),
    plus the tty directory itself for minimal fake-sysfs layouts.
    First directory holding both files wins.
    """
    candidates: list[Path] = []
    try:
        resolved = device_dir.resolve()
    except OSError:
        resolved = device_dir
    node: Path | None = resolved
    for _ in range(4):
        if node is None:
            break
        candidates.append(node)
        node = node.parent if node.parent != node else None
    candidates.append(device_dir)
    candidates.append(tty_dir)
    for candidate in candidates:
        vid = _read_id(candidate / "idVendor")
        pid = _read_id(candidate / "idProduct")
        if vid is not None and pid is not None:
            return vid, pid
    return None, None


def find_serial_ports(
    vid: str, pid: str, sysfs_root: str | os.PathLike[str] = "/sys/class/tty"
) -> list[str]:
    """Return /dev paths whose USB IDs match vid and pid.

    Only names starting with ttyACM or ttyUSB are considered. The
    sysfs root is a parameter so tests can pass a fake folder.
    Never opens a serial port; only reads idVendor/idProduct files.
    """
    want_vid = vid.strip().lower()
    want_pid = pid.strip().lower()
    root = Path(sysfs_root)
    try:
        entries = sorted(root.iterdir())
    except OSError:
        return []
    found: list[str] = []
    for entry in entries:
        if not entry.name.startswith(_TTY_PREFIXES):
            continue
        device_dir = entry / "device"
        if not os.path.lexists(device_dir):
            continue
        got_vid, got_pid = _ids_of(entry, device_dir)
        if got_vid == want_vid and got_pid == want_pid:
            found.append(f"/dev/{entry.name}")
    return found


def _is_capture_node(entry: Path) -> bool:
    """Return True for capture nodes of a video device entry.

    Keeps entries whose "index" file holds 0 (the capture node of a
    camera exposing several /dev/videoN nodes). When the file is
    missing, the entry is kept.
    """
    try:
        text = (entry / "index").read_text(encoding="utf-8").strip()
    except OSError:
        return True
    return text == "0"


def find_video_devices(
    vid: str,
    pid: str,
    sysfs_root: str | os.PathLike[str] = "/sys/class/video4linux",
) -> list[str]:
    """Return /dev/videoN paths whose USB IDs match vid and pid.

    Only names starting with "video" are considered, and only capture
    nodes (see _is_capture_node) are kept. The sysfs root is a
    parameter so tests can pass a fake folder. Never opens a camera;
    only reads idVendor/idProduct (plus index) files.
    """
    want_vid = vid.strip().lower()
    want_pid = pid.strip().lower()
    root = Path(sysfs_root)
    try:
        entries = sorted(root.iterdir())
    except OSError:
        return []
    found: list[str] = []
    for entry in entries:
        if not entry.name.startswith("video"):
            continue
        device_dir = entry / "device"
        if not os.path.lexists(device_dir):
            continue
        if not _is_capture_node(entry):
            continue
        got_vid, got_pid = _ids_of(entry, device_dir)
        if got_vid == want_vid and got_pid == want_pid:
            found.append(f"/dev/{entry.name}")
    return found
