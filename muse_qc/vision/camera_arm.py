"""Save one test image from the arm camera (read-only, no motion).

The arm camera is an Orbbec Gemini 305 on the gripper. It exposes
several /dev/videoN nodes: interface 1.0 holds depth/IR/Bayer,
interface 1.4 holds the colour stream and interface 1.6 a second
YUYV stream. The working colour node is the one with index 0 on
USB interface 4, found by USB ID (2bc5:0840) plus the interface
number, never by fixed /dev number. The Orbbec SDK is not used;
plain V4L2 through OpenCV is enough.

The frame is opened with the V4L2 backend as YUYV 640x480, warmed
up with 15 reads, retried up to 3 times (the first open sometimes
gives no frame), and returned unrotated: the image is upright.
No arm code is imported and nothing is moved. Run by the human only:
uv run python -m muse_qc.vision.camera_arm.
"""

from __future__ import annotations

import os
import sys
from datetime import datetime
from pathlib import Path

import cv2

ARM_CAM_USB_VID = "2bc5"
ARM_CAM_USB_PID = "0840"
ARM_CAM_INTERFACE = "04"
ARM_CAM_WIDTH = 640
ARM_CAM_HEIGHT = 480
ARM_CAM_FOURCC = "YUYV"
ARM_CAM_WARMUP_FRAMES = 15
ARM_CAM_ATTEMPTS = 3
ARM_CAM_READ_TIMEOUT_MS = 2000
ARM_CAM_MAX_CONSECUTIVE_WARMUP_FAILURES = 3


def _read_text(path: Path) -> str | None:
    """Return stripped lowercase file content, or None when unreadable."""
    try:
        return path.read_text(encoding="utf-8").strip().lower()
    except OSError:
        return None


def find_arm_color_node(
    sysfs_root: str | os.PathLike[str] = "/sys/class/video4linux",
) -> str | None:
    """Return the /dev/videoN path of the arm colour stream, or None.

    Scans <sysfs_root>/video* (sysfs_root is a parameter
    so tests can pass a fake folder). For each node the "device" link
    is followed to the USB interface directory: its bInterfaceNumber
    must be 04 and the parent USB device must carry idVendor 2bc5
    with idProduct 0840. The node index must be 0. Never opens
    a camera; only reads sysfs text files.
    """
    root = Path(sysfs_root)
    try:
        entries = sorted(root.iterdir())
    except OSError:
        return None
    for entry in entries:
        if not entry.name.startswith("video"):
            continue
        device_link = entry / "device"
        if not os.path.lexists(device_link):
            continue
        try:
            interface_dir = device_link.resolve()
        except OSError:
            continue
        if _read_text(interface_dir / "bInterfaceNumber") != ARM_CAM_INTERFACE:
            continue
        vid: str | None = None
        pid: str | None = None
        node: Path = interface_dir
        for _ in range(3):
            node = node.parent
            vid = _read_text(node / "idVendor")
            pid = _read_text(node / "idProduct")
            if vid is not None and pid is not None:
                break
        if vid != ARM_CAM_USB_VID or pid != ARM_CAM_USB_PID:
            continue
        if _read_text(entry / "index") != "0":
            continue
        return f"/dev/{entry.name}"
    return None


def open_arm_camera(
    node: str,
    attempts: int = ARM_CAM_ATTEMPTS,
    warmup_frames: int = ARM_CAM_WARMUP_FRAMES,
):
    """Read one unrotated BGR frame from the arm colour node, or None.

    Opens the node with the V4L2 backend as YUYV 640x480, asks for
    a 2 s read timeout (only when the installed OpenCV knows
    CAP_PROP_READ_TIMEOUT_MSEC), discards warm-up reads, then reads
    the returned frame. An attempt ends early after 3 consecutive
    failed warm-up reads, so a bad attempt never blocks for
    minutes. Up to <attempts> attempts are made because the first
    open sometimes gives no frame. The capture is always released.
    """
    read_timeout_prop = getattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC", None)
    for attempt in range(1, attempts + 1):
        capture = cv2.VideoCapture(node, cv2.CAP_V4L2)
        try:
            capture.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*ARM_CAM_FOURCC)
            )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, ARM_CAM_WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, ARM_CAM_HEIGHT)
            if read_timeout_prop is not None:
                capture.set(read_timeout_prop, ARM_CAM_READ_TIMEOUT_MS)
            aborted = False
            consecutive_failures = 0
            for _ in range(warmup_frames):
                ok_warm, _ = capture.read()
                if ok_warm:
                    consecutive_failures = 0
                    continue
                consecutive_failures += 1
                if (
                    consecutive_failures
                    >= ARM_CAM_MAX_CONSECUTIVE_WARMUP_FAILURES
                ):
                    aborted = True
                    break
            good = False
            if not aborted:
                ok, frame = capture.read()
                good = bool(ok) and frame is not None
            print(f"attempt {attempt} ok: {good}")
            if good:
                return frame
        finally:
            capture.release()
    return None


def main() -> int:
    """Find the arm colour node, grab one frame, save it as PNG."""
    node = find_arm_color_node()
    if node is None:
        print(
            "arm camera not found by USB ID"
            f" {ARM_CAM_USB_VID}:{ARM_CAM_USB_PID}"
            f" interface {ARM_CAM_INTERFACE}"
        )
        return 1
    print(f"node: {node}")
    frame = open_arm_camera(node)
    if frame is None:
        print(f"no frame from {node}")
        return 2
    print(f"frame shape: {frame.shape}")
    target_dir = Path("recordings")
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target_dir / f"armcam_{stamp}.png"
    if not cv2.imwrite(str(path), frame):
        print(f"failed to write image to {path}")
        return 3
    print(f"saved {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
