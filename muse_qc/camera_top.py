"""Save one test image from the table camera (read-only, no motion).

The table camera is mounted upside down above the table, so every frame
is rotated by 180 degrees (TABLE_CAM_ROTATE_180 = True).

The camera is found by USB ID, never by fixed index. The camera keeps
its last settings even after being unplugged, so values set earlier by
hand or by another program would carry over; every start applies the
fixed TABLE_CAM_* values (manual exposure, fixed gain, brightness,
contrast and white balance) before the warm-up frames, which makes the
result reproducible. Each value is read back after setting and printed,
so the real values are always visible. No arm code is imported and
nothing is moved. Run by the human only:
uv run python -m muse_qc.camera_top.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2

from muse_qc.usb_devices import (
    TABLE_CAM_USB_PID,
    TABLE_CAM_USB_VID,
    find_video_devices,
)

TABLE_CAM_ROTATE_180 = True
TABLE_CAM_WIDTH = 1920
TABLE_CAM_HEIGHT = 1080
TABLE_CAM_FOURCC = "MJPG"
TABLE_CAM_WARMUP_FRAMES = 45

TABLE_CAM_AUTO_EXPOSURE = 1
TABLE_CAM_EXPOSURE = 450
TABLE_CAM_GAIN = 0
TABLE_CAM_BRIGHTNESS = 160
TABLE_CAM_CONTRAST = 150
TABLE_CAM_AUTO_WB = 0
TABLE_CAM_WB_TEMPERATURE = 3200

TABLE_CAM_SETTINGS = (
    ("auto_exposure", cv2.CAP_PROP_AUTO_EXPOSURE, TABLE_CAM_AUTO_EXPOSURE),
    ("exposure", cv2.CAP_PROP_EXPOSURE, TABLE_CAM_EXPOSURE),
    ("gain", cv2.CAP_PROP_GAIN, TABLE_CAM_GAIN),
    ("brightness", cv2.CAP_PROP_BRIGHTNESS, TABLE_CAM_BRIGHTNESS),
    ("contrast", cv2.CAP_PROP_CONTRAST, TABLE_CAM_CONTRAST),
    ("auto_wb", cv2.CAP_PROP_AUTO_WB, TABLE_CAM_AUTO_WB),
    (
        "wb_temperature",
        cv2.CAP_PROP_WB_TEMPERATURE,
        TABLE_CAM_WB_TEMPERATURE,
    ),
)


def open_table_camera():
    """Find the table camera by USB ID, read one full-resolution frame.

    Opens the single matching device with the V4L2 backend, requests
    MJPG at 1920x1080, then applies the fixed TABLE_CAM_* settings
    (the values are passed to the matching V4L2 controls unchanged),
    discards warm-up frames (letting the camera settle after the
    settings are applied), then reads and returns
    the frame rotated by 180 degrees together with the read-back
    setting values as a (frame, settings) pair.

    Raises a clear error when zero or several capture nodes match, when
    no frame can be read, or when the real frame size is not 1920x1080;
    never falls back to a fixed index or a lower resolution. The camera
    is always released.
    """
    devices = find_video_devices(TABLE_CAM_USB_VID, TABLE_CAM_USB_PID)
    if not devices:
        raise RuntimeError(
            "table camera not found by USB ID"
            f" {TABLE_CAM_USB_VID}:{TABLE_CAM_USB_PID}"
        )
    if len(devices) > 1:
        raise RuntimeError(
            "several table cameras found by USB ID"
            f" {TABLE_CAM_USB_VID}:{TABLE_CAM_USB_PID}:"
            f" {', '.join(devices)}"
        )
    capture = cv2.VideoCapture(devices[0], cv2.CAP_V4L2)
    try:
        capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*TABLE_CAM_FOURCC)
        )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, TABLE_CAM_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, TABLE_CAM_HEIGHT)
        for _, prop, value in TABLE_CAM_SETTINGS:
            capture.set(prop, value)
        for _ in range(TABLE_CAM_WARMUP_FRAMES):
            capture.read()
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read a frame from {devices[0]}")
        height, width = frame.shape[0], frame.shape[1]
        if width != TABLE_CAM_WIDTH or height != TABLE_CAM_HEIGHT:
            raise RuntimeError(
                f"unexpected frame size {width}x{height} from {devices[0]},"
                f" expected {TABLE_CAM_WIDTH}x{TABLE_CAM_HEIGHT}"
            )
        if TABLE_CAM_ROTATE_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        read_back = {
            name: capture.get(prop) for name, prop, _ in TABLE_CAM_SETTINGS
        }
        return frame, read_back
    finally:
        capture.release()


def save_frame(frame, recordings_dir: str | Path = "recordings") -> Path:
    """Save one frame as PNG, return the file path."""
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target_dir / f"table_{stamp}.png"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"failed to write image to {path}")
    return path


def main() -> None:
    """Read one frame from the table camera and save it as PNG."""
    frame, settings = open_table_camera()
    path = save_frame(frame)
    height, width = frame.shape[0], frame.shape[1]
    shown = " ".join(f"{name}={value:g}" for name, value in settings.items())
    print(f"{path} {width}x{height} {shown}")


if __name__ == "__main__":
    main()
