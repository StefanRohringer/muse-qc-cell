"""Capture checkerboard photos for table camera calibration (run by hand).

Terminal loop: "Enter = take photo, q = quit". Each photo is read
with open_table_camera() from muse_qc.camera_top (no new camera code
here) and checked with cv2.findChessboardCornersSB against
CHECKER_INNER_CORNERS from muse_qc.checkerboard. Photos with a found
board are saved to recordings/calibration/calib_<timestamp>.png, the
others are skipped. The running count of saved photos is printed.

OpenCV is headless, so no GUI windows are used. Run by hand only
(it opens the real table camera):

    uv run python -m muse_qc.capture_calibration
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import cv2
import numpy

from muse_qc.camera_top import open_table_camera
from muse_qc.checkerboard import CHECKER_INNER_CORNERS

CALIBRATION_DIR = Path("recordings/calibration")

PROMPT = "Enter = take photo, q = quit: "


def find_board(frame: numpy.ndarray) -> tuple[bool, numpy.ndarray | None]:
    """Check frame for the calibration checkerboard.

    Returns (True, corners) when the board is found, else (False,
    None). The frame itself is never modified.
    """
    if frame is None:
        return False, None
    gray = frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray,
        CHECKER_INNER_CORNERS,
        cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found or corners is None:
        return False, None
    return True, corners


def save_photo(frame: numpy.ndarray, target_dir: Path = CALIBRATION_DIR) -> Path:
    """Save frame as calib_<timestamp>.png in target_dir, return the path."""
    target_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = target_dir / f"calib_{stamp}.png"
    if not cv2.imwrite(str(path), frame):
        raise RuntimeError(f"failed to write image to {path}")
    return path


def capture_session(
    open_fn=open_table_camera,
    read_input=input,
    target_dir: Path = CALIBRATION_DIR,
) -> int:
    """Run the photo loop, return the number of saved photos.

    open_fn takes no arguments and returns a frame (a (frame,
    settings) pair from open_table_camera is unwrapped). read_input
    takes the prompt and returns one line. Camera errors are reported
    and the loop continues.
    """
    saved = 0
    while True:
        line = read_input(PROMPT).strip().lower()
        if line == "q":
            break
        try:
            result = open_fn()
        except RuntimeError as error:
            print(f"camera error: {error}")
            continue
        frame = result[0] if isinstance(result, tuple) else result
        found, _ = find_board(frame)
        if found:
            path = save_photo(frame, target_dir)
            saved += 1
            print(f"board found, saved {path} (count: {saved})")
        else:
            print("board NOT found, photo skipped")
    return saved


def main() -> None:
    """Run the hand-operated photo loop and print the saved count."""
    print("Checkerboard photo capture. Board: 10x7 squares of 25 mm.")
    saved = capture_session()
    print(f"saved {saved} calibration photos")


if __name__ == "__main__":
    main()
