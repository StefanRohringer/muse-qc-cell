"""Calibrate the table camera from checkerboard photos (run by hand).

Loads every PNG in recordings/calibration/, finds the inner corners
with cv2.findChessboardCornersSB (CHECKER_INNER_CORNERS from
muse_qc.checkerboard), pairs them with object points built from
CHECKER_SQUARE_MM, and runs cv2.calibrateCamera. At least 10 boards
are required and all images must share one size, otherwise the run is
refused with a clear error. Prints the images used, the RMS
reprojection error in px and the distortion coefficients, then writes
calibration/table_camera.json (camera_matrix, dist_coeffs,
image_size, rms_px, images_used, created).

load_calibration() and undistort() below are reused later by the
marker and coordinate mapping; detect_markers.py and table_coords.py
are unchanged by this module. Run by hand only (offline, no camera):

    uv run python -m muse_qc.calibrate_camera
"""

from __future__ import annotations

import json
from datetime import date
from pathlib import Path

import cv2
import numpy

from muse_qc.checkerboard import CHECKER_INNER_CORNERS, CHECKER_SQUARE_MM

MIN_BOARDS = 10

CALIBRATION_DIR = Path("recordings/calibration")
OUTPUT_PATH = Path("calibration/table_camera.json")

_SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


def board_object_points() -> numpy.ndarray:
    """Return the 3D object points of the inner corners in mm (z = 0)."""
    columns, rows = CHECKER_INNER_CORNERS
    points = numpy.zeros((columns * rows, 3), dtype=numpy.float32)
    grid = numpy.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] = grid * numpy.float32(CHECKER_SQUARE_MM)
    return points


def find_corners(image: numpy.ndarray) -> numpy.ndarray | None:
    """Return the refined inner corners, or None when no board is found."""
    if image is None:
        return None
    gray = image if len(image.shape) == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray,
        CHECKER_INNER_CORNERS,
        cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not found or corners is None:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX_CRITERIA)


def collect_points(paths: list[Path]) -> tuple[list, list, tuple[int, int], list[str]]:
    """Map every readable board image to object and image points.

    Returns (object_points, image_points, image_size, names) where
    image_size is (width, height) and names holds the used file names.
    Images without a detectable board are skipped. Raises
    RuntimeError when no usable board is found or when the image sizes
    differ.
    """
    object_template = board_object_points()
    object_points: list = []
    image_points: list = []
    names: list[str] = []
    image_size: tuple[int, int] | None = None
    for path in sorted(paths):
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        height, width = image.shape[0], image.shape[1]
        corners = find_corners(image)
        if corners is None:
            continue
        if image_size is None:
            image_size = (width, height)
        elif (width, height) != image_size:
            raise RuntimeError(
                f"image sizes differ: {path} is {width}x{height},"
                f" expected {image_size[0]}x{image_size[1]}"
            )
        object_points.append(object_template)
        image_points.append(corners)
        names.append(path.name)
    if not image_points or image_size is None:
        raise RuntimeError(f"no checkerboard found in {CALIBRATION_DIR}")
    return object_points, image_points, image_size, names


def calibrate_from_points(
    object_points: list,
    image_points: list,
    image_size: tuple[int, int],
) -> tuple[float, numpy.ndarray, numpy.ndarray]:
    """Run cv2.calibrateCamera, return (rms_px, camera_matrix, dist)."""
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    return float(rms), camera_matrix, dist_coeffs


def calibrate_from_files(
    paths: list[Path],
    target_dir: Path = CALIBRATION_DIR,
) -> dict:
    """Calibrate from PNG files, return the calibration record dict.

    Raises RuntimeError when fewer than MIN_BOARDS boards are found.
    """
    object_points, image_points, image_size, names = collect_points(paths)
    if len(image_points) < MIN_BOARDS:
        raise RuntimeError(
            f"fewer than {MIN_BOARDS} checkerboards found"
            f" ({len(image_points)} in {target_dir}), take more photos"
        )
    rms, camera_matrix, dist_coeffs = calibrate_from_points(
        object_points, image_points, image_size
    )
    return {
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.tolist(),
        "image_size": [image_size[0], image_size[1]],
        "rms_px": rms,
        "images_used": names,
        "created": date.today().isoformat(),
    }


def save_calibration(calibration: dict, path: Path = OUTPUT_PATH) -> Path:
    """Write the calibration record as JSON, return the path."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(calibration, indent=2) + "\n", encoding="utf-8")
    return path


def load_calibration(path: str | Path = OUTPUT_PATH) -> dict:
    """Load a calibration JSON file, with matrices as numpy arrays."""
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    return {
        "camera_matrix": numpy.array(record["camera_matrix"], dtype=numpy.float64),
        "dist_coeffs": numpy.array(record["dist_coeffs"], dtype=numpy.float64),
        "image_size": (record["image_size"][0], record["image_size"][1]),
        "rms_px": float(record["rms_px"]),
        "images_used": list(record["images_used"]),
        "created": str(record["created"]),
    }


def undistort(frame: numpy.ndarray, calibration: dict) -> numpy.ndarray:
    """Return the distortion-corrected frame for a calibration record.

    Accepts records from calibrate_from_files() (plain lists) and from
    load_calibration() (numpy arrays).
    """
    camera_matrix = numpy.array(calibration["camera_matrix"], dtype=numpy.float64)
    dist_coeffs = numpy.array(calibration["dist_coeffs"], dtype=numpy.float64)
    return cv2.undistort(frame, camera_matrix, dist_coeffs)


def main() -> None:
    """Calibrate from recordings/calibration/ and write the JSON file."""
    paths = sorted(CALIBRATION_DIR.glob("*.png"))
    if not paths:
        raise RuntimeError(f"no PNG images in {CALIBRATION_DIR}")
    calibration = calibrate_from_files(paths)
    path = save_calibration(calibration)
    print(f"images used: {len(calibration['images_used'])}")
    for name in calibration["images_used"]:
        print(f"  {name}")
    print(f"RMS reprojection error: {calibration['rms_px']:.3f} px")
    print(f"distortion coefficients: {calibration['dist_coeffs']}")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
