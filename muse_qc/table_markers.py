"""Shared constants for the four printed table markers.

The detector imports these constants too, so both the
generator (muse_qc.make_markers) and the detector agree on the
dictionary, the IDs and the printed size.

The markers use the 4x4 ArUco dictionary with 50 entries. Each marker
is printed as a black square with an edge of TABLE_MARKER_SIZE_MM,
including the black border cells.
"""

from __future__ import annotations

import cv2
import numpy

TABLE_MARKER_DICT = cv2.aruco.DICT_4X4_50
TABLE_MARKER_IDS = (0, 1, 2, 3)
TABLE_MARKER_SIZE_MM = 40.0
MARKER_CELLS = 6

# Table frame in millimetres, measured by hand (centre to centre of the
# black marker squares): origin at the centre of marker 0, +x towards
# marker 1, +y towards marker 2 (away from the arm). IDs 2 to 3 are
# 977 mm apart, IDs 0 to 2 are 531 mm apart, the diagonal from ID 2 to
# ID 1 is 1110 mm. The ID 0 to ID 1 distance was not measurable (the
# arm is in the way) and is assumed from the rectangle, which the
# diagonal confirms (sqrt(977^2 + 531^2) = 1112 mm).
TABLE_MARKER_LAYOUT_MM = {
    0: (0.0, 0.0),
    1: (977.0, 0.0),
    2: (0.0, 531.0),
    3: (977.0, 531.0),
}


def get_dictionary():
    """Return the predefined ArUco dictionary used for the table markers."""
    return cv2.aruco.getPredefinedDictionary(TABLE_MARKER_DICT)


def marker_cells(marker_id: int) -> numpy.ndarray:
    """Return the 6x6 cell pattern of one table marker (1 is black).

    The pattern is built with cv2.aruco.generateImageMarker at 6 pixels,
    so each pixel is exactly one cell: the outer ring is the black
    border and the inner 4x4 cells hold the marker data.
    """
    if marker_id not in TABLE_MARKER_IDS:
        raise ValueError(
            f"unknown table marker ID {marker_id},"
            f" expected one of {list(TABLE_MARKER_IDS)}"
        )
    image = cv2.aruco.generateImageMarker(
        get_dictionary(), marker_id, MARKER_CELLS
    )
    return (image == 0).astype(numpy.uint8)
