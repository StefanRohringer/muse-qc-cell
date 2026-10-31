"""Last-good table mapping for frames with covered markers.

The table camera is fixed, so the pixel-to-millimetre mapping does
not change between images. Every time find_parts builds the table
mapping from all 4 markers it is saved here (homography, marker
ids, px/mm scale, local time stamp) under
recordings/table_mapping_last.json. When the arm at the inspection
station covers a marker, the saved mapping is reused instead, as
long as it is at most MAPPING_MAX_AGE_H hours old. recordings/
stays out of git.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

import numpy

from muse_qc.vision.table_markers import TABLE_MARKER_IDS

MAPPING_PATH = "recordings/table_mapping_last.json"
MAPPING_MAX_AGE_H = 12
_STAMP_FORMAT = "%Y%m%d_%H%M%S"


def save_last_mapping(homography, markers, scale, path=MAPPING_PATH,
                      now=None) -> str:
    """Save the mapping built from all markers; return the stamp.

    homography is the 3x3 pixel-to-mm matrix, markers the detected
    marker dict (only table marker ids are kept: the real
    detect_table_markers output carries extra non-marker keys like
    "ignored_ids"), scale the px/mm factor.
    now is an injectable zero-argument clock returning a datetime
    (local time, like every other recordings file name).
    """
    ticks = now if now is not None else datetime.now
    stamp = ticks().strftime(_STAMP_FORMAT)
    payload = {
        "homography": [[float(value) for value in row]
                       for row in homography],
        "marker_ids": sorted(key for key in markers
                             if key in TABLE_MARKER_IDS),
        "px_per_mm": float(scale),
        "stamp": stamp,
    }
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload), encoding="utf-8")
    return stamp


def load_last_mapping(path=MAPPING_PATH, now=None):
    """Load the saved mapping, or None when unusable.

    Returns (homography, px_per_mm, stamp). None when the file is
    missing or unreadable, when it holds no full 4-marker mapping,
    or when its stamp is older than MAPPING_MAX_AGE_H hours.
    now is an injectable zero-argument clock returning a datetime.
    """
    ticks = now if now is not None else datetime.now
    try:
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        stamp = datetime.strptime(payload["stamp"], _STAMP_FORMAT)
        matrix = numpy.array(payload["homography"], dtype=float)
        scale = float(payload["px_per_mm"])
        ids = [int(value) for value in payload["marker_ids"]]
    except (OSError, ValueError, TypeError, KeyError):
        return None
    if matrix.shape != (3, 3) or not numpy.isfinite(matrix).all():
        return None
    if any(marker_id not in ids for marker_id in TABLE_MARKER_IDS):
        return None
    age_s = (ticks() - stamp).total_seconds()
    if not 0 <= age_s <= MAPPING_MAX_AGE_H * 3600:
        return None
    return matrix, scale, payload["stamp"]
