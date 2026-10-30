"""Tests for parts_image.crop_to_markers. Synthetic images only."""

from __future__ import annotations

import numpy

from muse_qc.parts_image import CROP_MARGIN_PX, crop_to_markers


def _markers(boxes):
    """Fake detect_table_markers output from corner boxes."""
    markers = {}
    for marker_id, corners in enumerate(boxes):
        markers[marker_id] = {"corners": corners}
    markers["ignored_ids"] = []
    return markers


def test_crop_size_is_corner_box_plus_margin():
    image = numpy.zeros((480, 640, 3), numpy.uint8)
    markers = _markers([
        [(100, 80), (140, 80), (140, 120), (100, 120)],
        [(500, 80), (540, 80), (540, 120), (500, 120)],
        [(100, 360), (140, 360), (140, 400), (100, 400)],
        [(500, 360), (540, 360), (540, 400), (500, 400)],
    ])
    cropped = crop_to_markers(image, markers)
    assert cropped.shape == (
        400 - 80 + 2 * CROP_MARGIN_PX,
        540 - 100 + 2 * CROP_MARGIN_PX,
        3,
    )


def test_crop_clamped_at_image_edge():
    image = numpy.zeros((480, 640, 3), numpy.uint8)
    markers = _markers([
        [(2, 3), (42, 3), (42, 43), (2, 43)],
        [(600, 3), (638, 3), (638, 43), (600, 43)],
        [(2, 440), (42, 440), (42, 478), (2, 478)],
        [(600, 440), (638, 440), (638, 478), (600, 478)],
    ])
    cropped = crop_to_markers(image, markers)
    assert cropped.shape == (480, 640, 3)
