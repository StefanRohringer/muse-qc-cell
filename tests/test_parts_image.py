"""Tests for parts_image cropping. Synthetic images only."""

from __future__ import annotations

import numpy

from muse_qc.vision.parts_image import (
    CROP_MARGIN_PX,
    crop_to_box,
    crop_to_markers,
    marker_crop_box,
)


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


def test_marker_crop_box_matches_crop():
    image = numpy.zeros((480, 640, 3), numpy.uint8)
    markers = _markers([
        [(100, 80), (140, 80), (140, 120), (100, 120)],
        [(500, 80), (540, 80), (540, 120), (500, 120)],
        [(100, 360), (140, 360), (140, 400), (100, 400)],
        [(500, 360), (540, 360), (540, 400), (500, 400)],
    ])
    box = marker_crop_box(markers, 640, 480)
    assert box == [100 - CROP_MARGIN_PX, 80 - CROP_MARGIN_PX,
                   540 + CROP_MARGIN_PX, 400 + CROP_MARGIN_PX]
    assert crop_to_markers(image, markers).shape == \
        crop_to_box(image, box).shape


def test_crop_to_box_clipped_to_image_size():
    image = numpy.zeros((480, 640, 3), numpy.uint8)
    cropped = crop_to_box(image, [-50, -40, 700, 520])
    assert cropped.shape == (480, 640, 3)
    cropped = crop_to_box(image, [100, 80, 540, 400])
    assert cropped.shape == (400 - 80, 540 - 100, 3)
