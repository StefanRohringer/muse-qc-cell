"""Dark-slot long-axis cue for the lever direction, vision only.

The small dark slot is an elongated rectangle whose long axis
runs across the levers, so its own orientation does not move
with the part shadow inside the diff contour. Inside the
filled contour pixels with V at most SLOT_AXIS_V_MAX are split
into 8-connected components; components with area
SLOT_AXIS_MIN_AREA to SLOT_AXIS_MAX_AREA and elongation
(sqrt of the eigenvalue ratio of the second moments) from
SLOT_AXIS_MIN_ELONG to SLOT_AXIS_MAX_ELONG pass, and the one
nearest the filled-contour centroid is the slot. Its long axis
(second moments, image frame) plus 90 deg is the lever
direction. The long dark gap of an open lever (elongation
about 6) is above SLOT_AXIS_MAX_ELONG and never passes.
This module moves nothing.
"""

from __future__ import annotations

import math

import cv2
import numpy

# Dark slot pixels in OpenCV HSV: V at most this value. Wood is
# far brighter, so only the filled part contour is read.
SLOT_AXIS_V_MAX = 60
# Component area in pixels accepted as the slot.
SLOT_AXIS_MIN_AREA = 5
SLOT_AXIS_MAX_AREA = 40
# Elongation (sqrt of the ratio of the two second-moment
# eigenvalues) accepted as the slot.
SLOT_AXIS_MIN_ELONG = 1.6
SLOT_AXIS_MAX_ELONG = 4.5
# Outline direction (or outline + 90) within this distance of
# the slot-axis estimate: use the outline direction (it is more
# precise); otherwise use the estimate itself.
SLOT_AXIS_AGREE_DEG = 15.0


def _angle_mod_180(deg):
    """Wrap an angle in degrees into [0, 180)."""
    wrapped = math.fmod(float(deg), 180.0)
    if wrapped < 0.0:
        wrapped += 180.0
    if wrapped >= 180.0:
        wrapped = 0.0
    return wrapped


def _component_elongation(xs, ys):
    """Elongation and long-axis angle of pixel coords.

    Returns (elongation, axis_deg) with axis_deg in [0, 180)
    from the second moments in the image frame (x right, y
    down); (None, None) when a direction cannot be read.
    """
    count = len(xs)
    if count < 2:
        return None, None
    xs = numpy.asarray(xs, dtype=numpy.float64)
    ys = numpy.asarray(ys, dtype=numpy.float64)
    mean_x, mean_y = float(xs.mean()), float(ys.mean())
    var_x = float(((xs - mean_x) ** 2).mean())
    var_y = float(((ys - mean_y) ** 2).mean())
    cov = float(((xs - mean_x) * (ys - mean_y)).mean())
    trace = var_x + var_y
    if trace <= 0.0:
        return None, None
    disc = math.sqrt(max(0.0, (var_x - var_y) ** 2 + 4.0 * cov * cov))
    large, small = (trace + disc) / 2.0, (trace - disc) / 2.0
    if small <= 1e-12:
        return float("inf"), _angle_mod_180(
            math.degrees(math.atan2(2.0 * cov, var_x - var_y)) / 2.0)
    elongation = math.sqrt(large / small)
    axis_deg = _angle_mod_180(
        math.degrees(math.atan2(2.0 * cov, var_x - var_y)) / 2.0)
    return elongation, axis_deg


def estimate(crop_bgr, filled):
    """Lever direction from the slot long axis, image frame.

    crop_bgr is the BGR crop, filled the boolean filled-contour
    mask (not eroded) in crop coordinates. Returns the lever
    estimate (slot long axis + 90 deg) in [0, 180) or None
    when no component passes.
    """
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    dark = ((hsv[:, :, 2] <= SLOT_AXIS_V_MAX) & filled).astype(numpy.uint8)
    if not dark.any():
        return None
    count, labels, _, _ = cv2.connectedComponentsWithStats(
        dark, connectivity=8)
    filled_ys, filled_xs = numpy.nonzero(filled)
    center = (float(filled_xs.mean()), float(filled_ys.mean()))
    best, best_dist = None, float("inf")
    for label in range(1, count):
        ys, xs = numpy.nonzero(labels == label)
        area = len(xs)
        if area < SLOT_AXIS_MIN_AREA or area > SLOT_AXIS_MAX_AREA:
            continue
        elongation, axis_deg = _component_elongation(xs, ys)
        if elongation is None or axis_deg is None:
            continue
        if math.isinf(elongation):
            continue
        if not SLOT_AXIS_MIN_ELONG <= elongation <= SLOT_AXIS_MAX_ELONG:
            continue
        dist = math.dist(center, (float(xs.mean()), float(ys.mean())))
        if dist < best_dist:
            best_dist = dist
            best = axis_deg
    if best is None:
        return None
    return _angle_mod_180(best + 90.0)
