"""Lever direction of a connector part on the table camera image.

The part is a grey connector (about 17 x 20.5 mm) with 3 orange
levers lying side by side on top. The levers can look brownish
under the lamp. The part outline alone gives a precise angle but
only modulo 90 deg (the part is nearly square), so the lever
stripes pick which of the two outline directions is the lever
direction. The final angle always comes from the outline.

Only pixels INSIDE the part contour are used: the wooden table
itself falls into the lever HSV range, so any wood pixel outside
the contour must not influence the direction. The stripe cue is
the structure tensor of the grayscale gradients (Sobel) inside
the eroded part region, weighted where the pixel is lever-like
or next to one (dilated lever mask, covering the dark gaps
between merged bars). The dominant gradient runs across the
bars, so the lever direction is the tensor direction + 90 deg,
with strength = coherence ((l1 - l2) / (l1 + l2)). When the
stripes are too weak, a small dark slot (V at most SLOT_V_MAX)
at one end of the levers picks instead: the direction from the
part centre to the slot centroid is parallel to the levers
(modulo 180) and chooses the same outline vs outline + 90.

Angles are measured in image pixels (x right, y down) and then
mapped into the table frame with to_table, so the result is
independent of camera rotation. lever_deg is in [0, 180):
0 means bars parallel to table +x, 90 parallel to table +y.
A part rotated by 180 deg looks the same, so modulo 180 is all
the arm needs. Vision only: this module moves nothing.
"""

from __future__ import annotations

import math

import cv2
import numpy

# Orange/brown lever pixels in OpenCV HSV (H 0..179). S_MIN 110
# and V_MAX 165 keep bright wood (S about 66-110, V about
# 170-227) out of the mask while brownish levers (S about
# 110-215, V about 90-160) still match; the grey body (low
# saturation) never matches.
LEVER_H_MIN = 3
LEVER_H_MAX = 35
LEVER_S_MIN = 110
LEVER_S_MAX = 255
LEVER_V_MIN = 50
LEVER_V_MAX = 165
# Pixels around the blob bbox included in the crop, so slightly
# oversized blobs still show the whole part.
CROP_MARGIN_PX = 4
# Erosion of the filled-contour region mask: gradients are only
# read this far inside the part, so no wood pixel outside the
# contour influences the direction.
REGION_ERODE_PX = 2
# Dilation radius of the lever mask for the tensor weight: covers
# the dark gap lines between bars that touch in the HSV mask.
WEIGHT_DILATE_PX = 2
# Fewer lever pixels than this inside the region: no cue to read.
MIN_LEVER_PIXELS = 40
# Stripe coherence ((l1 - l2) / (l1 + l2)) at or above which the
# stripe cue picks outline vs outline + 90. Only used for that
# choice; the final angle always comes from the outline.
MIN_STRENGTH = 0.3
# Outline aspect ratio (long side over short side) at or above
# which the long side is a usable fallback direction.
FALLBACK_ASPECT_MIN = 1.1
# Dark slot pixels in OpenCV HSV: V at most SLOT_V_MAX. Wood is
# far brighter, so the filled part contour (not eroded) is read.
SLOT_V_MAX = 70
# Fewer dark pixels than this inside the filled contour: no slot.
MIN_SLOT_PIXELS = 5
# Slot centroid closer than this to the contour centroid: no slot
# direction (centred dark area, not a slot at one end).
MIN_SLOT_OFFSET_PX = 5


def _angle_mod_180(deg):
    """Wrap an angle in degrees into [0, 180)."""
    wrapped = math.fmod(float(deg), 180.0)
    if wrapped < 0.0:
        wrapped += 180.0
    if wrapped >= 180.0:
        wrapped = 0.0
    return wrapped


def _angle_distance_mod_180(first, second):
    """Smallest distance of two directions modulo 180 deg."""
    return abs((float(first) - float(second) + 90.0) % 180.0 - 90.0)


def _long_side_deg(points):
    """Direction of the longest edge of a polygon in [0, 180).

    Points are (x, y) image pixels (x right, y down); the angle
    comes from atan2(dy, dx) of the longest edge.
    """
    best_len, best_deg = -1.0, 0.0
    count = len(points)
    for index in range(count):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % count]
        length = math.dist((x0, y0), (x1, y1))
        if length > best_len:
            best_len = length
            best_deg = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return _angle_mod_180(best_deg)


def _outline_direction(contour):
    """Long-side image direction and aspect ratio of a blob contour."""
    rect = cv2.minAreaRect(contour)
    (_, _), (width, height), _ = rect
    box = cv2.boxPoints(rect)
    long_deg = _long_side_deg([(float(p[0]), float(p[1])) for p in box])
    short_side = min(float(width), float(height))
    if short_side <= 0.0:
        return long_deg, 0.0
    return long_deg, float(max(float(width), float(height))) / short_side


def _lever_mask(crop_bgr):
    """Boolean mask of the orange/brown lever pixels in a BGR crop."""
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    lower = (LEVER_H_MIN, LEVER_S_MIN, LEVER_V_MIN)
    upper = (LEVER_H_MAX, LEVER_S_MAX, LEVER_V_MAX)
    return cv2.inRange(hsv, lower, upper) > 0


def _stripe_direction(gray, weight):
    """Lever direction from the gradient structure tensor.

    gray is the float64 crop gradient source, weight the boolean
    pixels to read (lever-like or gap pixels inside the part).
    The dominant gradient runs across the bars, so the lever
    direction is that direction + 90 deg. Returns (angle,
    coherence) with coherence (l1 - l2) / (l1 + l2) in 0..1;
    (None, 0.0) when no gradient energy is weighted.
    """
    grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    weighted = weight.astype(numpy.float64)
    sum_xx = float((weighted * grad_x * grad_x).sum())
    sum_yy = float((weighted * grad_y * grad_y).sum())
    sum_xy = float((weighted * grad_x * grad_y).sum())
    trace = sum_xx + sum_yy
    if trace <= 0.0:
        return None, 0.0
    disc = math.sqrt(max(0.0, (sum_xx - sum_yy) ** 2
                         + 4.0 * sum_xy * sum_xy))
    large, small = (trace + disc) / 2.0, (trace - disc) / 2.0
    coherence = (large - small) / (large + small)
    grad_deg = _angle_mod_180(
        math.degrees(math.atan2(2.0 * sum_xy, sum_xx - sum_yy)) / 2.0)
    return _angle_mod_180(grad_deg + 90.0), min(1.0, max(0.0, coherence))


def _slot_direction(crop_bgr, filled):
    """Direction from the part centre to the dark slot, image frame.

    crop_bgr is the BGR crop, filled the boolean filled-contour
    mask (not eroded) in crop coordinates. Pixels with V at most
    SLOT_V_MAX inside the filled contour are the slot; fewer than
    MIN_SLOT_PIXELS or a slot centroid closer than
    MIN_SLOT_OFFSET_PX to the filled centroid give no direction.
    Returns the centre-to-slot angle in [0, 180) or None.
    """
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    dark = (hsv[:, :, 2] <= SLOT_V_MAX) & filled
    count = int(dark.sum())
    if count < MIN_SLOT_PIXELS:
        return None
    filled_ys, filled_xs = numpy.nonzero(filled)
    slot_ys, slot_xs = numpy.nonzero(dark)
    center = (float(filled_xs.mean()), float(filled_ys.mean()))
    slot = (float(slot_xs.mean()), float(slot_ys.mean()))
    if math.dist(center, slot) < MIN_SLOT_OFFSET_PX:
        return None
    return _angle_mod_180(math.degrees(
        math.atan2(slot[1] - center[1], slot[0] - center[0])))


def _filled_mask(contour, x0, y0, shape):
    """Filled contour in crop coordinates (not eroded), or None."""
    height, width = shape
    mask = numpy.zeros((height, width), dtype=numpy.uint8)
    shifted = contour.copy().astype(numpy.float64)
    shifted[:, :, 0] -= float(x0)
    shifted[:, :, 1] -= float(y0)
    cv2.fillPoly(mask, [shifted.astype(numpy.int32)], 255)
    if not mask.any():
        return None
    return mask > 0


def _to_table_deg(image_deg, center, to_table):
    """Map an image direction at a pixel centre into table degrees."""
    rad = math.radians(float(image_deg))
    unit = (math.cos(rad), math.sin(rad))
    origin = (float(center[0]), float(center[1]))
    tip = (origin[0] + unit[0], origin[1] + unit[1])
    table_origin = to_table(origin)
    table_tip = to_table(tip)
    delta_x = float(table_tip[0]) - float(table_origin[0])
    delta_y = float(table_tip[1]) - float(table_origin[1])
    if delta_x == 0.0 and delta_y == 0.0:
        return None
    return _angle_mod_180(math.degrees(math.atan2(delta_y, delta_x)))


def _region_mask(contour, x0, y0, shape):
    """Filled diff contour in crop coordinates, eroded by px.

    shape is the (height, width) crop shape. Returns None when
    nothing of the contour lands inside the crop.
    """
    height, width = shape
    mask = numpy.zeros((height, width), dtype=numpy.uint8)
    shifted = contour.copy().astype(numpy.float64)
    shifted[:, :, 0] -= float(x0)
    shifted[:, :, 1] -= float(y0)
    cv2.fillPoly(mask, [shifted.astype(numpy.int32)], 255)
    if not mask.any():
        return None
    kernel = numpy.ones((3, 3), numpy.uint8)
    return cv2.erode(mask, kernel, iterations=REGION_ERODE_PX) > 0


def lever_angle_deg(frame, bbox, contour, to_table):
    """Measure the lever direction of one part, vision only.

    frame is the BGR table image, bbox the (x, y, w, h) blob box,
    contour the diff-mask blob contour, to_table a pixel to table
    mm mapping. Returns {"ok", "lever_deg", "strength", "reason",
    "source"}: lever_deg in [0, 180) in the table frame (None when
    not ok), strength in 0..1, source "levers", "slot" or
    "outline" (None when not ok). The precise angle comes from
    the blob outline (minAreaRect long side); the gradient stripe
    cue inside the part only chooses between that direction and
    +90 deg. Without a clear stripe cue, the dark slot at one end
    of the levers (centre to slot centroid, same choice) is read;
    without either cue, the long side is used when the outline
    aspect ratio reaches FALLBACK_ASPECT_MIN, else ok is False.
    """
    if contour is None or len(contour) < 3:
        return {"ok": False, "lever_deg": None, "strength": 0.0,
                "reason": "no contour", "source": None}
    try:
        outline_deg, aspect = _outline_direction(contour)
    except cv2.error:
        return {"ok": False, "lever_deg": None, "strength": 0.0,
                "reason": "bad contour", "source": None}
    height, width = frame.shape[0], frame.shape[1]
    x, y, box_w, box_h = (int(bbox[0]), int(bbox[1]),
                          int(bbox[2]), int(bbox[3]))
    x0 = max(0, x - CROP_MARGIN_PX)
    y0 = max(0, y - CROP_MARGIN_PX)
    x1 = min(width, x + box_w + CROP_MARGIN_PX)
    y1 = min(height, y + box_h + CROP_MARGIN_PX)
    if x1 <= x0 or y1 <= y0:
        return {"ok": False, "lever_deg": None, "strength": 0.0,
                "reason": "bbox outside frame", "source": None}
    center = (x + box_w / 2.0, y + box_h / 2.0)
    crop = frame[y0:y1, x0:x1]
    region = _region_mask(contour, x0, y0, (y1 - y0, x1 - x0))
    if region is None or not region.any():
        return {"ok": False, "lever_deg": None, "strength": 0.0,
                "reason": "no part region", "source": None}
    lever_inside = _lever_mask(crop) & region
    lever_pixels = int(lever_inside.sum())
    if lever_pixels >= MIN_LEVER_PIXELS:
        dilate = numpy.ones((2 * WEIGHT_DILATE_PX + 1,
                             2 * WEIGHT_DILATE_PX + 1), numpy.uint8)
        weight = cv2.dilate(lever_inside.astype(numpy.uint8),
                            dilate, iterations=1) > 0
        weight &= region
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY).astype(numpy.float64)
        stripe_deg, coherence = _stripe_direction(gray, weight)
        if stripe_deg is not None and coherence >= MIN_STRENGTH:
            second = _angle_mod_180(outline_deg + 90.0)
            if _angle_distance_mod_180(second, stripe_deg) < \
                    _angle_distance_mod_180(outline_deg, stripe_deg):
                outline_deg = second
            table_deg = _to_table_deg(outline_deg, center, to_table)
            if table_deg is None:
                return {"ok": False, "lever_deg": None,
                        "strength": 0.0,
                        "reason": "table mapping degenerate",
                        "source": None}
            return {"ok": True, "lever_deg": table_deg,
                    "strength": coherence, "reason": "levers",
                    "source": "levers"}
        reason, strength = "bars unclear", coherence
    else:
        reason, strength = "too few lever pixels", 0.0
    filled = _filled_mask(contour, x0, y0, (y1 - y0, x1 - x0))
    if filled is not None and filled.any():
        slot_deg = _slot_direction(crop, filled)
        if slot_deg is not None:
            second = _angle_mod_180(outline_deg + 90.0)
            if _angle_distance_mod_180(second, slot_deg) < \
                    _angle_distance_mod_180(outline_deg, slot_deg):
                outline_deg = second
            table_deg = _to_table_deg(outline_deg, center, to_table)
            if table_deg is None:
                return {"ok": False, "lever_deg": None,
                        "strength": 0.0,
                        "reason": "table mapping degenerate",
                        "source": None}
            return {"ok": True, "lever_deg": table_deg,
                    "strength": 0.5, "reason": "slot",
                    "source": "slot"}
    if aspect >= FALLBACK_ASPECT_MIN:
        table_deg = _to_table_deg(outline_deg, center, to_table)
        if table_deg is None:
            return {"ok": False, "lever_deg": None, "strength": 0.0,
                    "reason": "table mapping degenerate", "source": None}
        return {"ok": True, "lever_deg": table_deg,
                "strength": min(1.0, aspect - 1.0),
                "reason": "outline fallback", "source": "outline"}
    return {"ok": False, "lever_deg": None, "strength": strength,
            "reason": reason, "source": None}
