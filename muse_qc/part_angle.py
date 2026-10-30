"""Lever direction of a connector part on the table camera image.

The part is a grey connector (about 17 x 20.5 mm) with 3 orange
levers lying side by side on top. The levers can look brownish
under the lamp. The part outline alone gives a precise angle but
only modulo 90 deg (the part is nearly square), so the orange
lever bars pick which of the two outline directions is the lever
direction. The final angle always comes from the outline.

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

# Orange/brown lever pixels in OpenCV HSV (H 0..179). S_MIN and
# V_MIN stay low so brownish levers under the lamp still match,
# while the grey body (near-zero saturation) does not.
LEVER_H_MIN = 3
LEVER_H_MAX = 35
LEVER_S_MIN = 50
LEVER_S_MAX = 255
LEVER_V_MIN = 50
LEVER_V_MAX = 255
# Pixels around the blob bbox included in the lever crop, so
# slightly oversized blobs still show all three bars.
CROP_MARGIN_PX = 4
# A single lever contour below this area (px2) is noise, not a bar.
MIN_BAR_AREA_PX2 = 8
# A lever contour votes only when its minAreaRect is this
# elongated (long side over short side): merged lever blobs on
# real images are compact and must not decide the direction.
MIN_BAR_ASPECT = 2.0
# Fewer lever pixels than this: the levers are not visible.
MIN_LEVER_PIXELS = 40
# Pixel count at which the lever cue counts fully; below it the
# strength scales down linearly.
STRONG_LEVER_PIXELS = 120
# Directional agreement below this: the bars are unclear.
MIN_STRENGTH = 0.5
# Outline aspect ratio (long side over short side) at or above
# which the long side is a usable fallback direction.
FALLBACK_ASPECT_MIN = 1.1


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


def _bar_direction(mask):
    """Dominant bar direction of a lever mask in [0, 180).

    Uses minAreaRect of each lever contour: the long side of a
    thin bar is its direction. Only contours elongated enough
    (MIN_BAR_ASPECT) vote, so a blob of merged levers stays out.
    Contours are averaged with the doubled-angle method weighted
    by area, so bars 180 deg apart agree instead of cancelling.
    Returns (angle, agreement) where agreement is the resultant
    length in 0..1 (1 = parallel); (None, 0.0) when no contour
    votes.
    """
    contours, _ = cv2.findContours(mask.astype(numpy.uint8),
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    sum_cos, sum_sin, sum_weight = 0.0, 0.0, 0.0
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < MIN_BAR_AREA_PX2:
            continue
        rect = cv2.minAreaRect(contour)
        (_, _), (width, height), _ = rect
        short_side = min(float(width), float(height))
        if short_side <= 0.0:
            continue
        if float(max(float(width), float(height))) / short_side \
                < MIN_BAR_ASPECT:
            continue
        box = cv2.boxPoints(rect)
        angle = _long_side_deg([(float(p[0]), float(p[1])) for p in box])
        doubled = math.radians(2.0 * angle)
        sum_cos += area * math.cos(doubled)
        sum_sin += area * math.sin(doubled)
        sum_weight += area
    if sum_weight <= 0.0:
        return None, 0.0
    agreement = math.hypot(sum_cos, sum_sin) / sum_weight
    angle = _angle_mod_180(math.degrees(math.atan2(sum_sin, sum_cos)) / 2.0)
    return angle, min(1.0, max(0.0, agreement))


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


def lever_angle_deg(frame, bbox, contour, to_table):
    """Measure the lever direction of one part, vision only.

    frame is the BGR table image, bbox the (x, y, w, h) blob box,
    contour the diff-mask blob contour, to_table a pixel to table
    mm mapping. Returns {"ok", "lever_deg", "strength", "reason",
    "source"}: lever_deg in [0, 180) in the table frame (None when
    not ok), strength in 0..1, source "levers" or "outline" (None
    when not ok). The precise angle comes from the blob outline
    (minAreaRect long side); the orange lever bars only choose
    between that direction and +90 deg. Without clear levers, the
    long side is used when the outline aspect ratio reaches
    FALLBACK_ASPECT_MIN, else ok is False.
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
    mask = _lever_mask(frame[y0:y1, x0:x1])
    lever_pixels = int(mask.sum())
    if lever_pixels >= MIN_LEVER_PIXELS:
        bar_deg, agreement = _bar_direction(mask)
        if bar_deg is not None:
            strength = agreement * min(
                1.0, lever_pixels / STRONG_LEVER_PIXELS)
            if strength >= MIN_STRENGTH:
                second = _angle_mod_180(outline_deg + 90.0)
                if _angle_distance_mod_180(second, bar_deg) < \
                        _angle_distance_mod_180(outline_deg, bar_deg):
                    outline_deg = second
                table_deg = _to_table_deg(outline_deg, center, to_table)
                if table_deg is None:
                    return {"ok": False, "lever_deg": None,
                            "strength": 0.0,
                            "reason": "table mapping degenerate",
                            "source": None}
                return {"ok": True, "lever_deg": table_deg,
                        "strength": strength, "reason": "levers",
                        "source": "levers"}
            reason = "bars unclear"
        else:
            strength, reason = 0.0, "no bars found"
    else:
        strength, reason = 0.0, "too few lever pixels"
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
