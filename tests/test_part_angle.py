"""Tests for part_angle.py and the find_parts lever_deg line.

Synthetic images and fakes only: three orange bars on a grey
square drawn at known angles, plus a find_parts CLI run on a
saved synthetic frame.
"""

from __future__ import annotations

import math

import cv2
import numpy

import muse_qc.find_parts as find_parts
from muse_qc.part_angle import lever_angle_deg

WIDTH, HEIGHT = 200, 200
BACKGROUND = (190, 175, 150)
WOOD = (140, 190, 215)
GREY = (110, 110, 110)
# Dark orange/brown lever colours inside the tightened HSV range
# (S at least 110, V at most 165): real levers under the lamp.
ORANGE = (30, 90, 150)
BROWNISH = (40, 85, 130)
DARK_LINE = (40, 40, 40)
IDENTITY = staticmethod(lambda point: (float(point[0]), float(point[1])))


def _bar_polygons(center, deg, length=26.0, width=4.0, offsets=(-8.0, 0.0, 8.0)):
    """Three parallel bar polygons running along deg (image, y down)."""
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    polygons = []
    for offset in offsets:
        mid = (center[0] + normal[0] * offset,
               center[1] + normal[1] * offset)
        half = (direction[0] * length / 2.0, direction[1] * length / 2.0)
        side = (normal[0] * width / 2.0, normal[1] * width / 2.0)
        corners = [
            (mid[0] - half[0] - side[0], mid[1] - half[1] - side[1]),
            (mid[0] + half[0] - side[0], mid[1] + half[1] - side[1]),
            (mid[0] + half[0] + side[0], mid[1] + half[1] + side[1]),
            (mid[0] - half[0] + side[0], mid[1] - half[1] + side[1]),
        ]
        polygons.append(numpy.array(corners, dtype=numpy.int32))
    return polygons


def _part_frame(deg, bar_color=ORANGE, square=34.0, background=BACKGROUND,
                bar_offsets=(-8.0, 0.0, 8.0), gap_lines=False):
    """Grey square with three bars at deg on a plain background."""
    frame = numpy.full((HEIGHT, WIDTH, 3), background, numpy.uint8)
    center = (WIDTH / 2.0, HEIGHT / 2.0)
    half = square / 2.0
    # The square turns with the bars: on the real part the levers
    # run parallel to a part edge, which is what the outline cue
    # needs to pick from.
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            center[0] + direction[0] * sign_x * half
            + normal[0] * sign_y * half,
            center[1] + direction[1] * sign_x * half
            + normal[1] * sign_y * half,
        ))
    cv2.fillPoly(frame, [numpy.array(corners, dtype=numpy.int32)], GREY)
    for bar in _bar_polygons(center, deg, offsets=bar_offsets):
        cv2.fillPoly(frame, [bar], bar_color)
    if gap_lines:
        # Slightly darker 1 px lines between touching bars: outside
        # the lever HSV range, but they keep the stripe gradients.
        for offset in (-2.0, 2.0):
            mid = (center[0] + normal[0] * offset,
                   center[1] + normal[1] * offset)
            half_len = (direction[0] * 13.0, direction[1] * 13.0)
            cv2.line(frame,
                     (int(round(mid[0] - half_len[0])),
                      int(round(mid[1] - half_len[1]))),
                     (int(round(mid[0] + half_len[0])),
                      int(round(mid[1] + half_len[1]))),
                     DARK_LINE, 1)
    return frame


SLOT = (30, 30, 30)


def _slot_part_frame(deg, long_side=40.0, short_side=38.0,
                     background=BACKGROUND):
    """Nearly square part with weak stripes and a dark end slot.

    Brownish bars run along deg and across it, so the stripe
    gradients cancel (coherence below MIN_STRENGTH although many
    lever pixels are present); a small dark slot (V below 70) at
    one end of the lever axis must then pick the direction.
    """
    frame = numpy.full((HEIGHT, WIDTH, 3), background, numpy.uint8)
    center = (WIDTH / 2.0, HEIGHT / 2.0)
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    half_l, half_s = long_side / 2.0, short_side / 2.0
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            center[0] + direction[0] * sign_x * half_l
            + normal[0] * sign_y * half_s,
            center[1] + direction[1] * sign_x * half_l
            + normal[1] * sign_y * half_s,
        ))
    cv2.fillPoly(frame, [numpy.array(corners, dtype=numpy.int32)], GREY)
    for bar in _bar_polygons(center, deg):
        cv2.fillPoly(frame, [bar], BROWNISH)
    for bar in _bar_polygons(center, deg + 90):
        cv2.fillPoly(frame, [bar], BROWNISH)
    slot_center = (center[0] + direction[0] * (half_l - 8.0),
                   center[1] + direction[1] * (half_l - 8.0))
    cv2.rectangle(frame,
                  (int(round(slot_center[0] - 2)),
                   int(round(slot_center[1] - 2))),
                  (int(round(slot_center[0] + 2)),
                   int(round(slot_center[1] + 2))),
                  SLOT, -1)
    return frame


def _rect_part_frame(deg, long_side, short_side, bar_color=ORANGE,
                     background=BACKGROUND):
    """Grey rectangle (long side along deg) with three bars at deg."""
    frame = numpy.full((HEIGHT, WIDTH, 3), background, numpy.uint8)
    center = (WIDTH / 2.0, HEIGHT / 2.0)
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    half_l, half_s = long_side / 2.0, short_side / 2.0
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            center[0] + direction[0] * sign_x * half_l
            + normal[0] * sign_y * half_s,
            center[1] + direction[1] * sign_x * half_l
            + normal[1] * sign_y * half_s,
        ))
    cv2.fillPoly(frame, [numpy.array(corners, dtype=numpy.int32)], GREY)
    for bar in _bar_polygons(center, deg):
        cv2.fillPoly(frame, [bar], bar_color)
    return frame


def _diff_contour(frame, background=BACKGROUND):
    """Largest non-background blob contour (stands in for the diff)."""
    mask = numpy.any(
        frame != numpy.array(background, dtype=numpy.uint8), axis=2,
    ).astype(numpy.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    assert contours
    return max(contours, key=cv2.contourArea)


def _measure(deg, background=BACKGROUND, **kwargs):
    frame = _part_frame(deg, background=background, **kwargs)
    contour = _diff_contour(frame, background)
    x, y, width, height = cv2.boundingRect(contour)
    return lever_angle_deg(frame, (x, y, width, height), contour, IDENTITY)


def _distance_mod_180(first, second):
    return abs((first - second + 90.0) % 180.0 - 90.0)


def test_bars_at_known_angles():
    for deg in (0, 30, 60, 90, 135):
        result = _measure(deg)
        assert result["ok"], deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg
        assert 0.0 <= result["strength"] <= 1.0


def test_brownish_bars_tolerated():
    result = _measure(30, bar_color=BROWNISH)
    assert result["ok"]
    assert _distance_mod_180(result["lever_deg"], 30) <= 5.0


def test_grey_square_without_orange_not_ok():
    frame = numpy.full((HEIGHT, WIDTH, 3), BACKGROUND, numpy.uint8)
    cv2.rectangle(frame, (80, 85), (110, 115), GREY, -1)
    contour = _diff_contour(frame)
    x, y, width, height = cv2.boundingRect(contour)
    result = lever_angle_deg(frame, (x, y, width, height), contour,
                             IDENTITY)
    assert not result["ok"]
    assert result["lever_deg"] is None
    assert result["reason"]


def test_merged_levers_fall_back_to_outline():
    """One thick orange blob (aspect below 2) must not vote."""
    frame = numpy.full((HEIGHT, WIDTH, 3), BACKGROUND, numpy.uint8)
    cv2.rectangle(frame, (80, 87), (120, 113), GREY, -1)
    cv2.rectangle(frame, (88, 92), (112, 108), ORANGE, -1)
    contour = _diff_contour(frame)
    x, y, width, height = cv2.boundingRect(contour)
    result = lever_angle_deg(frame, (x, y, width, height), contour,
                             IDENTITY)
    assert result["ok"]
    assert result["source"] == "outline"
    assert _measure(30)["source"] == "levers"


def test_wood_background_with_brownish_levers():
    """Wood-coloured background (inside the old lever HSV range).

    The table wood falls into the lever hue, so only the pixels
    inside the part contour may decide: brownish levers at 0, 30
    and 90 deg still give the right direction with source levers.
    """
    for deg in (0, 30, 90):
        result = _measure(deg, background=WOOD, bar_color=BROWNISH)
        assert result["ok"], deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg
        assert result["source"] == "levers", deg


def test_touching_bars_with_dark_gap_lines():
    """Bars that touch in the HSV mask still give the direction.

    The three bars merge into one mask blob, but the slightly
    darker 1 px lines between them keep the stripe gradients, so
    the tensor cue still finds the lever direction.
    """
    for deg in (0, 30, 90):
        result = _measure(deg, bar_color=BROWNISH,
                          bar_offsets=(-4.0, 0.0, 4.0), gap_lines=True)
        assert result["ok"], deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg
        assert result["source"] == "levers", deg


def test_nearly_square_part_with_levers():
    """Aspect 1.05 outline (below the 1.1 fallback) with levers.

    Without the stripe cue this part would end as unknown; the
    levers must carry it to source levers instead.
    """
    frame = _rect_part_frame(30, 40.0, 38.0, bar_color=BROWNISH)
    contour = _diff_contour(frame)
    x, y, width, height = cv2.boundingRect(contour)
    result = lever_angle_deg(frame, (x, y, width, height), contour,
                             IDENTITY)
    assert result["ok"]
    assert result["source"] == "levers"
    assert _distance_mod_180(result["lever_deg"], 30) <= 5.0


def test_slot_picks_direction_when_stripes_are_weak():
    """Nearly square part (aspect 1.05, below the 1.1 fallback).

    The stripe gradients cancel (bars both ways, coherence below
    MIN_STRENGTH), so the dark slot at one end of the lever axis
    must pick outline vs outline + 90 at 0, 30 and 90 deg.
    """
    for deg in (0, 30, 90):
        frame = _slot_part_frame(deg)
        contour = _diff_contour(frame)
        x, y, width, height = cv2.boundingRect(contour)
        result = lever_angle_deg(frame, (x, y, width, height), contour,
                                 IDENTITY)
        assert result["ok"], deg
        assert result["source"] == "slot", deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg


def _conflict_part_frame(true_deg, long_side=40.0, short_side=38.0,
                         background=BACKGROUND):
    """Strong stripes across the true axis, slot at its end.

    Brownish bars run across true_deg (strong coherent stripes that
    would pick the wrong outline side), while the dark slot sits at
    one end of the true lever axis: the slot must win.
    """
    frame = numpy.full((HEIGHT, WIDTH, 3), background, numpy.uint8)
    center = (WIDTH / 2.0, HEIGHT / 2.0)
    rad = math.radians(true_deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    half_l, half_s = long_side / 2.0, short_side / 2.0
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            center[0] + direction[0] * sign_x * half_l
            + normal[0] * sign_y * half_s,
            center[1] + direction[1] * sign_x * half_l
            + normal[1] * sign_y * half_s,
        ))
    cv2.fillPoly(frame, [numpy.array(corners, dtype=numpy.int32)], GREY)
    for bar in _bar_polygons(center, true_deg + 90):
        cv2.fillPoly(frame, [bar], BROWNISH)
    slot_center = (center[0] + direction[0] * (half_l - 6.0),
                   center[1] + direction[1] * (half_l - 6.0))
    cv2.rectangle(frame,
                  (int(round(slot_center[0] - 2)),
                   int(round(slot_center[1] - 2))),
                  (int(round(slot_center[0] + 2)),
                   int(round(slot_center[1] + 2))),
                  SLOT, -1)
    return frame


def test_slot_wins_over_wrong_stripes():
    """Stripes across the true axis must not beat the slot.

    The bars are strong (coherent enough to decide on their own)
    but run across the true lever axis; the dark slot at the end
    of the true axis is tried first and must win.
    """
    for deg in (0, 30, 90):
        frame = _conflict_part_frame(deg)
        contour = _diff_contour(frame)
        x, y, width, height = cv2.boundingRect(contour)
        result = lever_angle_deg(frame, (x, y, width, height), contour,
                                 IDENTITY)
        assert result["ok"], deg
        assert result["source"] == "slot", deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg


def test_nearly_square_without_slot_or_stripes_not_ok():
    """Same aspect 1.05 geometry but no slot and no stripes."""
    frame = _rect_part_frame(30, 40.0, 38.0, bar_color=GREY)
    contour = _diff_contour(frame)
    x, y, width, height = cv2.boundingRect(contour)
    result = lever_angle_deg(frame, (x, y, width, height), contour,
                             IDENTITY)
    assert not result["ok"]
    assert result["lever_deg"] is None


def test_180_deg_rotation_same_direction():
    first = _measure(30)
    second = _measure(210)
    assert first["ok"] and second["ok"]
    assert _distance_mod_180(first["lever_deg"],
                             second["lever_deg"]) <= 5.0


def _fake_markers():
    return {
        0: {"center": (20.0, 20.0), "side_px": 40.0},
        1: {"center": (620.0, 20.0), "side_px": 40.0},
        2: {"center": (20.0, 460.0), "side_px": 40.0},
        3: {"center": (620.0, 460.0), "side_px": 40.0},
    }


def test_find_parts_line_shows_lever_deg(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(
        find_parts, "detect_table_markers", lambda frame: _fake_markers()
    )
    monkeypatch.setattr(find_parts, "load_table_calibration", lambda: None)
    width, height = 640, 480
    rng = numpy.random.default_rng(7)
    base = numpy.full((height, width, 3), (190, 175, 150), numpy.uint8)
    noise = rng.integers(-6, 7, (height, width, 3)).astype(numpy.int16)
    empty = numpy.clip(base.astype(numpy.int16) + noise, 0, 255).astype(
        numpy.uint8
    )
    frame = empty.copy()
    cv2.rectangle(frame, (307, 305), (346, 330), GREY, -1)
    for bar_y in (310, 317, 324):
        cv2.rectangle(frame, (310, bar_y), (343, bar_y + 3), ORANGE, -1)
    assert cv2.imwrite(str(tmp_path / "empty.png"), empty)
    assert cv2.imwrite(str(tmp_path / "frame.png"), frame)
    monkeypatch.chdir(tmp_path)
    code = find_parts.main([
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert "lever_deg=0.0 (levers)" in out
    assert "lever_deg=?" not in out


SHADOW_SLOT = (30, 30, 30)
SHADOW_GREY = (45, 45, 45)


def _across_rect(center, deg, long=8.0, wide=3.0, along=4.0):
    """Small dark slot polygon across the bars (long axis: normal)."""
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    middle = (center[0] + direction[0] * along,
              center[1] + direction[1] * along)
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            middle[0] + normal[0] * sign_x * long / 2.0
            + direction[0] * sign_y * wide / 2.0,
            middle[1] + normal[1] * sign_x * long / 2.0
            + direction[1] * sign_y * wide / 2.0,
        ))
    return numpy.array(corners, dtype=numpy.int32)


def _shadow_slot_frame(deg):
    """Grey square with bars, an across slot and an attached shadow.

    The dark slot (8 x 3 px) runs across the bars; a large
    darker shadow (V below 60, far above the slot area limit)
    is attached on one side, so the diff contour covers part
    plus shadow and the old centre-to-slot cue points the
    wrong way.
    """
    frame = numpy.full((HEIGHT, WIDTH, 3), BACKGROUND, numpy.uint8)
    center = (WIDTH / 2.0, HEIGHT / 2.0)
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    half = 17.0
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            center[0] + direction[0] * sign_x * half
            + normal[0] * sign_y * half,
            center[1] + direction[1] * sign_x * half
            + normal[1] * sign_y * half,
        ))
    cv2.fillPoly(frame, [numpy.array(corners, dtype=numpy.int32)], GREY)
    for bar in _bar_polygons((WIDTH / 2.0, HEIGHT / 2.0), deg):
        cv2.fillPoly(frame, [bar], ORANGE)
    cv2.fillPoly(frame, [_across_rect(center, deg)], SHADOW_SLOT)
    shadow_middle = (center[0] + normal[0] * 22.0,
                     center[1] + normal[1] * 22.0)
    shadow = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        shadow.append((
            shadow_middle[0] + direction[0] * sign_x * 17.0
            + normal[0] * sign_y * 6.0,
            shadow_middle[1] + direction[1] * sign_x * 17.0
            + normal[1] * sign_y * 6.0,
        ))
    cv2.fillPoly(frame, [numpy.array(shadow, dtype=numpy.int32)],
                 SHADOW_GREY)
    return frame


def test_slot_axis_with_shadow_at_known_angles():
    """Shadow inside the contour must not turn the lever cue."""
    for deg in (30, 90, 150):
        frame = _shadow_slot_frame(deg)
        contour = _diff_contour(frame)
        x, y, width, height = cv2.boundingRect(contour)
        result = lever_angle_deg(frame, (x, y, width, height), contour,
                                 IDENTITY)
        assert result["ok"], deg
        assert result["source"] == "slot axis", deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg


def _open_gap_frame(deg):
    """Real slot plus a long thin open-lever gap along the bars.

    The gap (a 14 px long 1 px wide line, elongation above 4.5)
    runs through the part centre and so is nearer than the slot
    but must be skipped by the elongation limit, so the slot
    still decides.
    """
    frame = numpy.full((HEIGHT, WIDTH, 3), BACKGROUND, numpy.uint8)
    center = (WIDTH / 2.0, HEIGHT / 2.0)
    rad = math.radians(deg)
    direction = (math.cos(rad), math.sin(rad))
    normal = (-direction[1], direction[0])
    half = 17.0
    corners = []
    for sign_x, sign_y in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        corners.append((
            center[0] + direction[0] * sign_x * half
            + normal[0] * sign_y * half,
            center[1] + direction[1] * sign_x * half
            + normal[1] * sign_y * half,
        ))
    cv2.fillPoly(frame, [numpy.array(corners, dtype=numpy.int32)], GREY)
    for bar in _bar_polygons(center, deg):
        cv2.fillPoly(frame, [bar], ORANGE)
    start = (int(round(center[0] - direction[0] * 7.0)),
               int(round(center[1] - direction[1] * 7.0)))
    end = (int(round(center[0] + direction[0] * 7.0)),
           int(round(center[1] + direction[1] * 7.0)))
    cv2.line(frame, start, end, SHADOW_SLOT, 1)
    cv2.fillPoly(frame, [_across_rect(center, deg, along=12.0)],
                 SHADOW_SLOT)
    return frame


def test_slot_axis_prefers_slot_over_open_lever_gap():
    """A long thin open-lever gap must not beat the real slot."""
    for deg in (30, 90):
        frame = _open_gap_frame(deg)
        contour = _diff_contour(frame)
        x, y, width, height = cv2.boundingRect(contour)
        result = lever_angle_deg(frame, (x, y, width, height), contour,
                                 IDENTITY)
        assert result["ok"], deg
        assert result["source"] == "slot axis", deg
        assert _distance_mod_180(result["lever_deg"], deg) <= 5.0, deg
