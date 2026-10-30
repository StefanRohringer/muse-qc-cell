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
GREY = (110, 110, 110)
ORANGE = (30, 140, 230)
BROWNISH = (60, 110, 170)
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


def _part_frame(deg, bar_color=ORANGE, square=34.0):
    """Grey square with three bars at deg on a plain background."""
    frame = numpy.full((HEIGHT, WIDTH, 3), BACKGROUND, numpy.uint8)
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
    for bar in _bar_polygons(center, deg):
        cv2.fillPoly(frame, [bar], bar_color)
    return frame


def _diff_contour(frame):
    """Largest non-background blob contour (stands in for the diff)."""
    mask = numpy.any(
        frame != numpy.array(BACKGROUND, dtype=numpy.uint8), axis=2,
    ).astype(numpy.uint8) * 255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    assert contours
    return max(contours, key=cv2.contourArea)


def _measure(deg, **kwargs):
    frame = _part_frame(deg, **kwargs)
    contour = _diff_contour(frame)
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
