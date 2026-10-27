"""Tests for make_markers.py. Offline only, files stay inside tmp_path."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ElementTree
from pathlib import Path

from muse_qc.make_markers import (
    OUTPUT_PATH,
    SCALE_LINE_LENGTH_MM,
    SCALE_LINE_TEXT,
    TABLE_MARKER_SIZE_MM,
    build_svg,
    main,
    write_markers,
)
from muse_qc.table_markers import TABLE_MARKER_IDS

SVG_NS = "{http://www.w3.org/2000/svg}"


def _parse(text):
    return ElementTree.fromstring(text)


def _millimetres(value, name):
    assert value.endswith("mm"), name
    return float(value[:-2])


def test_svg_is_valid_xml_with_a4_size_in_mm():
    root = _parse(build_svg())
    assert root.tag == f"{SVG_NS}svg"
    assert _millimetres(root.attrib["width"], "width") == 210.0
    assert _millimetres(root.attrib["height"], "height") == 297.0
    assert root.attrib["viewBox"] == "0 0 210.0 297.0"


def test_svg_has_four_markers_of_exact_size_with_labels():
    text = build_svg()
    root = _parse(text)
    assert len(root.findall(f"{SVG_NS}g")) == 4
    for marker_id in TABLE_MARKER_IDS:
        group = root.find(f"{SVG_NS}g[@id='marker-{marker_id}']")
        assert group is not None
        rects = group.findall(f"{SVG_NS}rect")
        assert rects, f"marker {marker_id} has no black cells"
        left = min(float(rect.attrib["x"]) for rect in rects)
        top = min(float(rect.attrib["y"]) for rect in rects)
        right = max(
            float(rect.attrib["x"]) + float(rect.attrib["width"])
            for rect in rects
        )
        bottom = max(
            float(rect.attrib["y"]) + float(rect.attrib["height"])
            for rect in rects
        )
        assert right - left == TABLE_MARKER_SIZE_MM == 40.0
        assert bottom - top == TABLE_MARKER_SIZE_MM == 40.0
        labels = [
            element
            for element in group.findall(f"{SVG_NS}text")
            if element.text == f"ID {marker_id}"
        ]
        assert len(labels) == 1
    assert f"ID {TABLE_MARKER_IDS[0]}" in text


def test_svg_has_100mm_scale_line():
    root = _parse(build_svg())
    line = root.find(f"{SVG_NS}line[@id='scale-line']")
    assert line is not None
    length = abs(float(line.attrib["x2"]) - float(line.attrib["x1"]))
    assert length == SCALE_LINE_LENGTH_MM == 100.0
    texts = [element.text for element in root.findall(f"{SVG_NS}text")]
    assert SCALE_LINE_TEXT in texts


def test_markers_keep_white_margin():
    root = _parse(build_svg())
    boxes = []
    for marker_id in TABLE_MARKER_IDS:
        group = root.find(f"{SVG_NS}g[@id='marker-{marker_id}']")
        rects = group.findall(f"{SVG_NS}rect")
        left = min(float(rect.attrib["x"]) for rect in rects)
        top = min(float(rect.attrib["y"]) for rect in rects)
        right = max(
            float(rect.attrib["x"]) + float(rect.attrib["width"])
            for rect in rects
        )
        bottom = max(
            float(rect.attrib["y"]) + float(rect.attrib["height"])
            for rect in rects
        )
        assert left >= 10.0
        assert top >= 10.0
        assert 210.0 - right >= 10.0
        boxes.append((left, top, right, bottom))
    for index, first in enumerate(boxes):
        for second in boxes[index + 1 :]:
            gap_x = max(
                0.0,
                max(first[0], second[0]) - min(first[2], second[2]),
            )
            gap_y = max(
                0.0,
                max(first[1], second[1]) - min(first[3], second[3]),
            )
            assert max(gap_x, gap_y) >= 10.0


def test_write_creates_file_and_main_prints_path(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    path = write_markers()
    assert path == OUTPUT_PATH
    assert path.is_file()
    assert _parse(path.read_text(encoding="utf-8")).tag == f"{SVG_NS}svg"
    main()
    assert str(OUTPUT_PATH) in capsys.readouterr().out


def test_committed_svg_matches_generator():
    committed = (
        Path(__file__).resolve().parent.parent / "docs" / "table_markers.svg"
    )
    hint = "run uv run python -m muse_qc.make_markers and commit the file"
    assert committed.is_file(), hint
    assert committed.read_text(encoding="utf-8") == build_svg(), hint


def test_never_imports_arm_or_sdk_code():
    path = os.path.join(
        os.path.dirname(__file__), "..", "muse_qc", "make_markers.py"
    )
    with open(path, encoding="utf-8") as handle:
        source = handle.read()
    assert "from muse_qc.arm" not in source
    assert "from muse_qc import arm" not in source
    assert "import arm" not in source
    assert "reBotArm" not in source
    assert "motorbridge" not in source
