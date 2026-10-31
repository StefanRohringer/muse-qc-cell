"""Tests for muse_qc/inspection/export_site.py.

Offline only: records and images are synthetic files in tmp_path,
never real recordings.
"""

from __future__ import annotations

import json

import cv2
import numpy

from muse_qc.inspection.export_site import (
    export_run,
    export_table_scan,
    main,
)


def _crop(path, height=600, width=500):
    frame = numpy.zeros((height, width, 3), dtype=numpy.uint8)
    frame[100:500, 100:400] = (60, 120, 200)
    cv2.imwrite(str(path), frame)


def _recordings(target, stamps, verdicts):
    target.mkdir(parents=True, exist_ok=True)
    for stamp, verdict in zip(stamps, verdicts):
        levers = 3 if verdict == "good" else 2
        _crop(target / f"count_{stamp}_crop.png")
        (target / f"count_{stamp}.png").write_bytes(b"full")
        record = {
            "timestamp": f"2026-10-31T00:00:00+00:00",
            "source_image": f"recordings/armcam_{stamp}.png",
            "full_image": f"recordings/count_{stamp}.png",
            "crop_image": f"recordings/count_{stamp}_crop.png",
            "part_box": {"x1": 1, "y1": 2, "x2": 3, "y2": 4},
            "part_confidence": 0.9,
            "lever_prompt": "brown bar",
            "kept_boxes": [],
            "dropped_boxes": [],
            "levers": levers,
            "verdict": verdict,
            "expected_levers": 3,
        }
        (target / f"count_{stamp}.json").write_text(
            json.dumps(record), encoding="utf-8")


def test_window_filters_and_reencodes(tmp_path, capsys):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_013900", "20261031_014235", "20261031_015900"],
                ["good", "bad", "good"])
    out = tmp_path / "site" / "data"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    assert len(data["parts"]) == 1
    assert data["parts"][0]["verdict"] == "bad"
    assert data["parts"][0]["levers"] == 2
    assert data["parts"][0]["image"] == "media/count_20261031_014235_crop.jpg"
    assert data["good"] == 0
    assert data["bad"] == 1
    assert data["date"] == "31 Oct 2026"
    assert (out / "run.json").exists()
    assert (out / "media" / "count_20261031_014235_crop.jpg").exists()
    assert not (out / "media" / "count_20261031_013900_crop.jpg").exists()


def test_longest_side_limited(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    _crop(src / "count_20261031_014235_crop.png", height=1200, width=900)
    out = tmp_path / "out"
    export_run(src, "20261031_014000", "20261031_015000", out)
    image = cv2.imread(
        str(out / "media" / "count_20261031_014235_crop.jpg"), cv2.IMREAD_COLOR)
    assert max(image.shape[:2]) <= 800


def test_run_json_has_no_absolute_paths(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    text = json.dumps(data) + (out / "run.json").read_text(encoding="utf-8")
    assert str(src) not in text
    assert str(tmp_path) not in text
    assert "recordings/" not in text


def test_optional_box_and_retries_pass_through(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    record_path = src / "count_20261031_014235.json"
    record = json.loads(record_path.read_text(encoding="utf-8"))
    record["box"] = "good"
    record["retries"] = 2
    record_path.write_text(json.dumps(record), encoding="utf-8")
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    assert data["parts"][0]["box"] == "good"
    assert data["parts"][0]["retries"] == 2


def test_full_image_never_copied(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    (src / "count_20261031_014235.png").write_bytes(b"full-table")
    out = tmp_path / "out"
    export_run(src, "20261031_014000", "20261031_015000", out)
    names = [path.name for path in (out / "media").iterdir()]
    assert names == ["count_20261031_014235_crop.jpg"]


def test_empty_window_fails(tmp_path, capsys):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    code = main(["--recordings", str(src), "--since", "20261030_000000",
                 "--until", "20261030_010000", "--out", str(tmp_path / "o")])
    assert code == 1
    assert "export failed" in capsys.readouterr().out


def test_main_prints_summary(tmp_path, capsys):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235", "20261031_014319"], ["good", "bad"])
    out = tmp_path / "o"
    code = main(["--recordings", str(src), "--since", "20261031_014000",
                 "--until", "20261031_015000", "--out", str(out)])
    assert code == 0
    summary = capsys.readouterr().out
    assert "parts 2" in summary
    assert "good 1" in summary
    assert "bad 1" in summary
    assert "table scans 0" in summary
    assert "KB" in summary


def _scan(target, stamp, height=700, width=900):
    frame = numpy.zeros((height, width, 3), dtype=numpy.uint8)
    frame[100:600, 100:800] = (200, 170, 140)
    cv2.imwrite(str(target / f"parts_{stamp}.png"), frame)


def test_table_image_is_newest_earlier_scan(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235", "20261031_014500"], ["good", "bad"])
    _scan(src, "20261031_014000")
    _scan(src, "20261031_014200")
    _scan(src, "20261031_014300")
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    assert data["parts"][0]["table_image"] == "media/parts_20261031_014200.jpg"
    assert data["parts"][1]["table_image"] == "media/parts_20261031_014300.jpg"
    assert "final_table_image" not in data


def test_no_scan_before_first_part(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    _scan(src, "20261031_014300")
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    assert "table_image" not in data["parts"][0]
    assert data["final_table_image"] == "media/parts_20261031_014300.jpg"


def test_no_final_scan_after_last_part(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    _scan(src, "20261031_014000")
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    assert data["parts"][0]["table_image"] == "media/parts_20261031_014000.jpg"
    assert "final_table_image" not in data


def test_shared_scan_written_once(tmp_path, capsys):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235", "20261031_014319"], ["good", "bad"])
    _scan(src, "20261031_014000")
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    assert data["parts"][0]["table_image"] == data["parts"][1]["table_image"]
    assert [path.name for path in (out / "media").iterdir()
            if path.name.startswith("parts_")] == ["parts_20261031_014000.jpg"]
    assert data["table_scans"] == 1


def test_refused_scan_names(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    frame = numpy.zeros((100, 100, 3), dtype=numpy.uint8)
    cv2.imwrite(str(src / "armcam_20261031_014000.png"), frame)
    cv2.imwrite(str(src / "table_20261031_014000.png"), frame)
    cv2.imwrite(str(src / "count_20261031_014000.png"), frame)
    out = tmp_path / "out"
    export_run(src, "20261031_014000", "20261031_015000", out)
    names = sorted(path.name for path in (out / "media").iterdir())
    assert names == ["count_20261031_014235_crop.jpg"]
    assert export_table_scan(src / "armcam_20261031_014000.png", out) is None
    assert export_table_scan(src / "table_20261031_014000.png", out) is None
    assert export_table_scan(src / "count_20261031_014000.png", out) is None


def test_table_scan_longest_side_limited(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    _scan(src, "20261031_014000", height=1600, width=1400)
    out = tmp_path / "out"
    export_run(src, "20261031_014000", "20261031_015000", out)
    image = cv2.imread(
        str(out / "media" / "parts_20261031_014000.jpg"), cv2.IMREAD_COLOR)
    assert max(image.shape[:2]) <= 1200


def test_scan_run_json_has_no_absolute_paths(tmp_path):
    src = tmp_path / "recordings"
    _recordings(src, ["20261031_014235"], ["good"])
    _scan(src, "20261031_014000")
    _scan(src, "20261031_014300")
    out = tmp_path / "out"
    data = export_run(src, "20261031_014000", "20261031_015000", out)
    text = json.dumps(data) + (out / "run.json").read_text(encoding="utf-8")
    assert str(src) not in text
    assert str(tmp_path) not in text
    assert "recordings/" not in text
