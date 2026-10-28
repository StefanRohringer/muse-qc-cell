"""Tests for muse_qc/inspection/report.py. Offline only, tmp_path only."""

from __future__ import annotations

from pathlib import Path

from muse_qc.inspection.record import write_record
from muse_qc.inspection.report import main


def _record(tmp_path, stamp, verdict, levers, prompt="orange bar"):
    full = tmp_path / f"count_{stamp}.png"
    crop = tmp_path / f"count_{stamp}_crop.png"
    full.write_bytes(b"png")
    crop.write_bytes(b"png")
    record = {
        "timestamp": f"2026-10-28T{stamp[9:11]}:{stamp[11:13]}:00+00:00",
        "source_image": "armcam.png",
        "full_image": str(full),
        "crop_image": str(crop),
        "part_box": {"x1": 317, "y1": 187, "x2": 352, "y2": 233},
        "part_confidence": 0.76,
        "lever_prompt": prompt,
        "kept_boxes": [],
        "dropped_boxes": (
            [{"id": 1, "x1": 323, "y1": 206, "x2": 364,
              "y2": 296, "confidence": 0.59}]
            if verdict == "bad" else []
        ),
        "levers": levers,
        "verdict": verdict,
        "expected_levers": 3,
    }
    return write_record(record, tmp_path)


def test_report_with_good_and_bad(tmp_path):
    _record(tmp_path, "20261028_120000", "good", 3)
    _record(tmp_path, "20261028_120100", "bad", 2)
    out = tmp_path / "report.html"
    assert main(["--recordings", str(tmp_path), "--out", str(out)]) == 0
    page = out.read_text(encoding="utf-8")
    assert "Muse QC Cell inspection report" in page
    assert "GOOD" in page
    assert "BAD" in page
    assert "parts inspected: 2, good: 1, bad: 1" in page
    assert "count_20261028_120000_crop.png" in page
    assert "count_20261028_120100_crop.png" in page
    assert "count_20261028_120000.png" in page
    assert str(tmp_path) not in page
    assert "http" not in page
    assert "<script" not in page
    newest = page.index("count_20261028_120100_crop.png")
    oldest = page.index("count_20261028_120000_crop.png")
    assert newest < oldest
    assert "dropped remnants" in page


def test_report_defaults_out_inside_recordings(tmp_path, capsys):
    _record(tmp_path, "20261028_120000", "good", 3)
    assert main(["--recordings", str(tmp_path)]) == 0
    out = tmp_path / "report.html"
    assert out.is_file()
    assert str(out) in capsys.readouterr().out


def test_report_escapes_record_text(tmp_path):
    _record(tmp_path, "20261028_120000", "bad", 2,
            prompt='<script>alert("x")</script>')
    out = tmp_path / "report.html"
    assert main(["--recordings", str(tmp_path), "--out", str(out)]) == 0
    page = out.read_text(encoding="utf-8")
    assert '<script>alert("x")</script>' not in page
    assert "&lt;script&gt;" in page


def test_empty_folder_says_no_inspections(tmp_path):
    out = tmp_path / "report.html"
    assert main(["--recordings", str(tmp_path), "--out", str(out)]) == 0
    page = out.read_text(encoding="utf-8")
    assert "no inspections yet" in page
    assert "parts inspected: 0, good: 0, bad: 0" in page


def test_write_failure_gives_exit_code_1(tmp_path):
    _record(tmp_path, "20261028_120000", "good", 3)
    assert main(["--recordings", str(tmp_path),
                 "--out", str(tmp_path)]) == 1


def test_missing_folder_gives_empty_report(tmp_path):
    out = tmp_path / "report.html"
    assert main(["--recordings", str(tmp_path / "absent"),
                 "--out", str(out)]) == 0
    assert "no inspections yet" in out.read_text(encoding="utf-8")
