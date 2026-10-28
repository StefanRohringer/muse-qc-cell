"""Tests for muse_qc/inspection/record.py and the count_clips JSON record.

Offline only: every file lands in tmp_path, never in recordings/.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import numpy

import muse_qc.count_clips as count_clips
from muse_qc.inspection.record import build_record, load_records, write_record

SECRET = "[REDACTED]"

PART_AND_GRIPPER = (
    "8<|box;x1=423;y1=402;x2=456;y2=429;w=33;h=27;c=0.76|>,"
    "7<|box;x1=317;y1=187;x2=352;y2=233;w=35;h=46;c=0.76|>"
)
THREE_BARS = (
    "1<|box;x1=158;y1=206;x2=210;y2=457;w=52;h=251;c=0.70|>,"
    "2<|box;x1=237;y1=206;x2=289;y2=457;w=52;h=251;c=0.70|>,"
    "0<|box;x1=313;y1=213;x2=363;y2=464;w=50;h=251;c=0.68|>"
)
BAD_BARS = (
    "2<|box;x1=241;y1=201;x2=290;y2=457;w=49;h=256;c=0.77|>,"
    "0<|box;x1=162;y1=205;x2=212;y2=458;w=50;h=253;c=0.75|>,"
    "1<|box;x1=323;y1=206;x2=364;y2=296;w=41;h=90;c=0.59|>"
)


class _SeqClient:
    """Fake client with one answer text per call."""

    def __init__(self, texts):
        self._texts = list(texts)
        self.phrases: list[str] = []
        self.responses = self

    def create(self, **kwargs):
        try:
            phrase = kwargs["input"][0]["content"][0]["text"]
        except (KeyError, IndexError, TypeError):
            phrase = ""
        self.phrases.append(phrase)
        index = min(len(self.phrases) - 1, len(self._texts) - 1)
        return SimpleNamespace(output_text=self._texts[index])


def _arm_frame():
    """640x480 frame like the real arm camera image."""
    frame = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
    frame[187:233, 317:352] = (0, 140, 255)
    return frame


def _record(tmp_path, **overrides):
    record = {
        "timestamp": "2026-10-28T12:00:00+00:00",
        "source_image": "armcam.png",
        "full_image": str(tmp_path / "count_20261028_120000.png"),
        "crop_image": str(tmp_path / "count_20261028_120000_crop.png"),
        "part_box": {"x1": 317, "y1": 187, "x2": 352, "y2": 233},
        "part_confidence": 0.76,
        "lever_prompt": "orange bar",
        "kept_boxes": [
            {"id": 1, "x1": 158, "y1": 206, "x2": 210,
             "y2": 457, "confidence": 0.7},
        ],
        "dropped_boxes": [],
        "levers": 1,
        "verdict": "bad",
        "expected_levers": 3,
    }
    record.update(overrides)
    return record


def test_record_round_trip(tmp_path):
    record = _record(tmp_path)
    path = write_record(record, tmp_path)
    assert path == tmp_path / "count_20261028_120000.json"
    assert path.is_file()
    loaded = load_records(tmp_path)
    assert loaded == [record]


def test_load_records_sorted_by_timestamp(tmp_path):
    write_record(_record(tmp_path, timestamp="2026-10-28T12:01:00+00:00",
                         full_image=str(tmp_path / "count_20261028_120001.png"),
                         crop_image=str(tmp_path / "count_20261028_120001_crop.png")),
                 tmp_path)
    write_record(_record(tmp_path), tmp_path)
    loaded = load_records(tmp_path)
    assert [item["timestamp"] for item in loaded] == [
        "2026-10-28T12:00:00+00:00",
        "2026-10-28T12:01:00+00:00",
    ]


def test_unreadable_json_is_skipped(tmp_path, capsys):
    write_record(_record(tmp_path), tmp_path)
    broken = tmp_path / "count_20261028_120002.json"
    broken.write_text("{not json", encoding="utf-8")
    loaded = load_records(tmp_path)
    assert len(loaded) == 1
    out = capsys.readouterr().out
    assert "warning" in out
    assert "count_20261028_120002.json" in out


def test_non_dict_json_is_skipped(tmp_path, capsys):
    (tmp_path / "count_20261028_120003.json").write_text("[1, 2]", encoding="utf-8")
    assert load_records(tmp_path) == []
    assert "warning" in capsys.readouterr().out


def test_empty_folder_gives_empty_list(tmp_path):
    assert load_records(tmp_path) == []


def test_build_record_verdict(tmp_path):
    part = {"x1": 317, "y1": 187, "x2": 352, "y2": 233, "confidence": 0.76}
    good = build_record(
        source_image="armcam.png",
        full_image=str(tmp_path / "count_20261028_120000.png"),
        crop_image=str(tmp_path / "count_20261028_120000_crop.png"),
        part=part,
        lever_prompt="orange bar",
        kept=[
            {"id": 1, "x1": 1, "y1": 2, "x2": 3, "y2": 400,
             "confidence": 0.7},
            {"id": 2, "x1": 4, "y1": 2, "x2": 6, "y2": 400,
             "confidence": 0.7},
            {"id": 0, "x1": 7, "y1": 2, "x2": 9, "y2": 400,
             "confidence": 0.68},
        ],
        dropped=[],
        expected_levers=3,
        timestamp="2026-10-28T12:00:00+00:00",
    )
    assert good["levers"] == 3
    assert good["verdict"] == "good"
    assert good["timestamp"] == "2026-10-28T12:00:00+00:00"
    assert good["expected_levers"] == 3
    assert good["timestamp"]  # default path also sets one
    auto = build_record(
        source_image="armcam.png",
        full_image="count_20261028_120000.png",
        crop_image="count_20261028_120000_crop.png",
        part=part,
        lever_prompt="orange bar",
        kept=[],
        dropped=[],
        expected_levers=3,
    )
    assert auto["verdict"] == "bad"
    assert auto["levers"] == 0


def test_count_clips_run_writes_good_json(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    files = sorted(tmp_path.glob("count_*.json"))
    assert len(files) == 1
    assert str(files[0]) in out
    record = json.loads(files[0].read_text(encoding="utf-8"))
    assert record["levers"] == 3
    assert record["verdict"] == "good"
    assert record["expected_levers"] == 3
    assert record["lever_prompt"] == "orange bar"
    assert len(record["kept_boxes"]) == 3
    assert record["dropped_boxes"] == []
    assert record["part_box"] == {"x1": 317, "y1": 187, "x2": 352, "y2": 233}
    assert record["timestamp"]
    full = [path for path in tmp_path.glob("count_*.png")
            if not path.name.endswith("_crop.png")]
    assert Path(record["full_image"]).name == full[0].name
    assert files[0].stem == full[0].stem


def test_count_clips_run_writes_bad_json_with_dropped(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, BAD_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    capsys.readouterr()
    files = sorted(tmp_path.glob("count_*.json"))
    assert len(files) == 1
    record = json.loads(files[0].read_text(encoding="utf-8"))
    assert record["levers"] == 2
    assert record["verdict"] == "bad"
    assert len(record["kept_boxes"]) == 2
    assert [box["id"] for box in record["dropped_boxes"]] == [1]
