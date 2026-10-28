"""Tests for count_clips.py and vision_sam --image skip. Offline only."""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy

import muse_qc.count_clips as count_clips
import muse_qc.vision_sam as vision_sam
from muse_qc.count_clips import count_levers, verdict
from muse_qc.vision_sam import parse_output_text, run_detection

SECRET = "[REDACTED]"

THREE_LEVERS = (
    "1<|box;x1=10;y1=10;x2=50;y2=50;w=40;h=40;c=0.9|>,"
    "2<|box;x1=60;y1=10;x2=100;y2=50;w=40;h=40;c=0.8|>,"
    "3<|box;x1=110;y1=10;x2=150;y2=50;w=40;h=40;c=0.7|>"
)
TWO_LEVERS = (
    "1<|box;x1=10;y1=10;x2=50;y2=50;w=40;h=40;c=0.9|>,"
    "2<|box;x1=60;y1=10;x2=100;y2=50;w=40;h=40;c=0.8|>"
)
FOUR_LEVERS = (
    "1<|box;x1=10;y1=10;x2=50;y2=50;w=40;h=40;c=0.9|>,"
    "2<|box;x1=60;y1=10;x2=100;y2=50;w=40;h=40;c=0.8|>,"
    "3<|box;x1=110;y1=10;x2=150;y2=50;w=40;h=40;c=0.7|>,"
    "4<|box;x1=160;y1=10;x2=200;y2=50;w=40;h=40;c=0.6|>"
)
ONE_BOX = "1<|box;x1=100;y1=200;x2=300;y2=400;w=200;h=200;c=0.95|>"


class _FakeResponses:
    def __init__(self, output_text="", error=None):
        self._output_text = output_text
        self._error = error

    def create(self, **kwargs):
        if self._error is not None:
            raise self._error
        return SimpleNamespace(output_text=self._output_text)


class _FakeClient:
    def __init__(self, output_text="", error=None):
        self.responses = _FakeResponses(output_text, error)


def _frame():
    frame = numpy.zeros((120, 160, 3), dtype=numpy.uint8)
    frame[40:80, 50:110] = (200, 200, 200)
    return frame


def _real_count_files():
    return set(
        (Path(__file__).resolve().parent.parent / "recordings").glob("count_*.png")
    )


def test_three_levers_gives_good(tmp_path, capsys):
    before = _real_count_files()
    code = count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=THREE_LEVERS),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "levers: 3" in out
    assert "verdict: good" in out
    assert _real_count_files() == before


def test_two_levers_gives_bad(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=TWO_LEVERS),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "levers: 2" in out
    assert "verdict: bad" in out


def test_four_levers_gives_bad(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=FOUR_LEVERS),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "levers: 4" in out
    assert "verdict: bad" in out


def test_verdict_values():
    assert verdict(3) == "good"
    assert verdict(2) == "bad"
    assert verdict(4) == "bad"
    assert verdict(0) == "bad"


def test_low_confidence_boxes_are_ignored():
    boxes = parse_output_text(
        "1<|box;x1=10;y1=10;x2=50;y2=50;w=40;h=40;c=0.9|>,"
        "2<|box;x1=60;y1=10;x2=100;y2=50;w=40;h=40;c=0.2|>,"
        "3<|box;x1=110;y1=10;x2=150;y2=50;w=40;h=40|>"
    )
    kept = count_levers(boxes)
    assert [box["id"] for box in kept] == [1]


def test_duplicates_are_merged_keep_higher_confidence():
    boxes = parse_output_text(
        "1<|box;x1=10;y1=10;x2=50;y2=50;w=40;h=40;c=0.9|>,"
        "2<|box;x1=12;y1=12;x2=48;y2=48;w=36;h=36;c=0.6|>"
    )
    kept = count_levers(boxes)
    assert [box["id"] for box in kept] == [1]


def test_non_overlapping_boxes_are_all_kept():
    kept = count_levers(parse_output_text(THREE_LEVERS))
    assert [box["id"] for box in kept] == [1, 2, 3]


def test_run_prints_every_kept_box_with_confidence(tmp_path, capsys):
    count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=THREE_LEVERS),
        recordings_dir=tmp_path,
    )
    out = capsys.readouterr().out
    assert "0.90" in out
    assert "0.80" in out
    assert "0.70" in out
    assert out.index("levers: 3") > out.index("0.70")
    assert out.index("verdict: good") > out.index("levers: 3")


def test_annotated_image_is_written(tmp_path):
    before = _real_count_files()
    code = count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=TWO_LEVERS),
        recordings_dir=tmp_path / "recordings",
    )
    assert code == 0
    files = sorted((tmp_path / "recordings").glob("count_*.png"))
    assert len(files) == 1
    saved = cv2.imread(str(files[0]))
    assert saved is not None
    assert (saved.shape[1], saved.shape[0]) == (160, 120)
    assert _real_count_files() == before


def test_write_failure_returns_nonzero(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(count_clips.cv2, "imwrite", lambda *a, **k: False)
    code = count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=TWO_LEVERS),
        recordings_dir=tmp_path,
    )
    assert code == 1
    assert "levers: 2" in capsys.readouterr().out


def test_api_error_gives_exit_code_2(tmp_path, capsys):
    error = RuntimeError("boom")
    code = count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(error=error),
        recordings_dir=tmp_path,
    )
    assert code == 2
    assert "request failed" in capsys.readouterr().out


def test_missing_key_gives_exit_code_2(capsys):
    code = count_clips.run_count(
        None, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=THREE_LEVERS),
    )
    assert code == 2
    assert "MODEL_API_KEY" in capsys.readouterr().out


def test_key_never_appears_in_output(tmp_path, capsys):
    count_clips.run_count(
        SECRET, _frame(), "orange lever",
        lambda key: _FakeClient(output_text=THREE_LEVERS),
        recordings_dir=tmp_path,
    )
    captured = capsys.readouterr()
    assert SECRET not in captured.out
    assert SECRET not in captured.err


def test_main_image_not_readable(tmp_path, capsys):
    code = count_clips.main(
        ["--image", str(tmp_path / "missing.png")]
    )
    assert code == 1


def test_main_counts_levers_from_image_file(tmp_path, capsys, monkeypatch):
    image_path = tmp_path / "armcam.png"
    assert cv2.imwrite(str(image_path), _frame())
    monkeypatch.setattr(count_clips, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        count_clips, "_default_client_factory",
        lambda key: _FakeClient(output_text=THREE_LEVERS),
    )
    monkeypatch.chdir(tmp_path)
    code = count_clips.main(["--image", str(image_path)])
    assert code == 0
    out = capsys.readouterr().out
    assert "levers: 3" in out
    assert "verdict: good" in out
    assert len(sorted((tmp_path / "recordings").glob("count_*.png"))) == 1


def test_vision_sam_image_skips_table_mapping(tmp_path, capsys):
    before = set(
        (Path(__file__).resolve().parent.parent / "recordings").glob("sam_*.png")
    )
    code = run_detection(
        SECRET, "orange lever", _frame(),
        lambda key: _FakeClient(output_text=ONE_BOX),
        recordings_dir=tmp_path,
        map_table=False,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "table mapping: skipped (image file)" in out
    assert "table_mm=" not in out
    assert "boxes: 1" in out
    assert set(
        (Path(__file__).resolve().parent.parent / "recordings").glob("sam_*.png")
    ) == before


def test_vision_sam_main_image_skips_table_mapping(tmp_path, capsys, monkeypatch):
    image_path = tmp_path / "table.png"
    assert cv2.imwrite(str(image_path), _frame())
    monkeypatch.setattr(vision_sam, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        vision_sam, "_default_client_factory",
        lambda key: _FakeClient(output_text=ONE_BOX),
    )
    monkeypatch.chdir(tmp_path)
    code = vision_sam.main(["grey connector", "--image", str(image_path)])
    assert code == 0
    out = capsys.readouterr().out
    assert "table mapping: skipped (image file)" in out
    assert "table_mm=" not in out


def test_vision_sam_default_still_maps_table(tmp_path, capsys):
    code = run_detection(
        SECRET, "small part", _frame(),
        lambda key: _FakeClient(output_text=ONE_BOX),
        recordings_dir=tmp_path,
        load_calibration=lambda: None,
        detect_markers=lambda frame: {},
    )
    assert code == 0
    assert "table mapping: unavailable" in capsys.readouterr().out


def test_never_imports_arm_safety_or_sdk_code():
    forbidden = ("arm", "safety", "sdk", "rebotarm", "motorbridge")
    for module in (count_clips, vision_sam):
        tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            for name in names:
                assert all(
                    word not in name.lower() for word in forbidden
                ), name
