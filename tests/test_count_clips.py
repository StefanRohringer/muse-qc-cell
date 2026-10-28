"""Tests for count_clips.py and vision_sam --image skip. Offline only."""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy

import muse_qc.count_clips as count_clips
import muse_qc.vision_sam as vision_sam
from muse_qc.count_clips import count_levers, drop_short_bars, find_part, verdict
from muse_qc.vision_sam import parse_output_text, run_detection

SECRET = "[REDACTED]"

# Real fixture numbers from the human run on 2026-10-28: the part at
# (317, 187, 352, 233) and a gripper screw box at (423, 402, 456, 429),
# both at confidence 0.76. The gripper box comes first so a naive
# highest-confidence pick without the zone filter would take it.
PART_AND_GRIPPER = (
    "8<|box;x1=423;y1=402;x2=456;y2=429;w=33;h=27;c=0.76|>,"
    "7<|box;x1=317;y1=187;x2=352;y2=233;w=35;h=46;c=0.76|>"
)
GRIPPER_ONLY = "8<|box;x1=423;y1=402;x2=456;y2=429;w=33;h=27;c=0.76|>"
LOW_PART = "7<|box;x1=317;y1=187;x2=352;y2=233;w=35;h=46;c=0.49|>"
# Real lever numbers from the first two-step human run on 2026-10-28
# (crop height 608): the good part has three bars 250 to 257 px
# high, the bad part has two such bars plus a 90 px remnant in the
# slot of the torn-out lever.
THREE_BARS = (
    "1<|box;x1=158;y1=206;x2=210;y2=457;w=52;h=251;c=0.70|>,"
    "2<|box;x1=237;y1=206;x2=289;y2=457;w=52;h=251;c=0.70|>,"
    "0<|box;x1=313;y1=213;x2=363;y2=464;w=50;h=251;c=0.68|>"
)
TWO_BARS = (
    "2<|box;x1=241;y1=201;x2=290;y2=457;w=49;h=256;c=0.77|>,"
    "0<|box;x1=162;y1=205;x2=212;y2=458;w=50;h=253;c=0.75|>"
)
BAD_BARS = (
    "2<|box;x1=241;y1=201;x2=290;y2=457;w=49;h=256;c=0.77|>,"
    "0<|box;x1=162;y1=205;x2=212;y2=458;w=50;h=253;c=0.75|>,"
    "1<|box;x1=323;y1=206;x2=364;y2=296;w=41;h=90;c=0.59|>"
)
PART_BAD = "9<|box;x1=319;y1=186;x2=354;y2=232;w=35;h=46;c=0.76|>"
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


class _SeqClient:
    """Fake client with one answer text per call, recording phrases."""

    def __init__(self, texts, error=None):
        self._texts = list(texts)
        self._error = error
        self.phrases: list[str] = []
        self.responses = self

    def create(self, **kwargs):
        try:
            phrase = kwargs["input"][0]["content"][0]["text"]
        except (KeyError, IndexError, TypeError):
            phrase = ""
        self.phrases.append(phrase)
        if self._error is not None:
            raise self._error
        index = min(len(self.phrases) - 1, len(self._texts) - 1)
        return SimpleNamespace(output_text=self._texts[index])


def _frame():
    frame = numpy.zeros((120, 160, 3), dtype=numpy.uint8)
    frame[40:80, 50:110] = (200, 200, 200)
    return frame


def _arm_frame():
    """640x480 frame like the real arm camera image."""
    frame = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
    frame[187:233, 317:352] = (0, 140, 255)
    return frame


def _real_count_files():
    return set(
        (Path(__file__).resolve().parent.parent / "recordings").glob("count_*.png")
    )


def test_constants_match_spec():
    assert count_clips.PART_PROMPT == "small part"
    assert count_clips.LEVER_PROMPT == "orange bar"
    assert count_clips.GRIPPER_ZONE_Y == 380
    assert count_clips.CROP_MARGIN_PX == 15
    assert count_clips.CROP_SCALE == 8


def test_gripper_zone_box_with_equal_confidence_is_ignored(tmp_path, capsys):
    before = _real_count_files()
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "part: box=(317, 187, 352, 233) confidence=0.76" in out
    assert "(423, 402, 456, 429)" not in out.split("levers:")[0]
    assert "levers: 3" in out
    assert "verdict: good" in out
    assert out.index("part: box=") < out.index("levers: 3")
    assert _real_count_files() == before


def test_three_bars_give_good(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "levers: 3" in out
    assert "verdict: good" in out


def test_two_bars_give_bad(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, TWO_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "levers: 2" in out
    assert "verdict: bad" in out


def test_find_part_picks_highest_confidence_above_zone():
    boxes = parse_output_text(PART_AND_GRIPPER)
    part = find_part(boxes)
    assert part is not None
    assert (part["x1"], part["y1"], part["x2"], part["y2"]) == (317, 187, 352, 233)


def test_find_part_returns_none_without_usable_box():
    assert find_part(parse_output_text("")) is None
    assert find_part(parse_output_text(GRIPPER_ONLY)) is None
    assert find_part(parse_output_text(LOW_PART)) is None


def test_no_part_gives_exit_code_3(tmp_path, capsys):
    before = _real_count_files()
    for part_text in ("", GRIPPER_ONLY, LOW_PART):
        code = count_clips.run_count(
            SECRET, _arm_frame(), "orange bar",
            lambda key: _SeqClient([part_text, THREE_BARS]),
            recordings_dir=tmp_path,
        )
        assert code == 3
        assert "part: not found" in capsys.readouterr().out
    assert list(tmp_path.glob("count_*.png")) == []
    assert _real_count_files() == before


def test_crop_has_margin_and_scale_8():
    frame = _arm_frame()
    part = find_part(parse_output_text(PART_AND_GRIPPER))
    assert part is not None
    crop = count_clips.crop_and_enlarge(frame, part)
    assert (crop.shape[1], crop.shape[0]) == (65 * 8, 76 * 8)
    assert crop.shape == (608, 520, 3)


def test_crop_is_clamped_at_image_edge():
    frame = numpy.zeros((100, 100, 3), dtype=numpy.uint8)
    crop = count_clips.crop_and_enlarge(
        frame, {"x1": 0, "y1": 0, "x2": 10, "y2": 10}
    )
    assert crop.shape == (25 * 8, 25 * 8, 3)
    crop = count_clips.crop_and_enlarge(
        frame, {"x1": 90, "y1": 90, "x2": 200, "y2": 200}
    )
    assert crop.shape == (25 * 8, 25 * 8, 3)


def test_prompt_overrides_lever_prompt_but_not_part_prompt(tmp_path, capsys):
    holder: dict = {}

    def factory(key):
        holder["client"] = _SeqClient([PART_AND_GRIPPER, TWO_BARS])
        return holder["client"]

    code = count_clips.run_count(
        SECRET, _arm_frame(), "custom phrase", factory,
        recordings_dir=tmp_path,
    )
    assert code == 0
    assert holder["client"].phrases == ["small part", "custom phrase"]


def test_main_default_prompt_is_lever_prompt(tmp_path, capsys, monkeypatch):
    image_path = tmp_path / "armcam.png"
    assert cv2.imwrite(str(image_path), _arm_frame())
    holder: dict = {}

    def factory(key):
        holder["client"] = _SeqClient([PART_AND_GRIPPER, TWO_BARS])
        return holder["client"]

    monkeypatch.setattr(count_clips, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(count_clips, "_default_client_factory", factory)
    monkeypatch.chdir(tmp_path)
    code = count_clips.main(["--image", str(image_path)])
    assert code == 0
    assert holder["client"].phrases == ["small part", count_clips.LEVER_PROMPT]
    out = capsys.readouterr().out
    assert "levers: 2" in out
    assert "verdict: bad" in out


def test_both_images_are_written(tmp_path):
    before = _real_count_files()
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    files = sorted(tmp_path.glob("count_*.png"))
    assert len(files) == 2
    full = [path for path in files if not path.name.endswith("_crop.png")]
    crop = [path for path in files if path.name.endswith("_crop.png")]
    assert len(full) == 1 and len(crop) == 1
    assert crop[0].stem == full[0].stem + "_crop"
    saved_full = cv2.imread(str(full[0]))
    saved_crop = cv2.imread(str(crop[0]))
    assert saved_full is not None
    assert saved_crop is not None
    assert (saved_full.shape[1], saved_full.shape[0]) == (640, 480)
    assert (saved_crop.shape[1], saved_crop.shape[0]) == (520, 608)
    assert _real_count_files() == before


def test_write_failure_returns_nonzero(tmp_path, capsys, monkeypatch):
    monkeypatch.setattr(count_clips.cv2, "imwrite", lambda *a, **k: False)
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, TWO_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 1
    assert "levers: 2" in capsys.readouterr().out


def test_api_error_gives_exit_code_2(tmp_path, capsys):
    error = RuntimeError("boom")
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, TWO_BARS], error=error),
        recordings_dir=tmp_path,
    )
    assert code == 2
    assert "request failed" in capsys.readouterr().out


def test_second_call_error_gives_exit_code_2(tmp_path, capsys):
    holder: dict = {}

    class _FailSecond(_SeqClient):
        def create(self, **kwargs):
            try:
                phrase = kwargs["input"][0]["content"][0]["text"]
            except (KeyError, IndexError, TypeError):
                phrase = ""
            self.phrases.append(phrase)
            if len(self.phrases) > 1:
                raise RuntimeError("boom")
            return SimpleNamespace(output_text=self._texts[0])

    def factory(key):
        holder["client"] = _FailSecond([PART_AND_GRIPPER])
        return holder["client"]

    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar", factory,
        recordings_dir=tmp_path,
    )
    assert code == 2
    assert "request failed" in capsys.readouterr().out


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
    kept = count_levers(parse_output_text(THREE_BARS))
    assert [box["id"] for box in kept] == [1, 2, 0]


def test_bad_part_short_remnant_is_dropped(tmp_path, capsys):
    before = _real_count_files()
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_BAD, BAD_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "part: box=(319, 186, 354, 232) confidence=0.76" in out
    assert (
        "dropped (too short): id 1 box=(323, 206, 364, 296)"
        " confidence=0.59" in out
    )
    assert "levers: 3" not in out
    assert "levers: 2" in out
    assert "verdict: bad" in out
    assert out.index("dropped (too short)") < out.index("levers: 2")
    assert _real_count_files() == before


def test_good_part_drops_nothing(tmp_path, capsys):
    code = count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "dropped (too short)" not in out
    assert "levers: 3" in out
    assert "verdict: good" in out


def test_bar_of_exactly_min_height_is_kept():
    boxes = parse_output_text(
        "1<|box;x1=0;y1=0;x2=10;y2=180;w=10;h=180;c=0.9|>,"
        "2<|box;x1=20;y1=0;x2=30;y2=179;w=10;h=179;c=0.9|>"
    )
    kept, dropped = drop_short_bars(boxes, 600)
    assert [box["id"] for box in kept] == [1]
    assert [box["id"] for box in dropped] == [2]


def test_run_prints_every_kept_box_with_confidence(tmp_path, capsys):
    count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
        recordings_dir=tmp_path,
    )
    out = capsys.readouterr().out
    assert "0.68" in out
    assert "0.70" in out
    assert out.index("levers: 3") > out.index("0.70")
    assert out.index("verdict: good") > out.index("levers: 3")


def test_missing_key_gives_exit_code_2(capsys):
    code = count_clips.run_count(
        None, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
    )
    assert code == 2
    assert "MODEL_API_KEY" in capsys.readouterr().out


def test_key_never_appears_in_output(tmp_path, capsys):
    count_clips.run_count(
        SECRET, _arm_frame(), "orange bar",
        lambda key: _SeqClient([PART_AND_GRIPPER, THREE_BARS]),
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
