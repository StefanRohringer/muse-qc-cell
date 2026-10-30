"""Tests for the 0-lever retry: count_retry, crop params, inspect line.

Offline only: scripted counters and a fake SAM client, fake arm
camera, fake API key. No bus, no serial port, no camera, no
network. Never run against real devices.
"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy

from muse_qc import count_clips, count_retry, inspect_step

SECRET = "[REDACTED]"

# Held-part box near EXPECTED_PART_PX (342, 290), as in
# tests/test_inspect.py.
PART = (
    "8<|box;x1=423;y1=402;x2=456;y2=429;w=33;h=27;c=0.76|>,"
    "7<|box;x1=323;y1=267;x2=362;y2=312;w=39;h=44;c=0.75|>"
)
THREE_BARS = (
    "1<|box;x1=158;y1=206;x2=210;y2=457;w=52;h=251;c=0.70|>,"
    "2<|box;x1=237;y1=206;x2=289;y2=457;w=52;h=251;c=0.70|>,"
    "0<|box;x1=313;y1=213;x2=363;y2=464;w=50;h=251;c=0.68|>"
)
TWO_BARS = (
    "2<|box;x1=241;y1=201;x2=290;y2=457;w=49;h=256;c=0.77|>,"
    "0<|box;x1=162;y1=205;x2=212;y2=458;w=50;h=253;c=0.75|>"
)


class _ScriptedCounter:
    """count_fn stand-in returning one lever count per call."""

    def __init__(self, levers):
        self._levers = list(levers)
        self.calls: list[tuple] = []

    def __call__(self, client, frame, prompt, recordings_dir="",
                 source_image="", expected_px=None,
                 crop_scale=8, crop_margin_px=15):
        self.calls.append(
            (frame, prompt, crop_scale, crop_margin_px))
        levers = self._levers[min(len(self.calls) - 1,
                                  len(self._levers) - 1)]
        return {
            "ok": True,
            "reason": "",
            "stage": "done",
            "levers": levers,
            "verdict": "good" if levers == 3 else "bad",
        }


class _SeqClient:
    """Fake SAM client with one answer text per call."""

    def __init__(self, texts):
        self._texts = list(texts)
        self.calls = 0
        self.responses = self

    def create(self, **kwargs):
        text = self._texts[min(self.calls, len(self._texts) - 1)]
        self.calls += 1
        return SimpleNamespace(output_text=text)


def _frame():
    frame = numpy.zeros((480, 640, 3), dtype=numpy.uint8)
    frame[187:233, 317:352] = (0, 140, 255)
    return frame


def _no_grab():
    raise AssertionError("grab_frame must not be called")


def test_no_retry_on_3():
    counter = _ScriptedCounter([3])
    result = count_retry.count_with_retry(
        object(), _frame(), "brown bar", grab_frame=_no_grab,
        count_fn=counter)
    assert result["levers"] == 3
    assert result["verdict"] == "good"
    assert len(counter.calls) == 1
    assert result["attempts"] == [("brown bar", 8, 15, 3)]


def test_1_first_is_retried():
    counter = _ScriptedCounter([1, 0, 0, 0, 0, 0])
    result = count_retry.count_with_retry(
        object(), _frame(), "brown bar", grab_frame=lambda: _frame(),
        count_fn=counter)
    assert len(counter.calls) > 1
    assert result["levers"] == 1
    assert result["verdict"] == "bad"


def test_2_first_then_two_threes_is_good(tmp_path):
    counter = _ScriptedCounter([2, 3, 3])
    frame = _frame()
    fresh = _frame() + 1
    grabs: list = []

    def grab():
        grabs.append(True)
        return fresh

    result = count_retry.count_with_retry(
        object(), frame, "brown bar", recordings_dir=tmp_path,
        count_fn=counter, grab_frame=grab)
    assert result["levers"] == 3
    assert result["verdict"] == "good"
    assert len(grabs) == 1
    assert len(counter.calls) == 3
    assert counter.calls[0][0] is frame
    assert counter.calls[0][1:] == ("brown bar", 8, 15)
    assert counter.calls[1][0] is fresh
    assert counter.calls[1][1:] == ("brown bar", 8, 15)
    assert counter.calls[2][1:] == ("brown bar", 6, 15)
    assert result["attempts"] == [
        ("brown bar", 8, 15, 2),
        ("brown bar", 8, 15, 3),
        ("brown bar", 6, 15, 3),
    ]
    assert len(list(Path(tmp_path).glob("armcam_*.png"))) == 1


def test_single_3_stays_bad_and_returns_first():
    first = {"ok": True, "levers": 1, "verdict": "bad"}
    third = {"ok": True, "levers": 3, "verdict": "good"}
    rest = {"ok": True, "levers": 0, "verdict": "bad"}
    results = [first, third, rest, rest, rest, rest]
    calls: list = []

    def counter(*args, **kwargs):
        calls.append(args)
        return results[min(len(calls) - 1, len(results) - 1)]

    result = count_retry.count_with_retry(
        object(), _frame(), "brown bar", grab_frame=lambda: _frame(),
        count_fn=counter)
    assert result is first
    assert result["levers"] == 1
    assert result["verdict"] == "bad"
    assert [entry[3] for entry in result["attempts"]] == [1, 3, 0, 0, 0, 0]


def test_all_zero_stops_early(tmp_path):
    counter = _ScriptedCounter([0])
    result = count_retry.count_with_retry(
        object(), _frame(), "brown bar", recordings_dir=tmp_path,
        grab_frame=lambda: _frame(), count_fn=counter)
    assert result["levers"] == 0
    assert result["verdict"] == "bad"
    assert len(counter.calls) == 5
    assert result["attempts"] == [
        ("brown bar", 8, 15, 0),
        ("brown bar", 8, 15, 0),
        ("brown bar", 6, 15, 0),
        ("brown bar", 10, 25, 0),
        ("orange bar", 8, 15, 0),
    ]
    assert count_retry.RETRY_VARIANTS == (
        ("brown bar", 6, 15),
        ("brown bar", 10, 25),
        ("orange bar", 8, 15),
        ("orange bar", 6, 15),
    )


def test_4_first_is_retried():
    counter = _ScriptedCounter([4, 3, 3])
    result = count_retry.count_with_retry(
        object(), _frame(), "brown bar", grab_frame=lambda: _frame(),
        count_fn=counter)
    assert result["levers"] == 3
    assert result["verdict"] == "good"
    assert len(counter.calls) == 3


def test_failed_first_count_is_not_retried():
    failed = {
        "ok": False,
        "reason": "part: target not in view",
        "stage": "no-part",
        "levers": None,
        "verdict": None,
    }
    calls: list = []

    def counter(*args, **kwargs):
        calls.append(args)
        return failed

    result = count_retry.count_with_retry(
        object(), _frame(), "brown bar", grab_frame=_no_grab,
        count_fn=counter)
    assert result is failed
    assert "attempts" not in result
    assert len(calls) == 1


def test_crop_default_unchanged_and_honours_scale_margin():
    frame = _frame()
    part = {"x1": 317, "y1": 187, "x2": 352, "y2": 233}
    assert count_clips.crop_and_enlarge(frame, part).shape == (608, 520, 3)
    assert count_clips.crop_and_enlarge(
        frame, part, 6, 25).shape == (576, 510, 3)


def _inspect_kwargs(tmp_path, counter):
    return {
        "find_node": lambda: "/dev/videoFAKE",
        "open_camera": lambda node: _frame(),
        "read_key": lambda: SECRET,
        "make_client": lambda key: object(),
        "count_frame_fn": counter,
        "recordings_dir": tmp_path,
    }


def _inspect_result(levers, attempts=None):
    result = {
        "ok": True,
        "reason": "",
        "levers": levers,
        "verdict": "good" if levers == 3 else "bad",
        "full_image": "full.png",
        "crop_image": "crop.png",
        "record": "record.json",
    }
    if attempts is not None:
        result["attempts"] = attempts
    return result


def test_inspect_prints_retry_line_only_on_retry(tmp_path):
    lines: list[str] = []
    box = inspect_step.run_inspection(
        lines.append,
        **_inspect_kwargs(
            tmp_path,
            lambda *a, **k: _inspect_result(3, [
                ("brown bar", 8, 15, 0), ("brown bar", 8, 15, 3)])))
    assert box == "good"
    text = "\n".join(lines)
    assert "inspect retry: 2 attempts" in text
    assert text.index("inspect retry: 2 attempts") < text.index(
        "inspect: levers 3 verdict good")

    lines.clear()
    box = inspect_step.run_inspection(
        lines.append,
        **_inspect_kwargs(
            tmp_path, lambda *a, **k: _inspect_result(3)))
    assert box == "good"
    assert "inspect retry" not in "\n".join(lines)


def test_inspect_default_uses_retry_with_camera_grab(tmp_path,
                                                     monkeypatch):
    seen: dict = {}
    cameras: list = [_frame(), _frame()]

    def fake_retry(client, frame, prompt, recordings_dir="",
                   source_image="", expected_px=None, grab_frame=None):
        seen["grab_frame"] = grab_frame
        grabbed = grab_frame()
        seen["grabbed"] = grabbed
        return _inspect_result(3, [("brown bar", 8, 15, 0),
                                   ("brown bar", 8, 15, 3)])

    monkeypatch.setattr(
        inspect_step.count_retry, "count_with_retry", fake_retry)
    lines: list[str] = []
    box = inspect_step.run_inspection(
        lines.append,
        find_node=lambda: "/dev/videoFAKE",
        open_camera=lambda node: cameras.pop(0),
        read_key=lambda: SECRET,
        make_client=lambda key: object(),
        recordings_dir=tmp_path)
    assert box == "good"
    assert seen["grabbed"] is not None
    assert not cameras
    assert "inspect retry: 2 attempts" in "\n".join(lines)


def test_cli_prints_attempt_lines(tmp_path, capsys, monkeypatch):
    image_path = tmp_path / "armcam.png"
    assert cv2.imwrite(str(image_path), _frame())
    monkeypatch.setattr(count_retry, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        count_retry, "_default_client_factory",
        lambda key: _SeqClient(
            [PART, "", PART, THREE_BARS, PART, THREE_BARS]))
    monkeypatch.chdir(tmp_path)
    code = count_retry.main(["--image", str(image_path)])
    assert code == 0
    out = capsys.readouterr().out
    assert "attempt 1: prompt brown bar scale 8 margin 15 levers 0" in out
    assert "attempt 2: prompt brown bar scale 6 margin 15 levers 3" in out
    assert "attempt 3: prompt brown bar scale 10 margin 25 levers 3" in out
    assert "levers: 3" in out
    assert "verdict: good" in out


def test_cli_all_zero_stays_bad(tmp_path, capsys, monkeypatch):
    image_path = tmp_path / "armcam.png"
    assert cv2.imwrite(str(image_path), _frame())
    monkeypatch.setattr(count_retry, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        count_retry, "_default_client_factory",
        lambda key: _SeqClient(
            [PART, "", PART, "", PART, "", PART, ""]))
    monkeypatch.chdir(tmp_path)
    code = count_retry.main(["--image", str(image_path)])
    assert code == 0
    out = capsys.readouterr().out
    assert out.count("attempt ") == 4
    assert "attempt 4: prompt orange bar scale 8 margin 15 levers 0" in out
    assert "levers: 0" in out
    assert "verdict: bad" in out
