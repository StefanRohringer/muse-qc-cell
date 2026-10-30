"""Tests for the Muse Spark agent loop. Offline, fakes only.

The fake client returns scripted Responses-API-like outputs with
message items and function_call items. No network, no camera, no
arm bus: scan/sort/report functions are injected fakes, and the
agent, pick_test, find_parts and count_clips never run against
real devices or the real API here.
"""

from __future__ import annotations

import json
import signal
import sys
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import muse_qc.agent as agent_mod
from muse_qc import agent_tools
from muse_qc.agent import main, run_agent

SECRET = "fake-test-key-abc123"
P0 = [500.0, 360.0]
P1 = [540.0, 330.0]
P2 = [560.0, 300.0]


def _message(text):
    return {"type": "message",
            "content": [{"type": "output_text", "text": text}]}


def _call(call_id, name, args):
    return {"type": "function_call", "call_id": call_id, "name": name,
            "arguments": json.dumps(args)}


class _FakeResponses:
    def __init__(self, script, seen):
        self._script = list(script)
        self._seen = seen

    def create(self, **kwargs):
        self._seen.append(kwargs)
        if not self._script:
            raise AssertionError("model called more often than scripted")
        item = self._script.pop(0)
        if isinstance(item, Exception):
            raise item
        if isinstance(item, SimpleNamespace) and hasattr(item, "output"):
            return item
        return SimpleNamespace(output=item)


class FakeClient:
    """Scripted Responses-API-like client; script holds outputs."""

    def __init__(self, script, seen=None):
        self.responses = _FakeResponses(
            script, seen if seen is not None else [])


def _scan_fn_factory(scans, seen_images=None):
    """Fake scan_fn returning the queued part lists in order."""
    calls = []

    def scan():
        calls.append(True)
        parts = scans[min(len(calls) - 1, len(scans) - 1)]
        return {"parts": [{"table_mm": list(pos)} for pos in parts],
                "skipped": 0, "image": "recordings/fake.png"}

    scan.calls = calls  # type: ignore[attr-defined]
    return scan


def _sort_fn_factory(outcomes):
    """Fake sort_fn returning queued outcome dicts, logging positions."""
    seen = []
    queue = list(outcomes)

    def sort(position):
        seen.append(list(position))
        if not queue:
            raise AssertionError("sort called more often than scripted")
        result = dict(queue.pop(0))
        result.setdefault("lines", ["fake line"])
        return result

    sort.seen = seen  # type: ignore[attr-defined]
    return sort


def _sorted_outcome(levers, verdict, record):
    return {"outcome": "sorted", "levers": levers, "verdict": verdict,
            "box": verdict, "record": record, "reason": None,
            "lines": [f"inspect: levers {levers} verdict {verdict}",
                      f"dropped into the {verdict} box", record]}


def _report_fn_factory(tmp_path):
    seen = []

    def report(record_paths):
        seen.append(list(record_paths))
        out_path = tmp_path / f"report_{len(seen)}.html"
        out_path.write_text("<html></html>", encoding="utf-8")
        return str(out_path)

    report.seen = seen  # type: ignore[attr-defined]
    return report


def _run(script, scan, sort, report, tmp_path, **kwargs):
    lines = []
    client = FakeClient(script)
    result = run_agent(client, scan, sort, report, out=lines.append,
                       recordings_dir=str(tmp_path), **kwargs)
    return result, lines, client


def test_scan_sort_rescan_finish_flow(tmp_path):
    scan = _scan_fn_factory([[P0, P1], [P1]])
    sort = _sort_fn_factory([
        _sorted_outcome(3, "good", "recordings/count_a.json"),
        _sorted_outcome(2, "bad", "recordings/count_b.json"),
    ])
    report = _report_fn_factory(tmp_path)
    script = [
        [_message("scanning"), _call("c1", "scan_table", {})],
        [_message("sorting one"), _call("c2", "inspect_and_sort",
                                        {"part_id": 0})],
        [_message("rescanning"), _call("c3", "scan_table", {})],
        [_message("sorting two"),
         _call("c4", "inspect_and_sort", {"part_id": 0})],
        [_message("reporting"), _call("c5", "write_report", {})],
        [_message("done"), _call("c6", "finish",
                                 {"summary": "2 sorted: 1 good, 1 bad"})],
    ]
    result, lines, _ = _run(script, scan, sort, report, tmp_path)
    assert result["finished"] is True
    assert result["good"] == 1 and result["bad"] == 1
    assert sort.seen == [P0, P1]
    assert report.seen == [["recordings/count_a.json",
                            "recordings/count_b.json"]]
    text = "\n".join(lines)
    assert "agent -> scan_table()" in text
    assert "tool: 2 parts found" in text
    assert "tool: 1 part found" in text
    assert "agent -> inspect_and_sort(part 0)" in text
    assert "tool: part 0 levers 3 good, sorted" in text
    assert "agent finished: 2 sorted: 1 good, 1 bad" in text
    assert (tmp_path / "report_1.html").is_file()


def test_tool_call_uses_model_spark_with_tools(tmp_path):
    seen = []
    client = FakeClient(
        [[_message("bye"), _call("c1", "finish", {"summary": "empty"})]],
        seen)
    run_agent(client, _scan_fn_factory([[]]),
              _sort_fn_factory([]), _report_fn_factory(tmp_path),
              out=lambda line: None, recordings_dir=str(tmp_path))
    assert len(seen) == 1
    call = seen[0]
    assert call["model"] == "muse-spark-1.3"
    assert call["reasoning"] == {"effort": "low"}
    assert call["store"] is False
    assert call["include"] == ["reasoning.encrypted_content"]
    assert {tool["name"] for tool in call["tools"]} == {
        "scan_table", "inspect_and_sort", "write_report", "finish"}


def test_unknown_and_stale_id_refused(tmp_path):
    scan = _scan_fn_factory([[P0]])
    sort = _sort_fn_factory([_sorted_outcome(3, "good", "rec.json")])
    report = _report_fn_factory(tmp_path)
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 99})],
        [_call("c3", "inspect_and_sort", {"part_id": 0})],
        [_call("c4", "inspect_and_sort", {"part_id": 0})],
        [_call("c5", "finish", {"summary": "done"})],
    ]
    result, lines, _ = _run(script, scan, sort, report, tmp_path)
    assert result["finished"] is True
    assert len(sort.seen) == 1
    text = "\n".join(lines)
    assert "unknown part id, call scan_table first" in text
    assert text.count("refused") == 2


def test_coordinates_from_model_are_ignored(tmp_path):
    names = {tool["name"]: tool
             for tool in agent_tools.TOOL_DEFINITIONS}
    assert set(names["inspect_and_sort"]["parameters"]
               ["properties"]) == {"part_id"}
    scan = _scan_fn_factory([[P0]])
    sort = _sort_fn_factory([_sorted_outcome(3, "good", "rec.json")])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort",
               {"part_id": 0, "x": 1.0, "y": 2.0,
                "table_mm": [1.0, 2.0]})],
        [_call("c3", "finish", {"summary": "done"})],
    ]
    result, _, _ = _run(script, scan, sort,
                         _report_fn_factory(tmp_path), tmp_path)
    assert result["finished"] is True
    assert sort.seen == [P0]


def test_max_sorts_stops_loop(tmp_path):
    scan = _scan_fn_factory([[P0], [P1], [P2]])
    sort = _sort_fn_factory([
        _sorted_outcome(3, "good", "a.json"),
        _sorted_outcome(3, "good", "b.json"),
    ])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 0})],
        [_call("c3", "scan_table", {})],
        [_call("c4", "inspect_and_sort", {"part_id": 0})],
        [_call("c5", "scan_table", {})],
        [_call("c6", "inspect_and_sort", {"part_id": 0})],
    ]
    result, lines, _ = _run(script, scan, sort,
                             _report_fn_factory(tmp_path), tmp_path,
                             max_sorts=2)
    assert result["finished"] is False
    assert len(sort.seen) == 2
    assert "sort limit reached (2)" in "\n".join(lines)


def test_max_steps_stops_loop(tmp_path):
    script = [[_call(f"c{i}", "scan_table", {})] for i in range(10)]
    result, lines, _ = _run(script, _scan_fn_factory([[]]),
                             _sort_fn_factory([]),
                             _report_fn_factory(tmp_path), tmp_path,
                             max_steps=3)
    assert result["steps"] == 3
    assert "step limit reached (3)" in "\n".join(lines)


def test_two_consecutive_failures_stop(tmp_path):
    scan = _scan_fn_factory([[P0], [P0]])
    failed = {"outcome": "inspect_failed", "levers": None,
              "verdict": None, "box": None, "record": None,
              "reason": "inspect failed: no part",
              "lines": ["inspect failed: no part"]}
    sort = _sort_fn_factory([dict(failed), dict(failed)])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 0})],
        [_call("c3", "scan_table", {})],
        [_call("c4", "inspect_and_sort", {"part_id": 0})],
    ]
    result, lines, _ = _run(script, scan, sort,
                             _report_fn_factory(tmp_path), tmp_path)
    assert len(sort.seen) == 2
    assert "2 consecutive failures, stopping" in "\n".join(lines)
    assert result["finished"] is False


def test_missed_grip_retried_once_then_skipped(tmp_path):
    scan = _scan_fn_factory([[P0], [P0], [P0]])
    missed = {"outcome": "missed_grip", "levers": None,
              "verdict": None, "box": None, "record": None,
              "reason": None, "lines": ["missed grip"]}
    sort = _sort_fn_factory([dict(missed), dict(missed)])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 0})],
        [_call("c3", "scan_table", {})],
        [_call("c4", "inspect_and_sort", {"part_id": 0})],
        [_call("c5", "scan_table", {})],
        [_call("c6", "inspect_and_sort", {"part_id": 0})],
        [_call("c7", "finish", {"summary": "skipped part 0"})],
    ]
    result, lines, _ = _run(script, scan, sort,
                             _report_fn_factory(tmp_path), tmp_path)
    assert len(sort.seen) == 2
    assert "already tried twice" in "\n".join(lines)
    assert result["finished"] is True
    assert result["summary"] == "skipped part 0"


def test_api_error_stops_with_short_message(tmp_path):
    lines = []
    client = FakeClient([RuntimeError("boom offline")])
    result = run_agent(client, _scan_fn_factory([[]]),
                       _sort_fn_factory([]),
                       _report_fn_factory(tmp_path),
                       out=lines.append,
                       recordings_dir=str(tmp_path))
    text = "\n".join(lines)
    assert "model request failed: boom offline" in text
    assert SECRET not in text
    assert result["finished"] is False
    transcripts = list(tmp_path.glob("agent_*.json"))
    assert len(transcripts) == 1


def test_transcript_has_no_key_or_encrypted_content(tmp_path):
    scan = _scan_fn_factory([[P0]])
    sort = _sort_fn_factory(
        [_sorted_outcome(3, "good", "rec.json")])
    script = [
        [_message("go"), _call("c1", "scan_table", {})],
        [_message("sort"), _call("c2", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("end"), _call("c3", "finish", {"summary": "ok"})],
    ]
    _, _, client = _run(script, scan, sort,
                        _report_fn_factory(tmp_path), tmp_path)
    assert client.responses._seen
    transcripts = list(tmp_path.glob("agent_*.json"))
    assert len(transcripts) == 1
    text = transcripts[0].read_text(encoding="utf-8")
    assert SECRET not in text
    assert "encrypted" not in text
    assert "reasoning" not in text
    loaded = json.loads(text)
    assert loaded["summary"] == "ok"
    assert loaded["steps"][0]["model_text"] == "go"
    assert loaded["steps"][0]["calls"][0]["name"] == "scan_table"


def test_execute_without_yes_moves_nothing(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    lines = []

    def boom(key):
        raise AssertionError("no client without confirmation")

    code = main(["--execute"], client_factory=boom,
                input_fn=lambda prompt: "no", out=lines.append)
    assert code == 2
    assert "confirmation missing, nothing moved" in "\n".join(lines)
    assert list(tmp_path.glob("recordings/agent_*.json")) == []


def test_simulate_end_to_end_with_fake_table(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    script = [
        [_message("scan"), _call("c1", "scan_table", {})],
        [_message("sort"), _call("c2", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("scan"), _call("c3", "scan_table", {})],
        [_message("sort"), _call("c4", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("scan"), _call("c5", "scan_table", {})],
        [_message("sort"), _call("c6", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("scan"), _call("c7", "scan_table", {})],
        [_message("sort"), _call("c8", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("scan"), _call("c9", "scan_table", {})],
        [_message("report"), _call("c10", "write_report", {})],
        [_message("end"), _call("c11", "finish", {
            "summary": "sorted 3 parts: 2 good, 1 bad, 1 missed grip"})],
    ]
    lines = []
    code = main([], client_factory=lambda key: FakeClient(script),
                out=lines.append)
    assert code == 0
    text = "\n".join(lines)
    assert "tool: 3 parts found" in text
    assert "tool: no parts found" in text
    assert "missed grip" in text
    assert "agent finished:" in text
    assert SECRET not in text
    reports = list((tmp_path / "recordings").glob("report_fake_*.html"))
    assert len(reports) == 1
    page = reports[0].read_text(encoding="utf-8")
    assert "count_fake_" in page
    transcripts = list((tmp_path / "recordings").glob("agent_*.json"))
    assert len(transcripts) == 1
    loaded = json.loads(transcripts[0].read_text(encoding="utf-8"))
    assert "sorted 3 parts" in loaded["summary"]


def test_parse_pick_lines_interrupted():
    parsed = agent_tools.parse_pick_lines(
        ["interrupted by human, holding position"])
    assert parsed["outcome"] == "interrupted"
    mixed = agent_tools.parse_pick_lines(
        ["inspect: levers 3 verdict good",
         "dropped into the good box",
         "interrupted by human, holding position"])
    assert mixed["outcome"] == "interrupted"


def test_interrupted_sort_stops_loop(tmp_path):
    scan = _scan_fn_factory([[P0]])
    halted = {"outcome": "interrupted", "levers": None,
              "verdict": None, "box": None, "record": None,
              "reason": "interrupted by human, holding position",
              "lines": ["interrupted by human, holding position"]}
    sort = _sort_fn_factory([halted])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 0})],
    ]
    seen = []
    lines = []
    client = FakeClient(script, seen)
    result = run_agent(client, scan, sort,
                       _report_fn_factory(tmp_path),
                       out=lines.append,
                       recordings_dir=str(tmp_path))
    assert len(sort.seen) == 1
    assert len(seen) == 2
    assert result["finished"] is False
    text = "\n".join(lines)
    assert "tool: part 0 interrupted" in text
    assert "agent: interrupted by human, stopping" in text
    assert len(list(tmp_path.glob("agent_*.json"))) == 1


class _SdkText:
    def __init__(self, text):
        self.type = "output_text"
        self.text = text


class _SdkMessage:
    def __init__(self, chunks):
        self.type = "message"
        self.content = chunks

    def model_dump(self, exclude_none=True):
        return {"type": "message", "content": [
            {"type": chunk.type, "text": chunk.text}
            for chunk in self.content]}


class _SdkReasoning:
    def __init__(self):
        self.type = "reasoning"
        self.encrypted_content = "enc123"

    def model_dump(self, exclude_none=True):
        return {"type": "reasoning",
                "encrypted_content": self.encrypted_content}


def test_history_replays_reasoning_as_plain_dicts(tmp_path):
    seen = []
    lines = []
    client = FakeClient([
        [_SdkMessage([_SdkText("scanning")]), _SdkReasoning(),
         _call("c1", "scan_table", {})],
        [_message("end"), _call("c2", "finish", {"summary": "ok"})],
    ], seen)
    result = run_agent(
        client, _scan_fn_factory([[]]), _sort_fn_factory([]),
        _report_fn_factory(tmp_path), out=lines.append,
        recordings_dir=str(tmp_path))
    assert result["finished"] is True
    assert "agent: scanning" in "\n".join(lines)
    replayed = seen[1]["input"]
    assert all(isinstance(item, dict) for item in replayed)
    reasoning = [item for item in replayed
                 if item.get("type") == "reasoning"]
    assert reasoning == [{"type": "reasoning",
                          "encrypted_content": "enc123"}]
    transcripts = list(tmp_path.glob("agent_*.json"))
    assert len(transcripts) == 1
    text = transcripts[0].read_text(encoding="utf-8")
    assert "encrypted" not in text
    assert "reasoning" not in text


def test_main_exit_code_finish_and_unfinished(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    done_lines = []
    done = main(
        [], client_factory=lambda key: FakeClient([
            [_message("end"),
             _call("c1", "finish", {"summary": "all sorted"})],
        ]), out=done_lines.append)
    assert done == 0
    open_lines = []
    open_end = main(
        [], client_factory=lambda key: FakeClient([
            [_call("c1", "scan_table", {})],
        ]), out=open_lines.append)
    assert open_end == 1
    assert "agent finished: all sorted" in "\n".join(done_lines)


def test_usage_line_printed_and_optional(tmp_path):
    with_usage = SimpleNamespace(
        output=[_message("end"),
                _call("c1", "finish", {"summary": "ok"})],
        usage=SimpleNamespace(input_tokens=120, output_tokens=34))
    lines = []
    result = run_agent(
        FakeClient([with_usage]), _scan_fn_factory([[]]),
        _sort_fn_factory([]), _report_fn_factory(tmp_path),
        out=lines.append, recordings_dir=str(tmp_path))
    assert result["finished"] is True
    assert "agent: tokens in 120 out 34" in lines
    bare_lines = []
    bare = run_agent(
        FakeClient([[ _message("end"),
                      _call("c1", "finish", {"summary": "ok"})]]),
        _scan_fn_factory([[]]), _sort_fn_factory([]),
        _report_fn_factory(tmp_path), out=bare_lines.append,
        recordings_dir=str(tmp_path))
    assert bare["finished"] is True
    assert not [line for line in bare_lines if "tokens in" in line]


def test_parse_pick_lines_outcomes():
    parsed = agent_tools.parse_pick_lines([
        "inspect: levers 3 verdict good",
        "dropped into the good box",
        "recordings/count_20261029_120000.json",
        "extra line",
    ])
    assert parsed["outcome"] == "sorted"
    assert (parsed["levers"], parsed["verdict"], parsed["box"]) == (
        3, "good", "good")
    assert parsed["record"] == "recordings/count_20261029_120000.json"
    missed = agent_tools.parse_pick_lines(["missed grip"])
    assert missed["outcome"] == "missed_grip"
    assert missed["record"] is None
    failed = agent_tools.parse_pick_lines(["inspect failed: no part"])
    assert failed["outcome"] == "inspect_failed"
    assert agent_tools.parse_pick_lines(["weird"])["outcome"] == "error"
    both = agent_tools.parse_pick_lines(
        ["inspect failed: no frame", "missed grip",
         "dropped into the bad box"])
    assert both["outcome"] == "sorted"


def test_parse_pick_lines_unverified_part_sorts_bad():
    parsed = agent_tools.parse_pick_lines([
        "inspect failed: no frame from /dev/videoFAKE",
        "inspect failed, sorting into the bad box (unverified part)",
        "dropped into the bad box",
    ])
    assert parsed["outcome"] == "sorted"
    assert parsed["box"] == "bad"
    assert parsed["levers"] is None
    assert parsed["verdict"] is None
    assert parsed["reason"] is not None
    assert "unverified part" in parsed["reason"]


def test_parse_pick_lines_timeout_sorts_bad_unverified():
    parsed = agent_tools.parse_pick_lines([
        "inspect timeout, sorting into the bad box (unverified part)",
        "dropped into the bad box",
    ])
    assert parsed["outcome"] == "sorted"
    assert parsed["box"] == "bad"
    assert parsed["levers"] is None
    assert "unverified part" in (parsed["reason"] or "")


def test_parse_pick_lines_inspect_skipped_is_not_sorted():
    skipped = agent_tools.parse_pick_lines(
        ["missed grip", "inspect skipped: no part held"])
    assert skipped["outcome"] == "missed_grip"
    assert skipped["box"] is None
    alone = agent_tools.parse_pick_lines(
        ["inspect skipped: no part held"])
    assert alone["outcome"] == "inspect_failed"
    assert alone["box"] is None
    assert "inspect skipped" in (alone["reason"] or "")


def test_find_parts_main_output_unchanged(monkeypatch, tmp_path, capsys):
    import cv2
    import numpy

    import muse_qc.find_parts as find_parts

    width, height = 640, 480
    rng = numpy.random.default_rng(7)
    base = numpy.full((height, width, 3), (190, 175, 150), numpy.uint8)
    noise = rng.integers(-6, 7, (height, width, 3)).astype(numpy.int16)
    background = numpy.clip(
        base.astype(numpy.int16) + noise, 0, 255).astype(numpy.uint8)
    frame = background.copy()
    cv2.rectangle(frame, (307, 305), (346, 330), (110, 110, 110), -1)
    cv2.rectangle(frame, (345, 270), (384, 295), (30, 140, 230), -1)
    empty_path = tmp_path / "empty.png"
    frame_path = tmp_path / "frame.png"
    assert cv2.imwrite(str(empty_path), background)
    assert cv2.imwrite(str(frame_path), frame)
    markers = {
        0: {"center": (20.0, 20.0), "side_px": 40.0},
        1: {"center": (620.0, 20.0), "side_px": 40.0},
        2: {"center": (20.0, 460.0), "side_px": 40.0},
        3: {"center": (620.0, 460.0), "side_px": 40.0},
    }
    monkeypatch.setattr(find_parts, "detect_table_markers",
                        lambda frame: markers)
    monkeypatch.setattr(find_parts, "load_table_calibration", lambda: None)
    monkeypatch.chdir(tmp_path)
    code = find_parts.main([
        "--image", str(frame_path), "--empty", str(empty_path), "--no-sam",
    ])
    assert code == 0
    lines = capsys.readouterr().out.splitlines()
    assert lines[:-1] == [
        "part 0 center=(327, 318) area_mm2=973"
        " table_mm=(505.7, 359.8) sam=skipped lever_deg=0.0 (outline)",
        "part 1 center=(365, 283) area_mm2=975"
        " table_mm=(567.4, 317.7) sam=skipped lever_deg=0.0 (outline)",
        "candidates: 2",
    ]
    assert lines[-1].startswith("recordings/parts_")
    assert lines[-1].endswith(".png")


def test_report_built_only_from_this_run_records(tmp_path):
    from muse_qc.inspection.record import write_record
    from muse_qc.inspection.report import write_report_from_records

    def record(stamp, verdict):
        full = tmp_path / f"count_{stamp}.png"
        crop = tmp_path / f"count_{stamp}_crop.png"
        full.write_bytes(b"png")
        crop.write_bytes(b"png")
        return write_record({
            "timestamp": f"2026-10-29T12:{stamp[-2:]}:00+00:00",
            "source_image": "armcam.png",
            "full_image": str(full),
            "crop_image": str(crop),
            "part_box": {"x1": 1, "y1": 2, "x2": 3, "y2": 4},
            "part_confidence": 0.9,
            "lever_prompt": "orange bar",
            "kept_boxes": [],
            "dropped_boxes": [],
            "levers": 3,
            "verdict": verdict,
            "expected_levers": 3,
        }, tmp_path)

    old = record("20261028_120000", "good")
    first = record("20261029_120000", "good")
    second = record("20261029_120100", "bad")
    out = tmp_path / "report_run.html"
    path = write_report_from_records([first, second], out=out)
    assert path == out
    page = out.read_text(encoding="utf-8")
    assert "count_20261029_120000_crop.png" in page
    assert "count_20261029_120100_crop.png" in page
    assert "count_20261028_120000" not in page
    assert "parts inspected: 2, good: 1, bad: 1" in page
    assert old.is_file()


class _FakeStdin:
    def __init__(self):
        self.written: list[str] = []
        self.closed = False

    def write(self, text):
        self.written.append(text)

    def flush(self):
        pass

    def close(self):
        self.closed = True


class _FakeStdout:
    def __init__(self, proc):
        self._proc = proc

    def readline(self):
        proc = self._proc
        if proc.raise_ki and not proc.ki_raised:
            proc.ki_raised = True
            raise KeyboardInterrupt
        if proc.pos < len(proc.lines):
            line = proc.lines[proc.pos]
            proc.pos += 1
            return line + "\n"
        return ""

    def __iter__(self):
        while True:
            line = self.readline()
            if line == "":
                return
            yield line


class _FakeProc:
    """Fake pick_test child: scripted lines, no real process."""

    def __init__(self, lines, exit_code=0, raise_ki=False):
        self.lines = list(lines)
        self.pos = 0
        self.exit_code = exit_code
        self.raise_ki = raise_ki
        self.ki_raised = False
        self.stdin = _FakeStdin()
        self.stdout = _FakeStdout(self)
        self.signals: list = []
        self.kills = 0
        self.waits = 0

    def send_signal(self, number):
        self.signals.append(number)

    def kill(self):
        self.kills += 1

    def wait(self):
        self.waits += 1
        return self.exit_code


def test_real_sort_child_streams_and_parses(capsys):
    made = []

    def factory(cmd):
        proc = _FakeProc([
            "inspect: levers 3 verdict good",
            "dropped into the good box",
            "recordings/count_child_000001.json",
        ])
        made.append((cmd, proc))
        return proc

    result = agent_tools.real_sort([478.7, 344.7], popen_factory=factory)
    cmd, proc = made[0]
    assert cmd[:3] == [sys.executable, "-m", "muse_qc.pick_test"]
    assert cmd[cmd.index("--table-mm") + 1:][:2] == ["478.7", "344.7"]
    assert "--inspect" in cmd and "--execute" in cmd
    assert proc.stdin.written == ["yes\n"]
    assert proc.stdin.closed is True
    assert (result["outcome"], result["levers"], result["verdict"],
            result["box"]) == ("sorted", 3, "good", "good")
    assert result["record"] == "recordings/count_child_000001.json"
    assert result["exit_code"] == 0
    shown = capsys.readouterr().out
    assert "dropped into the good box" in shown


def test_real_sort_interrupt_drains_without_killing():
    proc = _FakeProc(
        ["contact: gripper commanded deg -45.82",
         "interrupted by human, holding position"],
        exit_code=1, raise_ki=True)
    result = agent_tools.real_sort(
        [478.7, 344.7], popen_factory=lambda cmd: proc)
    assert result["outcome"] == "interrupted"
    assert proc.kills == 0
    assert proc.signals == []
    assert proc.waits >= 1
    assert result["exit_code"] == 1
    assert "interrupted by human, holding position" in result["lines"]


def test_real_sort_timeout_sends_sigint():
    ticks = iter([0.0, 1000.0, 1000.0])
    proc = _FakeProc(["working..."], exit_code=1)
    result = agent_tools.real_sort(
        [478.7, 344.7], popen_factory=lambda cmd: proc,
        timeout_s=600.0, clock=lambda: next(ticks))
    assert proc.signals == [signal.SIGINT]
    assert proc.kills == 0
    assert result["outcome"] == "error"
    assert result["reason"] == "sort timed out after 600 s"
    assert result["exit_code"] == 1


def test_parse_pick_lines_error_reasons():
    busy = ("request failed: new_dm_serial failed: open serial port"
            " /dev/ttyACM0 failed: Device or resource busy")
    parsed = agent_tools.parse_pick_lines(["plan ok", busy])
    assert parsed["outcome"] == "error"
    assert parsed["reason"] == busy
    motion = agent_tools.parse_pick_lines(
        ["motion failed: old", "request failed: later"])
    assert motion["outcome"] == "error"
    assert motion["reason"] == "request failed: later"
    only_motion = agent_tools.parse_pick_lines(["motion failed: stuck"])
    assert only_motion["reason"] == "motion failed: stuck"
    ok = agent_tools.parse_pick_lines(
        ["inspect: levers 3 verdict good",
         "dropped into the good box"])
    assert ok["outcome"] == "sorted"
    assert ok["reason"] is None


def test_reports_use_local_time(tmp_path):
    fixed = datetime(2026, 10, 29, 21, 55, 12)
    fake = agent_tools.fake_report(
        ["a.json"], str(tmp_path), now=lambda: fixed)
    assert Path(fake).name == "report_fake_20261029_215512.html"
    from muse_qc.inspection.report import write_report_from_records

    real = write_report_from_records(
        [], recordings_dir=str(tmp_path), now=lambda: fixed)
    assert Path(real).name == "report_20261029_215512.html"


def test_two_child_sorts_in_one_run(tmp_path):
    made = []

    def factory(cmd):
        made.append(cmd)
        number = len(made)
        return _FakeProc([
            "inspect: levers 3 verdict good",
            "dropped into the good box",
            f"recordings/count_child_{number:06d}.json",
        ])

    sort = lambda pos: agent_tools.real_sort(  # noqa: E731
        pos, popen_factory=factory)
    scan = _scan_fn_factory([[[P0[0], P0[1]], [P1[0], P1[1]]], [P1]])
    report = _report_fn_factory(tmp_path)
    script = [
        [_message("scan"), _call("c1", "scan_table", {})],
        [_message("sort"), _call("c2", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("scan"), _call("c3", "scan_table", {})],
        [_message("sort"), _call("c4", "inspect_and_sort",
                                 {"part_id": 0})],
        [_message("report"), _call("c5", "write_report", {})],
        [_message("end"), _call("c6", "finish",
                                {"summary": "2 sorted"})],
    ]
    result, _, _ = _run(script, scan, sort, report, tmp_path)
    assert result["finished"] is True
    assert result["good"] == 2
    assert len(made) == 2
    assert report.seen == [["recordings/count_child_000001.json",
                            "recordings/count_child_000002.json"]]
