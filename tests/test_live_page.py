"""Tests for the live status page. Offline, fakes only.

Scripted Responses-API-like clients drive run_agent with a
FakeTable and a LivePage in tmp_path; no network, no camera, no
serial or bus. The page is plain static HTML read back from disk.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

from muse_qc.agent import agent_tools
from muse_qc.agent.agent import run_agent
from muse_qc.agent.live_page import LivePage


def _message(text):
    return {"type": "message",
            "content": [{"type": "output_text", "text": text}]}


def _call(call_id, name, args):
    return {"type": "function_call", "call_id": call_id, "name": name,
            "arguments": json.dumps(args)}


class _FakeResponses:
    def __init__(self, script):
        self._script = list(script)

    def create(self, **kwargs):
        if not self._script:
            raise AssertionError("model called more often than scripted")
        return SimpleNamespace(output=self._script.pop(0))


class FakeClient:
    """Scripted Responses-API-like client; script holds outputs."""

    def __init__(self, script):
        self.responses = _FakeResponses(script)


def _full_sort_script():
    """Drive the FakeTable through all 3 parts (one missed grip).

    Every sort rescans at once with new ids, so no scan_table
    call sits between two sorts; one explicit rescan follows
    the missed grip.
    """
    return [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 0})],
        [_call("c3", "inspect_and_sort", {"part_id": 1})],
        [_call("c4", "inspect_and_sort", {"part_id": 0})],
        [_call("c5", "scan_table", {})],
        [_call("c6", "inspect_and_sort", {"part_id": 0})],
        [_call("c7", "write_report", {})],
        [_message("done"),
         _call("c8", "finish", {"summary": "2 good, 1 bad"})],
    ]


def _report_fn(tmp_path):
    def report(record_paths):
        return agent_tools.fake_report(
            record_paths, recordings_dir=str(tmp_path))
    return report


def test_fake_table_run_writes_live_page(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    fake = agent_tools.FakeTable()
    page = LivePage(tmp_path / "live.html")
    lines = []
    result = run_agent(FakeClient(_full_sort_script()), fake.scan,
                       fake.sort, _report_fn(tmp_path),
                       out=lines.append,
                       recordings_dir=str(tmp_path), live=page)
    assert result["finished"] is True
    assert result["good"] == 2 and result["bad"] == 1
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert '<meta http-equiv="refresh" content="2">' in text
    assert "good: 2" in text
    assert "bad: 1" in text
    assert "missed or failed: 1" in text
    assert "parts left: 0" in text
    assert "fake_table.png" in text
    assert text.count('class="card"') == 3
    assert "GOOD" in text and "BAD" in text
    assert "finished: 2 good, 1 bad" in text


def test_live_page_shows_rescan_image_after_sort(tmp_path):
    calls = []

    def scan(known=None):
        index = len(calls)
        calls.append(known)
        pos = [[500.0, 360.0]] if index == 0 else [[540.0, 330.0]]
        return {"parts": [{"table_mm": list(point)} for point in pos],
                "skipped": 0, "image": f"recordings/scan_{index}.png"}

    def sort(position, lever_deg=None):
        return {"outcome": "sorted", "levers": 3, "verdict": "good",
                "box": "good", "record": "recordings/r.json",
                "reason": None, "lines": ["dropped into the good box"]}

    page = LivePage(tmp_path / "live.html")
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "inspect_and_sort", {"part_id": 0})],
        [_message("done"), _call("c3", "finish", {"summary": "1 good"})],
    ]
    result = run_agent(FakeClient(script), scan, sort,
                       _report_fn(tmp_path),
                       out=lambda line: None,
                       recordings_dir=str(tmp_path), live=page)
    assert result["finished"] is True
    assert len(calls) == 2
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert "scan_1.png" in text
    assert "scan_0.png" not in text
    assert "parts left: 1" in text


def test_console_line_is_escaped(tmp_path):
    page = LivePage(tmp_path / "live.html")
    page.line("<script>alert(1)</script>")
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert "&lt;script&gt;" in text
    assert "<script>alert" not in text


def test_console_keeps_last_15_lines(tmp_path):
    page = LivePage(tmp_path / "live.html")
    for index in range(20):
        page.line(f"msg {index:02d}")
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert "msg 00" not in text
    assert "msg 19" in text


def test_status_lines_for_tool_calls(tmp_path):
    page = LivePage(tmp_path / "live.html")
    page.before_tool("scan_table", {})
    assert "scanning the table" in \
        (tmp_path / "live.html").read_text(encoding="utf-8")
    page.before_tool("inspect_and_sort", {"part_id": 2})
    assert "sorting part 2" in \
        (tmp_path / "live.html").read_text(encoding="utf-8")
    page.before_tool("write_report", {})
    assert "writing the report" in \
        (tmp_path / "live.html").read_text(encoding="utf-8")


def test_sorted_card_shows_record_crop(tmp_path):
    crop = tmp_path / "crop.png"
    crop.write_bytes(b"fake-png")
    record = tmp_path / "count_x.json"
    record.write_text(json.dumps({
        "crop_image": str(crop),
        "levers": 3,
        "verdict": "good",
    }), encoding="utf-8")
    page = LivePage(tmp_path / "live.html")
    page.sorted({"outcome": "sorted", "levers": 3, "verdict": "good",
                 "box": "good", "record": str(record)})
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert "crop.png" in text
    assert "levers 3" in text


def test_sorted_card_without_record_file(tmp_path):
    page = LivePage(tmp_path / "live.html")
    page.sorted({"outcome": "sorted", "levers": 2, "verdict": "bad",
                 "box": "bad", "record": str(tmp_path / "missing.json")})
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert text.count('class="card"') == 1
    assert "BAD" in text


def test_write_error_warns_once_and_run_finishes(
        monkeypatch, tmp_path, capsys):
    def boom(src, dst):
        raise OSError("disk full")

    monkeypatch.setattr("os.replace", boom)
    page = LivePage(tmp_path / "live.html")
    fake = agent_tools.FakeTable()
    lines = []
    result = run_agent(
        FakeClient([
            [_call("c1", "scan_table", {})],
            [_message("done"),
             _call("c2", "finish", {"summary": "ok"})],
        ]),
        fake.scan, fake.sort, _report_fn(tmp_path),
        out=lines.append, recordings_dir=str(tmp_path), live=page)
    assert result["finished"] is True
    warnings = [line for line in capsys.readouterr().out.splitlines()
                if line.startswith("live page:")]
    assert len(warnings) == 1
    assert "disk full" in warnings[0]


def test_run_output_unchanged_without_live(tmp_path):
    def run(live):
        fake = agent_tools.FakeTable()
        lines = []
        run_agent(
            FakeClient([
                [_call("c1", "scan_table", {})],
                [_message("done"),
                 _call("c2", "finish", {"summary": "ok"})],
            ]),
            fake.scan, fake.sort, _report_fn(tmp_path),
            out=lines.append, recordings_dir=str(tmp_path), live=live)
        return lines

    plain = run(None)
    with_live = run(LivePage(tmp_path / "live.html"))
    assert with_live == plain
    assert "agent finished: ok" in with_live
