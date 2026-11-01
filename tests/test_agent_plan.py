"""Tests for plan-once (--plan-once). Offline, fakes only.

The fake client returns scripted Responses-API-like outputs. No
network, no camera, no arm bus: scan/sort/report functions are
injected fakes. Without the flag the run behaves exactly as
before (tool not offered, unknown when called); with the flag
one sort_in_order call sorts the whole order and the model only
replans on a deviation.
"""

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from muse_qc.agent import agent_plan, agent_tools
from muse_qc.agent.agent import SYSTEM_PROMPT, run_agent
from muse_qc.agent.agent_plan import SORT_IN_ORDER_TOOL, run_plan
from muse_qc.agent.agent_state import RunState
from muse_qc.agent.agent_tools import FakeTable
from muse_qc.agent.live_page import LivePage

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
        return SimpleNamespace(output=self._script.pop(0))


class FakeClient:
    """Scripted Responses-API-like client; script holds outputs."""

    def __init__(self, script, seen=None):
        self.responses = _FakeResponses(
            script, seen if seen is not None else [])


def _scan_fn_factory(scans):
    """Fake scan_fn returning the queued part lists in order."""
    calls = []

    def scan(known=None):
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

    def sort(position, lever_deg=None):
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


def _missed_outcome():
    return {"outcome": "missed_grip", "levers": None,
            "verdict": None, "box": None, "record": None,
            "reason": None, "lines": ["missed grip"]}


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


def _transcript(tmp_path):
    paths = list(Path(str(tmp_path)).glob("agent_*.json"))
    assert len(paths) == 1
    return json.loads(paths[0].read_text(encoding="utf-8"))


def test_plan_sorts_all_with_one_call(tmp_path):
    fake = FakeTable()
    # The retry is already consumed, so this plan has no deviation.
    fake._tried.add(fake._missed_key)
    script = [
        [_message("scan"), _call("c1", "scan_table", {})],
        [_message("order"), _call("c2", "sort_in_order",
                                 {"part_ids": [0, 1, 2]})],
        [_message("check"), _call("c3", "scan_table", {})],
        [_message("report"), _call("c4", "write_report", {})],
        [_message("done"), _call("c5", "finish",
                                {"summary": "3 sorted: 2 good, 1 bad"})],
    ]
    result, lines, _ = _run(script, fake.scan, fake.sort,
                            _report_fn_factory(tmp_path), tmp_path,
                            plan_once=True)
    assert result["finished"] is True
    assert result["good"] == 2 and result["bad"] == 1
    assert result["model_calls"] == 5
    assert fake.parts == []
    text = "\n".join(lines)
    assert "plan: [0, 1, 2]" in text
    assert "agent: 5 model calls" in text
    loaded = _transcript(tmp_path)
    names = [step["calls"][0]["name"] for step in loaded["steps"]]
    assert names.count("sort_in_order") == 1
    plan = loaded["steps"][1]["results"][0]
    assert plan["outcome"] == "plan_done"
    assert len(plan["sorted"]) == 3


def test_missed_grip_deviates_and_replans(tmp_path):
    fake = FakeTable()
    script = [
        [_message("scan"), _call("c1", "scan_table", {})],
        [_message("order"), _call("c2", "sort_in_order",
                                 {"part_ids": [0, 1, 2]})],
        [_message("rescan"), _call("c3", "scan_table", {})],
        [_message("sort"), _call("c4", "inspect_and_sort",
                                {"part_id": 0})],
        [_message("sort"), _call("c5", "inspect_and_sort",
                                {"part_id": 0})],
        [_message("scan"), _call("c6", "scan_table", {})],
        [_message("report"), _call("c7", "write_report", {})],
        [_message("done"), _call("c8", "finish",
                                {"summary": "3 sorted, 1 missed grip"})],
    ]
    result, lines, _ = _run(script, fake.scan, fake.sort,
                            _report_fn_factory(tmp_path), tmp_path,
                            plan_once=True)
    assert result["finished"] is True
    assert result["good"] == 2 and result["bad"] == 1
    assert fake.parts == []
    text = "\n".join(lines)
    assert "deviation: part 0 missed its grip." in text
    loaded = _transcript(tmp_path)
    plan = loaded["steps"][1]["results"][0]
    assert plan["outcome"] == "plan_deviation"
    assert plan["deviation"] == "part 0 missed its grip."
    assert len(plan["sorted"]) == 1
    assert plan["sorted"][0]["verdict"] == "good"


def test_moved_part_deviates(tmp_path):
    moved = [P1[0] + 50.0, P1[1]]
    scan = _scan_fn_factory([[P0, P1], [moved]])
    sort = _sort_fn_factory([
        _sorted_outcome(3, "good", "a.json"),
    ])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "sort_in_order", {"part_ids": [0, 1]})],
        [_call("c3", "scan_table", {})],
        [_call("c4", "finish", {"summary": "replan needed"})],
    ]
    result, lines, _ = _run(script, scan, sort,
                            _report_fn_factory(tmp_path), tmp_path,
                            plan_once=True)
    assert result["finished"] is True
    assert len(sort.seen) == 1
    text = "\n".join(lines)
    assert "deviation: planned part 1 moved or is gone." in text
    loaded = _transcript(tmp_path)
    plan = loaded["steps"][1]["results"][0]
    assert plan["outcome"] == "plan_deviation"
    assert len(plan["sorted"]) == 1
    assert plan["table after sort"][0]["table_mm"] == moved


def test_new_part_deviates(tmp_path):
    extra = [600.0, 320.0]
    scan = _scan_fn_factory([[P0, P1], [P1, extra]])
    sort = _sort_fn_factory([
        _sorted_outcome(3, "good", "a.json"),
    ])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "sort_in_order", {"part_ids": [0, 1]})],
        [_call("c3", "finish", {"summary": "replan needed"})],
    ]
    result, lines, _ = _run(script, scan, sort,
                            _report_fn_factory(tmp_path), tmp_path,
                            plan_once=True)
    assert result["finished"] is True
    assert len(sort.seen) == 1
    assert "deviation: a new part appeared on the table." in \
        "\n".join(lines)
    loaded = _transcript(tmp_path)
    plan = loaded["steps"][1]["results"][0]
    assert plan["outcome"] == "plan_deviation"
    assert len(plan["table after sort"]) == 2


def test_failed_rescan_deviates(tmp_path):
    state = RunState()
    state.new_scan([P0, P1], [None, None], [None, None])
    lines = []

    def boom(known=None):
        raise RuntimeError("camera gone")

    result, stop = run_plan(
        state, [0, 1],
        _sort_fn_factory([
            _sorted_outcome(3, "good", "a.json"),
        ]), boom, out=lines.append)
    assert stop is False
    assert result["outcome"] == "plan_deviation"
    assert result["deviation"] == "the rescan failed."
    assert len(result["sorted"]) == 1
    assert "deviation: the rescan failed." in lines


def test_plan_limit_stops_loop(tmp_path):
    scan = _scan_fn_factory([[P0, P1], [P1]])
    sort = _sort_fn_factory([
        _sorted_outcome(3, "good", "a.json"),
        _sorted_outcome(2, "bad", "b.json"),
    ])
    script = [
        [_call("c1", "scan_table", {})],
        [_call("c2", "sort_in_order", {"part_ids": [0, 1]})],
    ]
    result, lines, _ = _run(script, scan, sort,
                            _report_fn_factory(tmp_path), tmp_path,
                            plan_once=True, max_sorts=1)
    assert result["finished"] is False
    assert len(sort.seen) == 1
    assert "sort limit reached (1)" in "\n".join(lines)


def test_plan_two_tries_per_position(tmp_path):
    state = RunState()
    state.new_scan([P0], [None], [None])
    lines = []
    sort = _sort_fn_factory([_missed_outcome(), _missed_outcome()])
    scan = _scan_fn_factory([[P0]])
    first, _ = run_plan(state, [0], sort, scan, out=lines.append)
    assert first["outcome"] == "plan_deviation"
    agent_tools.store_scan(state, scan())
    second, _ = run_plan(state, [0], sort, scan, out=lines.append)
    assert second["outcome"] == "plan_deviation"
    agent_tools.store_scan(state, scan())
    before = len(sort.seen)
    third, _ = run_plan(state, [0], sort, scan, out=lines.append)
    assert third["outcome"] == "plan_deviation"
    assert third["deviation"] == "part 0 refused."
    assert len(sort.seen) == before


def test_plan_order_validation(tmp_path):
    state = RunState()
    state.new_scan([P0, P1], [None, None], [None, None])
    scan = _scan_fn_factory([[P0, P1]])
    sort = _sort_fn_factory([])
    for bad in ([], [0, 0], [99], ["0"], [True], "nope", None):
        result, stop = run_plan(state, bad, sort, scan,
                                out=lambda line: None)
        assert result["outcome"] == "refused"
        assert stop is False
    assert sort.seen == []


def test_flag_off_tool_not_offered(tmp_path):
    seen = []
    lines = []
    client = FakeClient([
        [_message("try"), _call("c1", "sort_in_order",
                               {"part_ids": [0]})],
        [_message("scan"), _call("c2", "scan_table", {})],
        [_message("end"), _call("c3", "finish", {"summary": "ok"})],
    ], seen)
    result = run_agent(client, _scan_fn_factory([[P0]]),
                       _sort_fn_factory([]),
                       _report_fn_factory(tmp_path),
                       out=lines.append,
                       recordings_dir=str(tmp_path))
    assert result["finished"] is True
    assert seen[0]["instructions"] == SYSTEM_PROMPT
    assert {tool["name"] for tool in seen[0]["tools"]} == {
        "scan_table", "inspect_and_sort", "write_report", "finish"}
    assert "unknown tool sort_in_order" in "\n".join(lines)


def test_flag_on_offers_tool_and_prompt(tmp_path):
    seen = []
    client = FakeClient([
        [_message("end"), _call("c1", "finish", {"summary": "ok"})],
    ], seen)
    result = run_agent(client, _scan_fn_factory([[]]),
                       _sort_fn_factory([]),
                       _report_fn_factory(tmp_path),
                       out=lambda line: None,
                       recordings_dir=str(tmp_path), plan_once=True)
    assert result["finished"] is True
    assert {tool["name"] for tool in seen[0]["tools"]} == {
        "scan_table", "inspect_and_sort", "write_report", "finish",
        "sort_in_order"}
    assert seen[0]["instructions"].startswith(SYSTEM_PROMPT)
    assert "sort_in_order" in seen[0]["instructions"]
    planned = {tool["name"]: tool for tool in seen[0]["tools"]}
    params = planned["sort_in_order"]["parameters"]
    assert params["required"] == ["part_ids"]
    assert params["properties"]["part_ids"]["type"] == "array"
    assert params["properties"]["part_ids"]["items"] == {
        "type": "integer"}


def test_plan_run_writes_live_page(tmp_path):
    fake = FakeTable()
    fake._tried.add(fake._missed_key)
    page = LivePage(tmp_path / "live.html")
    lines = []
    result = run_agent(
        FakeClient([
            [_call("c1", "scan_table", {})],
            [_call("c2", "sort_in_order", {"part_ids": [0, 1, 2]})],
            [_call("c3", "scan_table", {})],
            [_message("done"), _call("c4", "finish",
                                    {"summary": "2 good, 1 bad"})],
        ]), fake.scan, fake.sort,
        _report_fn_factory(tmp_path),
        out=lines.append, recordings_dir=str(tmp_path), live=page,
        plan_once=True)
    assert result["finished"] is True
    text = (tmp_path / "live.html").read_text(encoding="utf-8")
    assert "good: 2" in text
    assert "bad: 1" in text
    assert text.count('class="card"') == 3
    assert "plan: [0, 1, 2]" in text


def test_tool_definition_describes_order():
    assert SORT_IN_ORDER_TOOL["name"] == "sort_in_order"
    assert "no" in SORT_IN_ORDER_TOOL["description"].lower()
    assert agent_plan.PLAN_PROMPT.count(".") == 1
