"""Tests for human orders by chat (--task and --chat). Offline.

Scripted Responses-API-like clients and a scripted input_fn drive
main with the FakeTable tools; no network, no camera, no serial
or bus. The tools and their refusals are untouched: the order
only changes which parts the model asks for.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import muse_qc.agent.agent as agent_mod
from muse_qc.agent import agent_tools
from muse_qc.agent.agent import main
from muse_qc.agent.agent_chat import run_session

SECRET = "fake-test-key-abc123"


def _call(call_id, name, args):
    return {"type": "function_call", "call_id": call_id, "name": name,
            "arguments": json.dumps(args)}


class FakeClient:
    """Scripted Responses-API-like client recording its requests."""

    def __init__(self, script):
        self.responses = self
        self._script = list(script)
        self.seen = []

    def create(self, **kwargs):
        self.seen.append(
            {**kwargs, "input": list(kwargs.get("input", []))})
        if not self._script:
            raise AssertionError("model called more often than scripted")
        return SimpleNamespace(output=self._script.pop(0))


def _scan_and_finish(summary):
    return [[_call("c1", "scan_table", {})],
            [_call("c2", "finish", {"summary": summary})]]


def _toolset(tmp_path):
    fake = agent_tools.FakeTable()

    def report(record_paths):
        return agent_tools.fake_report(
            record_paths, recordings_dir=str(tmp_path))

    return (fake.scan, fake.sort, report)


def _first_user_text(request):
    return request["input"][0]["content"][0]["text"]


def test_task_replaces_first_order(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    client = FakeClient(_scan_and_finish("right half done"))
    lines = []
    code = main(["--task", "sort only the right half"],
                client_factory=lambda key: client,
                toolset=_toolset(tmp_path), out=lines.append)
    assert code == 0
    assert _first_user_text(client.seen[0]) == "sort only the right half"
    text = "\n".join(lines)
    assert "order: sort only the right half" in text
    assert "agent finished: right half done" in text
    page = (tmp_path / "recordings" / "live.html").read_text(
        encoding="utf-8")
    assert "order: sort only the right half" in page


def test_chat_two_orders_then_quit(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    script = _scan_and_finish("first done") + _scan_and_finish("rest done")
    client = FakeClient(script)
    answers = iter(["sort the good ones", "sort the rest", ""])
    prompts = []
    lines = []
    code = main(["--chat"],
                client_factory=lambda key: client,
                toolset=_toolset(tmp_path),
                input_fn=lambda prompt: (prompts.append(prompt),
                                         next(answers))[1],
                out=lines.append)
    assert code == 0
    text = "\n".join(lines)
    assert text.count("agent finished:") == 2
    assert prompts == ["order (empty line to quit): ",
                       "next order (empty line to quit): ",
                       "next order (empty line to quit): "]
    assert len(client.seen) == 4
    assert len(client.seen[2]["input"]) > len(client.seen[0]["input"])
    assert _first_user_text(client.seen[0]) == "sort the good ones"
    second_first = client.seen[2]["input"]
    assert "sort the rest" in second_first[-1]["content"][0]["text"]


def test_chat_execute_asks_yes_once(monkeypatch, tmp_path):
    # toolset None keeps the real "yes" gate; the model only calls
    # finish, so no real tool (no camera, no arm) ever runs.
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    script = [[_call("c1", "finish", {"summary": "first done"})],
              [_call("c2", "finish", {"summary": "rest done"})]]
    client = FakeClient(script)
    answers = iter(["yes", "sort the rest", ""])
    prompts = []
    lines = []
    code = main(["--execute", "--chat", "--task", "sort all parts"],
                client_factory=lambda key: client,
                input_fn=lambda prompt: (prompts.append(prompt),
                                         next(answers))[1],
                out=lines.append)
    assert code == 0
    assert sum("Type yes" in prompt for prompt in prompts) == 1
    assert sum("next order" in prompt for prompt in prompts) == 2
    assert "\n".join(lines).count("agent finished:") == 2
    assert _first_user_text(client.seen[0]) == "sort all parts"


def test_second_order_needs_new_scan(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    script = (_scan_and_finish("first done")
              + [[_call("c3", "inspect_and_sort", {"part_id": 0}),
                  _call("c4", "finish", {"summary": "rest done"})]])
    client = FakeClient(script)
    fake = agent_tools.FakeTable()
    sorts_before = list(fake.parts)
    answers = iter(["sort the good ones", "sort the rest", ""])
    lines = []
    code = main(
        ["--chat"],
        client_factory=lambda key: client,
        toolset=(fake.scan, fake.sort,
                 lambda records: agent_tools.fake_report(
                     records, recordings_dir=str(tmp_path))),
        input_fn=lambda prompt: next(answers),
        out=lines.append)
    assert code == 0
    text = "\n".join(lines)
    assert text.count("agent finished:") == 2
    assert "call scan_table first" in text
    assert fake.parts == sorts_before


def test_chat_first_order_asked_before_run(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    client = FakeClient(_scan_and_finish("good ones done"))
    answers = iter(["sort the good ones", ""])
    prompts = []
    lines = []
    code = main(["--chat"],
                client_factory=lambda key: client,
                toolset=_toolset(tmp_path),
                input_fn=lambda prompt: (prompts.append(prompt),
                                         next(answers))[1],
                out=lines.append)
    assert code == 0
    assert prompts[0] == "order (empty line to quit): "
    assert _first_user_text(client.seen[0]) == "sort the good ones"
    assert "order: sort the good ones" in "\n".join(lines)


def test_chat_empty_first_order_runs_nothing(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    client = FakeClient([])
    fake = agent_tools.FakeTable()
    sorts_before = list(fake.parts)
    prompts = []
    lines = []
    code = main(
        ["--chat"],
        client_factory=lambda key: client,
        toolset=(fake.scan, fake.sort,
                 lambda records: agent_tools.fake_report(
                     records, recordings_dir=str(tmp_path))),
        input_fn=lambda prompt: (prompts.append(prompt), "")[1],
        out=lines.append)
    assert code == 1
    assert prompts == ["order (empty line to quit): "]
    assert client.seen == []
    assert fake.parts == sorts_before


def test_chat_with_task_asks_no_first_order(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    client = FakeClient(_scan_and_finish("all done"))
    answers = iter([""])
    prompts = []
    lines = []
    code = main(["--chat", "--task", "sort all parts"],
                client_factory=lambda key: client,
                toolset=_toolset(tmp_path),
                input_fn=lambda prompt: (prompts.append(prompt),
                                         next(answers))[1],
                out=lines.append)
    assert code == 0
    assert prompts == ["next order (empty line to quit): "]
    assert _first_user_text(client.seen[0]) == "sort all parts"


def test_no_task_no_chat_unchanged(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    client = FakeClient(_scan_and_finish("plain run"))
    lines = []
    code = main([], client_factory=lambda key: client,
                toolset=_toolset(tmp_path), out=lines.append)
    assert code == 0
    assert "Start the inspection" in _first_user_text(client.seen[0])
    assert not [line for line in lines if line.startswith("order:")]


P_A = [500.0, 360.0]
P_NEAR = [505.0, 363.0]


def _order_scan(queued):
    """Fake scan_fn returning the queued position lists in order."""
    calls = []

    def scan(known=None):
        index = min(len(calls), len(queued) - 1)
        calls.append(True)
        return {"parts": [{"table_mm": list(pos)}
                           for pos in queued[index]],
                "skipped": 0, "image": "recordings/fake.png"}

    scan.calls = calls  # type: ignore[attr-defined]
    return scan


def _order_sort(outcomes):
    """Fake sort_fn playing the queued outcome dicts in order."""
    calls = []

    def sort(table_mm, lever_deg=None):
        calls.append([float(table_mm[0]), float(table_mm[1])])
        return dict(outcomes[min(len(calls) - 1, len(outcomes) - 1)])

    sort.calls = calls  # type: ignore[attr-defined]
    return sort


def _sorted_outcome(record):
    return {"outcome": "sorted", "levers": 3, "verdict": "good",
            "box": "good", "record": record,
            "lines": ["inspect: levers 3 verdict good",
                      "dropped into the good box", record]}


_MISSED_OUTCOME = {"outcome": "missed_grip", "levers": None,
                   "verdict": None, "box": None, "record": None,
                   "reason": None, "lines": ["missed grip"]}


def test_same_order_near_sorted_position_refused(tmp_path):
    client = FakeClient(
        [[_call("c1", "scan_table", {})],
         [_call("c2", "inspect_and_sort", {"part_id": 0})],
         [_call("c3", "inspect_and_sort", {"part_id": 0})],
         [_call("c4", "finish", {"summary": "one order done"})]])
    scan = _order_scan([[P_A], [P_NEAR]])
    sort = _order_sort([_sorted_outcome("recordings/fake_1.json")])
    lines = []
    result = run_session(
        client, scan, sort,
        lambda records: agent_tools.fake_report(
            records, recordings_dir=str(tmp_path)),
        task="sort all parts", chat=False, out=lines.append,
        recordings_dir=str(tmp_path))
    assert result["finished"] is True
    assert result["good"] == 1
    assert sort.calls == [P_A]
    assert "part already sorted" in "\n".join(lines)


def test_new_chat_order_clears_sorted_positions(tmp_path):
    client = FakeClient(
        [[_call("c1", "scan_table", {})],
         [_call("c2", "inspect_and_sort", {"part_id": 0})],
         [_call("c3", "finish", {"summary": "first done"})],
         [_call("c4", "scan_table", {})],
         [_call("c5", "inspect_and_sort", {"part_id": 0})],
         [_call("c6", "finish", {"summary": "rest done"})]])
    scan = _order_scan([[P_A], [], [P_A], []])
    sort = _order_sort([_sorted_outcome("recordings/fake_1.json"),
                        _sorted_outcome("recordings/fake_2.json")])
    answers = iter(["sort the rest", ""])
    lines = []
    result = run_session(
        client, scan, sort,
        lambda records: agent_tools.fake_report(
            records, recordings_dir=str(tmp_path)),
        task="sort all parts", chat=True,
        ask=lambda prompt: next(answers), out=lines.append,
        recordings_dir=str(tmp_path))
    assert result["finished"] is True
    assert result["good"] == 1
    assert sort.calls == [P_A, P_A]
    assert "part already sorted" not in "\n".join(lines)


def test_new_chat_order_retries_exhausted_position(tmp_path):
    client = FakeClient(
        [[_call("c1", "scan_table", {})],
         [_call("c2", "inspect_and_sort", {"part_id": 0})],
         [_call("c3", "scan_table", {})],
         [_call("c4", "inspect_and_sort", {"part_id": 0})],
         [_call("c5", "scan_table", {})],
         [_call("c6", "inspect_and_sort", {"part_id": 0})],
         [_call("c7", "finish", {"summary": "first done"})],
         [_call("c8", "scan_table", {})],
         [_call("c9", "inspect_and_sort", {"part_id": 0})],
         [_call("c10", "finish", {"summary": "rest done"})]])
    scan = _order_scan([[P_A], [P_A], [P_A], [P_A], []])
    sort = _order_sort([_MISSED_OUTCOME, _MISSED_OUTCOME,
                        _sorted_outcome("recordings/fake_1.json")])
    answers = iter(["sort the rest", ""])
    lines = []
    result = run_session(
        client, scan, sort,
        lambda records: agent_tools.fake_report(
            records, recordings_dir=str(tmp_path)),
        task="sort all parts", chat=True,
        ask=lambda prompt: next(answers), out=lines.append,
        recordings_dir=str(tmp_path))
    text = "\n".join(lines)
    assert "already tried twice" in text
    assert result["finished"] is True
    assert result["good"] == 1
    assert sort.calls == [P_A, P_A, P_A]


def test_task_mode_keeps_order_memory(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    client = FakeClient(
        [[_call("c1", "scan_table", {})],
         [_call("c2", "inspect_and_sort", {"part_id": 0})],
         [_call("c3", "inspect_and_sort", {"part_id": 0})],
         [_call("c4", "finish", {"summary": "task done"})]])
    scan = _order_scan([[P_A], [P_NEAR]])
    sort = _order_sort([_sorted_outcome("recordings/fake_1.json")])
    lines = []
    code = main(
        ["--task", "sort all parts"],
        client_factory=lambda key: client,
        toolset=(scan, sort,
                 lambda records: agent_tools.fake_report(
                     records, recordings_dir=str(tmp_path))),
        out=lines.append)
    assert code == 0
    assert sort.calls == [P_A]
    assert "part already sorted" in "\n".join(lines)
