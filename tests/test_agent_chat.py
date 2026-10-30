"""Tests for human orders by chat (--task and --chat). Offline.

Scripted Responses-API-like clients and a scripted input_fn drive
main with the FakeTable tools; no network, no camera, no serial
or bus. The tools and their refusals are untouched: the order
only changes which parts the model asks for.
"""

from __future__ import annotations

import json
from types import SimpleNamespace

import muse_qc.agent as agent_mod
from muse_qc import agent_tools
from muse_qc.agent import main

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
    answers = iter(["sort the rest", ""])
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
    assert prompts == ["next order (empty line to quit): "] * 2
    assert len(client.seen) == 4
    assert len(client.seen[2]["input"]) > len(client.seen[0]["input"])
    assert "Start the inspection" in _first_user_text(client.seen[0])
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
    answers = iter(["sort the rest", ""])
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
