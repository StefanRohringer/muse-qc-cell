"""Tests for --arm-session (one pick_session child per plan). Offline, fakes only.

Fake child processes speak the session protocol with the real
line shapes ("session: part done", "session: end (...)"); the
agent side parses them with the existing parser. No bus, no
camera, no network.
"""

from __future__ import annotations

import signal
import threading

import pytest

from muse_qc.agent import agent_plan, agent_session
from muse_qc.agent.agent import main as agent_main
from muse_qc.agent.agent_plan import run_plan
from muse_qc.agent.agent_state import RunState

P0 = [500.0, 360.0]
P1 = [540.0, 330.0]


class FakeStdin:
    def __init__(self, proc):
        self._proc = proc

    def write(self, text):
        self._proc.writes.append(text)

    def flush(self):
        pass

    def close(self):
        self._proc.stdin_closed = True


class FakeStdout:
    def __init__(self, proc):
        self._proc = proc

    def readline(self):
        return self._proc.next_line()


class FakeProc:
    """Scripted pick_session child with the real line shapes."""

    def __init__(self, lines, exit_code=0):
        self._lines = list(lines)
        self._exit_code = exit_code
        self.stdin = FakeStdin(self)
        self.stdout = FakeStdout(self)
        self.writes: list = []
        self.stdin_closed = False
        self.signals: list = []
        self.killed = False

    def next_line(self):
        if self._lines:
            return self._lines.pop(0)
        return ""

    def poll(self):
        return None

    def wait(self, timeout=None):
        return self._exit_code

    def send_signal(self, number):
        self.signals.append(number)

    def kill(self):
        self.killed = True


def _part_block(levers=3, verdict="good", record="recordings/count_000001.json"):
    return [f"inspect: levers {levers} verdict {verdict}",
            f"dropped into the {verdict} box",
            record,
            "session: part done"]


def test_session_sort_parses_part_block():
    proc = FakeProc(_part_block() + ["session: end (end)"])
    session = agent_session.ArmSession(
        ["pick_session"], popen_factory=lambda cmd: proc, out=lambda text: None)
    result = session.sort(P0, 89.8)
    assert proc.writes[0] == "yes\n"
    assert proc.writes[1] == f"part {P0[0]} {P0[1]} 89.8\n"
    assert result["outcome"] == "sorted"
    assert (result["levers"], result["verdict"], result["box"]) == (3, "good", "good")
    assert result["record"] == "recordings/count_000001.json"
    assert result["exit_code"] == 0
    assert result["session_over"] is False
    session.close()
    assert proc.writes[-1] == "end\n"
    assert proc.stdin_closed is True
    assert session.over is True


def test_session_sort_without_lever_omits_it():
    proc = FakeProc(_part_block() + ["session: end (end)"])
    session = agent_session.ArmSession(
        ["pick_session"], popen_factory=lambda cmd: proc, out=lambda text: None)
    session.sort(P0)
    assert proc.writes[1] == f"part {P0[0]} {P0[1]}\n"
    session.close()


def test_session_end_line_during_sort_ends_session():
    proc = FakeProc(["session: end (eof)"])
    session = agent_session.ArmSession(
        ["pick_session"], popen_factory=lambda cmd: proc, out=lambda text: None)
    result = session.sort(P0, 89.8)
    assert result["session_over"] is True
    assert result["outcome"] == "error"
    session.close()


def test_session_early_exit_is_an_error():
    proc = FakeProc([])
    session = agent_session.ArmSession(
        ["pick_session"], popen_factory=lambda cmd: proc, out=lambda text: None)
    result = session.sort(P0, 89.8)
    assert result["outcome"] == "error"
    assert result["session_over"] is True


def test_session_timeout_sends_sigint():
    gate = threading.Event()

    class SlowProc(FakeProc):
        def next_line(self):
            gate.wait(timeout=30.0)
            return ""

    proc = SlowProc([])
    session = agent_session.ArmSession(
        ["pick_session"], popen_factory=lambda cmd: proc, timeout_s=0.2,
        close_grace_s=0.5, out=lambda text: None)
    result = session.sort(P0, 89.8)
    gate.set()
    assert signal.SIGINT in proc.signals
    assert result["outcome"] == "error"
    assert result["session_over"] is True


class FakeSession:
    """Scripted ArmSession surface for run_plan (sort/close only)."""

    def __init__(self, outcomes):
        self._outcomes = list(outcomes)
        self.sent: list = []
        self.closed: list = []

    def sort(self, position, lever_deg=None):
        self.sent.append((list(position), lever_deg))
        return dict(self._outcomes.pop(0))

    def close(self, reason="done"):
        self.closed.append(reason)


def _sorted_outcome(levers, verdict, record):
    return {"outcome": "sorted", "levers": levers, "verdict": verdict,
            "box": verdict, "record": record, "reason": None,
            "lines": [f"inspect: levers {levers} verdict {verdict}",
                      f"dropped into the {verdict} box", record]}


def _missed_outcome():
    return {"outcome": "missed_grip", "levers": None,
            "verdict": None, "box": None, "record": None,
            "reason": None, "lines": ["missed grip"]}


def _scan_fn_factory(scans):
    calls = []

    def scan(known=None):
        calls.append(True)
        parts = scans[min(len(calls) - 1, len(scans) - 1)]
        return {"parts": [{"table_mm": list(pos)} for pos in parts],
                "skipped": 0, "image": "recordings/fake.png"}

    return scan


def _state_two_parts():
    state = RunState()
    state.new_scan([P0, P1], [None, None], [None, None])
    return state


def test_run_plan_session_sorts_order_and_closes():
    made: list = []
    session = FakeSession([
        _sorted_outcome(2, "bad", "recordings/count_000001.json"),
        _sorted_outcome(3, "good", "recordings/count_000002.json"),
    ])
    scan = _scan_fn_factory([[P1], []])
    result, stop = run_plan(
        _state_two_parts(), [0, 1], None, scan, out=lambda text: None,
        session_factory=lambda: made.append(session) or session)
    assert result["outcome"] == "plan_done"
    assert [entry["verdict"] for entry in result["sorted"]] == ["bad", "good"]
    assert stop is False
    assert session.closed == ["done"]
    assert made == [session]


def test_run_plan_deviation_closes_and_replan_opens_new_session():
    made: list = []
    first = FakeSession([_missed_outcome()])
    scan = _scan_fn_factory([[P0, P1]])
    result, _ = run_plan(
        _state_two_parts(), [0, 1], None, scan, out=lambda text: None,
        session_factory=lambda: made.append(first) or first)
    assert result["outcome"] == "plan_deviation"
    assert first.closed == ["done"]
    second = FakeSession([
        _sorted_outcome(2, "bad", "recordings/count_000001.json"),
        _sorted_outcome(3, "good", "recordings/count_000002.json"),
    ])
    scan = _scan_fn_factory([[P1], []])
    result, _ = run_plan(
        _state_two_parts(), [0, 1], None, scan, out=lambda text: None,
        session_factory=lambda: made.append(second) or second)
    assert result["outcome"] == "plan_done"
    assert made == [first, second]


def test_run_plan_session_over_mid_order_is_deviation():
    outcome = _sorted_outcome(2, "bad", "recordings/count_000001.json")
    outcome["session_over"] = True
    session = FakeSession([outcome])
    scan = _scan_fn_factory([[P0, P1], [P1]])
    result, stop = run_plan(
        _state_two_parts(), [0, 1], None, scan, out=lambda text: None,
        session_factory=lambda: session)
    assert result["outcome"] == "plan_deviation"
    assert result["deviation"] == "the arm session ended."
    assert stop is False
    assert session.closed == ["done"]


def test_run_plan_without_factory_uses_sort_fn():
    seen: list = []
    scan = _scan_fn_factory([[P1], []])

    def sort_fn(position, lever_deg=None):
        seen.append(list(position))
        return _sorted_outcome(2, "bad", "recordings/count_000001.json") \
            if len(seen) == 1 else \
            _sorted_outcome(3, "good", "recordings/count_000002.json")

    result, _ = run_plan(_state_two_parts(), [0, 1], sort_fn, scan,
                         out=lambda text: None)
    assert result["outcome"] == "plan_done"
    assert seen == [P0, P1]


def test_pick_commands_share_arm_arguments():
    import sys

    from muse_qc.agent import agent_tools

    for scan in (False, True):
        tail = agent_tools.arm_child_tail(scan)
        assert agent_tools.pick_session_cmd(scan) == (
            [sys.executable, "-m", "muse_qc.pick.pick_session"]
            + tail + ["--confirm-stdin"])
    made = []

    def factory(cmd):
        made.append(cmd)
        return FakeProc([
            "inspect: levers 3 verdict good",
            "dropped into the good box",
            "recordings/count_000001.json",
        ])

    agent_tools.real_sort([478.7, 344.7], popen_factory=factory)
    agent_tools.real_sort([478.7, 344.7], popen_factory=factory,
                          lever_deg=30.0, scan_at_station=True)
    assert made[0] == (
        [sys.executable, "-m", "muse_qc.pick.pick_test",
         "--table-mm", "478.7", "344.7"]
        + agent_tools.arm_child_tail(False))
    assert made[1] == (
        [sys.executable, "-m", "muse_qc.pick.pick_test",
         "--table-mm", "478.7", "344.7"]
        + agent_tools.arm_child_tail(True) + ["--lever-deg", "30.0"])


def test_arm_session_flag_needs_plan_once_and_execute():
    with pytest.raises(SystemExit) as info:
        agent_main(["--arm-session"], input_fn=lambda prompt: "yes")
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        agent_main(["--plan-once", "--arm-session"],
                   input_fn=lambda prompt: "yes")
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        agent_main(["--execute", "--arm-session"],
                   input_fn=lambda prompt: "yes")
    assert info.value.code == 2
