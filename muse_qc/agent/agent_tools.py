"""Tools the Muse Spark agent loop can call. No hardware here directly.

scan_table, inspect_and_sort and write_report run as plain Python
functions; muse_qc.agent exposes them to the model as Responses API
function tools and executes them in order. Positions always come
from our own latest scan: the model only passes part ids, never
coordinates. The --simulate fakes (FakeTable, fake_report) run the
same tool surface without hardware, camera or network for tools.

Responses API assumption (unchecked against this repository, which
only uses output_text so far): function tools are passed as
{"type": "function", "name", "description", "parameters", "strict"}
inside tools=[...], a call arrives as an output item with
{"type": "function_call", "call_id", "name", "arguments"} where
arguments is a JSON string, and the result goes back as an input
item {"type": "function_call_output", "call_id": ..., "output": ...}
with output a JSON string. Reasoning is requested with
reasoning={"effort": "low"}, stateless replay with store=False and
include=["reasoning.encrypted_content"]. The human checks this in
the --simulate run.
"""

from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

from muse_qc.agent.agent_parse import parse_pick_lines as parse_pick_lines
from muse_qc.agent.agent_rescan import (
    rescan_after_sort as rescan_after_sort,
)
from muse_qc.agent.agent_rescan import store_scan as store_scan
from muse_qc.agent.agent_state import (
    SAME_PART_MM,
    RunState,
    _dist,
)

TOOL_DEFINITIONS = [
    {
        "type": "function",
        "name": "scan_table",
        "description": "Look at the table and list the unsorted parts.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": [],
            "additionalProperties": False,
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "inspect_and_sort",
        "description": (
            "Pick the part with this scan id, count its levers and "
            "drop it into the good or bad box. Ids come from the "
            "latest list: the scan_table result or the table "
            "after sort in the last inspect_and_sort result. "
            "Every sort rescans at once, so always use the ids "
            "from the latest list; a failed or missed sort "
            "expires the whole scan."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "part_id": {
                    "type": "integer",
                    "description": "Part id from the latest scan.",
                },
            },
            "required": ["part_id"],
            "additionalProperties": False,
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "write_report",
        "description": "Build the HTML report from this run's records.",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": [],
            "additionalProperties": False,
        },
        "strict": True,
    },
    {
        "type": "function",
        "name": "finish",
        "description": "End the run with a short summary.",
        "parameters": {
            "type": "object",
            "properties": {
                "summary": {
                    "type": "string",
                    "description": "Parts sorted good/bad and failures.",
                },
            },
            "required": ["summary"],
            "additionalProperties": False,
        },
        "strict": True,
    },
]

class FakeTable:
    """Fake table for --simulate: 3 parts, two good, one bad.

    No camera, no arm, no API. rescans list the parts still on the
    table; the second part misses the grip on its first try, then
    sorts normally. Records are fake paths collected for the fake
    report.
    """

    def __init__(self) -> None:
        self.parts = [
            {"table_mm": [500.0, 360.0], "levers": 3, "verdict": "good",
             "lever_deg": 89.8},
            {"table_mm": [540.0, 330.0], "levers": 3, "verdict": "good",
             "lever_deg": 31.9},
            {"table_mm": [560.0, 300.0], "levers": 2, "verdict": "bad",
             "lever_deg": None},
        ]
        self._missed_key = tuple(self.parts[1]["table_mm"])
        self._tried: set[tuple[float, float]] = set()
        self._records = 0
        self.seen_levers: list = []

    def scan(self, known=None) -> dict:
        """Return the parts still on the fake table.

        known is accepted like the real scan (confirmed points
        of the previous scan); the fake has no SAM to skip, so
        the same remaining parts come back either way.
        """
        return {
            "parts": [
                {"table_mm": list(part["table_mm"]),
                 "lever_deg": part["lever_deg"]}
                for part in self.parts
            ],
            "skipped": 0,
            "image": "recordings/fake_table.png",
        }

    def sort(self, table_mm, lever_deg=None) -> dict:
        """Sort the fake part at table_mm, with one missed grip."""
        self.seen_levers.append(lever_deg)
        key = (float(table_mm[0]), float(table_mm[1]))
        for part in self.parts:
            if _dist(table_mm, part["table_mm"]) <= SAME_PART_MM:
                break
        else:
            return {
                "outcome": "error",
                "levers": None,
                "verdict": None,
                "box": None,
                "record": None,
                "reason": "no fake part at that position",
                "lines": ["sort failed: no fake part at that position"],
            }
        if key == self._missed_key and key not in self._tried:
            self._tried.add(key)
            lines = ["missed grip"]
            return {
                "outcome": "missed_grip",
                "levers": None,
                "verdict": None,
                "box": None,
                "record": None,
                "reason": None,
                "lines": lines,
            }
        self.parts.remove(part)
        self._records += 1
        record = f"recordings/count_fake_{self._records:06d}.json"
        lines = [
            f"inspect: levers {part['levers']} verdict {part['verdict']}",
            f"dropped into the {part['verdict']} box",
            record,
        ]
        return {
            "outcome": "sorted",
            "levers": part["levers"],
            "verdict": part["verdict"],
            "box": part["verdict"],
            "record": record,
            "reason": None,
            "lines": lines,
        }


def fake_report(record_paths, recordings_dir="recordings",
               now=None) -> str:
    """Write a fake HTML report listing exactly these records.

    now is an injectable zero-argument clock returning a datetime
    (local time, like every other recordings file name).
    """
    ticks = now if now is not None else datetime.now
    target = Path(recordings_dir)
    target.mkdir(parents=True, exist_ok=True)
    stamp = ticks().strftime("%Y%m%d_%H%M%S")
    out_path = target / f"report_fake_{stamp}.html"
    items = "".join(f"<p>{path}</p>" for path in record_paths)
    out_path.write_text(
        "<html><body><h1>fake report</h1>"
        f"<p>parts: {len(record_paths)}</p>{items}</body></html>",
        encoding="utf-8",
    )
    return str(out_path)


def real_scan(known=None) -> dict:
    """Scan the real table (camera plus SAM). Only --execute.

    known holds the confirmed points of the previous scan;
    known parts skip SAM (first scan: known empty, all to SAM).
    """
    from muse_qc.agent.agent_rescan import scan_frame
    from muse_qc.vision.camera_top import open_table_camera

    try:
        frame, _ = open_table_camera()
    except (OSError, RuntimeError) as exc:
        raise RuntimeError(f"request failed: {exc}") from exc
    return scan_frame(frame, known)


def arm_child_tail(scan_at_station=False) -> list:
    """Shared arm flags for every pick child command.

    Both pick_test (one child per part) and pick_session (one
    child per plan) take exactly these arm arguments, so a new
    flag added here reaches both entry points and they cannot
    drift. Callers add only their target arguments (and
    --confirm-stdin for the session).
    """
    cmd = ["--inspect", "--execute"]
    if scan_at_station:
        cmd += ["--station", "--station-table-shot"]
    return cmd


def pick_session_cmd(scan_at_station=False) -> list:
    """Full pick_session child command for --arm-session (with confirm)."""
    return ([sys.executable, "-m", "muse_qc.pick.pick_session"]
            + arm_child_tail(scan_at_station) + ["--confirm-stdin"])


def real_sort(table_mm, lever_deg=None, popen_factory=None,
              timeout_s=None, clock=None, scan_at_station=False) -> dict:
    """Run one real pick_test inspect sort as a child. --execute only.

    Each sort is its own process so the operating system frees the
    serial port when the child exits. The child's yes prompt is
    answered on stdin (the human confirmed once for the whole run).
    lever_deg adds --lever-deg to the child command when not None.
    scan_at_station adds --station --station-table-shot so the
    table overview is taken during the station hold; without it
    the child command is exactly the command used so far.
    Returns the parsed result with the last lines and exit_code.
    """
    from muse_qc.agent.agent_io import SORT_TIMEOUT_S, default_popen, run_child

    limit = SORT_TIMEOUT_S if timeout_s is None else timeout_s
    factory = popen_factory if popen_factory is not None \
        else default_popen
    cmd = ([sys.executable, "-m", "muse_qc.pick.pick_test",
            "--table-mm", f"{float(table_mm[0])}",
            f"{float(table_mm[1])}"]
           + arm_child_tail(scan_at_station))
    if lever_deg is not None:
        cmd += ["--lever-deg", f"{float(lever_deg)}"]
    try:
        lines, exit_code, timed_out = run_child(
            cmd, factory, limit, clock)
    except Exception as exc:  # noqa: BLE001 - collected, no key
        lines, exit_code, timed_out = \
            [f"request failed: {exc}"], None, False
    if timed_out:
        lines = list(lines) + [f"sort timed out after {limit:g} s"]
    result = parse_pick_lines(lines)
    result["lines"] = lines[-12:]
    result["exit_code"] = exit_code
    return result


def real_report(record_paths, recordings_dir="recordings") -> str:
    """Build the real HTML report from exactly these records."""
    from muse_qc.inspection import report

    return str(report.write_report_from_records(
        record_paths, recordings_dir=recordings_dir))
