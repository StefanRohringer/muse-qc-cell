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

from datetime import datetime
from pathlib import Path

from muse_qc.agent.agent_parse import parse_pick_lines as parse_pick_lines
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

def store_scan(state, found) -> dict:
    """Store a scan in state under new ids; return the tool result part.

    Same shape as a scan_table call: {"parts", "skipped", "image"}
    with one {"id", "table_mm", "confidence", "lever_deg"} dict
    per part. The state is fresh again under new ids.
    """
    items = found.get("parts", [])
    positions = [part["table_mm"] for part in items]
    state.new_scan(
        positions,
        [part.get("lever_deg") for part in items],
        [part.get("confidence") for part in items],
    )
    skipped = int(found.get("skipped", 0) or 0)
    return {"parts": [
        {"id": index,
         "table_mm": [float(pos[0]), float(pos[1])],
         "confidence": part.get("confidence"),
         "lever_deg": (None if part.get("lever_deg") is None
                       else round(float(part["lever_deg"]), 1))}
        for index, (pos, part) in enumerate(zip(positions, items))
    ], "skipped": skipped, "image": found.get("image")}


def rescan_after_sort(state, scan_fn, out=print) -> dict:
    """Rescan right after a sorted part; return the result update.

    The pick_test child has ended (arm at rest, motors off), so
    the scan runs at once with known = the confirmed points of
    the previous scan: known parts skip SAM, a slipped part gets
    its new position. The new scan is stored under new ids like
    a scan_table call and returned as "table after sort" with
    the same part fields. When the rescan fails or finds
    nothing, the scan expires and the update says so; the model
    then calls scan_table as usual. Prints nothing but the
    one-line rescan outcome through out.
    """
    known = state.known_parts()
    try:
        found = scan_fn(known=known)
    except Exception as exc:  # noqa: BLE001 - short, no key
        state.mark_stale()
        out(f"tool: rescan failed: {exc}")
        return {"rescan": f"failed: {exc}, call scan_table"}
    items = found.get("parts", []) if isinstance(found, dict) else []
    if not items:
        state.mark_stale()
        out("tool: rescan found no parts, call scan_table")
        return {"table after sort": [],
                "rescan": "no parts found, call scan_table"}
    result = store_scan(state, found)
    count = len(result["parts"])
    if count == 1:
        out("tool: rescan found 1 part")
    else:
        out(f"tool: rescan found {count} parts")
    return {"table after sort": result["parts"],
            "image after sort": result["image"]}


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
    import cv2

    from muse_qc.vision import find_parts
    from muse_qc.vision.camera_top import open_table_camera
    from muse_qc.checks.model_check import KEY_NAME, read_api_key
    from muse_qc.vision.vision_sam import _default_client_factory

    try:
        frame, _ = open_table_camera()
    except (OSError, RuntimeError) as exc:
        raise RuntimeError(f"request failed: {exc}") from exc
    empty = cv2.imread(find_parts.EMPTY_PATH)
    if empty is None:
        raise RuntimeError(
            f"missing empty reference {find_parts.EMPTY_PATH}:"
            " take one with --save-empty")
    key = read_api_key()
    if not (key or "").strip():
        raise RuntimeError(
            f"{KEY_NAME} is missing: set it in .env or the environment")
    try:
        client = _default_client_factory(key)
    except Exception as exc:  # noqa: BLE001 - mapped, key never printed
        raise RuntimeError(
            f"request failed: {exc}") from exc
    results, path = find_parts.scan_table(
        frame, empty, find_parts.DEFAULT_PHRASE, client,
        known=known)
    parts = [
        {"table_mm": [float(item["table_mm"][0]),
                      float(item["table_mm"][1])],
         "confidence": item["confidence"],
         "lever_deg": item["lever_deg"]}
        for item in results
        if item["skipped"] is None and item["confirmed"]
    ]
    skipped = sum(1 for item in results if item["skipped"] is not None)
    return {"parts": parts, "skipped": skipped, "image": str(path)}


def real_sort(table_mm, lever_deg=None, popen_factory=None,
              timeout_s=None, clock=None) -> dict:
    """Run one real pick_test inspect sort as a child. --execute only.

    Each sort is its own process so the operating system frees the
    serial port when the child exits. The child's yes prompt is
    answered on stdin (the human confirmed once for the whole run).
    lever_deg adds --lever-deg to the child command when not None.
    Returns the parsed result with the last lines and exit_code.
    """
    import sys

    from muse_qc.agent.agent_io import SORT_TIMEOUT_S, default_popen, run_child

    limit = SORT_TIMEOUT_S if timeout_s is None else timeout_s
    factory = popen_factory if popen_factory is not None \
        else default_popen
    cmd = [sys.executable, "-m", "muse_qc.pick.pick_test",
           "--table-mm", f"{float(table_mm[0])}",
           f"{float(table_mm[1])}", "--inspect", "--execute"]
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
