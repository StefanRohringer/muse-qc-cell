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

import math
import re
from datetime import datetime, timezone
from pathlib import Path

SAME_PART_MM = 15.0

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
            "latest scan_table result and expire after every sort."
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

_INSPECT_RE = re.compile(r"inspect: levers (\d+) verdict (good|bad)")
_BOX_RE = re.compile(r"dropped into the (good|bad) box")


def _dist(first, second) -> float:
    """Return the table-mm distance of two [x, y] positions."""
    return math.dist((float(first[0]), float(first[1])),
                     (float(second[0]), float(second[1])))


class RunState:
    """Latest scan, attempts and records of one agent run."""

    def __init__(self) -> None:
        self.parts: dict[int, list[float]] = {}
        self.fresh = False
        self.attempts: list[list[float]] = []
        self.sorted_positions: list[list[float]] = []
        self.records: list[str] = []
        self.sort_count = 0

    def new_scan(self, positions) -> dict[int, list[float]]:
        """Store the scan positions under fresh ids, return the mapping."""
        self.parts = {
            index: [float(pos[0]), float(pos[1])]
            for index, pos in enumerate(positions)
        }
        self.fresh = True
        return dict(self.parts)

    def lookup(self, part_id) -> list[float] | None:
        """Return the stored position for part_id, or None.

        None means unknown or stale: not an int id, no fresh scan,
        or the id is not from the latest scan. Extra coordinates
        from the model are never consulted.
        """
        if isinstance(part_id, bool) or not isinstance(part_id, int):
            return None
        if not self.fresh:
            return None
        found = self.parts.get(part_id)
        return [found[0], found[1]] if found is not None else None

    def mark_stale(self) -> None:
        """Expire the latest scan; the table changed after a sort."""
        self.fresh = False

    def attempts_at(self, position) -> int:
        """Count inspect_and_sort calls near this position."""
        return sum(1 for seen in self.attempts
                   if _dist(seen, position) <= SAME_PART_MM)

    def note_attempt(self, position) -> None:
        """Record one inspect_and_sort call at this position."""
        self.attempts.append([float(position[0]), float(position[1])])

    def already_sorted(self, position) -> bool:
        """True when a part was already sorted near this position."""
        return any(_dist(seen, position) <= SAME_PART_MM
                   for seen in self.sorted_positions)

    def note_sorted(self, position) -> None:
        """Remember a successful sort at this position."""
        self.sorted_positions.append(
            [float(position[0]), float(position[1])])


def parse_pick_lines(lines: list[str]) -> dict:
    """Parse collected pick_test output lines into a result dict.

    interrupted wins over everything (Ctrl+C aborts the pick, the
    agent run must stop), then sorted over missed_grip (the part
    still landed in a box), missed_grip over inspect_failed.
    record is the last count_*.json path in the lines, or None.
    """
    levers = None
    verdict = None
    box = None
    record = None
    missed = False
    failed = None
    halted = None
    for line in lines:
        match = _INSPECT_RE.search(line)
        if match is not None:
            levers, verdict = int(match.group(1)), match.group(2)
        match = _BOX_RE.search(line)
        if match is not None:
            box = match.group(1)
        if "missed grip" in line:
            missed = True
        if line.startswith("inspect failed:"):
            failed = line
        if "interrupted by human, holding position" in line:
            halted = line
        stripped = line.strip()
        if stripped.endswith(".json") and "count_" in Path(stripped).name:
            record = stripped
    if halted is not None:
        outcome = "interrupted"
    elif box is not None:
        outcome = "sorted"
    elif missed:
        outcome = "missed_grip"
    elif failed is not None:
        outcome = "inspect_failed"
    else:
        outcome = "error"
    return {
        "outcome": outcome,
        "levers": levers,
        "verdict": verdict,
        "box": box,
        "record": record,
        "reason": halted or failed,
    }


class FakeTable:
    """Fake table for --simulate: 3 parts, two good, one bad.

    No camera, no arm, no API. rescans list the parts still on the
    table; the second part misses the grip on its first try, then
    sorts normally. Records are fake paths collected for the fake
    report.
    """

    def __init__(self) -> None:
        self.parts = [
            {"table_mm": [500.0, 360.0], "levers": 3, "verdict": "good"},
            {"table_mm": [540.0, 330.0], "levers": 3, "verdict": "good"},
            {"table_mm": [560.0, 300.0], "levers": 2, "verdict": "bad"},
        ]
        self._missed_key = tuple(self.parts[1]["table_mm"])
        self._tried: set[tuple[float, float]] = set()
        self._records = 0

    def scan(self) -> dict:
        """Return the parts still on the fake table."""
        return {
            "parts": [
                {"table_mm": list(part["table_mm"])}
                for part in self.parts
            ],
            "skipped": 0,
            "image": "recordings/fake_table.png",
        }

    def sort(self, table_mm) -> dict:
        """Sort the fake part at table_mm, with one missed grip."""
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


def fake_report(record_paths, recordings_dir="recordings") -> str:
    """Write a fake HTML report listing exactly these records."""
    target = Path(recordings_dir)
    target.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out_path = target / f"report_fake_{stamp}.html"
    items = "".join(f"<p>{path}</p>" for path in record_paths)
    out_path.write_text(
        "<html><body><h1>fake report</h1>"
        f"<p>parts: {len(record_paths)}</p>{items}</body></html>",
        encoding="utf-8",
    )
    return str(out_path)


def real_scan() -> dict:
    """Scan the real table (camera plus SAM). Only --execute."""
    import cv2

    from muse_qc import find_parts
    from muse_qc.camera_top import open_table_camera
    from muse_qc.model_check import KEY_NAME, read_api_key
    from muse_qc.vision_sam import _default_client_factory

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
        frame, empty, find_parts.DEFAULT_PHRASE, client)
    parts = [
        {"table_mm": [float(item["table_mm"][0]),
                      float(item["table_mm"][1])],
         "confidence": item["confidence"]}
        for item in results
        if item["skipped"] is None and item["confirmed"]
    ]
    skipped = sum(1 for item in results if item["skipped"] is not None)
    return {"parts": parts, "skipped": skipped, "image": str(path)}


def real_sort(table_mm, input_fn) -> dict:
    """Run one real pick_test inspect sort at table_mm. --execute only."""
    from muse_qc import pick_test

    lines: list[str] = []
    try:
        pick_test.main(
            ["--table-mm", f"{float(table_mm[0])}",
             f"{float(table_mm[1])}", "--inspect", "--execute"],
            input_fn=input_fn, out=lines.append)
    except Exception as exc:  # noqa: BLE001 - collected, key never printed
        lines.append(f"request failed: {exc}")
    result = parse_pick_lines(lines)
    result["lines"] = lines[-12:]
    return result


def real_report(record_paths, recordings_dir="recordings") -> str:
    """Build the real HTML report from exactly these records."""
    from muse_qc.inspection import report

    return str(report.write_report_from_records(
        record_paths, recordings_dir=recordings_dir))
