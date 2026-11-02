"""Pick output parsing for the agent tools. Moved out of
muse_qc.agent.agent_tools so that module stays within the 400 line
limit. Pure string parsing, no hardware, no network.
"""

from __future__ import annotations

import re
from pathlib import Path

_INSPECT_RE = re.compile(r"inspect: levers (\d+) verdict (good|bad)")
_BOX_RE = re.compile(r"dropped into the (good|bad) box")
_STATION_TABLE_RE = re.compile(r"^station table shot: (\S+)\s*$")
_EXCEPTION_RE = re.compile(r"(?:[A-Za-z_]\w*\.)+([A-Z]\w*)\s*:.*$")


def _crash_reason(lines: list[str]) -> str:
    """Build an error reason from a crashed pick run (no result lines).

    Returns the last exception-like line's short reason: a joint
    feedback failure names the power supply, any other exception
    names only its class (never a traceback), and output without
    such a line reports that the program ended without a result.
    """
    name = None
    for line in lines:
        match = _EXCEPTION_RE.search(line.strip())
        if match is not None:
            name = match.group(1)
    if name == "JointStateUnavailable":
        return ("no joint feedback from the arm "
                "(is the arm power supply on?)")
    if name is not None:
        return f"arm program crashed: {name}"
    return "arm program ended without a result"


def parse_pick_lines(lines: list[str]) -> dict:
    """Parse collected pick_test output lines into a result dict.

    interrupted wins over everything (Ctrl+C aborts the pick, the
    agent run must stop), then sorted over missed_grip (the part
    still landed in a box), missed_grip over inspect_failed.
    An unverified part sorted into the bad box after "inspect
    failed" is sorted too, with the failed line kept as the
    reason; "inspect skipped" alone never counts as sorted.
    record is the last count_*.json path in the lines, or None.
    station_table_image is the "station table shot:" path, or None
    when the line is missing or the shot failed. An error with no
    known error line takes its reason from the last exception-like
    line (JointStateUnavailable names the power supply, any other
    exception only its class name, otherwise the program ended
    without a result); known error lines always win.
    """
    levers = None
    verdict = None
    box = None
    record = None
    station_table_image = None
    missed = False
    failed = None
    halted = None
    err_reason = None
    for line in lines:
        match = _INSPECT_RE.search(line)
        if match is not None:
            levers, verdict = int(match.group(1)), match.group(2)
        match = _BOX_RE.search(line)
        if match is not None:
            box = match.group(1)
        match = _STATION_TABLE_RE.search(line)
        if match is not None:
            station_table_image = match.group(1)
        if "missed grip" in line:
            missed = True
        if line.startswith(("inspect failed", "inspect skipped:",
                            "inspect timeout")):
            failed = line
        if "interrupted by human, holding position" in line:
            halted = line
        if line.startswith(("request failed:", "motion failed:",
                            "sort timed out")):
            err_reason = line
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
    if outcome == "error":
        reason = err_reason if err_reason is not None else _crash_reason(lines)
    else:
        reason = halted or failed
    return {
        "outcome": outcome,
        "levers": levers,
        "verdict": verdict,
        "box": box,
        "record": record,
        "station_table_image": station_table_image,
        "reason": reason,
    }
