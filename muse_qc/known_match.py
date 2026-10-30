"""Cheap table rescan: already confirmed parts skip SAM. Pure, sends nothing.

After a sorted part the arm is back at rest, so the agent scans
again right away with known = the confirmed table_mm points of
the previous scan. A candidate within KNOWN_MATCH_MM of a known
point is confirmed without any SAM request (a slipped part gets
its new position from the diff); every other candidate goes
through SAM exactly as before.
"""

from __future__ import annotations

import math

KNOWN_MATCH_MM = 10.0


def _known_table_mm(entry):
    """Return the table_mm point of a known entry, or None.

    Entries are table_mm points ([x, y]) or dicts holding
    table_mm (plus confidence for the copy-back).
    """
    if isinstance(entry, dict):
        point = entry.get("table_mm")
    else:
        point = entry
    try:
        x, y = float(point[0]), float(point[1])
    except (TypeError, ValueError, IndexError):
        return None
    if not (math.isfinite(x) and math.isfinite(y)):
        return None
    return (x, y)


def match_known(known, table_mm) -> tuple:
    """Match table_mm against the known points; return (hit, conf, point).

    Hit when a known point is within KNOWN_MATCH_MM; confidence
    is copied from that known part (None for bare points) and
    point is the known table_mm to keep (the earlier SAM based
    position: the diff blob centre can be mm off through the
    part shadow). No match (or no known list) returns
    (False, None, None).
    """
    if not known:
        return (False, None, None)
    try:
        x, y = float(table_mm[0]), float(table_mm[1])
    except (TypeError, ValueError, IndexError):
        return (False, None, None)
    for entry in known:
        point = _known_table_mm(entry)
        if point is None:
            continue
        if math.dist((x, y), point) <= KNOWN_MATCH_MM:
            confidence = None
            if isinstance(entry, dict):
                try:
                    confidence = float(entry.get("confidence"))
                except (TypeError, ValueError):
                    confidence = None
            return (True, confidence, point)
    return (False, None, None)
