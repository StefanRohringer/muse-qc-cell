"""Run state for the Muse Spark agent loop: latest scan and records.

Moved out of muse_qc.agent.agent_tools so that module stays within
the 400 line limit. Positions always come from our own latest
scan: the model only passes part ids, never coordinates.
"""

from __future__ import annotations

import math

SAME_PART_MM = 15.0


def _dist(first, second) -> float:
    """Return the table-mm distance of two [x, y] positions."""
    return math.dist((float(first[0]), float(first[1])),
                     (float(second[0]), float(second[1])))


class RunState:
    """Latest scan, attempts and records of one agent run."""

    def __init__(self) -> None:
        self.parts: dict[int, list[float]] = {}
        self.levers: dict[int, float | None] = {}
        self.confidences: dict[int, float | None] = {}
        self.fresh = False
        self.attempts: list[list[float]] = []
        self.sorted_positions: list[list[float]] = []
        self.records: list[str] = []
        self.sort_count = 0

    def new_scan(self, positions, levers=None,
                 confidences=None) -> dict[int, list[float]]:
        """Store the scan positions under fresh ids, return the mapping.

        levers holds the find_parts lever_deg per part (or None);
        lever_of returns it for real_sort's --lever-deg. confidences
        holds the SAM confidence per part (or None); known_parts
        returns both for the cheap rescan after a sorted part.
        """
        self.parts = {
            index: [float(pos[0]), float(pos[1])]
            for index, pos in enumerate(positions)
        }
        by_id = list(levers) if levers is not None else [None] * len(positions)
        self.levers = {
            index: (None if value is None else float(value))
            for index, value in enumerate(by_id)
        }
        by_conf = list(confidences) if confidences is not None \
            else [None] * len(positions)
        self.confidences = {
            index: (None if value is None else float(value))
            for index, value in enumerate(by_conf)
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

    def lever_of(self, part_id) -> float | None:
        """Return the stored lever_deg for part_id, or None.

        None means unknown, stale or no lever direction; the sort
        then runs without --lever-deg.
        """
        if isinstance(part_id, bool) or not isinstance(part_id, int):
            return None
        if not self.fresh:
            return None
        return self.levers.get(part_id)

    def known_parts(self) -> list[dict]:
        """Return the confirmed parts of the latest scan for a rescan.

        Each entry holds table_mm and confidence for the known
        match; an empty list means no fresh scan (known empty).
        """
        if not self.fresh:
            return []
        return [
            {"table_mm": [self.parts[index][0], self.parts[index][1]],
             "confidence": self.confidences.get(index)}
            for index in sorted(self.parts)
        ]

    def remove_sorted(self, part_id) -> None:
        """Drop one sorted id; the other ids of the scan stay valid."""
        self.parts.pop(part_id, None)
        self.levers.pop(part_id, None)
        self.confidences.pop(part_id, None)
        if not self.parts:
            self.fresh = False

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
