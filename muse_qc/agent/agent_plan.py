"""Plan-once execution for the agent loop (--plan-once).

The model decides the sorting order once after the first scan and
Python executes that plan part by part without asking the model
again, as long as every rescan matches the plan. Only on a
deviation the model is called again and replans. The model stays
the decision maker; it simply is not asked when nothing changed.

sort_single holds the one-part sort with the inspect_and_sort
limits (MAX_SORTS, MAX_PER_POSITION, never sort a part twice);
the agent loop uses it for inspect_and_sort too, so both paths
sort exactly the same way. run_plan loops it over the ordered
ids and matches the remaining planned parts to every rescan by
position (within SAME_PART_MM).
"""

from __future__ import annotations

from muse_qc.agent.agent_rescan import rescan_after_sort
from muse_qc.agent.agent_state import SAME_PART_MM, _dist

DEFAULT_MAX_SORTS = 10
DEFAULT_MAX_PER_POSITION = 2

SORT_IN_ORDER_TOOL = {
    "type": "function",
    "name": "sort_in_order",
    "description": (
        "Sort the parts with these scan ids in this order; "
        "execution stops and reports back at the first "
        "deviation. Ids come from the latest list with no "
        "duplicates."
    ),
    "parameters": {
        "type": "object",
        "properties": {
            "part_ids": {
                "type": "array",
                "items": {"type": "integer"},
                "description": "Part ids from the latest scan, no duplicates.",
            },
        },
        "required": ["part_ids"],
        "additionalProperties": False,
    },
    "strict": True,
}

PLAN_PROMPT = (
    " After the first scan choose an order (for example nearest"
    " the arm first) and call sort_in_order once, then when the"
    " plan result reports no deviation call scan_table once more"
    " before write_report and finish, and replan only when a"
    " result reports a deviation."
)


def validate_order(state, part_ids) -> str | None:
    """Check the planned ids; return the refusal reason or None."""
    if not isinstance(part_ids, list) or not part_ids:
        return "empty order, call scan_table first"
    seen = set()
    for item in part_ids:
        if isinstance(item, bool) or not isinstance(item, int):
            return f"unknown part id {item}, call scan_table first"
        if item in seen:
            return f"duplicate part id {item}"
        seen.add(item)
        if state.lookup(item) is None:
            return f"unknown part id {item}, call scan_table first"
    return None


def sort_single(state, part_id, sort_fn, scan_fn, out=print,
                max_sorts=DEFAULT_MAX_SORTS,
                max_per_position=DEFAULT_MAX_PER_POSITION):
    """Sort one part with the inspect_and_sort limits.

    Returns (result, stop_loop) exactly like the agent loop tool:
    the same checks, the same sort_fn call, the same rescan after
    a sorted part and the same stop on limit or interrupt.
    """
    position = state.lookup(part_id)
    if position is None:
        result = {"outcome": "refused",
                  "reason": "unknown part id,"
                  " call scan_table first"}
        out(f"tool: part {part_id} refused: {result['reason']}")
        return result, False
    if state.already_sorted(position):
        result = {"outcome": "refused",
                  "reason": "part already sorted"}
        out(f"tool: part {part_id} refused: {result['reason']}")
        return result, False
    if state.attempts_at(position) >= max_per_position:
        result = {"outcome": "refused",
                  "reason": "already tried twice, skipping part"}
        out(f"tool: part {part_id} refused: {result['reason']}")
        return result, False
    if state.sort_count >= max_sorts:
        out(f"agent: sort limit reached ({max_sorts}), stopping")
        return {"outcome": "refused",
                "reason": "sort limit reached"}, True
    state.note_attempt(position)
    state.sort_count += 1
    lever = state.lever_of(part_id)
    try:
        result = dict(sort_fn(position, lever))
    except Exception as exc:  # noqa: BLE001 - short, no key
        result = {"outcome": "error", "levers": None,
                  "verdict": None, "box": None, "record": None,
                  "reason": f"sort failed: {exc}"}
    result["lines"] = list(result.get("lines") or [])[-12:]
    outcome = result.get("outcome")
    if outcome == "sorted":
        if result.get("record"):
            state.records.append(str(result["record"]))
        state.note_sorted(position)
        out(f"tool: part {part_id} levers {result.get('levers')}"
            f" {result.get('verdict')}, sorted")
        result.update(rescan_after_sort(
            state, scan_fn, out,
            station_image=result.get("station_table_image")))
    else:
        # Any other outcome (missed grip, error, inspect_failed,
        # interrupted, refused): the table may have changed, so
        # the whole scan expires and the model must rescan.
        state.mark_stale()
    if outcome == "interrupted":
        out(f"tool: part {part_id} interrupted")
        return result, True
    if outcome == "missed_grip":
        out(f"tool: part {part_id} missed grip")
    elif outcome == "inspect_failed":
        out(f"tool: part {part_id} inspect failed")
    elif outcome == "refused":
        out(f"tool: part {part_id} refused:"
            f" {result.get('reason')}")
    elif outcome != "sorted":
        out(f"tool: part {part_id} error: {result.get('reason')}")
    return result, False


def _deviation_sentence(part_id, result) -> str:
    """Return the one-sentence deviation for a failed single sort."""
    outcome = result.get("outcome")
    if outcome == "missed_grip":
        return f"part {part_id} missed its grip."
    if outcome == "inspect_failed":
        return f"part {part_id} inspection failed."
    if outcome == "interrupted":
        return f"part {part_id} interrupted."
    if outcome == "refused":
        return f"part {part_id} refused."
    return f"part {part_id} failed."


def _report(sorted_entries, sentence, update, outcome=None) -> dict:
    """Build the plan_deviation result with the current table list."""
    result: dict = {"outcome": "plan_deviation",
                    "sorted": sorted_entries,
                    "deviation": sentence}
    if outcome is not None:
        result["deviation_outcome"] = outcome
    for key in ("table after sort", "image after sort", "rescan"):
        if key in update:
            result[key] = update[key]
    return result


def run_plan(state, part_ids, sort_fn, scan_fn, out=print,
             max_sorts=DEFAULT_MAX_SORTS,
             max_per_position=DEFAULT_MAX_PER_POSITION,
             session_factory=None):
    """Execute the ordered ids; return (result, stop_loop).

    Every id is sorted with sort_single and rescanned as usual;
    the remaining planned parts are matched to every rescan by
    position. The first deviation stops the plan: what was sorted
    so far, the deviation in one short sentence and the current
    table list with new ids. A full plan returns plan_done with
    the sorted list plus the final rescan result. With
    session_factory (a zero-argument ArmSession maker for
    --arm-session) one child serves the whole order: sorts run
    through the session and it is closed ("end") on every exit.
    """
    if session_factory is None:
        return _execute_plan(state, part_ids, sort_fn, scan_fn, out,
                             max_sorts, max_per_position)
    session = session_factory()
    try:
        return _execute_plan(state, part_ids, session.sort, scan_fn, out,
                             max_sorts, max_per_position)
    finally:
        try:
            session.close()
        except Exception:  # noqa: BLE001 - the plan result wins
            pass


def _execute_plan(state, part_ids, sort_fn, scan_fn, out=print,
                  max_sorts=DEFAULT_MAX_SORTS,
                  max_per_position=DEFAULT_MAX_PER_POSITION):
    """Run one order part by part; see run_plan."""
    reason = validate_order(state, part_ids)
    if reason is not None:
        out(f"tool: sort_in_order refused: {reason}")
        return {"outcome": "refused", "reason": reason}, False
    pending = list(part_ids)
    expected = {pid: state.lookup(pid) for pid in pending}
    out(f"plan: {pending}")
    sorted_entries: list[dict] = []
    index = 0
    while index < len(pending):
        pid = pending[index]
        result, stop = sort_single(
            state, pid, sort_fn, scan_fn, out, max_sorts,
            max_per_position)
        if result.get("outcome") == "interrupted":
            sentence = _deviation_sentence(pid, result)
            out(f"deviation: {sentence}")
            return _report(sorted_entries, sentence, result,
                           "interrupted"), True
        if stop:
            done = dict(result)
            done["sorted"] = sorted_entries
            return done, True
        if result.get("outcome") != "sorted":
            sentence = _deviation_sentence(pid, result)
            out(f"deviation: {sentence}")
            return _report(
                sorted_entries, sentence,
                {"rescan": "scan expired, call scan_table"},
                result.get("outcome")), False
        sorted_entries.append({
            "part_id": pid,
            "levers": result.get("levers"),
            "verdict": result.get("verdict"),
            "box": result.get("box"),
            "record": result.get("record"),
        })
        remaining = pending[index + 1:]
        if remaining and result.get("session_over"):
            sentence = "the arm session ended."
            out(f"deviation: {sentence}")
            return _report(sorted_entries, sentence, result,
                           "session_over"), False
        if not remaining:
            done = {"outcome": "plan_done",
                    "sorted": sorted_entries}
            for key in ("table after sort", "image after sort",
                        "rescan"):
                if key in result:
                    done[key] = result[key]
            return done, False
        new_parts = result.get("table after sort")
        if new_parts is None:
            sentence = "the rescan failed."
            out(f"deviation: {sentence}")
            return _report(sorted_entries, sentence, result,
                           "error"), False
        matched = {}
        used: set = set()
        for old in remaining:
            found = None
            for part in new_parts:
                if part.get("id") in used:
                    continue
                if _dist(expected[old],
                         part["table_mm"]) <= SAME_PART_MM:
                    found = part
                    break
            if found is None:
                sentence = (f"planned part {old} moved or is gone.")
                out(f"deviation: {sentence}")
                return _report(sorted_entries, sentence, result,
                               "moved"), False
            matched[old] = found
            used.add(found.get("id"))
        for part in new_parts:
            if part.get("id") not in used:
                sentence = "a new part appeared on the table."
                out(f"deviation: {sentence}")
                return _report(sorted_entries, sentence, result,
                               "new_part"), False
        for pos, old in enumerate(remaining):
            fresh = matched[old]["id"]
            pending[index + 1 + pos] = fresh
            expected[fresh] = matched[old]["table_mm"]
        index += 1
    return {"outcome": "plan_done",
            "sorted": sorted_entries}, False
