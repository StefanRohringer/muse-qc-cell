"""Inspect-hold phase of the pick loop: settle, verdict, sort.

After the lift finishes with the part held, PickLoop holds the last
lift waypoint here while the main thread inspects the part in the
gripper (see muse_qc.inspect_step). A "good" or "bad" verdict walks
the to-box waypoints of that box. A missing verdict or a timeout
sorts the unverified part into the bad box with exit code 1; the
gripper is never opened above the table. Pure loop logic, moves
nothing by itself.
"""
from __future__ import annotations

from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.pick_plan import SEG_TOSTATION


def do_inspect_hold(loop) -> None:
    """Run one 100 Hz inspect-hold cycle on a PickLoop."""
    from muse_qc import pick_loop  # local import: pick_loop calls back here

    if not loop._hold():
        return
    if loop._settle_left > 0:
        loop._settle_left -= 1
        if loop._settle_left > 0:
            return
        loop.inspect_ready.set()
        loop.left = int(pick_loop.INSPECT_TIMEOUT_S * LOOP_RATE_HZ)
        return
    if loop._verdict == "pending":
        loop.left -= 1
        if loop.left > 0:
            return
        loop._out("inspect timeout, sorting into the bad box (unverified part)")
        loop.exit_code = 1
        loop._inspected = True
        loop._sort_box("bad")
        return
    box = loop._verdict
    loop._inspected = True
    if box is None:
        loop.exit_code = 1
        loop._out(
            "inspect failed, sorting into the bad box (unverified part)")
        loop._sort_box("bad")
        return
    loop._sort_box(box)


def _settle_cycles() -> int:
    """Inspect settle cycles; single source stays in pick_loop."""
    from muse_qc import pick_loop  # local import: pick_loop calls back here

    return int(pick_loop.INSPECT_SETTLE_S * LOOP_RATE_HZ)


def _station_shared(loop) -> dict:
    """Shared ranges holding the to-station segment, or {} without station."""
    shared = getattr(loop, "shared_ranges", None) or {}
    if SEG_TOSTATION not in shared:
        return {}
    return shared


def after_lift(loop) -> None:
    """Walk on after the lift: station, inspect hold, drop or put-down."""
    if _station_shared(loop) and SEG_TOSTATION in loop.ranges:
        if loop.lost:
            loop.inspect_skipped = True
            loop._start_abort()
            return
        loop._walk_seg(SEG_TOSTATION)
        return
    if loop.inspect and not loop._inspected and loop.ranges_by_box:
        if loop.lost:
            loop.inspect_skipped = True
            loop._start_abort()
            return
        loop._settle_left = int(_settle_cycles())
        loop.phase = "inspect_hold"
        return
    if loop.drop_box is None:
        loop._walk_seg("put-down")
    else:
        loop._walk_seg("to-box")


def after_station(loop) -> None:
    """Hold for inspection after reaching the station. Part stays held."""
    if loop.inspect and not loop._inspected and loop.ranges_by_box:
        if loop.lost:
            loop.inspect_skipped = True
            loop._start_abort()
            return
        loop._settle_left = int(_settle_cycles())
        loop.phase = "inspect_hold"
        return
    if loop.drop_box is None:
        loop._walk_seg("put-down")
    else:
        loop._walk_seg("to-box")


def sort_box(loop, box) -> None:
    """Take the inspection verdict box and walk its to-box waypoints."""
    loop.drop_box = box
    loop.ranges = {**loop.shared_ranges, **loop.ranges_by_box[box]}
    loop._walk_seg("to-box")


def station_abort_detour(loop) -> list[int] | None:
    """Abort path during to-station or at the station. Planned waypoints only.

    From the held position walks the to-station segment forward to
    the station again, then the good branch to-box and return
    waypoints to rest. The gripper is untouched, so the part stays
    held, exactly like today's abort after the lift. Returns None
    when the station is not in play or the loop is elsewhere.
    """
    shared = _station_shared(loop)
    if not shared:
        return None
    seg_low, seg_high = shared[SEG_TOSTATION]
    pos = loop.pos
    at_station = loop.phase == "inspect_hold"
    in_seg = pos is not None and seg_low <= pos < seg_high
    at_end = pos is not None and pos == seg_high - 1
    if not (in_seg or (at_station and (at_end or in_seg))):
        return None
    by_box = getattr(loop, "ranges_by_box", None) or {}
    branch = by_box.get("good", loop.ranges)
    try:
        box_low, box_high = branch["to-box"]
        ret_low, ret_high = branch["return"]
    except (KeyError, TypeError):
        return None
    hold = [] if pos is None else [pos]
    forward = [] if at_end else list(range(pos + 1, seg_high))
    return (hold + forward + list(range(box_low, box_high))
            + list(range(ret_low, ret_high)))
