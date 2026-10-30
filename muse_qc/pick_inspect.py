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
