"""Gripper speed profile for the pick loop. Pure, moves nothing.

Opening (pre-open during hover, release over the box, miss/abort
re-open) runs at OPEN_RATE_DEG_S; closing runs at FAST_CLOSE_RATE_DEG_S
until the jaw width reaches FAST_CLOSE_UNTIL_MM, then at
SLOW_CLOSE_RATE_DEG_S until contact, so contact detection and the
squeeze behave exactly as before. step_toward advances one 100 Hz
command toward a target at a given step; close_step picks the step
from the current jaw width. GRIPPER_VLIM_FRACTION scales the gripper
POS_VEL vlim so the configured cap allows the fast rate.
"""

from __future__ import annotations

from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ

OPEN_RATE_DEG_S = 40.0
FAST_CLOSE_RATE_DEG_S = 40.0
SLOW_CLOSE_RATE_DEG_S = 15.0
FAST_CLOSE_UNTIL_MM = 22.0
GRIPPER_VLIM_FRACTION = 0.25
OPEN_STEP_DEG = OPEN_RATE_DEG_S / LOOP_RATE_HZ
FAST_CLOSE_STEP_DEG = FAST_CLOSE_RATE_DEG_S / LOOP_RATE_HZ
SLOW_CLOSE_STEP_DEG = SLOW_CLOSE_RATE_DEG_S / LOOP_RATE_HZ


def step_toward(cmd_deg: float, target_deg: float,
                step_deg: float) -> tuple[float, bool]:
    """One command toward target at step_deg. Returns (next, done)."""
    diff = float(target_deg) - float(cmd_deg)
    if abs(diff) <= float(step_deg):
        return float(target_deg), True
    return float(cmd_deg) + (1.0 if diff > 0.0 else -1.0) * float(step_deg), False


def close_step(cmd_deg: float, target_deg: float,
               width_mm: float) -> tuple[float, bool]:
    """One closing command: fast above FAST_CLOSE_UNTIL_MM, slow below."""
    step = (FAST_CLOSE_STEP_DEG if float(width_mm) > FAST_CLOSE_UNTIL_MM
            else SLOW_CLOSE_STEP_DEG)
    return step_toward(cmd_deg, target_deg, step)
