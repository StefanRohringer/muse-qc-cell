"""Chained pick check with the real rest reading from the dry run.

Offline only: fake solve_hover, constant valid tip, real safety.
The clamped rest pose has joint 2 at exactly 0.0, which safety
rejects as a live current reading; the chained pre-check must nudge
it while leaving commanded waypoints untouched.
"""

import pytest

from muse_qc import pick_plan, safety

REST = [-2.79, 1.10, -1.72, 2.72, 2.44, -1.91]
TABLE = (495.9, 330.0)
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}


def _fake_solve(table_xy, height_mm, seed):
    """Straight line in z: joints linear in the height."""
    height = float(height_mm)
    return {
        "ok": True,
        "joints_deg": (-3.0, -90.0 - 0.1 * height, -80.0, 75.0, 1.0, -4.0),
    }


def _valid_tip(joints_deg):
    """Constant reachable tip so only the joint-0.0 rule is exercised."""
    return (0.30, 0.0, 0.10)


def test_clamped_rest_zero_passes_real_safety():
    if safety.JOINT_LIMITS_DEG is None:
        pytest.skip("Seeed SDK joint limits unavailable")
    old_ref = safety.J6_REF
    safety.set_j6_ref(REST[5])
    try:
        built = pick_plan.build_pick_plan(REST, TABLE, 8.0, TEST_LIMITS,
                                          solve_fn=_fake_solve)
        assert built["ok"] is True
        assert built["rest_joints"][1] == 0.0
        ok, bad, res = pick_plan.check_pick_plan(built["plan"], REST,
                                                 tip_fn=_valid_tip)
        assert (ok, bad, res) == (True, -1, None)
        assert any(0.0 in wp for _, wp, _ in built["plan"])
    finally:
        safety.set_j6_ref(old_ref)
