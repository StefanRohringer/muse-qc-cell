"""Tests for the box-exit transit. Offline, fakes only.

No bus, no camera, no network: the transit ramp is checked
pure, and one spliced plan runs the real safety.check_target
with fake limits and a fake tip.
"""

from __future__ import annotations

from muse_qc.arm import safety
from muse_qc.pick import box_transit, pick_plan
from muse_qc.pick.box_transit import build_box_transit, splice_hover
from muse_qc.pick.pick_plan import check_pick_plan

BOX_EXIT = (-3.5, -99.03, -80.0, 75.0, 1.0, -4.0)
HOVER = (-3.0, -100.0, -80.0, 75.0, 1.0, -4.0)
REST = [-3.33, -99.0, -79.0, 74.0, 0.91, -3.97]
TABLE = (495.9, 322.7)
TEST_LIMITS = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (-180.0, 0.0),
    "joint4": (-107.0, 90.0),
    "joint5": (-90.0, 90.0),
    "joint6": (-180.0, 180.0),
}


def _high_tip(joints_deg):
    """Tip well above the transit floor, inside the workspace."""
    return (0.30, 0.0, 0.12)


def _low_tip(joints_deg):
    """Tip below the transit floor."""
    return (0.30, 0.0, 0.01)


def _fake_solve(table_xy, height_mm, seed):
    height = float(height_mm)
    return {
        "ok": True,
        "joints_deg": (-3.0, -90.0 - 0.1 * height, -80.0, 75.0, 1.0, -4.0),
        "tip_base_mm": (300.0, 0.0, height),
        "tip_table_mm": (300.0, 0.0, height),
        "error_mm": 0.0,
        "tilt_deg": 0.0,
        "iterations": 1,
    }


def _fake_tip(joints_deg):
    height = -(float(joints_deg[1]) + 90.0) * 10.0
    return (0.30, 0.0, height / 1000.0)


def test_transit_ramps_box_to_hover_with_hover_label():
    transit = build_box_transit(BOX_EXIT, HOVER, tip_fn=_high_tip)
    assert transit["ok"]
    plan = transit["plan"]
    assert len(plan) > 2
    assert all(seg == "hover" for seg, _, _ in plan)
    assert all(grasp is False for _, _, grasp in plan)
    assert all(abs(a - b) <= 1.0 for a, b in zip(plan[0][1], BOX_EXIT))
    assert tuple(plan[-1][1]) == HOVER
    assert transit["ranges"] == {"hover": (0, len(plan))}
    assert transit["lowest_tip_z_mm"] >= 90.0


def test_transit_uses_same_speed_and_accel():
    fast = build_box_transit(BOX_EXIT, HOVER, free_speed_deg_s=50.0,
                             accel_deg_s2=100.0, tip_fn=_high_tip)
    slow = build_box_transit(BOX_EXIT, HOVER, free_speed_deg_s=5.0,
                             accel_deg_s2=15.0, tip_fn=_high_tip)
    assert fast["ok"] and slow["ok"]
    assert len(fast["plan"]) < len(slow["plan"])


def test_transit_floor_refusal_names_waypoint():
    transit = build_box_transit(BOX_EXIT, HOVER, tip_fn=_low_tip)
    assert not transit["ok"]
    assert transit["reason"].startswith("transit too low")


def test_transit_bad_joints_refused():
    assert build_box_transit(None, HOVER)["ok"] is False
    assert build_box_transit(BOX_EXIT, (1.0, 2.0))["ok"] is False


def test_splice_reindexes_ranges_and_branches():
    built = {
        "plan": [("hover", (1.0,) * 6, False)] * 3
        + [("lower", (2.0,) * 6, True)] * 2
        + [("return", (3.0,) * 6, False)] * 2,
        "ranges": {"hover": (0, 3), "lower": (3, 5), "return": (5, 7)},
        "shared_ranges": {"hover": (0, 3), "lower": (3, 5)},
        "ranges_by_box": {"good": {"return": (5, 7)}},
    }
    transit = {"ok": True, "plan": [("hover", (0.0,) * 6, False)] * 4,
               "ranges": {"hover": (0, 4)}}
    spliced = splice_hover(built, transit)
    assert [seg for seg, _, _ in spliced["plan"][:4]] == ["hover"] * 4
    assert spliced["ranges"]["hover"] == (0, 4)
    assert spliced["ranges"]["lower"] == (4, 6)
    assert spliced["ranges"]["return"] == (6, 8)
    assert spliced["shared_ranges"]["lower"] == (4, 6)
    assert spliced["ranges_by_box"]["good"]["return"] == (6, 8)
    assert len(built["plan"]) == 7  # input unchanged


def _two_branch_built():
    """Minimal inspect-shaped plan with distinct release joints per box."""
    good_drop = (-3.5, -99.03, -80.0, 75.0, 1.0, -4.0)
    bad_drop = (-5.0, -99.47, -81.0, 75.0, 1.5, -4.5)
    rest = (0.0, -90.0, -80.0, 75.0, 1.0, -4.0)
    plan = ([("hover", (0.0,) * 6, False)] * 2
            + [("to-box", good_drop, False)] * 2
            + [("release", good_drop, False)]
            + [("return", rest, False)] * 2
            + [("to-box", bad_drop, False)] * 2
            + [("release", bad_drop, False)]
            + [("return", rest, False)] * 2)
    return {
        "plan": plan,
        "ranges": {"hover": (0, 2), "to-box": (2, 4),
                   "release": (4, 5), "return": (5, 7)},
        "shared_ranges": {"hover": (0, 2)},
        "ranges_by_box": {
            "good": {"to-box": (2, 4), "release": (4, 5),
                     "return": (5, 7)},
            "bad": {"to-box": (7, 9), "release": (9, 10),
                    "return": (10, 12)},
        },
        "rest_joints": rest,
    }


def test_retarget_return_points_both_branches_at_rest():
    from muse_qc.pick.box_transit import retarget_return

    built = _two_branch_built()
    target = (-3.33, -99.0, -79.0, 74.0, 0.91, -3.97)
    result = retarget_return(built, target, free_speed_deg_s=50.0,
                             accel_deg_s2=100.0)
    assert result["ok"]
    fixed = result["built"]
    for box in ("good", "bad"):
        low, high = fixed["ranges_by_box"][box]["return"]
        assert tuple(fixed["plan"][high - 1][1]) == target
        assert tuple(fixed["plan"][low][1]) != target or (high - low) == 1
    good_low, good_high = fixed["ranges_by_box"]["good"]["return"]
    bad_low, bad_high = fixed["ranges_by_box"]["bad"]["return"]
    assert (bad_low, bad_high) == (good_high + 3, good_high + 3
                                   + (bad_high - bad_low))
    assert fixed["ranges"]["return"] == fixed["ranges_by_box"]["good"]["return"]
    assert fixed["ranges_by_box"]["good"]["to-box"] == (2, 4)
    assert fixed["plan"][2:5] == built["plan"][2:5]
    assert fixed["rest_joints"] == target
    assert len(built["plan"]) == 12  # input unchanged


def test_retarget_return_bad_rest_refused():
    from muse_qc.pick.box_transit import retarget_return

    result = retarget_return(_two_branch_built(), (1.0, 2.0))
    assert result["ok"] is False


def test_spliced_plan_passes_real_safety_check():
    base = pick_plan.build_pick_plan(
        REST, TABLE, 8.0, TEST_LIMITS, solve_fn=_fake_solve,
        tip_fn=_fake_tip)
    assert base["ok"]
    transit = build_box_transit(
        BOX_EXIT, base["hover_joints"], free_speed_deg_s=50.0,
        accel_deg_s2=100.0, tip_fn=_fake_tip)
    assert transit["ok"]
    spliced = splice_hover(base, transit)
    old_limits, old_ref = safety.JOINT_LIMITS_DEG, safety.J6_REF
    safety.JOINT_LIMITS_DEG = TEST_LIMITS
    safety.set_j6_ref(BOX_EXIT[5])
    try:
        ok, bad, res = check_pick_plan(
            spliced["plan"], BOX_EXIT, safety.check_target, _fake_tip)
    finally:
        safety.JOINT_LIMITS_DEG = old_limits
        safety.set_j6_ref(old_ref)
    assert ok, f"waypoint {bad}: {res}"
