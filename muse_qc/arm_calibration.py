"""Measured tip correction for the reBot B601-DM arm. Pure, offline, no bus.

What is corrected (human measurements of 2026-10-28, docs/DEVLOG.md):
base height BASE_Z_MM = -5.0 (joint axis centres power-off: joint 2
and 3 both 135 mm vs 140.2 mm in the model); tool length
TOOL_LENGTH_MM = 190.0 from the end of motor 6 (link6) to the finger
tips, with the tip offset beyond end_link derived from the model
(link6 to end_link distance, 155.39 mm); joint 3 sag
true_q3 = reported_q3 + delta3(q), fitted on DATA_POINTS below
(the real forearm points about 5 deg lower at the hover pose).

Limits of validity: ruler readings +-5 mm, repeat runs at the same
joints spread 15 mm (A1/B/C1 tip 75/65/60); F points use the
commanded settle joints (live reading unlogged, joint 2 up to 1 deg
higher); only valid near the measured poses (q3 around -65 to -95 deg
plus rest). The 100 mm hover rule stays until the human verifies the
correction with the same three hover points.
"""

from __future__ import annotations

import functools
import math

BASE_Z_MM = -5.0  # Table height = base_link z + BASE_Z_MM.
TOOL_LENGTH_MM = 190.0  # End of motor 6 (link6) to finger tips.

TOL_MM = 0.5  # solve_corrected stops below this 3D error.
MAX_ITER = 10  # solve_corrected gives up after this many IK solves.

# Joint 3 correction, fit on DATA_POINTS below (see fit_ratio/fit_load).
# Ratio form wins: fit RMS 5.45 mm; load form fit RMS 6.76 mm.
RATIO_K = -0.0556  # delta3 = k * q3, fit RMS 5.45 mm.
LOAD_G = 4.1395  # delta3 = g * cos(forearm angle), fit RMS 6.76 mm.
CHOSEN_FORM = "ratio"  # Lower RMS wins; ties would keep "ratio".

# (name, hold joints deg, model frame, measured z above table in mm).
DATA_POINTS = (
    ("A1", (-0.01, -97.45, -82.52, 75.07, -0.01, 0.05), "tip", 75.0),
    ("A2", (-0.01, -97.45, -82.52, 75.07, -0.01, 0.05), "link6", 263.0),
    ("A3", (-0.01, -97.45, -82.52, 75.07, -0.01, 0.05), "joint4", 365.0),
    ("A4", (-0.01, -97.45, -82.52, 75.07, -0.01, 0.05), "joint3", 395.0),
    ("B", (-0.01, -97.45, -82.52, 75.16, -0.03, -0.03), "tip", 65.0),
    ("C1", (-0.01, -97.45, -82.52, 75.11, -0.03, -0.03), "tip", 60.0),
    ("C2", (-0.01, -97.45, -82.52, 75.11, -0.03, -0.01), "link6", 260.0),
    ("D", (-0.01, -98.13, -94.85, 86.72, -0.01, -0.01), "tip", 130.0),
    ("E", (-0.01, -80.01, -65.49, 75.66, -0.01, -0.01), "tip", 80.0),
    ("F1", (-1.83, 0.00, -1.02, 1.67, 1.45, -0.47), "joint2", 135.0),
    ("F2", (-1.83, 0.00, -1.02, 1.67, 1.45, -0.47), "joint3", 135.0),
    ("F3", (-1.83, 0.00, -1.02, 1.67, 1.45, -0.47), "joint4", 190.0),
)


@functools.lru_cache(maxsize=1)
def _load_model() -> object:
    """Load the SDK robot model once; parsing the URDF every call is slow."""
    from reBotArm_control_py.kinematics import load_robot_model

    return load_robot_model()


def _frame_mm(joints_deg: tuple[float, ...], frame: str) -> tuple[float, ...]:
    """Return one model frame origin in mm for six joint angles in degrees."""
    import numpy as np
    from reBotArm_control_py.kinematics import (
        compute_fk,
        get_arm_joint_count,
        pad_q_for_model,
    )

    model = _load_model()
    full_q = pad_q_for_model(
        model,
        np.radians(np.array([float(v) for v in joints_deg])),
        controlled_joints=get_arm_joint_count(model),
    )
    position, _, _ = compute_fk(model, full_q, frame_name=frame)
    return tuple(float(v) * 1000.0 for v in position)


@functools.lru_cache(maxsize=1)
def link_span_mm() -> float:
    """Model distance from the link6 origin to the end_link origin in mm."""
    link6 = _frame_mm((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), "link6")
    end = _frame_mm((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), "end_link")
    return math.dist(link6, end)


def tip_beyond_end_mm(tool_length_mm: float | None = None) -> float:
    """Tip offset beyond end_link along the tool axis in mm."""
    if tool_length_mm is None:
        tool_length_mm = TOOL_LENGTH_MM
    return float(tool_length_mm) - link_span_mm()


def forearm_cos(joints_deg: object) -> float:
    """Cosine of the forearm elevation above horizontal at this pose.

    From the model joint3 to joint4 segment: horizontal length over
    total length, so a level forearm gives 1.0 either way up.
    """
    joints = _checked_joints(joints_deg)
    high = _frame_mm(tuple(joints), "joint4")
    low = _frame_mm(tuple(joints), "joint3")
    delta = [h - lo for h, lo in zip(high, low)]
    horizontal = math.hypot(delta[0], delta[1])
    return horizontal / math.sqrt(horizontal ** 2 + delta[2] ** 2)


def _checked_joints(joints_deg: object) -> list[float]:
    """Six finite joint angles in degrees, or ValueError."""
    joints = [float(v) for v in joints_deg]
    if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
        raise ValueError("Need six finite joint angles in degrees.")
    return joints


def true_joints_deg(joints_deg: object) -> tuple[float, ...]:
    """Reported joints with the fitted joint 3 correction applied."""
    joints = _checked_joints(joints_deg)
    if CHOSEN_FORM == "ratio":
        joints[2] += RATIO_K * joints[2]
    elif CHOSEN_FORM == "load":
        joints[2] += LOAD_G * forearm_cos(joints)
    else:
        raise ValueError(f"Unknown correction form {CHOSEN_FORM!r}.")
    return tuple(joints)


def model_table_z_mm(
    joints_deg: object,
    frame: str,
    delta3_deg: float,
    tool_length_mm: float | None = None,
    base_z_mm: float | None = None,
) -> float:
    """Model z above the table for one data point at a given delta3."""
    joints = _checked_joints(joints_deg)
    joints[2] += float(delta3_deg)
    if frame == "tip":
        end = _frame_mm(tuple(joints), "end_link")
        link6 = _frame_mm(tuple(joints), "link6")
        axis = [e - b for e, b in zip(end, link6)]
        norm = math.sqrt(sum(v * v for v in axis))
        if not math.isfinite(norm) or norm == 0.0:
            raise ValueError("Pose has no usable tool axis.")
        beyond = tip_beyond_end_mm(tool_length_mm)
        height = end[2] + beyond * axis[2] / norm
    elif frame in ("link6", "joint2", "joint3", "joint4"):
        height = _frame_mm(tuple(joints), frame)[2]
    else:
        raise ValueError(f"Unknown data frame {frame!r}.")
    if base_z_mm is None:
        base_z_mm = BASE_Z_MM
    return height + float(base_z_mm)


def _rms_for(form: str, param: float) -> float:
    """RMS residual in mm over DATA_POINTS for one parameter value."""
    total = 0.0
    for _, joints, frame, measured in DATA_POINTS:
        if form == "ratio":
            delta3 = param * joints[2]
        elif form == "load":
            delta3 = param * forearm_cos(joints)
        else:
            raise ValueError(f"Unknown correction form {form!r}.")
        total += (model_table_z_mm(joints, frame, delta3) - measured) ** 2
    return math.sqrt(total / len(DATA_POINTS))


def _fit_form(form: str, low: float, high: float) -> tuple[float, float]:
    """Best parameter and RMS for one form by deterministic grid refine."""
    for _ in range(8):
        best_param, best_rms = low, _rms_for(form, low)
        for index in range(1, 41):
            param = low + (high - low) * index / 40
            rms = _rms_for(form, param)
            if rms < best_rms:
                best_param, best_rms = param, rms
        step = (high - low) / 40
        low, high = best_param - 2.0 * step, best_param + 2.0 * step
    return best_param, best_rms


def fit_ratio() -> tuple[float, float]:
    """Best (k, rms) for delta3 = k * q3 on DATA_POINTS."""
    return _fit_form("ratio", -0.2, 0.2)


def fit_load() -> tuple[float, float]:
    """Best (g, rms) for delta3 = g * cos(forearm angle) on DATA_POINTS."""
    return _fit_form("load", -15.0, 15.0)


def fit_report_lines() -> list[str]:
    """One line per point (model z, residual, both forms) plus RMS lines."""
    k, rms_k = fit_ratio()
    g, rms_g = fit_load()
    lines = []
    for name, joints, frame, measured in DATA_POINTS:
        model_k = model_table_z_mm(joints, frame, k * joints[2])
        model_g = model_table_z_mm(joints, frame, g * forearm_cos(joints))
        lines.append(
            f"{name}: measured={measured:.1f} "
            f"ratio z={model_k:.1f} r={model_k - measured:+.1f} "
            f"load z={model_g:.1f} r={model_g - measured:+.1f}"
        )
    lines.append(f"ratio: k={k:.4f} rms={rms_k:.2f} mm")
    lines.append(f"load: g={g:.4f} deg rms={rms_g:.2f} mm")
    return lines


def tip_table_mm(joints_deg: object) -> tuple[float, float, float]:
    """Corrected tip: x, y in base frame mm, z above the table in mm.

    Forward kinematics of the true joints with TOOL_LENGTH_MM,
    shifted by BASE_Z_MM into table height.
    """
    true = true_joints_deg(joints_deg)
    end = _frame_mm(true, "end_link")
    link6 = _frame_mm(true, "link6")
    axis = [e - b for e, b in zip(end, link6)]
    norm = math.sqrt(sum(v * v for v in axis))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Pose has no usable tool axis.")
    beyond = tip_beyond_end_mm()
    return (
        end[0] + beyond * axis[0] / norm,
        end[1] + beyond * axis[1] / norm,
        end[2] + beyond * axis[2] / norm + BASE_Z_MM,
    )


def solve_corrected(
    target_base_xy_mm: tuple[float, float],
    tip_table_z_mm: float,
    seed_deg: object,
    *,
    tol_mm: float = TOL_MM,
    max_iterations: int = MAX_ITER,
    yaw_offset_deg: float = 0.0,
) -> dict:
    """Iterate the nominal IK so the corrected tip lands on the target.

    Uses target_pose's existing IK (no second implementation): solve
    the nominal target, measure the corrected tip of the result,
    shift the nominal target by the error and repeat until the 3D
    error is below tol_mm or max_iterations solves run. Returns the
    joints, the final corrected tip (base x/y mm, table z mm), the
    3D error and the iteration count; IK failure fails cleanly.
    """
    from muse_qc.target_pose import _solve_nominal, base_to_table_mm

    try:
        want = (
            float(target_base_xy_mm[0]),
            float(target_base_xy_mm[1]),
            float(tip_table_z_mm),
        )
        seed = _checked_joints(seed_deg)
        tol = float(tol_mm)
        limit = int(max_iterations)
    except (TypeError, ValueError, IndexError):
        return {"ok": False, "reason": "unreachable: bad corrected target"}
    if not all(math.isfinite(v) for v in want + (tol,)) or limit < 1:
        return {"ok": False, "reason": "unreachable: bad corrected target"}
    nominal = list(want)
    joints: tuple[float, ...] = tuple(seed)
    tip: tuple[float, float, float] = (0.0, 0.0, 0.0)
    error = math.inf
    iterations = 0
    current_seed: object = seed
    while iterations < limit:
        table_x, table_y = base_to_table_mm(nominal[0], nominal[1])
        solved = _solve_nominal((table_x, table_y), nominal[2],
                                current_seed,
                                yaw_offset_deg=yaw_offset_deg)
        if not solved.get("ok"):
            return {"ok": False, "reason": str(solved.get("reason"))}
        joints = solved["joints_deg"]
        iterations += 1
        tip = tip_table_mm(joints)
        error = math.dist(tip, want)
        if error <= tol:
            break
        nominal = [n + (w - t) for n, w, t in zip(nominal, want, tip)]
        current_seed = joints
    return {
        "ok": True,
        "joints_deg": joints,
        "tip_table_mm": tip,
        "error_mm": error,
        "iterations": iterations,
    }


if __name__ == "__main__":
    for line in fit_report_lines():
        print(line)
