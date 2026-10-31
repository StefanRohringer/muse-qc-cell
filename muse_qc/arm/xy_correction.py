"""x/y tip correction for the reBot B601-DM arm. Pure, offline, no bus.

Human measurements of 2026-10-28 (method in docs/SETUP.md section 27):
the table camera measured the white 25 mm cube (table_mm A), the arm
held a 100 mm hover over A, the human slid the cube exactly under the
finger tips, and the camera measured the cube again (table_mm B). B is
where the real tip was when commanded to A.

Model in arm base polar coordinates (base x forward, y left, origin on
the joint 1 axis): the real tip is rotated left and reaches further out,
real_angle = cmd_angle + YAW_DEG, real_r = cmd_r + R_ERR(cmd_r) with
R_ERR linear in cmd_r. YAW_DEG is the mean of the three yaw errors and
R_ERR is the least-squares line over the three radial errors. Outside
the fitted radius range R_ERR holds the end value (no extrapolation).
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from muse_qc.arm.target_pose import table_to_base_mm

# ((commanded table_mm A), (measured table_mm B)) per point.
MEASUREMENTS = (
    ((394.6, 318.6), (376.5, 331.3)),
    ((505.0, 348.7), (491.6, 371.8)),
    ((690.4, 307.6), (698.2, 333.4)),
)

_FIT = None


def _polar(x: float, y: float) -> tuple[float, float]:
    """Cartesian mm to (radius mm, angle rad)."""
    return (math.hypot(x, y), math.atan2(y, x))


def _fit() -> dict:
    """Fit YAW_DEG and the R_ERR line once; cached, prints nothing."""
    global _FIT
    if _FIT is not None:
        return _FIT
    yaw_errs = []
    cmd_radii = []
    rad_errs = []
    for (ax, ay), (bx, by) in MEASUREMENTS:
        cmd = table_to_base_mm(ax, ay)
        real = table_to_base_mm(bx, by)
        cmd_r, cmd_th = _polar(*cmd)
        real_r, real_th = _polar(*real)
        delta_deg = math.degrees(real_th - cmd_th)
        delta_deg = (delta_deg + 180.0) % 360.0 - 180.0
        yaw_errs.append(delta_deg)
        cmd_radii.append(cmd_r)
        rad_errs.append(real_r - cmd_r)
    yaw_deg = float(sum(yaw_errs) / len(yaw_errs))
    slope, intercept = (float(v) for v in np.polyfit(cmd_radii, rad_errs, 1))
    lo, hi = min(cmd_radii), max(cmd_radii)
    _FIT = {
        "yaw_deg": yaw_deg,
        "slope": slope,
        "intercept": intercept,
        "r_min": lo,
        "r_max": hi,
        "yaw_errs": yaw_errs,
        "cmd_radii": cmd_radii,
        "rad_errs": rad_errs,
    }
    return _FIT


def _r_err(cmd_r: float, fit: dict) -> float:
    """Radial error in mm, clamped to the fitted radius range."""
    clamped = min(max(float(cmd_r), fit["r_min"]), fit["r_max"])
    return fit["slope"] * clamped + fit["intercept"]


def predict_real_mm(x_cmd: float, y_cmd: float) -> tuple[float, float]:
    """Forward model: where the real tip lands for one commanded target."""
    fit = _fit()
    cmd_r, cmd_th = _polar(float(x_cmd), float(y_cmd))
    real_r = cmd_r + _r_err(cmd_r, fit)
    real_th = cmd_th + math.radians(fit["yaw_deg"])
    return (real_r * math.cos(real_th), real_r * math.sin(real_th))


def correct_base_target_mm(x: float, y: float) -> tuple[float, float]:
    """Inverse of the model: command this so the real tip lands at (x, y).

    The angle inverts in closed form; the radius inverts by fixed-point
    iteration against the clamped R_ERR line. Round-trips within 0.1 mm.
    """
    fit = _fit()
    want_r, want_th = _polar(float(x), float(y))
    cmd_th = want_th - math.radians(fit["yaw_deg"])
    cmd_r = want_r
    for _ in range(20):
        cmd_r = want_r - _r_err(cmd_r, fit)
    return (cmd_r * math.cos(cmd_th), cmd_r * math.sin(cmd_th))


def fit_report() -> list[str]:
    """YAW_DEG, the radial line, and the residual per point in mm."""
    fit = _fit()
    lines = [
        f"yaw: YAW_DEG={fit['yaw_deg']:.2f} deg "
        f"(mean of {', '.join(f'{v:+.2f}' for v in fit['yaw_errs'])})",
        f"radial: R_ERR(cmd_r) = {fit['slope']:.4f} * cmd_r "
        f"{fit['intercept']:+.2f} mm, "
        f"fitted cmd_r in [{fit['r_min']:.1f}, {fit['r_max']:.1f}] mm",
    ]
    for index, (((ax, ay), (bx, by)), cmd_r) in enumerate(
        zip(MEASUREMENTS, fit["cmd_radii"])
    ):
        cmd = table_to_base_mm(ax, ay)
        real = table_to_base_mm(bx, by)
        predicted = predict_real_mm(*cmd)
        residual = math.dist(predicted, real)
        lines.append(
            f"P{index + 1}: cmd_r={cmd_r:.1f} mm "
            f"r_err={fit['slope'] * cmd_r + fit['intercept']:+.1f} mm "
            f"residual={residual:.2f} mm"
        )
    return lines


def main(argv: object = None) -> int:
    """Print the fit report. Offline, no hardware."""
    parser = argparse.ArgumentParser(description="x/y tip correction fit.")
    parser.parse_args(argv)
    for line in fit_report():
        print(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
