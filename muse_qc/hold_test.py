"""Hold test: hover over a table point and measure the joint swing.

While the arm holds a pose, joints 4 and 5 visibly swing a little
(limit cycle of the motor position loop, not a planning error).
This module flies the hover_move plan (100 mm over the corrected
target, same safety checks per cycle, same 5 deg/s ramp, return to
rest and settle, disable in finally), reads all six joints every
cycle while holding, and reports per-joint statistics plus the
dominant swing frequency of joints 4 and 5. The human runs it by
hand, twice: once with --gains default, once with --gains soft.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from datetime import datetime
from pathlib import Path

from muse_qc import hover_move
from muse_qc.arm_gains import GAIN_PROFILES, describe, make_arm
from muse_qc.first_move import dry_run as first_dry_run
from muse_qc.first_move import execute as first_execute
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.target_pose import HOVER_MM
from muse_qc.waypoint_stream import WaypointStream

HOLD_S = (5.0, 60.0, 20.0)
SWING_JOINTS = (3, 4)  # zero-based: joints 4 and 5
MIN_CROSSINGS = 4


def swing_hz(errors, rate_hz) -> float | None:
    """Dominant swing frequency from zero crossings around the mean.

    Returns None with fewer than MIN_CROSSINGS crossings (no swing).
    """
    if len(errors) < 2:
        return None
    mean = sum(errors) / len(errors)
    crossings = sum(
        1 for before, after in zip(errors, errors[1:])
        if (before - mean) * (after - mean) < 0
    )
    if crossings < MIN_CROSSINGS:
        return None
    return (crossings / 2.0) / (len(errors) / rate_hz)


def hold_lines(samples, goal) -> list[str]:
    """One stats line per joint: mean err, peak-to-peak, RMS, swing."""
    lines = []
    for joint in range(6):
        errors = [sample[joint] - goal[joint] for sample in samples]
        mean = sum(errors) / len(errors)
        peak = max(errors) - min(errors)
        rms = math.sqrt(sum(error * error for error in errors) / len(errors))
        text = (f"joint {joint + 1}: mean err {mean:+.2f} deg,"
                f" p2p {peak:.2f} deg, rms {rms:.2f} deg")
        if joint in SWING_JOINTS:
            freq = swing_hz(errors, LOOP_RATE_HZ)
            text += f", swing {freq:.1f} Hz" if freq is not None else ", swing n/a"
        lines.append(text)
    return lines


def save_hold_csv(samples, recordings_dir="recordings") -> Path:
    """Save the hold samples (100 Hz joint degrees) as CSV."""
    target = Path(recordings_dir)
    target.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target / f"hold_{stamp}.csv"
    step = 1.0 / LOOP_RATE_HZ
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s"] + [f"joint{joint}_deg" for joint in range(1, 7)])
        for index, sample in enumerate(samples):
            writer.writerow([f"{index * step:.2f}"] + list(sample))
    return path


class HoldStream(WaypointStream):
    """WaypointStream that keeps every hold-segment joint reading."""

    def __init__(self, *args, segments=(), **kwargs):
        super().__init__(*args, **kwargs)
        self._segments = list(segments)
        self.hold_goal = None
        self.hold_samples: list[list[float]] = []

    def on_sample(self, current_deg: list[float]) -> None:
        """Keep the reading while the stream holds the hover pose."""
        if not self._segments:
            return
        index = min(self.index, len(self._segments) - 1)
        if self._segments[index] != hover_move.SEG_HOLD:
            return
        if self.hold_goal is None:
            self.hold_goal = [float(v) for v in self.waypoints[self.index]]
        self.hold_samples.append([float(v) for v in current_deg])


def _stream_factory(captured: dict):
    """Build the first_move stream factory keeping the HoldStream."""

    def build(group, waypoints, tips, slow, first_rad, check_fn, tip_fn,
              snapshot_index=None):
        stream = HoldStream(group, waypoints, tips, slow, first_rad,
                            check_fn, tip_fn, segments=captured.get("segments", ()),
                            snapshot_index=snapshot_index)
        captured["stream"] = stream
        return stream

    return build


def report_hold(captured: dict, out=print) -> None:
    """Print hold statistics and save the CSV (or "hold: no samples")."""
    stream = captured.get("stream")
    samples = stream.hold_samples if stream is not None else []
    if not samples or stream.hold_goal is None:
        out("hold: no samples")
        return
    for line in hold_lines(samples, stream.hold_goal):
        out(line)
    try:
        out(save_hold_csv(samples))
    except OSError as exc:
        out(f"request failed: {exc}")


def main(argv=None, sdk_factory=None, **kwargs) -> int:
    """Entry point for the human: dry run by default, --execute holds."""
    parser = argparse.ArgumentParser(description="Hold test (human-run only).")
    parser.add_argument("--table-mm", nargs=2, type=float, required=True,
                        metavar=("X", "Y"), help="part position in table mm")
    parser.add_argument("--hold-s", type=float, default=HOLD_S[2],
                        help="hover hold time in seconds (5.0 to 60.0)")
    parser.add_argument("--gains", choices=tuple(GAIN_PROFILES),
                        default="default",
                        help="motor gain profile for joints 4/5")
    parser.add_argument("--execute", action="store_true",
                        help="move the arm after confirmation")
    args = parser.parse_args(argv)
    if not HOLD_S[0] <= args.hold_s <= HOLD_S[1]:
        parser.error("--hold-s must be within 5.0 to 60.0 seconds")
    out = kwargs.get("out", print)
    for line in describe(args.gains):
        out(line)
    sdk = make_arm(args.gains, factory=sdk_factory)
    hold_cycles = round(args.hold_s * LOOP_RATE_HZ)
    captured: dict = {}

    def plan_fn(start_deg):
        plan = hover_move.make_hover_return_plan_fn(
            (args.table_mm[0], args.table_mm[1]), HOVER_MM,
            kwargs.get("joint_limits"), out, hold_cycles)(start_deg)
        captured["segments"] = [segment for segment, _ in plan]
        return plan

    kwargs = dict(kwargs)
    kwargs["plan_fn"] = plan_fn
    if args.execute:
        kwargs["stream_factory"] = _stream_factory(captured)
        code = first_execute(sdk, **kwargs)
        report_hold(captured, out)
        return code
    return first_dry_run(sdk, **kwargs)


if __name__ == "__main__":
    sys.exit(main())
