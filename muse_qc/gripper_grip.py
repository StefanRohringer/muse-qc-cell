"""First grip with touch feedback, arm resting on its support. Human-run only.

Dry run (default): connect, read arm and gripper, open the touch sensor,
print the plan, move nothing. With --execute the human types "yes" and
the gripper opens to --open-mm, waits 3 s for the human to hold a part
between the jaws, closes slowly until the finger sensors report contact
on the left AND the right side, squeezes a little more, holds 3 s, then
opens again, while the arm holds its clamped rest pose every cycle.

Scale from the human ruler readings of 2026-10-28 (power off, jaws moved
by hand): width_mm = MM_PER_DEG * (CLOSED_DEG - deg), CLOSED_DEG = 2.8,
MM_PER_DEG = 0.31 (the 5 mm point sits off the line, soft Fin-Ray tips).
Negative = open. The motor angle wraps at +-180 deg: fully open reads
about +33 deg, which means nearly fully OPEN, not closed. Every target
must stay inside [SAFE_MIN_DEG, SAFE_MAX_DEG]; anything else refuses
before enabling, and a mid-run jump over 20 deg stops the grip. No
gripper-project code is used, only the serial line protocol and the SDK
motor protocol. The loop itself lives in muse_qc.gripper_stream.
"""

from __future__ import annotations

import argparse
import math
import sys

from muse_qc import safety
from muse_qc.arm import Arm
from muse_qc.first_move import CONFIRM_WORD
from muse_qc.first_move_plan import (
    LOOP_RATE_HZ,
    STEP_DEG,
    VLIM_FRACTION,
    read_sdk_vlims,
    scale_vlim,
)
from muse_qc.gripper_motor import CONFIRM_PROMPT, GRIPPER_CONFIG_VLIM
from muse_qc.gripper_stream import (
    FAULT_CYCLES,
    HOLD_CYCLES,
    JUMP_DEG,
    SETTLE_CYCLES,
    WAIT_CYCLES,
    GripLoop,
    check_arm_hold,
    open_touch_sensor,
)

CLOSED_DEG = 2.8  # jaws fully closed (human ruler reading 2026-10-28)
MM_PER_DEG = 0.31  # mm total jaw opening per motor degree, negative = open
SAFE_MIN_DEG = -175.0  # past this the wrapped reading is not trustworthy
SAFE_MAX_DEG = CLOSED_DEG + 3.0  # a little past closed, no further
OPEN_MIN_MM = 20.0
OPEN_MAX_MM = 50.0
SQUEEZE_MIN_DEG = 0.0
SQUEEZE_MAX_DEG = 5.0
SPEED_MIN_DEG_S, SPEED_MAX_DEG_S = 2.0, 15.0
SPEED_DEFAULT_DEG_S = 15.0
OUT_OF_RANGE_MSG = (
    "gripper reading outside the safe range (wrapped or past closed): "
    "close the jaws by hand to about 30 mm with the power off and run again"
)


def width_to_deg(open_mm: float) -> float:
    """Jaw opening in mm to gripper motor degrees. Pure."""
    return CLOSED_DEG - float(open_mm) / MM_PER_DEG


def deg_to_width(deg: float) -> float:
    """Gripper motor degrees to jaw opening in mm. Pure."""
    return MM_PER_DEG * (CLOSED_DEG - float(deg))


def _ranged_float(text: str, name: str, low: float, high: float, unit: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{name} must be a number, got {text!r}.")
    if not low <= value <= high:
        raise argparse.ArgumentTypeError(
            f"{name} must be between {low:g} and {high:g} {unit}, got {text!r}.")
    return value


def _open_mm_type(text: str) -> float:
    return _ranged_float(text, "--open-mm", OPEN_MIN_MM, OPEN_MAX_MM, "mm")


def _squeeze_type(text: str) -> float:
    return _ranged_float(text, "--squeeze-deg", SQUEEZE_MIN_DEG, SQUEEZE_MAX_DEG, "deg")


def _speed_type(text: str) -> float:
    return _ranged_float(text, "--speed-deg-s", SPEED_MIN_DEG_S, SPEED_MAX_DEG_S, "deg/s")


def validate_targets(start_deg: float, open_deg: float) -> str | None:
    """Refusal text when a commanded target is unsafe, else None. Pure.

    Ramps run straight between checked endpoints and the squeeze never
    passes CLOSED_DEG, so checking start and open width covers the whole
    plan before anything is enabled.
    """
    if not (math.isfinite(start_deg) and SAFE_MIN_DEG <= start_deg <= SAFE_MAX_DEG):
        return OUT_OF_RANGE_MSG
    if not (math.isfinite(open_deg) and SAFE_MIN_DEG <= open_deg <= SAFE_MAX_DEG):
        return (f"gripper plan target {open_deg:+.2f} deg is outside the safe range "
                f"[{SAFE_MIN_DEG:g}, {SAFE_MAX_DEG:g}] deg, not moving")
    return None


def _ramp_steps(from_deg: float, to_deg: float, step_deg: float = STEP_DEG) -> int:
    """Cycle count for a gripper ramp between two angles at step_deg."""
    return max(1, math.ceil(abs(to_deg - from_deg) / step_deg))


def _create_sdk() -> object:
    """Create the real SDK object. Called only by main(), never by tests."""
    from reBotArm_control_py.actuator import RebotArm

    return RebotArm()


def _read_start(sdk: object) -> tuple[object, list[float], float]:
    """Connect and read arm joints plus gripper through muse_qc.arm."""
    reader = Arm(sdk_factory=lambda: sdk)
    reader.connect()
    joints = reader.read_joints()
    gripper = reader.read_gripper()
    return reader, [joint.degrees for joint in joints], gripper.degrees


def _plan_lines(open_mm: float, open_deg: float, squeeze_deg: float,
                 speed_deg_s: float) -> list[str]:
    """Human-readable plan lines shared by dry run and execute."""
    return [
        f"open width: {open_mm:.0f} mm -> gripper {open_deg:+.2f} deg",
        f"squeeze: {squeeze_deg:.1f} deg past first bilateral contact",
        f"phases at 100 Hz: open at {speed_deg_s:g} deg/s, wait 3 s (place the part),"
        f" close at {speed_deg_s:g} deg/s to bilateral contact, squeeze, hold 3 s,"
        " open back, hold 1 s",
    ]


def dry_run(sdk: object, open_mm: float, squeeze_deg: float, out: object = print,
            find_port: object = None, sensor_factory: object = None,
            speed_deg_s: float = SPEED_DEFAULT_DEG_S) -> int:
    """Print the rest readings, the sensor state and the plan; move nothing."""
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        open_deg = width_to_deg(open_mm)
        refused = validate_targets(grip_start_deg, open_deg)
        if refused is not None:
            out(refused)
            return 1
        for line in _plan_lines(open_mm, open_deg, squeeze_deg, speed_deg_s):
            out(line)
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        try:
            out("dry run: moves nothing, motors never enabled")
            return 0
        finally:
            sensor.close()
    finally:
        reader.disconnect()


def execute(sdk: object, open_mm: float, squeeze_deg: float,
            input_fn: object = input, check_fn: object = None,
            tip_fn: object = None, arm_vlims: object = None,
            joint_limits: object = None, find_port: object = None,
            sensor_factory: object = None, out: object = print,
            speed_deg_s: float = SPEED_DEFAULT_DEG_S) -> int:
    """Check the arm hold pose once, ask, then run the touch-feedback grip."""
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    stream: GripLoop | None = None
    sensor: object | None = None
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        open_deg = width_to_deg(open_mm)
        refused = validate_targets(grip_start_deg, open_deg)
        if refused is not None:
            out(refused)
            return 1
        for line in _plan_lines(open_mm, open_deg, squeeze_deg, speed_deg_s):
            out(line)
        hold_deg = check_arm_hold(arm_start_deg, joint_limits, check_fn, tip_fn, out)
        if hold_deg is None:
            return 1
        vlims = read_sdk_vlims() if arm_vlims is None else list(arm_vlims)
        if vlims is None:
            out("SDK speed caps unreadable, not moving")
            return 1
        slow = scale_vlim(vlims)
        gripper_slow = [GRIPPER_CONFIG_VLIM * VLIM_FRACTION]
        if input_fn(CONFIRM_PROMPT).strip() != CONFIRM_WORD:
            out("confirmation missing, nothing was enabled")
            return 2
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        arm_group = sdk.groups["arm"]
        gripper_group = sdk.groups["gripper"]
        arm_start_rad = [math.radians(v) for v in hold_deg]
        arm_group.mode_pos_vel(vlim=slow)
        gripper_group.mode_pos_vel(vlim=gripper_slow)
        arm_group.send_pos_vel(arm_start_rad, vlim=slow)
        gripper_group.send_pos_vel([math.radians(grip_start_deg)], vlim=gripper_slow)
        sdk.enable_all()
        step_deg = float(speed_deg_s) / LOOP_RATE_HZ
        stream = GripLoop(arm_group, gripper_group, arm_start_rad, slow,
                          gripper_slow, grip_start_deg, open_deg, squeeze_deg,
                          CLOSED_DEG, SAFE_MIN_DEG, SAFE_MAX_DEG,
                          JUMP_DEG, sensor, deg_to_width, out, step_deg=step_deg)
        sdk.start_control_loop(stream, rate=LOOP_RATE_HZ)
        worst = (_ramp_steps(grip_start_deg, open_deg, step_deg)
                 + _ramp_steps(open_deg, CLOSED_DEG, step_deg)
                 + _ramp_steps(CLOSED_DEG, open_deg, step_deg)
                 + int(squeeze_deg / step_deg)
                 + WAIT_CYCLES + HOLD_CYCLES + SETTLE_CYCLES + FAULT_CYCLES + 2)
        try:
            stream.done.wait(timeout=worst / LOOP_RATE_HZ + 120.0)
        except KeyboardInterrupt:
            out("interrupted by human, opening back to the open width")
            stream.abort()
            stream.done.wait(timeout=worst / LOOP_RATE_HZ + 120.0)
            return 1
        try:
            back_rad = float(gripper_group.get_positions()[0])
            out(f"gripper final deg: commanded {stream.commanded:+.2f}"
                f" measured {math.degrees(back_rad):+.2f}")
        except Exception as exc:
            out(f"gripper final deg: commanded {stream.commanded:+.2f}"
                f" measured unknown ({exc})")
        if stream.fault is not None:
            return 1
        if stream.missed:
            out("motion finished without a held grip, gripper back at the open width")
        else:
            out("motion finished, gripper back at the open width")
        return 0
    except Exception as exc:
        out(f"motion failed: {exc}")
        if stream is not None:
            stream.abort()
        return 1
    finally:
        safety.set_j6_ref(old_ref)
        out("stopping: the motors will be disabled now")
        try:
            sdk.stop_control_loop()
        except Exception:
            pass
        try:
            sdk.disable_all()
        except Exception:
            pass
        try:
            if sensor is not None:
                sensor.close()
        except Exception:
            pass
        reader.disconnect()


def main(argv: object = None, sdk_factory: object = None,
         **kwargs: object) -> int:
    """Entry point for the human: dry run by default, --execute grips."""
    parser = argparse.ArgumentParser(description="First grip with touch feedback (human-run only).")
    parser.add_argument("--open-mm", type=_open_mm_type, default=40.0,
                        help=f"jaw opening in mm, {OPEN_MIN_MM:g} to {OPEN_MAX_MM:g} (default 40)")
    parser.add_argument("--squeeze-deg", type=_squeeze_type, default=2.0,
                        help=f"extra close past first contact in deg, {SQUEEZE_MIN_DEG:g} to"
                             f" {SQUEEZE_MAX_DEG:g} (default 2)")
    parser.add_argument("--speed-deg-s", type=_speed_type, default=SPEED_DEFAULT_DEG_S,
                        help=f"gripper ramp speed in deg/s, {SPEED_MIN_DEG_S:g} to"
                             f" {SPEED_MAX_DEG_S:g} (default {SPEED_DEFAULT_DEG_S:g})")
    parser.add_argument("--execute", action="store_true",
                        help="run the grip after confirmation")
    args = parser.parse_args(argv)
    sdk = sdk_factory() if sdk_factory is not None else _create_sdk()
    if args.execute:
        return execute(sdk, args.open_mm, args.squeeze_deg, speed_deg_s=args.speed_deg_s,
                       **kwargs)
    return dry_run(sdk, args.open_mm, args.squeeze_deg, speed_deg_s=args.speed_deg_s,
                   **kwargs)


if __name__ == "__main__":
    sys.exit(main())
