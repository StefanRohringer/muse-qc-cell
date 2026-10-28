"""First gripper motor test, direction only. The human runs it by hand.

Default is a dry run: connect, read the six arm joints and the
gripper through muse_qc.arm, print them with the plan, move nothing.

With --execute the human types "yes" (same prompt style as
first_move); only then the gripper ramps from its start reading by
--delta-deg at 5 deg/s, holds 2 s, ramps back and holds 1 s, while the
arm is commanded to the rest reading clamped into its URDF limits
every cycle and never moves.

SDK facts checked in the installed Seeed package (config file
config/rebotarm_dm.yaml and reBotArm_control_py/actuator/
rebotarm.py, read only, never imported by this module except through
muse_qc.arm and the RebotArm factory in _create_sdk): the gripper is
motor 0x07 (model 4310, vendor damiao) in group "gripper" with the one
joint "gripper", positions in rad; per cycle
gripper.send_pos_vel([pos], vlim=[v]) after mode_pos_vel(); the
gripper POS_VEL vlim is 3.0 rad/s, run here at 10 %. JointGroup
enable() calls the vendor controller's enable_all(), so enabling the
gripper also enables all arm motors; therefore the arm group is
commanded every cycle too. RebotArmEndPose.close_gripper uses target
0.0 rad and open_gripper a positive value, but our gripper reads about
-167.13 deg with the jaws 5.3 cm open, so the direction must be
confirmed on the real arm: from the URDF (finger_left prismatic 0 to
0.0285 m, finger_right mimic) 0 deg = closed and more negative = more
open, about 0.32 mm total opening per motor degree.
"""

from __future__ import annotations

import argparse
import math
import sys
import threading

from muse_qc import safety
from muse_qc.arm import Arm
from muse_qc.first_move import CONFIRM_WORD
from muse_qc.first_move_plan import (
    HOLD_CYCLES,
    LOOP_RATE_HZ,
    SEG_BACK,
    SEG_HOLD,
    SEG_OUT,
    SEG_SETTLE,
    SETTLE_HOLD_CYCLES,
    SPEED_FRACTION,
    STEP_DEG,
    VLIM_FRACTION,
    clamp_to_limits,
    read_sdk_vlims,
    real_tip_m,
    scale_vlim,
)

GRIPPER_CONFIG_VLIM = 3.0  # config/rebotarm_dm.yaml gripper POS_VEL vlim
DELTA_MIN_DEG = -20.0
DELTA_MAX_DEG = 20.0
MM_PER_MOTOR_DEG = 0.32  # about 0.32 mm total jaw opening per motor degree
CONFIRM_PROMPT = "Type yes to move the gripper: "


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


def _delta_type(text: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"--delta-deg must be a number, got {text!r}.")
    if value == 0.0 or not DELTA_MIN_DEG <= value <= DELTA_MAX_DEG:
        raise argparse.ArgumentTypeError(
            f"--delta-deg must be between {DELTA_MIN_DEG:g} and {DELTA_MAX_DEG:g}"
            f" deg and not 0, got {text!r}."
        )
    return value


def build_gripper_plan(start_deg: float, delta_deg: float) -> list[tuple[str, float]]:
    """Ramp out by delta at 5 deg/s, hold 2 s, ramp back, hold 1 s.

    STEP_DEG is one 5 deg/s step at LOOP_RATE_HZ, reused from
    first_move_plan so the ramp keeps the same per-cycle speed budget.
    Stays within start +- delta with exact endpoints.
    """
    if delta_deg == 0.0 or not math.isfinite(start_deg + delta_deg):
        raise ValueError("Need a finite start and a nonzero delta.")
    target = start_deg + delta_deg
    steps = max(1, math.ceil(abs(delta_deg) / STEP_DEG))
    plan = [(SEG_OUT, start_deg + delta_deg * i / steps) for i in range(1, steps + 1)]
    plan += [(SEG_HOLD, target)] * HOLD_CYCLES
    plan += [(SEG_BACK, target - delta_deg * i / steps) for i in range(1, steps + 1)]
    plan += [(SEG_SETTLE, start_deg)] * SETTLE_HOLD_CYCLES
    return plan


class GripperStream:
    """Control-loop callback: hold the arm start pose, stream one plan."""

    def __init__(self, arm_group: object, gripper_group: object,
                 arm_start_rad: list[float], arm_vlim: list[float],
                 start_rad: float, waypoints_rad: list[float],
                 gripper_vlim: list[float], snapshot_index: int | None) -> None:
        self.arm_group = arm_group
        self.gripper_group = gripper_group
        self.arm_start_rad = [float(v) for v in arm_start_rad]
        self.arm_vlim = [float(v) for v in arm_vlim]
        self.last_rad = float(start_rad)
        self.waypoints = [float(v) for v in waypoints_rad]
        self.gripper_vlim = [float(v) for v in gripper_vlim]
        self.snapshot_index = snapshot_index
        self.snapshot_deg: float | None = None
        self.index = 0
        self.halt = False
        self.done = threading.Event()

    def _send(self, grip_rad: float) -> None:
        self.gripper_group.send_pos_vel([grip_rad], vlim=self.gripper_vlim)
        self.arm_group.send_pos_vel(list(self.arm_start_rad), vlim=self.arm_vlim)

    def _hold(self) -> None:
        self._send(self.last_rad)
        self.done.set()

    def __call__(self, _sdk: object, _dt: float) -> None:
        if self.halt or self.index >= len(self.waypoints):
            self._hold()
            return
        if self.index == self.snapshot_index:
            try:
                positions = self.gripper_group.get_positions()
                self.snapshot_deg = math.degrees(float(positions[0]))
            except Exception:
                pass
        commanded = self.waypoints[self.index]
        self._send(commanded)
        self.last_rad = commanded
        self.index += 1
        if self.index >= len(self.waypoints):
            self.done.set()


def _expectation(delta_deg: float) -> str:
    """One-line jaw direction expectation for the dry run."""
    direction = "close" if delta_deg > 0 else "open"
    return (f"expectation: 0 deg = closed, more negative = more open, about "
            f"{MM_PER_MOTOR_DEG:g} mm total opening per motor degree, so "
            f"{delta_deg:+.1f} deg should {direction} the jaws by about "
            f"{abs(delta_deg) * MM_PER_MOTOR_DEG:.1f} mm")


def dry_run(sdk: object, delta_deg: float, out: object = print) -> int:
    """Print the rest readings and the plan; never enable or send."""
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}")
        out(_expectation(delta_deg))
        plan = build_gripper_plan(grip_start_deg, delta_deg)
        out(f"plan: {len(plan)} waypoints at {LOOP_RATE_HZ:.0f} Hz")
        out("dry run: moves nothing, motors never enabled")
        return 0
    finally:
        reader.disconnect()


def execute(sdk: object, delta_deg: float, input_fn: object = input,
            check_fn: object = None, tip_fn: object = None,
            arm_vlims: object = None, joint_limits: object = None,
            out: object = print) -> int:
    """Check the arm hold pose once, ask, then ramp the gripper out."""
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}")
        out(_expectation(delta_deg))
        plan = build_gripper_plan(grip_start_deg, delta_deg)
        out(f"plan: {len(plan)} waypoints at {LOOP_RATE_HZ:.0f} Hz")
        limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
        if limits is None:
            out("Joint limits unavailable, not moving")
            return 1
        hold_deg = list(clamp_to_limits(arm_start_deg, limits))
        out(f"arm hold deg: {[f'{v:+.2f}' for v in hold_deg]}")
        check = safety.check_target if check_fn is None else check_fn
        tip_of = real_tip_m if tip_fn is None else tip_fn
        hold_tip, rest_tip = tip_of(hold_deg), tip_of(list(arm_start_deg))
        target = safety.Target(x=hold_tip[0], y=hold_tip[1], z=hold_tip[2],
                               joints_deg=tuple(hold_deg),
                               speed=SPEED_FRACTION)
        current = safety.CurrentState(x=rest_tip[0], y=rest_tip[1], z=rest_tip[2],
                                      joints_deg=tuple(arm_start_deg),
                                      reading_age_ms=0.0)
        held = check(target, current)
        if not held.get("ok"):
            out(f"arm hold refused: {held.get('reason')}, not moving")
            return 1
        out("arm hold: ok")
        vlims = read_sdk_vlims() if arm_vlims is None else list(arm_vlims)
        if vlims is None:
            out("SDK speed caps unreadable, not moving")
            return 1
        slow = scale_vlim(vlims)
        gripper_slow = [GRIPPER_CONFIG_VLIM * VLIM_FRACTION]
        out(f"SDK vlim: {[f'{v:.1f}' for v in vlims]} -> 10 %: "
            f"{[f'{v:.2f}' for v in slow]}")
        out(f"gripper vlim: {GRIPPER_CONFIG_VLIM:.1f} -> 10 %: "
            f"{gripper_slow[0]:.2f}")
        if input_fn(CONFIRM_PROMPT).strip() != CONFIRM_WORD:
            out("confirmation missing, nothing was enabled")
            return 2
        arm_group = sdk.groups["arm"]
        gripper_group = sdk.groups["gripper"]
        arm_start_rad = [math.radians(v) for v in hold_deg]
        start_rad = math.radians(grip_start_deg)
        arm_group.mode_pos_vel(vlim=slow)
        gripper_group.mode_pos_vel(vlim=gripper_slow)
        arm_group.send_pos_vel(arm_start_rad, vlim=slow)
        gripper_group.send_pos_vel([start_rad], vlim=gripper_slow)
        sdk.enable_all()
        waypoints_rad = [math.radians(point) for _, point in plan]
        out_steps = sum(1 for segment, _ in plan if segment == SEG_OUT)
        stream = GripperStream(arm_group, gripper_group, arm_start_rad,
                               slow, start_rad, waypoints_rad,
                               gripper_slow, out_steps + HOLD_CYCLES - 1)
        sdk.start_control_loop(stream, rate=LOOP_RATE_HZ)
        try:
            stream.done.wait(timeout=len(plan) / LOOP_RATE_HZ + 120.0)
        except KeyboardInterrupt:
            out("interrupted by human, holding position")
            stream.halt = True
            stream.done.wait(timeout=30.0)
            return 1
        if stream.snapshot_deg is None:
            out("gripper commanded deg: "
                f"{grip_start_deg + delta_deg:+.2f}")
            out("gripper measured deg: unknown")
        else:
            out("gripper commanded deg: "
                f"{grip_start_deg + delta_deg:+.2f}")
            out(f"gripper measured deg: {stream.snapshot_deg:+.2f}")
        try:
            back_rad = float(gripper_group.get_positions()[0])
            out(f"gripper back deg: {math.degrees(back_rad):+.2f}")
        except Exception as exc:
            out(f"gripper back deg: unknown ({exc})")
        out("motion finished, gripper back at the start reading")
        result = 0
        return result
    except Exception as exc:
        out(f"motion failed: {exc}")
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
        reader.disconnect()


def main(argv: object = None, sdk_factory: object = None,
         **kwargs: object) -> int:
    """Entry point for the human: dry run by default, --execute moves."""
    parser = argparse.ArgumentParser(description="Gripper motor direction test (human-run only).")
    parser.add_argument("--delta-deg", type=_delta_type, required=True,
                        help=f"gripper move in deg, {DELTA_MIN_DEG:g} to "
                             f"{DELTA_MAX_DEG:g}, not 0")
    parser.add_argument("--execute", action="store_true",
                        help="move the gripper after confirmation")
    args = parser.parse_args(argv)
    sdk = sdk_factory() if sdk_factory is not None else _create_sdk()
    if args.execute:
        return execute(sdk, args.delta_deg, **kwargs)
    return dry_run(sdk, args.delta_deg, **kwargs)


if __name__ == "__main__":
    sys.exit(main())
