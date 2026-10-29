"""First pick with the arm. The human runs it by hand.

Dry run builds and checks the plan, moving nothing. --execute flies
one 100 Hz loop (muse_qc.pick_loop.PickLoop): hover (gripper opens
during the hover ramp), lower, grip, lift, hold, put down, open,
raise, return. --no-grip only lowers and holds for a ruler check.
"""
from __future__ import annotations
import argparse
import math
import sys
from muse_qc import safety
from muse_qc.arm import Arm
from muse_qc.arm_gains import describe, make_arm
from muse_qc.first_move import CONFIRM_WORD
from muse_qc.first_move_plan import LOOP_RATE_HZ, VLIM_FRACTION
from muse_qc.first_move_plan import clamp_to_limits, read_sdk_vlims, real_tip_m, scale_vlim
from muse_qc.gripper_grip import deg_to_width, validate_targets, width_to_deg
from muse_qc.gripper_motor import CONFIRM_PROMPT, GRIPPER_CONFIG_VLIM
from muse_qc.gripper_stream import open_touch_sensor
from muse_qc.pick_loop import PickLoop
from muse_qc.pick_plan import build_pick_plan, check_pick_plan, plan_summary
GRIP_Z_MM = (5.0, 60.0, 5.0)
OPEN_MM = (25.0, 50.0, 28.0)
HOLD_S = (0.5, 30.0, 1.0)
SPEED_DEG_S = (2.0, 15.0, 15.0)
def _ranged(text, name, low, high, unit):
    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{name} must be a number, got {text!r}.")
    if not low <= value <= high:
        raise argparse.ArgumentTypeError(
            f"{name} must be between {low:g} and {high:g} {unit}, got {text!r}.")
    return value
def _grip_z_type(text): return _ranged(text, "--grip-z-mm", *GRIP_Z_MM[:2], "mm")
def _open_mm_type(text): return _ranged(text, "--open-mm", *OPEN_MM[:2], "mm")
def _hold_s_type(text): return _ranged(text, "--hold-s", *HOLD_S[:2], "s")
def _speed_type(text): return _ranged(text, "--speed-deg-s", *SPEED_DEG_S[:2], "deg/s")
def _read_start(sdk):
    reader = Arm(sdk_factory=lambda: sdk)
    reader.connect()
    joints = reader.read_joints()
    return reader, [j.degrees for j in joints], reader.read_gripper().degrees
def _create_sdk():
    from reBotArm_control_py.actuator import RebotArm
    return RebotArm()
def _plan_and_check(arm_start_deg, grip_start_deg, table_xy, grip_z_mm, open_mm,
                    out, joint_limits, solve_fn, check_fn, tip_fn,
                    use_xy_correction=True, speed_deg_s=SPEED_DEG_S[2]):
    open_deg = width_to_deg(open_mm)
    refused = validate_targets(grip_start_deg, open_deg)
    if refused is not None:
        out(refused)
        return None, None
    out(f"open width: {open_mm:.0f} mm -> gripper {open_deg:+.2f} deg")
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    try:
        built = build_pick_plan(arm_start_deg, table_xy, grip_z_mm,
                                joint_limits, solve_fn=solve_fn,
                                use_xy_correction=use_xy_correction,
                                free_speed_deg_s=speed_deg_s)
        if not built.get("ok"):
            out(f"plan failed: {built.get('reason')}")
            return None, None
        for line in plan_summary(built["plan"], built):
            out(line)
        check = safety.check_target if check_fn is None else check_fn
        tip_of = real_tip_m if tip_fn is None else tip_fn
        ok, bad, res = check_pick_plan(built["plan"], arm_start_deg, check, tip_of)
    finally:
        safety.set_j6_ref(old_ref)
    if not ok:
        out(f"waypoint {bad + 1}/{len(built['plan'])} REFUSED {res.get('reason')}")
        return None, None
    out("all waypoints ok")
    return open_deg, built
def dry_run(sdk, table_xy, grip_z_mm, open_mm, no_grip, hold_s, out=print,
            find_port=None, sensor_factory=None, joint_limits=None, check_fn=None,
            tip_fn=None, solve_fn=None, use_xy_correction=True,
            speed_deg_s=SPEED_DEG_S[2]):
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        try:
            _, built = _plan_and_check(arm_start_deg, grip_start_deg, table_xy,
                                       grip_z_mm, open_mm, out, joint_limits,
                                       solve_fn, check_fn, tip_fn,
                                       use_xy_correction,
                                       speed_deg_s=speed_deg_s)
            if built is None:
                return 1
            out("dry run: moves nothing, motors never enabled")
            return 0
        finally:
            sensor.close()
    finally:
        reader.disconnect()
def execute(sdk, table_xy, grip_z_mm, open_mm, no_grip, hold_s, input_fn=input,
            out=print, find_port=None, sensor_factory=None, joint_limits=None,
            check_fn=None, tip_fn=None, arm_vlims=None, solve_fn=None,
            use_xy_correction=True, speed_deg_s=SPEED_DEG_S[2]):
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    stream, sensor = None, None
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        open_deg, built = _plan_and_check(arm_start_deg, grip_start_deg, table_xy,
                                          grip_z_mm, open_mm, out, joint_limits,
                                          solve_fn, check_fn, tip_fn,
                                          use_xy_correction,
                                          speed_deg_s=speed_deg_s)
        if built is None:
            return 1
        limits = safety.JOINT_LIMITS_DEG if joint_limits is None else joint_limits
        if limits is None:
            out("Joint limits unavailable, not moving")
            return 1
        vlims = read_sdk_vlims() if arm_vlims is None else list(arm_vlims)
        if vlims is None:
            out("SDK speed caps unreadable, not moving")
            return 1
        slow, gripper_slow = scale_vlim(vlims), [GRIPPER_CONFIG_VLIM * VLIM_FRACTION]
        out(f"arm vlim 10 %: {[f'{v:.2f}' for v in slow]}"
            f" gripper vlim 10 %: {gripper_slow[0]:.2f}")
        if input_fn(CONFIRM_PROMPT).strip() != CONFIRM_WORD:
            out("confirmation missing, nothing was enabled")
            return 2
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        arm_group, gripper_group = sdk.groups["arm"], sdk.groups["gripper"]
        hold_deg = list(clamp_to_limits(arm_start_deg, limits))
        arm_start_rad = [math.radians(v) for v in hold_deg]
        arm_group.mode_pos_vel(vlim=slow); gripper_group.mode_pos_vel(vlim=gripper_slow)
        arm_group.send_pos_vel(arm_start_rad, vlim=slow)
        gripper_group.send_pos_vel([math.radians(grip_start_deg)], vlim=gripper_slow)
        sdk.enable_all()
        check = safety.check_target if check_fn is None else check_fn
        tip_of = real_tip_m if tip_fn is None else tip_fn
        plan, ranges = built["plan"], built["ranges"]
        tips = [tip_of(list(wp)) for _, wp, _ in plan]
        stream = PickLoop(arm_group, gripper_group, slow, gripper_slow, arm_start_rad,
                          plan, tips, ranges, tips[ranges["hover"][1] - 1][2], open_deg,
                          grip_start_deg, round(hold_s * LOOP_RATE_HZ), no_grip, sensor,
                          out, check, tip_of)
        total = len(plan) + 2 * round(hold_s * LOOP_RATE_HZ) + 500
        live = False
        try:
            sdk.start_control_loop(stream, rate=LOOP_RATE_HZ)
            live = True
            stream.done.wait(timeout=total / LOOP_RATE_HZ + 120.0)
        except (KeyboardInterrupt, Exception) as exc:
            out("interrupted by human, holding position" if isinstance(exc, KeyboardInterrupt)
                else f"motion failed: {exc}")
            stream.abort()
            if live:
                stream.done.wait(timeout=total / LOOP_RATE_HZ + 120.0)
            else:
                for _ in range(total):
                    if stream.done.is_set():
                        break
                    stream(sdk, 1.0 / LOOP_RATE_HZ)
            return 1
        return stream.exit_code
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
            arm_end = [math.degrees(v) for v in sdk.groups["arm"].get_positions()]
            grip_end = math.degrees(float(sdk.groups["gripper"].get_positions()[0]))
            out("end readings: arm [" + " ".join(f"{v:+.2f}" for v in arm_end) + "]"
                f" gripper {grip_end:+.2f} deg")
        except Exception as exc:
            out(f"end readings unknown ({exc})")
        try:
            if sensor is not None:
                sensor.close()
        except Exception:
            pass
        reader.disconnect()
def main(argv=None, sdk_factory=None, **kwargs):
    parser = argparse.ArgumentParser(description="First pick with the arm (human-run only).")
    parser.add_argument("--table-mm", nargs=2, type=float, required=True,
                        metavar=("X", "Y"), help="part position in table mm")
    parser.add_argument("--grip-z-mm", type=_grip_z_type, default=GRIP_Z_MM[2],
                        help="grip tip height in mm, 5 to 60 (default 5)")
    parser.add_argument("--open-mm", type=_open_mm_type, default=OPEN_MM[2],
                        help="jaw opening in mm, 25 to 50 (default 28)")
    parser.add_argument("--no-grip", action="store_true",
                        help="only lower to --grip-z-mm and hold for a ruler check")
    parser.add_argument("--hold-s", type=_hold_s_type, default=HOLD_S[2],
                        help="hold time in s, 0.5 to 30 (default 1)")
    parser.add_argument("--speed-deg-s", type=_speed_type, default=SPEED_DEG_S[2],
                        help="free ramp speed in deg/s, 2 to 15 (default 15)")
    parser.add_argument("--gains", choices=("default", "soft"), default="default",
                        help="motor gain profile (default or soft joints 4/5)")
    parser.add_argument("--execute", action="store_true", help="fly the pick after confirmation")
    parser.add_argument("--no-xy-correction", action="store_true",
                        help="fly the uncorrected target (for A/B tests)")
    args = parser.parse_args(argv)
    out = kwargs.get("out", print)
    for line in describe(args.gains):
        out(line)
    sdk = make_arm(args.gains, factory=sdk_factory)
    run = execute if args.execute else dry_run
    return run(sdk, (args.table_mm[0], args.table_mm[1]), args.grip_z_mm,
               args.open_mm, args.no_grip, args.hold_s,
               use_xy_correction=not args.no_xy_correction,
               speed_deg_s=args.speed_deg_s, **kwargs)
if __name__ == "__main__":
    sys.exit(main())
