"""First pick with the arm. The human runs it by hand.

Dry run builds and checks the plan, moving nothing. --execute flies
one 100 Hz loop (muse_qc.pick_loop.PickLoop): hover (gripper opens
during the hover ramp), lower, grip, lift, hold, put down, open,
raise, return. --no-grip only lowers and holds for a ruler check.
With --drop good|bad the plan is hover, lower, lift, to-box,
release, return instead: the arm carries the part to the good
(left) or bad (right) box and opens the gripper over it.
With --inspect the arm grips the part, lifts it, holds the last
lift waypoint while the arm camera grabs one frame of the part in
the gripper, the levers are counted and the verdict picks the
box; a failed inspection sorts the unverified part into the bad
box with exit code 1. With --station the arm carries the part to
the fixed inspection station first and inspects there; without
it (default) the part is inspected at the lift pose. The default arm mode is mit-ff (MIT with
gravity feed-forward); --arm-mode posvel selects the old POS_VEL
loop. The default grip tip height is 8 mm. With --lever-deg the
gripper turns about the vertical axis so the part is gripped like
a straight part.
"""
from __future__ import annotations
import argparse
import math
import sys
from muse_qc import mit_hold, pick_mit, safety
from muse_qc.arm_gains import GAIN_PROFILES, describe, make_arm
from muse_qc.grip_yaw import grip_yaw_offset_deg
from muse_qc.pick_args import (
    ACCEL_DEG_S2,
    GRIP_Z_MM,
    HOLD_S,
    OPEN_MM,
    POSVEL_SPEED_MAX_DEG_S,
    SPEED_DEG_S,
    _accel_type,
    _create_sdk,
    _grip_z_type,
    _hold_s_type,
    _lever_deg_type,
    _open_mm_type,
    _read_start,
    _speed_type,
)
from muse_qc.drop_plan import RELEASE_OPEN_MM
from muse_qc.first_move import CONFIRM_WORD
from muse_qc.first_move_plan import LOOP_RATE_HZ
from muse_qc.first_move_plan import clamp_to_limits, read_sdk_vlims, real_tip_m, scale_vlim
from muse_qc.gripper_grip import deg_to_width, width_to_deg
from muse_qc.grip_speed import GRIPPER_VLIM_FRACTION
from muse_qc.gripper_motor import CONFIRM_PROMPT, GRIPPER_CONFIG_VLIM
from muse_qc.gripper_stream import open_touch_sensor
from muse_qc import inspect_step
from muse_qc.inspect_station import USE_STATION_DEFAULT
from muse_qc.pick_loop import PickLoop
from muse_qc.pick_plan_check import _plan_and_check


def _lever_solve(table_xy, lever_deg, solve_fn, out):
    """Wrap solve_fn with the grip yaw offset; print the grip yaw line.

    Without lever_deg the solve_fn is used unchanged. With it the
    offset from grip_yaw_offset_deg is applied through
    solve_hover's yaw_offset_deg (or passed to a fake solver that
    takes the same keyword) for hover, lower, lift and put-down,
    which all go through the same solve_fn in pick_plan.
    """
    if lever_deg is None:
        return solve_fn
    offset, clamped = grip_yaw_offset_deg(table_xy, lever_deg)
    out(f"grip yaw: lever {float(lever_deg):.1f} deg,"
        f" offset {offset:+.1f} deg" + (" (clamped)" if clamped else ""))
    from muse_qc.target_pose import solve_hover as real_solve
    base = real_solve if solve_fn is None else solve_fn
    return lambda t, h, s: base(t, h, s, yaw_offset_deg=offset)


def dry_run(sdk, table_xy, grip_z_mm, open_mm, no_grip, hold_s, out=print,
            find_port=None, sensor_factory=None, joint_limits=None, check_fn=None,
            tip_fn=None, solve_fn=None, use_xy_correction=True,
            speed_deg_s=SPEED_DEG_S[2], accel_deg_s2=ACCEL_DEG_S2[2],
            drop=None, solve_drop_fn=None,
            inspect=False, find_arm_node=None, read_key=None,
            arm_mode="mit-ff", tau_scale=None, gravity_fn=None,
            lever_deg=None, use_station=None):
    out(f"arm mode: {arm_mode}")
    if inspect and inspect_step.check_prereqs(
            out, find_arm_node, read_key) is None:
        return 2
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        solve_fn = _lever_solve(table_xy, lever_deg, solve_fn, out)
        try:
            _, built = _plan_and_check(arm_start_deg, grip_start_deg, table_xy,
                                       grip_z_mm, open_mm, out, joint_limits,
                                       solve_fn, check_fn, tip_fn,
                                       use_xy_correction, speed_deg_s=speed_deg_s,
                                       accel_deg_s2=accel_deg_s2, drop=drop,
                                       solve_drop_fn=solve_drop_fn, inspect=inspect,
                                       use_station=use_station)
            if built is None:
                return 1
            if arm_mode == "mit-ff":
                for line in pick_mit.dry_run_lines(
                        built, drop, arm_start_deg, gravity_fn,
                        tau_scale, solve_drop_fn):
                    out(line)
            out("dry run: moves nothing, motors never enabled")
            return 0
        finally:
            sensor.close()
    finally:
        reader.disconnect()
def execute(sdk, table_xy, grip_z_mm, open_mm, no_grip, hold_s, input_fn=input,
            out=print, find_port=None, sensor_factory=None, joint_limits=None,
            check_fn=None, tip_fn=None, arm_vlims=None, solve_fn=None,
            use_xy_correction=True, speed_deg_s=SPEED_DEG_S[2],
            accel_deg_s2=ACCEL_DEG_S2[2],
            drop=None, solve_drop_fn=None, inspect=False, find_arm_node=None,
            read_key=None, open_arm_camera=None, make_client=None,
            count_frame=None, inspect_recordings_dir="recordings",
            arm_mode="mit-ff", tau_scale=None, gravity_fn=None,
            lever_deg=None, use_station=None):
    out(f"arm mode: {arm_mode}")
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    stream, sensor, arm_sender = None, None, None
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        if inspect and inspect_step.check_prereqs(
                out, find_arm_node, read_key) is None:
            return 2
        solve_fn = _lever_solve(table_xy, lever_deg, solve_fn, out)
        open_deg, built = _plan_and_check(arm_start_deg, grip_start_deg, table_xy,
                                          grip_z_mm, open_mm, out, joint_limits,
                                          solve_fn, check_fn, tip_fn,
                                          use_xy_correction, speed_deg_s=speed_deg_s,
                                          accel_deg_s2=accel_deg_s2, drop=drop,
                                          solve_drop_fn=solve_drop_fn, inspect=inspect,
                                          use_station=use_station)
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
        slow, gripper_slow = scale_vlim(vlims), [GRIPPER_CONFIG_VLIM * GRIPPER_VLIM_FRACTION]
        out(f"arm vlim 10 %: {[f'{v:.2f}' for v in slow]}"
            f" gripper vlim 25 %: {gripper_slow[0]:.2f}")
        if input_fn(CONFIRM_PROMPT).strip() != CONFIRM_WORD:
            out("confirmation missing, nothing was enabled")
            return 2
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        arm_group, gripper_group = sdk.groups["arm"], sdk.groups["gripper"]
        hold_deg = list(clamp_to_limits(arm_start_deg, limits))
        arm_start_rad = [math.radians(v) for v in hold_deg]
        guard = None
        if arm_mode == "mit-ff":
            arm_sender = pick_mit.MitArmSender(
                arm_group, tau_scale=tau_scale, gravity_fn=gravity_fn)
            arm_sender.begin(arm_start_rad)
            guard = pick_mit.TrackingGuard()
        else:
            arm_group.mode_pos_vel(vlim=slow)
            arm_group.send_pos_vel(arm_start_rad, vlim=slow)
        gripper_group.mode_pos_vel(vlim=gripper_slow)
        gripper_group.send_pos_vel([math.radians(grip_start_deg)], vlim=gripper_slow)
        sdk.enable_all()
        check = safety.check_target if check_fn is None else check_fn
        tip_of = real_tip_m if tip_fn is None else tip_fn
        plan, ranges = built["plan"], built["ranges"]
        tips = [tip_of(list(wp)) for _, wp, _ in plan]
        release_open_deg = (width_to_deg(RELEASE_OPEN_MM)
                              if (drop is not None or inspect) else None)
        stream = PickLoop(arm_group, gripper_group, slow, gripper_slow, arm_start_rad,
                          plan, tips, ranges, tips[ranges["hover"][1] - 1][2], open_deg,
                          grip_start_deg, round(hold_s * LOOP_RATE_HZ), no_grip, sensor,
                          out, check, tip_of, drop, release_open_deg,
                          inspect=inspect,
                          ranges_by_box=built.get("ranges_by_box"),
                          shared_ranges=built.get("shared_ranges"),
                          arm_sender=arm_sender, tracking_guard=guard)
        total = len(plan) + 2 * round(hold_s * LOOP_RATE_HZ) + 500
        live = False
        try:
            sdk.start_control_loop(stream, rate=LOOP_RATE_HZ)
            live = True
            if inspect:
                box = inspect_step.wait_and_inspect(
                    stream, out,
                    ready_timeout_s=total / LOOP_RATE_HZ + 120.0,
                    find_node=find_arm_node, open_camera=open_arm_camera,
                    read_key=read_key, make_client=make_client,
                    count_frame_fn=count_frame,
                    recordings_dir=inspect_recordings_dir)
                stream.set_verdict(box)
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
        if arm_sender is not None:
            out(arm_sender.trim_line())
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
                        help="grip tip height in mm, 5 to 60 (default 8)")
    parser.add_argument("--open-mm", type=_open_mm_type, default=OPEN_MM[2],
                        help="jaw opening in mm, 25 to 50 (default 28)")
    parser.add_argument("--no-grip", action="store_true",
                        help="only lower to --grip-z-mm and hold for a ruler check")
    parser.add_argument("--hold-s", type=_hold_s_type, default=HOLD_S[2],
                        help="hold time in s, 0.5 to 30 (default 1)")
    parser.add_argument("--speed-deg-s", type=_speed_type, default=None,
                        help="free ramp speed in deg/s, 2 to 80"
                        " (default 50 mit-ff, 15 posvel; explicit above 15 needs mit-ff)")
    parser.add_argument("--accel-deg-s2", type=_accel_type, default=None,
                        help="free ramp accel in deg/s2, 15 to 160 (default 100 mit-ff, 30 posvel)")
    parser.add_argument("--gains", choices=tuple(GAIN_PROFILES),
                        default="vel2",
                        help="motor gain profile for joints 4/5")
    parser.add_argument("--arm-mode", choices=("posvel", "mit-ff"), default="mit-ff",
                        help="arm control mode: mit-ff (default) or the old posvel loop")
    parser.add_argument("--tau-scale", nargs=6, type=float, default=None,
                        metavar=("S1", "S2", "S3", "S4", "S5", "S6"),
                        help="per-joint gravity torque scale for mit-ff (six values, 0.0 to 2.0)")
    parser.add_argument("--lever-deg", type=_lever_deg_type, default=None,
                        help="lever direction in deg, 0 to 180 (from find_parts;"
                        " turns the gripper so every part is gripped"
                        " like a straight part)")
    parser.add_argument("--execute", action="store_true", help="fly the pick after confirmation")
    parser.add_argument("--no-xy-correction", action="store_true",
                        help="fly the uncorrected target (for A/B tests)")
    exclusive = parser.add_mutually_exclusive_group()
    exclusive.add_argument("--drop", choices=("good", "bad"), default=None,
                           help="carry the part to the good (left) or bad (right)"
                           " box and release it over the box (default: put down)")
    exclusive.add_argument("--inspect", action="store_true",
                           help="inspect the part in the gripper after the lift"
                           " and sort into the box the lever count decides")
    parser.add_argument("--station", dest="station", action="store_true",
                        default=None,
                        help="inspect at the fixed station left of the table"
                        " (needs --inspect)")
    parser.add_argument("--no-station", dest="station", action="store_false",
                        help="inspect at the lift pose, as today")
    args = parser.parse_args(argv)
    if args.tau_scale is not None and args.arm_mode != "mit-ff":
        parser.error("--tau-scale needs --arm-mode mit-ff")
    speed_explicit = args.speed_deg_s is not None
    if args.speed_deg_s is None:
        args.speed_deg_s = SPEED_DEG_S[2] if args.arm_mode == "mit-ff" else 15.0
    if args.accel_deg_s2 is None:
        args.accel_deg_s2 = ACCEL_DEG_S2[2] if args.arm_mode == "mit-ff" else 30.0
    if args.arm_mode != "mit-ff" and speed_explicit and args.speed_deg_s > POSVEL_SPEED_MAX_DEG_S:
        parser.error("--speed-deg-s above 15 needs --arm-mode mit-ff"
                     " (the POS_VEL vlim would cap the motors)")
    tau_scale = (tuple(args.tau_scale) if args.tau_scale is not None
                 else mit_hold.TAU_SCALE)
    try:
        mit_hold.validate_tau_scale(tau_scale)
    except ValueError as exc:
        parser.error(str(exc))
    out = kwargs.get("out", print)
    for line in describe(args.gains):
        out(line)
    sdk = make_arm(args.gains, factory=sdk_factory)
    run = execute if args.execute else dry_run
    kwargs.setdefault("drop", args.drop)
    kwargs.setdefault("inspect", args.inspect)
    if args.station is None:
        kwargs.setdefault("use_station", USE_STATION_DEFAULT)
    else:
        kwargs.setdefault("use_station", args.station)
    kwargs.setdefault("arm_mode", args.arm_mode)
    kwargs.setdefault("tau_scale", tau_scale)
    kwargs.setdefault("lever_deg", args.lever_deg)
    return run(sdk, (args.table_mm[0], args.table_mm[1]), args.grip_z_mm,
               args.open_mm, args.no_grip, args.hold_s,
               use_xy_correction=not args.no_xy_correction,
               speed_deg_s=args.speed_deg_s, accel_deg_s2=args.accel_deg_s2,
               **kwargs)
if __name__ == "__main__":
    sys.exit(main())
