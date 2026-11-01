"""One arm session for a whole sort_in_order plan. Human-run only.

Same motion as muse_qc.pick.pick_test per part (hover, lower,
grip, lift, station hold, to-box, release), but the arm does NOT
return to rest between parts: after the release hold the loop
keeps holding the box exit pose (same _hold mechanism, tracking
guard active) for up to HOLD_WAIT_S while the agent computes the
next target from the station image. A transit segment then goes
from the box exit straight to the next hover. Return to rest,
settle and disable happen ONCE at the end of the session.

Protocol: targets come on stdin, one per line ("part X Y
LEVER", "part X Y" without a lever, or "end"). After every
release the child prints the same per-part lines as pick_test
plus one framing line "session: part done". The session ends
with "session: end (<reason>)" after walking the existing
return-to-rest plus settle, then disables once. A missed grip
or any uncertain grip state never transits to another part.

Confirmation for --execute: with --confirm-stdin the first
stdin line is the confirmation and must read exactly "yes";
it is never parsed as a target (the agent always passes this
flag). Without it the confirmation comes only from /dev/tty
(manual run); when /dev/tty cannot be opened nothing is
enabled. With input_fn (tests) that function is asked.
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading

from muse_qc.arm import mit_hold, safety
from muse_qc.arm.arm_gains import GAIN_PROFILES, describe, make_arm
from muse_qc.pick import pick_mit
from muse_qc.pick import session_driver
from muse_qc.pick.box_transit import build_box_transit
from muse_qc.pick.pick_args import (
    ACCEL_DEG_S2,
    GRIP_Z_MM,
    HOLD_S,
    OPEN_MM,
    POSVEL_SPEED_MAX_DEG_S,
    SPEED_DEG_S,
    _accel_type,
    _grip_z_type,
    _hold_s_type,
    _open_mm_type,
    _read_start,
    _speed_type,
)
from muse_qc.pick.session_loop import join_done
from muse_qc.pick.session_protocol import (
    HOLD_WAIT_S,
    _EOF,
    feed_lines,
)
from muse_qc.gripper.gripper_grip import deg_to_width
from muse_qc.gripper.gripper_stream import open_touch_sensor
from muse_qc.inspection import inspect_step
from muse_qc.bringup.first_move import CONFIRM_WORD
from muse_qc.gripper.gripper_motor import CONFIRM_PROMPT

_CONFIRM_TIMEOUT_S = 60.0


def _confirm_session(input_fn, lines, out, tty_opener=open,
                     from_stdin=False) -> bool:
    """Confirm --execute (see the module docstring for the routing).

    With --confirm-stdin the first stdin line is the confirmation
    and must read exactly CONFIRM_WORD; it is never parsed as a
    target. Otherwise the confirmation comes only from /dev/tty
    (tty_opener is injectable for tests); stdin is never read
    for it.
    """
    if input_fn is not None:
        return input_fn(CONFIRM_PROMPT).strip() == CONFIRM_WORD
    if from_stdin:
        try:
            first = lines.get(timeout=_CONFIRM_TIMEOUT_S)
        except queue.Empty:
            return False
        return first is not _EOF and str(first).strip() == CONFIRM_WORD
    try:
        with tty_opener("/dev/tty", "r", encoding="utf-8") as handle:
            print(CONFIRM_PROMPT, end="", file=sys.stderr)
            return handle.readline().strip() == CONFIRM_WORD
    except OSError:
        return False


def _teardown(sdk, owned, out) -> None:
    """Stop the loop, disable and print end readings once (mirrors pick_test)."""
    arm_sender = owned.get("arm_sender")
    if arm_sender is not None:
        out(arm_sender.trim_line())
    out("stopping: the motors will be disabled now")
    try:
        sdk.stop_control_loop()
    except Exception:  # noqa: BLE001 - teardown must not fail
        pass
    try:
        sdk.disable_all()
    except Exception:  # noqa: BLE001 - teardown must not fail
        pass
    try:
        arm_end = [math.degrees(v) for v in sdk.groups["arm"].get_positions()]
        grip_end = math.degrees(float(sdk.groups["gripper"].get_positions()[0]))
        out("end readings: arm [" + " ".join(f"{v:+.2f}" for v in arm_end) + "]"
            f" gripper {grip_end:+.2f} deg")
    except Exception as exc:  # noqa: BLE001 - report, never raise here
        out(f"end readings unknown ({exc})")
    try:
        sensor = owned.get("sensor")
        if sensor is not None:
            sensor.close()
    except Exception:  # noqa: BLE001 - teardown must not fail
        pass


def execute_session(sdk, stdin, out=print, find_port=None,
                    sensor_factory=None, joint_limits=None, check_fn=None,
                    tip_fn=None, arm_vlims=None, solve_fn=None,
                    solve_drop_fn=None,
                    use_xy_correction=True, speed_deg_s=SPEED_DEG_S[2],
                    accel_deg_s2=ACCEL_DEG_S2[2], grip_z_mm=GRIP_Z_MM[2],
                    open_mm=OPEN_MM[2], hold_s=HOLD_S[2],
                    find_arm_node=None, read_key=None, open_arm_camera=None,
                    make_client=None, count_frame=None,
                    inspect_recordings_dir="recordings", arm_mode="mit-ff",
                    tau_scale=None, gravity_fn=None, use_station=None,
                    station_j6_deg=None, station_table_shot=False,
                    open_table_camera=None, hold_wait_s=HOLD_WAIT_S,
                    input_fn=None, clock=None, confirm_stdin=False,
                    tty_opener=None):
    """Run one arm session over the stdin targets. --execute only."""
    out(f"arm mode: {arm_mode}")
    lines: queue.Queue = queue.Queue()
    threading.Thread(target=feed_lines, args=(stdin, lines),
                     daemon=True).start()
    if not _confirm_session(
            input_fn, lines, out,
            tty_opener=open if tty_opener is None else tty_opener,
            from_stdin=confirm_stdin):
        out("confirmation missing, nothing was enabled")
        return 2
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    owned: dict = {}
    old_ref = safety.J6_REF
    old_term = None
    try:
        import signal as _signal
        old_term = _signal.getsignal(_signal.SIGTERM)

        def _term(_signum, _frame):
            raise KeyboardInterrupt()

        _signal.signal(_signal.SIGTERM, _term)
    except Exception:  # noqa: BLE001 - without SIGTERM the agent uses SIGINT
        old_term = None
    safety.set_j6_ref(arm_start_deg[5])
    end_reason = "error"
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        if inspect_step.check_prereqs(out, find_arm_node, read_key) is None:
            return 2
        deps = {"grip_start_deg": grip_start_deg, "grip_z_mm": grip_z_mm,
                "open_mm": open_mm, "hold_s": hold_s,
                "joint_limits": joint_limits, "solve_fn": solve_fn,
                "check_fn": check_fn, "tip_fn": tip_fn,
                "use_xy_correction": use_xy_correction,
                "speed_deg_s": speed_deg_s, "accel_deg_s2": accel_deg_s2,
                "solve_drop_fn": solve_drop_fn, "use_station": use_station,
                "station_j6_deg": station_j6_deg}
        hw = {"arm_mode": arm_mode, "tau_scale": tau_scale,
              "gravity_fn": gravity_fn, "arm_vlims": arm_vlims,
              "find_port": find_port, "sensor_factory": sensor_factory,
              "find_arm_node": find_arm_node, "read_key": read_key,
              "open_arm_camera": open_arm_camera, "make_client": make_client,
              "count_frame": count_frame,
              "inspect_recordings_dir": inspect_recordings_dir,
              "station_table_shot": station_table_shot,
              "open_table_camera": open_table_camera,
              "hold_wait_s": hold_wait_s}
        target = session_driver.next_target(lines)
        if target is None:
            end_reason = "eof"
        elif target[0] == "bad":
            end_reason = f"bad line {target[1]!r}"
        elif target[0] != "part":
            end_reason = "end"
        else:
            _, x, y, lever_deg = target
            table_xy = (x, y)
            open_deg, built = session_driver.plan_part(
                arm_start_deg, table_xy, lever_deg, None, out, deps)
            if open_deg is None:
                end_reason = str(built)
            else:
                end_reason = session_driver.run_parts(
                    sdk, lines, out, deps, hw, arm_start_deg,
                    open_deg, built, owned, clock)
        stream = owned.get("stream")
        if stream is not None:
            join_done(stream, clock=clock)
        out(f"session: end ({end_reason})")
        return 0 if end_reason == "end" else 1
    except KeyboardInterrupt:
        out("interrupted by human, holding position")
        stream = owned.get("stream")
        if stream is not None:
            stream.request_end("interrupted")
            join_done(stream, clock=clock)
        out("session: end (interrupted)")
        return 1
    except Exception as exc:  # noqa: BLE001 - session must disable, not crash
        out(f"motion failed: {exc}")
        stream = owned.get("stream")
        if stream is not None:
            stream.request_end(f"failed: {exc}")
            join_done(stream, clock=clock)
        out(f"session: end (failed: {exc})")
        return 1
    finally:
        safety.set_j6_ref(old_ref)
        try:
            if old_term is not None:
                import signal as _signal
                _signal.signal(_signal.SIGTERM, old_term)
        except Exception:  # noqa: BLE001 - teardown must not fail
            pass
        _teardown(sdk, owned, out)
        reader.disconnect()


def dry_run_session(sdk, stdin, out=print, find_port=None,
                    sensor_factory=None, joint_limits=None, check_fn=None,
                    tip_fn=None, solve_fn=None, solve_drop_fn=None,
                    use_xy_correction=True, speed_deg_s=SPEED_DEG_S[2],
                    accel_deg_s2=ACCEL_DEG_S2[2], grip_z_mm=GRIP_Z_MM[2],
                    open_mm=OPEN_MM[2], hold_s=HOLD_S[2],
                    find_arm_node=None, read_key=None, arm_mode="mit-ff",
                    tau_scale=None, gravity_fn=None, use_station=None,
                    station_j6_deg=None, solve_drop_fn_for_mit=None):
    """Check every session plan including the transits; move nothing."""
    out(f"arm mode: {arm_mode}")
    if inspect_step.check_prereqs(out, find_arm_node, read_key) is None:
        return 2
    reader, arm_start_deg, grip_start_deg = _read_start(sdk)
    old_ref = safety.J6_REF
    safety.set_j6_ref(arm_start_deg[5])
    try:
        out(f"rest joints deg: {[f'{v:+.2f}' for v in arm_start_deg]}")
        out(f"gripper rest deg: {grip_start_deg:+.2f}"
            f" (width about {deg_to_width(grip_start_deg):.1f} mm)")
        sensor = open_touch_sensor(find_port, sensor_factory, out)
        if sensor is None:
            return 2
        try:
            deps = {"grip_start_deg": grip_start_deg, "grip_z_mm": grip_z_mm,
                    "open_mm": open_mm, "hold_s": hold_s,
                    "joint_limits": joint_limits, "solve_fn": solve_fn,
                    "check_fn": check_fn, "tip_fn": tip_fn,
                    "use_xy_correction": use_xy_correction,
                    "speed_deg_s": speed_deg_s, "accel_deg_s2": accel_deg_s2,
                    "solve_drop_fn": solve_drop_fn, "use_station": use_station,
                    "station_j6_deg": station_j6_deg}
            lines: queue.Queue = queue.Queue()
            feed_lines(stdin, lines)
            targets = []
            while True:
                item = lines.get()
                if item is _EOF:
                    break
                targets.append(item)
            from muse_qc.pick.session_protocol import parse_session_line

            box_exit = None
            bad_exit = None
            for raw in targets:
                parsed = parse_session_line(raw)
                if parsed is None:
                    out(f"bad session line {raw!r}")
                    return 1
                if parsed[0] != "part":
                    break
                _, x, y, lever_deg = parsed
                table_xy = (x, y)
                start = arm_start_deg if box_exit is None else box_exit
                rest = None if box_exit is None else arm_start_deg
                open_deg, bundle = session_driver.plan_part(
                    start, table_xy, lever_deg, box_exit, out, deps,
                    rest_joints=rest)
                if open_deg is None:
                    out(str(bundle))
                    return 1
                if arm_mode == "mit-ff":
                    for line in pick_mit.dry_run_lines(
                            bundle, None, start, gravity_fn,
                            tau_scale, solve_drop_fn_for_mit):
                        out(line)
                exits = bundle["exits"]
                box_exit = exits["good"]
                if bad_exit is not None:
                    transit = build_box_transit(
                        bad_exit, bundle["hover_joints"],
                        free_speed_deg_s=speed_deg_s,
                        accel_deg_s2=accel_deg_s2, tip_fn=tip_fn)
                    if not transit.get("ok"):
                        out(f"plan refused: {transit.get('reason')}")
                        return 1
                    out("bad-box transit waypoints ok")
                bad_exit = exits["bad"]
            out("dry run: moves nothing, motors never enabled")
            out("session: dry run ok")
            return 0
        finally:
            sensor.close()
    finally:
        safety.set_j6_ref(old_ref)
        reader.disconnect()


def main(argv=None, sdk_factory=None, stdin=None, **kwargs):
    """Entry point: python -m muse_qc.pick.pick_session."""
    parser = argparse.ArgumentParser(
        description="Arm session over stdin targets (human-run only).")
    parser.add_argument("--grip-z-mm", type=_grip_z_type, default=GRIP_Z_MM[2])
    parser.add_argument("--open-mm", type=_open_mm_type, default=OPEN_MM[2])
    parser.add_argument("--hold-s", type=_hold_s_type, default=HOLD_S[2])
    parser.add_argument("--speed-deg-s", type=_speed_type, default=None)
    parser.add_argument("--accel-deg-s2", type=_accel_type, default=None)
    parser.add_argument("--gains", default="vel2")
    parser.add_argument("--arm-mode", choices=("posvel", "mit-ff"),
                        default="mit-ff")
    parser.add_argument("--tau-scale", nargs=6, type=float, default=None)
    parser.add_argument("--no-xy-correction", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--station", dest="station", action="store_true",
                        default=None)
    parser.add_argument("--no-station", dest="station", action="store_false")
    parser.add_argument("--station-j6", type=float, default=None)
    parser.add_argument("--station-table-shot", action="store_true")
    parser.add_argument("--inspect", action="store_true")
    parser.add_argument("--confirm-stdin", action="store_true",
                        help="take the --execute confirmation as the"
                        " first stdin line instead of /dev/tty")
    args = parser.parse_args(argv)
    if not args.inspect:
        parser.error("--inspect is required for a session")
    if args.station_table_shot and args.station is not True:
        parser.error("--station-table-shot needs --station")
    if args.station_j6 is not None and args.station is not True:
        parser.error("--station-j6 needs --station")
    if args.tau_scale is not None and args.arm_mode != "mit-ff":
        parser.error("--tau-scale needs --arm-mode mit-ff")
    if args.speed_deg_s is None:
        args.speed_deg_s = SPEED_DEG_S[2] if args.arm_mode == "mit-ff" else 15.0
    if args.accel_deg_s2 is None:
        args.accel_deg_s2 = ACCEL_DEG_S2[2] if args.arm_mode == "mit-ff" else 30.0
    if args.arm_mode != "mit-ff" and args.speed_deg_s > POSVEL_SPEED_MAX_DEG_S:
        parser.error("--speed-deg-s above 15 needs --arm-mode mit-ff")
    tau_scale = (tuple(args.tau_scale) if args.tau_scale is not None
                 else mit_hold.TAU_SCALE)
    try:
        mit_hold.validate_tau_scale(tau_scale)
    except ValueError as exc:
        parser.error(str(exc))
    out = kwargs.get("out", print)
    if args.gains not in GAIN_PROFILES:
        parser.error(f"--gains must be one of {sorted(GAIN_PROFILES)}")
    for line in describe(args.gains):
        out(line)
    sdk = make_arm(args.gains, factory=sdk_factory)
    run = execute_session if args.execute else dry_run_session
    if stdin is None:
        stdin = sys.stdin
    kwargs.setdefault("use_station", args.station)
    kwargs.setdefault("arm_mode", args.arm_mode)
    kwargs.setdefault("tau_scale", tau_scale)
    kwargs.setdefault("station_j6_deg", args.station_j6)
    kwargs.setdefault("station_table_shot", args.station_table_shot)
    if args.execute:
        kwargs.setdefault("confirm_stdin", args.confirm_stdin)
    return run(sdk, stdin,
               use_xy_correction=not args.no_xy_correction,
               speed_deg_s=args.speed_deg_s, accel_deg_s2=args.accel_deg_s2,
               grip_z_mm=args.grip_z_mm, open_mm=args.open_mm,
               hold_s=args.hold_s, **kwargs)


if __name__ == "__main__":
    sys.exit(main())
