"""MIT arm mode for the pick: sender, tracking guard and abort ramp.

Used by muse_qc.pick.pick_test --arm-mode mit-ff: the arm group flies the
same pick_loop.PickLoop plan through MIT with per-cycle gravity
feed-forward (muse_qc.arm.mit_hold) instead of send_pos_vel, while the
gripper group stays on pos_vel exactly as today. Offline-safe: every
entry point takes an injectable gravity_fn so tests run with a fake
and never touch the bus.
"""

from __future__ import annotations

import math

from muse_qc.arm import mit_hold, safety
from muse_qc.pick.drop_plan import BOXES_TABLE_MM, solve_drop
from muse_qc.bringup.first_move_plan import LOOP_RATE_HZ, SPEED_FRACTION
from muse_qc.arm.mit_hold import MIT_KD, MIT_KP, TAU_SCALE
from muse_qc.pick.pick_plan import FREE_STEP_DEG

# Slow per-joint position trim (software integrator): the SDK gravity
# model does not match the real gripper and camera, so MIT keeps a
# pose-dependent steady-state error (about 3 deg on joint 4 at rest,
# 6 mm in tip height at the grip pose). Adding a fraction of the
# remaining error per second removes it in every pose, like the
# POS_VEL loop does internally, while MIT keeps its calm behaviour.
# The 4 deg clamp keeps a stuck joint from winding the trim up.
# The gate below it keeps fast moves from winding it up either: with
# vel 0 the kd term brakes against the motion, so every joint lags
# its target by about kd*v/kp during a free move (1.7 deg on joints
# 4-6 at 15 deg/s). That lag is motion, not steady-state error, so
# the trim only integrates while every joint target moves at most
# TRIM_GATE_DEG_PER_S since the last send (holds, settles and the
# slow lower/lift ladder).
TRIM_RATE_PER_S = 1.0
TRIM_LIMIT_DEG = 4.0
TRIM_GATE_DEG_PER_S = 5.0


class MitArmSender:
    """Send arm commands through MIT with gravity feed-forward.

    begin(start_rad) switches the group to MIT (before enable, never
    mid-run) and sends the first command at the start pose; send()
    commands later poses the same way with tau = scale * g(q) of the
    commanded pose. Bad scales raise ValueError here, before anything
    is enabled.
    """

    def __init__(self, group, kp=MIT_KP, kd=MIT_KD,
                 tau_scale=TAU_SCALE, gravity_fn=None):
        self.group = group
        self.kp = [float(value) for value in kp]
        self.kd = [float(value) for value in kd]
        self.tau_scale = mit_hold.validate_tau_scale(
            TAU_SCALE if tau_scale is None else tau_scale)
        self.gravity_fn = gravity_fn
        self._trim = [0.0] * 6
        self._frozen = False
        self._last_sent: list[float] | None = None
        self._last_step_deg_per_s = [0.0] * 6

    def _tau(self, joint_rad: list[float]) -> list[float]:
        """Return scale * g(q) per joint for one command in radians."""
        function = (self.gravity_fn if self.gravity_fn is not None
                    else mit_hold.gravity_torque_Nm)
        tau = [float(value) for value in function(list(joint_rad))]
        if len(tau) != 6:
            raise ValueError(f"Need six gravity torques, got {len(tau)}.")
        return [s * value for s, value in zip(self.tau_scale, tau)]

    def begin(self, start_rad: object) -> list[float]:
        """Switch to MIT and send the first command at the start pose."""
        first = [float(value) for value in start_rad]
        self.group.mode_mit()
        self.send(first)
        return list(first)

    def observe(self, measured_deg: object, target_deg: object) -> None:
        """Fold a fraction of the remaining error into the trim.

        Skips the update when frozen, when any reading is not finite,
        or when any joint target moved faster than TRIM_GATE_DEG_PER_S
        since the last send; each joint trim is clamped to
        +-TRIM_LIMIT_DEG.
        """
        if self._frozen:
            return
        measured = [float(value) for value in measured_deg]
        target = [float(value) for value in target_deg]
        if len(measured) != 6 or len(target) != 6:
            return
        if not all(math.isfinite(v) for v in measured + target):
            return
        if any(speed > TRIM_GATE_DEG_PER_S
               for speed in self._last_step_deg_per_s):
            return
        step = TRIM_RATE_PER_S / LOOP_RATE_HZ
        self._trim = [min(max(old + step * (want - got),
                              -TRIM_LIMIT_DEG), TRIM_LIMIT_DEG)
                      for old, got, want in zip(self._trim, measured, target)]

    def freeze(self) -> None:
        """Stop updating the trim, keeping its value."""
        self._frozen = True

    def trim_line(self) -> str:
        """Return one status line with the six trim values in degrees."""
        return "mit trim deg: " + " ".join(f"{v:+.2f}" for v in self._trim)

    def send(self, arm_rad: object) -> None:
        """Command target plus trim through MIT with gravity feed-forward.

        Vel is the unbiased target step times the loop rate (0 on the
        first send and while holding); the trim stays out of it. The
        torque stays scale * g(q) of the unbiased target pose, not of
        the trimmed pose.
        """
        commanded = [float(value) for value in arm_rad]
        if self._last_sent is None:
            vel = [0.0] * len(commanded)
        else:
            vel = [(c - p) * LOOP_RATE_HZ
                   for c, p in zip(commanded, self._last_sent)]
        self._last_step_deg_per_s = [abs(v) * 180.0 / math.pi for v in vel]
        self._last_sent = list(commanded)
        trimmed = [c + math.radians(t)
                   for c, t in zip(commanded, self._trim)]
        self.group.send_mit(
            trimmed, vel=vel,
            kp=self.kp, kd=self.kd, tau=self._tau(commanded),
        )


class TrackingGuard:
    """Trip once when a joint keeps missing its command.

    feed(current_deg, commanded_deg) returns the 1-based joint number
    once any joint stays further than TRACKING_ERR_DEG from its
    command for TRACKING_ERR_CYCLES cycles in a row, else None. After
    the trip every later feed returns None.
    """

    def __init__(self) -> None:
        self._bad = 0
        self._joint = 0
        self._tripped = False

    def feed(self, current_deg: object, commanded_deg: object) -> int | None:
        """Feed one measured/commanded pair, trip after 10 bad cycles."""
        if self._tripped:
            return None
        errors = [abs(c - w) for c, w in zip(current_deg, commanded_deg)]
        worst = max(range(len(errors)), key=lambda joint: errors[joint])
        if errors[worst] <= mit_hold.TRACKING_ERR_DEG:
            self._bad = 0
            return None
        self._bad += 1
        self._joint = worst + 1
        if self._bad < mit_hold.TRACKING_ERR_CYCLES:
            return None
        self._tripped = True
        return self._joint


def feed_guard(loop: object, current_deg: list[float]) -> bool:
    """Feed the loop guard; on a trip hold the measured pose.

    Returns True when the guard tripped: cmd is set to the measured
    joints (kept sending with gravity feed-forward, never disabled
    mid-run in MIT mode) and the existing fault path takes over.
    """
    guard = loop.tracking_guard
    if guard is None:
        return False
    commanded = [math.degrees(v) for v in loop.cmd_arm_rad]
    if loop.arm_sender is not None:
        loop.arm_sender.observe(list(current_deg), commanded)
    tripped = guard.feed(list(current_deg), commanded)
    if tripped is None:
        return False
    if loop.arm_sender is not None:
        loop.arm_sender.freeze()
    loop.cmd_arm_rad = [math.radians(v) for v in current_deg]
    loop._abort_from = list(current_deg)
    loop._fault(f"tracking error on joint {tripped}, holding")
    return True


def maybe_ramp(loop: object) -> None:
    """Insert a ramp when the abort detour starts with a refused step.

    After a tracking trip the arm holds where it actually is, which
    can be far from the first detour waypoint; safety.check_target
    would refuse that jump and the abort would end without walking
    back to rest. When the probe refuses, prepend a ramp from the
    measured pose to the first detour waypoint at the plan free speed
    (FREE_STEP_DEG per cycle, same segment label and grasp floor);
    every ramp step still passes the per-cycle safety check live.
    """
    start = loop._abort_from
    loop._abort_from = None
    if start is None or not loop.detour:
        return
    _, waypoint, grasp = loop.plan[loop.detour[0]]
    tip = loop.tip_of(list(waypoint))
    target = safety.Target(
        x=tip[0], y=tip[1], z=tip[2],
        joints_deg=tuple(waypoint), speed=SPEED_FRACTION)
    here = loop.tip_of([float(v) for v in start])
    current = safety.CurrentState(
        x=here[0], y=here[1], z=here[2],
        joints_deg=tuple(float(v) for v in start), reading_age_ms=0.0)
    extra = {"z_min_m": safety.GRASP_Z_MIN_M} if grasp else {}
    if loop.check(target, current, **extra).get("ok"):
        return
    peak = max(abs(w - s) for w, s in zip(waypoint, start))
    steps = max(1, int(math.ceil(peak / FREE_STEP_DEG)))
    points = [tuple(s + (w - s) * i / steps for s, w in zip(start, waypoint))
              for i in range(1, steps + 1)]
    label = loop.plan[loop.detour[0]][0]
    base = len(loop.plan)
    for point in points:
        loop.plan.append((label, point, grasp))
        loop.tips.append(loop.tip_of(list(point)))
    loop.detour = list(range(base, base + len(points))) + loop.detour


def dry_run_lines(built: dict, drop: object, arm_start_deg: object,
                  gravity_fn: object, tau_scale: object,
                  solve_drop_fn: object = None) -> list[str]:
    """describe_mit plus one gravity block per pose for the dry run."""
    lines = list(mit_hold.describe_mit(tau_scale))
    _, high = built["ranges"]["lower"]
    poses = [("hover", built["hover_joints"]),
             ("grip", built["plan"][high - 1][1])]
    box = drop or "good"
    drop_pose = built.get("drop_joints")
    if drop_pose is None:
        drop_pose = (built.get("drop_joints_by_box") or {}).get(box)
    if drop_pose is None:
        solver = solve_drop_fn if solve_drop_fn is not None else solve_drop
        solved = solver(BOXES_TABLE_MM[box], list(arm_start_deg))
        drop_pose = solved["joints_deg"] if solved.get("ok") else None
    poses.append((f"drop ({box})", drop_pose))
    for label, pose in poses:
        if pose is None:
            lines.append(f"{label} pose unavailable")
            continue
        lines.append(f"{label} pose gravity:")
        lines.extend(mit_hold.gravity_lines(
            pose, gravity_fn=gravity_fn, tau_scale=tau_scale))
    return lines
