"""MIT arm mode for the pick: sender, tracking guard and abort ramp.

Used by muse_qc.pick_test --arm-mode mit-ff: the arm group flies the
same pick_loop.PickLoop plan through MIT with per-cycle gravity
feed-forward (muse_qc.mit_hold) instead of send_pos_vel, while the
gripper group stays on pos_vel exactly as today. Offline-safe: every
entry point takes an injectable gravity_fn so tests run with a fake
and never touch the bus.
"""

from __future__ import annotations

import math

from muse_qc import mit_hold, safety
from muse_qc.drop_plan import BOXES_TABLE_MM, solve_drop
from muse_qc.first_move_plan import SPEED_FRACTION
from muse_qc.mit_hold import MIT_KD, MIT_KP, TAU_SCALE
from muse_qc.pick_plan import FREE_STEP_DEG


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
        self.group.send_mit(
            first, vel=[0.0] * len(first),
            kp=self.kp, kd=self.kd, tau=self._tau(first),
        )
        return list(first)

    def send(self, arm_rad: object) -> None:
        """Command one arm pose through MIT with gravity feed-forward."""
        commanded = [float(value) for value in arm_rad]
        self.group.send_mit(
            commanded, vel=[0.0] * len(commanded),
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
    tripped = guard.feed(list(current_deg), commanded)
    if tripped is None:
        return False
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
