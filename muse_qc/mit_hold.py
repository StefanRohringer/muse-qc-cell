"""MIT hold with gravity feed-forward for the hold test.

Alternative arm mode for muse_qc/hold_test.py (--hold-mode mit-ff):
the whole run uses the SDK MIT interface with per-cycle gravity
torque from compute_generalized_gravity instead of the POS_VEL
position loop. Same plan, same per-cycle safety.check_target, same
ramps and loop rate; only the send call changes. The gripper group
is never touched here either.

Offline-safe: the SDK model is imported only inside the gravity
function, and every entry point takes an injectable gravity_fn so
tests run with a fake and never touch the bus.
"""

from __future__ import annotations

import functools
import math

from muse_qc import hover_move
from muse_qc.first_move_plan import read_current_deg
from muse_qc.waypoint_stream import WaypointStream

# MIT position-loop gains. Copied from the SDK DM hardware config
# (config/rebotarm_dm.yaml in the Seeed SDK checkout): joints 1-3
# kp 120 kd 8, joints 4-6 kp 18 kd 2. The SDK loads the same values
# into the arm group, so mode_mit() without arguments and every
# send_mit call below agree with the DM configuration.
MIT_KP = (120.0, 120.0, 120.0, 18.0, 18.0, 18.0)
MIT_KD = (8.0, 8.0, 8.0, 2.0, 2.0, 2.0)
# Gravity feed-forward scale per joint. Constant for now, no CLI flag.
TAU_SCALE = 1.0
# Tracking guard: abort when any arm joint stays further than this
# from its commanded position for this many cycles in a row.
TRACKING_ERR_DEG = 5.0
TRACKING_ERR_CYCLES = 10


@functools.lru_cache(maxsize=1)
def _load_model_data() -> tuple:
    """Load the SDK robot model plus a Pinocchio data object, once.

    Same URDF the SDK config uses: load_robot_model follows
    config/rebotarm.yaml to the hardware yaml and its urdf_path.
    Parsing the URDF every control cycle would be far too slow.
    """
    from reBotArm_control_py.kinematics import load_robot_model

    model = load_robot_model()
    return model, model.createData()


def gravity_torque_Nm(joint_rad: object) -> list[float]:
    """Return the gravity torque g(q) in N*m for six arm joint radians.

    Pure SDK model math through compute_generalized_gravity on the
    cached robot model: no bus, no motion, safe in a dry run.
    """
    values = [float(value) for value in joint_rad]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError("Need six finite joint angles in radians.")
    import numpy as np
    from reBotArm_control_py.dynamics import compute_generalized_gravity
    from reBotArm_control_py.kinematics import (
        get_arm_joint_count,
        pad_q_for_model,
    )

    model, data = _load_model_data()
    full_q = pad_q_for_model(
        model, np.array(values),
        controlled_joints=get_arm_joint_count(model),
    )
    tau = compute_generalized_gravity(model, full_q, data)
    return [float(value) for value in list(tau)[:6]]


def describe_mit() -> list[str]:
    """Return one line each for the MIT gains and the torque scale."""
    return [
        f"mit kp: {' '.join(f'{v:.0f}' for v in MIT_KP)}",
        f"mit kd: {' '.join(f'{v:.0f}' for v in MIT_KD)}",
        f"tau scale: {TAU_SCALE:.1f}",
    ]


def gravity_lines(hover_deg: object, gravity_fn: object = None) -> list[str]:
    """Return one N*m line per joint for the gravity torque at hover."""
    function = gravity_fn if gravity_fn is not None else gravity_torque_Nm
    q_rad = [math.radians(float(value)) for value in hover_deg]
    tau = [float(value) for value in function(q_rad)]
    if len(tau) != 6:
        raise ValueError(f"Need six gravity torques, got {len(tau)}.")
    return [
        f"joint {joint} gravity torque: {value:+.3f} N*m"
        for joint, value in enumerate(tau, start=1)
    ]


def make_begin_fn(
    gravity_fn: object = None,
    kp: object = MIT_KP,
    kd: object = MIT_KD,
    tau_scale: float = TAU_SCALE,
):
    """Build the first_move begin_fn for MIT mode.

    Switches the arm group to MIT (before enable, never mid-run),
    then sends the first command at the measured start pose with
    the config gains and tau = TAU_SCALE * g(q). The gripper group
    is never touched. Returns the first command in radians.
    """
    kp_list = [float(value) for value in kp]
    kd_list = [float(value) for value in kd]
    scale = float(tau_scale)

    def begin(group: object, slow: object) -> list[float]:
        del slow  # MIT sends kp/kd, not velocity limits.
        group.mode_mit()
        first_rad = [math.radians(v) for v in read_current_deg(group)]
        function = gravity_fn if gravity_fn is not None else gravity_torque_Nm
        tau = [scale * float(v) for v in function(list(first_rad))]
        group.send_mit(
            list(first_rad), vel=[0.0] * len(first_rad),
            kp=kp_list, kd=kd_list, tau=tau,
        )
        return list(first_rad)

    return begin


class MitHoldStream(WaypointStream):
    """WaypointStream sending MIT with gravity feed-forward.

    Same waypoints, same per-cycle safety check, same hold/settle
    machinery; only the send call differs. Records the hold-segment
    samples like hold_test.HoldStream (same attribute names, so the
    same report code reads them). Trips the tracking guard when any
    arm joint stays further than TRACKING_ERR_DEG from its commanded
    position for TRACKING_ERR_CYCLES cycles in a row.
    """

    def __init__(
        self,
        *args,
        segments: object = (),
        kp: object = MIT_KP,
        kd: object = MIT_KD,
        tau_scale: float = TAU_SCALE,
        gravity_fn: object = None,
        on_tracking_error: object = None,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        self._segments = list(segments)
        self.hold_goal = None
        self.hold_samples: list[list[float]] = []
        self._kp = [float(value) for value in kp]
        self._kd = [float(value) for value in kd]
        self._tau_scale = float(tau_scale)
        self._gravity_fn = gravity_fn
        self._on_tracking_error = on_tracking_error
        self._bad_cycles = 0
        self._bad_joint = 0
        self.tracking_error: dict | None = None

    def _command_tau(self, joint_rad: list[float]) -> list[float]:
        """Return TAU_SCALE * g(q) for one command in radians."""
        function = (
            self._gravity_fn if self._gravity_fn is not None
            else gravity_torque_Nm
        )
        tau = [float(value) for value in function(list(joint_rad))]
        if len(tau) != 6:
            raise ValueError(f"Need six gravity torques, got {len(tau)}.")
        return [self._tau_scale * value for value in tau]

    def _send(self, joint_rad: list[float]) -> None:
        commanded = [float(value) for value in joint_rad]
        try:
            self.group.send_mit(
                commanded, vel=[0.0] * len(commanded),
                kp=self._kp, kd=self._kd,
                tau=self._command_tau(commanded),
            )
        except Exception:
            pass

    def on_sample(self, current_deg: list[float]) -> None:
        """Keep hold readings and watch the tracking guard."""
        reading = [float(value) for value in current_deg]
        if self._segments:
            index = min(self.index, len(self._segments) - 1)
            if self._segments[index] == hover_move.SEG_HOLD:
                if self.hold_goal is None:
                    self.hold_goal = [float(v) for v in self.waypoints[self.index]]
                self.hold_samples.append(list(reading))
        self._watch_tracking(reading)

    def _watch_tracking(self, current_deg: list[float]) -> None:
        """Abort through refuse/hold/settle after 10 bad cycles in a row."""
        if self.tracking_error is not None:
            return
        if self.refusal is not None or self.halt:
            return
        if self.index >= len(self.waypoints):
            return
        commanded = self.waypoints[self.index]
        errors = [abs(c - w) for c, w in zip(current_deg, commanded)]
        worst = max(range(6), key=lambda joint: errors[joint])
        if errors[worst] <= TRACKING_ERR_DEG:
            self._bad_cycles = 0
            return
        self._bad_cycles += 1
        self._bad_joint = worst + 1
        if self._bad_cycles < TRACKING_ERR_CYCLES:
            return
        self.tracking_error = {"joint": self._bad_joint}
        if self._on_tracking_error is not None:
            try:
                self._on_tracking_error(self._bad_joint)
            except Exception:
                pass
        # Hold where the arm actually is, then abort through the same
        # refuse/hold/settle machinery Ctrl+C uses. Disable stays in
        # the execute finally, never inside the loop.
        self.last_commanded_rad = [math.radians(v) for v in current_deg]
        self._refuse(
            "tracking-error", f"joint {self._bad_joint}",
            f"{TRACKING_ERR_DEG:.1f} deg for {TRACKING_ERR_CYCLES} cycles",
        )


def make_stream_factory(
    captured: dict,
    gravity_fn: object = None,
    on_tracking_error: object = None,
):
    """Build the first_move stream factory keeping a MitHoldStream."""

    def build(group, waypoints, tips, slow, first_rad, check_fn, tip_fn,
              snapshot_index=None):
        stream = MitHoldStream(
            group, waypoints, tips, slow, first_rad,
            check_fn, tip_fn, segments=captured.get("segments", ()),
            gravity_fn=gravity_fn, on_tracking_error=on_tracking_error,
            snapshot_index=snapshot_index,
        )
        captured["stream"] = stream
        return stream

    return build
