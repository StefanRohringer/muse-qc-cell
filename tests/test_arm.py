"""Tests for arm.py using a fake RebotArm. No hardware is touched."""

from __future__ import annotations

import math

import pytest

from muse_qc.arm import Arm, ArmError, GripperState, JointAngle, JointStateUnavailable, NotConnectedError

ARM_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ARM_POSITIONS = [0.5, -1.0, 0.25, 1.5, -0.75, 2.0]
GRIPPER_NAMES = ["gripper"]
GRIPPER_POSITIONS = [0.3]

FORBIDDEN_ARM_ATTRS = [
    "enable_all",
    "disable_all",
    "set_zero",
    "estop",
    "reconnect",
    "start_control_loop",
    "stop_control_loop",
    "add_group",
    "enable",
    "disable",
    "mode_mit",
    "mode_pos_vel",
    "mode_vel",
    "send_mit",
    "send_pos_vel",
    "send_vel",
]

ALLOWED_CALLS = {"connect", "arm.get_positions", "gripper.get_positions", "disconnect"}


class FakeGroup:
    """Mimics the SDK JointGroup read surface and traps motion commands."""

    def __init__(self, log, name, joint_names, positions, position_frames=None):
        self._log = log
        self._name = name
        self._joint_names = list(joint_names)
        self._positions = list(positions)
        self._frames = None
        if position_frames is not None:
            self._frames = [list(frame) for frame in position_frames]
        self._calls = 0

    @property
    def joint_names(self):
        return list(self._joint_names)

    def get_positions(self):
        self._log.append(f"{self._name}.get_positions")
        if self._frames is None:
            return list(self._positions)
        index = min(self._calls, len(self._frames) - 1)
        self._calls += 1
        return list(self._frames[index])

    def _forbidden(self, method):
        self._log.append(f"{self._name}.{method}")
        raise AssertionError(f"{method} must never be called through arm.py")

    def enable(self):
        self._forbidden("enable")

    def disable(self):
        self._forbidden("disable")

    def mode_mit(self):
        self._forbidden("mode_mit")

    def mode_pos_vel(self):
        self._forbidden("mode_pos_vel")

    def mode_vel(self):
        self._forbidden("mode_vel")

    def send_mit(self, *args, **kwargs):
        self._forbidden("send_mit")

    def send_pos_vel(self, *args, **kwargs):
        self._forbidden("send_pos_vel")

    def send_vel(self, *args, **kwargs):
        self._forbidden("send_vel")


class FakeRebotArm:
    """Mimics the SDK RebotArm read surface and traps motion commands."""

    def __init__(self, arm_positions=None, gripper_positions=None, arm_frames=None, gripper_frames=None):
        self.calls: list[str] = []
        self._groups = {
            "arm": FakeGroup(
                self.calls, "arm", ARM_NAMES, arm_positions if arm_positions is not None else ARM_POSITIONS, arm_frames
            ),
            "gripper": FakeGroup(
                self.calls,
                "gripper",
                GRIPPER_NAMES,
                gripper_positions if gripper_positions is not None else GRIPPER_POSITIONS,
                gripper_frames,
            ),
        }

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    @property
    def groups(self):
        return self._groups

    def _forbidden(self, method):
        self.calls.append(method)
        raise AssertionError(f"{method} must never be called through arm.py")

    def enable_all(self):
        self._forbidden("enable_all")

    def disable_all(self):
        self._forbidden("disable_all")

    def set_zero(self):
        self._forbidden("set_zero")

    def estop(self):
        self._forbidden("estop")

    def reconnect(self):
        self._forbidden("reconnect")

    def start_control_loop(self, *args, **kwargs):
        self._forbidden("start_control_loop")

    def stop_control_loop(self):
        self._forbidden("stop_control_loop")

    def add_group(self, *args, **kwargs):
        self._forbidden("add_group")


def make_arm(**kwargs):
    arm_kwargs = {}
    for key in ("max_read_attempts", "read_retry_delay_s"):
        if key in kwargs:
            arm_kwargs[key] = kwargs.pop(key)
    fake = FakeRebotArm(**kwargs)
    return Arm(sdk_factory=lambda: fake, **arm_kwargs), fake


def test_read_joints_returns_names_degrees_and_radians():
    arm, _ = make_arm()
    arm.connect()
    try:
        joints = arm.read_joints()
    finally:
        arm.disconnect()
    assert [joint.name for joint in joints] == ARM_NAMES
    for joint, expected in zip(joints, ARM_POSITIONS):
        assert isinstance(joint, JointAngle)
        assert joint.radians == pytest.approx(expected)
        assert joint.degrees == pytest.approx(math.degrees(expected))


def test_read_gripper_returns_state():
    arm, _ = make_arm()
    arm.connect()
    try:
        state = arm.read_gripper()
    finally:
        arm.disconnect()
    assert isinstance(state, GripperState)
    assert state.name == "gripper"
    assert state.radians == pytest.approx(GRIPPER_POSITIONS[0])
    assert state.degrees == pytest.approx(math.degrees(GRIPPER_POSITIONS[0]))


def test_read_never_triggers_enable_zero_or_send_commands():
    arm, fake = make_arm()
    arm.connect()
    try:
        arm.read_joints()
        arm.read_gripper()
    finally:
        arm.disconnect()
    assert set(fake.calls) <= ALLOWED_CALLS
    assert "connect" in fake.calls
    assert "arm.get_positions" in fake.calls
    assert "gripper.get_positions" in fake.calls


def test_missing_state_raises_instead_of_reporting_zero():
    arm, _ = make_arm(arm_positions=[0.0] * len(ARM_NAMES))
    arm.connect()
    try:
        with pytest.raises(JointStateUnavailable):
            arm.read_joints()
    finally:
        arm.disconnect()


def test_single_zero_reading_raises():
    positions = list(ARM_POSITIONS)
    positions[2] = 0.0
    arm, _ = make_arm(arm_positions=positions)
    arm.connect()
    try:
        with pytest.raises(JointStateUnavailable):
            arm.read_joints()
    finally:
        arm.disconnect()


def test_read_joints_retries_initial_zero_then_reports_real_values():
    frames = [[0.0] * len(ARM_NAMES), list(ARM_POSITIONS)]
    arm, fake = make_arm(arm_frames=frames, read_retry_delay_s=0)
    arm.connect()
    try:
        joints = arm.read_joints()
    finally:
        arm.disconnect()
    assert [joint.radians for joint in joints] == pytest.approx(ARM_POSITIONS)
    assert fake.calls.count("arm.get_positions") == 2


def test_read_gripper_retries_initial_zero_then_reports_real_value():
    arm, fake = make_arm(gripper_frames=[[0.0], list(GRIPPER_POSITIONS)], read_retry_delay_s=0)
    arm.connect()
    try:
        state = arm.read_gripper()
    finally:
        arm.disconnect()
    assert state.radians == pytest.approx(GRIPPER_POSITIONS[0])
    assert fake.calls.count("gripper.get_positions") == 2


def test_persistent_zero_still_raises_after_all_attempts():
    arm, fake = make_arm(arm_positions=[0.0] * len(ARM_NAMES), read_retry_delay_s=0)
    arm.connect()
    try:
        with pytest.raises(JointStateUnavailable):
            arm.read_joints()
    finally:
        arm.disconnect()
    assert fake.calls.count("arm.get_positions") == 5


def test_persistent_gripper_zero_still_raises_after_all_attempts():
    arm, fake = make_arm(gripper_positions=[0.0], read_retry_delay_s=0)
    arm.connect()
    try:
        with pytest.raises(JointStateUnavailable):
            arm.read_gripper()
    finally:
        arm.disconnect()
    assert fake.calls.count("gripper.get_positions") == 5


def test_missing_gripper_state_raises():
    arm, _ = make_arm(gripper_positions=[0.0])
    arm.connect()
    try:
        with pytest.raises(JointStateUnavailable):
            arm.read_gripper()
    finally:
        arm.disconnect()


def test_non_finite_reading_raises():
    positions = list(ARM_POSITIONS)
    positions[0] = float("nan")
    arm, _ = make_arm(arm_positions=positions)
    arm.connect()
    try:
        with pytest.raises(JointStateUnavailable):
            arm.read_joints()
    finally:
        arm.disconnect()


def test_read_without_connect_raises():
    arm, _ = make_arm()
    with pytest.raises(NotConnectedError):
        arm.read_joints()
    with pytest.raises(NotConnectedError):
        arm.read_gripper()


def test_motion_commands_are_not_reachable():
    arm, _ = make_arm()
    for name in FORBIDDEN_ARM_ATTRS:
        assert not hasattr(arm, name), name


def test_errors_share_base_class():
    assert issubclass(NotConnectedError, ArmError)
    assert issubclass(JointStateUnavailable, ArmError)


def test_creating_arm_without_factory_touches_no_hardware():
    arm = Arm()
    assert not arm.connected
