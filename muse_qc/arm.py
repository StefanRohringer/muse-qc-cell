"""Read-only joint-angle access for the reBot B601-DM arm.

This module wraps the official Seeed SDK object (RebotArm) and exposes
only three things: connect, reading joint angles and gripper state, and
disconnect. Enable, zeroing, mode switches and send commands of the SDK
stay hidden inside the private SDK object, so using this module can
never move the arm.

The SDK reports 0.0 for joints without feedback, so an exact 0.0 reading
cannot be told apart from missing data. This module therefore raises
JointStateUnavailable instead of ever reporting 0.0 as an angle.

The first reading right after connect() may contain 0.0 values before
real feedback arrives, so reads are retried while any value is exactly
0.0. Only when 0.0 persists after all attempts is it reported missing.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass

ARM_GROUP = "arm"
GRIPPER_GROUP = "gripper"
DEFAULT_MAX_READ_ATTEMPTS = 5
DEFAULT_READ_RETRY_DELAY_S = 0.2


class ArmError(Exception):
    """Base error for read-only arm access."""


class NotConnectedError(ArmError):
    """Raised when reading without an active connection."""


class JointStateUnavailable(ArmError):
    """Raised when the SDK reports no real joint feedback."""


@dataclass(frozen=True)
class JointAngle:
    """One arm joint angle with its joint name."""

    name: str
    radians: float
    degrees: float


@dataclass(frozen=True)
class GripperState:
    """Gripper position with its joint name."""

    name: str
    radians: float
    degrees: float


def _default_sdk_factory() -> object:
    """Create the real SDK object. Called only by connect()."""
    from reBotArm_control_py.actuator import RebotArm

    return RebotArm()


def _checked_angle(name: str, value: float) -> JointAngle:
    """Build a JointAngle, rejecting missing feedback disguised as 0.0."""
    if not math.isfinite(value):
        raise JointStateUnavailable(
            f"No joint feedback for {name!r}: reading {value!r} is not finite."
        )
    if value == 0.0:
        raise JointStateUnavailable(
            f"No joint feedback for {name!r}: the SDK reports 0.0 when "
            "feedback is missing, so 0.0 is never reported as an angle."
        )
    return JointAngle(name=name, radians=value, degrees=math.degrees(value))


class Arm:
    """Read-only wrapper around the Seeed RebotArm SDK object."""

    def __init__(
        self,
        sdk_factory: Callable[[], object] | None = None,
        max_read_attempts: int = DEFAULT_MAX_READ_ATTEMPTS,
        read_retry_delay_s: float = DEFAULT_READ_RETRY_DELAY_S,
    ) -> None:
        if max_read_attempts < 1:
            raise ValueError("max_read_attempts must be at least 1.")
        if read_retry_delay_s < 0:
            raise ValueError("read_retry_delay_s must not be negative.")
        self._sdk_factory = sdk_factory or _default_sdk_factory
        self._max_read_attempts = max_read_attempts
        self._read_retry_delay_s = read_retry_delay_s
        self._sdk: object | None = None

    @property
    def connected(self) -> bool:
        """Whether a connection is currently open."""
        return self._sdk is not None

    def connect(self) -> None:
        """Open the bus connection. Reads feedback only, moves nothing."""
        if self._sdk is not None:
            return
        sdk = self._sdk_factory()
        sdk.connect()
        self._sdk = sdk

    def disconnect(self) -> None:
        """Close the bus connection. Safe to call when disconnected.

        Warning: the SDK disables all motors on disconnect, so the arm
        can sag under its own weight. Support the arm by hand first.
        """
        if self._sdk is None:
            return
        try:
            self._sdk.disconnect()
        finally:
            self._sdk = None

    def _group(self, group_name: str) -> object:
        """Return one SDK joint group, or fail when disconnected."""
        if self._sdk is None:
            raise NotConnectedError(
                "Arm is not connected. Call connect() before reading."
            )
        try:
            return self._sdk.groups[group_name]
        except KeyError:
            raise JointStateUnavailable(
                f"Joint group {group_name!r} is not configured."
            ) from None

    @staticmethod
    def _group_positions(group: object) -> tuple[list[str], list[float]]:
        """Return joint names with raw radian positions of one group."""
        names = list(group.joint_names)
        positions = [float(value) for value in group.get_positions()]
        if len(positions) != len(names):
            raise JointStateUnavailable(
                f"Got {len(positions)} positions for {len(names)} joints."
            )
        return names, positions

    def _read_group_with_retry(self, group_name: str) -> tuple[list[str], list[float]]:
        """Return one group's names and positions, retrying early 0.0 reads.

        Repeats the position query while any value is exactly 0.0, which
        the SDK also reports when feedback has not arrived yet. Returns
        the first reading without 0.0, or the last reading when 0.0
        persists after all attempts. Reads feedback only, moves nothing.
        """
        names: list[str] = []
        positions: list[float] = []
        for attempt in range(self._max_read_attempts):
            names, positions = self._group_positions(self._group(group_name))
            if not any(value == 0.0 for value in positions):
                return names, positions
            if attempt < self._max_read_attempts - 1:
                time.sleep(self._read_retry_delay_s)
        return names, positions

    def read_joints(self) -> list[JointAngle]:
        """Return arm joint angles in radians and degrees, with names."""
        names, positions = self._read_group_with_retry(ARM_GROUP)
        return [
            _checked_angle(name, value) for name, value in zip(names, positions)
        ]

    def read_gripper(self) -> GripperState:
        """Return the gripper position in radians and degrees, with name."""
        names, positions = self._read_group_with_retry(GRIPPER_GROUP)
        if not names:
            raise JointStateUnavailable("No gripper is configured.")
        if len(names) != 1:
            raise JointStateUnavailable(
                f"Expected one gripper joint, found {len(names)}: {names}."
            )
        checked = _checked_angle(names[0], positions[0])
        return GripperState(
            name=checked.name, radians=checked.radians, degrees=checked.degrees
        )
