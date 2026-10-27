"""Diagnose missing joint feedback on the real arm.

Read-only script: it connects, prints raw get_positions() values of
the arm and gripper groups four times (then every 0.5 s three more
times), and disconnects. It never enables, zeroes, mode-switches, or
moves anything. Values are printed unfiltered, including 0.0.
Warning: disconnect() makes the SDK disable all motors, so the arm can
sag under its own weight; support the arm by hand first.
"""

from __future__ import annotations

import time

SAMPLE_COUNT = 4
SAMPLE_INTERVAL_S = 0.5
ARM_GROUP = "arm"
GRIPPER_GROUP = "gripper"


def _raw_positions(group: object) -> tuple[list[str], list[float]]:
    """Return joint names with raw position values of one group."""
    names = list(group.joint_names)
    positions = [float(value) for value in group.get_positions()]
    return names, positions


def _print_sample(index: int, sdk: object) -> None:
    """Read arm and gripper groups once and print raw values."""
    print(f"sample {index + 1}/{SAMPLE_COUNT}")
    for group_name in (ARM_GROUP, GRIPPER_GROUP):
        group = sdk.groups[group_name]
        names, positions = _raw_positions(group)
        if not names:
            print(f"  {group_name}: (no joints reported)")
            continue
        for name, value in zip(names, positions):
            print(f"  {group_name}.{name} = {value!r}")


def main() -> int:
    """Connect, print four raw samples, disconnect. Returns exit code."""
    from reBotArm_control_py.actuator import RebotArm

    sdk = RebotArm()
    sdk.connect()
    try:
        for index in range(SAMPLE_COUNT):
            if index > 0:
                time.sleep(SAMPLE_INTERVAL_S)
            _print_sample(index, sdk)
    finally:
        sdk.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
