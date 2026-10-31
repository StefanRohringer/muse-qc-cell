"""Tests for diagnose_joints.py printing. No hardware is touched."""

from __future__ import annotations

from muse_qc.bringup import diagnose_joints


class FakeGroup:
    """Mimics one SDK joint group read surface."""

    def __init__(self, joint_names, positions):
        self.joint_names = list(joint_names)
        self._positions = list(positions)

    def get_positions(self):
        return list(self._positions)


class FakeSdk:
    """Mimics the SDK group mapping used by _print_sample."""

    def __init__(self, groups):
        self.groups = dict(groups)


def test_empty_group_reported_per_group(capsys):
    sdk = FakeSdk(
        {
            "arm": FakeGroup([], []),
            "gripper": FakeGroup(["gripper"], [0.3]),
        }
    )
    diagnose_joints._print_sample(0, sdk)
    out = capsys.readouterr().out
    assert "arm: (no joints reported)" in out
    assert "gripper.gripper = 0.3" in out


def test_populated_groups_print_raw_values(capsys):
    sdk = FakeSdk(
        {
            "arm": FakeGroup(["joint1"], [0.5]),
            "gripper": FakeGroup(["gripper"], [0.0]),
        }
    )
    diagnose_joints._print_sample(0, sdk)
    out = capsys.readouterr().out
    assert "arm.joint1 = 0.5" in out
    assert "gripper.gripper = 0.0" in out
    assert "(no joints reported)" not in out
