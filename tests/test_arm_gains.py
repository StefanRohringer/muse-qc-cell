"""Tests for arm_gains.py. Offline only: fake SDK config dir, no bus."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

import muse_qc.arm_gains as arm_gains
from muse_qc.arm_gains import (
    GAIN_PROFILES,
    describe,
    make_arm,
    write_runtime_yaml,
)

HARDWARE_YAML = """\
channel: /dev/ttyACM0
rate: 500
groups:
  arm:
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
  gripper:
    joints: [gripper]
joints:
  - name: joint1
    motor_id: 1
    feedback_id: 17
    model: "4340P"
    vendor: "damiao"
    MIT:
      kp: 120.0
      kd: 8.0
    POS_VEL:
      vel_kp: 0.0125
      vel_ki: 0.004
      pos_kp: 150.0
      pos_ki: 0.5
      vlim: 5.0
  - name: joint4
    motor_id: 4
    feedback_id: 20
    model: "4310"
    vendor: "damiao"
    MIT:
      kp: 18.0
      kd: 2.0
    POS_VEL:
      vel_kp: 0.0008
      vel_ki: 0.002
      pos_kp: 70.0
      pos_ki: 1.0
      vlim: 3.0
  - name: joint5
    motor_id: 5
    feedback_id: 21
    model: "4310"
    vendor: "damiao"
    MIT:
      kp: 18.0
      kd: 2.0
    POS_VEL:
      vel_kp: 0.0008
      vel_ki: 0.002
      pos_kp: 60.0
      pos_ki: 1.0
      vlim: 3.0
  - name: joint6
    motor_id: 6
    feedback_id: 22
    model: "4310"
    vendor: "damiao"
    MIT:
      kp: 18.0
      kd: 2.0
    POS_VEL:
      vel_kp: 0.0008
      vel_ki: 0.002
      pos_kp: 60.0
      pos_ki: 1.0
      vlim: 3.0
"""


def _fake_config(tmp_path):
    """Write a fake SDK config dir, point arm_gains at it."""
    tmp_path.mkdir(parents=True, exist_ok=True)
    (tmp_path / "rebotarm.yaml").write_text(
        'hardware_yaml: "rebotarm_dm.yaml"\n', encoding="utf-8"
    )
    (tmp_path / "rebotarm_dm.yaml").write_text(
        HARDWARE_YAML, encoding="utf-8"
    )
    return tmp_path


def _use_fake_config(monkeypatch, tmp_path):
    config = _fake_config(tmp_path / "sdkcfg")
    monkeypatch.setattr(
        arm_gains, "_hardware_yaml_path",
        lambda config_dir=None: config / "rebotarm_dm.yaml",
    )


def test_soft_profile_only_lists_joints_4_and_5():
    assert set(GAIN_PROFILES["soft"]) == {4, 5}
    assert GAIN_PROFILES["default"] == {}


def test_stiff_profile_only_lists_joints_4_and_5():
    assert set(GAIN_PROFILES["stiff"]) == {4, 5}
    for joint in (4, 5):
        change = GAIN_PROFILES["stiff"][joint]
        assert change["pos_kp_factor"] == pytest.approx(1.3)
        assert change["vel_kp_factor"] == pytest.approx(1.5)
        assert "pos_ki" not in change
        assert "vel_ki" not in change


def test_describe_soft_lines(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    assert describe("soft") == [
        "joint 4: pos_kp 70 -> 49, pos_ki 1.0 -> 0.0",
        "joint 5: pos_kp 60 -> 42, pos_ki 1.0 -> 0.0",
    ]
    assert describe("default") == []


def test_vel2_profile_only_lists_joints_4_and_5():
    assert set(GAIN_PROFILES["vel2"]) == {4, 5}
    for joint in (4, 5):
        change = GAIN_PROFILES["vel2"][joint]
        assert change["vel_kp_factor"] == pytest.approx(2.0)
        assert "pos_kp_factor" not in change
        assert "pos_ki" not in change
        assert "vel_ki" not in change


def test_describe_vel2_lines(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    assert describe("vel2") == [
        "joint 4: vel_kp 0.0008 -> 0.0016",
        "joint 5: vel_kp 0.0008 -> 0.0016",
    ]


def test_vel2_runtime_yaml_round_trips(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    out_dir = tmp_path / "runtime"
    path = write_runtime_yaml("vel2", out_dir)
    assert path.is_absolute()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    loops = {entry["name"]: entry["POS_VEL"] for entry in data["joints"]}
    assert loops["joint4"]["vel_kp"] == pytest.approx(0.0016)
    assert loops["joint5"]["vel_kp"] == pytest.approx(0.0016)
    assert loops["joint4"]["pos_kp"] == 70.0
    assert loops["joint5"]["pos_kp"] == 60.0
    assert loops["joint4"]["pos_ki"] == 1.0
    assert loops["joint4"]["vel_ki"] == 0.002
    assert loops["joint5"]["pos_ki"] == 1.0
    assert loops["joint5"]["vel_ki"] == 0.002
    assert loops["joint1"]["vel_kp"] == 0.0125
    assert loops["joint6"] == {
        "vel_kp": 0.0008, "vel_ki": 0.002,
        "pos_kp": 60.0, "pos_ki": 1.0, "vlim": 3.0,
    }


def test_describe_stiff_lines(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    assert describe("stiff") == [
        "joint 4: pos_kp 70 -> 91, vel_kp 0.0008 -> 0.0012",
        "joint 5: pos_kp 60 -> 78, vel_kp 0.0008 -> 0.0012",
    ]


def test_runtime_yaml_outside_repo_and_round_trips(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    out_dir = tmp_path / "runtime"
    path = write_runtime_yaml("soft", out_dir)
    assert path.is_absolute()
    assert path.parent == out_dir
    assert Path.cwd() not in path.parents
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    loops = {entry["name"]: entry["POS_VEL"] for entry in data["joints"]}
    assert loops["joint4"]["pos_kp"] == 49.0
    assert loops["joint4"]["pos_ki"] == 0.0
    assert loops["joint5"]["pos_kp"] == 42.0
    assert loops["joint5"]["pos_ki"] == 0.0
    assert loops["joint6"] == {
        "vel_kp": 0.0008, "vel_ki": 0.002,
        "pos_kp": 60.0, "pos_ki": 1.0, "vlim": 3.0,
    }
    assert loops["joint4"]["vel_kp"] == 0.0008
    assert loops["joint4"]["vel_ki"] == 0.002
    assert loops["joint1"]["pos_kp"] == 150.0
    assert loops["joint1"]["pos_ki"] == 0.5


def test_stiff_runtime_yaml_round_trips(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    out_dir = tmp_path / "runtime"
    path = write_runtime_yaml("stiff", out_dir)
    assert path.is_absolute()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    loops = {entry["name"]: entry["POS_VEL"] for entry in data["joints"]}
    assert loops["joint4"]["pos_kp"] == pytest.approx(91.0)
    assert loops["joint4"]["vel_kp"] == pytest.approx(0.0012)
    assert loops["joint5"]["pos_kp"] == pytest.approx(78.0)
    assert loops["joint5"]["vel_kp"] == pytest.approx(0.0012)
    assert loops["joint4"]["pos_ki"] == 1.0
    assert loops["joint4"]["vel_ki"] == 0.002
    assert loops["joint5"]["pos_ki"] == 1.0
    assert loops["joint5"]["vel_ki"] == 0.002
    assert loops["joint1"]["pos_kp"] == 150.0
    assert loops["joint1"]["vel_kp"] == 0.0125
    assert loops["joint1"]["pos_ki"] == 0.5
    assert loops["joint6"] == {
        "vel_kp": 0.0008, "vel_ki": 0.002,
        "pos_kp": 60.0, "pos_ki": 1.0, "vlim": 3.0,
    }


def test_make_arm_stiff_passes_runtime_yaml(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return "sdk-stiff"

    assert make_arm("stiff", factory=factory) == "sdk-stiff"
    (kwargs,) = seen
    runtime = Path(kwargs["hw_yaml"])
    assert runtime.is_file()
    data = yaml.safe_load(runtime.read_text(encoding="utf-8"))
    loops = {entry["name"]: entry["POS_VEL"] for entry in data["joints"]}
    assert loops["joint4"]["pos_kp"] == pytest.approx(91.0)
    assert loops["joint4"]["vel_kp"] == pytest.approx(0.0012)


def test_default_returns_none_and_writes_nothing(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    out_dir = tmp_path / "runtime"
    assert write_runtime_yaml("default", out_dir) is None
    assert not out_dir.exists()


def test_unknown_profile_raises():
    with pytest.raises(ValueError):
        write_runtime_yaml("firm", "/tmp")


def test_make_arm_default_calls_factory_without_args():
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return "sdk-default"

    assert make_arm("default", factory=factory) == "sdk-default"
    assert seen == [{}]


def test_make_arm_soft_passes_runtime_yaml(monkeypatch, tmp_path):
    _use_fake_config(monkeypatch, tmp_path)
    seen = []

    def factory(**kwargs):
        seen.append(kwargs)
        return "sdk-soft"

    assert make_arm("soft", factory=factory) == "sdk-soft"
    (kwargs,) = seen
    runtime = Path(kwargs["hw_yaml"])
    assert runtime.is_file()
    assert Path.cwd() not in runtime.parents
    data = yaml.safe_load(runtime.read_text(encoding="utf-8"))
    loops = {entry["name"]: entry["POS_VEL"] for entry in data["joints"]}
    assert loops["joint4"]["pos_kp"] == 49.0
    assert loops["joint5"]["pos_ki"] == 0.0
