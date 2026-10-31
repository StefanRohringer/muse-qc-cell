"""Tests for sdk_config.py. No hardware is touched; config is faked."""

from __future__ import annotations

from muse_qc.arm.sdk_config import read_sdk_channel


def _write_config(root, hardware_name, hardware_text):
    (root / "rebotarm.yaml").write_text(
        f'hardware_yaml: "{hardware_name}"\n', encoding="utf-8"
    )
    (root / hardware_name).write_text(hardware_text, encoding="utf-8")


def test_channel_found(tmp_path):
    _write_config(tmp_path, "hw.yaml", "channel: /dev/ttyACM0\nrate: 500\n")
    assert read_sdk_channel(tmp_path) == "/dev/ttyACM0"


def test_hardware_yaml_is_followed(tmp_path):
    _write_config(tmp_path, "custom.yaml", "channel: can0\nrate: 500\n")
    assert read_sdk_channel(tmp_path) == "can0"


def test_missing_global_config_returns_none(tmp_path):
    assert read_sdk_channel(tmp_path) is None


def test_missing_hardware_config_returns_none(tmp_path):
    (tmp_path / "rebotarm.yaml").write_text(
        "hardware_yaml: no-such.yaml\n", encoding="utf-8"
    )
    assert read_sdk_channel(tmp_path) is None


def test_missing_channel_returns_none(tmp_path):
    _write_config(tmp_path, "hw.yaml", "rate: 500\n")
    assert read_sdk_channel(tmp_path) is None


def test_missing_hardware_yaml_key_returns_none(tmp_path):
    (tmp_path / "rebotarm.yaml").write_text("rate: 500\n", encoding="utf-8")
    assert read_sdk_channel(tmp_path) is None
