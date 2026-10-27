"""Read the arm channel the Seeed SDK will open (read-only).

Locates the installed reBotArm_control_py package with
importlib.util.find_spec only (it is never imported), reads
config/rebotarm.yaml, follows hardware_yaml to the hardware config
and returns its "channel" value. Returns None when the channel
cannot be read. No serial port is ever opened. Only the standard
library is used.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path

_PACKAGE_NAME = "reBotArm_control_py"
_GLOBAL_CONFIG_NAME = "rebotarm.yaml"


def _scalar(path: Path, key: str) -> str | None:
    """Return a top-level-ish "key: value" scalar without a YAML parser."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        name, _, value = stripped.partition(":")
        if name.strip() != key:
            continue
        cleaned = value.split("#", 1)[0].strip().strip("\"'")
        return cleaned or None
    return None


def _default_config_dir() -> Path | None:
    """Return the SDK config folder next to the installed package."""
    try:
        spec = importlib.util.find_spec(_PACKAGE_NAME)
    except (ImportError, AttributeError, ValueError):
        return None
    if spec is None:
        return None
    package_dir: Path | None = None
    locations = spec.submodule_search_locations
    if locations:
        package_dir = Path(next(iter(locations)))
    elif spec.origin:
        package_dir = Path(spec.origin).parent
    if package_dir is None:
        return None
    for candidate in (package_dir.parent / "config", package_dir / "config"):
        try:
            if candidate.is_dir():
                return candidate
        except OSError:
            continue
    return None


def read_sdk_channel(
    config_dir: str | os.PathLike[str] | None = None,
) -> str | None:
    """Return the SDK arm channel, or None when it cannot be read.

    config_dir is the folder holding rebotarm.yaml, so tests can pass
    a fake folder. With None, the folder is located next to the
    installed SDK package (found, never imported).
    """
    if config_dir is not None:
        base = Path(config_dir)
    else:
        found = _default_config_dir()
        if found is None:
            return None
        base = found
    hardware_name = _scalar(base / _GLOBAL_CONFIG_NAME, "hardware_yaml")
    if not hardware_name:
        return None
    hardware_path = (
        Path(hardware_name)
        if Path(hardware_name).is_absolute()
        else base / hardware_name
    )
    return _scalar(hardware_path, "channel")
