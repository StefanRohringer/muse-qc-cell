# Third-Party Components

Only official Seeed sources are used for the reBot Arm B601-DM.
No third-party code is copied into this repository; everything is
referenced as a package dependency.

## motorbridge (installed)

- Purpose: multi-motor coordination SDK used by the reBot arm to talk
  to the Damiao joint motors (serial/CAN transport, MIT and POS_VEL modes).
- Source (PyPI): `motorbridge>=0.4.9` (resolved to 0.5.6 on 2026-10-26).
  The lower bound matches the minimum required by the SDK below.
- Upstream repository: https://github.com/motorbridge/motorbridge
- Official Seeed documentation: https://wiki.seeedstudio.com/rebot_b601_dm_getting_started/
  (`pip install motorbridge` is the documented install step).
- License: MIT (PyPI metadata and LICENSE file in the upstream repository).
- Installed with: `uv add "motorbridge>=0.4.9"`.

## reBotArm_control_py (installed, editable)

- Purpose: official Seeed Python SDK for the reBot arm (kinematics via
  Pinocchio, trajectory planning, `RebotArm` hardware abstraction with
  `connect()`, `get_positions()` and `disconnect()`).
- Source: https://github.com/Seeed-Projects/reBotArm_control_py
- Version: package version 0.1.0, pinned to commit
  `d4f3591bc0183de03f3ae8651298db0269aadfdd` (main, 2026-10-21).
- Official Seeed documentation:
  https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/
- License: MIT (per the license badge on the official Seeed Wiki pages;
  no LICENSE file was found in the SDK repository itself, so please
  re-check upstream before redistributing).
- Status: installed as an editable dependency from a local checkout at
  `~/meta-hackathon/seeed-sdk` (outside this repository, so no foreign
  code is copied in). Declared in `pyproject.toml` via
  `[tool.uv.sources]` (`path = "../seeed-sdk", editable = true`) and
  pinned in `uv.lock`, so `uv sync` keeps it installed. Installed with:
  `uv add --editable ../seeed-sdk`.
- Requires Python `>=3.10,<3.12` (no Pinocchio build for 3.12), which is
  why this project was switched from Python 3.12 to Python 3.11.
- Local checkout changes (kept outside this repository):
  `pyproject.toml` gained `[tool.setuptools.packages.find]` with
  `include = ["reBotArm_control_py*"]` to fix the upstream setuptools
  packaging bug (`Multiple top-level packages discovered in a
  flat-layout` for `urdf`, `config`, `reBotArm_control_py`);
  `config/rebotarm.yaml` sets `hardware_yaml: "rebotarm_dm.yaml"`
  (upstream default is the RS arm).
- Verified 2026-10-26: `import reBotArm_control_py` succeeds under
  Python 3.11. No `connect()` call, no hardware touched.

## Author's public gripper project (declared, not vendored)

- Purpose: tactile gripper reporting grip and contact; contact triggers STOP.
- Link: TODO
- License: TODO
- Status: no code copied into this repository yet; read-only use planned
  via a small `gripper.py` reader.

## Meta workshop repo

- Ideas only, no code copied.
