# Third-Party Components

Official Seeed sources are used for the reBot Arm B601-DM. No third-party code is copied into this repository; everything is referenced as a package dependency or declared below.

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

## opencv-python-headless (installed)

- Purpose: table camera capture and image saving for `muse_qc/camera_top.py`
  (one frame via VideoCapture, 180-degree rotation, PNG output).
  ArUco marker generation and detection (cv2.aruco).
- Source (PyPI): `opencv-python-headless>=5.0.0.93` (resolved to 5.0.0.93).
- License: Apache 2.0 (PyPI package metadata).
- Installed with: `uv add opencv-python-headless`.

## openai (installed)

- Purpose: client for the Meta Model API in `muse_qc/model_check.py`
  (hand-run connection check: one `GET /v1/models` call against the
  base URL `https://api.meta.ai/v1`).
- Source (PyPI): `openai>=3.40.0` (resolved to 3.40.0).
- License: Apache-2.0 (PyPI package metadata).
- Installed with: `uv add openai`.

## Author's public gripper project (declared, not vendored)

- Purpose: touch-sensing gripper fingers reporting grip and contact (4 sensors).
- Link: https://github.com/StefanRohringer/Touch-Sensing-Fin-Ray-Gripper
- License: software GPL-3.0-or-later, hardware CERN-OHL-S-2.0.
- Status: no code is copied into this MIT repository. `muse_qc/gripper.py` (planned)
  will be newly written and only uses the gripper's documented USB
  serial protocol.
- The Touch-Sensing Fin-Ray Gripper is an additional open hardware component for the reBot arm, built by the author before the hackathon. It is listed in the official Seeed reBot-DevArm repository under "DIY Soft Finger" (https://github.com/Seeed-Projects/reBot-DevArm). Gripper hardware and firmware are not part of this hackathon submission. New in this hackathon: muse_qc/gripper.py (planned), which will read the sensors over USB, and everything else in this repository.

## Meta workshop repo

- Link: https://github.com/meta-models/legal-agent-workshop (ideas only, no code copied).
