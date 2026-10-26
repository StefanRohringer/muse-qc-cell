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

## reBotArm_control_py (documented, not installed)

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
- Status: declared here but NOT installed. The SDK is not published on
  PyPI, and installing it directly from the Git URL fails because of an
  upstream setuptools packaging bug (`Multiple top-level packages
  discovered in a flat-layout`), which is also described in Seeed's own
  Wiki with a manual checkout workaround. Cloning the SDK into this
  repository would violate the project rule against copying foreign code,
  so on the hardware machine follow the official Seeed Wiki checkout
  procedure instead.
