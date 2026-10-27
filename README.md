# muse-qc-cell

Built during the Meta Global AI Developer Hackathon, Oct 26 to Nov 4, 2026. Built with Muse Code, runs on Meta models.

## Requirements

- Python 3.11
- uv
- The official Seeed SDK (reBotArm_control_py) checked out next to this repository as `../seeed-sdk`, see docs/SETUP.md. Without it, `uv sync` fails.

## Contributions

The Touch-Sensing Fin-Ray Gripper was built by the author before the hackathon and is listed by Seeed under "DIY Soft Finger". Gripper hardware and firmware are not part of this submission. New in this hackathon: muse_qc/gripper.py (planned), which will read the sensors over USB, and everything else in this repository. See docs/THIRD_PARTY.md for details.
