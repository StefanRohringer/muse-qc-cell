# Muse Code Log

## 2026-10-26

- First test with model muse-spark-1.3, effort medium.
- Set up reBot B601-DM software sources: installed motorbridge 0.5.6 via uv,
  documented reBotArm_control_py (0.1.0, commit d4f3591) with source, version
  and license in docs/THIRD_PARTY.md. No hardware touched, nothing committed.
- Switched the project from Python 3.12 to 3.11 (SDK needs <3.12), cloned the
  Seeed SDK to ~/meta-hackathon/seeed-sdk, fixed its flat-layout packaging bug,
  set hardware_yaml to rebotarm_dm.yaml, installed it editable via uv (sync-safe),
  verified the import only, and wrote docs/SETUP.md. No hardware touched,
  nothing committed.
- Built module 1 (arm.py): read-only RebotArm wrapper exposing only
  connect, joint/gripper reads (degrees and radians with names) and
  disconnect; exact 0.0 is rejected as missing feedback. Added
  tests/test_arm.py (11 tests with a RebotArm fake, all green) and a
  manual hardware-check section in docs/SETUP.md. No hardware touched,
  nothing committed.
