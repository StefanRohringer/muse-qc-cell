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
- With model muse-spark-1.3, effort medium, reviewed the SDK read-only (no
  thread or motion from constructor/connect, loop only via
  start_control_loop, disconnect calls disable_all) and added the
  "Safety notes" section to docs/SETUP.md. No hardware touched,
  nothing committed.
- Built preflight.py (read-only start checks: competing processes, device
  node, port access incl. dialout, port holder via fuser/lsof, exit 0 only
  when all pass) with tests/test_preflight.py (27 tests, all green, full
  suite 37 green). Sandbox has no /dev/ttyACM0 and no USB view, so the
  script correctly reports FAIL here. No hardware touched, nothing
  committed.
- With model Muse Spark (Muse Code), fixed check 1 in preflight.py to match
  only the basename of argv[0] at a word start or after -, _, . or / (so
  rosmaster/rebot_driver/motorbridge hit, gnome-shell/chrome/process miss,
  own PID excluded) and extended tests/test_preflight.py; full suite 42
  green. No hardware touched, port never opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, added diagnose_joints.py:
  read-only joint-feedback probe using the SDK directly (connect, four raw
  get_positions prints for arm+gripper at 0.5 s spacing, disconnect; no
  enable/zero/mode/motion calls), verified with a fake SDK only. No hardware
  touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, fixed the first-read 0.0
  issue found on hardware: read_joints/read_gripper retry get_positions up to
  5 times at 0.2 s while any value is exactly 0.0 (configurable via
  max_read_attempts/read_retry_delay_s), raising JointStateUnavailable only
  when 0.0 persists. Added 4 tests (0.0-then-real passes, persistent 0.0
  still raises after 5 attempts); full suite 46 green. No hardware touched,
  nothing committed.
- Human ran the first hardware check (started by the human, as required):
  preflight.py overall OK, then Arm.read_joints/read_gripper returned real
  angles for all 6 joints plus gripper with no JointStateUnavailable, so the
  retry fix works on the real arm. Nothing committed.
