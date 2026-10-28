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

## 2026-10-27

- With model Muse Code powered by Meta Muse Spark, documentation only:
  created docs/PLAN.md, extended AGENTS.md and docs/THIRD_PARTY.md, added
  DEVLOG entry. No code changed, no hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, built safety.py
  (292 lines, pure checks, no motion) with tests/test_safety.py
  (33 tests, all green, full suite green) and filled in the gripper
  entry in docs/THIRD_PARTY.md. No hardware touched, port never opened,
  nothing committed.
- With model Muse Code powered by Meta Muse Spark, strict targets in safety.py: every Target now needs full x/y/z plus all six joints ("incomplete-target" otherwise) with joint 6 always checked against J6_REF; tests updated. No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, documentation only: shortened docs/PLAN.md (hackathon context, connector goal, setup, daily schedule), updated AGENTS.md (stack, layout, ground rules), updated docs/THIRD_PARTY.md (Seeed sources line, gripper purpose, workshop link) and docs/SETUP.md (test list, SDK checkout note); verified with uv run pytest. No code changed, no hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, made tests/test_safety.py SDK-independent (fake limits fixture, SDK test skips when missing), added README Contributions section, fixed AGENTS.md layout and stack wording; safety.py unchanged. No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, review only: checked every .py file, test, AGENTS.md, README.md, pyproject.toml and docs/ for bugs, safety gaps, test/doc contradictions and public-repo risks without changing code; reported findings in chat and ran uv run pytest (81 passed). No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, fixed review findings in safety.py (finite current position, positive speed, derived work limits, gripper verification note), preflight.py (full argv plus -m check, fuser-to-lsof fallback), arm.py and diagnose_joints.py (disconnect sag warning, per-group empty notice) with tests for every fix; verified with uv run pytest. No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, fixed docs (DEVLOG, SETUP, AGENTS, README, THIRD_PARTY) and safety.py right-side limit (WORK_Y_MIN_M, both-side tests); uv run pytest 98 passed. No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, added usb_devices.py (USB-ID port lookup, 82 lines) and reworked preflight.py (arm-usb check, no fixed port, ps-None FAIL, 288 lines) with tests/test_usb_devices.py plus extended tests/test_preflight.py; uv run pytest 116 passed. No hardware touched, port never opened, nothing committed.
- Human ran preflight.py on the real PC (started by the human, arm power off, USB only): arm-usb OK with the cable plugged in, arm-usb FAIL with port checks skipped when unplugged.
- With model Muse Code powered by Meta Muse Spark, added sdk_config.py (read-only SDK channel lookup, 88 lines) and a preflight.py sdk-port check (SDK channel vs USB arm port, skipped without arm port, 291 lines) with tests/test_sdk_config.py plus new sdk-port cases; uv run pytest 128 passed. No hardware touched, port never opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, moved all modules into the muse_qc/ package, deleted main.py, and updated imports, tests and docs paths without changing logic; uv run pytest 128 passed. No hardware touched, port never opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, added camera_top.py (77 lines, USB-ID table camera lookup, one OpenCV frame rotated 180 degrees for the upside-down mount, PNG saved to recordings/) plus find_video_devices in usb_devices.py (133 lines) with tests/test_camera_top.py and video tests; added opencv-python-headless 5.0.0.93 (Apache 2.0); uv run pytest 143 passed. No camera or serial port opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, set the table camera to full resolution in camera_top.py (V4L2 backend, MJPG FOURCC, 1920x1080, 5 warm-up frames, size check with no silent fallback, rotation kept) and extended tests/test_camera_top.py (fake capture: set order, warm-up discard, wrong size, release); uv run pytest 147 passed. No camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, raised the table camera warm-up frames from 5 to 45 (about 1.5 s at 30 fps) so auto exposure can settle; tests use the constant unchanged. No camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, added table_markers.py and make_markers.py (printable A4 SVG with four 40 mm markers plus 100 mm scale line) with offline tests and a SETUP section. No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, fixed the table camera settings in code (manual exposure 450, gain 0, brightness 160, contrast 150, manual white balance at 3200 K, read back and printed) with extended offline tests and a SETUP section update. No hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, small follow-ups: moved the seven DEVLOG camera entries into time order and extended the THIRD_PARTY opencv purpose with ArUco marker generation and detection, then moved the printable marker sheet into the repository (docs/table_markers.svg with a byte-exact committed-file test) and fixed the stale warm-up wording. The generator was not run by the agent, nothing committed.
- With model Muse Code powered by Meta Muse Spark, corrected the untested assumption about the camera losing its settings (it keeps them, per the human 10-second unplug test) in the camera_top docstring, docs/SETUP.md section 9 and one docs/DEVLOG.md entry, and logged the human make_markers and unplug test runs. Nothing committed.
- With model Muse Code powered by Meta Muse Spark, added detect_markers.py (185 lines: ArUco detection of IDs 0-3, missing/duplicated errors, ignored_ids, corner names, px/mm scale, hand-run main with annotated PNG) with offline tests/test_detect_markers.py (8 tests, all green) plus docs/SETUP.md section 11 and the AGENTS.md layout entry. No camera opened, nothing committed.

## 2026-10-28

- With model Muse Code powered by Meta Muse Spark, added table_coords.py (179 lines: pixel-to-mm homography against TABLE_MARKER_LAYOUT_MM, forward/back mapping, marker sides in mm, labelled 100 mm grid, hand-run main) with offline tests/test_table_coords.py (7 tests, all green) plus the layout constant in table_markers.py, docs/SETUP.md section 12, the AGENTS.md layout entry and DEVLOG entries. No camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, documentation only: replaced the guessed section 12 example in docs/SETUP.md with the real human run output and added the known-limitation paragraph (uncorrected lens distortion), and logged the human table_coords and cube test runs in docs/DEVLOG.md. No code changed, no hardware touched, nothing committed.
- With model Muse Code powered by Meta Muse Spark, added checkerboard.py (shared calibration constants plus hand-run A4 landscape SVG generator with 100 mm scale line) with offline tests/test_checkerboard.py plus docs/SETUP.md section 13, the AGENTS.md layout entry and DEVLOG entries. The generator was not run by the agent, no camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, follow-up on checkerboard.py: removed the black board outline (plain white margin for detection on real photos), moved the instruction text into the top margin (baseline 11 mm, left aligned with the board), kept the 100 mm scale line below the board, and added a black-only-squares test. The generator was not run by the agent, no camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, fixed review findings: arm.py wraps unreadable position values as JointStateUnavailable with a new test, safety.py J6_REF comment corrected and dead joint-step filter removed, docs/SETUP.md sections 6, 9, 10, 11 and Safety notes fixed, docs/PLAN.md schedule and README wording updated, missing 2026-10-26 arm-check entries backfilled in docs/DEVLOG.md from this log. No hardware touched, no camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, added capture_calibration.py (102 lines, hand-run Enter/q photo loop via open_table_camera with findChessboardCornersSB check, saves to recordings/calibration/) and calibrate_camera.py (190 lines, 10-board minimum, equal-size check, RMS/distortion printout, calibration/table_camera.json plus load_calibration/undistort) with offline tests/test_calibration.py (8 tests) plus docs/SETUP.md section 14, the AGENTS.md layout entries and DEVLOG entries; uv run pytest 196 passed. No camera opened, detect_markers.py and table_coords.py unchanged, nothing committed.
- With model Muse Code powered by Meta Muse Spark, used the calibration in the table mapping: undistort_points/distort_points in calibrate_camera.py (241 lines), optional calibration in table_coords.py (242 lines: main prints the calibration or none line, point-wise undistort, grid on the raw frame), detect_markers.py unchanged; extended tests/test_table_coords.py and tests/test_calibration.py with a synthetic distorted frame; uv run pytest 203 passed. No camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, follow-up: removed the unmeasured 40 mm sentence from docs/SETUP.md section 12, reworded the expected output line in section 14, undistort_points now uses its own fixed-point iteration (50 iterations, eps 1e-9) instead of cv2.undistortPoints, added a strong-distortion corner test; uv run pytest: 204 passed. No camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, updated the table marker layout to the fitted quadrilateral (ID1 (982.0, 0.0), ID2 (6.9, 530.8), ID3 (983.6, 529.9)) in table_markers.py, renamed RECTANGLE_ORDER to CORNER_ORDER in table_coords.py, replaced the diagonal test with a six-distance check in tests/test_table_coords.py, updated docs/SETUP.md section 12 and logged the human table_coords, measurement and cube runs in docs/DEVLOG.md. No camera opened, nothing committed.
- With model Muse Code powered by Meta Muse Spark, follow-up: replaced the docs/SETUP.md section 12 example with the real calibrated human run (recorded output is never edited), moved the old uncalibrated numbers into the history text, added the human cube re-run to docs/DEVLOG.md; uv run pytest 204 passed. No camera opened, nothing committed.
