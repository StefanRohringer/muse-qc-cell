# Dev Log

## 2026-10-26

- Set up accounts, environment, repository, and Muse Code.

## 2026-10-27

- Documentation only: created docs/PLAN.md (goal, setup, schedule), extended AGENTS.md
  with agent-loop, evidence, offline-test and no-legacy rules, and
  declared the gripper project plus workshop repo in
  docs/THIRD_PARTY.md. No code changed, nothing committed.
- Built safety.py (pure pre-move checks: workspace box with 5 cm
  margin, tool z, SDK URDF joint limits with refuse-all fallback,
  joint 6 cable limit around user-set J6_REF, speed cap, step limits,
  200 ms state freshness, latched STOP, 4-sensor gripper rules) with
  tests/test_safety.py (33 tests, all green) and filled in the gripper
  entry in docs/THIRD_PARTY.md. No hardware touched, nothing committed.
- Fixed safety.py strict targets: every target needs full x/y/z position
  and all six joints ("incomplete-target" otherwise), joint 6 always
  checked against J6_REF; tests updated. No hardware touched, nothing committed.
- Documentation only: shortened docs/PLAN.md, updated AGENTS.md stack and
  layout plus ground rules, updated docs/THIRD_PARTY.md and docs/SETUP.md.
  No code changed, nothing committed.
- Made tests/test_safety.py SDK-independent (fake joint limits via fixture,
  SDK test skips without reBotArm_control_py), added README Contributions
  section, fixed AGENTS.md layout (inspection.py, tools/) and stack wording.
  safety.py unchanged. Nothing committed.
- Fixed review findings: safety.py refuses non-finite current x/y/z
  ("invalid-current-position"), requires 0 < speed, derives WORK_X_MAX_M
  and WORK_Y_MAX_M from TABLE_* minus EDGE_MARGIN_M (values unchanged)
  and notes the holding-lowering rule still needs real-gripper
  verification; preflight.py checks every command-line element plus the
  -m module, drops the unused constant and falls back from a failed fuser
  probe to lsof; arm.py and diagnose_joints.py warn about motor
  disable/sag on disconnect; tests added for every fix, full suite green.
  No hardware touched, nothing committed.
- Fixed docs (PLAN wording, full test list with SDK-independent note in
  SETUP, planned markers and line rule in AGENTS, planned gripper plus
  Requirements in README and THIRD_PARTY) and safety.py right-side limit
  (WORK_Y_MIN_M with both-side tests); full suite green (98 passed).
  No hardware touched, nothing committed.
- Added usb_devices.py (read-only arm port lookup by USB ID 2e88:4603
  via sysfs, no port ever opened) and switched preflight.py to a new
  "arm-usb" check with no fixed port (port checks skip when the lookup
  fails; failed ps probe is FAIL, never a crash); preflight.py is back
  under 300 lines. Added tests/test_usb_devices.py (fake sysfs) and
  extended tests/test_preflight.py; full suite green (116 passed).
  Documented the USB-ID check in docs/SETUP.md. No hardware touched,
  no serial port opened, nothing committed.
- Human ran preflight.py on the real PC (started by the human, arm power
  off, USB only): arm-usb OK with the cable plugged in, arm-usb FAIL
  with port checks skipped when unplugged.
- Added sdk_config.py (read-only SDK channel lookup: finds
  reBotArm_control_py via find_spec without importing it, follows
  config/rebotarm.yaml via hardware_yaml, returns the channel) and a new
  preflight.py "sdk-port" check (OK only when SDK channel and USB arm
  port are the same device; skipped without an arm port); preflight.py
  stays under 300 lines. Added tests/test_sdk_config.py and new
  sdk-port cases in tests/test_preflight.py; full suite green
  (128 passed). No hardware touched, no serial port opened, nothing
  committed.
- Human ran preflight.py on the real PC again (started by the human, arm
  power off, USB only): sdk-port OK (SDK channel and USB arm port are the
  same device), overall OK.
- Moved safety.py, arm.py, usb_devices.py, sdk_config.py, preflight.py
  and diagnose_joints.py into the muse_qc/ package (new one-line
  __init__.py), deleted the main.py placeholder, and updated all imports,
  source-path tests, module runs (python -m muse_qc.preflight) and docs
  paths. No logic changed; full suite green (128 passed).
- Human ran python -m muse_qc.preflight on the real PC after the package
  move (started by the human, arm power off, USB only): all checks OK.
- Added camera_top.py (finds the table camera by USB ID 145f:0203 via a
  new find_video_devices in usb_devices.py, capture nodes only, reads
  one frame with OpenCV, rotates it 180 degrees because the camera is
  mounted upside down, saves recordings/table_<timestamp>.png; never
  imports arm code, never moves anything) with tests/test_camera_top.py
  (fake capture only) and video lookup tests; added
  opencv-python-headless 5.0.0.93; documented the hand-only camera test
  in docs/SETUP.md. Full suite green (143 passed). No camera or serial
  port opened by the agent, nothing committed.
- Human ran python -m muse_qc.camera_top on the real PC (started by the human, arm power off): table camera found by USB ID, 640x480 image saved, upright after the 180 degree rotation, whole table visible.
- Set the table camera to full resolution: camera_top.py now opens the
  device with the V4L2 backend, sets MJPG FOURCC plus 1920x1080,
  discards 5 warm-up frames and raises "unexpected frame size WxH" when
  the real frame differs (no silent fallback); rotation kept.
  Extended tests/test_camera_top.py (fake capture: set order before
  first read, warm-up discard, wrong size raises, release in every
  case). Full suite green (147 passed). No camera opened by the agent,
  nothing committed.
- Human ran python -m muse_qc.camera_top again after the resolution change (started by the human, arm power off): 1920x1080 image saved, upright, whole table and the four corner markers visible, but the image was too dark with 5 warm-up frames.
- Raised the warm-up frames from 5 to 45 (about 1.5 s at 30 fps) so auto
  exposure can settle; set in camera_top.py (TABLE_CAM_WARMUP_FRAMES),
  tests use the constant unchanged, docs/SETUP.md section 9 updated.
  Full suite green (147 passed). No camera opened by the agent, nothing
  committed.
- Human ran python -m muse_qc.camera_top again with 45 warm-up frames (arm power off, USB only): 1920x1080 image saved, still too dark (mean grey value about 79 of 255).
- Human ran v4l2-ctl --list-ctrls on the table camera: manual exposure mode, exposure_time_absolute 450, gain 0, brightness 120, contrast 120, automatic white balance off at 3200 K.
- Human ran camera_top with exposure_time_absolute 900: no visible change (mean about 80).
- Human ran camera_top with exposure 900 and gain 64: no visible change (mean about 78). At 1920x1080 MJPG exposure and gain have no visible effect.
- Human ran camera_top with automatic exposure (auto_exposure 3): brighter (mean about 100).
- Human ran camera_top in manual mode with exposure 450, gain 0, brightness 180: bright (mean about 140), but the black of the markers is lifted.
- Human ran camera_top with brightness 160 and contrast 150: mean about 118, markers clearly black and white, lamp spot on the left about 1 percent overexposed. These values were chosen.
- Added table_markers.py (shared ArUco constants DICT_4X4_50, IDs 0-3,
  40 mm size, plus marker_cells built with generateImageMarker at
  6 pixels) and make_markers.py (hand-run A4 SVG generator:
  print/table_markers.svg with four 40 mm markers, ID labels and a
  100 mm scale line; print/ is git-ignored) with
  tests/test_table_markers.py (6x6 cells, black outer ring, distinct
  patterns, detect round trip) and tests/test_make_markers.py
  (valid A4 XML, 40 mm markers, 100 mm line); new docs/SETUP.md section
  10 (print at 100 %, check with a ruler, one marker per table corner)
  and AGENTS.md layout entries. No hardware touched, nothing committed.
- Fixed the table camera settings in code: camera_top.py now applies
  manual exposure 450, gain 0, brightness 160, contrast 150 and manual
  white balance at 3200 K on every start (set after format and size,
  before the warm-up frames, read back and printed); extended
  tests/test_camera_top.py (fake capture with get, set order, fixed
  values, read-back, printed line) and documented the fixed values plus
  the v4l2-ctl check in docs/SETUP.md section 9. No camera opened by
  the agent, nothing committed.
- Moved the printable marker sheet into the repository: make_markers.py
  now writes docs/table_markers.svg (print/ removed from .gitignore)
  with a new test that the committed file is byte for byte equal to
  build_svg(); fixed the stale warm-up wording (the frames let the
  camera settle after the settings are applied, exposure is manual
  now). The generator was not run by the agent, nothing committed.
- Human ran python -m muse_qc.make_markers (no hardware): docs/table_markers.svg written; four 40 mm markers (IDs 0 to 3) and a 100 mm scale line; full suite green (164 passed).
- Human ran camera_top after unplugging the table camera for 10 seconds (arm power off): before the run v4l2-ctl already showed brightness 160, contrast 150 and manual exposure, so the camera keeps its settings when unplugged. camera_top printed and v4l2-ctl confirmed auto_exposure 1, exposure 450, gain 0, brightness 160, contrast 150, automatic white balance off at 3200 K; 1920x1080 image saved, mean grey value about 117.
- Fixed the untested assumption that the camera loses its settings when
  unplugged: it keeps them (shown by the 10-second unplug test), so the
  v4l2-ctl entry lost its last sentence and the camera_top docstring
  plus docs/SETUP.md section 9 now say the fixed values are applied on
  every start to make the result reproducible. Nothing committed.
- Added muse_qc/detect_markers.py (hand-run table marker detection:
  one frame via open_table_camera, ArUcoDetector over the shared
  dictionary, each ID 0-3 required exactly once with missing/duplicated
  errors, unknown IDs under ignored_ids, image corner names from the
  centers, px/mm scale from the 40 mm size, annotated PNG saved to
  recordings/) with tests/test_detect_markers.py (offline synthetic
  1920x1080 frame: accuracy within 2 px, ignored ID, corner names,
  scale, missing/duplicated errors, faked main writing the PNG, no
  arm/safety/SDK import) plus docs/SETUP.md section 11 and the AGENTS.md
  layout entry. No camera opened by the agent, nothing committed.
- Human ran python -m muse_qc.detect_markers on the real table (arm power off): all four markers found. ID 2 top-left, ID 3 top-right, ID 0 bottom-left, ID 1 bottom-right (the arm stands at the bottom edge of the image); marker sides 58.1 to 59.3 px; scale 1.465 px/mm; annotated image saved and checked by eye, every marker outlined correctly.
