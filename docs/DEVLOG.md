# Dev Log

## 2026-10-26

- Set up accounts, environment, repository, and Muse Code.
- Human ran the first read-only arm check (started by the human, arm power supply on, USB connected, no motion command): the first reading returned 0.0 for joints without feedback yet.
- Human ran the read-only arm check again after the retry fix (arm power supply on, no motion command): preflight overall OK, read_joints and read_gripper returned real angles for all six joints and the gripper.
- (The two entries above were added on 2026-10-28 from docs/MUSE_CODE_LOG.md; the DEVLOG rule was introduced on 2026-10-27.)

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

## 2026-10-28

- Human measured the marker distances with a tape measure, centre to centre: ID 2 to ID 3 977 mm, ID 0 to ID 2 531 mm, diagonal ID 2 to ID 1 1110 mm (rectangle check 1112 mm). ID 0 to ID 1 not measurable because the arm is in the way.
- Added muse_qc/table_coords.py (179 lines: homography from the four
  marker centres to TABLE_MARKER_LAYOUT_MM, pixel to mm and back,
  marker side lengths in mm as an independent check, 100 mm grid
  drawing with labelled origin and axes, hand-run main with one frame
  and a grid PNG in recordings/) with tests/test_table_coords.py
  (offline synthetic frame at a known scale and offset: centres map to
  the layout within 1 mm, round trip within 0.5 px, sides about
  40 mm, layout diagonal within 5 mm of 1110 mm, faked main writing
  the PNG, no arm/safety/SDK import) plus TABLE_MARKER_LAYOUT_MM in
  table_markers.py, docs/SETUP.md section 12 and the AGENTS.md layout
  entry. Full suite green. No camera opened by the agent, nothing
  committed.
- Human ran python -m muse_qc.table_coords on the real table (arm power off): marker centres mapped exactly to the measured layout; marker sides from the corners 37.0 to 37.8 mm instead of 40 mm (about 7 percent small, most likely lens distortion towards the image corners); grid image checked by eye, grid lines straight and parallel to the table edges.
- Human placed a 25 mm white cube with its centre at x = 300 mm, y = 300 mm (tape measure from the centre of marker 0) and ran table_coords again: in the grid image the cube centre is about 19 mm too far in -x and on the line in y; after allowing about 5 mm for the cube height the table error is about 15 mm in x. Recorded as a known limitation; lens distortion is not corrected yet.
- Documentation only: replaced the guessed example output in
  docs/SETUP.md section 12 with the real human run output and added
  the known-limitation paragraph (lens distortion not corrected, 37 mm
  sides, 15 mm cube offset in x). No code changed, nothing committed.
- Added muse_qc/checkerboard.py (shared calibration constants: 10 by 7
  squares of 25 mm, 9 by 6 inner corners; hand-run A4 landscape SVG
  generator docs/checkerboard.svg with a 100 mm scale line) with
  tests/test_checkerboard.py (offline A4, square and margin checks
  plus a cv2.findChessboardCorners round trip) plus docs/SETUP.md
  section 13 and the AGENTS.md layout entry. The generator was not run
  by the agent, nothing committed.
- Removed the black outline rect around the checkerboard (plain white
  margin for findChessboardCorners on real photos) and moved the
  instruction text into the top margin, left aligned with the board;
  the 100 mm scale line stays below the board. Extended
  tests/test_checkerboard.py with a black-only-squares check. The
  generator was not run by the agent, nothing committed.
- Human ran python -m muse_qc.checkerboard (no hardware): docs/checkerboard.svg written, 35 black squares of 25 mm, 100 mm scale line, instruction text in the top margin; full suite green (187 passed).
- Fixed review findings: arm.py wraps unreadable position values as JointStateUnavailable (new test), safety.py J6_REF comment corrected and dead joint-step filter removed, docs/SETUP.md sections 6, 9, 10, 11 and Safety notes fixed, docs/PLAN.md schedule and README wording updated, missing 2026-10-26 arm-check entries backfilled from docs/MUSE_CODE_LOG.md. No hardware touched, nothing committed.
- Added muse_qc/capture_calibration.py (hand-run photo loop: Enter takes a photo via open_table_camera, q quits, board check with findChessboardCornersSB, only found boards saved to recordings/calibration/, 102 lines) and muse_qc/calibrate_camera.py (hand-run offline calibration: 10-board minimum, equal-size check, RMS and distortion printout, calibration/table_camera.json plus load_calibration and undistort for later use, 190 lines) with tests/test_calibration.py (8 offline tests: synthetic projectPoints recovery of focal length within 2 percent with k1 sign, under-10 and mixed-size refusal, JSON round trip with undistort, capture loop with faked camera, no arm/safety/SDK import; full suite 196 green) plus docs/SETUP.md section 14 (photo plan: about 8 flat photos over centre, corners and edges plus about 8 tilted 20 to 40 degrees, board fully visible, arm power off) and the AGENTS.md layout entries. No camera opened by the agent, detect_markers.py and table_coords.py unchanged, nothing committed.
- Human printed docs/checkerboard.svg at 100 percent and checked it with a ruler: scale line 100 mm, one square 25 mm; the sheet was glued flat on cardboard.
- Human ran python -m muse_qc.capture_calibration on the real table (arm power off): 26 checkerboard photos saved, board found in every saved photo, flat and tilted poses.
- Human ran python -m muse_qc.calibrate_camera: 26 images used, RMS reprojection error 0.199 px, fx about 1518 px, principal point about (959, 542), mild barrel distortion (about 1.4 percent at the markers, about 4 percent in the image corners); calibration/table_camera.json written.
- Used the calibration in the table mapping: undistort_points (cv2.undistortPoints with P=camera_matrix) and distort_points (cv2.projectPoints way back) added to muse_qc/calibrate_camera.py, optional calibration argument threaded through table_homography, pixel_to_table_mm, table_mm_to_pixel, marker_sides_mm and draw_grid in muse_qc/table_coords.py (main prints the calibration or none line, grid still drawn on the raw frame, whole-image undistort never used), detect_markers.py unchanged. Offline tests with a synthetic distorted marker frame (centres within 1 mm with calibration, mid-table point closer with calibration, raw-mm-raw round trip within 0.5 px, missing-file fallback); full suite 203 green. No camera opened by the agent, nothing committed.
- Follow-up: removed the unmeasured 40 mm sentence from docs/SETUP.md section 12, reworded the expected output line in section 14, undistort_points now uses its own fixed-point iteration (50 iterations, eps 1e-9) instead of cv2.undistortPoints, added a strong-distortion corner test; uv run pytest: 204 passed. No camera opened by the agent, nothing committed.
- Human ran python -m muse_qc.table_coords with calibration/table_camera.json present (arm power off): first line "calibration: calibration/table_camera.json (rms 0.199 px)", marker sides 39.1 to 40.1 mm (before the calibration about 37 to 38 mm).
- Human measured the remaining table marker distances centre to centre: ID0-ID1 982 mm, ID1-ID3 530 mm, diagonals ID0-ID3 1117 mm and ID1-ID2 1110 mm (together with ID2-ID3 977 mm and ID0-ID2 531 mm measured on 2026-10-27).
- Human repeated the cube test with the calibration: 25 mm cube placed by tape measure at x = 300 mm, y = 300 mm; read from the grid image with the old rectangle layout, the cube centre sat at about x = 291 mm, y = 311 mm (top face corrected for the 25 mm cube height).
- Updated the table marker layout to the fitted quadrilateral (origin at marker 0, +x towards marker 1: ID1 (982.0, 0.0), ID2 (6.9, 530.8), ID3 (983.6, 529.9); least-squares fit to all six hand distances, residuals under 0.5 mm): TABLE_MARKER_LAYOUT_MM in muse_qc/table_markers.py with the six distances in the comment, rectangle wording and RECTANGLE_ORDER renamed to CORNER_ORDER in muse_qc/table_coords.py, canvas sizes derived from the layout and a six-distance check (0.5 mm) in tests/test_table_coords.py, docs/SETUP.md section 12 updated (fit, quadrilateral, new cube reading). Left alone: the historical DEVLOG entries with the old distances, the recorded pixel columns of the SETUP example, and safety.py whose TABLE_* constants describe the arm-base workspace box, not the markers. No camera opened by the agent, nothing committed.
- Human ran python -m muse_qc.table_coords again with the calibration and the measured layout (arm power off, cube unchanged): marker sides 39.3 to 39.9 mm; the cube centre read from the grid image sat at about x = 297 mm, y = 312 mm (top face corrected for the 25 mm cube height), so 3 mm off in x and 12 mm in y; the remaining y offset is probably the tape-measure placement and will be checked with the arm.
- Replaced the docs/SETUP.md section 12 example with that real calibrated run and moved the old uncalibrated numbers (sides about 37 mm, cube about 15 mm off in -x) into the history text; recorded program output is never edited. No camera opened by the agent, nothing committed.
- Added muse_qc/model_check.py (hand-run Meta Model API check: MODEL_API_KEY from .env or the environment, never printed; one GET /v1/models via the openai SDK against https://api.meta.ai/v1; prints the model count plus OK/MISSING for muse-spark-1.3 and sam-3.1; exit 0 only when both are listed; "key rejected" on 401/403, short reason on network errors, no stack trace) with tests/test_model_check.py (16 offline tests with a faked client: both found, one missing, missing key, key never in stdout/stderr, no arm/safety/SDK/camera import) plus docs/SETUP.md section 15 and the openai entry in docs/THIRD_PARTY.md; added openai 3.40.0 (Apache-2.0). Full suite green (220 passed). The check was not run by the agent, no network call was made, .env was never opened, nothing committed.
- Human ran python -m muse_qc.model_check with the old MODEL_API_KEY in .env: "key rejected", exit 1 (curl showed HTTP 401 invalid_api_key); the key was invalid, the base URL was confirmed correct by the official docs.
- Human created a new Model API key, put it into .env by hand (never committed) and ran python -m muse_qc.model_check again: models: 9, muse-spark-1.3: OK, sam-3.1: OK, exit 0.
- Added muse_qc/vision_sam.py (265 lines, hand-run SAM 3.1 box detector: one frame via open_table_camera or --image, key reading reused from model_check.py, one POST /v1/responses with model sam-3.1, JPEG base64 data URL and metadata include_confidence "true"; own output_text parser for id<|box;...|> records by key with sparse ids, mask segments ignored, empty means zero matches; one printed line per box plus count, annotated copy at recordings/sam_<timestamp>.png; 400 "prompt rejected", 401/403 "key rejected", short network reason, no stack trace, never the key) with tests/test_vision_sam.py (17 offline tests: parser samples, payload shape, error mapping, key never in output, no arm/safety/SDK import) plus docs/SETUP.md section 16. Full suite green (237 passed). No camera opened by the agent, no API call, .env never opened, nothing committed.
- Human ran python -m muse_qc.vision_sam on the table camera (arm power off) with two connectors and the white 25 mm cube on the table: "grey connector" gave boxes: 0; "white cube" gave one box (638, 417)-(686, 461), centre (662.0, 439.0) px, confidence 0.78, within 4 px of the hand reading from the grid image.
- Human ran python -m muse_qc.vision_sam with --image on the saved frame of that scene: "connector", "electrical connector", "wire connector", "terminal block" and "orange lever" gave boxes: 0; "small part" found both connectors at centres (1188, 208) and (1156, 396) px, confidence 0.62 and 0.58, the cube not included.
- Human ran python -m muse_qc.vision_sam "small part" on the live table camera after the table_mm change (started by the human, arm power off): table mapping calibrated, 3 boxes, cube mapped to (294.8, 311.9) mm, which matches the earlier hand reading of about (296, 311) mm within about 1 to 2 mm; the two connectors mapped to (600.1, 337.8) and (620.0, 452.8) mm. Result OK.
- Human ran the read-only joint read from SETUP 7 at the rest position (started by the human, arm power supply on, no motion command): joint1 +0.75, joint2 +1.58, joint3 -1.45, joint4 +2.72, joint5 +1.30, joint6 -0.47 deg; the rest position is within 3 deg of the SDK home (all joints 0); gripper channel -167.13 deg at 5.3 cm opening. The gripper channel is the real reBot gripper motor (the touch-sensing fingers sit on its jaws).
- Human measured the gripper: 185 mm from the joint 6 mounting face to the finger tips.
- Human ran python -m muse_qc.fk_check at the rest position (started by the human, arm power supply on, no motion command): joint1 -3.33, joint2 +1.58, joint3 -1.45, joint4 +2.72, joint5 +0.91, joint6 -3.97 deg; end_link x=+256.6 y=-17.9 z=+198.6 mm roll=-3.97 pitch=-0.32 yaw=-4.24 deg; tip x=+256.6 y=-15.8 z=+228.6 mm (offset 30 mm along end_link z). The tip was placed 30 mm above end_link although the gripper points forward, so the tip direction was wrong. Result FAIL for the tip; end_link checked by hand in the next entry.
- Human measured the finger tips at that rest position (arm power off, base standing directly on the table): 186 mm above the table and 296 mm horizontally from the centre of the base turntable. With the corrected tool axis the tip should be at about 287 mm out and 199 mm high, so the hand measurement differs by about +9 mm out and -13 mm in height. First FK check within about 1.5 cm; to be verified with arm motion.
- Human ran python -m muse_qc.fk_check again after the tool-axis fix at the same rest position (started by the human, arm power supply on, no motion command): same joint values, end_link unchanged, tip x=+286.5 y=-20.1 z=+198.8 mm, about 287 mm out and 199 mm high as predicted; the hand measurement (296 mm, 186 mm) differs by about +9 mm and -13 mm. Result OK for the tip direction.
- Human ran first_move without --execute (dry run), arm power supply on, no motion: rest reading j1 -3.33, j2 +1.58, j3 -1.45, j4 +2.72, j5 +0.93, j6 -3.97 deg; 240 waypoints all ok; home tip (290.3, 0.0, 191.7) mm; j1 +5 deg moves the tip y by about +25 mm. Result OK.
- Human ran first_move --execute, arm power supply on, vlim 10 %: 240/240 waypoints ok; joint 1 turned to the left, so +y = left is confirmed; the arm returned to home and held it; after the motors were disabled the arm dropped a little onto its rest support. Result OK. Follow-up: settle step before disable.
- Human ran first_move (dry run, then --execute) with the new settle segment, arm power supply on, vlim 10 %: rest reading [-2.26, +0.12, -0.08, +2.31, +0.86, -0.08] deg (the arm had stayed close to home after the previous run), 298/298 waypoints ok, settle target with joint2 clamped from +0.12 to 0.00 deg. The human reported that the motion ran well and the arm no longer dropped when the motors were disabled. Result OK.
- Human ran first_move with cycle timing (dry run, then --execute), arm power supply on, vlim 10 %, LOOP_RATE_HZ 50: dry run "cycle work (dry run): mean 1.27 ms, p95 1.44 ms, max 8.17 ms over 296 waypoints", all waypoints pass; --execute printed "cycle work (dry run): mean 1.23 ms, p95 1.30 ms, max 8.05 ms over 296 waypoints", then "cycle work: mean 2.12 ms, p95 2.35 ms, max 5.62 ms over 296 cycles" and "cycle period: mean 20.06 ms, p95 20.09 ms, max 20.19 ms (target 20.00 ms)"; "motion finished, arm holds the settle pose". Rest reading this time had joint2 at -0.01 deg, so no clamping was needed. Result OK. Decision: the worst real cycle (5.62 ms) fits into 10 ms, so LOOP_RATE_HZ was raised to 100 (step 0.05 deg, speed unchanged at 5 deg/s), to be verified with a new run.
- Human ran first_move at 100 Hz (dry run, then --execute), arm power supply on, vlim 10 %, LOOP_RATE_HZ 100: dry run alone gave 590 waypoints with "cycle work (dry run): mean 1.19 ms, p95 1.26 ms, max 7.70 ms over 590 waypoints" and "dry run: all waypoints pass"; --execute read rest joints deg ['-2.22', '-0.01', '-0.08', '+1.85', '+1.06', '-0.08'], planned "plan: 590 waypoints at 100 Hz" with all 590 ok, printed "cycle work (dry run): mean 1.20 ms, p95 1.25 ms, max 8.07 ms over 590 waypoints", was confirmed with yes, then printed "cycle work: mean 2.04 ms, p95 2.10 ms, max 2.21 ms over 590 cycles", "cycle period: mean 10.09 ms, p95 10.16 ms, max 10.20 ms (target 10.00 ms)", "motion finished, arm holds the settle pose" and "stopping: the motors will be disabled now". Result OK: 100 Hz verified, worst real cycle 2.21 ms of 10 ms, period within 0.2 ms of the target.
- Human ran v4l2-ctl on the arm camera (started by the human, arm power supply off): the Gemini 305 (USB 2bc5:0840) creates /dev/video0 to video9. video0 Z16 depth, video2 GREY IR, video4 BA81 Bayer, video6 and video8 YUYV 640x480/848x480/848x530, odd nodes metadata. The Trust camera is on video10/11. Result OK.
- Human ran the first OpenCV grab on video6 (started by the human, arm power supply off): no frame (select() timeout), stopped with Ctrl+C. USB speed read 5000 (USB 3). v4l2-ctl --stream-mmap on video6 and video8 got no frames (stopped by timeout after 10 s). Result FAIL.
- Human ran v4l2-ctl --stream-mmap after unplugging and replugging the camera (started by the human, arm power supply off): video6 received 10 frames, video8 still got none. Result OK for video6.
- Human installed the official Orbbec udev rule file on the host and tried pyorbbecsdk2 2.1.2 in a throwaway venv outside the repo (started by the human, arm power supply off): "devices: 1", then get_device_by_index failed with OBError "NULL pointer passed for argument deviceMgr". Not pursued, because V4L2 works. Not part of the project. Whether the V4L2 path needs the udev rule was not tested.
- Human ran OpenCV on video6 with YUYV 640x480 and 15 reads (started by the human, arm power supply off): the first try got no frame and the second gave (480, 640, 3). A rerun from the repo with retries gave "attempt 1 ok: True" and "written: True (480, 640, 3)". The human looked at the image and confirmed: the camera looks forward between the gripper fingers, the fingertips are visible at the bottom centre, and the picture is sharp and well exposed. The image was not committed. Result OK.
- Human mapped the sysfs nodes (started by the human, arm power supply off): video0 to video5 are on USB interface 1.0, video6/7 on 1.4, video8/9 on 1.6, and the colour node is index 0 on interface 1.4. This mapping is the basis for camera_arm.find_arm_color_node.
- Human ran camera_arm (arm power supply off): found /dev/video6 by USB ID plus interface 4, attempt 1 ok, frame (480, 640, 3), image saved to recordings/, exit 0. The human looked at the image: same view as the earlier test (forward between the gripper fingers, sharp). The image was not committed. Result OK.
- Human took three arm camera images (started by the human, arm power supply off, arm tilted down by hand, arm camera 640x480): the part held in the hand at about 30-40 cm, then the good part (3 levers) and the bad part (right lever torn out, 2 levers) lying on the table at the same spot. The test pair recordings/armcam_20261028_145156.png (good part) and recordings/armcam_20261028_145329.png (bad part) was kept for the counting tests. The images are not committed.
- Human ran count_clips --image on both images of the test pair with prompt "orange lever" (started by the human, arm power supply off, arm tilted down by hand): "levers: 0", "verdict: bad" for both, so the full-image phrase finds no lever on the small part.
- Human ran vision_sam --image on the good full image (started by the human, arm power supply off, arm tilted down by hand): "orange lever", "lever", "orange", "orange tab" gave boxes: 0. "small part" gave 9 boxes: id 7 box=(317, 187, 352, 233) confidence 0.76 is the part, and the 8 others (all with center y >= 415) are screws/connectors on the gripper at the bottom of the image, one of them also at confidence 0.76. "connector" found only a gripper connector (423, 402, 456, 429) 0.67.
- Human ran the crop test (started by the human, arm power supply off, arm tilted down by hand): manual crop of (317-15, 187-15) to (352+15, 233+15), enlarged 8x with INTER_CUBIC to 520x608. On the enlarged crop "orange lever", "lever", "orange tab", "orange stripe", "orange rectangle", "orange button" and "handle" gave 0 boxes on both images. "orange bar" gave 3 boxes on the good part (confidence 0.68, 0.70, 0.70) and 2 boxes on the bad part (0.76, 0.77). "orange switch" gave 2 on the good part and 1 on the bad part (around 0.5, not usable). Decision: count_clips finds the part with "small part" and counts with "orange bar".
- Human asked muse-spark-1.3 on both enlarged crops "How many orange levers does it have?" (started by the human, arm power supply off, arm tilted down by hand): good part "3 ... It has three orange levers.", bad part "3 ... It is a three-port lever connector." (wrong, the bad part has 2 levers). Decision: Muse Spark is not used for counting.
- Human ran count_clips (two-step flow) on both images of the test pair (started by the human, arm power supply off, arm tilted down by hand):

```text
=== 145156
part: box=(317, 187, 352, 233) confidence=0.76
id 1 box=(158, 206, 210, 457) center=(184.0, 331.5) confidence=0.70
id 2 box=(237, 206, 289, 457) center=(263.0, 331.5) confidence=0.70
id 0 box=(313, 213, 363, 464) center=(338.0, 338.5) confidence=0.68
levers: 3
verdict: good
=== 145329
part: box=(319, 186, 354, 232) confidence=0.76
id 2 box=(241, 201, 290, 457) center=(265.5, 329.0) confidence=0.77
id 0 box=(162, 205, 212, 458) center=(187.0, 331.5) confidence=0.75
id 1 box=(323, 206, 364, 296) center=(343.5, 251.0) confidence=0.59
levers: 3
verdict: good
```

Good part 3/good OK. Bad part 3/good FAIL because a short remnant box (323, 206, 364, 296), 90 px high at 0.59, was counted. Decision: MIN_BAR_HEIGHT_FRAC 0.3, to be verified with a new run.
- Human ran count_clips with MIN_BAR_HEIGHT_FRAC 0.3 on both images of the test pair (started by the human, arm power supply off, arm tilted down by hand):

```text
=== 145156
part: box=(317, 187, 352, 233) confidence=0.76
id 1 box=(158, 206, 210, 457) center=(184.0, 331.5) confidence=0.70
id 2 box=(237, 206, 289, 457) center=(263.0, 331.5) confidence=0.70
id 0 box=(313, 213, 363, 464) center=(338.0, 338.5) confidence=0.68
levers: 3
verdict: good
recordings/count_20261028_164947.png
recordings/count_20261028_164947_crop.png
=== 145329
part: box=(319, 186, 354, 232) confidence=0.75
dropped (too short): id 1 box=(323, 206, 364, 296) confidence=0.59
id 2 box=(241, 201, 290, 457) center=(265.5, 329.0) confidence=0.77
id 0 box=(162, 205, 212, 458) center=(187.0, 331.5) confidence=0.75
levers: 2
verdict: bad
recordings/count_20261028_164952.png
recordings/count_20261028_164952_crop.png
```

Good part 3/good OK, bad part remnant dropped, 2/bad OK. MIN_BAR_HEIGHT_FRAC 0.3 verified on the test pair. The images are not committed.
- Human ran count_clips on both images of the test pair and then the inspection report (started by the human, arm power supply off, no camera, 4 SAM calls):

```text
=== 145156
part: box=(317, 187, 352, 233) confidence=0.76
id 1 box=(158, 206, 210, 457) center=(184.0, 331.5) confidence=0.70
id 2 box=(237, 206, 289, 457) center=(263.0, 331.5) confidence=0.70
id 0 box=(313, 213, 363, 464) center=(338.0, 338.5) confidence=0.68
levers: 3
verdict: good
recordings/count_20261028_170351.png
recordings/count_20261028_170351_crop.png
recordings/count_20261028_170351.json
exit 0
=== 145329
part: box=(319, 186, 354, 232) confidence=0.75
dropped (too short): id 1 box=(323, 206, 364, 296) confidence=0.59
id 2 box=(241, 201, 290, 457) center=(265.5, 329.0) confidence=0.77
id 0 box=(162, 205, 212, 458) center=(187.0, 331.5) confidence=0.75
levers: 2
verdict: bad
recordings/count_20261028_170354.png
recordings/count_20261028_170354_crop.png
recordings/count_20261028_170354.json
exit 0
recordings/report.html
```

The generated report.html showed "parts inspected: 2, good: 1, bad: 1", the bad card first (BAD, levers 2 / expected 3, dropped remnant id 1 box=(323, 206, 364, 296)) and the good card second (GOOD, levers 3 / expected 3, no dropped remnants). OK. Found in that run: confidence was shown with 16 decimals and the card time was UTC without a label while file names use local time; both fixed in this change. Images, JSON and the report are not committed.
- Human measured the arm base position with a tape measure (2026-10-28, arm power supply off): distance from the centre of the joint 1 axis to the centre of marker 0 495 mm, to marker 1 485 mm, to marker 2 705 mm; least-squares fit with TABLE_MARKER_LAYOUT_MM gives the joint 1 axis at table (495.9, 22.7) mm with residuals of at most 1.6 mm. The arm base stands directly on the table top (as delivered by Seeed), so base z 0 is the table surface; with joint 1 at 0 the arm points along table +y (away from the table edge).
- Human ran python -m muse_qc.target_pose --table-mm 495.9 322.7 --start home (2026-10-28, offline, no hardware): OK, exit 0.
```text
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
ik joints deg: -0.00 -97.43 -82.51 +75.08 -0.00 +0.01
fk tip base mm: x=+300.00 y=+0.00 z=+100.00
fk tip table mm: x=+495.90 y=+322.70 z=+100.00
error to target: 0.00 mm
waypoints: 2149
all waypoints ok
dry run: no motion
exit 0
```
- Human ran the same target with --start arm (2026-10-28, arm power supply on, joints read once, no motion): FAIL, exit 1. The first waypoint is the real rest reading, where joint 2 sits slightly above its URDF limit; check_plan checked every waypoint against itself, so the LIMIT_RECOVERY_DEG rule could not see the motion back into range. The start joints were not printed in this version.
```text
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
ik joints deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
fk tip base mm: x=+300.00 y=-0.00 z=+99.99
fk tip table mm: x=+495.90 y=+322.70 z=+99.99
error to target: 0.01 mm
waypoints: 2163
waypoint 1/2163 REFUSED joint-limit
dry run: no motion
exit 1
```
- Human ran --start arm again after the check_plan fix (2026-10-28, arm power supply on, joints read once, no motion): OK, exit 0; joint 2 started at +0.69 deg and the recovery rule accepted the motion back into range.
```text
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
start joints deg: -1.89 +0.69 -0.23 +2.55 +1.30 -0.08
ik joints deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
fk tip base mm: x=+300.00 y=-0.00 z=+99.99
fk tip table mm: x=+495.90 y=+322.70 z=+99.99
error to target: 0.01 mm
waypoints: 2163
all waypoints ok
dry run: no motion
exit 0
```
Human work on the hover move (2026-10-28): the human requested muse_qc/hover_move.py, had the full diff reviewed before commit 1ae88d2 (the review found that the plan check compared every waypoint with the rest pose and would have refused the 97 deg motion; fixed before the commit), ran the dry run and the first real motion by hand, watched the arm with a hand on the power supply switch, and reported the joint 4 and joint 5 behaviour below.
- Human ran python -m muse_qc.hover_move --table-mm 495.9 322.7 (dry run, arm power supply on, no motion): OK. Last lines of the output:
```text
wp 4212/4212 [settle] joints=[-1.89 +0.00 -0.23 +2.55 +1.30 -0.08] tip=[+289.4 -14.4 +180.9] ok
cycle work (dry run): mean 1.16 ms, p95 1.22 ms, max 1.35 ms over 4212 waypoints
dry run: all waypoints pass
```
- Human ran the same target with --execute, output saved with tee (2026-10-28, arm power supply on, hand on the power supply switch, nothing on the table): OK, first motion to a table target. The arm ramped from the rest pose to the hover pose 100 mm (FK) above table point (495.9, 322.7), held 2 s, ramped back to the clamped rest pose, held 1 s, then the motors were disabled. The human typed yes at the prompt (tee does not record typed input, so the prompt line shows no answer). The per-waypoint lines are cut ("..."). Observation by the human: joint 4 moved in small visible steps instead of smoothly, and joint 5 moved slightly back and forth during the motion; no refusal, the run completed. The tip height above the table was not measured in this run.
```text
rest joints deg: ['-1.89', '+0.69', '-0.23', '+2.55', '+1.30', '-0.08']
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
start joints deg: -1.89 +0.69 -0.23 +2.55 +1.30 -0.08
ik joints deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
fk tip base mm: x=+300.00 y=-0.00 z=+99.99
fk tip table mm: x=+495.90 y=+322.70 z=+99.99
error to target: 0.01 mm
plan: 4212 waypoints at 100 Hz
...
cycle work (dry run): mean 1.17 ms, p95 1.23 ms, max 1.33 ms over 4212 waypoints
SDK vlim: ['5.0', '5.0', '5.0', '3.0', '3.0', '3.0'] -> 10 %: ['0.50', '0.50', '0.50', '0.30', '0.30', '0.30']
Type yes to move the arm: cycle work: mean 1.95 ms, p95 2.03 ms, max 2.19 ms over 4212 cycles
cycle period: mean 10.12 ms, p95 10.17 ms, max 10.23 ms (target 10.00 ms)
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
- Human ran python -m muse_qc.hover_move --table-mm 495.9 322.7 --execute a second time to measure the tip height (2026-10-28, arm power supply on, hand on the power supply switch, a ruler standing upright next to the target point): OK. The human read about 65 mm from the jaw tips to the table during the 2 s hold, about 35 mm lower than the FK hover of 100 mm (at the rest pose the difference was about 13 mm). Cause not yet known. Until it is understood, no target below 100 mm FK hover.
```text
rest joints deg: ['-1.89', '+1.02', '-0.34', '+2.55', '+1.34', '-0.08']
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
start joints deg: -1.89 +1.02 -0.34 +2.55 +1.34 -0.08
ik joints deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
fk tip base mm: x=+300.00 y=-0.00 z=+99.99
fk tip table mm: x=+495.90 y=+322.70 z=+99.99
error to target: 0.01 mm
plan: 4218 waypoints at 100 Hz
...
cycle work (dry run): mean 1.17 ms, p95 1.23 ms, max 1.34 ms over 4218 waypoints
SDK vlim: ['5.0', '5.0', '5.0', '3.0', '3.0', '3.0'] -> 10 %: ['0.50', '0.50', '0.50', '0.30', '0.30', '0.30']
Type yes to move the arm: cycle work: mean 1.97 ms, p95 2.07 ms, max 2.31 ms over 4218 cycles
cycle period: mean 10.12 ms, p95 10.17 ms, max 10.20 ms (target 10.00 ms)
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
Human work on the tip height (2026-10-28 evening): the human ran seven more hover moves by hand (one without a clear reading is not listed) (arm power supply on, hand on the power supply switch, nothing on the table), measured heights with a ruler and a folding rule (from the table top, about +-5 mm), measured joint axis heights and the gripper length with the power supply off, checked the joints for play by hand, and ran two offline helper scripts (one-off, in the git-ignored recordings/ folder). Together this located the height error.
- Human ran python -m muse_qc.hover_move --table-mm 495.9 322.7 --execute a third time with the new hold lines: OK. The measured hold joints match the commanded ones within 0.08 deg and FK of the measured joints gives z = 100.0 mm, so the motors reach their targets.
```text
hold commanded deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
hold measured deg: -0.01 -97.45 -82.52 +75.16 -0.03 -0.03
hold measured tip mm: x=+299.6 y=+0.1 z=+100.0
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
- Human ran python -m muse_qc.hover_move --table-mm 495.9 322.7 --hover 150 --execute: OK. Ruler at the jaw tips: about 130 mm (FK 149.9 mm).
```text
cycle work: mean 1.98 ms, p95 2.08 ms, max 7.98 ms over 4246 cycles
cycle period: mean 10.11 ms, p95 10.16 ms, max 11.56 ms (target 10.00 ms)
hold commanded deg: -0.00 -98.10 -94.85 +86.74 +0.00 +0.01
hold measured deg: -0.01 -98.13 -94.85 +86.72 -0.01 -0.01
hold measured tip mm: x=+300.1 y=-0.0 z=+149.9
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
- Human ran python -m muse_qc.hover_move --table-mm 495.9 242.7 --execute (220 mm in front of the base): OK. Ruler at the jaw tips: about 80 mm (FK 100.0 mm).
```text
cycle period: mean 10.12 ms, p95 10.17 ms, max 10.24 ms (target 10.00 ms)
hold commanded deg: -0.00 -79.99 -65.48 +75.49 +0.00 +0.01
hold measured deg: -0.01 -80.01 -65.49 +75.66 -0.01 -0.01
hold measured tip mm: x=+219.2 y=-0.0 z=+100.0
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
- First offline fit (recordings/z_offset_fit.py) over the three tip readings, model true joint = reading + offset. Columns: model z / residual in mm. It could not tell the models apart; the measurements below superseded it.
```text
(a) j2 off -5.7 deg | +72.4/-7.4 +122.0/+8.0 +80.4/-0.4 | rms 6.28
(b) j3 off +6.2 deg | +75.2/-10.2 +125.1/+4.9 +75.3/+4.7 | rms 7.09
(c) j4 off -0.6 deg | +99.9/-34.9 +149.9/-19.9 +99.9/-19.9 | rms 25.90
(d) j2j3 off -5.00 +0.75 deg | +72.9/-7.9 +122.6/+7.4 +79.9/+0.1 | rms 6.26
(e) zshift dz -24.9 mm | +75.0/-10.0 +124.9/+5.1 +75.0/+5.0 | rms 7.09
best single: (a) j2 dx at P1 -5.4 mm
```
- Human ran the 495.9 322.7 hover twice more: jaw tips about 60 mm; gripper mounting face on joint 6 about 260 mm.
- Second offline script (recordings/link_angles.py) printed model joint positions: at the rest pose the upper arm (joint 2 to joint 3) is horizontal, pointing backwards, and the forearm rises about 58 mm from joint 3 to joint 4; at the hover pose the joint 6 mounting face (link6) is at z = 285.3 mm in base_link.
- Human measured with the power supply off, arm on its support (table top to the centre of each joint axis): joint 2 135 mm, joint 3 135 mm, joint 4 190 mm (model 140, 140, 198.5 mm). So the upper arm is level as in the model, the forearm rise matches within about 1 deg, and the base sits about 5 mm lower than the model assumes. Gripper length from the end of motor 6 to the finger tips: 190 mm (the code uses 185 mm). No play felt at joints 2 and 3 when moved by hand.
- Human ran python -m muse_qc.hover_move --table-mm 495.9 322.7 --hold-s 15 --execute and measured during the 15 s hold, from the table top: finger tips 75 mm, mounting face 263 mm, joint 4 centre 365 mm, joint 3 centre 395 mm. Expected from the model (base 5 mm lower, gripper 190 mm): 90, 280, 387, 397 mm.
```text
cycle period: mean 10.08 ms, p95 10.10 ms, max 11.10 ms (target 10.00 ms)
hold commanded deg: -0.00 -97.43 -82.51 +75.08 +0.00 +0.01
hold measured deg: -0.01 -97.45 -82.52 +75.07 -0.01 +0.05
hold measured tip mm: x=+300.1 y=-0.0 z=+99.9
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
Result: shoulder and upper arm match the model (joint 3 within 2 mm). The error starts between joint 3 and joint 4: in this pose the forearm points about 5 deg lower than joint 3 reports, about 22 mm at joint 4; everything after joint 4 matches again (263 - 190 = 73 mm vs 75 mm measured). At the rest pose the forearm matches within 1 deg. Open: a joint 3 ratio error at large angles or the forearm giving way under its own weight; a pose with joint 3 strongly bent and little forearm load will tell them apart. Two small corrections are known: base about 5 mm lower, gripper 190 mm. No correction is applied in code yet. Rule until then: no target below 100 mm FK hover; the final approach to a part will descend slowly until the gripper touch sensors report contact.
Human verified the arm_calibration tip correction (2026-10-28 evening, arm power supply on, hand on the power supply switch, nothing on the table): two hover moves at 100 mm with --hold-s 15, finger tips measured with a ruler from the table top (about +-5 mm). Before the correction the same targets gave 60 to 75 mm and 80 mm.
- Human ran python -m muse_qc.hover_move --table-mm 495.9 322.7 --hold-s 15 --execute: OK. Ruler at the finger tips: 100 mm (target 100 mm).
```text
rest joints deg: ['-2.74', '+1.08', '-1.34', '+2.09', '+1.48', '-0.47']
table target: x=+495.90 y=+322.70 mm
base target: x=+300.00 y=-0.00 mm
hover: +100.0 mm
ik joints deg: -0.00 -103.55 -97.02 +83.47 -0.00 +0.01
fk tip base mm: x=+326.52 y=+0.00 z=+131.20
fk tip table mm: x=+495.90 y=+349.22 z=+131.20
error to target: 0.21 mm
corrected tip table mm: x=+495.90 y=+322.73 z=+99.79 (iterations 3)
plan: 5764 waypoints at 100 Hz
cycle work: mean 2.04 ms, p95 2.15 ms, max 5.71 ms over 5764 cycles
cycle period: mean 10.08 ms, p95 10.15 ms, max 12.93 ms (target 10.00 ms)
hold commanded deg: -0.00 -103.55 -97.02 +83.47 -0.00 +0.01
hold measured deg: -0.01 -103.57 -97.03 +83.53 -0.01 -0.01
hold measured tip mm: x=+299.8 y=-0.0 z=+99.8
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
- Human ran python -m muse_qc.hover_move --table-mm 495.9 242.7 --hold-s 15 --execute: OK. Ruler at the finger tips: 100 mm (target 100 mm).
```text
rest joints deg: ['-2.74', '+1.02', '-1.41', '+2.07', '+1.48', '-0.47']
table target: x=+495.90 y=+242.70 mm
base target: x=+220.00 y=-0.00 mm
hover: +100.0 mm
ik joints deg: -0.00 -84.70 -75.69 +80.99 -0.00 +0.01
fk tip base mm: x=+241.19 y=+0.00 z=+126.69
fk tip table mm: x=+495.90 y=+263.89 z=+126.69
error to target: 0.21 mm
corrected tip table mm: x=+495.90 y=+242.61 z=+99.81 (iterations 3)
plan: 5010 waypoints at 100 Hz
cycle work: mean 2.02 ms, p95 2.13 ms, max 7.56 ms over 5010 cycles
cycle period: mean 10.09 ms, p95 10.16 ms, max 13.20 ms (target 10.00 ms)
hold commanded deg: -0.00 -84.70 -75.69 +80.99 -0.00 +0.01
hold measured deg: -0.01 -84.71 -75.70 +80.97 -0.01 -0.01
hold measured tip mm: x=+220.1 y=-0.0 z=+99.8
motion finished, arm holds the settle pose
stopping: the motors will be disabled now
```
- Agent ran python -m muse_qc.arm_calibration (offline fit report, no hardware):
```text
A1: measured=75.0 ratio z=71.7 r=-3.3 load z=73.4 r=-1.6
A2: measured=263.0 ratio z=261.1 r=-1.9 load z=262.9 r=-0.1
A3: measured=365.0 ratio z=366.8 r=+1.8 load z=368.8 r=+3.8
A4: measured=395.0 ratio z=397.0 r=+2.0 load z=397.0 r=+2.0
B: measured=65.0 ratio z=71.7 r=+6.7 load z=73.4 r=+8.4
C1: measured=60.0 ratio z=71.7 r=+11.7 load z=73.4 r=+13.4
C2: measured=260.0 ratio z=261.1 r=+1.1 load z=262.9 r=+2.9
D: measured=130.0 ratio z=119.0 r=-11.0 load z=123.6 r=-6.4
E: measured=80.0 ratio z=75.4 r=-4.6 load z=73.5 r=-6.5
F1: measured=135.0 ratio z=135.2 r=+0.2 load z=135.2 r=+0.2
F2: measured=135.0 ratio z=135.2 r=+0.2 load z=135.2 r=+0.2
F3: measured=190.0 ratio z=193.3 r=+3.3 load z=176.4 r=-13.6
ratio: k=-0.0556 rms=5.45 mm
load: g=4.1395 deg rms=6.76 mm
```
Result: with the correction both 100 mm hovers land at 100 mm (within the ruler accuracy); the nominal model would have put the tip at 131 and 127 mm for these joints. The nominal 150 mm hover at 495.9 322.7 is outside the tool-down workspace once corrected (joint 4 limit), which matches the 130 mm measured there before. Verified only at these two points; heights below 100 mm are not verified yet, and the final approach to a part will still descend slowly until the gripper touch sensors report contact.
Human ran python -m muse_qc.gripper --watch 30 twice (2026-10-28, arm power supply off, only USB, gripper Pico on /dev/ttyACM1 found by USB ID).
- First run, fingers not touched: OK, no contact events.
```text
pico: /dev/ttyACM1
firmware reply: 1,24,25,- / D,594566,3,-0,18,18,2,3,27,27,1,0,17,17,9,1,25,27,- / D,594782,5,-0,16,17,1,1,34,34,0,-2,22,22,3,1,27,27,- / # Ausgabe: Rohwerte (expected 2)
do not touch the fingers
start: contacts -
summary: lines=1219 data=1213 ignored=6 changes=0
events: L1=0 L2=0 R1=0 R2=0
max magnitude uT: L1=7576.0 L2=9710.0 R1=7909.0 R2=7356.0
```
- Second run, the human pressed the fingers by hand in no fixed order: the firmware reported contacts on all four sensors (L1 3, L2 4, R1 4, R2 5 events) and "-" again after release; neighbouring lamellae often reported together. The firmware reply line (about 45 raw data lines plus "# firmware 2") is shortened here. Found two bugs, fixed in the next change: all change lines show +30.0s (lines evaluated only at the end), and the version check matched a "2" inside a data line.
```text
pico: /dev/ttyACM1
firmware reply: [about 45 raw data lines omitted] / # firmware 2 / [raw data lines omitted] (expected 2)
do not touch the fingers
start: contacts -
+30.0s contacts: L1 L2 (safety open: unexpected-contact)
+30.0s contacts: L1 L2 R2 (safety open: unexpected-contact)
+30.0s contacts: L1 L2 R1 R2 (safety open: unexpected-contact)
[20 more change lines, all +30.0s, omitted]
summary: lines=1235 data=1229 ignored=6 changes=23
events: L1=3 L2=4 R1=4 R2=5
max magnitude uT: L1=7782.0 L2=10365.0 R1=8277.0 R2=7601.0
```
- Human ran python -m muse_qc.gripper --watch 15 after the live-read fix (2026-10-28, arm power supply off, only USB): OK. The human pressed one finger once near the end; the change line shows its real time stamp and the firmware version is read from "# firmware 2".
```text
pico: /dev/ttyACM1
firmware: 2
do not touch the fingers
start: contacts -
+14.6s contacts: L2 (safety open: unexpected-contact)
summary: lines=591 data=584 ignored=7 changes=1
events: L1=0 L2=1 R1=0 R2=0
max magnitude uT (as sent by the firmware): L1=7640.0 L2=9775.0 R1=7950.0 R2=7228.0
```
- Human ran python -m muse_qc.gripper_motor --delta-deg 10 (dry run) and --delta-deg 10 --execute (2026-10-28, arm power supply on, hand on the power supply switch, nothing between the jaws, jaws about 5 mm open at the tips): OK. The gripper moved +10 deg and back; the arm held its clamped rest pose (joint 2 from +1.21 to 0.00).
```text
rest joints deg: ['-2.74', '+1.21', '-1.45', '+2.72', '+2.83', '-0.49']
gripper rest deg: -4.97
plan: 700 waypoints at 100 Hz
arm hold deg: ['-2.74', '+0.00', '-1.45', '+2.72', '+2.83', '-0.49']
arm hold: ok
gripper commanded deg: +5.03
gripper measured deg: +4.78
gripper back deg: -4.97
motion finished, gripper back at the start reading
stopping: the motors will be disabled now
```
- Human measured the gripper scale (power supply off, jaws moved by hand, read with the dry run, tip gap with a ruler): +2.79 deg closed (0 mm), -4.97 deg 5 mm, -102.63 deg 32 mm, fully open 98 mm read +33.21 deg (the reading wraps at +-180 deg); with the earlier -167.13 deg at 53 mm this gives about 0.31 mm per motor degree, negative = open (the URDF-based guess was 0.32).
- Human ran python -m muse_qc.gripper_grip --open-mm 40 (dry run) and --open-mm 40 --execute (2026-10-28, arm power supply on, hand on the power supply switch, jaws first set by hand to about 32 mm with the power off, the human held one connector part (about 17 mm wide) between the jaws): OK. The gripper opened to 40 mm, closed at 5 deg/s until the left and right fingers reported contact, squeezed 2 deg, held 3 s and opened again; the part was held firmly. Closing at 5 deg/s felt too slow, changed in the next change.
```text
rest joints deg: ['-2.74', '+1.10', '-1.45', '+2.72', '+2.42', '-1.96']
gripper rest deg: -100.12 (width about 31.9 mm)
open width: 40 mm -> gripper -126.23 deg
squeeze: 2.0 deg past first bilateral contact
arm hold deg: ['-2.74', '+0.00', '-1.45', '+2.72', '+2.42', '-1.96']
arm hold: ok
pico: /dev/ttyACM1
firmware: 2
do not touch the fingers
output mode: deviation
place the part between the jaws now
contact: gripper commanded deg -46.18 measured deg -47.27 contacts L2 R2
grip held at deg -44.18 width about 14.6 mm contacts L1 L2 R1 R2
hold end: gripper commanded deg -44.18 measured deg -44.36
gripper final deg: commanded -126.23 measured -126.32
motion finished, gripper back at the open width
stopping: the motors will be disabled now
```
The width at contact (about 15 mm by the scale) is below the part width because the soft Fin-Ray fingers wrap around the part.
- Human ran python -m muse_qc.gripper_grip --open-mm 40 --execute with the new default speed 15 deg/s (2026-10-28, arm power supply on, hand on the power supply switch, one connector part held between the jaws): OK, held with all four finger sensors, the gripper lagged about 3 deg behind the command while closing.
```text
gripper rest deg: -126.32 (width about 40.0 mm)
phases at 100 Hz: open at 15 deg/s, wait 3 s (place the part), close at 15 deg/s to bilateral contact, squeeze, hold 3 s, open back, hold 1 s
arm hold: ok
place the part between the jaws now
contact: gripper commanded deg -44.63 measured deg -47.97 contacts L1 R2
grip held at deg -42.63 width about 14.1 mm contacts L1 L2 R1 R2
hold end: gripper commanded deg -42.63 measured deg -42.63
gripper final deg: commanded -126.23 measured -126.43
motion finished, gripper back at the open width
```
- Human measured the x/y error of the tip at three points (2026-10-28, arm power supply on, hand on the power supply switch, method as in SETUP section 27; "small part" found no box this time, "white cube" did, confidence 0.62 to 0.88). The arm reached its commanded joints within 0.21 deg every time (joint 4 visibly oscillated a little while holding), so the error is outside the motor control.
  P1 A=(394.6, 318.6) B=(376.5, 331.3); hold measured tip mm: x=+297.1 y=+101.6 z=+99.7
  P2 A=(505.0, 348.7) B=(491.6, 371.8); hover output not kept
  P3 A=(690.4, 307.6) B=(698.2, 333.4); hold measured tip mm: x=+285.6 y=-195.0 z=+99.7
  A first try for P2 at table (509.7, 383.2) = base (360.5, -13.8) was refused before moving: "plan failed: unreachable: IK did not converge". With the gripper pointing straight down at 100 mm hover the reach ends between 345 mm (P3 worked) and 360 mm from the joint 1 axis.
- Human ran python -m muse_qc.pick_test --table-mm 495.9 330 (dry run, 2026-10-29, arm power supply on): the first try was refused at waypoint 7673 (invalid current reading: the clamped joint 2 read exactly 0.0 in the chained pre-check), fixed in b15bfd8. After the fix:
```text
rest joints deg: ['-2.79', '+1.13', '-1.72', '+2.72', '+2.44', '-1.91']
gripper rest deg: -126.45 (width about 40.1 mm)
segments at 100 Hz: hover 2016 lower 920 lift 920 put-down 900 raise 900 return 2116 (total 7772)
hover tip z: 99.8 mm, grip tip z: 8.0 mm
corrected base target: x=+288.35 y=-9.75 mm
xy correction: on
all waypoints ok
dry run: moves nothing, motors never enabled
```
- Human ran python -m muse_qc.pick_test --table-mm 495.9 330 --no-grip --grip-z-mm 40 --execute and the same with --grip-z-mm 20 (2026-10-29, arm power supply on, hand on the power supply switch): OK. Printed "holding at tip z 39.7 mm" and "holding at tip z 19.7 mm"; measured with a ruler 40 mm and 19 mm. The height correction holds within about 1 mm down to 20 mm.
- Human checked the x/y correction at low height (2026-10-29): during the 40 mm hold the white cube was slid under the centre between the finger tips; afterwards python -m muse_qc.vision_sam "white cube" gave table_mm=(491.0, 327.8) (confidence 0.80) for the commanded (495.9, 330), about 5 mm off.
- Human ran python -m muse_qc.pick_test --table-mm 495.9 330 --execute (2026-10-29, grip z 8 mm): "missed grip", correct: the table image showed that no part lay at the target.
- Human ran the same with one connector part placed at the target (2026-10-29, grip z 8 mm): the fingers closed to bilateral contact, but the part slipped out while lifting:
```text
contact: gripper commanded deg -50.93 measured deg -54.22 contacts L1 L2 R2
contacts L1 L2 R1 R2 (lifting)
contacts L2 R1 R2 (lifting)
contacts L2 R2 (lifting)
contacts R2 (lifting)
part lost while lifting
contacts - (lifting)
```
- Human ran python -m muse_qc.vision_sam with "small part", "grey block", "orange and grey object" and "small grey box" on the connector part from the table camera (2026-10-29): 0 boxes each (on 2026-10-28 "small part" had found it). The part is only about 25 px wide in the 1920x1080 image.
- Human ran python -m muse_qc.pick_test --table-mm 495.9 330 --grip-z-mm 5 --execute (2026-10-29, arm power supply on, hand on the power supply switch, the part placed by hand under the finger tips during a --no-grip 40 mm hold): OK, FIRST PICK. The part was lifted to 100 mm, held with all four sensors, put down and released.
```text
segments at 100 Hz: hover 2016 lower 950 lift 950 put-down 930 raise 930 return 2116 (total 7892)
hover tip z: 99.8 mm, grip tip z: 5.0 mm
corrected base target: x=+288.35 y=-9.75 mm
xy correction: on
all waypoints ok
contact: gripper commanded deg -45.83 measured deg -49.39 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
stopping: the motors will be disabled now
end readings: arm [-2.79 +0.12 -2.04 +2.94 +2.55 -0.36] gripper -126.28 deg
```
