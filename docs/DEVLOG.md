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
