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
- Human ran python -m muse_qc.find_parts --save-empty (table empty, arm at rest), then --no-sam and the SAM run with one connector part and the white cube on the table (2026-10-29): the diff found the part (area about 430 mm2), the cube was not a candidate (probably merged with its long shadow above 3000 mm2), and SAM on the 640 px crop gave "no box" for "small part", "connector", "terminal block", "electrical connector" and "object", but confirmed it with "grey object":
```text
part 0 center=(1148, 344) area_mm2=431 table_mm=(594.8, 369.4) sam=skipped
candidates: 1
part 0 center=(1146, 340) area_mm2=416 table_mm=(593.8, 371.7) sam=confirmed 0.82
parts: 1 confirmed of 1 candidates
```
  (594, 372) is about 362 mm from the joint 1 axis, beyond the reach, so the part was moved closer.
- Human ran python -m muse_qc.find_parts --phrase "grey object" and then python -m muse_qc.pick_test --table-mm 622.1 311.3 --execute with the camera result (2026-10-29, arm power supply on, hand on the power supply switch): OK, FIRST FULLY AUTOMATIC PICK (camera finds the part, SAM confirms it, the arm picks it). Only one lamella per side touched (the arm was turned about 23 deg, the part lay diagonally between the jaws); human: "a bit imprecise but OK".
```text
part 0 center=(1192, 438) area_mm2=419 table_mm=(622.1, 311.3) sam=confirmed 0.61
parts: 1 confirmed of 1 candidates
segments at 100 Hz: hover 2053 lower 950 lift 950 put-down 930 raise 930 return 2153 (total 7966)
corrected base target: x=+267.22 y=-127.78 mm
all waypoints ok
contact: gripper commanded deg -58.28 measured deg -61.63 contacts L2 R2
contacts L2 R2 (lifting)
part lifted, holding
end readings: arm [-2.79 +0.25 -2.09 +2.98 +2.57 -0.36] gripper -126.28 deg
```
- Human ran python -m muse_qc.hold_test --table-mm 495.9 330 --execute with --gains default and with --gains soft (2026-10-29, arm power supply on, 20 s hold at 100 mm): no visible difference; soft was slightly worse, default stays.
```text
default: joint 4: mean err -0.01 deg, p2p 1.07 deg, rms 0.05 deg, swing 0.1 Hz / joint 5: mean err -0.02 deg, p2p 0.15 deg, rms 0.03 deg, swing n/a
soft:    joint 4: mean err -0.02 deg, p2p 1.25 deg, rms 0.09 deg, swing 1.0 Hz / joint 5: mean err -0.03 deg, p2p 0.35 deg, rms 0.05 deg, swing n/a
```
  The CSVs show that the peak-to-peak comes only from the first 0.7 s (joints 2/3/4 lag 1.0 to 1.4 deg behind the end of the ramp and then settle); for the remaining 19 s all joints read within +-0.02 deg. The human still sees about 5 mm of swing at the finger tips while moving and standing, so it happens behind the motor encoders (gear play in joints 4/5 or the soft fingers), which no position gain can see. Hence the smoother trapezoid ramps and the 0.5 s settle in the next change.
- Human ran python -m muse_qc.pick_test --table-mm 507.6 327.0 --execute after the speed-up (2026-10-29, arm power supply on, hand on the power supply switch; two earlier attempts stopped with JointStateUnavailable on joint 1 because the arm power supply was still off): OK, plan 2818 cycles at 100 Hz (about 28 s instead of about 79 s), contact L2 R2 at gripper commanded -45.22 deg / measured -48.62 deg, all four sensors held while lifting. Human: "still a bit wobbly, but better".
- Human ran python -m muse_qc.hold_test --table-mm 495.9 330 --execute with --gains default (2026-10-29, arm power supply on) and pushed the finger tips gently about 5 mm sideways by hand during the hold: the wrist visibly moved with the push ("the arm moves along slightly"), and the encoders saw it, so the swing is mostly servo compliance of joints 4/5, not only play behind the encoder.
```text
hold commanded deg: -1.94 -100.76 -93.42 +82.66 -0.00 +0.01
hold measured deg: -1.93 -100.77 -93.43 +82.61 -0.21 -0.01
hold measured tip mm: x=+288.7 y=-9.0 z=+99.7
joint 1: mean err -0.01 deg, p2p 0.07 deg, rms 0.02 deg
joint 2: mean err +0.00 deg, p2p 1.36 deg, rms 0.11 deg
joint 3: mean err +0.00 deg, p2p 1.22 deg, rms 0.10 deg
joint 4: mean err -0.01 deg, p2p 1.09 deg, rms 0.06 deg, swing 0.8 Hz
joint 5: mean err -0.01 deg, p2p 1.77 deg, rms 0.27 deg, swing 0.5 Hz
joint 6: mean err -0.01 deg, p2p 0.07 deg, rms 0.02 deg
```
  Without the push (earlier run, same point) joint 5 had p2p 0.15 deg and rms 0.03 deg. Next: A/B with a stiffer joint 4/5 profile.
- Human ran python -m muse_qc.hold_test --table-mm 495.9 330 --gains stiff --execute (2026-10-29, arm power supply on) with the same gentle hand push on the finger tips during the hold: no improvement. Joint 4 got worse (p2p 4.63 deg vs 1.09 deg with default), no buzzing or oscillation on its own, but the motion felt "strange" to the human. Hand pushes are not exactly repeatable, so the comparison is rough; stiff is not used.
```text
hold commanded deg: -1.94 -100.76 -93.42 +82.66 -0.00 +0.01
hold measured deg: -1.98 -100.75 -93.45 +84.03 -0.01 +0.10
hold measured tip mm: x=+281.5 y=-9.7 z=+100.7
joint 1: mean err -0.01 deg, p2p 0.15 deg, rms 0.03 deg
joint 2: mean err +0.00 deg, p2p 1.42 deg, rms 0.11 deg
joint 3: mean err +0.00 deg, p2p 1.25 deg, rms 0.10 deg
joint 4: mean err -0.01 deg, p2p 4.63 deg, rms 0.41 deg, swing 1.1 Hz
joint 5: mean err -0.01 deg, p2p 2.27 deg, rms 0.35 deg, swing 0.9 Hz
joint 6: mean err -0.01 deg, p2p 0.17 deg, rms 0.04 deg
```
- Human ran python -m muse_qc.hold_test --table-mm 495.9 330 --gains vel2 --execute (2026-10-29, arm power supply on), first without touching, then with the gentle hand push on the finger tips. Without push vel2 is clearly calmer than default (joint 4 p2p 0.55 vs 1.07 deg, joint 5 p2p 0.04 vs 0.15 deg, no swing frequency any more); with push the numbers mostly show how far the hand pushed (not repeatable), the joints came back exactly. No buzzing.
```text
without push:
hold measured deg: -1.93 -100.77 -93.43 +82.65 -0.01 -0.01
joint 4: mean err -0.01 deg, p2p 0.55 deg, rms 0.03 deg, swing n/a
joint 5: mean err -0.01 deg, p2p 0.04 deg, rms 0.02 deg, swing n/a
with push:
hold measured deg: -1.93 -100.77 -93.43 +82.65 -0.01 -0.01
joint 4: mean err -0.01 deg, p2p 4.13 deg, rms 0.52 deg, swing 2.1 Hz
joint 5: mean err -0.01 deg, p2p 3.23 deg, rms 0.40 deg, swing 0.8 Hz
```
- Human ran python -m muse_qc.find_parts with the two boxes in place (new empty-table reference with the boxes, 2026-10-29): the part was confirmed, four candidates on the gripper fingers and cables (the arm rested slightly off the reference pose) were rejected by SAM.
```text
part 0 center=(952, 740) area_mm2=206 table_mm=(472.8, 123.9) sam=no box
part 1 center=(960, 503) area_mm2=918 table_mm=(478.1, 271.8) sam=no box
part 2 center=(1002, 772) area_mm2=202 table_mm=(503.8, 104.4) sam=no box
part 3 center=(1003, 360) area_mm2=575 table_mm=(505.3, 360.0) sam=confirmed 0.82
part 4 center=(1054, 512) area_mm2=751 table_mm=(536.5, 265.6) sam=no box
parts: 1 confirmed of 5 candidates
```
- Human ran python -m muse_qc.pick_test --table-mm 505.3 360.0 --gains vel2 --execute (2026-10-29, arm power supply on, hand on the power supply switch): OK, part picked and lifted with all four sensors. Human: "still wobbles, but not as strongly any more".
```text
segments at 100 Hz: hover 820 lower 390 lift 270 put-down 386 raise 186 return 870 (total 2922)
corrected base target: x=+316.93 y=-19.57 mm
contact: gripper commanded deg -46.57 measured deg -49.82 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
```
- Human ran python -m muse_qc.vision_sam "box" with the two sorting boxes (100 x 100 x 70 mm) on the table (2026-10-29): both found, left box = good, right box = bad.
```text
table mapping: calibrated
id 0 box=(1187, 170, 1372, 350) center=(1279.5, 260.0) confidence=0.83 table_mm=(675.4, 420.3)
id 1 box=(714, 173, 875, 342) center=(794.5, 257.5) confidence=0.83 table_mm=(376.4, 423.8)
boxes: 2
```
- Human ran python -m muse_qc.find_parts and python -m muse_qc.pick_test --table-mm 502.5 360.5 --drop good (dry run, then --execute) (2026-10-29, arm power supply on, hand on the power supply switch; one earlier attempt stopped with JointStateUnavailable because the power supply was still off): OK, FIRST FULL SORTING RUN. The arm picked the part, carried it with the tool tilted 30 deg to the left box and dropped it in; the touch sensors confirmed the release. Human: "it gripped it and put it into the green box".
```text
part 0 center=(999, 360) area_mm2=553 table_mm=(502.5, 360.5) sam=confirmed 0.88
segments at 100 Hz: hover 821 lower 390 lift 270 to-box 321 release 1 return 761 (total 2564)
drop box: good at table mm (+376.4, +423.8), r=418.5 mm
drop joints deg: +16.59 -91.53 -83.54 +52.01 -0.00 +0.00
drop tilt: 30.0 deg, fk tip base mm (+401.1, +119.5, +170.0), real tip z 129.9 mm
to-box lowest tip z: 99.7 mm
contact: gripper commanded deg -45.82 measured deg -49.32 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
dropped into the good box
```
- Human ran python -m muse_qc.find_parts and python -m muse_qc.pick_test --table-mm 513.4 353.3 --drop bad --execute (2026-10-29, arm power supply on): OK, the part went into the right (bad) box. find_parts also reported the part already lying in the good box and three candidates on the gripper fingers and cables (fixed in this change by the skip zones).
```text
part 0 center=(818, 212) area_mm2=900 table_mm=(390.9, 451.5) sam=confirmed 0.72
part 1 center=(920, 600) area_mm2=1384 table_mm=(452.8, 211.7) sam=no box
part 2 center=(952, 738) area_mm2=200 table_mm=(472.7, 125.9) sam=no box
part 3 center=(954, 510) area_mm2=2412 table_mm=(474.3, 267.4) sam=no box
part 4 center=(1016, 371) area_mm2=545 table_mm=(513.4, 353.3) sam=confirmed 0.86
parts: 2 confirmed of 5 candidates
segments at 100 Hz: hover 811 lower 390 lift 270 to-box 316 release 1 return 787 (total 2575)
drop box: bad at table mm (+675.4, +420.3), r=436.2 mm
drop joints deg: -24.29 -95.40 -87.66 +52.25 +0.00 +0.01
drop tilt: 30.0 deg, fk tip base mm (+397.6, -179.5, +170.0), real tip z 128.4 mm
to-box lowest tip z: 99.3 mm
contact: gripper commanded deg -45.67 measured deg -49.21 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
dropped into the bad box
```
- Human ran python -m muse_qc.find_parts --no-sam after retaking the empty-table reference with the two boxes in place (2026-10-29, before the skip zones): "candidates: 0".
- Human ran python -m muse_qc.find_parts with the skip zones (2026-10-29, one part in each box, gripper resting 40 mm open, empty-table reference taken with a different gripper opening): every candidate was skipped, no SAM call.
```text
part 0 center=(814, 212) area_mm2=987 table_mm=(388.5, 451.7) skipped: in box good
part 1 center=(922, 598) area_mm2=1061 table_mm=(454.3, 212.4) skipped: arm zone
part 2 center=(956, 510) area_mm2=2127 table_mm=(475.1, 267.8) skipped: arm zone
part 3 center=(1061, 516) area_mm2=2284 table_mm=(540.7, 263.5) skipped: arm zone
part 4 center=(1075, 614) area_mm2=1220 table_mm=(549.2, 202.8) skipped: arm zone
part 5 center=(1228, 246) area_mm2=742 table_mm=(644.0, 428.9) skipped: in box bad
parts: 0 confirmed of 6 candidates (6 skipped)
```
- Human retook the empty-table reference with python -m muse_qc.find_parts --save-empty (2026-10-29, boxes empty, gripper resting 40 mm open as after every pick), then ran --no-sam on the empty table ("candidates: 0") and the SAM run with one part placed in front of the gripper:
```text
part 0 center=(1014, 380) area_mm2=655 table_mm=(511.3, 348.0) sam=confirmed 0.88
parts: 1 confirmed of 1 candidates
```
  The human noted the shadows of the lamp, the arm and the boxes; they are part of the reference, so only lighting changes matter (retake the reference after a lighting change).
- Human ran python -m muse_qc.pick_test --table-mm 511.3 348.0 --drop good (dry run, then --execute) (2026-10-29, arm power supply on, hand on the power supply switch): OK, third sorting run.
```text
to-box lowest tip z: 99.6 mm
contact: gripper commanded deg -45.97 measured deg -49.43 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
dropped into the good box
end readings: arm [-2.79 +0.14 -2.68 +4.62 +2.57 -0.45] gripper -126.30 deg
```
- Human ran python -m muse_qc.find_parts and python -m muse_qc.pick_test --table-mm 512.3 343.7 --drop bad (dry run, then --execute) (2026-10-29, arm power supply on): OK, fourth sorting run. The arm settles a few tenths of a degree differently after each run, so small changes on the gripper show up as candidates; the arm zone skips them.
```text
part 0 center=(781, 216) area_mm2=868 table_mm=(368.3, 449.6) skipped: in box good
part 1 center=(901, 634) area_mm2=193 table_mm=(440.9, 190.0) skipped: arm zone
part 2 center=(913, 568) area_mm2=187 table_mm=(448.6, 231.7) skipped: arm zone
part 3 center=(949, 502) area_mm2=781 table_mm=(471.2, 272.1) skipped: arm zone
part 4 center=(953, 740) area_mm2=198 table_mm=(473.0, 124.4) skipped: arm zone
part 5 center=(1015, 387) area_mm2=618 table_mm=(512.3, 343.7) sam=confirmed 0.73
part 6 center=(1066, 514) area_mm2=924 table_mm=(543.6, 264.7) skipped: arm zone
part 7 center=(1085, 620) area_mm2=182 table_mm=(555.4, 198.5) skipped: arm zone
parts: 1 confirmed of 8 candidates (7 skipped)
to-box lowest tip z: 99.1 mm
contact: gripper commanded deg -45.67 measured deg -49.25 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
dropped into the bad box
```
- Human tried the full chain by hand (2026-10-29, arm power supply on): python -m muse_qc.find_parts found the part at table mm (514.8, 342.0) (sam=confirmed 0.81, 2 candidates skipped "in box good"), then python -m muse_qc.camera_arm took recordings/armcam_20261029_133942.png with the arm back in the home pose (the part lies free in front of the fingers, about 31 px high), then python -m muse_qc.count_clips on that image: FAIL, a good part with 3 intact levers was judged bad because all three lever boxes were dropped as too short (fixed in this change).
```text
part: box=(327, 187, 354, 218) confidence=0.80
dropped (too short): id 2 box=(222, 249, 258, 373) confidence=0.77
dropped (too short): id 1 box=(171, 251, 208, 374) confidence=0.74
dropped (too short): id 0 box=(282, 244, 314, 371) confidence=0.73
levers: 0
verdict: bad
```
- Human ran python -m muse_qc.count_clips --image recordings/armcam_20261029_133942.png again with the new bar rules (2026-10-29, no arm motion, 2 SAM calls): OK, the good part is now counted correctly.
```text
part: box=(327, 187, 354, 218) confidence=0.80
id 2 box=(222, 249, 258, 373) center=(240.0, 311.0) confidence=0.77
id 1 box=(171, 251, 208, 374) center=(189.5, 312.5) confidence=0.74
id 0 box=(282, 244, 314, 371) center=(298.0, 307.5) confidence=0.73
levers: 3
verdict: good
```
Human work on sorting (2026-10-29): the human placed and measured the two sorting boxes (100 x 100 x 70 mm, good left, bad right) after the first placement left too little room, decided on the vel2 gains after comparing default, soft, stiff and vel2 hold runs with a gentle hand push, reviewed every diff (review file before the commit, push review before every push), asked for the skip zones after seeing candidates in the boxes and on the gripper, retook the empty-table reference, ran all sorting runs with a hand on the power supply switch, and found the lever-count failure by running the camera, counting and picking steps one after another.
- Human ran python -m muse_qc.hover_move --table-mm 513.3 340.7 --hold-s 30 --execute and python -m muse_qc.camera_arm in a second terminal during the hold, then python -m muse_qc.count_clips on that image (2026-10-29 evening, lamp light only, arm power supply on): OK. The arm camera is mounted tilted forward, so at the hover pose it sees the part free in front of the finger tips (not straight down); the hover pose works as the inspection pose. Joint 4 swayed again during hover_move (it runs with the default gains, vel2 is only in pick_test). Correction to the earlier "full chain by hand" entry: recordings/armcam_20261029_133942.png was taken at the hover pose too, not in the home pose (same view, same part position in the image).
```text
part: box=(327, 192, 356, 223) confidence=0.83
id 2 box=(239, 249, 274, 372) center=(256.5, 310.5) confidence=0.76
id 1 box=(189, 251, 224, 373) center=(206.5, 312.0) confidence=0.74
id 0 box=(298, 245, 327, 369) center=(312.5, 307.0) confidence=0.72
levers: 3
verdict: good
```
- Human ran python -m muse_qc.hold_test --table-mm 495.9 330 --gains vel3 --execute twice (2026-10-29 evening, arm power supply on, 20 s hold at 100 mm), first without touching, then with the gentle hand push on the finger tips: NOT ADOPTED. Without push joint 4 was a little calmer than vel2 (p2p 0.44 vs 0.55 deg), but the human heard the motors hum. With the push joint 4 rang at 1.7 Hz (p2p 2.71 deg) instead of coming back cleanly as with vel2. vel2 stays the pick default, vel3 is not used.
```text
=== without push
joint 1: mean err -0.02 deg, p2p 0.04 deg, rms 0.02 deg
joint 2: mean err +0.00 deg, p2p 1.36 deg, rms 0.11 deg
joint 3: mean err +0.00 deg, p2p 1.22 deg, rms 0.10 deg
joint 4: mean err -0.01 deg, p2p 0.44 deg, rms 0.04 deg, swing 0.2 Hz
joint 5: mean err -0.01 deg, p2p 0.02 deg, rms 0.01 deg, swing n/a
joint 6: mean err -0.01 deg, p2p 0.07 deg, rms 0.02 deg
=== with push
joint 1: mean err -0.01 deg, p2p 0.17 deg, rms 0.03 deg
joint 2: mean err +0.01 deg, p2p 1.46 deg, rms 0.12 deg
joint 3: mean err +0.00 deg, p2p 1.27 deg, rms 0.10 deg
joint 4: mean err -0.01 deg, p2p 2.71 deg, rms 0.35 deg, swing 1.7 Hz
joint 5: mean err -0.01 deg, p2p 1.07 deg, rms 0.15 deg, swing 0.6 Hz
joint 6: mean err -0.01 deg, p2p 0.24 deg, rms 0.06 deg
```
- Human ran python -m muse_qc.find_parts and python -m muse_qc.pick_test --table-mm 513.3 341.1 --inspect (dry run, then --execute) with a good part (3 levers) (2026-10-29 evening, lamp light only, arm power supply on, hand on the power supply switch): OK, FIRST SELF-DECIDED SORTING RUN. The arm hovered over the part, took the arm camera image, counted 3 levers, decided "good" on its own and dropped the part into the good box. find_parts also showed two candidates on the cable next to the base (table mm (350.1, 4.9) and (358.7, 56.9)), both rejected by SAM ("no box").
```text
part 9 center=(1016, 390) area_mm2=285 table_mm=(513.3, 341.1) sam=confirmed 0.72
parts: 1 confirmed of 11 candidates (8 skipped)
inspect: camera /dev/video6
inspect: api key present
all waypoints ok
attempt 1 ok: True
inspect: levers 3 verdict good
contact: gripper commanded deg -45.82 measured deg -49.36 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
dropped into the good box
end readings: arm [-2.79 +0.14 -2.96 +5.56 +2.57 -0.45] gripper -126.28 deg
```
- Human ran python -m muse_qc.find_parts and python -m muse_qc.pick_test --table-mm 510.1 318.0 --inspect (dry run, then --execute) with a bad part (one lever torn out, 2 levers) (2026-10-29 evening, same conditions): OK. The arm counted 2 levers, decided "bad" on its own and dropped the part into the bad box. The part lay closer to the arm than the fitted range of the x/y correction (corrected base x 275.9 mm, fit range about 313 to 345 mm); the grip still worked.
```text
part 9 center=(1011, 428) area_mm2=287 table_mm=(510.1, 318.0) sam=confirmed 0.63
parts: 1 confirmed of 11 candidates (8 skipped)
corrected base target: x=+275.93 y=-22.64 mm
all waypoints ok
attempt 1 ok: True
inspect: levers 2 verdict bad
contact: gripper commanded deg -46.72 measured deg -49.91 contacts L2 R2
contacts L2 R2 (lifting)
part lifted, holding
dropped into the bad box
end readings: arm [-2.79 +0.32 -3.07 +5.72 +2.57 -0.45] gripper -126.30 deg
```
Human work on the inspection (2026-10-29 evening): the human found by watching the arm that the arm camera looks forward and not straight down at the hover pose, which made the hover pose usable as the inspection pose; had the --inspect change reviewed before the commit (the review found that the plan check chained the bad branch after the good return and would have refused every real plan), and ran both self-decided runs with a hand on the power supply switch.
- Human ran `uv run python -m muse_qc.agent` (2026-10-29 ~21:55, simulate: real Meta API, fake table, no arm): OK. scan_table, 3 sorts on the fake table (one missed grip, retried and sorted), write_report, finish "Sorted 3 parts: 2 good (3 levers), 1 bad (2 levers). 1 missed grip retried successfully, 0 failures/unresolved." Token use per call between 830 and 3160 in, 46 to 495 out.
- Human ran `uv run python -m muse_qc.agent --execute` (2026-10-29 ~22:23, power supply on, hand on the switch): FIRST AGENT RUN WITH THE ARM. Fresh empty reference taken before (`find_parts --no-sam` gave candidates: 0). Three parts on the table; scan_table returned 2 parts (the third was a candidate that SAM did not confirm, shown yellow in the image, so it was not offered to the agent). Sort 1 at (478.7, 344.7): "inspect: levers 3 verdict good", contact L2 R2 -45.82/-49.34, "dropped into the good box". Sort 2 and its retry at (703.4, 259.6) failed with "request failed: new_dm_serial failed: open serial port /dev/ttyACM0 failed: Device or resource busy". Agent: "2 consecutive failures, stopping", exit 1. Cause: pick_test ran inside the agent process and the serial port stayed held; fixed by running each sort as a child process.
- Human ran `uv run python -m muse_qc.agent --execute` (2026-10-29 ~22:41 to 22:44, power supply on, hand on the switch): child-process fix confirmed, three sorts in a row without a busy port, write_report, finish "Sorted 3 parts: 0 good, 3 bad, 0 failures. Table empty, report written.", exit 0. All three parts were good parts but were counted "levers 0 verdict bad" and dropped into the bad box. One part stayed on the table: the last scan listed it as an unconfirmed candidate at (713, 238) (SAM did not confirm it, it lay tilted), so the agent saw "no parts found". Contacts per sort: L2 R2 at -46.12/-49.56, -49.27/-52.64, -45.97/-49.43, all "dropped into the bad box".
- The human had the saved images reviewed (AI-assisted review): two wrong-part picks (highest confidence instead of the target) and one prompt miss under the evening lamp. count_20261029_224139 and count_20261029_224236: find_part picked the highest-confidence box, another part further right (lying on its side); the gripped target sat top centre in front of the fingers. count_20261029_224330: the right part (box (325,186,355,217)), three levers visible in the crop, but SAM returned 0 boxes for "orange bar" (the levers look brown). Human prompt test on recordings/armcam_20261029_224328.png: "orange bar" 0, "orange lever" 0, "orange stripe" 0, "lever" 0, "brown bar" 3. Human re-count of every inspection image of the day with "brown bar" (orange -> brown): 134008 0->3, 134656 3->3, 204601 3->3, 211720 3->3, 212016 2->2 (the bad part), 222330 3->3, 224139 0->0 and 224236 0->0 (wrong part picked), 224330 0->3, 225250/225253/225255/225257 0->3 (the prompt test runs), 225300 3->3. "brown bar" was never lower than "orange bar" and kept the bad part at 2.
- Human ran the arm-camera height test (2026-10-29 ~23:21 to 23:28, power supply on, hand on the switch): one good part flat at table (506.4, 338.1) (find_parts: confirmed 0.71). `hover_move --table-mm 506.4 338.1 --hover 100 --hold-s 30 --execute` with `camera_arm` during the hold, then the same with `--hover 80`, then `--table-mm 506.4 298.1 --hover 80` (40 mm towards the base). `count_clips` on the three images: part boxes (327,181,355,212) conf 0.80, (326,197,357,231) conf 0.81, (322,125,358,165) conf 0.82, all "levers: 3". Lowering the hover from 100 to 80 mm made the part only about 1 px larger, because the arm camera looks forward; the inspection height stays at 100 mm. The human saw joint 4 swing during hover_move (default gains).
- Human ran `uv run python -m muse_qc.hold_test --table-mm 506.4 338.1 --hold-s 20 --gains vel2 --hold-mode mit-ff` (dry run, 2026-10-30 ~00:11): all waypoints pass, mit kp 120 120 120 18 18 18, kd 8 8 8 2 2 2, tau scale 1.0, gravity torque at hover J1 -0.000, J2 +7.474, J3 -5.564, J4 -0.236, J5 -0.000, J6 -0.000 N*m.
- Human ran the same with `--execute` (2026-10-30 ~00:13, power supply on, hand on the switch), then `--hold-mode posvel --execute` at the same point (~00:15). Recorded output:
```text
mit-ff:
hold commanded deg: -3.84 -102.77 -95.99 +83.23 -0.00 +0.01
hold measured deg: -3.44 -102.85 -94.89 +82.67 +0.34 -0.32
hold measured tip mm: x=+293.6 y=-19.0 z=+95.1
motion finished, arm holds the settle pose
joint 1: mean err +0.40 deg, p2p 0.00 deg, rms 0.40 deg
joint 2: mean err -0.08 deg, p2p 0.26 deg, rms 0.08 deg
joint 3: mean err +1.10 deg, p2p 0.17 deg, rms 1.10 deg
joint 4: mean err -0.56 deg, p2p 0.39 deg, rms 0.56 deg, swing n/a
joint 5: mean err +0.34 deg, p2p 0.00 deg, rms 0.34 deg, swing n/a
joint 6: mean err -0.32 deg, p2p 0.00 deg, rms 0.32 deg
```
```text
posvel:
hold commanded deg: -3.84 -102.77 -95.99 +83.23 -0.00 +0.01
hold measured deg: -3.86 -102.78 -96.01 +82.89 -0.01 -0.01
hold measured tip mm: x=+297.9 y=-20.1 z=+99.6
motion finished, arm holds the settle pose
joint 1: mean err -0.01 deg, p2p 0.04 deg, rms 0.02 deg
joint 2: mean err +0.00 deg, p2p 1.31 deg, rms 0.11 deg
joint 3: mean err +0.00 deg, p2p 1.20 deg, rms 0.10 deg
joint 4: mean err -0.01 deg, p2p 1.11 deg, rms 0.20 deg, swing 0.8 Hz
joint 5: mean err -0.01 deg, p2p 0.00 deg, rms 0.01 deg, swing n/a
joint 6: mean err -0.02 deg, p2p 0.15 deg, rms 0.03 deg
```
Result in one paragraph: with MIT plus gravity feed-forward joint 4 did not swing (p2p 0.39 deg, standard deviation 0.023 deg from the CSV, no swing frequency found) while the position-velocity hold swung at 0.8 Hz with p2p 1.11 deg; joints 2 and 3 were also much calmer (p2p 0.26/0.17 vs 1.31/1.20 deg). The human watched both runs: MIT hold looked steady, posvel swung again. Cost: MIT has a steady offset because the gripper is heavier than the SDK model (no integral term): joint 3 +1.10 deg, the tip about 5 mm lower (z 95.1 vs 99.6 mm). Next step: per-joint torque scale to remove the offset before MIT is used for picking. CSV files: recordings/hold_mit-ff_20261030_001308.csv and recordings/hold_posvel_20261030_001520.csv (not in the repo).
- Human work on the hold: the human asked for a fix of the joint 4 swing, chose to test the SDK gravity compensation, had the research answer and the code change reviewed before the commit (the review checked that the mode switch happens only before enable and that the gravity model is the DM URDF of the SDK config), ran the dry run first so the gravity torques were checked before anything moved, and ran both hardware runs with a hand on the power supply switch.
- Human ran `hold_test --table-mm 506.4 338.1 --hold-s 20 --gains vel2 --hold-mode mit-ff --tau-scale 1.0 1.0 1.4 1.0 1.0 1.0` (2026-10-30 ~00:41, dry run then --execute, power supply on, hand on the switch). Dry run: joint 3 gravity torque -5.564 N*m, scaled -7.790 N*m. Run:
```text
hold commanded deg: -3.84 -102.77 -95.99 +83.23 -0.00 +0.01
hold measured deg: -3.55 -102.96 -95.77 +82.67 +0.34 -0.32
hold measured tip mm: x=+297.8 y=-19.8 z=+97.8
hold tip z offset: -2.0 mm
joint 2: mean err -0.19 deg, p2p 0.42 deg
joint 3: mean err +0.23 deg, p2p 0.00 deg
joint 4: mean err -0.56 deg, p2p 0.33 deg, swing n/a
```
- Human ran the same with `--tau-scale 1.0 1.05 1.45 0.25 1.0 1.0 --execute` (~00:44): the hold was close (joint 2 -0.02, joint 3 -0.10, joint 4 -0.08 deg mean err, joint 4 swing n/a), but on the way back to the rest pose joint 4 slowly sagged; the tracking guard stopped the run: "tracking error on joint 4, holding", REFUSED {"reason": "tracking-error", "value": "joint 4", "limit": "5.0 deg for 10 cycles"} after 5203 of 6229 cycles, then the motors were disabled as designed. Conclusion: the joint 4 model error depends on the pose (the gripper's centre of mass differs from the SDK model), so a single joint 4 factor below 1.0 only fits the hover pose.
- Human ran the same with `--tau-scale 1.0 1.05 1.45 1.0 1.0 1.0 --execute` (~00:47):
```text
hold commanded deg: -3.84 -102.77 -95.99 +83.23 -0.00 +0.01
hold measured deg: -3.55 -102.78 -96.09 +82.70 +0.34 -0.34
hold measured tip mm: x=+299.4 y=-19.9 z=+99.8
hold tip z offset: +0.0 mm
motion finished, arm holds the settle pose
joint 1: mean err +0.29 deg, p2p 0.11 deg, rms 0.29 deg
joint 2: mean err -0.02 deg, p2p 0.42 deg, rms 0.02 deg
joint 3: mean err -0.10 deg, p2p 0.24 deg, rms 0.10 deg
joint 4: mean err -0.53 deg, p2p 0.33 deg, rms 0.54 deg, swing n/a
joint 5: mean err +0.38 deg, p2p 0.44 deg, rms 0.40 deg, swing n/a
joint 6: mean err -0.35 deg, p2p 0.04 deg, rms 0.35 deg
```
Result: tip z offset 0.0 mm at the hover pose, no joint 4 swing, whole path without a guard stop. These values become the default.
- Human work on the torque scale: the human ran the dry run before every hardware run, kept a hand on the power supply switch, and had the per-joint factors computed from the measured offsets (AI-assisted) between the runs.
- Human ran (2026-10-30 ~01:02, power supply off, USB only, arm at rest) `uv run python -m muse_qc.camera_arm` with a good part held between the fingertips by hand, then `uv run python -m muse_qc.count_clips --image recordings/armcam_20261030_010239.png`:
part: box=(323, 268, 362, 312) confidence=0.75
levers: 3
verdict: good
- Human ran the same with a bad part (~01:05), `count_clips --image recordings/armcam_20261030_010524.png`:
part: box=(323, 268, 360, 316) confidence=0.59
levers: 2
verdict: bad
- Decision: the part is inspected in the gripper after the lift, because the camera sits on the gripper and the held part always appears at the same image position (no alignment, no arm shadow). An unverified part goes to the bad box.
- Human ran (2026-10-30 ~08:45, power supply on, hand on the switch) `uv run python -m muse_qc.pick_test --table-mm 503.5 344.3 --inspect --arm-mode mit-ff --execute` after `find_parts` found the good part at table_mm=(503.5, 344.3) sam=confirmed 0.73:
```text
contact: gripper commanded deg -40.57 measured deg -43.83 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
inspect: levers 3 verdict good
dropped into the good box
end readings: arm [-3.11 -0.14 -2.79 +5.23 +1.48 -1.80] gripper -126.28 deg
```
The record gave part_offset_px 4.6 (part in the gripper at the expected image position). First run of the in-gripper inspection and of the MIT arm mode in a pick; no tracking error.
- Human ran `pick_test --table-mm 503.5 344.3 --no-grip --hold-s 15 --arm-mode mit-ff --execute` (~08:49): "holding at tip z 4.7 mm", human measured 8 mm tip to table; end readings arm [-3.11 -0.27 -2.76 +5.17 +1.45 -1.78].
- Human ran the same without --arm-mode (posvel, ~08:51): "holding at tip z 4.7 mm", human measured 2 mm; end readings arm [-3.14 +0.14 -1.63 +2.02 +1.41 -1.93].
- Human observation: in mit-ff joint 4 stands visibly too low at rest. Conclusion: MIT leaves a pose-dependent steady-state error (model mismatch of gripper and camera), so a slow position trim was added.
- Human found after a PC restart that the arm port moved to /dev/ttyACM1 while the SDK channel stayed /dev/ttyACM0 (all joints read 0.0); `muse_qc.preflight` showed "sdk-port: FAIL"; plugging the arm in before the gripper Pico fixed it.
- Human ran (2026-10-30 ~09:07, power supply on, hand on the switch, posvel) `uv run python -m muse_qc.pick_test --table-mm 511.2 348.4 --inspect --execute` on the bad part (2 levers) after `find_parts` found it at table_mm=(511.2, 348.4) sam=confirmed 0.71:
```text
contact: gripper commanded deg -43.42 measured deg -46.76 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
inspect: levers 2 verdict bad
dropped into the bad box
end readings: arm [-3.16 +0.14 -1.72 +2.66 +1.48 -1.93] gripper -126.30 deg
```
First real bad-part sort with the in-gripper inspection.
- Human ran (2026-10-30 ~09:15, power supply on, hand on the switch, part removed) `uv run python -m muse_qc.pick_test --table-mm 503.5 344.3 --no-grip --hold-s 15 --arm-mode mit-ff --execute` with the position trim (commit 2844f5d):
```text
holding at tip z 4.7 mm, measure the tip height now
mit trim deg: +0.30 +0.49 +2.24 +0.04 +0.34 -0.06
end readings: arm [-3.09 +0.19 -1.93 +2.83 +1.48 -1.80] gripper -87.46 deg
```
Human measured 1 mm tip to table (before the trim: 8 mm in mit-ff, 2 mm in posvel). Rest readings before the run: joint 4 +1.89 deg; after: +2.83 (before the trim the end reading was +5.17). Decision: mit-ff becomes the default arm mode.
- Human ran (2026-10-30 ~09:18, same conditions) the same command with --grip-z-mm 8:
```text
holding at tip z 7.7 mm, measure the tip height now
mit trim deg: +0.02 +0.45 +2.39 +0.34 +0.37 -0.04
end readings: arm [-3.27 +0.27 -2.42 +4.08 +1.48 -1.56] gripper -87.46 deg
```
Human checked the tip height by eye and judged it right ("perfect", no ruler value). Decision: default grip z 8 mm for 3 mm more margin to the table.
- Human ran (2026-10-30 ~09:30, power supply on, hand on the switch) `uv run python -m muse_qc.pick_test --table-mm 504.2 349.5 --inspect --execute` with the new defaults (mit-ff, grip z 8 mm, uncommitted working tree) after `find_parts` found the good part at table_mm=(504.2, 349.5) sam=confirmed 0.81:
```text
contact: gripper commanded deg -43.57 measured deg -46.94 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
inspect: levers 3 verdict good
dropped into the good box
mit trim deg: -0.08 +0.31 +2.62 +0.30 +0.09 -0.04
end readings: arm [-2.90 +0.01 -2.59 +4.49 +1.45 -1.54] gripper -126.28 deg
```
The record gave part_offset_px 4.1. The part held at grip z 8 mm in mit-ff (on 2026-10-29 a grip at 8 mm in posvel had slipped), so the 8 mm default stays.
- Human ran (2026-10-30 ~09:50, power supply on, hand on the switch, part removed) `uv run python -m muse_qc.pick_test --table-mm 504.2 349.5 --no-grip --hold-s 3 --speed-deg-s 30 --accel-deg-s2 60 --execute`: "segments at 100 Hz: hover 452 lower 384 lift 264 put-down 380 raise 180 return 502 (total 2162)", "mit trim deg: +0.15 +0.31 +2.44 +0.43 -0.00 -0.12", end readings arm [-3.01 +0.03 -1.98 +2.92 +1.45 -4.43]; human: "perfekt".
- Human ran the same with --speed-deg-s 50 --accel-deg-s2 100 (~09:51): "segments at 100 Hz: hover 312 lower 384 lift 264 put-down 380 raise 180 return 362 (total 1882)", "mit trim deg: +0.15 +0.29 +2.41 +0.18 +0.04 -0.13", end readings arm [-3.22 +0.03 -2.42 +4.08 +1.45 -4.43]; no tracking error.
- Human took a new empty reference (`find_parts --save-empty`, ~09:53) because find_parts reported "too many changes"; then `find_parts` gave exactly one candidate: part 0 table_mm=(508.9, 340.4) sam=confirmed 0.72.
- Human ran (~09:54) `uv run python -m muse_qc.pick_test --table-mm 508.9 340.4 --inspect --speed-deg-s 50 --accel-deg-s2 100 --execute`:
```text
segments at 100 Hz: hover 307 lower 384 lift 264 to-box 326 release 2 return 675 (total 1958)
contact: gripper commanded deg -44.02 measured deg -47.68 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
inspect: levers 3 verdict good
dropped into the good box
mit trim deg: -0.11 +0.38 +2.61 +0.18 -0.02 -0.13
end readings: arm [-1.93 +0.80 -1.93 +2.74 +1.43 -4.40] gripper -126.30 deg
```
The part held during the fast move. Human remarks: gripper slow, slow approach jerky. Decision: 50/100 default, faster gripper, 20 mm/s approach.
- Human ran (~10:02) the --no-grip run at 508.9 340.4 once with --arm-mode posvel and once in mit-ff (15/30) to compare the slow approach; both ran to "holding at tip z 7.7 mm" without errors; human saw hardly any difference, so the jerk is not specific to MIT.
- Human ran (2026-10-30 ~10:26, power supply on, hand on the switch) `uv run python -m muse_qc.pick_test --table-mm 503.2 346.6 --inspect --execute` with the new defaults (commit 39412b4) after `find_parts` found the good part at table_mm=(503.2, 346.6) sam=confirmed 0.75:
segments at 100 Hz: hover 310 lower 259 lift 214 to-box 328 release 2 return 675 (total 1788)
contact: gripper commanded deg -44.37 measured deg -47.94 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
inspect: levers 0 verdict bad
dropped into the bad box
mit trim deg: +0.16 +0.54 +2.48 +0.32 +0.32 -0.09
end readings: arm [-2.50 +0.10 -2.68 +4.64 +1.50 -4.43] gripper -126.30 deg
Mechanics fine (faster gripper, 20 mm/s approach, part_offset_px 1.1), but the good part was judged bad: the crop shows 3 levers, SAM returned none.
- Human ran `count_clips --image recordings/armcam_20261030_102620.png --expect-target` twice and once with --prompt "orange bar": each time "part: box=(323, 267, 362, 311) confidence=0.71", "levers: 0", "verdict: bad". Conclusion: deterministic miss on this image, so a retry (fresh camera frame first, then other crop sizes and prompts) was added for the 0-lever case only.
- Human ran (2026-10-30 ~10:28, power supply off) `uv run python -m muse_qc.count_retry --image recordings/armcam_20261030_102620.png`:
attempt 1: prompt brown bar scale 8 margin 15 levers 1
levers: 1
verdict: bad
Conclusion: SAM is not fully deterministic on this image (0 three times, now 1), so the retry now covers every count other than 3, and good needs two retry counts of exactly 3. AI-assisted review of the crop image count_20261030_112838_crop.png from the CLI run (human agreed): all 3 levers visible to the eye, SAM boxed only the middle one, the whole image is blurred (text on the part unreadable), so the settle before the photo was raised to 2 s.
- Human ran (2026-10-30 ~10:40, power supply off) `uv run python -m muse_qc.count_retry --image recordings/armcam_20261030_102620.png` with the new rule (commit 97398a8):
attempt 1: prompt brown bar scale 8 margin 15 levers 0
attempt 2: prompt brown bar scale 6 margin 15 levers 0
attempt 3: prompt brown bar scale 10 margin 25 levers 0
attempt 4: prompt orange bar scale 8 margin 15 levers 0
levers: 0
verdict: bad
Early stop after 4 attempts as designed; crop and prompt variants cannot rescue the blurred image, only a sharper fresh frame can (the CLI never grabs one).
- Human ran (~10:41) `find_parts`: part 4 table_mm=(506.5, 327.7) sam=confirmed 0.86 (good part), then (~10:42, power supply on, hand on the switch) `uv run python -m muse_qc.pick_test --table-mm 506.5 327.7 --inspect --execute` (2 s settle, commit 97398a8):
segments at 100 Hz: hover 301 lower 259 lift 214 to-box 323 release 2 return 675 (total 1774)
contact: gripper commanded deg -44.52 measured deg -47.94 contacts L2 R2
contacts L1 L2 R2 (lifting)
part lifted, holding
inspect: levers 3 verdict good
dropped into the good box
mit trim deg: -0.13 +0.60 +2.47 +0.23 -0.06 -0.15
end readings: arm [-2.13 +0.32 -2.74 +4.89 +1.43 -4.40] gripper -126.28 deg
First count gave 3, no retry.
- Human ran (~10:43) `find_parts`: part 7 table_mm=(505.3, 336.1) sam=confirmed 0.58 (bad part), then (~10:44, power supply on, hand on the switch) `uv run python -m muse_qc.pick_test --table-mm 505.3 336.1 --inspect --execute`:
segments at 100 Hz: hover 305 lower 259 lift 214 to-box 325 release 2 return 675 (total 1780)
contact: gripper commanded deg -44.52 measured deg -47.94 contacts L2 R2
contacts L1 L2 R1 R2 (lifting)
part lifted, holding
inspect retry: 5 attempts
inspect: levers 2 verdict bad
dropped into the bad box
mit trim deg: +0.16 +0.55 +2.41 +0.24 -0.01 -0.13
end readings: arm [-2.46 +0.21 -2.96 +5.61 +1.37 -4.43] gripper -126.28 deg
The retry ran on hardware (fresh frame plus variants, early stop after 5 attempts) and the bad part stayed bad. File times in recordings/ (human listed them): photo armcam_20261030_114342.png at 11:43:42, first count 11:43:45, fresh frame armcam_20261030_114346.png at 11:43:46, retry counts at 11:43:49, :51, :53, :55, so about 13 s from photo to verdict; the good part took about 3 s (photo 11:41:54, count 11:41:57). Each count is about 2 s; the 60 s inspect timeout leaves ample margin. Joint 4 rest reading creeps up (+4.64, +4.89, +5.06 at the start of the last three runs, +5.61 at the end); watched, no action yet.
- Human ran (2026-10-30 ~10:57, power supply on) `uv run python -m muse_qc.agent --execute` with 2 parts in the feed area: "2 parts found", sorted part 0 (levers 3, good box), then both rescans failed with "scan failed: too many changes: light or arm moved? take a new empty reference with --save-empty" and the agent stopped after 2 consecutive failures ("agent: 2 consecutive failures, stopping").
- Human ran (~11:02, nothing changed on the table) `uv run python -m muse_qc.find_parts --no-sam`: "too many changes: light or arm moved? take a new empty reference with --save-empty". The boxes were empty, so the extra blobs come from the gripper and arm in the arm zone or from light.
