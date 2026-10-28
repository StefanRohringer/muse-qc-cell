# muse-qc-cell

AI quality inspection cell for injection-molded parts, built during the
Meta Global AI Developer Hackathon (Oct 26 to Nov 4, 2026) with Muse Code
(powered by Muse Spark muse-spark-1.3, see docs/MUSE_CODE_LOG.md) and
SAM 3.1 through the Meta Model API.

A reBot B601-DM arm is being built to find small connector parts on a
table, check them against a rule in plain language, and sort them into a
good bin and a bad bin. Rule in plain language: good = all 3 levers
present; bad = a lever missing or torn out.

## How it works

1. The table camera finds four ArUco markers and maps pixels to table
   millimetres (`camera_top.py`, `detect_markers.py`, `table_coords.py`),
   undistorted with a checkerboard calibration (`checkerboard.py`,
   `capture_calibration.py`, `calibrate_camera.py`).
2. SAM 3.1 finds candidate parts on the table view (`vision_sam.py`) and
   reports each box center in table coordinates.
3. The arm camera takes a close-up between the gripper fingers
   (`camera_arm.py`); the lever counter finds the part with "small part",
   enlarges the crop 8x, and counts "orange bar" boxes (`count_clips.py`).
   Exactly 3 levers = good, otherwise bad.
4. Each inspection is saved as a JSON record plus annotated images
   (`inspection/record.py`, `inspection/draw.py`) and collected into one
   offline HTML report (`inspection/report.py`).
5. Every arm motion is validated waypoint by waypoint against `safety.py`
   before anything is sent (`first_move.py`, `first_move_plan.py`).

```text
table camera
  -> SAM 3.1 "small part"
  -> part positions in table mm
  -> arm moves above the part (planned)
  -> arm camera image
  -> SAM 3.1 "small part"
  -> crop + enlarge 8x
  -> SAM 3.1 "orange bar"
  -> height filter (drops remnants of torn-out levers)
  -> 3 levers = GOOD, otherwise BAD
  -> JSON record + HTML report
  -> sort into bin (planned)
```

Safety: `safety.py` holds workspace, joint, cable, speed, step, freshness,
and STOP checks. The dry run validates the whole plan without enabling the
motors; `--execute` asks for typed confirmation and re-checks every control
cycle. Devices are found by USB ID, never by fixed `/dev` names.

Models: SAM 3.1 (sam-3.1) is used at run time through the Meta Model API
(`vision_sam.py`, `count_clips.py`). Muse Spark (muse-spark-1.3) powers
Muse Code, which wrote the code; its use as the decision maker in the
agent loop is planned. A quick Muse Spark vision test miscounted levers
on the test pair, so counting uses SAM.

## What is new in this hackathon

Everything in `muse_qc/`, `tests/`, `docs/`, and `calibration/` was written
during the hackathon. Declared prior work (not part of the submission):
the Touch-Sensing Fin-Ray Gripper hardware and firmware are pre-existing,
not built during the hackathon (listed by Seeed under "DIY Soft Finger").
New in this hackathon: the future `muse_qc/gripper.py` sensor reader, which
will read the sensors over USB, and everything else in this repository.
See `docs/THIRD_PARTY.md`. The official Seeed SDK (`reBotArm_control_py`)
and `motorbridge` are dependencies, never copied in.

## Status with evidence (human runs only)

Every "works" row below is a human run (started by the human, recorded
output never edited).

| Step | State | Evidence |
| --- | --- | --- |
| Preflight USB + SDK-port checks | works | SETUP 8, DEVLOG 2026-10-27 |
| Table camera, 1920x1080, fixed exposure | works | SETUP 9, DEVLOG 2026-10-27 |
| Table markers, 4 detected, 1.465 px/mm | works | SETUP 10-11, DEVLOG 2026-10-27 |
| Pixel-to-mm mapping, sides 39.3 to 39.9 mm | works | SETUP 12, DEVLOG 2026-10-28 |
| Test cube, 3 mm off in x and 12 mm in y | works | SETUP 12, DEVLOG 2026-10-28 |
| Calibration, 26 images, rms 0.199 px | works | SETUP 13-14, DEVLOG 2026-10-28 |
| Model API check, 9 models, both listed | works | SETUP 15, DEVLOG 2026-10-28 |
| SAM "small part", cube + 2 connectors | works | SETUP 16, DEVLOG 2026-10-28 |
| Forward-kinematics tip, tool-axis fix | tip within 9 mm horizontally and 13 mm vertically of a hand measurement | SETUP 17, DEVLOG 2026-10-28 |
| First motion, 100 Hz, 590 waypoints | works | SETUP 18, DEVLOG 2026-10-28 |
| Arm camera, 640x480, USB ID + interface | works | SETUP 19, DEVLOG 2026-10-28 |
| Lever count, 3 = good, 2 = bad | works | SETUP 20, DEVLOG 2026-10-28 |
| HTML report, 2 parts, 1 good, 1 bad, UTC | works | SETUP 21, DEVLOG 2026-10-28 |
| Arm moves above part, grip, sort into bin | planned | - |
| Gripper readout, hand camera, agent loop | planned | - |
| 20-part series, demo video | planned | - |

Currently only missing levers are detected. Open or broken levers are not tested yet.

Known limits: table mapping still carries about 12 mm y error on the taped
cube check; "small part" also matches non-part objects (the arm-camera step
decides).

The full evidence is in docs/DEVLOG.md and the hand-run manual in
docs/SETUP.md.

## Quick start

Prereqs: Python 3.11, `uv`, the Seeed SDK checked out next to this repo as
`../seeed-sdk` (see `docs/SETUP.md`, without it `uv sync` fails), and a
`.env` file with `MODEL_API_KEY` for the camera/model steps only.

```bash
uv sync
uv run pytest
uv run python -m muse_qc.preflight          # human only: USB + SDK checks
uv run python -m muse_qc.camera_top        # human only: one table photo
uv run python -m muse_qc.detect_markers     # human only: four markers
uv run python -m muse_qc.table_coords       # human only: pixels to mm
uv run python -m muse_qc.model_check        # human only: Model API check
uv run python -m muse_qc.vision_sam "small part"          # human only
uv run python -m muse_qc.camera_arm         # human only: arm close-up
uv run python -m muse_qc.count_clips --image recordings/armcam_<timestamp>.png  # human only
uv run python -m muse_qc.inspection.report --recordings recordings --out recordings/report.html  # offline
uv run python -m muse_qc.first_move         # human only: dry run (no motion)
```

`uv run pytest` runs the full offline suite (fake arm, fake camera, fake
model; no hardware, camera, or network). Every hardware or network step is
human-only: never run by CI or an agent.

## Files

| Path | Purpose |
| --- | --- |
| `muse_qc/safety.py`, `safety_limits.py` | Pre-move checks |
| `muse_qc/arm.py` | Read-only joint access |
| `muse_qc/preflight.py`, `usb_devices.py`, `sdk_config.py` | Device checks |
| `muse_qc/camera_top.py`, `camera_arm.py` | Capture |
| `muse_qc/table_markers.py`, `make_markers.py` | Marker constants, print sheet |
| `muse_qc/detect_markers.py`, `table_coords.py` | Detection, pixel-to-mm frame |
| `muse_qc/checkerboard.py`, `capture_calibration.py` | Print board, take photos |
| `muse_qc/calibrate_camera.py` | Calibration computation |
| `muse_qc/model_check.py` | Model API check |
| `muse_qc/vision_sam.py` | SAM box detection |
| `muse_qc/count_clips.py` | Lever count verdict |
| `muse_qc/inspection/` | JSON records, drawing, HTML report |
| `muse_qc/first_move.py`, `first_move_plan.py` | Motion program and plan |
| `muse_qc/cycle_timing.py`, `fk_check.py` | Timing helper, kinematics check |
| `muse_qc/diagnose_joints.py` | Joint feedback diagnosis |
| `tests/` | Offline suite mirroring the modules |
| `docs/SETUP.md` | Hand-run manual, sections 1-21 |
| `docs/DEVLOG.md` | Dated human-run evidence log |
| `docs/PLAN.md`, `docs/RESULTS.md` | Goal and schedule, verified numbers |
| `calibration/table_camera.json` | Table camera calibration |

## Limits

Table accuracy is good enough to find parts, not to grasp blindly (the arm
camera refines before any grasp); grip/sort, the agent loop, and the series
run are future work. Recorded images in `recordings/` are never committed.
