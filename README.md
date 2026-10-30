# muse-qc-cell

AI quality inspection cell for injection-molded parts, built during the
Meta Global AI Developer Hackathon (Oct 26 to Nov 4, 2026) with Muse Code
(powered by Muse Spark `muse-spark-1.3`, see `docs/MUSE_CODE_LOG.md`), SAM 3.1
and Muse Spark through the Meta Model API.

A reBot B601-DM arm finds small connector parts on a table, picks each
one, inspects it against a rule in plain language and sorts it into a
good box or a bad box. A Muse Spark agent decides the next step; our code
executes the tools and enforces every safety check. Rule in plain
language: good = all 3 levers present; bad = a lever missing.

## Why this project

I come from plastic injection molding: 18 years of hands-on
manufacturing experience, founder of an injection molding company that
I successfully sold (exit), and Bachelor Professional of Plastics and
Rubber Production and Management (CCI). Many companies still let people
do monotonous visual inspection and quality control of molded parts and
assembled molded parts by hand.

Physical AI helps from two sides. The company gains efficiency, a higher
quality rate and a better OEE (overall equipment effectiveness). The
staff are freed from monotonous checking and can be trained for other
tasks in the company. A company that uses Physical AI and develops its
own people can raise its profit, because robots combined with today's
cameras, sensors and AI add real value. One example: a plant that stops
production on the weekend could keep an inspection cell like this one
running unattended.

With this hackathon entry I want to show how a low-cost desktop arm
(the reBot Arm B601-DM by Seeed Studio) can be brought toward real
production with little effort. I built the repository from scratch,
and all code was written by Meta Muse Spark (muse-spark-1.3) through
Muse Code in the terminal, within the 150 USD of Meta credits given to
participants. The system is not perfect, but it works end to end on
real hardware.

## Safety layers

Every layer below is in the code and covered by offline tests:

1. Plan check before motion: every waypoint of a move is checked by
   `safety.py` (workspace box, minimum tool height above the table,
   joint limits, joint 6 cable limit, speed and step size per cycle)
   before the motors are enabled; one refused waypoint stops the whole
   move before it starts.
2. Check in every control cycle: during the move each command is checked
   again against fresh joint readings (stale or invalid readings stop
   the move).
3. Human gate: `--execute` needs a typed "yes"; without it nothing moves.
   Only the human starts hardware runs, with a hand on the power supply
   switch.
4. Slow by design: motor speed limits at 10 % (arm) and 25 % (gripper)
   of the configured values, ramps with limited acceleration, 20 mm/s
   final approach.
5. Tracking guard: if a joint falls too far behind its command, the move
   is aborted and the arm ramps back safely.
6. Touch sensing: unexpected contact while lowering stops the descent,
   the arm climbs back and returns; the part counts as held only with
   contact on both fingers.
7. Grasp floor: only the grasp waypoints may go down to 4 mm above the
   table, every other waypoint keeps at least 30 mm.
8. Agent limits: the model passes part ids only, never coordinates or
   motor commands; positions come from our own scan; hard limits on
   model calls, sorts, tries per part and consecutive failures; Ctrl+C
   stops the run safely.
9. Devices: found by USB ID, never by port order; a preflight check
   reports when another program already uses the arm.

## Not certified

muse-qc-cell is a hackathon research prototype. Its safety layers are
software checks only. They have not been assessed or certified by any
testing body (for example TÜV), and the cell does not comply with the
EU Machinery Regulation (EU) 2023/1230 or the robot safety standards
(for example ISO 10218). There is no certified emergency stop, no
guarding and no risk assessment. Do not use it in production or near
people without a proper safety concept, risk assessment and
certification.

## How it works

1. Scan: the table camera finds four ArUco markers and maps pixels to
   table millimetres with a checkerboard calibration. An empty-table
   reference shows what changed; SAM 3.1 confirms each candidate on a
   zoomed crop, and the lever direction is read from the image
   (`find_parts.py`, `part_angle.py`).
2. Pick: the arm moves above the part, turns the gripper to the lever
   direction, lowers, closes until the touch sensors report contact on
   both sides and lifts (`pick_test.py`, `grip_yaw.py`, `gripper.py`).
3. Inspect: with the part held between the fingers, the arm camera takes
   a close-up; SAM 3.1 finds the part, the crop is enlarged and the
   levers are counted (`inspect_step.py`, `count_clips.py`,
   `count_retry.py`). Exactly 3 levers = good, otherwise bad.
4. Sort: the part is dropped into the good or bad box (`drop_plan.py`),
   and the arm returns to rest.
5. Agent: Muse Spark calls `scan_table`, `inspect_and_sort(part_id)`,
   `write_report` and `finish` (`agent.py`, `agent_tools.py`). It only
   passes part ids from our own latest scan, never coordinates. Orders
   can be given in plain words (`--task`, `--chat`), and a live page in
   the browser shows the run (`live_page.py`).

```text
order in plain words (terminal)
  -> Muse Spark agent
  -> scan_table: table camera + empty reference + SAM 3.1 "grey object"
  -> part ids with table mm and lever direction
  -> inspect_and_sort(id): hover, turn gripper, lower, touch-sensed grip, lift
  -> arm camera close-up -> SAM 3.1 part + "brown bar" levers
  -> 3 levers = GOOD, otherwise BAD
  -> drop into the good or bad box, return to rest
  -> next id, rescan when the list is used up
  -> JSON records + HTML report + live page
```

Safety: every arm waypoint is checked against `safety.py` (workspace,
joint, cable, speed, step, freshness and STOP checks) before it is sent,
in the dry run and again in every control cycle. `--execute` asks for a
typed "yes". The agent cannot bypass a tool check. Devices are found by
USB ID, never by fixed `/dev` names. Only the human starts hardware runs,
with a hand on the power supply switch.

## What is new in this hackathon

Everything in `muse_qc/`, `tests/`, `docs/` and `calibration/` was
written during the hackathon. Declared prior work (not part of the
submission): the Touch-Sensing Fin-Ray Gripper hardware and firmware are
pre-existing, not built during the hackathon (listed by Seeed under "DIY
Soft Finger"). New in this hackathon: `muse_qc/gripper.py`, which reads
the gripper sensors over USB, and everything else in this repository.
See `docs/THIRD_PARTY.md`. The official Seeed SDK (`reBotArm_control_py`)
and `motorbridge` are dependencies, never copied in.

## Status with evidence (human runs only)

Every row is a human run (started by the human, recorded output never
edited); details in `docs/DEVLOG.md` and `docs/SETUP.md`.

| Step | State | Evidence |
| --- | --- | --- |
| Table camera, markers, calibration (rms 0.199 px) | works | SETUP 9-14, DEVLOG 2026-10-27/28 |
| Model API check, SAM 3.1 on the table | works | SETUP 15-16, DEVLOG 2026-10-28 |
| First motion at 100 Hz, every waypoint checked | works | SETUP 18, DEVLOG 2026-10-28 |
| Gripper touch sensors, grip with touch feedback | works | SETUP 24-26, DEVLOG 2026-10-28 |
| Find parts (empty reference + SAM confirm) | works | SETUP 29, DEVLOG 2026-10-29 |
| Pick and sort into the good and bad box | works | SETUP 28 and 31, DEVLOG 2026-10-29 |
| Inspect in the gripper after the lift | works | SETUP 32, DEVLOG 2026-10-30 |
| Gripper turned to the lever direction | works | SETUP 34-35, DEVLOG 2026-10-30 |
| Muse Spark agent, continuous run, 3 parts, 3 correct | works | SETUP 33 and 36, DEVLOG 2026-10-30 |
| Order by chat, live page | works | SETUP 36, DEVLOG 2026-10-30 |
| Test series with all 10 parts | planned | - |

Known limits: SAM 3.1 does not always confirm a real part on the table
crop (a second phrase is tried, the agent rescans); SAM sometimes counts
0 levers on a bad part with 2 levers (the verdict "bad" is still
correct, retries cost time); the lever direction can jump on some parts.
Only missing levers are tested so far, open or broken levers are not.

## Quick start

Prereqs: Python 3.11, `uv`, the Seeed SDK checked out next to this repo as
`../seeed-sdk` (see `docs/SETUP.md`, without it `uv sync` fails), and a
`.env` file with `MODEL_API_KEY` for the camera/model steps only.

```bash
uv sync
uv run pytest                                   # offline suite, no hardware
uv run python -m muse_qc.preflight              # human only: USB + SDK checks
uv run python -m muse_qc.model_check            # human only: Model API check
uv run python -m muse_qc.find_parts --save-empty  # human only: empty table reference
uv run python -m muse_qc.find_parts             # human only: scan the table
uv run python -m muse_qc.agent                  # human only: real Muse Spark, fake table, no arm
uv run python -m muse_qc.agent --execute --chat --task "sort all parts"  # human only: real run
firefox recordings/live.html &                  # watch the run
```

`uv run pytest` runs the full offline suite (fake arm, fake camera, fake
model; no hardware, camera, or network). Every hardware or network step is
human-only: never run by CI or an agent.

## Files

| Path | Purpose |
| --- | --- |
| `muse_qc/agent.py`, `agent_tools.py`, `agent_io.py`, `agent_parse.py`, `agent_chat.py` | Muse Spark agent loop, tools, chat orders |
| `muse_qc/live_page.py` | Live status page for the agent run |
| `muse_qc/find_parts.py`, `find_skip.py`, `parts_image.py`, `part_angle.py` | Table scan, skip zones, annotated image, lever direction |
| `muse_qc/pick_test.py`, `pick_loop.py`, `pick_plan.py`, `pick_plan_check.py`, `pick_args.py`, `pick_mit.py`, `pick_inspect.py` | Pick, inspect and sort one part |
| `muse_qc/grip_yaw.py`, `grip_speed.py`, `drop_plan.py` | Gripper turn, gripper speed, box drop plan |
| `muse_qc/inspect_step.py`, `count_clips.py`, `count_retry.py` | Lever count on the arm camera image |
| `muse_qc/inspection/` | JSON records, drawing, HTML report |
| `muse_qc/gripper.py`, `gripper_stream.py`, `gripper_watch.py`, `gripper_grip.py`, `gripper_motor.py` | Gripper sensors and motor |
| `muse_qc/safety.py`, `safety_limits.py` | Pre-move checks |
| `muse_qc/target_pose.py`, `arm_calibration.py`, `xy_correction.py`, `waypoint_stream.py` | Targets, tip correction, waypoint stream |
| `muse_qc/first_move.py`, `first_move_plan.py`, `hover_move.py`, `hold_test.py`, `mit_hold.py`, `arm_gains.py` | Motion tests, hold, gains |
| `muse_qc/arm.py`, `fk_check.py`, `diagnose_joints.py`, `cycle_timing.py` | Arm readout, kinematics and timing checks |
| `muse_qc/preflight.py`, `usb_devices.py`, `sdk_config.py` | Device checks |
| `muse_qc/camera_top.py`, `camera_arm.py` | Capture |
| `muse_qc/table_markers.py`, `make_markers.py`, `detect_markers.py`, `table_coords.py` | Markers and pixel-to-mm frame |
| `muse_qc/checkerboard.py`, `capture_calibration.py`, `calibrate_camera.py` | Camera calibration |
| `muse_qc/model_check.py`, `vision_sam.py` | Model API check, SAM box detection |
| `tests/` | Offline suite mirroring the modules |
| `docs/SETUP.md` | Hand-run manual, sections 1-36 |
| `docs/DEVLOG.md` | Dated human-run evidence log |
| `docs/PLAN.md`, `docs/RESULTS.md` | Goal and schedule, verified numbers |
| `calibration/table_camera.json` | Table camera calibration |

Recorded images in `recordings/` are never committed.
