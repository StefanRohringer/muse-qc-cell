# muse-qc-cell

[![demo video](https://img.shields.io/badge/demo%20video-3%20min-red)](https://youtu.be/iBYUEyhkhq8) [![demo page](https://img.shields.io/badge/demo%20page-replay%20of%20a%20real%20run-blue)](https://stefanrohringer.github.io/muse-qc-cell/) [![tests](https://github.com/StefanRohringer/muse-qc-cell/actions/workflows/ci.yml/badge.svg)](https://github.com/StefanRohringer/muse-qc-cell/actions/workflows/ci.yml) [![license](https://img.shields.io/badge/license-MIT-green)](LICENSE)

Built by [Stefan Rohringer](https://github.com/StefanRohringer) (solo) for the Meta Global AI Developer Hackathon 2026 (26 Oct to 4 Nov 2026), Indie Developer track, theme "Push the Frontier" (robotics and manufacturing).

**Demo page (replay of a real run):** https://stefanrohringer.github.io/muse-qc-cell/

**Quick links:** [Hackathon demo video](https://youtu.be/iBYUEyhkhq8) · [Demo page (replay of a real run)](https://stefanrohringer.github.io/muse-qc-cell/) · [Results](#results-real-hardware-measured) · [Where Meta AI is used](#where-meta-ai-is-used) · [Safety](#safety) · [Known limits](#known-limits) · [Run it](#run-it)

**Docs:** [Results in detail](docs/RESULTS.md) · [Dev log, every human hardware run](docs/DEVLOG.md) · [Meta models](docs/META_AI.md) · [Muse Code log](docs/MUSE_CODE_LOG.md) · [Setup](docs/SETUP.md) · [Third-party code](docs/THIRD_PARTY.md) · [Lever count evaluation](eval/results.md) · [AGENTS.md (rules Muse Code follows)](AGENTS.md) · [Plan](docs/PLAN.md)

An AI quality inspection cell for injection-molded parts. A low-cost
reBot Arm B601-DM picks each part from the table, a Muse Spark agent
decides every step, SAM 3.1 counts the levers in a close-up, and the
part goes into the good box or the bad box. The order can be typed or spoken. Real hardware, no simulation.

**Hackathon demo video:** https://youtu.be/iBYUEyhkhq8

<p align="center">
<img src="docs/media/table_scan_10_parts.png" width="64%" alt="Table camera view: ten parts found on the table, each with its id and table millimeters, next to the two sorting boxes and the arm">
<img src="docs/media/inspect_good_3_levers.png" width="31%" alt="The arm camera close-up of a held part with the three levers boxed by SAM 3.1">
</p>

*Left: the table camera finds every part and gives the arm its position in table millimeters. Right: a part held between the touch-sensing fingers in front of the arm camera; SAM 3.1 finds the part, the crop is enlarged and the levers are counted: 3 levers = good.*

## The problem

I have worked in plastic injection molding for 18 years. I founded an
injection molding company and later sold it. In many plants, people
still check molded parts by eye, part after part. It is monotonous work,
and it often decides when a machine can run unattended.

The rule for a good part is usually simple and fits in one sentence. For
the connectors in this demo it is: **good = all 3 levers present,
otherwise bad.** This project shows that a desktop arm, two cameras and
Meta models can follow such a rule end to end, with little setup and on a
hackathon budget.

Who it is for: small and mid-sized molding plants that cannot pay for a
dedicated vision inspection system, for example to keep an inspection
cell running over a weekend.

## Results (real hardware, measured)

**Hackathon demo video:** https://youtu.be/iBYUEyhkhq8

All numbers come from human-run tests, recorded in `docs/DEVLOG.md`.

### Sorting runs

| Run | What changed | Gripped on the first try | Sorted correctly | Time |
| --- | --- | --- | --- | --- |
| 30 Oct, run 1 | first 10-part run, close-up right after the lift | 10/10 | not checked: 5 parts on the left half read 0 levers (light), so they were sorted bad without a count | 7 min 20 s, about 44 s per part, 14 Muse Spark calls |
| 31 Oct, run 3 | lever direction from the slot axis, rescan after every part, fixed inspection station | 9/9 | **10/10** (9 in this agent run plus 1 single pick just before it; boxes checked by hand) | about 6.5 min, about 43 s per part |
| 1 Nov, arm session | one session for all 4 parts, no return home between parts | 4/4 | **4/4** (boxes checked by hand) | about 27 s between counts (count to count, not a full cycle) |
| 2 Nov, 10 parts, arm session | one session for all 10 parts (`--plan-once --scan-at-station --arm-session`), new empty-table image after turning the lamp away | 10/10 | **10/10** (7 good, 3 bad; 3 bad placed on purpose; boxes checked by hand, all correct) | 4 min 39 s first scan to report, about 28 s per part (24 to 28 s count to count), 5 Muse Spark calls |
| 2 Nov, long run with disturbances | spoken order, 10 parts at the start, no sort limit; the human added parts and moved some just before the grip | 34 of 41 arm runs (5 missed grips on parts moved by hand, 2 failures on the last part) | **33/33** lever counts correct (29 good, 4 bad), 1 unverified part into the bad box, no bad part in the good box; boxes checked by hand | about 19 min for 34 parts (16:32 to 16:51), 25 to 27 s per part inside one arm session, 24 Muse Spark calls |

Run 2 (31 Oct, first run with the inspection station) gripped 10/10, but
several parts were gripped along the wrong axis because the lamp shadow
fooled the lever direction. That led to the slot axis fix used in run 3.

Missed-grip recovery was shown in a retest: a part slid away by hand
during the approach was missed, the arm lifted and returned to rest, and
the agent rescanned and sorted the part at its new place.

### Building blocks

| Check | Result |
| --- | --- |
| Table camera calibration | RMS reprojection error 0.199 px (26 images) |
| Lever direction from the table camera (7 parts vs. photo) | 7/7 within about 10 deg with the slot axis (before: 3/7) |
| Control loop | 100 Hz, cycle period 10.09 ms mean, 10.20 ms max |
| Grip | touch contact on both fingers before the lift, every grip of run 3 on the first try |
| Lever count evaluation (10 labeled parts, 31 Oct) | lever count and verdict 10/10 correct (eval/results.md) |
| Spoken order (Muse Voice Transcribe) | real agent run on 2 Nov sorted 2 parts; sort was heard as short and Muse Spark still read it as a sort order |
| Long run with disturbances (2 Nov) | 34 parts in one spoken order, 33 of 33 lever counts correct, no bad part in the good box; parts added and moved by hand during the run, 1 unverified part sorted into the bad box, last part left after 2 consecutive failures |
| Offline test suite | 1039 tests, no hardware, camera or network needed |

## How it works

```mermaid
flowchart TD
    A["Order in plain words, typed or spoken<br/>e.g. 'sort all parts on the table'"] --> B["Muse Spark agent<br/>muse-spark-1.3, tool calling"]
    V["spoken: Muse Voice Transcribe<br/>muse-voice-transcribe-1.0"] -.-> A
    B -->|scan_table| C["Table camera<br/>empty-table reference + SAM 3.1 'grey object'"]
    C --> D["Part IDs with table mm<br/>and lever direction"]
    D --> B
    B -->|"inspect_and_sort(part ID)<br/>or sort_in_order(plan, --plan-once)"| E["Safety check of every waypoint<br/>safety.py"]
    E --> F["Pick: hover, turn gripper,<br/>lower, touch-sensed grip, lift"]
    F --> G["Inspection station<br/>arm camera close-up"]
    G --> H["SAM 3.1: find the part,<br/>count the 'brown bar' levers"]
    H -->|3 levers| I["Good box"]
    H -->|fewer| J["Bad box"]
    I --> K["Rescan the table"]
    J --> K
    K --> B
    B -->|"write_report, finish"| L["JSON records,<br/>HTML report, live page"]
```

1. **Order:** I type or speak an order in plain words (with --voice, Muse Voice Transcribe turns the spoken order into text and I confirm it). The Muse Spark agent decides
   which tool to call next: `scan_table`, `inspect_and_sort(part_id)`,
   `sort_in_order(part_ids)`, `write_report` or `finish` (with
   --plan-once the model picks the order once and the parts run in
   one go).
2. **Scan:** the table camera finds four ArUco markers and maps pixels to
   table millimeters with a checkerboard calibration. An empty-table
   reference shows what changed, SAM 3.1 confirms each candidate, and the
   lever direction is read from the dark slot at one end of the part.
3. **Pick:** the arm moves above the part, turns the gripper so the fingers
   close across the levers, lowers, closes until the touch sensors report
   contact on both sides, and lifts.
4. **Inspect:** the arm carries the part to a fixed inspection station.
   The arm camera takes a close-up, SAM 3.1 finds the part, the crop is
   enlarged and SAM 3.1 counts the levers. Exactly 3 levers = good. A
   doubtful count is retried on fresh frames; a part is only good when
   the count is confirmed.
5. **Sort:** the part is dropped into the good or bad box, and the
   table is scanned again, so a part that moved is found again. A part
   that appears while planned parts are still waiting joins the running plan; a part that appears after the last planned part is picked up by a new plan.
   With `--arm-session` the arm goes from the box straight to the next part
   and returns to rest once at the end; without it (default) it returns
   to rest after every part.
6. **Report:** every step is saved as JSON, and a live page in the browser
   shows the table, each part's close-up, its lever count and its box.

### Where Meta AI is used

| Meta tool | Where | What it does |
| --- | --- | --- |
| Muse Spark (`muse-spark-1.3`, Meta Model API) | `muse_qc/agent/agent.py` | The agent: reads the order, picks the next tool, reacts to failures and rescans, writes the summary |
| SAM 3.1 (`sam-3.1`, Meta Model API) | `muse_qc/vision/find_parts.py`, `muse_qc/vision/vision_sam.py` | Confirms parts on the table image |
| SAM 3.1 | `muse_qc/inspection/count_clips.py`, `muse_qc/inspection/count_retry.py` | Finds the held part and counts its levers in the close-up |
| Muse Voice Transcribe (`muse-voice-transcribe-1.0`, Meta Model API) | `muse_qc/agent/voice_order.py` | Turns a spoken order into the chat order text |
| Muse Code (powered by Muse Spark) | the whole repository | Wrote all the code, logged task by task in `docs/MUSE_CODE_LOG.md` |

Details and request formats: `docs/META_AI.md`.

### A real agent run (excerpt)

From run 3 on 31 Oct (`docs/DEVLOG.md`, output shortened):

```text
agent -> scan_table()
tool: 8 parts found
grip yaw: lever 119.3 deg, offset -19.9 deg
inspect: levers 3 verdict good
tool: rescan found 7 parts
grip yaw: lever 102.3 deg, offset +9.9 deg
inspect: levers 2 verdict bad
...
tool: rescan found no parts, call scan_table
agent -> scan_table()
tool: no parts found
agent -> write_report()
agent finished: Sorted 9 parts: 6 good (3 levers), 3 bad (<3 levers), 0 failures/missed grips. Final rescan found no parts.
```

## Inspection images

Close-ups from the arm camera. SAM 3.1 boxes only closed levers; an open
lever does not count.

| Good | Bad | Bad |
| --- | --- | --- |
| ![Good part with 3 closed levers](docs/media/inspect_good_3_levers.png) | ![Bad part, right lever open](docs/media/inspect_bad_right_lever_open.png) | ![Bad part, middle lever open](docs/media/inspect_bad_middle_lever_open.png) |
| 3 closed levers: good | right lever open, 2 counted: bad | middle lever open, 2 counted: bad |

## Safety

Every layer below is in the code and covered by offline tests.

1. **Plan check before motion:** `safety.py` checks every waypoint of a
   move (workspace box, minimum tool height, joint limits, joint 6 cable
   limit, speed and step size) before the motors are enabled. One refused
   waypoint stops the whole move before it starts.
2. **Check in every control cycle:** during the move, each command is
   checked again against fresh joint readings.
3. **Human gate:** `--execute` needs a typed "yes". Only a human operator
   starts hardware runs, with a hand on the power supply switch.
   A spoken order only replaces the order text; the yes stays typed and the heard text must be confirmed.
4. **Limited speed:** motor speed caps, ramps with limited acceleration,
   20 mm/s on the final approach.
5. **Tracking guard:** if a joint falls too far behind its command, the
   move is aborted and the arm ramps back.
6. **Touch sensing:** unexpected contact while lowering stops the descent
   and the arm moves back up. A part counts as held only with contact on
   both fingers.
7. **Grasp floor:** only the grasp waypoints may go down to 4 mm above the
   table; every other waypoint stays at least 30 mm above it.
8. **Agent limits:** the model passes part IDs only, never coordinates or
   motor commands. Positions come from our own scan. Hard limits on model
   calls, tries per part and consecutive failures. Ctrl+C stops the
   run safely.
9. **Devices:** found by USB ID, never by port order.

### Not certified

muse-qc-cell is a hackathon research prototype. Its safety layers are
software checks only. They have not been assessed or certified by any
testing body (for example TÜV), and the cell does not comply with the EU
Machinery Regulation (EU) 2023/1230 or robot safety standards such as
ISO 10218. There is no certified emergency stop, no guarding and no risk
assessment. Do not use it in production or near people without a proper
safety concept, risk assessment and certification.

## What is new in this hackathon

I started this repository from scratch during the hackathon. Everything
in `muse_qc/`, `tests/`, `docs/`, `calibration/`, `site/`, `eval/` and
`scripts/` was written during the event, plus the `Dockerfile` and the `Makefile`.

Prior work, declared and not part of this submission: my Touch-Sensing
Fin-Ray Gripper (hardware and firmware). I built it before the hackathon,
and it is listed with a link to its repository in the "DIY Soft Finger" section of the official Seeed reBot-DevArm README (https://github.com/Seeed-Projects/reBot-DevArm#diy-soft-finger). New in this hackathon: `muse_qc/gripper/gripper.py`, which reads
the gripper sensors over USB. See `docs/THIRD_PARTY.md`.

## Quickstart

1. Clone this repository.
2. `make setup`: prepares `../seeed-sdk` and syncs the environment.
3. `make test`: runs the offline test suite, no hardware needed.
4. Optional, for the agent with the fake table: `cp .env.example .env`,
   add your key, then `make simulate`.

or with Docker: `make docker`

## Run it

Prerequisites: Linux, Python 3.11, `uv`, the Seeed SDK checked out next to
this repository as `../seeed-sdk` (see `docs/SETUP.md`; without it
`uv sync` fails), and a `.env` file with `MODEL_API_KEY` for the camera and
model steps.

`scripts/setup_sdk.sh` covers SETUP steps 1 to 3, so a fresh clone only
needs the script plus `uv sync` (step 4):

```bash
bash scripts/setup_sdk.sh                        # prepares ../seeed-sdk, idempotent
uv sync
uv run pytest                                     # offline suite, no hardware
uv run python -m muse_qc.checks.preflight                # human only: USB + SDK checks
uv run python -m muse_qc.checks.model_check              # human only: Model API check
uv run python -m muse_qc.vision.find_parts --save-empty  # human only: empty-table reference
uv run python -m muse_qc.agent                    # real Muse Spark, fake table, no arm
uv run python -m muse_qc.agent --execute --chat --task "sort all parts"  # human only: real run
uv run python -m muse_qc.agent --execute --chat --plan-once --scan-at-station --arm-session   # human only: 10-part session run
uv run python -m muse_qc.agent --execute --chat --plan-once --scan-at-station --arm-session --voice   # human only: spoken order
```

Without local setup, the same offline suite runs in Docker:

```bash
docker build -t muse-qc-cell .
docker run --rm muse-qc-cell
```

The live page is written to `recordings/live.html`; open it in a browser
to watch the run. Every hardware step is described in `docs/SETUP.md`
with its recorded output.

## Repository

| Path | What |
| --- | --- |
| `muse_qc/agent/` | Muse Spark agent loop, tools, chat orders, live page |
| `muse_qc/vision/` | cameras, markers, calibration, table scan and lever direction |
| `muse_qc/inspection/` | lever count, records, HTML report |
| `muse_qc/arm/` | arm access, safety checks, pose solving |
| `muse_qc/gripper/` | gripper sensors and motor |
| `muse_qc/pick/` | pick, inspection station, sort |
| `muse_qc/checks/` | USB, SDK config and Model API checks |
| `muse_qc/bringup/` | hand-run arm bringup scripts |
| `tests/` | offline test suite |
| `docs/SETUP.md` | every hand-run step with recorded output |
| `docs/DEVLOG.md` | dated log of every real hardware and API test |
| `docs/RESULTS.md` | verified numbers |
| `docs/META_AI.md` | how each Meta model is called |
| `docs/THIRD_PARTY.md` | third-party components and licenses |
| `docs/MUSE_CODE_LOG.md` | what Muse Code built in each task |
| `eval/` | hand-labeled lever count evaluation |
| `site/` | demo page |
| `scripts/` | SDK setup |
| `calibration/` | table camera calibration |

## Known limits

- Missing and open levers were seen in the tests; broken levers are not
  tested yet.
- SAM 3.1 does not always confirm a real part on the first try; a second
  phrase and the rescan catch most of them. An unconfirmed part is shown
  yellow and is not picked. Bright lamp light on the left made this worse;
  turning the lamp away and saving a new empty-table image helped.
- Light matters: a lamp shadow once turned the lever direction by 90 deg,
  and poor light gave 0 levers on good parts. The fixed inspection station
  and the slot axis fixed this in run 3.
- Without `--arm-session`, after every part the arm returns to rest, which costs about 7 s per part.
- A part within 15 mm of an already sorted position is not picked again
  while the scans still see a part there; once a scan sees that spot
  empty, the spot is forgotten, and a new chat order starts with a
  clean order memory.
- Spoken orders: the voice model sometimes mishears a word (sort as short); the heard text is shown and must be confirmed before it is used.
- A part that appears while the last planned part is being sorted starts a new plan and a new arm session, so the arm returns to rest once before it is sorted.

## Next

- A longer test series with known good and bad parts, including broken
  and open levers.
- Make spoken orders more robust (the voice model heard sort as short; a shorter fixed phrase or more keywords may help).
- Other parts with other rules, given in plain words.

## Credits

Built during the Meta Global AI Developer Hackathon (26 Oct to 4 Nov 2026).

- **Meta models:** Muse Spark 1.3, SAM 3.1 and Muse Voice Transcribe 1.0 through the Meta Model API; Muse Code (powered by Muse Spark) wrote the code.
- **Hardware:** reBot Arm B601-DM by Seeed Studio, Orbbec Gemini 305 on the
  arm, a USB webcam above the table, and my pre-existing Touch-Sensing
  Fin-Ray Gripper ([repository](https://github.com/StefanRohringer/Touch-Sensing-Fin-Ray-Gripper)).
- **Software (dependencies, not copied in):** Seeed `reBotArm_control_py`
  (MIT), `motorbridge` (MIT), Pinocchio (BSD-2-Clause), OpenCV
  (Apache-2.0), the `openai` Python client for the Meta Model API
  (Apache-2.0), NumPy (BSD-3-Clause), pySerial (BSD), pytest (MIT), uv,
  arecord (alsa-utils, called as a system tool for spoken orders, not bundled).
  Details and versions: `docs/THIRD_PARTY.md`.
- **Ideas:** the public Meta workshop repository
  `meta-models/legal-agent-workshop` (checks inside tools, agent loop with
  hard limits; ideas only, no code copied).

## License

MIT, see `LICENSE`. Third-party components: `docs/THIRD_PARTY.md`.
