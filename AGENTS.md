# muse-qc-cell: project rules

Goal: AI quality inspection cell for injection molded parts. A reBot B601-DM arm finds parts,
inspects them against a rule in plain language, and sorts them into good and reject bins.
Built during the Meta Global AI Developer Hackathon (Oct 26 to Nov 4, 2026).

## Hard rules
- Never read, print, copy or commit the file .env or any API key.
- Never move the arm or run any hardware code. Hardware tests are started by the human only.
- Never install or import anything from outside this folder without asking first.
- Commit only when the human asks.

## Stack
- Python 3.11, uv, one virtual environment in .venv
- Hardware only through motorbridge and reBotArm_control_py (official Seeed packages, MIT)
- AI only through Meta Model API: Muse Spark (muse-spark-1.3) and SAM 3.1. Code is built with Muse Code.
- API key only from .env (MODEL_API_KEY), loaded by the program, never printed

## Design
- muse_qc/safety.py is the only path to the motors. The model never sends raw motor commands.
- muse_qc/safety.py is inside every tool that moves the arm; the agent cannot call or bypass it.
- Tools take part IDs and bin names only, never coordinates.
- Every safety limit has pytest tests, including targets that must be rejected.
- Small modules, one file per job, max 300 lines per module (tests excluded).
- One tool per file.

## Agent loop
- The agent loop has a round limit and a cost limit per run.
- Progress is saved after each round.
- The loop pauses instead of crashing, and resume is possible.

## Evidence and report
- Findings cite the image ID as evidence.
- The final report needs user confirmation.

## Tests
- The ground-truth test set lives outside the agent folder.
- Tests run offline with a fake arm and a fake model.

## Hardware and secrets
- Hardware commands are started only by the user, never by Muse Code.
- Muse Code never reads .env and never prints keys.

## Layout
muse_qc/ (safety.py, arm.py, usb_devices.py, sdk_config.py, preflight.py,
diagnose_joints.py, gripper.py (planned), camera_top.py, table_markers.py,
make_markers.py, detect_markers.py, table_coords.py, checkerboard.py, capture_calibration.py, calibrate_camera.py, camera_hand.py (planned), vision_sam.py (planned),
inspection.py (planned), tools/ (planned), agent.py (planned)), tests/, docs/

## Ground rules for Muse Code
- Build only what is asked, then stop and report files changed, what was run and limitations.
- One tool per file with definitions() and run().
- The agent system prompt is generic.
- Devices are found by USB ID, never by port order.
- Nothing is pushed to the public repository before the user has reviewed it.

## Commands
- Tests: uv run pytest

## Language
- Everything that goes into the repository is written in English: code, comments, docstrings, log and error messages, commit messages, README and docs.
- The human writes in German. Reply to the human in German in the chat, but never write German into files or commits.

## Session log
- At the end of every session, append a short entry to docs/MUSE_CODE_LOG.md (date, model, what was built, in two to three lines).
- Every hardware test run by the human is logged in docs/DEVLOG.md as a 'Human ran ...' entry with date, conditions (for example arm power off, USB only) and result, in the same or the next commit. Never write device serial numbers.
- Do not commit.

## No legacy system

Nothing from any earlier private robot-arm system may be used, copied,
imported, linked, referenced or mentioned in this repository, in commit
messages, in the README, in the demo video, or in any prompt given to
Muse Code. This includes workspaces, ROS packages, MCP servers, scripts,
configuration, calibration data and any knowledge taken from earlier
projects.

Allowed sources only:
- the official Seeed SDK (reBotArm_control_py) and motorbridge, both
  listed in docs/THIRD_PARTY.md;
- the author's public gripper project, declared openly in README and
  docs/THIRD_PARTY.md;
- everything newly written in this repository during the hackathon.

Muse Code: never mention, search for, open or ask about any earlier
system. If a file or message appears to refer to one, stop and report it
to the user instead of using it. Test fixtures and examples must use
neutral names.
