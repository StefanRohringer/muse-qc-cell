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
- AI only through Meta Model API: Muse Spark, Muse Wonder, SAM 3.1, Muse Voice Transcribe
- API key only from .env (MODEL_API_KEY), loaded by the program, never printed

## Design
- safety.py is the only path to the motors. The model never sends raw motor commands.
- Tools take part IDs and bin names only, never coordinates.
- Every safety limit has pytest tests, including targets that must be rejected.
- Small modules, one file per job, max 300 lines per file.

## Layout
safety.py, arm.py, camera_top.py, camera_hand.py, vision_sam.py, inspect.py,
tools.py, agent.py, planner.py, voice.py, web/, tests/, docs/

## Commands
- Tests: uv run pytest

## Language
- Everything that goes into the repository is written in English: code, comments, docstrings, log and error messages, commit messages, README and docs.
- The human writes in German. Reply to the human in German in the chat, but never write German into files or commits.

## Session log
- At the end of every session, append a short entry to docs/MUSE_CODE_LOG.md (date, model, what was built, in two to three lines).
- Do not commit.
