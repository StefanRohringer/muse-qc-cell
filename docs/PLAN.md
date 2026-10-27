# Plan: muse-qc-cell

Context: built for the Meta AI Developer Hackathon (Oct 26 to Nov 4, 2026).
All work in this repository was built during the hackathon, except the
declared gripper hardware (see docs/THIRD_PARTY.md).

## Goal

10 scattered connectors with 3 levers each.
Good = all 3 levers present, intact and closed.
Bad = a lever missing, broken or open.
The reBot B601-DM sorts them into a good bin and a bad bin.

## Setup

- Table camera (overview), depth camera on the arm (close-up).
- reBot arm with touch-sensing gripper.
- Models: muse-spark-1.3 (agent), sam-3.1 (detection), Muse Code (builds the code).

## Schedule

| Day | Work |
| --- | ---- |
| Tue 27.10 | safety.py done |
| Wed 28.10 | Table camera overview |
| Thu 29.10 | Close-up counting |
| Fri 30.10 | Grip and sort |
| Sat 31.10 | Full runs |
| Sun 1.11 | Test series |
| Mon 2.11 | Demo video |
| Tue 3.11 | README and post |
| Wed 4.11 | Submission |
