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
- reBot arm with the pre-existing touch-sensing gripper (declared in docs/THIRD_PARTY.md).
- Models: muse-spark-1.3 (agent), sam-3.1 (detection), Muse Code (builds the code).

## Schedule

| Day | Work |
| --- | ---- |
| Tue 27.10 | Rules, plan and safety.py done, table camera with ArUco markers and table coordinates |
| Wed 28.10 | Table camera calibration with a checkerboard |
| Thu 29.10 | First safe move, SAM 3.1 finds parts and checks levers (done 28.10, early); first pick and sorting into the boxes, first agent runs (done) |
| Fri 30.10 | Inspect in the gripper, gripper turned to the lever direction, Muse Spark agent continuous run, live page, order by chat (done) |
| Sat 31.10 | Test series with all 10 parts |
| Sun 1.11 | Results and fixes from the test series |
| Mon 2.11 | Demo video |
| Tue 3.11 | README and post |
| Wed 4.11 | Submission |
