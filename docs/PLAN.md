# Plan: muse-qc-cell

## Goal

Inspect 10 scattered injection-molded parts, each with 3 clips.
The reBot B601-DM arm finds the parts, inspects each one against a
plain-language rule, and sorts them into "good" (exactly 3 clips
present) and "bad" (anything else) by visual inspection.

## Setup

- Table overview: Trust USB camera sees the whole table. SAM 3.1 finds parts.
- Close-up: Orbbec Gemini 305 on the arm looks at one part at a time
  and counts its clips.
- Gripper: the author's own public gripper with tactile sensors reports
  grip and contact. Contact triggers STOP.
- Agent: Muse Spark 1.3. The agent chooses every tool itself.
- Code: Muse Code builds the code.

## Run flow

1. The table camera sees all parts.
2. The agent picks one part.
3. The arm camera counts the clips on that part.
4. The arm grips the part; the gripper confirms grip and contact.
5. The arm places the part in the good bin (3 clips) or the bad bin.
6. The run ends with an HTML report.
7. The user confirms the report (y/n).
8. Uncertain count: the agent asks the user instead of guessing.

## Schedule

| Day | Work | Done when | Fallback |
| --- | ---- | --------- | -------- |
| Wed 28.10 | safety.py + table camera with ArUco | Safety limits enforced with tests; table image with ArUco markers acquired | Table camera without ArUco, fixed table coordinates |
| Thu 29.10 | First safe move + SAM counting (Checkpoint 1) | Arm moves once inside safety limits; SAM counts clips on a test image | Manual bounding boxes instead of SAM |
| Fri 30.10 | Arm camera, gripper.py read-only, agent loop | Close-up images captured; gripper.py reads grip/contact only; agent loop runs end to end without hardware | Scripted pick order instead of agent choice |
| Sat 31.10 | Grip and sort, two full runs in a row (Checkpoint 2); after that only bug fixes | Two consecutive full sort runs pass; feature freeze, bug fixes only | Sort without grip confirmation |
| Sun 1.11 | Test series with 20 parts | 20 parts inspected and sorted, results logged | 10 parts if hardware time runs short |
| Mon 2.11 | Video (3 min) | 3-minute demo video recorded and reviewed | Screen recording of stills plus narration |
| Tue 3.11 | README and X post | README complete; X post drafted | README only, X post after submission |
| Wed 4.11 | Submission by 12:00 CET | Submission sent before 12:00 CET | Smallest working run submitted as-is |

## Cut list

Cut in this order if time runs short:

1. Muse Voice
2. Muse Wonder
3. FastAPI
4. MCP server
5. Second evidence check

Never cut:

- safety.py
- Counting
- Agent loop
- Test series
- Video
