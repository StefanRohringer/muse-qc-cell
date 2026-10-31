# Results

Verified numbers only: every row below is a successful human run from
docs/DEVLOG.md (started by the human, recorded output never edited).

| Metric | Value | How measured |
| --- | --- | --- |
| Table camera calibration | 26 images used, RMS reprojection error 0.199 px | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.calibrate_camera` |
| Table marker sides | 39.3 to 39.9 mm (expected 40 mm) | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.table_coords` with calibration |
| Test cube check | 3 mm off in x and 12 mm in y (cube at 300/300 mm read at about 297/312 mm) | Human run 2026-10-28 in docs/DEVLOG.md: taped 25 mm cube re-read from the grid image |
| First motion at 100 Hz | 590 waypoints, cycle work 2.04 ms mean / 2.21 ms max, cycle period 10.09 ms mean | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.first_move --execute` |
| Lever count on the test pair | Good part 3 levers = good, bad part 2 levers = bad (short remnant dropped) | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.count_clips --image` on both images |
| Inspection report | 2 parts inspected: 1 good, 1 bad | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.inspection.report` |
| 10-part sorting run 1 | 10/10 gripped on the first try, 0 missed grips; 4 good, 6 bad sorted; 7 min 20 s; about 0.22 USD (14 Muse Spark calls) | Human run 2026-10-30 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --chat` |
| 10-part sorting run 3 | 10 of 10 sorted correctly: 9 in the agent run plus 1 single pick; boxes checked by the human; cost not measured | Human run 2026-10-31 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --chat` |
| Lever direction from the slot axis | 7 of 7 within about 10 deg of the photo (before: 3 of 7) | Human run 2026-10-31 in docs/DEVLOG.md: `uv run python -m muse_qc.find_parts --no-sam` read against the table photo |
