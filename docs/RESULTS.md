# Results

Verified numbers only: every row below is a successful human run from
docs/DEVLOG.md (started by the human, recorded output never edited).

| Metric | Value | How measured |
| --- | --- | --- |
| Table camera calibration | 26 images used, RMS reprojection error 0.199 px | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.vision.calibrate_camera` |
| Table marker sides | 39.3 to 39.9 mm (expected 40 mm) | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.vision.table_coords` with calibration |
| Test cube check | 3 mm off in x and 12 mm in y (cube at 300/300 mm read at about 297/312 mm) | Human run 2026-10-28 in docs/DEVLOG.md: taped 25 mm cube re-read from the grid image |
| First motion at 100 Hz | 590 waypoints, cycle work 2.04 ms mean / 2.21 ms max, cycle period 10.09 ms mean | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.bringup.first_move --execute` |
| Lever count on the test pair | Good part 3 levers = good, bad part 2 levers = bad (short remnant dropped) | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.inspection.count_clips --image` on both images |
| Inspection report | 2 parts inspected: 1 good, 1 bad | Human run 2026-10-28 in docs/DEVLOG.md: `uv run python -m muse_qc.inspection.report` |
| 10-part sorting run 1 | 10/10 gripped on the first try, 0 missed grips; 4 good, 6 bad sorted; 7 min 20 s; 14 Muse Spark calls | Human run 2026-10-30 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --chat` |
| 10-part sorting run 3 | 10 of 10 sorted correctly: 9 in the agent run plus 1 single pick; boxes checked by the human | Human run 2026-10-31 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --chat` |
| Lever direction from the slot axis | 7 of 7 within about 10 deg of the photo (before: 3 of 7) | Human run 2026-10-31 in docs/DEVLOG.md: `uv run python -m muse_qc.vision.find_parts --no-sam` read against the table photo |
| 4-part arm session run | 4 of 4 sorted correctly (3 good, 1 bad); boxes checked by hand | Human run 2026-11-01 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --plan-once --scan-at-station --arm-session` |
| 10-part arm session run | 10/10 gripped on the first try, 10/10 sorted correctly (7 good, 3 bad); 4 min 39 s first scan to report; 5 Muse Spark calls | Human run 2026-11-02 in docs/DEVLOG.md: `uv run python -u -m muse_qc.agent --execute --chat --plan-once --scan-at-station --arm-session` |
| Lever count evaluation | 10 labeled parts, lever count accuracy 100 %, verdict accuracy 100 % | Offline, 2026-10-31 in eval/results.md: `python -m muse_qc.inspection.evaluate --labels eval/labels.csv --recordings recordings` |
| Spoken agent order | 2 parts sorted good, each grip on the first try; sort heard as short, Muse Spark still read it as a sort order; boxes checked by hand | Human run 2026-11-02 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --chat --plan-once --scan-at-station --arm-session --voice` |
| Long run with disturbances | 34 parts sorted in one spoken order (29 good, 4 bad, 1 unverified to the bad box), 33 of 33 lever counts correct, no bad part in the good box; 24 model calls (12 scan_table, 12 sort_in_order), 5 missed grips and 1 moved-part deviation with rescan and replan, parts added and moved by hand during the run; last part left after 2 consecutive failures; boxes checked by hand | Human run 2026-11-02 in docs/DEVLOG.md: `uv run python -m muse_qc.agent --execute --chat --plan-once --scan-at-station --arm-session --voice`, transcript recordings/agent_20261102_165153.json |
