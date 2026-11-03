# Lever count evaluation

Evaluates the arm camera close-ups: the lever count and the good/bad
verdict of each inspected part against labels written by hand.

## What is evaluated

Each `eval/records/count_<stamp>.json` record holds the counted levers
and the verdict (`good` means exactly the expected count, 3 levers for
the connectors in this demo). The evaluation joins those records with
`eval/labels.csv` by stamp and reports:

- number of labeled parts
- lever count accuracy (exact match of counted vs. true levers)
- verdict accuracy (`good` = exactly the expected count)
- confusion matrix good/bad (true vs. predicted)
- precision and recall for "bad"
- seconds per part (mean, p95) from the time between consecutive
  records in a run, when timestamps are available

`eval/records/` holds the 10 JSON records named in `eval/labels.csv`,
copied unchanged from `recordings/` (JSON only, no images), so the
evaluation runs without the git-ignored `recordings/` folder.

Records named in the labels but missing from `eval/records` are
listed as missing; nothing is invented for them.

## How the labels were made

The human looked at each arm camera crop (`count_<stamp>_crop.png`)
and wrote down the true number of closed levers in `eval/labels.csv`.
The boxes of run 3 were checked by hand.

## How to run it

`eval/labels.csv` holds the human labels for the 10 parts (header
plus 10 rows), one row per inspected part:

```csv
record,true_levers
20261031_014235,3
```

The `record` column is the `YYYYmmdd_HHMMSS` stamp of the
`count_<stamp>.json` file name. Then, from the project root:

```bash
uv run python -m muse_qc.inspection.evaluate --labels eval/labels.csv --recordings eval/records
uv run python -m muse_qc.inspection.evaluate --labels eval/labels.csv --recordings eval/records --out eval/results.md
```

The `--out` file (`eval/results.md`) holds a small markdown table plus
the command and the date. Without `--out` the report is only printed.
No camera, no arm, no API key, no network.
