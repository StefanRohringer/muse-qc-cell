# Lever count evaluation

Evaluates the arm camera close-ups: the lever count and the good/bad
verdict of each inspected part against labels written by hand.

## What is evaluated

Each `recordings/count_<stamp>.json` record holds the counted levers
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

Records named in the labels but missing from the recordings folder are
listed as missing; nothing is invented for them.

## How the labels were made

The human looked at each arm camera crop (`count_<stamp>_crop.png`)
and wrote down the true number of closed levers in `eval/labels.csv`.
The boxes of run 3 were checked by hand.

## How to run it

The labels file starts with the header only; the human fills in one
row per inspected part:

```csv
record,true_levers
20261031_014235,3
```

The `record` column is the `YYYYmmdd_HHMMSS` stamp of the
`count_<stamp>.json` file name. Then, from the project root:

```bash
uv run python -m muse_qc.inspection.evaluate --labels eval/labels.csv --recordings recordings
uv run python -m muse_qc.inspection.evaluate --labels eval/labels.csv --recordings recordings --out eval/results.md
```

The `--out` file (`eval/results.md`) holds a small markdown table plus
the command and the date. Without `--out` the report is only printed.
No camera, no arm, no API key, no network.
