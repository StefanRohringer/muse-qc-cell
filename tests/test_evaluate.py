"""Tests for muse_qc/inspection/evaluate.py.

Offline only: records and labels are synthetic files in tmp_path,
never real recordings.
"""

from __future__ import annotations

import json

from muse_qc.inspection.evaluate import evaluate, load_labels, main


def _record(stamp, levers, minute, expected=3):
    return {
        "timestamp": f"2026-10-31T00:{minute:02d}:00+00:00",
        "source_image": f"recordings/armcam_{stamp}.png",
        "full_image": f"recordings/count_{stamp}.png",
        "crop_image": f"recordings/count_{stamp}_crop.png",
        "part_box": {"x1": 1, "y1": 2, "x2": 3, "y2": 4},
        "part_confidence": 0.9,
        "lever_prompt": "brown bar",
        "kept_boxes": [],
        "dropped_boxes": [],
        "levers": levers,
        "verdict": "good" if levers == expected else "bad",
        "expected_levers": expected,
    }


def _setup(recordings, labels_path, entries):
    recordings.mkdir(parents=True, exist_ok=True)
    stamps = ["20261031_000100", "20261031_000200",
              "20261031_000300", "20261031_000400"]
    levers = [3, 2, 3, 1]
    for stamp, count, minute in zip(stamps, levers, range(1, 5)):
        (recordings / f"count_{stamp}.json").write_text(
            json.dumps(_record(stamp, count, minute)), encoding="utf-8")
    with open(labels_path, "w", encoding="utf-8") as handle:
        handle.write("record,true_levers\n")
        for stamp, true in zip(stamps, entries):
            handle.write(f"{stamp},{true}\n")
    return stamps


def test_scores_counts_and_verdicts(tmp_path):
    recordings = tmp_path / "recordings"
    labels_path = tmp_path / "labels.csv"
    # Truth: good, bad, bad (counted 3 but only 2 closed), good.
    # Predicted: good, bad, good, bad.
    _setup(recordings, labels_path, [3, 2, 2, 0])
    labels = load_labels(labels_path)
    from muse_qc.inspection.evaluate import load_records
    result = evaluate(labels, load_records(recordings))
    assert result["labeled"] == 4
    assert result["matched"] == 4
    assert result["missing"] == []
    assert result["lever_accuracy"] == 0.5
    assert result["verdict_accuracy"] == 0.75
    assert result["matrix"] == {"good_good": 1, "good_bad": 0,
                               "bad_good": 1, "bad_bad": 2}
    assert result["precision_bad"] == 1.0
    assert result["recall_bad"] == 2 / 3
    timing = result["seconds_per_part"]
    assert timing["count"] == 3
    assert timing["mean"] == 60.0
    assert timing["p95"] == 60.0
    assert timing["runs"] == 1
    assert timing["skipped"] == 0


def test_long_gap_starts_new_run(tmp_path):
    recordings = tmp_path / "recordings"
    recordings.mkdir(parents=True, exist_ok=True)
    stamps = ["20261031_000100", "20261031_000200",
              "20261031_001000", "20261031_001100"]
    moments = ["2026-10-31T00:01:00+00:00", "2026-10-31T00:02:00+00:00",
               "2026-10-31T00:10:00+00:00", "2026-10-31T00:11:00+00:00"]
    labels_path = tmp_path / "labels.csv"
    with open(labels_path, "w", encoding="utf-8") as handle:
        handle.write("record,true_levers\n")
        for stamp, moment in zip(stamps, moments):
            record = _record(stamp, 3, 1)
            record["timestamp"] = moment
            (recordings / f"count_{stamp}.json").write_text(
                json.dumps(record), encoding="utf-8")
            handle.write(f"{stamp},3\n")
    from muse_qc.inspection.evaluate import format_text, load_records
    result = evaluate(load_labels(labels_path), load_records(recordings))
    timing = result["seconds_per_part"]
    assert timing["count"] == 2
    assert timing["mean"] == 60.0
    assert timing["p95"] == 60.0
    assert timing["runs"] == 2
    assert timing["skipped"] == 1
    assert ("seconds per part: mean 60.0 s, p95 60.0 s "
            "(n=2, 2 runs, 1 gap skipped)") in format_text(result)


def test_missing_records_are_listed_not_invented(tmp_path):
    recordings = tmp_path / "recordings"
    recordings.mkdir(parents=True, exist_ok=True)
    (recordings / "count_20261031_000100.json").write_text(
        json.dumps(_record("20261031_000100", 3, 1)), encoding="utf-8")
    labels_path = tmp_path / "labels.csv"
    labels_path.write_text("record,true_levers\n"
                           "20261031_000100,3\n"
                           "20261031_000900,2\n", encoding="utf-8")
    from muse_qc.inspection.evaluate import load_records
    result = evaluate(load_labels(labels_path), load_records(recordings))
    assert result["labeled"] == 2
    assert result["matched"] == 1
    assert result["missing"] == ["20261031_000900"]
    assert result["lever_accuracy"] == 1.0


def test_main_writes_report_only_with_out(tmp_path, capsys):
    recordings = tmp_path / "recordings"
    labels_path = tmp_path / "labels.csv"
    _setup(recordings, labels_path, [3, 2, 3, 1])
    out_path = tmp_path / "results.md"
    assert main(["--labels", str(labels_path),
                 "--recordings", str(recordings),
                 "--out", str(out_path)]) == 0
    text = out_path.read_text(encoding="utf-8")
    assert "| Lever count accuracy | 100.0 % |" in text
    assert "python -m muse_qc.inspection.evaluate" in text
    printed = capsys.readouterr().out
    assert "lever count accuracy" in printed

    assert main(["--labels", str(labels_path),
                 "--recordings", str(recordings)]) == 0
    assert not (tmp_path / "results2.md").exists()


def test_main_rejects_bad_header(tmp_path, capsys):
    labels_path = tmp_path / "labels.csv"
    labels_path.write_text("stamp,count\n20261031_000100,3\n",
                           encoding="utf-8")
    assert main(["--labels", str(labels_path),
                 "--recordings", str(tmp_path)]) == 1
    assert "bad header" in capsys.readouterr().out
