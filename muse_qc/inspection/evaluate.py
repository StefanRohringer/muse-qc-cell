"""Evaluate the lever count against hand-written labels (offline).

Joins eval/labels.csv with recordings/count_<stamp>.json records by
stamp and reports lever count accuracy, verdict accuracy, the good/bad
confusion matrix, precision and recall for "bad", and seconds per part
from consecutive record timestamps when available.

Example:
    uv run python -m muse_qc.inspection.evaluate \\
        --labels eval/labels.csv --recordings recordings \\
        --out eval/results.md

Offline only: no camera, no arm, no API key, no network. Missing
records are listed, never invented.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from datetime import datetime
from pathlib import Path

STAMP_FORMAT = "%Y%m%d_%H%M%S"
DEFAULT_EXPECTED_LEVERS = 3


def parse_stamp(text: str) -> datetime:
    """Parse a YYYYmmdd_HHMMSS stamp, raising ValueError when invalid."""
    return datetime.strptime(text, STAMP_FORMAT)


def load_labels(path: str | Path) -> list[dict]:
    """Read labels.csv rows as {"record": stamp, "true_levers": int}."""
    rows: list[dict] = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != ["record", "true_levers"]:
            raise ValueError(
                f"bad header in {path}: {reader.fieldnames}"
            )
        for line in reader:
            stamp = (line.get("record") or "").strip()
            raw = (line.get("true_levers") or "").strip()
            if not stamp:
                continue
            parse_stamp(stamp)
            try:
                levers = int(raw)
            except ValueError:
                raise ValueError(f"bad lever count for {stamp}: {raw!r}")
            if levers < 0:
                raise ValueError(f"bad lever count for {stamp}: {raw!r}")
            rows.append({"record": stamp, "true_levers": levers})
    return rows


def load_records(recordings_dir: str | Path) -> dict[str, dict]:
    """Read count_<stamp>.json records keyed by stamp, skipping bad files."""
    target = Path(recordings_dir)
    records: dict[str, dict] = {}
    for path in sorted(target.glob("count_*.json")):
        stem = path.stem
        if not stem.startswith("count_"):
            continue
        stamp = stem[len("count_"):]
        try:
            parse_stamp(stamp)
            record = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            print(f"warning: skipping unreadable record {path.name}: {exc}")
            continue
        if isinstance(record, dict):
            records[stamp] = record
    return records


def _verdict(levers: int | None, expected: int, stored: object) -> str | None:
    """Return the record verdict, falling back to levers vs expected."""
    if stored in ("good", "bad"):
        return str(stored)
    if isinstance(levers, int):
        return "good" if levers == expected else "bad"
    return None


def evaluate(labels: list[dict], records: dict[str, dict]) -> dict:
    """Join labels with records and score counts, verdicts and timing."""
    matched: list[dict] = []
    missing: list[str] = []
    for row in labels:
        stamp = row["record"]
        record = records.get(stamp)
        if record is None:
            missing.append(stamp)
            continue
        levers = record.get("levers")
        expected = record.get("expected_levers")
        if not isinstance(expected, int):
            expected = DEFAULT_EXPECTED_LEVERS
        predicted = _verdict(
            levers if isinstance(levers, int) else None,
            expected, record.get("verdict"))
        if not isinstance(levers, int) or predicted is None:
            missing.append(stamp)
            continue
        truth = "good" if row["true_levers"] == expected else "bad"
        matched.append({
            "record": stamp,
            "true_levers": row["true_levers"],
            "levers": levers,
            "true_verdict": truth,
            "verdict": predicted,
            "timestamp": str(record.get("timestamp", "")),
        })
    lever_hits = sum(1 for item in matched
                     if item["levers"] == item["true_levers"])
    verdict_hits = sum(1 for item in matched
                       if item["verdict"] == item["true_verdict"])
    matrix = {"good_good": 0, "good_bad": 0, "bad_good": 0, "bad_bad": 0}
    for item in matched:
        matrix[f"{item['true_verdict']}_{item['verdict']}"] += 1
    tp = matrix["bad_bad"]
    fp = matrix["good_bad"]
    fn = matrix["bad_good"]
    precision = tp / (tp + fp) if (tp + fp) else None
    recall = tp / (tp + fn) if (tp + fn) else None
    moments: list[datetime] = []
    for item in matched:
        try:
            moments.append(datetime.fromisoformat(item["timestamp"]))
        except ValueError:
            continue
    moments.sort()
    gaps = [(later - earlier).total_seconds()
            for earlier, later in zip(moments, moments[1:])]
    gaps = [gap for gap in gaps if gap >= 0]
    seconds_per_part = None
    if gaps:
        ordered = sorted(gaps)
        rank = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
        seconds_per_part = {
            "count": len(gaps),
            "mean": sum(gaps) / len(gaps),
            "p95": ordered[rank],
        }
    total = len(matched)
    return {
        "labeled": len(labels),
        "matched": total,
        "missing": missing,
        "lever_accuracy": (lever_hits / total) if total else None,
        "verdict_accuracy": (verdict_hits / total) if total else None,
        "matrix": matrix,
        "precision_bad": precision,
        "recall_bad": recall,
        "seconds_per_part": seconds_per_part,
    }


def _percent(value: float | None) -> str:
    """Format a ratio as percent with one decimal, or n/a."""
    if value is None:
        return "n/a"
    return f"{100.0 * value:.1f} %"


def format_text(result: dict) -> str:
    """Format the evaluation result as plain text lines."""
    matrix = result["matrix"]
    lines = [
        f"labeled parts: {result['labeled']}",
        f"matched records: {result['matched']}",
        f"lever count accuracy: {_percent(result['lever_accuracy'])}",
        f"verdict accuracy: {_percent(result['verdict_accuracy'])}",
        "confusion (true vs predicted): "
        f"good/good {matrix['good_good']}, "
        f"good/bad {matrix['good_bad']}, "
        f"bad/good {matrix['bad_good']}, "
        f"bad/bad {matrix['bad_bad']}",
        f"precision (bad): {_percent(result['precision_bad'])}",
        f"recall (bad): {_percent(result['recall_bad'])}",
    ]
    timing = result["seconds_per_part"]
    if timing is None:
        lines.append("seconds per part: n/a (need 2+ timestamps)")
    else:
        lines.append(
            f"seconds per part: mean {timing['mean']:.1f} s, "
            f"p95 {timing['p95']:.1f} s (n={timing['count']})")
    if result["missing"]:
        lines.append("missing records: " + ", ".join(result["missing"]))
    else:
        lines.append("missing records: none")
    return "\n".join(lines)


def format_markdown(result: dict, command: str, date: str) -> str:
    """Format the evaluation result as a small markdown page."""
    matrix = result["matrix"]
    timing = result["seconds_per_part"]
    if timing is None:
        timing_text = "n/a (need 2+ timestamps)"
    else:
        timing_text = (f"mean {timing['mean']:.1f} s, "
                       f"p95 {timing['p95']:.1f} s (n={timing['count']})")
    missing = ", ".join(f"`{stamp}`" for stamp in result["missing"])
    return (
        "# Lever count evaluation\n"
        "\n"
        f"Command: `{command}`\n"
        f"Date: {date}\n"
        "\n"
        "| Metric | Value |\n"
        "| --- | --- |\n"
        f"| Labeled parts | {result['labeled']} |\n"
        f"| Matched records | {result['matched']} |\n"
        f"| Lever count accuracy | {_percent(result['lever_accuracy'])} |\n"
        f"| Verdict accuracy | {_percent(result['verdict_accuracy'])} |\n"
        f"| True good, predicted good | {matrix['good_good']} |\n"
        f"| True good, predicted bad | {matrix['good_bad']} |\n"
        f"| True bad, predicted good | {matrix['bad_good']} |\n"
        f"| True bad, predicted bad | {matrix['bad_bad']} |\n"
        f"| Precision (bad) | {_percent(result['precision_bad'])} |\n"
        f"| Recall (bad) | {_percent(result['recall_bad'])} |\n"
        f"| Seconds per part | {timing_text} |\n"
        f"| Missing records | {missing if missing else 'none'} |\n"
    )


def main(argv: list[str] | None = None) -> int:
    """Entry point: score labels against records, print and maybe write."""
    parser = argparse.ArgumentParser(
        description="Score lever counts against hand-written labels.")
    parser.add_argument("--labels", required=True,
                        help="path to labels.csv")
    parser.add_argument("--recordings", required=True,
                        help="folder with count_<stamp>.json records")
    parser.add_argument("--out", default=None,
                        help="write the markdown report to this file")
    args = parser.parse_args(argv)
    try:
        labels = load_labels(args.labels)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
    records = load_records(args.recordings)
    result = evaluate(labels, records)
    print(format_text(result))
    if args.out is not None:
        command = " ".join(["python -m muse_qc.inspection.evaluate",
                            f"--labels {args.labels}",
                            f"--recordings {args.recordings}"])
        date = datetime.now().astimezone().date().isoformat()
        try:
            Path(args.out).write_text(
                format_markdown(result, command, date), encoding="utf-8")
        except OSError as exc:
            print(f"error: cannot write {args.out}: {exc}")
            return 1
        print(f"report written to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
