"""JSON inspection records, one per count_clips run.

Each successful run leaves recordings/count_<timestamp>.json next to
its two PNGs. The report command reads all records back.
"""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path


def _box_entry(box: dict) -> dict:
    """Return the JSON-safe box fields of one SAM box dict."""
    return {
        "id": int(box["id"]),
        "x1": int(box["x1"]),
        "y1": int(box["y1"]),
        "x2": int(box["x2"]),
        "y2": int(box["y2"]),
        "confidence": float(box["confidence"]),
    }


def build_record(
    *,
    source_image: str | Path,
    full_image: str | Path,
    crop_image: str | Path,
    part: dict,
    lever_prompt: str,
    kept: list[dict],
    dropped: list[dict],
    expected_levers: int,
    timestamp: str | None = None,
) -> dict:
    """Build the JSON-safe record dict of one inspection run."""
    if timestamp is None:
        timestamp = datetime.now(timezone.utc).isoformat(timespec="seconds")
    count = len(kept)
    return {
        "timestamp": timestamp,
        "source_image": str(source_image),
        "full_image": str(full_image),
        "crop_image": str(crop_image),
        "part_box": {
            "x1": int(part["x1"]),
            "y1": int(part["y1"]),
            "x2": int(part["x2"]),
            "y2": int(part["y2"]),
        },
        "part_confidence": float(part["confidence"]),
        "lever_prompt": lever_prompt,
        "kept_boxes": [_box_entry(box) for box in kept],
        "dropped_boxes": [_box_entry(box) for box in dropped],
        "levers": count,
        "verdict": "good" if count == expected_levers else "bad",
        "expected_levers": int(expected_levers),
    }


def write_record(record: dict, recordings_dir: str | Path = "recordings") -> Path:
    """Write the record as count_<same timestamp>.json, return its path.

    The JSON name is derived from the full image path printed by
    count_clips, so it lands next to the two PNGs. Raises
    RuntimeError when the file cannot be written or verified.
    """
    target_dir = Path(recordings_dir)
    target_dir.mkdir(parents=True, exist_ok=True)
    path = target_dir / (Path(str(record["full_image"])).stem + ".json")
    try:
        path.write_text(json.dumps(record, indent=2), encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"failed to write record to {path}: {exc}") from exc
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"failed to verify record at {path}: {exc}") from exc
    if loaded != record:
        raise RuntimeError(f"failed to verify record at {path}")
    return path


def load_records(recordings_dir: str | Path = "recordings") -> list[dict]:
    """Read all count_*.json records sorted by timestamp.

    Unreadable files are skipped with a short warning; an empty
    folder gives an empty list.
    """
    target_dir = Path(recordings_dir)
    records: list[dict] = []
    for path in sorted(target_dir.glob("count_*.json")):
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            print(f"warning: skipping unreadable record {path.name}: {exc}")
            continue
        if not isinstance(record, dict):
            print(f"warning: skipping unreadable record {path.name}: not a dict")
            continue
        records.append(record)
    records.sort(key=lambda record: (str(record.get("timestamp", "")), str(record.get("full_image", ""))))
    return records
