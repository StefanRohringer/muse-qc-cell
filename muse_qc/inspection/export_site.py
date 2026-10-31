"""Export one recorded sorting run as static data for the demo page.

Reads the inspection records of a time window and writes a small
static bundle: run.json plus re-encoded crop images and the table
rescans taken before each pick. Offline only: no network, no
hardware, no API key.

Example:
    uv run python -m muse_qc.inspection.export_site \\
        --recordings recordings \\
        --since 20261031_013800 --until 20261031_014800 \\
        --out site/data
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import cv2

from muse_qc.inspection.record import load_records

STAMP_FORMAT = "%Y%m%d_%H%M%S"
MAX_SIDE_PX = 800
TABLE_MAX_SIDE_PX = 1200
JPEG_QUALITY = 85


def parse_stamp(text: str) -> datetime:
    """Parse a YYYYmmdd_HHMMSS stamp, raising ValueError when invalid."""
    return datetime.strptime(text, STAMP_FORMAT)


def record_stamp(record: dict) -> str:
    """Return the YYYYmmdd_HHMMSS stamp of a record.

    The stamp comes from the count_<stamp>.png file name so records
    sort in recording order; the ISO timestamp is the fallback.
    """
    stem = Path(str(record.get("full_image", ""))).stem
    if stem.startswith("count_"):
        return stem[len("count_"):]
    try:
        moment = datetime.fromisoformat(str(record.get("timestamp", "")))
    except ValueError:
        return ""
    return moment.strftime(STAMP_FORMAT)


def run_date(records: list[dict]) -> str:
    """Return the run date as '31 Oct 2026' from the first record."""
    try:
        moment = datetime.fromisoformat(str(records[0].get("timestamp", "")))
    except ValueError:
        return record_stamp(records[0])
    return f"{moment.day} {moment.strftime('%b %Y')}"


def _write_jpeg(source: Path, target: Path, max_side: int) -> bool:
    """Re-encode source as JPEG at target, longest side capped, True on success."""
    image = cv2.imread(str(source), cv2.IMREAD_COLOR)
    if image is None:
        return False
    height, width = image.shape[:2]
    longest = max(height, width)
    if longest > max_side:
        scale = max_side / longest
        image = cv2.resize(
            image,
            (round(width * scale), round(height * scale)),
            interpolation=cv2.INTER_AREA,
        )
    ok, encoded = cv2.imencode(
        ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    )
    if not ok:
        return False
    try:
        target.write_bytes(encoded.tobytes())
    except OSError:
        return False
    return True


def export_crop(source: Path, media_dir: Path) -> str | None:
    """Re-encode one crop PNG as JPEG, return its relative path or None.

    Only files named count_*_crop.png are accepted; anything else
    (full table or room images) is refused with None. The longest
    side is limited to MAX_SIDE_PX, quality is JPEG_QUALITY.
    """
    if not (source.name.startswith("count_") and source.name.endswith("_crop.png")):
        return None
    target = media_dir / (source.stem + ".jpg")
    if not _write_jpeg(source, target, MAX_SIDE_PX):
        return None
    return f"media/{target.name}"


def export_table_scan(source: Path, media_dir: Path) -> str | None:
    """Re-encode one table scan PNG as JPEG, return its relative path or None.

    Only files named parts_*.png are accepted; armcam_, table_,
    count_ and anything else is refused with None. The longest side
    is limited to TABLE_MAX_SIDE_PX, quality is JPEG_QUALITY.
    """
    if not (source.name.startswith("parts_") and source.name.endswith(".png")):
        return None
    target = media_dir / (source.stem + ".jpg")
    if not _write_jpeg(source, target, TABLE_MAX_SIDE_PX):
        return None
    return f"media/{target.name}"


def scan_stamp(path: Path) -> str:
    """Return the YYYYmmdd_HHMMSS stamp of a parts_<stamp>.png path, else ""."""
    if not (path.name.startswith("parts_") and path.name.endswith(".png")):
        return ""
    stamp = path.name[len("parts_"): -len(".png")]
    try:
        parse_stamp(stamp)
    except ValueError:
        return ""
    return stamp


def export_run(
    recordings_dir: str | Path,
    since: str,
    until: str,
    out_dir: str | Path,
) -> dict:
    """Export the records in [since, until] to out_dir, return run.json data."""
    start = parse_stamp(since)
    end = parse_stamp(until)
    if start > end:
        raise ValueError(f"empty window: --since {since} is after --until {until}")
    recordings = Path(recordings_dir)
    out = Path(out_dir)
    media_dir = out / "media"
    media_dir.mkdir(parents=True, exist_ok=True)

    scans: list[tuple[str, Path]] = []
    for path in sorted(recordings.glob("parts_*.png")):
        stamp = scan_stamp(path)
        if not stamp:
            continue
        if since <= stamp <= until:
            scans.append((stamp, path))

    written_scans: dict[str, str] = {}

    def table_path(source: Path) -> str | None:
        """Export one scan once, return its relative path or None."""
        key = source.name
        if key not in written_scans:
            image = export_table_scan(source, media_dir)
            if image is None:
                print(f"warning: skipping scan {key}: unreadable image")
                return None
            written_scans[key] = image
        return written_scans[key]

    parts: list[dict] = []
    kept: list[dict] = []
    part_stamps: list[str] = []
    for record in load_records(recordings):
        stamp = record_stamp(record)
        if not stamp or not (since <= stamp <= until):
            continue
        kept.append(record)
        if not isinstance(record.get("levers"), int):
            print(f"warning: skipping {stamp}: no lever count")
            continue
        if record.get("verdict") not in ("good", "bad"):
            print(f"warning: skipping {stamp}: no verdict")
            continue
        crop_source = recordings / Path(str(record.get("crop_image", ""))).name
        image = export_crop(crop_source, media_dir)
        if image is None:
            print(f"warning: skipping {stamp}: crop image missing")
            continue
        part: dict = {
            "time": str(record.get("timestamp", stamp)),
            "levers": record["levers"],
            "verdict": record["verdict"],
            "image": image,
        }
        earlier = [scan for scan in scans if scan[0] < stamp]
        if earlier:
            table_image = table_path(max(earlier)[1])
            if table_image is not None:
                part["table_image"] = table_image
        if "box" in record:
            part["box"] = str(record["box"])
        if "retries" in record:
            part["retries"] = record["retries"]
        parts.append(part)
        part_stamps.append(stamp)

    if not parts:
        raise ValueError(f"no usable records between {since} and {until}")
    good = sum(1 for part in parts if part["verdict"] == "good")
    data = {
        "date": run_date(kept),
        "parts": parts,
        "good": good,
        "bad": len(parts) - good,
    }
    later = [scan for scan in scans if scan[0] > max(part_stamps)]
    if later:
        final_table_image = table_path(max(later)[1])
        if final_table_image is not None:
            data["final_table_image"] = final_table_image
    (out / "run.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
    data["table_scans"] = len(written_scans)
    return data


def main(argv: list[str] | None = None) -> int:
    """CLI entry point, returns the exit code."""
    parser = argparse.ArgumentParser(description="Export one run for the demo page.")
    parser.add_argument("--recordings", default="recordings")
    parser.add_argument("--since", required=True, help="YYYYmmdd_HHMMSS, inclusive")
    parser.add_argument("--until", required=True, help="YYYYmmdd_HHMMSS, inclusive")
    parser.add_argument("--out", required=True, help="target dir, e.g. site/data")
    args = parser.parse_args(argv)
    try:
        data = export_run(args.recordings, args.since, args.until, args.out)
    except (ValueError, OSError) as exc:
        print(f"export failed: {exc}")
        return 1
    out = Path(args.out)
    size_kb = sum(
        path.stat().st_size for path in list(out.rglob("run.json")) + list((out / "media").glob("*.jpg"))
    ) // 1024
    print(f"parts {len(data['parts'])}, good {data['good']},"
          f" bad {data['bad']}, table scans {data.get('table_scans', 0)},"
          f" size {size_kb} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
