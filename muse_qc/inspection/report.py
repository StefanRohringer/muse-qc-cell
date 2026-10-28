"""Self-contained HTML inspection report over all JSON records.

CLI: uv run python -m muse_qc.inspection.report
     [--recordings recordings] [--out recordings/report.html]
"""

from __future__ import annotations

import argparse
import html
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

from muse_qc.inspection.record import load_records

TITLE = "Muse QC Cell inspection report"
EMPTY_TEXT = "no inspections yet"


def _relative(target: str, start: Path) -> str:
    """Return target as a relative posix path from the report folder."""
    return Path(os.path.relpath(target, start=start)).as_posix()


def _badge(verdict: str) -> str:
    """Return the green GOOD or red BAD verdict badge HTML."""
    good = verdict == "good"
    label = "GOOD" if good else "BAD"
    color = "#137333" if good else "#b3261e"
    background = "#ceead6" if good else "#fadadd"
    return (
        f'<span class="badge" style="color: {color};'
        f"background: {background};"
        f'border: 1px solid {color};">{label}</span>'
    )


def _card(record: dict, base: Path) -> str:
    """Return one inspection card HTML for a record (text escaped)."""
    timestamp = html.escape(str(record.get("timestamp", "?")))
    full = _relative(str(record.get("full_image", "")), base)
    crop = _relative(str(record.get("crop_image", "")), base)
    prompt = html.escape(str(record.get("lever_prompt", "")))
    levers = int(record.get("levers", 0))
    expected = int(record.get("expected_levers", 0))
    badge = _badge(str(record.get("verdict", "bad")))
    dropped = record.get("dropped_boxes", [])
    if dropped:
        items = "".join(
            f"<li>id {html.escape(str(box.get('id', '?')))}"
            f" box=({html.escape(str(box.get('x1', '?')))},"
            f" {html.escape(str(box.get('y1', '?')))},"
            f" {html.escape(str(box.get('x2', '?')))},"
            f" {html.escape(str(box.get('y2', '?')))}"
            f") confidence={html.escape(str(box.get('confidence', '?')))}</li>"
            for box in dropped
        )
        remnants = f"<p>dropped remnants:</p><ul>{items}</ul>"
    else:
        remnants = "<p>no dropped remnants</p>"
    return (
        '<div class="card">'
        f"<p><strong>{timestamp}</strong> {badge}</p>"
        f'<p><img src="{html.escape(crop, quote=True)}"'
        ' alt="crop with lever boxes"></p>'
        f'<p><a href="{html.escape(full, quote=True)}">full image</a></p>'
        f"<p>levers: {levers} / expected {expected}</p>"
        f"<p>prompt: {prompt}</p>"
        f"{remnants}"
        "</div>"
    )


def build_html(records: list[dict], generated: str, base: Path) -> str:
    """Build the full self-contained report page for newest-first records."""
    ordered = sorted(
        records,
        key=lambda record: (
            str(record.get("timestamp", "")),
            str(record.get("full_image", "")),
        ),
        reverse=True,
    )
    good = sum(1 for record in records if record.get("verdict") == "good")
    bad = len(records) - good
    if ordered:
        body = "\n".join(_card(record, base) for record in ordered)
    else:
        body = f"<p>{EMPTY_TEXT}</p>"
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{html.escape(TITLE)}</title>
<style>
body {{ font-family: sans-serif; margin: 2em; }}
.card {{ border: 1px solid #999; padding: 1em; margin: 1em 0; }}
.card img {{ max-width: 320px; }}
.badge {{ font-weight: bold; padding: 0.2em 0.6em; }}
</style>
</head>
<body>
<h1>{html.escape(TITLE)}</h1>
<p>generated: {html.escape(generated)}</p>
<p>parts inspected: {len(records)}, good: {good}, bad: {bad}</p>
{body}
</body>
</html>
"""


def write_report(
    recordings_dir: str | Path = "recordings",
    out: str | Path | None = None,
) -> Path:
    """Write the report HTML file, return its path (raises on failure)."""
    target_dir = Path(recordings_dir)
    out_path = Path(out) if out is not None else target_dir / "report.html"
    generated = datetime.now(timezone.utc).isoformat(timespec="seconds")
    page = build_html(load_records(target_dir), generated, out_path.parent)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(page, encoding="utf-8")
    return out_path


def main(argv: list[str] | None = None) -> int:
    """Build the HTML report over all records; 0 ok, 1 write failed."""
    parser = argparse.ArgumentParser(description="Build the inspection report.")
    parser.add_argument("--recordings", default="recordings")
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)
    try:
        out_path = write_report(args.recordings, args.out)
    except (OSError, RuntimeError) as exc:
        print(f"request failed: {exc}")
        return 1
    print(out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
