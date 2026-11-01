"""Live status page for the agent run (static HTML, auto-reload).

While run_agent works, LivePage keeps the run state and rewrites
recordings/live.html after every update, so the human (and the
demo video) can watch along in a browser pointed at that file.
No server, no JavaScript, no external URLs: the page reloads
through a meta refresh tag. Writes are atomic (temp file in the
same folder plus os.replace) so the browser never shows a
half-written page. All methods are cheap and never raise to the
caller: an OSError prints one short "live page: ..." warning and
the run goes on.
"""

from __future__ import annotations

import html
import json
import os
from datetime import datetime
from pathlib import Path

from muse_qc.inspection.report import _badge

TITLE = "Muse QC Cell live"
REFRESH_SECONDS = 2
MAX_LINES = 15


def _relative(target: str, start: Path) -> str:
    """Return target as a relative posix path from the page folder."""
    return Path(os.path.relpath(target, start=start)).as_posix()


def _format_lever(value: object) -> str:
    """Return a lever_deg value rounded for display ("?" unknown)."""
    try:
        return f"{float(value):.1f}"
    except (TypeError, ValueError):
        return "?"


def _format_position(value: object) -> str:
    """Return a [x, y] table position rounded for display."""
    try:
        return f"[{float(value[0]):.1f}, {float(value[1]):.1f}]"
    except (TypeError, ValueError, IndexError):
        return "?"


def _format_part(part: dict) -> str:
    """Return one scanned part list item HTML (text escaped)."""
    lever = _format_lever(part.get("lever_deg"))
    return (
        f"id {html.escape(str(part.get('id', '?')))}"
        f" at {html.escape(_format_position(part.get('table_mm')))}"
        f" lever {html.escape(lever)} deg"
    )


def _record_crop(record_path: str) -> str | None:
    """Return the crop_image of one record JSON file, or None."""
    try:
        data = json.loads(Path(record_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    crop = data.get("crop_image")
    return None if crop is None else str(crop)


def _format_card(card: dict, base: Path) -> str:
    """Return one sorted part card HTML, newest first (text escaped)."""
    badge = _badge(str(card.get("verdict", "bad")))
    crop = card.get("crop")
    if crop:
        image = (
            f'<p><img src="{html.escape(_relative(crop, base), quote=True)}"'
            ' alt="inspected part crop"></p>'
        )
    else:
        image = ""
    record = card.get("record")
    source = (
        f"<p>record: {html.escape(Path(record).name)}</p>" if record else ""
    )
    return (
        '<div class="card">'
        f"<p>{badge}</p>"
        f"{image}"
        f"<p>levers {html.escape(str(card.get('levers', '?')))}</p>"
        f"<p>box: {html.escape(str(card.get('box', '?')))}</p>"
        f"{source}"
        "</div>"
    )


class LivePage:
    """Run state plus its static HTML page, rewritten on update."""

    def __init__(self, path="recordings/live.html", now=None) -> None:
        self.path = Path(path)
        self._now = now
        self._status = "starting"
        self._lines: list[str] = []
        self._image: str | None = None
        self._parts: list[dict] = []
        self._cards: list[dict] = []
        self._good = 0
        self._bad = 0
        self._failed = 0
        self._left = 0
        self._warned = False
        self._render()

    def line(self, text: str) -> None:
        """Append one console line (keep the last 15)."""
        self._lines.append(str(text))
        del self._lines[:-MAX_LINES]
        self._render()

    def status(self, text: str) -> None:
        """Set the big status line."""
        self._status = str(text)
        self._render()

    def scan(self, image_path: str | None, parts: list[dict]) -> None:
        """Store the latest table image plus the part list."""
        self._image = None if image_path is None else str(image_path)
        self._parts = [dict(part) for part in (parts or [])]
        self._left = len(self._parts)
        self._render()

    def sorted(self, result: dict) -> None:
        """Record one finished part (counter, plus a card when sorted)."""
        if result.get("outcome") != "sorted":
            self._failed += 1
            self._render()
            return
        if result.get("verdict") == "good":
            self._good += 1
        else:
            self._bad += 1
        self._left = max(0, self._left - 1)
        record = result.get("record")
        self._cards.insert(0, {
            "levers": result.get("levers"),
            "verdict": result.get("verdict"),
            "box": result.get("box"),
            "record": None if record is None else str(record),
            "crop": _record_crop(str(record)) if record else None,
        })
        self._render()

    def finish(self, summary: str) -> None:
        """Set the final status line from the run summary."""
        text = str(summary or "").strip()
        self._status = f"finished: {text}" if text else "finished"
        self._render()

    def before_tool(self, name: str, args: dict) -> None:
        """Set the status line for one upcoming tool call."""
        if name == "scan_table":
            self.status("scanning the table")
        elif name == "inspect_and_sort":
            self.status(f"sorting part {args.get('part_id')}")
        elif name == "sort_in_order":
            order = args.get("part_ids") or []
            self.status(f"sorting {len(order)} parts in order")
        elif name == "write_report":
            self.status("writing the report")
        else:
            self.status(f"calling {name}")

    def after_tool(self, name: str, result: dict) -> None:
        """Feed one tool result into the page state."""
        if not isinstance(result, dict):
            return
        if name == "scan_table":
            if result.get("outcome") is None:
                self.scan(result.get("image"), result.get("parts") or [])
        elif name == "sort_in_order":
            for planned in result.get("sorted") or []:
                self.sorted({"outcome": "sorted",
                             "levers": planned.get("levers"),
                             "verdict": planned.get("verdict"),
                             "box": planned.get("box"),
                             "record": planned.get("record")})
            if result.get("deviation_outcome") in (
                    "missed_grip", "error", "inspect_failed",
                    "interrupted"):
                self.sorted(
                    {"outcome": result["deviation_outcome"]})
            if isinstance(result.get("table after sort"), list):
                self.scan(result.get("image after sort") or self._image,
                          result["table after sort"])
        elif name == "inspect_and_sort":
            self.sorted(result)
            if isinstance(result.get("table after sort"), list):
                self.scan(result.get("image after sort") or self._image,
                          result["table after sort"])

    def _render(self) -> None:
        """Rebuild the page and swap it in atomically (warns once)."""
        try:
            page = self._build()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.parent / (self.path.name + ".tmp")
            tmp.write_text(page, encoding="utf-8")
            os.replace(tmp, self.path)
        except OSError as exc:
            if not self._warned:
                self._warned = True
                print(f"live page: {exc}")

    def _build(self) -> str:
        """Build the full page HTML (all text escaped)."""
        base = self.path.parent
        if self._image:
            image = (
                '<p><img class="table"'
                f' src="{html.escape(_relative(self._image, base), quote=True)}"'
                ' alt="annotated table"></p>'
            )
        else:
            image = "<p>no table image yet</p>"
        if self._parts:
            items = "".join(
                f"<li>{_format_part(part)}</li>" for part in self._parts
            )
            parts = f"<ul>{items}</ul>"
        else:
            parts = "<p>no parts on the table</p>"
        if self._cards:
            cards = "\n".join(
                _format_card(card, base) for card in self._cards
            )
        else:
            cards = "<p>no parts sorted yet</p>"
        console = "\n".join(html.escape(line) for line in self._lines)
        ticks = self._now if self._now is not None else datetime.now
        stamp = ticks().strftime("%Y-%m-%d %H:%M:%S")
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta http-equiv="refresh" content="{REFRESH_SECONDS}">
<title>{html.escape(TITLE)}</title>
<style>
body {{ font-family: sans-serif; margin: 2em; font-size: 20px;
  background: #ffffff; color: #111111; }}
h1 {{ font-size: 2em; }}
.status {{ font-size: 2em; font-weight: bold; }}
.counters {{ font-size: 1.4em; }}
img.table {{ width: 100%; }}
.card {{ border: 1px solid #999999; padding: 1em; margin: 1em 0;
  font-size: 1.25em; }}
.card img {{ max-width: 480px; }}
.badge {{ font-weight: bold; padding: 0.2em 0.6em; }}
pre {{ font-size: 1em; background: #f0f0f0; padding: 1em;
  white-space: pre-wrap; }}
ul {{ font-size: 1.1em; }}
.meta {{ font-size: 0.9em; color: #333333; }}
</style>
</head>
<body>
<h1>{html.escape(TITLE)}</h1>
<p class="status">{html.escape(self._status)}</p>
<p class="counters">good: {self._good}, bad: {self._bad}, missed or failed: {self._failed}, parts left: {self._left}</p>
{image}
{parts}
{cards}
<pre>{console}</pre>
<p class="meta">updated: {html.escape(stamp)}, reloads every {REFRESH_SECONDS} s</p>
</body>
</html>
"""
