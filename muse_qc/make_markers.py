"""Print an A4 sheet with the four table markers (run by hand only).

Generates a single A4 SVG page where every marker black square has an
edge of exactly TABLE_MARKER_SIZE_MM. Nothing is detected or moved
here; this module only writes the printable file.

Run by hand from the project root:

    uv run python -m muse_qc.make_markers
"""

from __future__ import annotations

from pathlib import Path

from muse_qc.table_markers import (
    TABLE_MARKER_IDS,
    TABLE_MARKER_SIZE_MM,
    marker_cells,
)

PAGE_WIDTH_MM = 210.0
PAGE_HEIGHT_MM = 297.0
MARKER_MARGIN_MM = 10.0
LABEL_GAP_MM = 4.0
LABEL_HEIGHT_MM = 5.0
SCALE_LINE_LENGTH_MM = 100.0
SCALE_LINE_TEXT = "100 mm, print at 100 % / actual size"

MARKER_POSITIONS_MM = {
    0: (30.0, 30.0),
    1: (140.0, 30.0),
    2: (30.0, 150.0),
    3: (140.0, 150.0),
}
SCALE_LINE_START_MM = (30.0, 250.0)

OUTPUT_PATH = Path("docs/table_markers.svg")


def _number(value: float) -> str:
    """Format a millimetre value for the SVG output."""
    return repr(float(value))


def build_svg() -> str:
    """Build the full A4 SVG page with the four markers as a string."""
    cell = TABLE_MARKER_SIZE_MM / 6.0
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg"'
        f' width="{_number(PAGE_WIDTH_MM)}mm"'
        f' height="{_number(PAGE_HEIGHT_MM)}mm"'
        f' viewBox="0 0 {_number(PAGE_WIDTH_MM)}'
        f' {_number(PAGE_HEIGHT_MM)}">',
        f'<rect x="0" y="0" width="{_number(PAGE_WIDTH_MM)}"'
        f' height="{_number(PAGE_HEIGHT_MM)}" fill="white"/>',
    ]
    for marker_id in TABLE_MARKER_IDS:
        origin_x, origin_y = MARKER_POSITIONS_MM[marker_id]
        cells = marker_cells(marker_id)
        lines.append(f'<g id="marker-{marker_id}">')
        for row in range(6):
            for column in range(6):
                if cells[row, column] == 1:
                    x = origin_x + column * cell
                    y = origin_y + row * cell
                    lines.append(
                        f'<rect x="{_number(x)}" y="{_number(y)}"'
                        f' width="{_number(cell)}"'
                        f' height="{_number(cell)}" fill="black"/>'
                    )
        label_x = origin_x
        label_y = origin_y + TABLE_MARKER_SIZE_MM + LABEL_GAP_MM
        lines.append(
            f'<text x="{_number(label_x)}" y="{_number(label_y)}"'
            f' font-size="{_number(LABEL_HEIGHT_MM)}"'
            f' font-family="sans-serif" fill="black">ID {marker_id}</text>'
        )
        lines.append("</g>")
    start_x, start_y = SCALE_LINE_START_MM
    end_x = start_x + SCALE_LINE_LENGTH_MM
    lines.append(
        f'<line id="scale-line" x1="{_number(start_x)}"'
        f' y1="{_number(start_y)}" x2="{_number(end_x)}"'
        f' y2="{_number(start_y)}" stroke="black" stroke-width="0.5"/>'
    )
    lines.append(
        f'<text x="{_number(start_x)}" y="{_number(start_y + 7.0)}"'
        f' font-size="{_number(LABEL_HEIGHT_MM)}"'
        f' font-family="sans-serif" fill="black">{SCALE_LINE_TEXT}</text>'
    )
    lines.append("</svg>")
    return "\n".join(lines) + "\n"


def write_markers(path: str | Path = OUTPUT_PATH) -> Path:
    """Write the SVG page to path (creating folders) and return the path."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(build_svg(), encoding="utf-8")
    return target


def main() -> None:
    """Write docs/table_markers.svg and print its path."""
    print(write_markers())


if __name__ == "__main__":
    main()
