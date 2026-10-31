"""Printable checkerboard for table camera calibration (run by hand only).

Generates a single A4 landscape SVG page with a centred checkerboard
where every square has an edge of exactly CHECKER_SQUARE_MM. The
calibration added later imports the shared constants below, so both
the generator and the calibration agree on the square size, the
square count and the inner corner count.

Run by hand from the project root:

    uv run python -m muse_qc.vision.checkerboard
"""

from __future__ import annotations

from pathlib import Path

CHECKER_SQUARE_MM = 25.0
CHECKER_SQUARES = (10, 7)
CHECKER_INNER_CORNERS = (9, 6)

PAGE_WIDTH_MM = 297.0
PAGE_HEIGHT_MM = 210.0
SCALE_LINE_LENGTH_MM = 100.0
SCALE_TEXT = (
    "25 mm squares, print at 100 % / actual size, glue flat on cardboard"
)

OUTPUT_PATH = Path("docs/checkerboard.svg")


def _number(value: float) -> str:
    """Format a millimetre value for the SVG output."""
    return repr(float(value))


def board_origin_mm() -> tuple[float, float]:
    """Return the top-left board corner so the board is centred."""
    columns, rows = CHECKER_SQUARES
    board_width = columns * CHECKER_SQUARE_MM
    board_height = rows * CHECKER_SQUARE_MM
    origin_x = (PAGE_WIDTH_MM - board_width) / 2.0
    origin_y = (PAGE_HEIGHT_MM - board_height) / 2.0
    return (origin_x, origin_y)


def build_svg() -> str:
    """Build the full A4 landscape SVG page with the checkerboard."""
    columns, rows = CHECKER_SQUARES
    origin_x, origin_y = board_origin_mm()
    board_height = rows * CHECKER_SQUARE_MM
    text_x = origin_x
    text_y = 11.0
    scale_y = origin_y + board_height + 5.0
    scale_x = (PAGE_WIDTH_MM - SCALE_LINE_LENGTH_MM) / 2.0
    scale_end_x = scale_x + SCALE_LINE_LENGTH_MM
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg"'
        f' width="{_number(PAGE_WIDTH_MM)}mm"'
        f' height="{_number(PAGE_HEIGHT_MM)}mm"'
        f' viewBox="0 0 {_number(PAGE_WIDTH_MM)}'
        f' {_number(PAGE_HEIGHT_MM)}">',
        f'<rect x="0" y="0" width="{_number(PAGE_WIDTH_MM)}"'
        f' height="{_number(PAGE_HEIGHT_MM)}" fill="white"/>',
        f'<g id="checkerboard">',
    ]
    for row in range(rows):
        for column in range(columns):
            if (column + row) % 2 != 0:
                continue
            x = origin_x + column * CHECKER_SQUARE_MM
            y = origin_y + row * CHECKER_SQUARE_MM
            lines.append(
                f'<rect x="{_number(x)}" y="{_number(y)}"'
                f' width="{_number(CHECKER_SQUARE_MM)}"'
                f' height="{_number(CHECKER_SQUARE_MM)}"'
                f' fill="black"/>'
            )
    lines.append("</g>")
    lines.append(
        f'<text x="{_number(text_x)}" y="{_number(text_y)}"'
        f' font-size="{_number(5.0)}"'
        f' font-family="sans-serif" fill="black">{SCALE_TEXT}</text>'
    )
    lines.append(
        f'<line id="scale-line" x1="{_number(scale_x)}"'
        f' y1="{_number(scale_y)}" x2="{_number(scale_end_x)}"'
        f' y2="{_number(scale_y)}" stroke="black" stroke-width="0.5"/>'
    )
    lines.append("</svg>")
    return "\n".join(lines) + "\n"


def main() -> None:
    """Write docs/checkerboard.svg and print its path."""
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_PATH.write_text(build_svg(), encoding="utf-8")
    print(OUTPUT_PATH)


if __name__ == "__main__":
    main()
