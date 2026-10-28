"""Cycle timing summary for the first arm motion. Pure, moves nothing."""

from __future__ import annotations

import math


def summarize_ms(values: object) -> str:
    """Mean, p95 and max of durations in milliseconds.

    Returns "no samples" for an empty list. p95 uses the
    nearest-rank method so a single sample reports itself.
    """
    samples = [float(value) for value in values]
    if not samples or not all(math.isfinite(v) for v in samples):
        if not samples:
            return "no samples"
        raise ValueError("Cycle times must be finite numbers.")
    ordered = sorted(samples)
    rank = max(1, int(math.ceil(0.95 * len(ordered))))
    p95 = ordered[rank - 1]
    mean = sum(ordered) / len(ordered)
    peak = ordered[-1]
    return f"mean {mean:.2f} ms, p95 {p95:.2f} ms, max {peak:.2f} ms"
