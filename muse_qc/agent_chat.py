"""Human orders by chat for the agent loop (--task and --chat).

The tools and their refusals stay exactly the same: a plain-words
order only changes WHICH parts the model asks for, never bypasses
a tool check. Only the human starts this, never CI.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path


def write_transcript(recordings_dir, entries, summary, out) -> str:
    """Write the run transcript JSON (no key, no reasoning blobs)."""
    target = Path(recordings_dir)
    target.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target / f"agent_{stamp}.json"
    from muse_qc.agent import MODEL  # lazy: agent.py imports this module
    try:
        path.write_text(json.dumps({
            "model": MODEL,
            "summary": summary,
            "steps": entries,
        }, indent=2), encoding="utf-8")
    except OSError as exc:
        out(f"agent: transcript failed: {exc}")
        return ""
    out(f"agent: transcript {path}")
    return str(path)


def run_session(client, scan_fn, sort_fn, report_fn, *, task=None,
                chat=False, ask=None, out=print,
                recordings_dir="recordings", max_steps=None,
                max_sorts=None, live=None) -> dict:
    """Run the first order, then keep asking while chat is on.

    task replaces the default first order; with chat, ask (a
    drop-in for input) prompts "next order (empty line to quit): "
    after each run and a non-empty line continues the same run
    (same history, same RunState, same limits and safety). Before
    each new chat order (not the first) the scan expires
    (state.fresh is set to False), so the model must call
    scan_table again: the human may have moved parts between
    orders. Every human order and every final summary goes
    through out, hence onto the live page console when live is
    given. Returns the last run summary dict.
    """
    from muse_qc.agent import (MAX_SORTS, MAX_STEPS, START_TEXT,
                               run_agent)
    if max_steps is None:
        max_steps = MAX_STEPS
    if max_sorts is None:
        max_sorts = MAX_SORTS
    emit = out
    if live is not None:
        def out(text, _emit=emit, _page=live):
            _emit(text)
            _page.line(text)

    history = None
    state = None
    pending = task if task else START_TEXT
    result: dict = {"finished": False, "summary": ""}
    while True:
        out(f"order: {pending}")
        if history is not None:
            history.append({"role": "user", "content": [
                {"type": "input_text", "text": pending}]})
            if state is not None:
                state.fresh = False
        result = run_agent(
            client, scan_fn, sort_fn, report_fn, out=emit,
            recordings_dir=recordings_dir, max_steps=max_steps,
            max_sorts=max_sorts, live=live,
            first_text=pending, history=history, state=state)
        history = result["history"]
        state = result["state"]
        if result.get("summary"):
            out(f"agent summary: {result['summary']}")
        if not chat:
            break
        if ask is None:
            break
        try:
            answer = ask("next order (empty line to quit): ")
        except (EOFError, KeyboardInterrupt):
            break
        if not answer.strip():
            break
        pending = answer.strip()
    return result
