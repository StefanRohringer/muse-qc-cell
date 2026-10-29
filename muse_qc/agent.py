"""Agentic quality inspection cell, driven by Meta Muse Spark.

The model (muse-spark-1.3, Responses API, tool calling) decides
which tool to call next; this module only executes the tools from
muse_qc.agent_tools and enforces the hard limits. Default is
--simulate: the real Muse Spark API with the fake table tools, so
the human checks the tool calling without hardware. --execute uses
the real tools after one typed "yes" (the human keeps a hand on
the arm power switch). Only the human starts it, never CI.

Responses API assumption (unchecked here, only output_text is
used so far): create() takes instructions, input, tools,
reasoning={"effort": "low"}, store=False and
include=["reasoning.encrypted_content"]; calls arrive as
{"type": "function_call", ...} with arguments a JSON string and
return as {"type": "function_call_output", ...} with output a
JSON string. Reasoning items (with encrypted_content) are
replayed into the history but never written to the transcript.
The human checks this in the --simulate run.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

from muse_qc import agent_tools
from muse_qc.agent_io import (
    field,
    iter_calls,
    message_text,
    plain_item,
    usage_text,
)
from muse_qc.agent_tools import TOOL_DEFINITIONS, RunState
from muse_qc.model_check import BASE_URL, KEY_NAME, read_api_key

MODEL = "muse-spark-1.3"
MAX_STEPS = 40
MAX_SORTS = 10
MAX_PER_POSITION = 2
FAIL_STOP = 2

SYSTEM_PROMPT = (
    "You control a robot quality inspection cell. Parts are "
    "connectors with 3 orange levers: good means 3 levers, bad "
    "means fewer. Always scan before sorting, sort one part at a "
    "time, rescan after every sort, retry a missed grip at most "
    "once per part, never sort a part twice, stop when a scan "
    "finds no parts, then write the report and finish with a "
    "short summary (parts sorted good/bad, failures)."
)
START_TEXT = (
    "Start the inspection: scan the table, then sort every part."
)
CONTINUE_TEXT = "Continue with the next tool call."


def _default_client_factory(api_key: str) -> object:
    """Build the real client (only called by main, never by tests)."""
    from openai import OpenAI
    return OpenAI(api_key=api_key, base_url=BASE_URL)


def _model_call(client, history):
    """One Responses API call with tools and low reasoning effort."""
    create = getattr(getattr(client, "responses"), "create")
    return create(
        model=MODEL,
        instructions=SYSTEM_PROMPT,
        input=history,
        tools=TOOL_DEFINITIONS,
        reasoning={"effort": "low"},
        store=False,
        include=["reasoning.encrypted_content"],
    )


def run_agent(client, scan_fn, sort_fn, report_fn, out=print,
              recordings_dir="recordings",
              max_steps=MAX_STEPS, max_sorts=MAX_SORTS) -> dict:
    """Run the tool-calling loop; return the run summary dict.

    scan_fn returns {"parts", "skipped", "image"}, sort_fn takes a
    stored [x, y] position and returns the outcome dict, report_fn
    takes this run's record paths and returns the report path. The
    key never appears here: only the client factory sees it.
    """
    state = RunState()
    history: list = [{"role": "user", "content": [
        {"type": "input_text", "text": START_TEXT}]}]
    entries: list[dict] = []
    summary = ""
    finished = False
    good = bad = 0
    fails = 0
    step = 0
    try:
        while step < max_steps:
            step += 1
            try:
                response = _model_call(client, history)
            except Exception as exc:  # noqa: BLE001 - short, no key
                out(f"agent: model request failed: {exc}")
                break
            output = list(field(response, "output", []) or [])
            tokens = usage_text(response)
            if tokens is not None:
                out(tokens)
            text = "\n".join(message_text(item) for item in output
                             if field(item, "type") == "message").strip()
            if text:
                out(f"agent: {text}")
            calls = list(iter_calls(output))
            entry = {"model_text": text, "calls": [], "results": []}
            if not calls:
                for item in output:
                    history.append(plain_item(item))
                history.append(
                    {"role": "user", "content": [
                        {"type": "input_text", "text": CONTINUE_TEXT}]})
                entries.append(entry)
                continue
            outputs = []
            stop = False
            for call in calls:
                name = call["name"]
                try:
                    args = json.loads(call["arguments"] or "{}")
                except ValueError:
                    args = {}
                if not isinstance(args, dict):
                    args = {}
                entry["calls"].append({"name": name, "arguments": args})
                result, stop = _run_tool(
                    name, args, state, scan_fn, sort_fn, report_fn,
                    max_sorts, out)
                entry["results"].append(result)
                outputs.append({
                    "type": "function_call_output",
                    "call_id": call["call_id"],
                    "output": json.dumps(result),
                })
                if name == "finish":
                    summary = str(args.get("summary", ""))
                    finished = True
                if result.get("outcome") == "sorted":
                    if result.get("verdict") == "good":
                        good += 1
                    else:
                        bad += 1
                if stop:
                    break
            for tool_result in entry["results"]:
                outcome = tool_result.get("outcome")
                if outcome in ("error", "inspect_failed"):
                    fails += 1
                elif outcome is not None:
                    # A successful scan (no outcome) between two
                    # failed sorts keeps the count: rescanning is
                    # mandatory after every sort.
                    fails = 0
            for item in output:
                history.append(plain_item(item))
            history.extend(outputs)
            entries.append(entry)
            if finished or stop:
                if any(result.get("outcome") == "interrupted"
                       for result in entry["results"]):
                    out("agent: interrupted by human, stopping")
                break
            if fails >= FAIL_STOP:
                out("agent: 2 consecutive failures, stopping")
                break
        else:
            out(f"agent: step limit reached ({max_steps}), stopping")
    except KeyboardInterrupt:
        out("agent: interrupted by human, stopping")
    path = _write_transcript(recordings_dir, entries, summary, out)
    return {
        "summary": summary,
        "finished": finished,
        "good": good,
        "bad": bad,
        "steps": step,
        "transcript": path,
    }


def _run_tool(name, args, state, scan_fn, sort_fn, report_fn,
              max_sorts, out) -> tuple[dict, bool]:
    """Execute one model tool call; return (result, stop_loop)."""
    if name == "scan_table":
        out("agent -> scan_table()")
        try:
            found = scan_fn()
        except Exception as exc:  # noqa: BLE001 - short, no key
            result = {"outcome": "error",
                      "reason": f"scan failed: {exc}"}
            out(f"tool: {result['reason']}")
            return result, False
        items = found.get("parts", [])
        positions = [part["table_mm"] for part in items]
        state.new_scan(positions)
        count = len(positions)
        skipped = int(found.get("skipped", 0) or 0)
        result = {"parts": [
            {"id": index,
             "table_mm": [float(pos[0]), float(pos[1])],
             "confidence": part.get("confidence")}
            for index, (pos, part) in enumerate(zip(positions, items))
        ], "skipped": skipped, "image": found.get("image")}
        if count == 0:
            out("tool: no parts found")
        elif count == 1:
            out("tool: 1 part found")
        else:
            out(f"tool: {count} parts found")
        return result, False
    if name == "inspect_and_sort":
        part_id = args.get("part_id")
        out(f"agent -> inspect_and_sort(part {part_id})")
        position = state.lookup(part_id)
        if position is None:
            result = {"outcome": "refused",
                      "reason": "unknown part id,"
                      " call scan_table first"}
            out(f"tool: part {part_id} refused: {result['reason']}")
            return result, False
        if state.already_sorted(position):
            result = {"outcome": "refused",
                      "reason": "part already sorted"}
            out(f"tool: part {part_id} refused: {result['reason']}")
            return result, False
        if state.attempts_at(position) >= MAX_PER_POSITION:
            result = {"outcome": "refused",
                      "reason": "already tried twice, skipping part"}
            out(f"tool: part {part_id} refused: {result['reason']}")
            return result, False
        if state.sort_count >= max_sorts:
            out(f"agent: sort limit reached ({max_sorts}), stopping")
            return {"outcome": "refused",
                    "reason": "sort limit reached"}, True
        state.note_attempt(position)
        state.sort_count += 1
        try:
            result = dict(sort_fn(position))
        except Exception as exc:  # noqa: BLE001 - short, no key
            result = {"outcome": "error", "levers": None,
                      "verdict": None, "box": None, "record": None,
                      "reason": f"sort failed: {exc}"}
        result["lines"] = list(result.get("lines") or [])[-12:]
        state.mark_stale()
        outcome = result.get("outcome")
        if outcome == "sorted":
            if result.get("record"):
                state.records.append(str(result["record"]))
            state.note_sorted(position)
            out(f"tool: part {part_id} levers {result.get('levers')}"
                f" {result.get('verdict')}, sorted")
        elif outcome == "interrupted":
            out(f"tool: part {part_id} interrupted")
            return result, True
        elif outcome == "missed_grip":
            out(f"tool: part {part_id} missed grip")
        elif outcome == "inspect_failed":
            out(f"tool: part {part_id} inspect failed")
        elif outcome == "refused":
            out(f"tool: part {part_id} refused:"
                f" {result.get('reason')}")
        else:
            out(f"tool: part {part_id} error: {result.get('reason')}")
        return result, False
    if name == "write_report":
        out("agent -> write_report()")
        try:
            path = report_fn(list(state.records))
        except Exception as exc:  # noqa: BLE001 - short, no key
            result = {"outcome": "error",
                      "reason": f"report failed: {exc}"}
            out(f"tool: {result['reason']}")
            return result, False
        result = {"outcome": "written", "report": str(path)}
        out(f"tool: report written to {path}")
        return result, False
    if name == "finish":
        out("agent -> finish()")
        text = str(args.get("summary", ""))
        out(f"agent finished: {text}")
        return {"outcome": "finished"}, False
    out(f"agent -> {name}()")
    result = {"outcome": "refused",
              "reason": f"unknown tool {name}"}
    out(f"tool: refused: {result['reason']}")
    return result, False


def _write_transcript(recordings_dir, entries, summary, out) -> str:
    """Write the run transcript JSON (no key, no reasoning blobs)."""
    target = Path(recordings_dir)
    target.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = target / f"agent_{stamp}.json"
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


def main(argv=None, client_factory=None, toolset=None, input_fn=None,
         out=print) -> int:
    """Entry point: --simulate by default, --execute moves the arm.

    Returns 0 only when the agent called finish, else 1 (limit,
    failures, API error, interrupt). Exit 2 means missing key or
    missing "yes" confirmation with nothing moved.
    """
    parser = argparse.ArgumentParser(
        description="Agentic QC cell loop (Muse Spark, human-started).")
    parser.add_argument("--execute", action="store_true",
                        help="use the real tools and move the arm")
    args = parser.parse_args(argv)
    ask = input_fn if input_fn is not None else input
    if args.execute:
        if toolset is None:
            try:
                answer = ask("Type yes to let the agent move the arm: ")
            except (EOFError, KeyboardInterrupt):
                out("confirmation missing, nothing moved")
                return 2
            if answer.strip() != "yes":
                out("confirmation missing, nothing moved")
                return 2
            out("agent: human confirmed,"
                " keep a hand on the power supply switch")
            toolset = (
                agent_tools.real_scan,
                agent_tools.real_sort,
                agent_tools.real_report,
            )
    elif toolset is None:
        fake = agent_tools.FakeTable()
        toolset = (fake.scan, fake.sort, agent_tools.fake_report)
    key = read_api_key()
    if not (key or "").strip():
        out(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    factory = client_factory if client_factory is not None \
        else _default_client_factory
    try:
        client = factory(key)
    except Exception as exc:  # noqa: BLE001 - mapped, key never printed
        out(f"request failed: {exc}")
        return 1
    result = run_agent(client, *toolset, out=out)
    return 0 if result["finished"] else 1


if __name__ == "__main__":
    sys.exit(main())
