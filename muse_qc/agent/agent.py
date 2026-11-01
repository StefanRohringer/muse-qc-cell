"""Agentic quality inspection cell, driven by Meta Muse Spark.

The model (muse-spark-1.3, Responses API, tool calling) decides
which tool to call next; this module only executes the tools from
muse_qc.agent.agent_tools and enforces the hard limits. Default is
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
import functools
import json
import sys

from muse_qc.agent import agent_chat, agent_tools
from muse_qc.agent.agent_io import (
    field,
    iter_calls,
    message_text,
    plain_item,
    usage_text,
)
from muse_qc.agent.agent_plan import PLAN_PROMPT, SORT_IN_ORDER_TOOL
from muse_qc.agent.agent_toolrun import run_tool
from muse_qc.agent.agent_tools import TOOL_DEFINITIONS, RunState
from muse_qc.agent.live_page import LivePage
from muse_qc.checks.model_check import BASE_URL, KEY_NAME, read_api_key

MODEL = "muse-spark-1.3"
MAX_STEPS = 40
MAX_SORTS = 10
MAX_PER_POSITION = 2
FAIL_STOP = 2

SYSTEM_PROMPT = (
    "You control a robot quality inspection cell. Parts are "
    "connectors with 3 orange levers: good means 3 levers, bad "
    "means fewer. Scan once at the start; every inspect_and_sort "
    "result contains the updated table list with new part ids, "
    "always use the ids from the latest list, retry a missed "
    "grip at most once per part, never sort a part twice, "
    "rescan after a failed or missed sort and once the list is "
    "done, stop when a scan finds no parts, then write the "
    "report and finish with a short summary (parts sorted "
    "good/bad, failures)."
)
START_TEXT = (
    "Start the inspection: scan the table, then sort every part."
)
CONTINUE_TEXT = "Continue with the next tool call."


def _default_client_factory(api_key: str) -> object:
    """Build the real client (only called by main, never by tests)."""
    from openai import OpenAI
    return OpenAI(api_key=api_key, base_url=BASE_URL)


def _model_call(client, history, tools, instructions):
    """One Responses API call with tools and low reasoning effort."""
    create = getattr(getattr(client, "responses"), "create")
    return create(
        model=MODEL,
        instructions=instructions,
        input=history,
        tools=tools,
        reasoning={"effort": "low"},
        store=False,
        include=["reasoning.encrypted_content"],
    )


def _loop_tools(plan_once):
    """Return the tool definitions for this run.

    Without --plan-once exactly the tools used so far; with it
    plus sort_in_order.
    """
    if plan_once:
        return list(TOOL_DEFINITIONS) + [SORT_IN_ORDER_TOOL]
    return TOOL_DEFINITIONS


def _loop_instructions(plan_once):
    """Return the system prompt for this run (one added sentence)."""
    if plan_once:
        return SYSTEM_PROMPT + PLAN_PROMPT
    return SYSTEM_PROMPT


def run_agent(client, scan_fn, sort_fn, report_fn, out=print,
              recordings_dir="recordings", max_steps=MAX_STEPS,
              max_sorts=MAX_SORTS, live=None, first_text=None,
              history=None, state=None, plan_once=False) -> dict:
    """Run the tool-calling loop; return the run summary dict.

    scan_fn returns {"parts", "skipped", "image"}, sort_fn takes a
    stored [x, y] position and returns the outcome dict, report_fn
    takes this run's record paths and returns the report path. The
    key never appears here: only the client factory sees it.
    With plan_once the model may sort a whole order with one
    sort_in_order call; without it the run behaves as before.
    """
    tools = _loop_tools(plan_once)
    instructions = _loop_instructions(plan_once)
    if state is None:
        state = RunState()
    if history is None:
        history = [{"role": "user", "content": [
            {"type": "input_text",
             "text": first_text or START_TEXT}]}]
    entries: list[dict] = []
    summary = stop_reason = ""
    if live is not None:
        def out(text, _emit=out, _page=live):
            _emit(text)
            _page.line(text)
    finished = False
    good = bad = 0
    fails = 0
    step = 0
    try:
        while step < max_steps:
            step += 1
            try:
                response = _model_call(client, history, tools,
                                       instructions)
            except Exception as exc:  # noqa: BLE001 - short, no key
                out(f"agent: model request failed: {exc}")
                stop_reason = f"model request failed: {exc}"
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
                result, stop = run_tool(
                    name, args, state, scan_fn, sort_fn, report_fn,
                    max_sorts, MAX_PER_POSITION, out, live,
                    plan_once)
                if live is not None:
                    live.after_tool(name, result)
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
                for planned in result.get("sorted") or []:
                    if planned.get("verdict") == "good":
                        good += 1
                    else:
                        bad += 1
                if stop:
                    break
            for tool_result in entry["results"]:
                outcome = tool_result.get("outcome")
                deviation = tool_result.get("deviation_outcome")
                if outcome in ("error", "inspect_failed") or \
                        deviation in ("error", "inspect_failed"):
                    fails += 1
                elif outcome is not None:
                    # A successful scan (no outcome) between two
                    # failed sorts keeps the count: rescanning is
                    # mandatory after a failed or missed sort.
                    fails = 0
            for item in output:
                history.append(plain_item(item))
            history.extend(outputs)
            entries.append(entry)
            if finished or stop:
                if any(result.get("outcome") == "interrupted"
                       or result.get("deviation_outcome")
                       == "interrupted"
                       for result in entry["results"]):
                    out("agent: interrupted by human, stopping")
                    stop_reason = "interrupted by human"
                break
            if fails >= FAIL_STOP:
                out("agent: 2 consecutive failures, stopping")
                stop_reason = "2 consecutive failures"
                break
        else:
            out(f"agent: step limit reached ({max_steps}), stopping")
            stop_reason = f"step limit reached ({max_steps})"
    except KeyboardInterrupt:
        out("agent: interrupted by human, stopping")
        stop_reason = "interrupted by human"
    path = agent_chat.write_transcript(
        recordings_dir, entries, summary, out)
    out(f"agent: {step} model calls")
    if live is not None:
        live.finish(summary if finished else stop_reason or "stopped")
    return {
        "summary": summary,
        "finished": finished,
        "good": good,
        "bad": bad,
        "steps": step, "model_calls": step, "history": history,
        "transcript": path, "state": state,
    }


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
    parser.add_argument("--scan-at-station", action="store_true",
                        help="take the table overview during the station"
                        " hold and rescan from that image instead of"
                        " opening the table camera after the sort")
    parser.add_argument("--task", default=None,
                        help="first order in plain words")
    parser.add_argument("--chat", action="store_true",
                        help="keep asking for the next order")
    parser.add_argument("--plan-once", action="store_true",
                        help="let the model choose the sorting order"
                        " once, then run it without asking again"
                        " unless a rescan deviates")
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
            sort_fn = agent_tools.real_sort
            if args.scan_at_station:
                sort_fn = functools.partial(
                    agent_tools.real_sort, scan_at_station=True)
            toolset = (
                agent_tools.real_scan,
                sort_fn,
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
    page = LivePage()
    out(f"live page: {page.path} (open it in the browser)")
    if args.task or args.chat:
        result = agent_chat.run_session(
            client, *toolset, task=args.task, chat=args.chat,
            ask=input_fn if input_fn is not None else input,
            out=out, live=page, plan_once=args.plan_once)
    else:
        result = run_agent(client, *toolset, out=out, live=page,
                           plan_once=args.plan_once)
    return 0 if result["finished"] else 1


if __name__ == "__main__":
    sys.exit(main())
