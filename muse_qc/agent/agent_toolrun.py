"""Execute one agent tool call; return (result, stop_loop).

Moved out of muse_qc.agent.agent so that module stays within
the 400 line limit. The scan, single sort, report and finish
paths behave exactly as before; with plan_once the model may
also call sort_in_order, which runs the whole order in Python
through muse_qc.agent.agent_plan.
"""

from __future__ import annotations

from muse_qc.agent import agent_plan, agent_tools


def run_tool(name, args, state, scan_fn, sort_fn, report_fn,
             max_sorts, max_per_position, out, live=None,
             plan_once=False) -> tuple[dict, bool]:
    """Execute one model tool call; return (result, stop_loop)."""
    if live is not None:
        live.before_tool(name, args)
    if name == "scan_table":
        out("agent -> scan_table()")
        try:
            found = scan_fn()
        except Exception as exc:  # noqa: BLE001 - short, no key
            result = {"outcome": "error",
                      "reason": f"scan failed: {exc}"}
            out(f"tool: {result['reason']}")
            return result, False
        result = agent_tools.store_scan(state, found)
        count = len(result["parts"])
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
        return agent_plan.sort_single(
            state, part_id, sort_fn, scan_fn, out, max_sorts,
            max_per_position)
    if name == "sort_in_order":
        if not plan_once:
            out(f"agent -> {name}()")
            result = {"outcome": "refused",
                      "reason": f"unknown tool {name}"}
            out(f"tool: refused: {result['reason']}")
            return result, False
        part_ids = args.get("part_ids")
        out(f"agent -> sort_in_order(parts {part_ids})")
        return agent_plan.run_plan(
            state, part_ids, sort_fn, scan_fn, out, max_sorts,
            max_per_position)
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
