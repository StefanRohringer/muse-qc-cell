"""Response item helpers for the agent loop. No hardware, no API.

Reads message text and function calls from dict or SDK object
items, converts every output item to a plain dict for the replay
history (reasoning items with encrypted_content included: with
store=False they must be sent back), and formats the token usage
line. The transcript never sees these dicts; it only holds model
text, tool calls and tool results.
"""

from __future__ import annotations


def field(item, name, default=None):
    """Read a field from a dict or an SDK object item."""
    if isinstance(item, dict):
        return item.get(name, default)
    return getattr(item, name, default)


def message_text(item) -> str:
    """Return the joined output_text chunks of a message item."""
    texts = []
    for chunk in field(item, "content", []) or []:
        kind = field(chunk, "type", "")
        if isinstance(kind, str) and kind.endswith("_text"):
            text = field(chunk, "text", "")
            if text:
                texts.append(str(text))
    return "\n".join(texts)


def iter_calls(output):
    """Yield {call_id, name, arguments} for each function_call item."""
    for item in output:
        if field(item, "type") == "function_call":
            yield {
                "call_id": field(item, "call_id"),
                "name": field(item, "name"),
                "arguments": field(item, "arguments", "") or "",
            }


def _plain_value(value):
    """Convert nested SDK objects to plain JSON-safe values."""
    if isinstance(value, dict):
        return {key: _plain_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_value(item) for item in value]
    dump = getattr(value, "model_dump", None)
    if callable(dump):
        try:
            data = dump(exclude_none=True)
        except TypeError:
            try:
                data = dump()
            except Exception:  # noqa: BLE001 - fall through below
                data = None
        if isinstance(data, dict):
            return {key: _plain_value(item)
                    for key, item in data.items()}
    if hasattr(value, "__dict__") and not isinstance(
            value, (str, bytes, type)):
        try:
            members = vars(value)
        except TypeError:
            return value
        return {key: _plain_value(item) for key, item in members.items()
                if not key.startswith("_")}
    return value


def plain_item(item) -> dict:
    """Return the output item as a plain dict for the history.

    Never fails: SDK objects convert via model_dump, dicts pass
    through as copies, anything else keeps its readable fields.
    """
    try:
        converted = _plain_value(item)
    except Exception:  # noqa: BLE001 - history must never break
        converted = None
    if isinstance(converted, dict):
        return converted
    return {"type": field(item, "type", "unknown")}


def usage_text(response) -> str | None:
    """Return the token usage line, or None when usage is missing."""
    try:
        usage = field(response, "usage", None)
        if usage is None:
            return None
        tokens_in = field(usage, "input_tokens", None)
        tokens_out = field(usage, "output_tokens", None)
        if tokens_in is None or tokens_out is None:
            return None
        return f"agent: tokens in {tokens_in} out {tokens_out}"
    except Exception:  # noqa: BLE001 - usage must never break the run
        return None
