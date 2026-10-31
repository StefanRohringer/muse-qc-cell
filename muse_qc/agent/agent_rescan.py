"""Rescan after a sorted part, moved out of muse_qc.agent.agent_tools.

Keeps that module within the 400 line limit. Adds the station
image path: when the sort ran with --station-table-shot, the table
overview taken during the station hold is scanned instead of
opening the table camera again after the arm is home.
"""

from __future__ import annotations

from pathlib import Path


def store_scan(state, found) -> dict:
    """Store a scan in state under new ids; return the tool result part.

    Same shape as a scan_table call: {"parts", "skipped", "image"}
    with one {"id", "table_mm", "confidence", "lever_deg"} dict
    per part. The state is fresh again under new ids.
    """
    items = found.get("parts", [])
    positions = [part["table_mm"] for part in items]
    state.new_scan(
        positions,
        [part.get("lever_deg") for part in items],
        [part.get("confidence") for part in items],
    )
    skipped = int(found.get("skipped", 0) or 0)
    return {"parts": [
        {"id": index,
         "table_mm": [float(pos[0]), float(pos[1])],
         "confidence": part.get("confidence"),
         "lever_deg": (None if part.get("lever_deg") is None
                       else round(float(part["lever_deg"]), 1))}
        for index, (pos, part) in enumerate(zip(positions, items))
    ], "skipped": skipped, "image": found.get("image")}


def scan_frame(frame, known=None) -> dict:
    """Scan one table frame (camera plus SAM), return the found dict.

    Same empty reference, phrase and known= handling as the live
    scan: known holds the confirmed points of the previous scan.
    Raises RuntimeError with the user-facing message.
    """
    import cv2

    from muse_qc.vision import find_parts
    from muse_qc.checks.model_check import KEY_NAME, read_api_key
    from muse_qc.vision.vision_sam import _default_client_factory

    empty = cv2.imread(find_parts.EMPTY_PATH)
    if empty is None:
        raise RuntimeError(
            f"missing empty reference {find_parts.EMPTY_PATH}:"
            " take one with --save-empty")
    key = read_api_key()
    if not (key or "").strip():
        raise RuntimeError(
            f"{KEY_NAME} is missing: set it in .env or the environment")
    try:
        client = _default_client_factory(key)
    except Exception as exc:  # noqa: BLE001 - mapped, key never printed
        raise RuntimeError(
            f"request failed: {exc}") from exc
    results, path = find_parts.scan_table(
        frame, empty, find_parts.DEFAULT_PHRASE, client,
        known=known)
    parts = [
        {"table_mm": [float(item["table_mm"][0]),
                      float(item["table_mm"][1])],
         "confidence": item["confidence"],
         "lever_deg": item["lever_deg"]}
        for item in results
        if item["skipped"] is None and item["confirmed"]
    ]
    skipped = sum(1 for item in results if item["skipped"] is not None)
    return {"parts": parts, "skipped": skipped, "image": str(path)}


def read_station_image(station_image):
    """Read the saved station overview; None when missing or unreadable."""
    import cv2

    if not station_image:
        return None
    if not Path(station_image).is_file():
        return None
    return cv2.imread(str(station_image))


def _store_found(state, found, out) -> dict:
    """Store a rescan result under new ids; return the result update."""
    items = found.get("parts", []) if isinstance(found, dict) else []
    if not items:
        state.mark_stale()
        out("tool: rescan found no parts, call scan_table")
        return {"table after sort": [],
                "rescan": "no parts found, call scan_table"}
    result = store_scan(state, found)
    count = len(result["parts"])
    if count == 1:
        out("tool: rescan found 1 part")
    else:
        out(f"tool: rescan found {count} parts")
    return {"table after sort": result["parts"],
            "image after sort": result["image"]}


def rescan_after_sort(state, scan_fn, out=print,
                      station_image=None) -> dict:
    """Rescan right after a sorted part; return the result update.

    The pick_test child has ended (arm at rest, motors off), so
    the scan runs at once with known = the confirmed points of
    the previous scan: known parts skip SAM, a slipped part gets
    its new position. The new scan is stored under new ids like
    a scan_table call and returned as "table after sort" with
    the same part fields. When the rescan fails or finds
    nothing, the scan expires and the update says so; the model
    then calls scan_table as usual. Prints nothing but the
    one-line rescan outcome through out.

    With station_image (the "station table shot:" path of a sort
    that ran with --station-table-shot) the saved overview taken
    during the station hold is scanned instead of opening the
    table camera, and one line names the source used. Without it
    the behaviour is exactly the camera rescan above. A missing
    or unreadable station image falls back to the camera rescan.
    """
    known = state.known_parts()
    if station_image is not None:
        frame = read_station_image(station_image)
        if frame is None:
            out("tool: rescan from table camera")
        else:
            out("tool: rescan from station image")
            try:
                found = scan_frame(frame, known)
            except Exception as exc:  # noqa: BLE001 - short, no key
                state.mark_stale()
                out(f"tool: rescan failed: {exc}")
                return {"rescan": f"failed: {exc}, call scan_table"}
            return _store_found(state, found, out)
    try:
        found = scan_fn(known=known)
    except Exception as exc:  # noqa: BLE001 - short, no key
        state.mark_stale()
        out(f"tool: rescan failed: {exc}")
        return {"rescan": f"failed: {exc}, call scan_table"}
    return _store_found(state, found, out)
