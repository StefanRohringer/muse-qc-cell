"""Command line for find_parts, moved out of muse_qc.vision.find_parts.

Keeps that module within the 400 line limit. Names are read off
the find_parts module so tests can keep patching them there.
"""

from __future__ import annotations

import argparse

import cv2

from muse_qc.vision import find_parts
from muse_qc.vision.camera_top import open_table_camera
from muse_qc.checks.model_check import KEY_NAME


def main(argv=None):
    """Find table parts against the empty reference, confirm with SAM."""
    parser = argparse.ArgumentParser(
        description="Find connector parts via empty-table diff plus SAM."
    )
    parser.add_argument("--save-empty", action="store_true")
    parser.add_argument("--phrase", default=find_parts.DEFAULT_PHRASE)
    parser.add_argument("--no-sam", action="store_true")
    parser.add_argument("--image", default=None)
    parser.add_argument("--empty", default=find_parts.EMPTY_PATH)
    args = parser.parse_args(argv)
    if args.save_empty:
        return find_parts.save_empty()
    if args.image is not None:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"cannot read image {args.image}")
            return 1
    else:
        try:
            frame, _ = open_table_camera()
        except (OSError, RuntimeError) as exc:
            print(f"request failed: {exc}")
            return 1
    empty = cv2.imread(args.empty)
    if empty is None:
        print(f"missing empty reference {args.empty}:"
              " take one with --save-empty")
        return 2
    if empty.shape != frame.shape:
        print("size mismatch: take a new empty reference with --save-empty")
        return 2
    client = None
    if not args.no_sam:
        api_key = find_parts.read_api_key()
        if not api_key or not api_key.strip():
            print(f"{KEY_NAME} is missing: set it in .env or the environment")
            return 2
        try:
            client = find_parts._default_client_factory(api_key)
        except Exception as exc:  # noqa: BLE001 - mapped, key never printed
            print(find_parts._request_error(exc))
            return 1
    try:
        results, path = find_parts.scan_table(frame, empty, args.phrase,
                                              client)
    except find_parts.TooManyChanges as exc:
        skipped = sum(1 for item in exc.results if item["skipped"] is not None)
        print(f"blobs: {len(exc.results)} total, {skipped} skipped,"
              f" {len(exc.results) - skipped} unskipped")
        print(exc.path)
        print(exc)
        return 1
    except RuntimeError as exc:
        print(exc)
        return 1
    for index, item in enumerate(results):
        pixel_x, pixel_y = item["pixel"]
        table_x, table_y = item["table_mm"]
        if item["skipped"] is not None:
            print(f"part {index} center=({pixel_x:.0f}, {pixel_y:.0f})"
                  f" area_mm2={item['area_mm2']:.0f}"
                  f" table_mm=({table_x:.1f}, {table_y:.1f})"
                  f" skipped: {item['skipped']}")
            continue
        if client is None:
            sam_text = "skipped"
        elif item.get("known"):
            sam_text = "known"
        elif not item["confirmed"]:
            sam_text = "no box"
        elif item["confidence"] is None:
            sam_text = "confirmed n/a"
        else:
            sam_text = f"confirmed {item['confidence']:.2f}"
            if item.get("retried"):
                sam_text += " (small part)"
        if item["lever_deg"] is None:
            lever_text = f"lever_deg=? ({item['lever_reason']})"
        else:
            shown = float(item["lever_deg"])
            if round(shown, 1) >= 180.0:
                shown = 0.0
            lever_text = f"lever_deg={shown:.1f} ({item['lever_source']})"
        print(f"part {index} center=({pixel_x:.0f}, {pixel_y:.0f})"
              f" area_mm2={item['area_mm2']:.0f}"
              f" table_mm=({table_x:.1f}, {table_y:.1f}) sam={sam_text}"
              f" {lever_text}")
    skipped = sum(1 for item in results if item["skipped"] is not None)
    skipped_text = f" ({skipped} skipped)" if skipped else ""
    if client is None:
        print(f"candidates: {len(results)}{skipped_text}")
    else:
        done = sum(1 for item in results if item["confirmed"])
        print(f"parts: {done} confirmed of {len(results)}"
              f" candidates{skipped_text}")
    print(path)
    return 0
