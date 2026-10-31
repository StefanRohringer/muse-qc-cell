"""Tests for the last-good table mapping. Offline, fakes only.

No camera, no arm, no network. Timestamps use fixed datetimes
so the 12 hour age check is deterministic.
"""

from __future__ import annotations

import json
from datetime import datetime, timedelta

import numpy

from muse_qc.vision import table_mapping
from muse_qc.vision.table_markers import TABLE_MARKER_IDS

FIXED = datetime(2026, 10, 31, 23, 48, 47)


def _markers(ids=(0, 1, 2, 3)):
    return {marker_id: {} for marker_id in ids}


def test_max_age_is_12_hours():
    assert table_mapping.MAPPING_MAX_AGE_H == 12


def test_save_and_load_roundtrip(tmp_path):
    path = tmp_path / "table_mapping_last.json"
    stamp = table_mapping.save_last_mapping(
        numpy.eye(3), _markers(), 1.5, path, now=lambda: FIXED)
    assert stamp == "20261031_234847"
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["marker_ids"] == [0, 1, 2, 3]
    assert payload["stamp"] == stamp
    matrix, scale, back = table_mapping.load_last_mapping(
        path, now=lambda: FIXED)
    assert matrix.shape == (3, 3)
    assert scale == 1.5
    assert back == stamp


def test_load_missing_file_returns_none(tmp_path):
    assert table_mapping.load_last_mapping(
        tmp_path / "missing.json") is None


def test_load_corrupt_file_returns_none(tmp_path):
    bad = tmp_path / "table_mapping_last.json"
    bad.write_text("{nope", encoding="utf-8")
    assert table_mapping.load_last_mapping(bad) is None


def test_load_incomplete_ids_returns_none(tmp_path):
    path = tmp_path / "table_mapping_last.json"
    table_mapping.save_last_mapping(
        numpy.eye(3), _markers((1, 2, 3)), 1.5, path, now=lambda: FIXED)
    assert table_mapping.load_last_mapping(
        path, now=lambda: FIXED) is None


def test_load_too_old_returns_none(tmp_path):
    path = tmp_path / "table_mapping_last.json"
    table_mapping.save_last_mapping(
        numpy.eye(3), _markers(), 1.5, path, now=lambda: FIXED)
    assert table_mapping.load_last_mapping(
        path, now=lambda: FIXED + timedelta(hours=13)) is None


def test_load_at_age_limit_still_usable(tmp_path):
    path = tmp_path / "table_mapping_last.json"
    table_mapping.save_last_mapping(
        numpy.eye(3), _markers(), 1.5, path, now=lambda: FIXED)
    assert table_mapping.load_last_mapping(
        path, now=lambda: FIXED + timedelta(hours=12)) is not None


def test_saved_ids_cover_all_table_markers(tmp_path):
    path = tmp_path / "table_mapping_last.json"
    table_mapping.save_last_mapping(
        numpy.eye(3), _markers(), 1.5, path, now=lambda: FIXED)
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert all(marker_id in payload["marker_ids"]
               for marker_id in TABLE_MARKER_IDS)


def test_save_ignores_non_marker_keys(tmp_path):
    path = tmp_path / "table_mapping_last.json"
    markers = _markers()
    markers["ignored_ids"] = [7]
    stamp = table_mapping.save_last_mapping(
        numpy.eye(3), markers, 1.5, path, now=lambda: FIXED)
    assert stamp == "20261031_234847"
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["marker_ids"] == [0, 1, 2, 3]
