"""Tests for find_parts.py. Synthetic images and fakes only."""

from __future__ import annotations

from types import SimpleNamespace

import cv2
import numpy

import muse_qc.vision.find_parts as find_parts
from muse_qc.vision.find_parts import crop_box_to_frame, crop_for_blob, detect_changes

SECRET = "[REDACTED]"
WIDTH, HEIGHT = 640, 480


class _FakeResponses:
    def __init__(self, texts=None, seen=None):
        self._texts = list(texts or [])
        self._seen = seen

    def create(self, **kwargs):
        if self._seen is not None:
            self._seen.append(kwargs)
        if not self._texts:
            return SimpleNamespace(output_text="")
        return SimpleNamespace(output_text=self._texts.pop(0))


class _FakeClient:
    def __init__(self, texts=None, seen=None):
        self.responses = _FakeResponses(texts, seen)


def _fake_markers():
    """Marker centres at the frame corners, 40 px sides (1 px/mm)."""
    return {
        0: {"center": (20.0, 20.0), "side_px": 40.0},
        1: {"center": (620.0, 20.0), "side_px": 40.0},
        2: {"center": (20.0, 460.0), "side_px": 40.0},
        3: {"center": (620.0, 460.0), "side_px": 40.0},
    }


def _background():
    """Light-wood textured background shared by reference and frame."""
    rng = numpy.random.default_rng(7)
    base = numpy.full((HEIGHT, WIDTH, 3), (190, 175, 150), numpy.uint8)
    noise = rng.integers(-6, 7, (HEIGHT, WIDTH, 3)).astype(numpy.int16)
    return numpy.clip(base.astype(numpy.int16) + noise, 0, 255).astype(
        numpy.uint8
    )


def _frame_with(parts):
    """Copy of the background with filled (x1, y1, x2, y2, color) rects."""
    frame = _background().copy()
    for x1, y1, x2, y2, color in parts:
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, -1)
    return frame


def _two_parts():
    """Two part rects at reachable table spots (~(505, 360), ~(567, 317)).

    Both map outside every skip zone (boxes, arm zone, reach), so
    they still reach SAM as before.
    """
    grey = (110, 110, 110)
    orange = (30, 140, 230)
    return [(307, 305, 346, 330, grey), (345, 270, 384, 295, orange)]


def _rect_at(table_xy, half=(20, 13)):
    """Filled rect centred on the pixel mapping to table_xy."""
    import numpy

    from muse_qc.vision.table_coords import table_homography

    homography = table_homography(_fake_markers(), None)
    inv = numpy.linalg.inv(numpy.array(homography, dtype=float))
    vec = inv @ numpy.array([table_xy[0], table_xy[1], 1.0])
    center_x, center_y = int(round(vec[0] / vec[2])), int(round(vec[1] / vec[2]))
    return (
        center_x - half[0], center_y - half[1],
        center_x + half[0], center_y + half[1], (110, 110, 110),
    )


def _setup(monkeypatch, texts=None, seen=None):
    """Fake markers, no calibration, fake SAM client factory."""
    monkeypatch.setattr(
        find_parts, "detect_table_markers", lambda frame: _fake_markers()
    )
    monkeypatch.setattr(find_parts, "load_table_calibration", lambda: None)
    monkeypatch.setattr(find_parts, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        find_parts, "_default_client_factory",
        lambda key: _FakeClient(texts, seen),
    )


def _write(path, image):
    assert cv2.imwrite(str(path), image)


def _run_main(monkeypatch, tmp_path, argv):
    monkeypatch.chdir(tmp_path)
    return find_parts.main(argv)


def test_blobs_found_at_right_centres(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(_two_parts()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert "part 0 center=(327, 318)" in out
    assert "part 1 center=(365, 283)" in out
    assert "candidates: 2" in out


def test_tiny_noise_and_huge_change_filtered(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    parts = _two_parts() + [
        (200, 200, 202, 202, (110, 110, 110)),
        (450, 60, 570, 180, (110, 110, 110)),
    ]
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(parts))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 0
    assert "candidates: 2" in capsys.readouterr().out


def _unskipped_grid_13():
    """13 small rects on verified unskipped table spots (5 + 5 + 3)."""
    grey = (110, 110, 110)
    rects = []
    for py in (280, 300):
        for px in (296, 318, 340, 362, 384):
            rects.append((px - 8, py - 6, px + 8, py + 6, grey))
    for px in (300, 326, 348):
        rects.append((px - 8, 320 - 6, px + 8, 320 + 6, grey))
    return rects


def test_too_many_blobs_exit_1(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(_unskipped_grid_13()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 1
    out = capsys.readouterr().out
    assert "too many changes" in out
    assert "blobs: 13 total, 0 skipped, 13 unskipped" in out
    files = sorted((tmp_path / "recordings").glob("parts_*.png"))
    assert len(files) == 1
    assert files[0].name in out
    assert out.index("blobs: 13 total") < out.index("too many changes")


def test_skipped_blobs_do_not_count_against_limit(monkeypatch, tmp_path,
                                                 capsys):
    seen: list = []
    texts = ["0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.88|>"]
    # y 230 needs x inside the gripper rect (430 to 590), and the
    # (585, 270) point keeps its distance to the (495, 270) blob.
    arm = [(x, y) for x in (420, 495, 570) for y in (90, 160)]
    arm += [(470, 230), (545, 230), (495, 270), (585, 270)]
    good = [(335, 385), (415, 385), (335, 425), (415, 425),
            (335, 465), (415, 465)]
    bad = [(640, 390), (720, 390), (640, 430), (720, 430)]
    parts = ([_rect_at((505.0, 360.0))]
             + [_rect_at((float(x), float(y))) for x, y in arm]
             + [_rect_at((float(x), float(y))) for x, y in good]
             + [_rect_at((float(x), float(y))) for x, y in bad])
    out = _run_frame(monkeypatch, tmp_path, capsys, parts,
                     texts=texts, seen=seen)
    assert len(seen) == 1
    assert "parts: 1 confirmed of 21 candidates (20 skipped)" in out


def test_limit_trip_lists_skipped_and_saves_image(monkeypatch, tmp_path,
                                                 capsys):
    _setup(monkeypatch)
    parts = _unskipped_grid_13() + [_rect_at((500.0, 150.0))]
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(parts))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 1
    out = capsys.readouterr().out
    assert "too many changes" in out
    assert "blobs: 14 total, 1 skipped, 13 unskipped" in out
    files = sorted((tmp_path / "recordings").glob("parts_*.png"))
    assert len(files) == 1
    assert files[0].name in out


def test_missing_reference_exit_2(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    _write(tmp_path / "frame.png", _frame_with(_two_parts()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "gone.png"), "--no-sam",
    ])
    assert code == 2
    assert "missing empty reference" in capsys.readouterr().out


def test_size_mismatch_exit_2(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    small = cv2.resize(_background(), (320, 240))
    _write(tmp_path / "empty.png", small)
    _write(tmp_path / "frame.png", _frame_with(_two_parts()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 2
    assert "size mismatch" in capsys.readouterr().out


def test_crop_clipped_at_frame_edge():
    x0, y0, side = crop_for_blob((WIDTH, HEIGHT), (5.0, 5.0), (20, 25))
    assert (x0, y0) == (0, 0)
    assert side == 160
    x0, y0, side = crop_for_blob((WIDTH, HEIGHT), (120.0, 162.0), (40, 25))
    assert side == 160
    assert (x0, y0) == (40, 82)


def test_crop_box_mapped_back_to_frame():
    origin, scale = (40, 82), 640 / 160
    box = {"x1": 270, "y1": 270, "x2": 370, "y2": 370}
    frame_x, frame_y = crop_box_to_frame(origin, scale, box)
    assert (frame_x, frame_y) == (120.0, 162.0)


def test_confirmed_and_no_box(monkeypatch, tmp_path, capsys):
    texts = [
        "0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.81|>", "",
    ]
    _setup(monkeypatch, texts=texts)
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(_two_parts()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"),
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert "sam=confirmed 0.81" in out
    assert "sam=no box" in out
    assert "parts: 1 confirmed of 2 candidates" in out
    assert SECRET not in out


def test_no_sam_makes_no_client_call(monkeypatch, tmp_path, capsys):
    seen: list = []
    _setup(monkeypatch, seen=seen)
    monkeypatch.setattr(
        find_parts, "_default_client_factory",
        lambda key: (_ for _ in ()).throw(AssertionError("client built")),
    )
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(_two_parts()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert seen == []
    assert "sam=skipped" in out
    assert "candidates: 2" in out


def test_save_empty_writes_file(monkeypatch, tmp_path, capsys):
    frame = _frame_with(_two_parts())
    monkeypatch.setattr(
        find_parts, "open_table_camera", lambda: (frame, {})
    )
    code = _run_main(monkeypatch, tmp_path, ["--save-empty"])
    assert code == 0
    saved = tmp_path / "recordings" / "empty_table.png"
    assert saved.is_file()
    assert "empty_table.png" in capsys.readouterr().out
    reread = cv2.imread(str(saved))
    assert reread is not None
    assert reread.shape == frame.shape


def test_annotated_image_saved_and_sorted(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(_two_parts()))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 0
    out = capsys.readouterr().out
    assert out.index("part 0") < out.index("part 1")
    files = sorted((tmp_path / "recordings").glob("parts_*.png"))
    assert len(files) == 1
    assert files[0].name in out
    assert cv2.imread(str(files[0])) is not None


def test_detect_changes_pure_without_table_mapping():
    blobs = detect_changes(
        _frame_with(_two_parts()), _background(), 1.0, None
    )
    assert len(blobs) == 2
    centres = sorted(blob["center"] for blob in blobs)
    assert centres[0] == (327.0, 318.0)
    assert centres[1] == (365.0, 283.0)


def test_classify_skip_zones():
    assert find_parts.classify_skip(None) is None
    assert find_parts.classify_skip((376.4, 423.8)) == "in box good"
    assert find_parts.classify_skip((390.9, 451.5)) == "in box good"
    assert find_parts.classify_skip((675.4, 420.3)) == "in box bad"
    assert find_parts.classify_skip((500.0, 150.0)) == "arm zone"
    assert find_parts.classify_skip((100.0, 450.0)) == "out of reach"
    assert find_parts.classify_skip((505.0, 360.0)) is None
    assert find_parts.classify_skip((567.0, 317.0)) is None


def test_arm_zone_split_leaves_gap_for_parts():
    """Narrowed arm zone: (389, 270) passes, gripper blobs still skip."""
    assert find_parts.classify_skip((389.0, 270.0)) is None
    assert find_parts.classify_skip((535.0, 253.0)) == "arm zone"
    assert find_parts.classify_skip((604.0, 169.0)) == "arm zone"
    assert find_parts.classify_skip((500.0, 150.0)) == "arm zone"


def _run_frame(monkeypatch, tmp_path, capsys, parts, texts=None, seen=None):
    _setup(monkeypatch, texts=texts, seen=seen)
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(parts))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"),
    ])
    assert code == 0
    return capsys.readouterr().out


def test_skip_good_box_no_sam_call(monkeypatch, tmp_path, capsys):
    seen: list = []
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((376.4, 423.8))], seen=seen)
    assert seen == []
    assert "skipped: in box good" in out
    assert "parts: 0 confirmed of 1 candidates (1 skipped)" in out


def test_skip_bad_box_no_sam_call(monkeypatch, tmp_path, capsys):
    seen: list = []
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((675.4, 420.3))], seen=seen)
    assert seen == []
    assert "skipped: in box bad" in out
    assert "parts: 0 confirmed of 1 candidates (1 skipped)" in out


def test_skip_arm_zone_no_sam_call(monkeypatch, tmp_path, capsys):
    seen: list = []
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((500.0, 150.0))], seen=seen)
    assert seen == []
    assert "skipped: arm zone" in out
    assert "parts: 0 confirmed of 1 candidates (1 skipped)" in out


def test_skip_out_of_reach_no_sam_call(monkeypatch, tmp_path, capsys):
    seen: list = []
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((100.0, 450.0))], seen=seen)
    assert seen == []
    assert "skipped: out of reach" in out
    assert "parts: 0 confirmed of 1 candidates (1 skipped)" in out


def test_real_candidate_still_goes_to_sam(monkeypatch, tmp_path, capsys):
    seen: list = []
    texts = ["0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.88|>"]
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((505.0, 360.0))], texts=texts, seen=seen)
    assert len(seen) == 1
    assert "sam=confirmed 0.88" in out
    assert "parts: 1 confirmed of 1 candidates" in out
    assert "skipped" not in out


def _phrases(seen):
    """Noun phrases of the SAM calls recorded in seen."""
    return [call["input"][0]["content"][0]["text"] for call in seen]


def test_retry_small_part_confirms(monkeypatch, tmp_path, capsys):
    seen: list = []
    texts = ["",
             "0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.53|>"]
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((505.0, 360.0), half=(15, 10))],
                     texts=texts, seen=seen)
    assert len(seen) == 2
    assert _phrases(seen) == ["grey object", "small part"]
    assert "sam=confirmed 0.53 (small part)" in out
    assert "parts: 1 confirmed of 1 candidates" in out


def test_retry_skipped_outside_area(monkeypatch, tmp_path, capsys):
    seen: list = []
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((505.0, 360.0))], texts=[""], seen=seen)
    assert len(seen) == 1
    assert "sam=no box" in out
    assert "(small part)" not in out
    assert "parts: 0 confirmed of 1 candidates" in out


def test_no_retry_when_first_confirms(monkeypatch, tmp_path, capsys):
    seen: list = []
    texts = ["0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.88|>"]
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((505.0, 360.0), half=(15, 10))],
                     texts=texts, seen=seen)
    assert len(seen) == 1
    assert "sam=confirmed 0.88" in out
    assert "(small part)" not in out
    assert "parts: 1 confirmed of 1 candidates" in out


def test_skipped_summary_count(monkeypatch, tmp_path, capsys):
    seen: list = []
    texts = ["0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.88|>"]
    parts = [
        _rect_at((390.9, 451.5)),
        _rect_at((500.0, 150.0)),
        _rect_at((100.0, 450.0)),
        _rect_at((675.4, 420.3)),
        _rect_at((505.0, 360.0)),
    ]
    out = _run_frame(monkeypatch, tmp_path, capsys, parts,
                     texts=texts, seen=seen)
    assert len(seen) == 1
    assert "skipped: in box good" in out
    assert "skipped: in box bad" in out
    assert "skipped: arm zone" in out
    assert "skipped: out of reach" in out
    assert "parts: 1 confirmed of 5 candidates (4 skipped)" in out


def _scan_direct(monkeypatch, tmp_path, parts, texts=None, seen=None,
                 known=None):
    """Run scan_table on a synthetic frame, return (results, seen)."""
    monkeypatch.setattr(find_parts, "detect_table_markers",
                        lambda frame: _fake_markers())
    monkeypatch.setattr(find_parts, "load_table_calibration", lambda: None)
    monkeypatch.chdir(tmp_path)
    results, _ = find_parts.scan_table(
        _frame_with(parts), _background(), client=_FakeClient(texts, seen),
        recordings_dir=str(tmp_path), known=known)
    return results


def test_known_match_limit_is_10_mm():
    from muse_qc.vision.known_match import KNOWN_MATCH_MM, match_known

    assert KNOWN_MATCH_MM == 10.0
    assert match_known([{"table_mm": [505.0, 360.0], "confidence": 0.77}],
                       (505.0, 360.0)) == (True, 0.77, (505.0, 360.0))
    assert match_known([{"table_mm": [505.0, 360.0], "confidence": 0.77}],
                       (515.0, 360.0)) == (True, 0.77, (505.0, 360.0))
    assert match_known([[505.0, 360.0]],
                       (505.0, 360.0)) == (True, None, (505.0, 360.0))
    assert match_known([{"table_mm": [505.0, 360.0], "confidence": 0.77}],
                       (520.0, 360.0)) == (False, None, None)
    assert match_known(None, (505.0, 360.0)) == (False, None, None)


def test_known_candidate_makes_no_sam_call(monkeypatch, tmp_path):
    seen: list = []
    results = _scan_direct(
        monkeypatch, tmp_path, [_rect_at((505.0, 360.0))], seen=seen,
        known=[{"table_mm": [505.0, 360.0], "confidence": 0.77}])
    assert seen == []
    (item,) = [part for part in results if part["skipped"] is None]
    assert item["confirmed"] is True
    assert item["confidence"] == 0.77
    assert item["known"] is True
    assert item["retried"] is False
    assert "lever_deg" in item


def test_candidate_15_mm_away_goes_to_sam(monkeypatch, tmp_path):
    seen: list = []
    texts = ["0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.88|>"]
    results = _scan_direct(
        monkeypatch, tmp_path, [_rect_at((505.0, 360.0))],
        texts=texts, seen=seen,
        known=[{"table_mm": [520.0, 360.0], "confidence": 0.5}])
    assert len(seen) == 1
    (item,) = [part for part in results if part["skipped"] is None]
    assert item["confirmed"] is True
    assert item["confidence"] == 0.88
    assert item["known"] is False


def test_known_none_behaves_as_today(monkeypatch, tmp_path):
    seen: list = []
    texts = ["0<|box;x1=270;y1=270;x2=370;y2=370;w=100;h=100;c=0.88|>"]
    results = _scan_direct(
        monkeypatch, tmp_path, [_rect_at((505.0, 360.0))],
        texts=texts, seen=seen, known=None)
    assert len(seen) == 1
    (item,) = [part for part in results if part["skipped"] is None]
    assert item["confirmed"] is True
    assert item["known"] is False


def test_known_candidate_keeps_known_table_mm(monkeypatch, tmp_path):
    seen: list = []
    results = _scan_direct(
        monkeypatch, tmp_path, [_rect_at((505.0, 360.0))], seen=seen,
        known=[{"table_mm": [511.0, 360.0], "confidence": 0.77}])
    assert seen == []
    (item,) = [part for part in results if part["skipped"] is None]
    assert item["confirmed"] is True
    assert item["table_mm"] == [511.0, 360.0]
    assert item["confidence"] == 0.77


def test_known_printed_as_sam_known(monkeypatch, tmp_path, capsys):
    real_scan_table = find_parts.scan_table

    def with_known(frame, empty, phrase=None, client=None,
                   recordings_dir="recordings", known=None):
        return real_scan_table(
            frame, empty, phrase, client, recordings_dir,
            known=[{"table_mm": [505.0, 360.0], "confidence": 0.77}])

    monkeypatch.setattr(find_parts, "scan_table", with_known)
    out = _run_frame(monkeypatch, tmp_path, capsys,
                     [_rect_at((505.0, 360.0))])
    assert "sam=known" in out
    assert "parts: 1 confirmed of 1 candidates" in out


def test_full_scan_saves_mapping(monkeypatch, tmp_path):
    import json

    _scan_direct(monkeypatch, tmp_path, _two_parts())
    mapping = tmp_path / "table_mapping_last.json"
    assert mapping.is_file()
    payload = json.loads(mapping.read_text(encoding="utf-8"))
    assert payload["marker_ids"] == [0, 1, 2, 3]
    assert payload["stamp"]


def _scan_without_marker_0(monkeypatch, tmp_path, known):
    """Run scan_table with marker 0 covered, return (results, out)."""
    partial = {key: value for key, value in _fake_markers().items()
               if key != 0}
    monkeypatch.setattr(find_parts, "detect_table_markers",
                        lambda frame: partial)
    monkeypatch.setattr(find_parts, "load_table_calibration", lambda: None)
    results, _ = find_parts.scan_table(
        _frame_with(_two_parts()), _background(), client=None,
        recordings_dir=str(tmp_path), known=known)
    return results


def test_missing_marker_uses_saved_mapping(monkeypatch, tmp_path, capsys):
    full = _scan_direct(monkeypatch, tmp_path, _two_parts())
    points = [dict(table_mm=list(part["table_mm"]), confidence=0.9)
              for part in full if part["skipped"] is None]
    assert points
    capsys.readouterr()
    results = _scan_without_marker_0(monkeypatch, tmp_path, points)
    assert [part["table_mm"] for part in results if part.get("known")] == [
        point["table_mm"] for point in points]
    out = capsys.readouterr().out
    assert "table mapping: from last full scan" in out


def test_missing_marker_without_saved_mapping_stays_unavailable(
        monkeypatch, tmp_path):
    import pytest

    monkeypatch.chdir(tmp_path)
    with pytest.raises(RuntimeError, match="table mapping: unavailable"):
        _scan_without_marker_0(monkeypatch, tmp_path, known=None)


def test_stale_mapping_stays_unavailable(monkeypatch, tmp_path):
    import pytest
    from datetime import datetime, timedelta

    from muse_qc.vision import table_mapping
    from muse_qc.vision.table_coords import table_homography

    old = datetime.now() - timedelta(hours=13)
    table_mapping.save_last_mapping(
        table_homography(_fake_markers(), None), _fake_markers(), 1.0,
        tmp_path / "table_mapping_last.json", now=lambda: old)
    monkeypatch.chdir(tmp_path)
    with pytest.raises(RuntimeError, match="table mapping: unavailable"):
        _scan_without_marker_0(monkeypatch, tmp_path, known=None)
