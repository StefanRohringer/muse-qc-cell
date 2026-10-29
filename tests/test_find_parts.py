"""Tests for find_parts.py. Synthetic images and fakes only."""

from __future__ import annotations

from types import SimpleNamespace

import cv2
import numpy

import muse_qc.find_parts as find_parts
from muse_qc.find_parts import crop_box_to_frame, crop_for_blob, detect_changes

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
    grey = (110, 110, 110)
    orange = (30, 140, 230)
    return [(100, 150, 140, 175, grey), (400, 300, 430, 320, orange)]


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
    assert "part 0 center=(120, 163)" in out
    assert "part 1 center=(416, 310)" in out
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


def test_too_many_blobs_exit_1(monkeypatch, tmp_path, capsys):
    _setup(monkeypatch)
    parts = [
        (20 + i * 45, 100, 40 + i * 45, 120, (110, 110, 110))
        for i in range(13)
    ]
    _write(tmp_path / "empty.png", _background())
    _write(tmp_path / "frame.png", _frame_with(parts))
    code = _run_main(monkeypatch, tmp_path, [
        "--image", str(tmp_path / "frame.png"),
        "--empty", str(tmp_path / "empty.png"), "--no-sam",
    ])
    assert code == 1
    assert "too many changes" in capsys.readouterr().out


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
    assert centres[0] == (120.5, 163.0)
    assert centres[1] == (415.5, 310.5)
