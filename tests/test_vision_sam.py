"""Tests for vision_sam.py. Offline only: client and camera are faked."""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy

import muse_qc.vision_sam as vision_sam
from muse_qc.table_coords import pixel_to_table_mm, table_homography
from muse_qc.vision_sam import image_to_data_url, parse_output_text, run_detection

SECRET = "sk-test-secret-never-print-sam456"

ONE_BOX = "1<|box;x1=100;y1=200;x2=300;y2=400;w=200;h=200;c=0.95|>"


class _FakeResponses:
    """Fake client.responses with a canned answer or a canned failure."""

    def __init__(self, output_text="", error=None, seen=None):
        self._output_text = output_text
        self._error = error
        self._seen = seen

    def create(self, **kwargs):
        if self._seen is not None:
            self._seen.append(kwargs)
        if self._error is not None:
            raise self._error
        return SimpleNamespace(output_text=self._output_text)


class _FakeClient:
    def __init__(self, output_text="", error=None, seen=None):
        self.responses = _FakeResponses(output_text, error, seen)


class _FakeStatusError(Exception):
    def __init__(self, message, status_code):
        super().__init__(message)
        self.status_code = status_code


class _FakeConnectionError(Exception):
    pass


def _frame():
    frame = numpy.zeros((120, 160, 3), dtype=numpy.uint8)
    frame[40:80, 50:110] = (200, 200, 200)
    return frame


def _real_sam_files():
    """sam_*.png files in the real recordings/ folder (tests add none)."""
    return set(
        (Path(__file__).resolve().parent.parent / "recordings").glob("sam_*.png")
    )


def test_parse_one_box():
    (box,) = parse_output_text(ONE_BOX)
    assert (box["id"], box["x1"], box["y1"], box["x2"], box["y2"]) == (
        1, 100, 200, 300, 400,
    )
    assert box["center"] == (200.0, 300.0)
    assert box["confidence"] == 0.95


def test_parse_several_boxes_with_sparse_ids():
    text = (
        "2<|box;x1=10;y1=10;x2=20;y2=30;w=10;h=20;c=0.7|>,"
        "7<|box;x1=40;y1=50;x2=60;y2=90;w=20;h=40;c=0.8|>"
    )
    boxes = parse_output_text(text)
    assert [box["id"] for box in boxes] == [2, 7]
    assert boxes[1]["center"] == (50.0, 70.0)


def test_parse_fields_by_key_not_order():
    text = "3<|box;h=40;w=20;y2=90;x2=60;y1=50;x1=40;c=0.5|>"
    (box,) = parse_output_text(text)
    assert (box["x1"], box["y1"], box["x2"], box["y2"]) == (40, 50, 60, 90)
    assert box["confidence"] == 0.5


def test_parse_box_without_confidence():
    text = "4<|box;x1=1;y1=2;x2=3;y2=4;w=2;h=2|>"
    (box,) = parse_output_text(text)
    assert box["confidence"] is None


def test_parse_ignores_mask_segment():
    text = ONE_BOX + "<|mask;x=0;y=0;data=46,101,~0R$k9|>"
    (box,) = parse_output_text(text)
    assert box["id"] == 1
    assert box["center"] == (200.0, 300.0)


def test_parse_header_with_two_records_and_masks():
    text = (
        "<0f>1<|box;x1=100;y1=200;x2=300;y2=400;w=200;h=200;c=0.95|>"
        "<|mask;x=0;y=0;data=46,101,~0R$k9|>,"
        "5<|box;x1=10;y1=10;x2=20;y2=30;w=10;h=20|>"
        "<|mask;x=0;y=0;data=12,~3P$z|>"
    )
    boxes = parse_output_text(text)
    assert [box["id"] for box in boxes] == [1, 5]
    assert boxes[0]["confidence"] == 0.95
    assert boxes[1]["confidence"] is None


def test_parse_header_and_empty_output():
    (box,) = parse_output_text("<0f>" + ONE_BOX)
    assert box["id"] == 1
    assert parse_output_text("") == []
    assert parse_output_text("   ") == []


def test_request_payload_shape(tmp_path):
    before = _real_sam_files()
    seen: list[dict] = []
    client = _FakeClient(output_text=ONE_BOX, seen=seen)
    code = run_detection(
        SECRET, "grey connector", _frame(), lambda key: client,
        recordings_dir=tmp_path,
    )
    assert code == 0
    assert _real_sam_files() == before
    assert len(seen) == 1
    payload = seen[0]
    assert payload["model"] == "sam-3.1"
    (message,) = payload["input"]
    assert message["role"] == "user"
    kinds = {part["type"]: part for part in message["content"]}
    assert kinds["input_text"]["text"] == "grey connector"
    assert kinds["input_image"]["image_url"].startswith(
        "data:image/jpeg;base64,"
    )
    assert payload["metadata"] == {"include_confidence": "true"}
    assert isinstance(payload["metadata"]["include_confidence"], str)


def test_image_is_jpeg_data_url():
    url = image_to_data_url(_frame())
    assert url.startswith("data:image/jpeg;base64,")
    assert len(url) > len("data:image/jpeg;base64,")


def test_detection_prints_boxes_and_saves_png(tmp_path, capsys):
    before = _real_sam_files()
    client = _FakeClient(output_text=ONE_BOX)
    code = run_detection(
        SECRET, "grey connector", _frame(), lambda key: client,
        recordings_dir=tmp_path / "recordings",
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "id 1" in out
    assert "center=(200.0, 300.0)" in out
    assert "0.95" in out
    assert "boxes: 1" in out
    files = sorted((tmp_path / "recordings").glob("sam_*.png"))
    assert len(files) == 1
    assert files[0].name in out
    saved = cv2.imread(str(files[0]))
    assert saved is not None
    assert (saved.shape[1], saved.shape[0]) == (160, 120)
    assert _real_sam_files() == before


def test_zero_boxes_is_not_an_error(tmp_path, capsys):
    before = _real_sam_files()
    client = _FakeClient(output_text="")
    code = run_detection(
        SECRET, "grey connector", _frame(), lambda key: client,
        recordings_dir=tmp_path / "recordings",
    )
    assert code == 0
    assert "boxes: 0" in capsys.readouterr().out
    assert _real_sam_files() == before


def test_missing_key_exits_without_client(capsys):
    seen: list[str] = []
    code = run_detection(
        None, "grey connector", _frame(), lambda key: seen.append(key)
    )
    assert code != 0
    assert seen == []
    assert "MODEL_API_KEY" in capsys.readouterr().out


def test_bad_request_prints_prompt_rejected(capsys):
    error = _FakeStatusError("phrase too vague", 400)
    code = run_detection(
        SECRET, "thing", _frame(), lambda key: _FakeClient(error=error)
    )
    assert code != 0
    out = capsys.readouterr().out
    assert "prompt rejected" in out
    assert "phrase too vague" in out
    assert "Traceback" not in out


def test_auth_errors_print_key_rejected(capsys):
    for status in (401, 403):
        error = _FakeStatusError("bad key", status)
        code = run_detection(
            SECRET, "grey connector", _frame(),
            lambda key: _FakeClient(error=error),
        )
        assert code != 0
        out = capsys.readouterr().out
        assert "key rejected" in out
        assert "Traceback" not in out


def test_network_error_prints_short_reason(capsys):
    error = _FakeConnectionError("no route")
    code = run_detection(
        SECRET, "grey connector", _frame(),
        lambda key: _FakeClient(error=error),
    )
    assert code != 0
    out = capsys.readouterr().out
    assert "network error" in out
    assert "Traceback" not in out


def test_key_never_appears_in_output(tmp_path, capsys):
    before = _real_sam_files()
    run_detection(
        SECRET, "grey connector", _frame(),
        lambda key: _FakeClient(output_text=ONE_BOX),
        recordings_dir=tmp_path,
    )
    run_detection(
        SECRET, "grey connector", _frame(),
        lambda key: _FakeClient(error=_FakeStatusError("bad key", 401)),
        recordings_dir=tmp_path,
    )
    captured = capsys.readouterr()
    assert SECRET not in captured.out
    assert SECRET not in captured.err
    assert _real_sam_files() == before


def test_main_uses_saved_image(monkeypatch, tmp_path, capsys):
    image_path = tmp_path / "table.png"
    assert cv2.imwrite(str(image_path), _frame())
    monkeypatch.setattr(vision_sam, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        vision_sam, "_default_client_factory",
        lambda key: _FakeClient(output_text=ONE_BOX),
    )
    monkeypatch.chdir(tmp_path)
    code = vision_sam.main(["grey connector", "--image", str(image_path)])
    assert code == 0
    out = capsys.readouterr().out
    assert "boxes: 1" in out
    assert len(sorted((tmp_path / "recordings").glob("sam_*.png"))) == 1


def _fake_markers():
    """Return marker centres at the corners of the fake frame."""
    return {
        0: {"center": (10.0, 10.0)},
        1: {"center": (150.0, 10.0)},
        2: {"center": (10.0, 110.0)},
        3: {"center": (150.0, 110.0)},
    }


def _zero_distortion_calibration():
    """Return a calibration record with zero distortion (plain lists)."""
    return {
        "camera_matrix": [
            [1000.0, 0.0, 80.0],
            [0.0, 1000.0, 60.0],
            [0.0, 0.0, 1.0],
        ],
        "dist_coeffs": [[0.0, 0.0, 0.0, 0.0, 0.0]],
        "image_size": [160, 120],
        "rms_px": 0.1,
        "images_used": ["calib_test_00.png"],
        "created": "2026-10-28",
    }


def _expected_table_mm(markers, calibration):
    """Map the ONE_BOX centre through the real table mapping."""
    homography = table_homography(markers, calibration)
    x_mm, y_mm = pixel_to_table_mm(
        homography, (200.0, 300.0), calibration
    )
    return f"table_mm=({x_mm:.1f}, {y_mm:.1f})"


def test_table_mm_calibrated(tmp_path, capsys):
    markers = _fake_markers()
    calibration = _zero_distortion_calibration()
    client = _FakeClient(output_text=ONE_BOX)
    code = run_detection(
        SECRET, "small part", _frame(), lambda key: client,
        recordings_dir=tmp_path,
        load_calibration=lambda: calibration,
        detect_markers=lambda frame: markers,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "table mapping: calibrated" in out
    assert out.index("table mapping:") < out.index("id 1")
    assert _expected_table_mm(markers, calibration) in out
    assert "boxes: 1" in out
    assert len(sorted(tmp_path.glob("sam_*.png"))) == 1


def test_table_mm_raw_pixels_without_calibration(tmp_path, capsys):
    markers = _fake_markers()
    client = _FakeClient(output_text=ONE_BOX)
    code = run_detection(
        SECRET, "small part", _frame(), lambda key: client,
        recordings_dir=tmp_path,
        load_calibration=lambda: None,
        detect_markers=lambda frame: markers,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "table mapping: raw pixels" in out
    assert _expected_table_mm(markers, None) in out
    assert len(sorted(tmp_path.glob("sam_*.png"))) == 1


def test_table_mm_unavailable_when_markers_missing(tmp_path, capsys):
    client = _FakeClient(output_text=ONE_BOX)
    code = run_detection(
        SECRET, "small part", _frame(), lambda key: client,
        recordings_dir=tmp_path,
        load_calibration=lambda: None,
        detect_markers=lambda frame: {},
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "table mapping: unavailable" in out
    assert "table_mm=" not in out
    assert "id 1" in out
    assert "boxes: 1" in out
    assert len(sorted(tmp_path.glob("sam_*.png"))) == 1


def test_table_mm_unavailable_when_detector_raises(tmp_path, capsys):
    def _raise(frame):
        raise RuntimeError("missing table markers: [0, 1, 2, 3]")

    client = _FakeClient(output_text=ONE_BOX)
    code = run_detection(
        SECRET, "small part", _frame(), lambda key: client,
        recordings_dir=tmp_path,
        load_calibration=lambda: None,
        detect_markers=_raise,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "table mapping: unavailable" in out
    assert "missing table markers" in out
    assert "table_mm=" not in out
    assert len(sorted(tmp_path.glob("sam_*.png"))) == 1


def test_never_imports_arm_safety_or_sdk_code():
    forbidden = ("arm", "safety", "sdk", "rebotarm", "motorbridge")
    path = Path(vision_sam.__file__)
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        else:
            continue
        for name in names:
            assert all(word not in name.lower() for word in forbidden), name
