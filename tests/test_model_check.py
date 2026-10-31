"""Tests for model_check.py. Offline only: the client is faked, no network."""

from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

import muse_qc.checks.model_check as model_check
from muse_qc.checks.model_check import evaluate, extract_model_ids, read_api_key, run_check

SECRET = "sk-test-secret-never-print-abc123xyz"


class _FakeModels:
    """Fake client.models with a canned answer or a canned failure."""

    def __init__(self, ids=None, error=None):
        self._ids = ids
        self._error = error

    def list(self):
        if self._error is not None:
            raise self._error
        return SimpleNamespace(data=[SimpleNamespace(id=i) for i in self._ids])


class _FakeClient:
    def __init__(self, ids=None, error=None):
        self.models = _FakeModels(ids, error)


class _FakeAuthError(Exception):
    def __init__(self, status_code):
        super().__init__("fake auth failure")
        self.status_code = status_code


class _FakeConnectionError(Exception):
    pass


def _factory(ids=None, error=None, seen=None):
    def make(key):
        if seen is not None:
            seen.append(key)
        return _FakeClient(ids, error)

    return make


def test_both_models_found_exit_zero(capsys):
    code = run_check(SECRET, _factory(ids=["muse-spark-1.3", "sam-3.1"]))
    assert code == 0
    out = capsys.readouterr().out
    assert "models: 2" in out
    assert "muse-spark-1.3: OK" in out
    assert "sam-3.1: OK" in out


def test_extra_models_are_counted(capsys):
    code = run_check(
        SECRET, _factory(ids=["muse-spark-1.3", "sam-3.1", "other-1.0"])
    )
    assert code == 0
    assert "models: 3" in capsys.readouterr().out


def test_one_model_missing_exit_nonzero(capsys):
    code = run_check(SECRET, _factory(ids=["muse-spark-1.3"]))
    assert code != 0
    out = capsys.readouterr().out
    assert "muse-spark-1.3: OK" in out
    assert "sam-3.1: MISSING" in out


def test_no_models_listed_exit_nonzero(capsys):
    code = run_check(SECRET, _factory(ids=[]))
    assert code != 0
    out = capsys.readouterr().out
    assert "models: 0" in out
    assert "MISSING" in out


def test_missing_key_exits_nonzero_without_client(capsys):
    seen: list[str] = []
    code = run_check(None, _factory(ids=["muse-spark-1.3", "sam-3.1"], seen=seen))
    assert code != 0
    assert seen == []
    assert "MODEL_API_KEY" in capsys.readouterr().out


def test_blank_key_exits_nonzero(capsys):
    assert run_check("   ", _factory(ids=[])) != 0
    assert "MODEL_API_KEY" in capsys.readouterr().out


def test_auth_errors_print_key_rejected(capsys):
    for status in (401, 403):
        code = run_check(SECRET, _factory(error=_FakeAuthError(status)))
        assert code != 0
        out = capsys.readouterr().out
        assert "key rejected" in out
        assert "Traceback" not in out


def test_network_error_prints_short_reason(capsys):
    code = run_check(SECRET, _factory(error=_FakeConnectionError("no route")))
    assert code != 0
    out = capsys.readouterr().out
    assert "network error" in out
    assert "Traceback" not in out


def test_key_never_appears_in_output(capsys):
    run_check(SECRET, _factory(ids=["muse-spark-1.3", "sam-3.1"]))
    run_check(SECRET, _factory(error=_FakeAuthError(401)))
    run_check(SECRET, _factory(error=_FakeConnectionError("down")))
    captured = capsys.readouterr()
    assert SECRET not in captured.out
    assert SECRET not in captured.err


def test_read_api_key_from_environment(tmp_path):
    missing = tmp_path / "no-such.env"
    assert read_api_key(env={"MODEL_API_KEY": "env-key"}) == "env-key"
    assert read_api_key(env={}, dotenv_path=missing) is None
    assert read_api_key(env={"MODEL_API_KEY": "  "}, dotenv_path=missing) is None


def test_read_api_key_from_dotenv_file(tmp_path):
    dotenv = tmp_path / ".env"
    dotenv.write_text('MODEL_API_KEY="file-key"\n', encoding="utf-8")
    assert read_api_key(env={}, dotenv_path=dotenv) == "file-key"


def test_environment_wins_over_dotenv_file(tmp_path):
    dotenv = tmp_path / ".env"
    dotenv.write_text("MODEL_API_KEY=file-key\n", encoding="utf-8")
    assert read_api_key(env={"MODEL_API_KEY": "env-key"}, dotenv_path=dotenv) == "env-key"


def test_extract_model_ids_supports_dicts():
    response = SimpleNamespace(data=[{"id": "muse-spark-1.3"}, {"id": ""}])
    assert extract_model_ids(response) == ["muse-spark-1.3"]


def test_evaluate_reports_each_model():
    assert evaluate(["muse-spark-1.3", "sam-3.1"]) == {
        "muse-spark-1.3": True,
        "sam-3.1": True,
    }
    assert evaluate(["sam-3.1"])["muse-spark-1.3"] is False


def test_never_imports_arm_safety_sdk_or_camera_code():
    forbidden = ("arm", "safety", "sdk", "rebotarm", "motorbridge", "camera", "cv2")
    path = Path(model_check.__file__)
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


def test_list_ids_prints_sorted_ids_one_per_line(capsys):
    code = run_check(
        SECRET,
        _factory(ids=["sam-3.1", "other-1.0", "muse-spark-1.3"]),
        list_ids=True,
    )
    assert code == 0
    out = capsys.readouterr().out
    assert out.splitlines() == ["muse-spark-1.3", "other-1.0", "sam-3.1"]
    assert SECRET not in out


def test_list_ids_empty_list_prints_nothing(capsys):
    code = run_check(SECRET, _factory(ids=[]), list_ids=True)
    assert code == 0
    assert capsys.readouterr().out == ""


def test_main_list_flag_reaches_run_check(monkeypatch, capsys):
    monkeypatch.setattr(model_check, "read_api_key", lambda: SECRET)
    monkeypatch.setattr(
        model_check,
        "_default_client_factory",
        _factory(ids=["sam-3.1", "muse-spark-1.3"]),
    )
    assert model_check.main(["--list"]) == 0
    assert capsys.readouterr().out.splitlines() == [
        "muse-spark-1.3",
        "sam-3.1",
    ]


def test_main_reports_missing_key_without_network(monkeypatch, capsys, tmp_path):
    monkeypatch.delenv("MODEL_API_KEY", raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(model_check, "read_api_key", lambda: None)
    assert model_check.main([]) != 0
    assert "MODEL_API_KEY" in capsys.readouterr().out
