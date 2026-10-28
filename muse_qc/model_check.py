"""Hand-run connection check for the Meta Model API (read-only).

Reads MODEL_API_KEY from the environment or a .env file (the key is
never printed, logged or written, not even partly), calls GET
/v1/models once through the openai SDK with base URL
https://api.meta.ai/v1, prints the number of models and one OK/MISSING
line each for muse-spark-1.3 and sam-3.1. Exit code 0 only when both
are listed. Only the human runs it, never CI and never an agent, since
it contacts the network. Imports no arm, safety, SDK or camera code.
"""

from __future__ import annotations

import os
import sys
from collections.abc import Callable, Iterable, Mapping
from pathlib import Path

BASE_URL = "https://api.meta.ai/v1"
EXPECTED_MODELS = ("muse-spark-1.3", "sam-3.1")
KEY_NAME = "MODEL_API_KEY"


def _parse_dotenv_file(path: Path) -> dict[str, str]:
    """Return KEY=VALUE pairs from a small .env file (no expansion)."""
    values: dict[str, str] = {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return values
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        name, _, value = stripped.partition("=")
        name = name.strip()
        if name.startswith("export "):
            name = name[len("export "):].strip()
        if not name:
            continue
        cleaned = value.strip().strip("\"'")
        values[name] = cleaned
    return values


def read_api_key(
    env: Mapping[str, str] | None = None,
    dotenv_path: str | os.PathLike[str] | None = None,
) -> str | None:
    """Return MODEL_API_KEY from the environment or a .env file.

    The environment wins. Without it, the .env file at dotenv_path is
    used when given, else .env in the working directory, else .env in
    the repository root. Returns None when the key is missing or blank.
    """
    mapping: Mapping[str, str] = os.environ if env is None else env
    direct = mapping.get(KEY_NAME, "")
    if direct.strip():
        return direct.strip()
    if dotenv_path is not None:
        candidates = [Path(dotenv_path)]
    else:
        repo_root = Path(__file__).resolve().parent.parent
        candidates = [Path.cwd() / ".env", repo_root / ".env"]
    for candidate in candidates:
        if candidate.is_file():
            stored = _parse_dotenv_file(candidate).get(KEY_NAME, "")
            if stored.strip():
                return stored.strip()
            return None
    return None


def extract_model_ids(response: object) -> list[str]:
    """Return model IDs from a models.list() response (objects or dicts)."""
    data = getattr(response, "data", response)
    if not isinstance(data, Iterable) or isinstance(data, (str, bytes)):
        return []
    ids: list[str] = []
    for item in data:
        if isinstance(item, dict):
            found = item.get("id")
        else:
            found = getattr(item, "id", None)
        if isinstance(found, str) and found:
            ids.append(found)
    return ids


def evaluate(model_ids: Iterable[str]) -> dict[str, bool]:
    """Say for each expected model whether it is in model_ids."""
    present = set(model_ids)
    return {name: name in present for name in EXPECTED_MODELS}


def _is_auth_error(exc: BaseException) -> bool:
    """True for HTTP 401/403 failures (wrong or rejected key)."""
    if getattr(exc, "status_code", None) in (401, 403):
        return True
    return type(exc).__name__ in (
        "AuthenticationError",
        "PermissionDeniedError",
    )


def _is_connection_error(exc: BaseException) -> bool:
    """True for network failures without an HTTP status."""
    name = type(exc).__name__
    if "Connection" in name or "Timeout" in name or "Network" in name:
        return True
    return isinstance(exc, (ConnectionError, TimeoutError, OSError))


def run_check(
    api_key: str | None,
    make_client: Callable[[str], object],
) -> int:
    """Run the check with an injected client factory; return exit code.

    make_client receives the key and returns a client with
    models.list(). Returns 0 only when both expected models are
    listed, 2 when the key is missing, 1 otherwise. Errors print a
    short message without a stack trace and never include the key.
    """
    if not api_key or not api_key.strip():
        print(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    try:
        client = make_client(api_key)
        list_models = getattr(getattr(client, "models"), "list")
        ids = extract_model_ids(list_models())
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        if _is_auth_error(exc):
            print("key rejected")
            return 1
        if _is_connection_error(exc):
            print(f"network error: {exc}")
            return 1
        print(f"request failed: {exc}")
        return 1
    print(f"models: {len(ids)}")
    results = evaluate(ids)
    for name in EXPECTED_MODELS:
        print(f"{name}: {'OK' if results[name] else 'MISSING'}")
    return 0 if all(results.values()) else 1


def _default_client_factory(api_key: str) -> object:
    """Build the real client (only called by main, never by tests)."""
    from openai import OpenAI

    return OpenAI(api_key=api_key, base_url=BASE_URL)


def main() -> int:
    """Read the key and run the check against the real API."""
    return run_check(read_api_key(), _default_client_factory)


if __name__ == "__main__":
    sys.exit(main())
