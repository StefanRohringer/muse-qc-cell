"""Spoken chat orders through muse-voice-transcribe-1.0 (human only).

Typing "v" at an order prompt records 5 s from the default
microphone with arecord (alsa-utils, system tool, only called, not
bundled), sends the WAV to POST {BASE_URL}/asr/transcribe and uses
the returned transcript as the order text after one Enter
confirmation. Every order still runs through the same agent loop,
limits and safety checks; voice only replaces typing the order.
Only the human runs this, never CI and never an agent, since it
needs a microphone and the network. Tests use fakes only.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import wave
from datetime import datetime
from pathlib import Path

from muse_qc.checks.model_check import BASE_URL, KEY_NAME, read_api_key

try:  # httpx ships with the openai package (named httpx2 there)
    import httpx
except ImportError:  # pragma: no cover - resolved by the environment
    try:
        import httpx2 as httpx  # type: ignore[no-redef]
    except ImportError:
        httpx = None  # type: ignore[assignment]

MODEL = "muse-voice-transcribe-1.0"
TRANSCRIBE_URL = BASE_URL + "/asr/transcribe"
KEYWORDS = ["sort", "parts", "table", "good", "bad", "box", "new part"]
DEFAULT_SECONDS = 5
MIN_SECONDS = 1
MAX_SECONDS = 15
SAMPLE_RATE_HZ = 16000
REQUEST_TIMEOUT_S = 30.0
MAX_AUDIO_BYTES = 32 * 1024 * 1024


class VoiceError(Exception):
    """Short spoken-order failure (never carries the API key)."""


def default_wav_path(recordings_dir="recordings") -> str:
    """Return a fresh recordings/voice_<timestamp>.wav path."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(Path(recordings_dir) / f"voice_{stamp}.wav")


def record_wav(path, seconds=DEFAULT_SECONDS, device=None) -> str:
    """Record seconds of 16 kHz mono S16_LE WAV with arecord.

    device selects the arecord device (for example "plughw:0,0");
    without it arecord uses the system default. Returns the path
    as a string. Raises VoiceError when seconds is outside 1 to
    15, when arecord is missing (alsa-utils) or when the
    recording fails.
    """
    if seconds not in range(MIN_SECONDS, MAX_SECONDS + 1):
        raise VoiceError(
            f"seconds must be {MIN_SECONDS} to {MAX_SECONDS},"
            f" got {seconds}")
    if shutil.which("arecord") is None:
        raise VoiceError(
            "arecord is missing: install alsa-utils to speak orders")
    target = Path(path)
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise VoiceError(f"cannot write {target}: {exc}") from exc
    cmd = ["arecord"]
    if device:
        cmd += ["-D", device]
    cmd += ["-f", "S16_LE", "-r", str(SAMPLE_RATE_HZ),
            "-c", "1", "-d", str(seconds), str(target)]
    try:
        done = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=seconds + 10)
    except (OSError, subprocess.SubprocessError) as exc:
        raise VoiceError(f"arecord failed: {exc}") from exc
    if done.returncode != 0:
        detail = (done.stderr or "").strip().splitlines()
        hint = f": {detail[0]}" if detail else ""
        raise VoiceError(f"arecord failed (exit {done.returncode}){hint}")
    return str(target)


def transcribe(path, api_key, post=None) -> str:
    """Send the WAV file for transcription; return the stripped text.

    posts multipart/form-data with a "request" JSON part
    (application/json) and an "audio" WAV part (audio/wav) plus a
    Bearer key and an Accept: application/json header, with a 30 s
    timeout. Raises VoiceError on HTTP errors or a missing
    transcript. The key is never printed or logged.
    """
    try:
        audio = Path(path).read_bytes()
    except OSError as exc:
        raise VoiceError(f"cannot read {path}: {exc}") from exc
    if len(audio) > MAX_AUDIO_BYTES:
        raise VoiceError("audio too large: at most 32 MB")
    if not (api_key or "").strip():
        raise VoiceError(f"{KEY_NAME} is missing")
    request_json = json.dumps({
        "model": MODEL,
        "audioEncoding": "WAV",
        "mode": "PUSH_TO_TALK",
        "keywords": list(KEYWORDS),
    })
    files = {
        "request": (None, request_json, "application/json"),
        "audio": (Path(path).name, audio, "audio/wav"),
    }
    headers = {"Authorization": "Bearer " + api_key.strip(),
               "Accept": "application/json"}
    if post is None:
        if httpx is None:
            raise VoiceError("no HTTP client: install httpx")
        post = httpx.post
    try:
        response = post(TRANSCRIBE_URL, headers=headers, files=files,
                        timeout=REQUEST_TIMEOUT_S)
    except VoiceError:
        raise
    except Exception as exc:  # noqa: BLE001 - mapped, key never included
        raise VoiceError(f"transcribe failed: {exc}") from exc
    status = getattr(response, "status_code", None)
    if status is not None and status != 200:
        raise VoiceError(f"transcribe failed: HTTP {status}")
    try:
        data = response.json()
    except Exception as exc:  # noqa: BLE001 - mapped to a short message
        raise VoiceError("transcribe failed: bad response") from exc
    text = data.get("transcript") if isinstance(data, dict) else None
    if not isinstance(text, str) or not text.strip():
        raise VoiceError("transcribe failed: no transcript")
    return text.strip()


def voice_ask(ask, record_fn, transcribe_fn, out=print):
    """Wrap a chat ask function with spoken orders.

    Typing exactly "v" at an order prompt records, transcribes,
    prints the heard text and asks "use this order? (Enter = yes,
    anything else = no): ". Enter returns the transcript as the
    order; anything else, an empty transcript or a VoiceError
    prints one short line and asks for the order again (typed or
    "v"). Any other typed text passes through unchanged. Order
    prompts gain the hint "(v = speak)"; all other prompts
    (including the "use this order?" confirmation) stay exact.
    """

    def wrapped(prompt):
        if "(empty line to quit)" in prompt and prompt.endswith(": "):
            prompt = prompt[:-2] + " (v = speak): "
        while True:
            text = ask(prompt)
            if text != "v":
                return text
            out("voice: recording 5 s, speak now")
            try:
                heard = transcribe_fn(record_fn())
            except VoiceError as exc:
                out(f"voice: {exc}")
                continue
            if not (heard or "").strip():
                out("voice: empty transcript, try again")
                continue
            out(f"voice: heard \"{heard.strip()}\"")
            confirm = ask("use this order? "
                          "(Enter = yes, anything else = no): ")
            if confirm == "":
                return heard.strip()
            out("voice: discarded, type the order or press v")

    return wrapped


def record_default(seconds=DEFAULT_SECONDS,
                    recordings_dir="recordings", device=None) -> str:
    """Record one order WAV to recordings/; used to wire --voice."""
    return record_wav(default_wav_path(recordings_dir), seconds,
                      device=device)


def audio_seconds(path) -> float | None:
    """Return the WAV length in seconds, or None when unreadable."""
    try:
        with wave.open(str(path), "rb") as stream:
            rate = stream.getframerate()
            if not rate:
                return None
            return stream.getnframes() / rate
    except (OSError, wave.Error):
        return None


def main(argv=None) -> int:
    """Hand check without the arm: record (or reuse) and transcribe."""
    parser = argparse.ArgumentParser(
        description="Speak an order and print what the"
        " speech-to-text model heard (human only).")
    parser.add_argument("--seconds", type=int, default=DEFAULT_SECONDS,
                        help="recording length in seconds (1 to 15)")
    parser.add_argument("--file", default=None,
                        help="use this WAV file instead of recording")
    parser.add_argument("--mic", default=None,
                        help="arecord device (for example plughw:0,0);"
                        " without it arecord uses the system default")
    args = parser.parse_args(argv)
    if args.seconds not in range(MIN_SECONDS, MAX_SECONDS + 1):
        parser.error("--seconds must be 1 to 15")
    path = args.file
    if path is None:
        path = default_wav_path()
        try:
            record_wav(path, args.seconds, device=args.mic)
        except VoiceError as exc:
            print(exc)
            return 1
    key = read_api_key()
    if not (key or "").strip():
        print(f"{KEY_NAME} is missing: set it in .env or the environment")
        return 2
    try:
        text = transcribe(path, key)
    except VoiceError as exc:
        print(f"voice: {exc}")
        return 1
    length = audio_seconds(path)
    print(f"transcript: {text}")
    if length is None:
        print(f"audio: {path} (duration unknown)")
    else:
        print(f"audio: {path} ({length:.1f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
