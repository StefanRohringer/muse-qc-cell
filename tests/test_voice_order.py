"""Tests for spoken chat orders (muse-voice-transcribe-1.0). Offline.

Fake post functions check the request shape, fake responses drive
transcript parsing and HTTP errors, fake record/transcribe
functions drive voice_ask, and main runs with a fake client. No
microphone, no network, no .env.
"""

from __future__ import annotations

import json
import wave

import pytest

import muse_qc.agent.agent as agent_mod
from muse_qc.agent import agent_tools, voice_order
from muse_qc.agent.agent import main
from muse_qc.agent.voice_order import (
    KEYWORDS,
    MODEL,
    TRANSCRIBE_URL,
    VoiceError,
    transcribe,
    voice_ask,
)

SECRET = "fake-test-key-abc123"


class FakeResponse:
    """Minimal response double with status, JSON and headers."""

    def __init__(self, status_code, payload):
        self.status_code = status_code
        self._payload = payload

    def json(self):
        if isinstance(self._payload, Exception):
            raise self._payload
        return self._payload


def _ok_post(seen, transcript="  sort all parts  "):
    def post(url, headers=None, files=None, timeout=None):
        seen.append({"url": url, "headers": dict(headers or {}),
                     "files": files, "timeout": timeout})
        return FakeResponse(200, {"sessionId": "s1",
                                  "transcript": transcript,
                                  "audioDurationMs": 5000,
                                  "turns": []})

    return post


def _wav_file(tmp_path, name="order.wav"):
    path = tmp_path / name
    with wave.open(str(path), "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(16000)
        stream.writeframes(b"\x00\x00" * 16000)
    return path


def test_request_shape(tmp_path):
    seen = []
    path = _wav_file(tmp_path)
    text = transcribe(str(path), SECRET, post=_ok_post(seen))
    assert text == "sort all parts"
    assert len(seen) == 1
    call = seen[0]
    assert call["url"] == TRANSCRIBE_URL
    assert call["timeout"] == 30.0
    assert call["headers"]["Authorization"] == f"Bearer {SECRET}"
    assert call["headers"]["Accept"] == "application/json"
    assert "MODEL_API_KEY" not in json.dumps(call["headers"])
    assert set(call["files"]) == {"request", "audio"}
    request_name, request_text, request_type = call["files"]["request"]
    assert request_name is None
    assert request_type == "application/json"
    body = json.loads(request_text)
    assert body == {"model": MODEL, "audioEncoding": "WAV",
                    "mode": "PUSH_TO_TALK", "keywords": KEYWORDS}
    assert MODEL == "muse-voice-transcribe-1.0"
    audio_name, audio_bytes, audio_type = call["files"]["audio"]
    assert audio_name == "order.wav"
    assert audio_type == "audio/wav"
    assert audio_bytes == path.read_bytes()


def test_transcript_stripped(tmp_path):
    path = _wav_file(tmp_path)
    assert transcribe(str(path), SECRET,
                      post=_ok_post([], " sort the rest\n")) == "sort the rest"


@pytest.mark.parametrize("status", [400, 401, 406, 413, 429, 500])
def test_http_errors_raise_voice_error(tmp_path, status):
    path = _wav_file(tmp_path)
    calls = []

    def post(url, headers=None, files=None, timeout=None):
        calls.append(url)
        return FakeResponse(status, {})

    with pytest.raises(VoiceError, match=f"HTTP {status}"):
        transcribe(str(path), SECRET, post=post)
    assert calls == [TRANSCRIBE_URL]


def test_missing_transcript_raises(tmp_path):
    path = _wav_file(tmp_path)
    for payload in [{}, {"transcript": "   "}, {"transcript": 42}]:
        with pytest.raises(VoiceError, match="[Tt]ranscript"):
            transcribe(str(path), SECRET,
                       post=lambda **kwargs: FakeResponse(200, payload))


def test_bad_response_json_raises(tmp_path):
    path = _wav_file(tmp_path)
    with pytest.raises(VoiceError):
        transcribe(str(path), SECRET,
                   post=lambda **kwargs: FakeResponse(
                       200, ValueError("no json")))


def test_post_failure_raises_without_key(tmp_path):
    path = _wav_file(tmp_path)

    def post(**kwargs):
        raise ConnectionError("net down")

    with pytest.raises(VoiceError, match="transcribe failed") as info:
        transcribe(str(path), SECRET, post=post)
    assert SECRET not in str(info.value)


def test_missing_key_raises(tmp_path):
    path = _wav_file(tmp_path)
    with pytest.raises(VoiceError, match="MODEL_API_KEY"):
        transcribe(str(path), "   ", post=_ok_post([]))


def test_missing_arecord_clear_error(monkeypatch, tmp_path):
    monkeypatch.setattr(voice_order.shutil, "which", lambda name: None)
    with pytest.raises(VoiceError, match="arecord is missing"):
        voice_order.record_wav(str(tmp_path / "voice.wav"), 5)


def test_arecord_failure_clear_error(monkeypatch, tmp_path):
    monkeypatch.setattr(voice_order.shutil, "which", lambda name: "arecord")

    class Done:
        returncode = 1
        stderr = "busy\n"

    monkeypatch.setattr(voice_order.subprocess, "run",
                        lambda *args, **kwargs: Done())
    with pytest.raises(VoiceError, match="arecord failed"):
        voice_order.record_wav(str(tmp_path / "voice.wav"), 5)


def test_record_wav_command_and_range(monkeypatch, tmp_path):
    monkeypatch.setattr(voice_order.shutil, "which", lambda name: "arecord")
    seen = {}

    class Done:
        returncode = 0
        stderr = ""

    def run(cmd, **kwargs):
        seen["cmd"] = cmd
        return Done()

    monkeypatch.setattr(voice_order.subprocess, "run", run)
    out = voice_order.record_wav(str(tmp_path / "voice.wav"), 5)
    assert out == str(tmp_path / "voice.wav")
    assert seen["cmd"] == ["arecord", "-f", "S16_LE", "-r", "16000",
                           "-c", "1", "-d", "5", str(tmp_path / "voice.wav")]
    for bad in (0, 16):
        with pytest.raises(VoiceError, match="1 to 15"):
            voice_order.record_wav(str(tmp_path / "voice.wav"), bad)


def _ask_script(answers, prompts):
    it = iter(answers)

    def ask(prompt):
        prompts.append(prompt)
        return next(it)

    return ask


def test_voice_accepted_on_enter():
    prompts = []
    lines = []
    ask = voice_ask(_ask_script(["v", ""], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=lambda path: "sort all parts",
                    out=lines.append)
    assert ask("order (empty line to quit): ") == "sort all parts"
    assert prompts[0] == "order (empty line to quit) (v = speak): "
    assert prompts[1] == ("use this order? "
                          "(Enter or y = yes, anything else = no): ")
    assert lines[0] == "voice: recording 5 s, speak now"
    assert lines[1] == 'voice: heard "sort all parts"'


def test_voice_rejected_then_typed():
    prompts = []
    lines = []
    ask = voice_ask(_ask_script(["v", "n", "sort the rest"], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=lambda path: "sort all parts",
                    out=lines.append)
    assert ask("next order (empty line to quit): ") == "sort the rest"
    assert lines[-1].startswith("voice: discarded")


def test_empty_transcript_asks_again():
    prompts = []
    lines = []
    ask = voice_ask(_ask_script(["v", "sort the rest"], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=lambda path: "   ",
                    out=lines.append)
    assert ask("order (empty line to quit): ") == "sort the rest"
    assert "voice: empty transcript, try again" in lines


def test_voice_error_asks_again():
    prompts = []
    lines = []

    def fail(path):
        raise VoiceError("transcribe failed: HTTP 429")

    ask = voice_ask(_ask_script(["v", "sort the rest"], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=fail, out=lines.append)
    assert ask("order (empty line to quit): ") == "sort the rest"
    assert "voice: transcribe failed: HTTP 429" in lines


def test_typed_text_untouched():
    prompts = []
    ask = voice_ask(_ask_script(["sort the good ones"], prompts),
                    record_fn=lambda: (_ for _ in ()).throw(
                        AssertionError("no record expected")),
                    transcribe_fn=lambda path: (_ for _ in ()).throw(
                        AssertionError("no transcribe expected")),
                    out=lambda text: None)
    assert ask("order (empty line to quit): ") == "sort the good ones"
    assert "(v = speak)" in prompts[0]


def test_confirmation_prompt_stays_exact():
    prompts = []
    ask = voice_ask(_ask_script(["v", "", ""], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=lambda path: "sort all parts",
                    out=lambda text: None)
    ask("order (empty line to quit): ")
    assert prompts[1] == ("use this order? "
                          "(Enter or y = yes, anything else = no): ")


@pytest.mark.parametrize("answer", ["y", "yes", "Y", "YES", " yes ",
                                    "  Yes  "])
def test_voice_accepted_on_y_variants(answer):
    prompts = []
    lines = []
    ask = voice_ask(_ask_script(["v", answer], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=lambda path: "sort all parts",
                    out=lines.append)
    assert ask("order (empty line to quit): ") == "sort all parts"
    assert 'voice: heard "sort all parts"' in lines
    assert not any(line.startswith("voice: discarded") for line in lines)


@pytest.mark.parametrize("answer", ["n", "no", "N", "NO", "maybe", "yep"])
def test_voice_discarded_on_no_variants(answer):
    prompts = []
    lines = []
    ask = voice_ask(_ask_script(["v", answer, "sort the rest"], prompts),
                    record_fn=lambda: "recordings/voice_x.wav",
                    transcribe_fn=lambda path: "sort all parts",
                    out=lines.append)
    assert ask("order (empty line to quit): ") == "sort the rest"
    assert lines[-1].startswith("voice: discarded")


def test_voice_needs_chat(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    with pytest.raises(SystemExit) as info:
        main(["--voice"], client_factory=lambda key: None,
             toolset=None, out=lambda text: None)
    assert info.value.code == 2


def test_voice_chat_typed_order(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import json as json_mod

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)

    def call(call_id, name, args):
        return {"type": "function_call", "call_id": call_id, "name": name,
                "arguments": json_mod.dumps(args)}

    class ScriptedClient:
        def __init__(self):
            self.responses = self
            self.seen = []

        def create(self, **kwargs):
            self.seen.append(kwargs)
            if len(self.seen) == 1:
                return SimpleNamespace(output=[
                    call("c1", "scan_table", {})])
            return SimpleNamespace(output=[
                call("c2", "finish", {"summary": "voice order done"})])

    answers = iter(["sort all parts", ""])
    prompts = []
    lines = []
    fake = agent_tools.FakeTable()
    code = main(
        ["--chat", "--voice"],
        client_factory=lambda key: ScriptedClient(),
        toolset=(fake.scan, fake.sort,
                 lambda records: agent_tools.fake_report(
                     records, recordings_dir=str(tmp_path))),
        input_fn=lambda prompt: (prompts.append(prompt), next(answers))[1],
        out=lines.append)
    assert code == 0
    assert any("(v = speak)" in prompt for prompt in prompts)
    assert "order: sort all parts" in "\n".join(lines)


def test_voice_yes_confirmation_stays_typed(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import json as json_mod

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)

    def call(call_id, name, args):
        return {"type": "function_call", "call_id": call_id, "name": name,
                "arguments": json_mod.dumps(args)}

    class ScriptedClient:
        def __init__(self):
            self.responses = self

        def create(self, **kwargs):
            return SimpleNamespace(output=[
                call("c1", "finish", {"summary": "done"})])

    answers = iter(["yes", ""])
    prompts = []
    lines = []
    code = main(
        ["--execute", "--chat", "--voice", "--task", "sort all parts"],
        client_factory=lambda key: ScriptedClient(),
        input_fn=lambda prompt: (prompts.append(prompt), next(answers))[1],
        out=lines.append)
    assert code == 0
    yes_prompts = [prompt for prompt in prompts if "Type yes" in prompt]
    assert len(yes_prompts) == 1
    assert "(v = speak)" not in yes_prompts[0]


def test_record_wav_with_device(monkeypatch, tmp_path):
    monkeypatch.setattr(voice_order.shutil, "which", lambda name: "arecord")
    seen = {}

    class Done:
        returncode = 0
        stderr = ""

    def run(cmd, **kwargs):
        seen["cmd"] = cmd
        return Done()

    monkeypatch.setattr(voice_order.subprocess, "run", run)
    out = voice_order.record_wav(str(tmp_path / "voice.wav"), 5,
                                 device="plughw:0,0")
    assert out == str(tmp_path / "voice.wav")
    assert seen["cmd"] == ["arecord", "-D", "plughw:0,0", "-f", "S16_LE",
                           "-r", "16000", "-c", "1", "-d", "5",
                           str(tmp_path / "voice.wav")]


def test_record_wav_without_device_has_no_d(monkeypatch, tmp_path):
    monkeypatch.setattr(voice_order.shutil, "which", lambda name: "arecord")
    seen = {}

    class Done:
        returncode = 0
        stderr = ""

    def run(cmd, **kwargs):
        seen["cmd"] = cmd
        return Done()

    monkeypatch.setattr(voice_order.subprocess, "run", run)
    voice_order.record_wav(str(tmp_path / "voice.wav"), 5)
    assert "-D" not in seen["cmd"]


def test_voice_main_file_skips_record(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(voice_order, "read_api_key", lambda: SECRET)
    seen = {}
    monkeypatch.setattr(voice_order, "record_wav",
                        lambda path, seconds, device=None: seen.setdefault(
                            "device", device) or str(tmp_path / "order.wav"))
    monkeypatch.setattr(voice_order, "transcribe",
                        lambda path, key: "sort all parts")
    _wav_file(tmp_path)
    code = voice_order.main(["--mic", "plughw:0,0",
                             "--file", str(tmp_path / "order.wav")])
    assert code == 0
    assert seen == {}


def test_voice_main_mic_recording_path(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(voice_order, "read_api_key", lambda: SECRET)
    seen = {}

    def fake_record(path, seconds, device=None):
        seen["device"] = device
        return str(_wav_file(tmp_path))

    monkeypatch.setattr(voice_order, "record_wav", fake_record)
    monkeypatch.setattr(voice_order, "transcribe",
                        lambda path, key: "sort all parts")
    code = voice_order.main(["--mic", "plughw:0,0"])
    assert code == 0
    assert seen == {"device": "plughw:0,0"}


def test_mic_needs_voice(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    with pytest.raises(SystemExit) as info:
        main(["--chat", "--mic", "plughw:0,0"],
             client_factory=lambda key: None,
             toolset=None, out=lambda text: None)
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        main(["--mic", "plughw:0,0"],
             client_factory=lambda key: None,
             toolset=None, out=lambda text: None)
    assert info.value.code == 2


def test_voice_mic_wiring(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import json as json_mod

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(agent_mod, "read_api_key", lambda: SECRET)
    seen = {}
    monkeypatch.setattr(voice_order, "record_default",
                        lambda seconds=5, recordings_dir="recordings",
                        device=None: seen.setdefault(
                            "device", device) or "recordings/voice_x.wav")
    monkeypatch.setattr(voice_order, "transcribe",
                        lambda path, key: "sort all parts")

    def call(call_id, name, args):
        return {"type": "function_call", "call_id": call_id, "name": name,
                "arguments": json_mod.dumps(args)}

    class ScriptedClient:
        def __init__(self):
            self.responses = self

        def create(self, **kwargs):
            if len(kwargs["input"]) == 1:
                return SimpleNamespace(output=[
                    call("c1", "scan_table", {})])
            return SimpleNamespace(output=[
                call("c2", "finish", {"summary": "voice order done"})])

    answers = iter(["v", "", ""])
    prompts = []
    lines = []
    fake = agent_tools.FakeTable()
    code = main(
        ["--chat", "--voice", "--mic", "plughw:0,0"],
        client_factory=lambda key: ScriptedClient(),
        toolset=(fake.scan, fake.sort,
                 lambda records: agent_tools.fake_report(
                     records, recordings_dir=str(tmp_path))),
        input_fn=lambda prompt: (prompts.append(prompt), next(answers))[1],
        out=lines.append)
    assert code == 0
    assert seen == {"device": "plughw:0,0"}
    assert "order: sort all parts" in "\n".join(lines)


def test_voice_main_seconds_range():
    with pytest.raises(SystemExit) as info:
        voice_order.main(["--seconds", "0"])
    assert info.value.code == 2
    with pytest.raises(SystemExit) as info:
        voice_order.main(["--seconds", "16"])
    assert info.value.code == 2
