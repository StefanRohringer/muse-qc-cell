# How Meta AI is used in muse-qc-cell

Every measured number below comes from `docs/DEVLOG.md` or
`docs/RESULTS.md`. Nothing here is estimated. API reference:
<https://dev.meta.ai/docs>.

## Muse Spark (muse-spark-1.3)

The agent. It reads the plain-words order, picks the next tool,
reacts to failures and rescans, and writes the summary.

- Call site: `muse_qc/agent/agent.py`. `_model_call()` sends one
  Responses API call; `run_agent()` executes the returned tool
  calls in a loop; `main()` is the entry point (`--simulate` by
  default: the real Muse Spark API with the fake table tools;
  `--execute` uses the real tools after one typed "yes").
- Request: Responses API with tool calling, model `muse-spark-1.3`,
  `instructions` = system prompt, `input` = conversation history,
  `tools` = `TOOL_DEFINITIONS`, `reasoning={"effort": "low"}`,
  `store=False`, `include=["reasoning.encrypted_content"]`. A call
  arrives as `{"type": "function_call", ...}` with `arguments` a
  JSON string; the result goes back as
  `{"type": "function_call_output", ...}` with `output` a JSON
  string.
- Tools the model can call (from `TOOL_DEFINITIONS` in
  `muse_qc/agent/agent_tools.py`, all strict function tools):
  - `scan_table` (no arguments): look at the table and list the
    unsorted parts.
  - `inspect_and_sort` (`part_id`: integer, required): pick the
    part with this scan id, count its levers, drop it into the
    good or bad box.
  - `write_report` (no arguments): build the HTML report from
    this run's records.
  - `finish` (`summary`: string, required): end the run with a
    short summary.
- Hard limits (all enforced in `muse_qc/agent/agent.py`):
  40 model calls (`MAX_STEPS`), 10 sorts (`MAX_SORTS`), 2 tries
  per part position (`MAX_PER_POSITION`), stop after 2
  consecutive failures (`FAIL_STOP`).
- The model passes part IDs only, never coordinates. Positions
  come from our own latest scan (`state.lookup(part_id)`); an
  unknown, already-sorted, or twice-tried id is refused.
- Conversation replay: every model output item is appended to the
  history (`plain_item()`), tool results are appended as
  `function_call_output` items with JSON-string output, and the
  next call replays the whole history. Reasoning items (with
  `encrypted_content`) are replayed into the history but never
  written to the transcript in `recordings/`.
- Measured use: the first 10-part run (30 Oct) took 14 Muse Spark
  calls and cost about 0.22 USD (credits 5.37 to 5.15 USD, about
  2 cents per part) over 7 min 20 s, about 44 s per part. The 10
  of 10 run 3 (31 Oct) took about 6.5 min, about 43 s per part;
  its cost was not measured (the credits page did not load). A
  simulate run (29 Oct) used between 830 and 3160 input tokens
  and 46 to 495 output tokens per call.

## SAM 3.1 (sam-3.1)

All eyes. It confirms parts on the table image and finds the
held part plus its levers in the close-up.

- Call sites: `muse_qc/vision/vision_sam.py` (`fetch_output_text()`
  sends the request, `parse_output_text()` parses the answer);
  `muse_qc/vision/find_parts.py` (`scan_table()` confirms each
  table candidate on a zoomed 640 px crop); 
  `muse_qc/inspection/count_clips.py` (`count_frame()` finds the
  part on the full image, then counts the levers on the cropped
  and enlarged part); `muse_qc/inspection/count_retry.py`
  (`count_with_retry()` retries through `count_frame()`).
- Request shape: one `POST /v1/responses` call with model
  `sam-3.1`, `input` = one user message with `input_text` (the
  short noun phrase) plus `input_image` (the frame as a JPEG
  base64 data URL), and `metadata` with `include_confidence`
  set to `"true"`.
- Box parsing: records look like
  `id<|box;x1=..;y1=..;x2=..;y2=..;w=..;h=..;c=..|>`. Fields are
  read by key, ids may be sparse, mask segments are ignored, and
  empty output means zero matches. Each box keeps its id,
  corners, center, and confidence (`c=`, or none without it).
- Prompts in use: `"grey object"` on the table (`DEFAULT_PHRASE`
  in `muse_qc/vision/find_parts.py`), with `"small part"` as the
  second try for real-looking candidates (`RETRY_PHRASE`);
  `"small part"` to find the held part (`PART_PROMPT` in
  `muse_qc/inspection/count_clips.py`); `"brown bar"` to count
  the levers (`LEVER_PROMPT`: the orange levers look brown under
  the evening lamp). The retry variants in
  `muse_qc/inspection/count_retry.py` are `("brown bar", 6, 15)`,
  `("brown bar", 10, 25)`, `("orange bar", 8, 15)` and
  `("orange bar", 6, 15)` (prompt, crop scale, margin).
- What failed in the tests: `"orange lever"` found 0 levers on
  both test-pair full images; `"grey connector"` gave 0 boxes on
  the table while `"white cube"` gave one box at confidence
  0.78; `"connector"`, `"electrical connector"`,
  `"wire connector"`, `"terminal block"` and `"orange lever"`
  each gave 0 boxes on the saved frame where `"small part"`
  found both connectors (confidence 0.62 and 0.58). Asking Muse
  Spark directly ("How many orange levers does it have?") counted
  3 on the bad 2-lever part, so Muse Spark is not used for
  counting.
- Measured results: on the test pair the good part counts 3
  (good) and the bad part 2 (bad) after the short remnant box
  is dropped by the bar-height rule. A good part takes about
  3 s from photo to verdict; a retried bad part took about 13 s
  over 5 attempts, with each count about 2 s.

## Muse Code

Muse Code (powered by Muse Spark) wrote all the code in this
repository, task by task, logged in `docs/MUSE_CODE_LOG.md`
(154 entries). Every hardware run in `docs/DEVLOG.md` was
started by the human by hand.

## How to verify

No hardware, no `.env`, no network for the suite; the three
checks below need a key or a camera and are human-run only.

- `uv run python -m muse_qc.checks.model_check --list`
- `uv run python -m muse_qc.vision.vision_sam "<phrase>" --image <path>` on a saved image
- `make simulate` (agent on the fake table, needs
  `MODEL_API_KEY` in `.env`, see the README Quickstart)
