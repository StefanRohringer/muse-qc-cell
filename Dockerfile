# Offline test image for muse-qc-cell.
# Runs: the offline pytest suite (default CMD) and the agent with a fake
# table (uv run python -m muse_qc.agent, key passed at run time only).
# Does NOT: move the arm, open cameras, or touch USB devices. No .env is
# baked into the image; pass it with --env-file at run time if needed.
FROM python:3.11-slim

COPY --from=ghcr.io/astral-sh/uv:0.11.21 /uv /uvx /bin/

RUN apt-get update \
    && apt-get install -y --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work/muse-qc-cell
COPY . .

RUN bash scripts/setup_sdk.sh /work/seeed-sdk
RUN uv sync --locked

CMD ["uv", "run", "pytest", "-q"]
