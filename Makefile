# Offline shortcuts for muse-qc-cell. No target moves the arm.
.PHONY: help setup test docker simulate export-site

help:
	@echo "make setup      - prepare the SDK and sync the environment"
	@echo "make test       - run the offline test suite"
	@echo "make docker     - build and run the offline test image"
	@echo "make simulate   - run the agent with the fake table (needs MODEL_API_KEY)"
	@echo "make export-site - show the export_site command from docs/SETUP.md"

setup:
	bash scripts/setup_sdk.sh && uv sync

test:
	uv run pytest -q

docker:
	docker build -t muse-qc-cell . && docker run --rm muse-qc-cell

simulate:
	uv run python -m muse_qc.agent

export-site:
	@echo "uv run python -m muse_qc.inspection.export_site --recordings recordings --since <YYYYmmdd_HHMMSS> --until <YYYYmmdd_HHMMSS> --out site/data"
