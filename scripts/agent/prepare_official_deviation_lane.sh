#!/usr/bin/env bash
set -euo pipefail

export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-/home/ssy/ssy_files/mjlab/.venv}"

uv run --no-sync python scripts/agent/prepare_official_deviation_lane.py "$@"
