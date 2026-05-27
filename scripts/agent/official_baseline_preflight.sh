#!/usr/bin/env bash
set -euo pipefail

report_out="${1:-logs/g1_tracking_phase1/2026-05-23T-official-baseline-preflight/official_baseline_preflight.json}"

scripts/agent/status.sh >/dev/null

export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-/home/ssy/ssy_files/mjlab/.venv}"

uv run --no-sync python \
  scripts/agent/official_baseline_preflight_stdlib.py \
  --expect-ready \
  --report-out "${report_out}"
