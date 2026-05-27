#!/usr/bin/env bash
set -euo pipefail

scripts/agent/status.sh

for script in scripts/agent/*.sh; do
  bash -n "${script}"
done

export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-/home/ssy/ssy_files/mjlab/.venv}"

PYTHONPATH="${PYTHONPATH:-src}" uv run --no-sync python - <<'PY'
import importlib.util
import sys

missing = [
    name
    for name in ("mjlab", "mujoco", "yaml", "tyro")
    if importlib.util.find_spec(name) is None
]
if missing:
    raise SystemExit(f"missing modules: {missing}")
print(f"python: {sys.executable}")
print("project import smoke: ok")
PY

printf 'harness check: ok\n'
