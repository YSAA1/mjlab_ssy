#!/usr/bin/env bash
set -euo pipefail

scripts/agent/status.sh >/dev/null

export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-/home/ssy/ssy_files/mjlab/.venv}"

printf 'environment: mjlab project no-install probe\n'
printf 'cwd: %s\n' "$(pwd)"
printf 'uv_project_environment: %s\n' "${UV_PROJECT_ENVIRONMENT}"

if [[ -x .venv/bin/python ]]; then
  printf 'venv_python: .venv/bin/python\n'
else
  printf 'venv_python: missing\n'
fi

if [[ -x "${UV_PROJECT_ENVIRONMENT}/bin/python" ]]; then
  printf 'selected_env_python: %s/bin/python\n' "${UV_PROJECT_ENVIRONMENT}"
else
  printf 'selected_env_python: missing\n'
fi

printf 'default_python: '
command -v python || true

set +e
probe_output="$(uv run --no-sync python - <<'PY'
import importlib.util
import sys

print(f"uv_python: {sys.executable}")
for name in ("mujoco", "yaml", "tyro", "mjlab"):
    status = "ok" if importlib.util.find_spec(name) is not None else "missing"
    print(f"module_{name}: {status}")
PY
)"
status=$?
set -e

printf '%s\n' "${probe_output}"
printf 'uv_no_sync_probe_exit: %s\n' "${status}"
if [[ "${status}" -ne 0 ]]; then
  printf 'meaning: uv itself failed before module probing; inspect sandbox/env\n'
  exit "${status}"
elif grep -q 'module_.*: missing' <<<"${probe_output}"; then
  printf 'meaning: missing modules indicate environment preparation, not joystick state\n'
  exit 1
else
  printf 'meaning: selected project environment is ready for mjlab Python checks\n'
fi
