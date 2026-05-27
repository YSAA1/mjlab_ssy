#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${MJLAB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
ISAACLAB_ROOT="${ISAACLAB_ROOT:-/home/ssy/ssy_files/IsaacLab}"
ISAACLAB_CONDA_ENV="${ISAACLAB_CONDA_ENV:-env_isaaclab}"
EXTERNAL_ROOT="${MJLAB_EXTERNAL_ROOT:-$ROOT/.external}"
if [[ ! -d "$EXTERNAL_ROOT" && "$ROOT" == */.worktrees/* ]]; then
  PRIMARY_ROOT="${ROOT%%/.worktrees/*}"
  if [[ -d "$PRIMARY_ROOT/.external" ]]; then
    EXTERNAL_ROOT="$PRIMARY_ROOT/.external"
  fi
fi

usage() {
  cat <<'EOF'
Usage:
  scripts/tools/run_g1_kick_isaaclab_sim2sim.sh --action <flying_kick|roundhouse_leading_right> [runner args...]

Runs the selected G1 deploy bundle in the existing Isaac Lab conda environment.
The runner writes evidence under logs/g1_isaaclab_sim2sim/ by default.

Environment:
  ISAACLAB_ROOT       Isaac Lab checkout, default /home/ssy/ssy_files/IsaacLab
  ISAACLAB_CONDA_ENV  Conda env name, default env_isaaclab
  MJLAB_ROOT          mjlab checkout, default this repository root
  MJLAB_EXTERNAL_ROOT external runtime assets, default <MJLAB_ROOT>/.external
EOF
}

if [[ $# -eq 0 ]]; then
  usage >&2
  exit 2
fi

case "${1:-}" in
  -h|--help)
    usage
    exit 0
    ;;
esac

[[ -d "$ISAACLAB_ROOT" ]] || {
  printf 'Missing Isaac Lab checkout: %s\n' "$ISAACLAB_ROOT" >&2
  exit 2
}
[[ -x "$ISAACLAB_ROOT/isaaclab.sh" ]] || {
  printf 'Missing Isaac Lab launcher: %s\n' "$ISAACLAB_ROOT/isaaclab.sh" >&2
  exit 2
}

mkdir -p /tmp/mjlab-matplotlib

cd "$ISAACLAB_ROOT"
env \
  TERM=xterm \
  MPLCONFIGDIR=/tmp/mjlab-matplotlib \
  MJLAB_ROOT="$ROOT" \
  MJLAB_EXTERNAL_ROOT="$EXTERNAL_ROOT" \
  PYTHONPATH="$ROOT/src:${PYTHONPATH:-}" \
  conda run -n "$ISAACLAB_CONDA_ENV" \
  ./isaaclab.sh -p "$ROOT/scripts/isaaclab/g1_kick_sim2sim.py" \
  --headless \
  "$@"
