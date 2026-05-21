#!/usr/bin/env bash
set -euo pipefail

WORKTREE="${MJLAB_WORKTREE:-/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main}"
MJLAB_VIRTUAL_ENV="${MJLAB_VIRTUAL_ENV:-/home/ssy/ssy_files/mjlab/.venv}"
TASK_ID="${MJLAB_TASK_ID:-Mjlab-Tracking-Flat-Unitree-G1-Acrobatics-No-State-Estimation}"
EXPERIMENT_NAME="${MJLAB_EXPERIMENT_NAME:-g1_tracking_acrobatics_no_state}"
MOTION_FILE="${MJLAB_MOTION_FILE:-data/motions/g1_flying_kick/mjlab/motion.npz}"
NUM_ENVS="${MJLAB_NUM_ENVS:-4096}"
MAX_ITERATIONS="${MJLAB_MAX_ITERATIONS:-5000}"
RUN_NAME_PREFIX="${MJLAB_RUN_NAME_PREFIX:-g1_mode15_flying_kick_4096env_5000iter}"
RUN_NAME_PATTERN="${MJLAB_RUN_NAME_PATTERN:-*${RUN_NAME_PREFIX}*}"
SESSION="${MJLAB_TRAIN_SESSION:-g1_mode15_flying_kick_5000iter}"
LOG_ROOT="$WORKTREE/logs/g1_mode15_flying_kick_5000iter"

if [[ -d "$MJLAB_VIRTUAL_ENV" ]]; then
  export VIRTUAL_ENV="$MJLAB_VIRTUAL_ENV"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
fi

usage() {
  cat <<'EOF'
Usage:
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh start
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh status
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh export [checkpoint.pt] [output.onnx]
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh finalize-deploy
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh stop

Defaults:
  task: Mjlab-Tracking-Flat-Unitree-G1-Acrobatics-No-State-Estimation
  motion: data/motions/g1_flying_kick/mjlab/motion.npz
  iterations: 5000
EOF
}

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    printf 'Missing required file: %s\n' "$path" >&2
    exit 2
  fi
}

assert_mode15_asset() {
  (
    cd "$WORKTREE"
    uv run --active --no-sync python -c "import xml.etree.ElementTree as ET; from mjlab.asset_zoo.robots.unitree_g1 import g1_constants; model = ET.parse(g1_constants.G1_XML).getroot().attrib['model']; assert model == 'g1_29dof_mode_15_aligned', model; assert g1_constants.G1_URDF.name == 'g1_29dof_mode_15.urdf', g1_constants.G1_URDF; print(f'[INFO] Default G1 asset: {model} | {g1_constants.G1_XML}')"
  )
}

latest_run_dir() {
  local root="$WORKTREE/logs/rsl_rl/$EXPERIMENT_NAME"
  if [[ ! -d "$root" ]]; then
    return 1
  fi
  find "$root" -maxdepth 1 -mindepth 1 -type d -name "$RUN_NAME_PATTERN" \
    -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-
}

latest_checkpoint() {
  local run_dir="$1"
  find "$run_dir" -maxdepth 1 -type f -name 'model_*.pt' \
    -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-
}

highest_checkpoint() {
  local run_dir="$1"
  find "$run_dir" -maxdepth 1 -type f -name 'model_*.pt' -printf '%f %p\n' \
    | sed -E 's/^model_([0-9]+)\.pt /\1 /' \
    | sort -n \
    | tail -n 1 \
    | cut -d' ' -f2-
}

checkpoint_iteration() {
  local checkpoint="$1"
  basename "$checkpoint" | sed -E 's/^model_([0-9]+)\.pt$/\1/'
}

start_train() {
  require_file "$WORKTREE/$MOTION_FILE"
  assert_mode15_asset

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    printf 'Training tmux session already exists: %s\n' "$SESSION" >&2
    exit 2
  fi

  mkdir -p "$LOG_ROOT"
  local stamp run_name log_file cmd
  stamp="$(date '+%Y%m%d_%H%M%S')"
  run_name="${MJLAB_RUN_NAME:-${RUN_NAME_PREFIX}_${stamp}}"
  log_file="$LOG_ROOT/${run_name}.log"
  cmd="set -o pipefail; uv run --active --no-sync train $TASK_ID --env.scene.num-envs $NUM_ENVS --env.commands.motion.motion-file $MOTION_FILE --agent.max-iterations $MAX_ITERATIONS --agent.run-name $run_name 2>&1 | tee $log_file"

  tmux new-session -d -s "$SESSION" -c "$WORKTREE" "bash -lc '$cmd'"
  printf 'Started 5000-iter mode15 flying-kick training in tmux session: %s\n' "$SESSION"
  printf 'Run name: %s\n' "$run_name"
  printf 'Launcher log: %s\n' "$log_file"
  printf 'Attach: tmux attach -t %s\n' "$SESSION"
}

export_policy() {
  assert_mode15_asset
  local run_dir checkpoint output
  run_dir="$(latest_run_dir || true)"
  if [[ -z "$run_dir" ]]; then
    printf 'No mode15 5000-iter run found under logs/rsl_rl/%s.\n' "$EXPERIMENT_NAME" >&2
    exit 2
  fi

  checkpoint="${1:-}"
  if [[ -z "$checkpoint" ]]; then
    checkpoint="$(latest_checkpoint "$run_dir" || true)"
  fi
  if [[ -z "$checkpoint" ]]; then
    printf 'No model_*.pt checkpoint found in latest run: %s\n' "$run_dir" >&2
    exit 2
  fi

  output="${2:-$run_dir/flying_kick_deploy_actor.onnx}"
  (
    cd "$WORKTREE"
    uv run --active --no-sync python scripts/tools/export_rsl_rl_actor_onnx.py \
      "$TASK_ID" \
      --checkpoint-file "$checkpoint" \
      --motion-file "$MOTION_FILE" \
      --output-file "$output" \
      --device cpu
  )
}

finalize_deploy() {
  assert_mode15_asset
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    printf 'Waiting for training session to finish: %s\n' "$SESSION"
    while tmux has-session -t "$SESSION" 2>/dev/null; do
      sleep 60
    done
  fi

  local run_dir checkpoint iter min_final_iter
  run_dir="$(latest_run_dir || true)"
  if [[ -z "$run_dir" ]]; then
    printf 'No mode15 5000-iter run found under logs/rsl_rl/%s.\n' "$EXPERIMENT_NAME" >&2
    exit 2
  fi
  checkpoint="$(highest_checkpoint "$run_dir" || true)"
  if [[ -z "$checkpoint" ]]; then
    printf 'No model_*.pt checkpoint found in latest run: %s\n' "$run_dir" >&2
    exit 2
  fi
  iter="$(checkpoint_iteration "$checkpoint")"
  min_final_iter=$((MAX_ITERATIONS - 1))
  if (( iter < min_final_iter )); then
    printf 'Latest checkpoint is only model_%s.pt; expected at least model_%s.pt for %s full iterations.\n' \
      "$iter" "$min_final_iter" "$MAX_ITERATIONS" >&2
    exit 2
  fi

  export_policy "$checkpoint" "$run_dir/flying_kick_deploy_actor.onnx"
  bash "$WORKTREE/scripts/tools/run_flying_kick_real_deploy.sh" prepare
  printf 'Finalized deploy bundle from checkpoint: %s\n' "$checkpoint"
}

status() {
  tmux ls 2>/dev/null | rg "$SESSION" || true
  pgrep -af "train $TASK_ID|$SESSION" || true
  local run_dir
  run_dir="$(latest_run_dir || true)"
  if [[ -n "$run_dir" ]]; then
    printf 'Latest run: %s\n' "$run_dir"
    find "$run_dir" -maxdepth 1 -type f -name 'model_*.pt' -printf '%f\n' | sort -V | tail -n 5
  fi
  local latest_log
  latest_log="$(find "$LOG_ROOT" -maxdepth 1 -type f -name '*.log' ! -name 'finalize_deploy.log' -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 | cut -d' ' -f2- || true)"
  if [[ -n "$latest_log" ]]; then
    printf 'Latest launcher log: %s\n' "$latest_log"
    local latest_iter latest_reward
    latest_iter="$(sed -r 's/\x1B\[[0-9;]*[mK]//g' "$latest_log" \
      | rg 'Learning iteration [0-9]+/[0-9]+' \
      | tail -n 1 \
      || true)"
    latest_reward="$(rg 'Mean reward:' "$latest_log" | tail -n 1 || true)"
    if [[ -n "$latest_iter" ]]; then
      printf 'Latest iteration: %s\n' "$latest_iter"
    fi
    if [[ -n "$latest_reward" ]]; then
      printf '%s\n' "$latest_reward"
    fi
    tail -n 40 "$latest_log"
  fi
}

stop_train() {
  tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
  true
}

cmd="${1:-}"
case "$cmd" in
  start)
    start_train
    ;;
  status)
    status
    ;;
  export)
    export_policy "${2:-}" "${3:-}"
    ;;
  finalize-deploy)
    finalize_deploy
    ;;
  stop)
    stop_train
    ;;
  *)
    usage
    exit 2
    ;;
esac
