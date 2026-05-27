#!/usr/bin/env bash
set -euo pipefail

ROOT="${MJLAB_ROOT:-/home/ssy/ssy_files/mjlab}"
WORKTREE="${MJLAB_WORKTREE:-/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main}"
MJLAB_VIRTUAL_ENV="${MJLAB_VIRTUAL_ENV:-/home/ssy/ssy_files/mjlab/.venv}"
UNITREE="$ROOT/.external/unitree_rl_mjlab"
DEPLOY="$UNITREE/deploy/robots/g1"

EXPERIMENT_NAME="${MJLAB_EXPERIMENT_NAME:-g1_tracking_acrobatics_no_state}"
RUN_NAME_PATTERN="${MJLAB_RUN_NAME_PATTERN:-*g1_mode15_flying_kick_4096env_5000iter*}"
SELECTED_RUN_DIR=""
POLICY_SRC="${MJLAB_POLICY_ONNX:-}"
MOTION_SRC="$WORKTREE/data/motions/g1_flying_kick/mjlab/motion.npz"
DEPLOY_YAML_SRC="$DEPLOY/config/policy/mimic/getup/params/deploy.yaml"

ACTIVE_CONFIG="$DEPLOY/config/config.yaml"
FLYING_POLICY_DIR="$DEPLOY/config/policy/mimic/flying_kick"
FLYING_POLICY="$FLYING_POLICY_DIR/exported/policy.onnx"
FLYING_MOTION="$FLYING_POLICY_DIR/params/flying_kick.npz"
FLYING_DEPLOY_YAML="$FLYING_POLICY_DIR/params/deploy.yaml"

LOG_ROOT="$WORKTREE/logs/flying_kick_real_deploy"
CTRL_SESSION="flying_kick_real_ctrl"

if [[ -d "$MJLAB_VIRTUAL_ENV" ]]; then
  export VIRTUAL_ENV="$MJLAB_VIRTUAL_ENV"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
fi

usage() {
  cat <<'EOF'
Usage:
  bash scripts/tools/run_flying_kick_real_deploy.sh prepare
  bash scripts/tools/run_flying_kick_real_deploy.sh start <network-interface>
  bash scripts/tools/run_flying_kick_real_deploy.sh stop
  bash scripts/tools/run_flying_kick_real_deploy.sh status

Before prepare/start, export the latest trained actor with:
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh export

Real-robot FSM:
  Passive -> [X] -> Velocity
  Passive -> [A] -> Mimic_Getup -> Velocity
  Velocity -> [B] -> Passive
  Velocity -> [RB + X] -> Mimic_FlyingKick
  Mimic_FlyingKick -> [B] -> Passive
  Mimic_FlyingKick -> [RB + B] -> Velocity
  Mimic_FlyingKick ends -> Velocity
EOF
}

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    printf 'Missing required file: %s\n' "$path" >&2
    exit 2
  fi
}

project_python() {
  (
    cd "$WORKTREE"
    uv run --active --no-sync python "$@"
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

select_policy_src() {
  if [[ -n "$POLICY_SRC" ]]; then
    SELECTED_RUN_DIR="$(dirname "$POLICY_SRC")"
    return
  fi
  SELECTED_RUN_DIR="${MJLAB_RUN_DIR:-}"
  if [[ -z "$SELECTED_RUN_DIR" ]]; then
    SELECTED_RUN_DIR="$(latest_run_dir || true)"
  fi
  if [[ -z "$SELECTED_RUN_DIR" ]]; then
    printf 'No mode15 5000-iter run found under logs/rsl_rl/%s.\n' "$EXPERIMENT_NAME" >&2
    printf 'Start training with: bash scripts/tools/run_g1_mode15_flying_kick_train.sh start\n' >&2
    return 2
  fi
  POLICY_SRC="$SELECTED_RUN_DIR/flying_kick_deploy_actor.onnx"
}

stop_ctrl() {
  tmux has-session -t "$CTRL_SESSION" 2>/dev/null && tmux kill-session -t "$CTRL_SESSION"
  true
}

prepare_bundle() {
  select_policy_src
  if [[ ! -f "$POLICY_SRC" ]]; then
    printf 'Missing exported deploy ONNX: %s\n' "$POLICY_SRC" >&2
    printf 'After the 5000-iter train has a checkpoint, run: bash scripts/tools/run_g1_mode15_flying_kick_train.sh export\n' >&2
    exit 2
  fi
  require_file "$POLICY_SRC"
  require_file "$MOTION_SRC"
  require_file "$DEPLOY_YAML_SRC"
  require_file "$ACTIVE_CONFIG"
  require_file "$DEPLOY/build/g1_ctrl"

  mkdir -p "$FLYING_POLICY_DIR/exported" "$FLYING_POLICY_DIR/params"
  chmod u+w "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" 2>/dev/null || true
  cp "$POLICY_SRC" "$FLYING_POLICY"
  cp "$MOTION_SRC" "$FLYING_MOTION"
  cp "$DEPLOY_YAML_SRC" "$FLYING_DEPLOY_YAML"
  chmod u+w "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" 2>/dev/null || true

  project_python - "$ACTIVE_CONFIG" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])

data = yaml.safe_load(path.read_text())
fsm = data.setdefault("FSM", {})
fsm["initial_state"] = "Passive"
enabled = fsm.setdefault("_", {})
enabled["Passive"] = {"id": 1}
enabled["FixStand"] = {"id": 2}
enabled["Velocity"] = {"id": 3, "type": "RLBase"}
enabled["Mimic_Getup"] = {"id": 6, "type": "Mimic"}
enabled["Mimic_FlyingKick"] = {"id": 7, "type": "Mimic"}
enabled.pop("PreFlyingKick", None)

passive = fsm.setdefault("Passive", {})
passive_transitions = passive.setdefault("transitions", {})
passive_transitions.pop("FixStand", None)
passive_transitions["Velocity"] = "X.on_pressed"
passive_transitions["Mimic_Getup"] = "A.on_pressed"

fixstand = fsm.setdefault("FixStand", {})
fixstand_transitions = fixstand.setdefault("transitions", {})
fixstand_transitions["Passive"] = "B.on_pressed"
fixstand_transitions.pop("Mimic_FlyingKick", None)
fixstand_transitions.pop("PreFlyingKick", None)
fixstand_transitions["Velocity"] = "RT + A.on_pressed"

velocity = fsm.setdefault("Velocity", {})
velocity_transitions = velocity.setdefault("transitions", {})
velocity_transitions["Passive"] = "B.on_pressed"
velocity_transitions["Mimic_Getup"] = "RB + A.on_pressed"
velocity_transitions["Mimic_FlyingKick"] = "RB + X.on_pressed"
velocity["policy_dir"] = "config/policy/velocity"

mimic_getup = fsm.setdefault("Mimic_Getup", {})
mimic_getup.setdefault("transitions", {})["Passive"] = "B.on_pressed"
mimic_getup["end_state"] = "Velocity"

fsm["Mimic_FlyingKick"] = {
    "transitions": {
        "Passive": "B.on_pressed",
        "Velocity": "RB + B.on_pressed",
    },
    "motion_file": "config/policy/mimic/flying_kick/params/flying_kick.npz",
    "policy_dir": "config/policy/mimic/flying_kick/",
    "time_start": 0.0,
    "time_end": 1000.0,
    "end_state": "Velocity",
}
fsm.pop("PreFlyingKick", None)
path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY

  printf 'Selected deploy policy: %s\n' "$POLICY_SRC"
  printf 'Prepared flying-kick motion: %s\n' "$MOTION_SRC"
}

start_ctrl() {
  local iface="${1:-}"
  if [[ -z "$iface" ]]; then
    usage
    exit 2
  fi
  if [[ "$iface" == "lo" ]]; then
    printf 'Refusing to deploy real robot on loopback interface lo.\n' >&2
    exit 2
  fi
  if [[ ! -d "/sys/class/net/$iface" ]]; then
    printf 'Network interface not found: %s\n' "$iface" >&2
    exit 2
  fi
  if [[ "$(cat "/sys/class/net/$iface/operstate" 2>/dev/null || true)" == "down" ]]; then
    printf 'Network interface is DOWN: %s\n' "$iface" >&2
    exit 2
  fi
  if pgrep -f 'unitree_mujoco|g1_ctrl' >/dev/null; then
    printf 'Refusing to start: unitree_mujoco or g1_ctrl process is already running.\n' >&2
    pgrep -af 'unitree_mujoco|g1_ctrl' >&2 || true
    exit 2
  fi

  prepare_bundle
  mkdir -p "$LOG_ROOT"
  local stamp log_dir
  stamp="$(date '+%Y%m%d-%H%M%S')"
  log_dir="$LOG_ROOT/$stamp"
  mkdir -p "$log_dir"
  cp "$ACTIVE_CONFIG" "$log_dir/config.yaml"
  sha256sum "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" > "$log_dir/hash_selected.txt"

  stop_ctrl
  tmux new-session -d -s "$CTRL_SESSION" -c "$DEPLOY/build" \
    "set -o pipefail; ./g1_ctrl --network='$iface' 2>&1 | tee '$log_dir/g1_ctrl.log'"

  printf 'Started real G1 controller in tmux session: %s\n' "$CTRL_SESSION"
  printf 'Log dir: %s\n' "$log_dir"
  printf 'Attach: tmux attach -t %s\n' "$CTRL_SESSION"
}

status() {
  tmux ls 2>/dev/null | grep -F "$CTRL_SESSION" || true
  pgrep -af 'unitree_mujoco|g1_ctrl' || true
  if [[ -f "$ACTIVE_CONFIG" ]]; then
    project_python - "$ACTIVE_CONFIG" <<'PY'
from pathlib import Path
import sys
import yaml
cfg = yaml.safe_load(Path(sys.argv[1]).read_text())
fsm = cfg["FSM"]
print("initial_state:", fsm.get("initial_state"))
print("Passive transitions:", fsm.get("Passive", {}).get("transitions"))
print("Velocity transitions:", fsm.get("Velocity", {}).get("transitions"))
print("Mimic_Getup end_state:", fsm.get("Mimic_Getup", {}).get("end_state"))
print("Mimic_FlyingKick:", fsm.get("Mimic_FlyingKick"))
PY
  fi
}

cmd="${1:-}"
case "$cmd" in
  prepare)
    prepare_bundle
    printf 'Prepared real deploy bundle and safe FSM config.\n'
    ;;
  start)
    start_ctrl "${2:-}"
    ;;
  stop)
    stop_ctrl
    ;;
  status)
    status
    ;;
  *)
    usage
    exit 2
    ;;
esac
