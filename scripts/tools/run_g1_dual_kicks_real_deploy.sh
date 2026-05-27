#!/usr/bin/env bash
set -euo pipefail

ROOT="${MJLAB_ROOT:-/home/ssy/ssy_files/mjlab}"
WORKTREE="${MJLAB_WORKTREE:-/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main}"
MJLAB_VIRTUAL_ENV="${MJLAB_VIRTUAL_ENV:-/home/ssy/ssy_files/mjlab/.venv}"
UNITREE="$ROOT/.external/unitree_rl_mjlab"
DEPLOY="$UNITREE/deploy/robots/g1"

STANDARD_EXPERIMENT_NAME="${MJLAB_STANDARD_EXPERIMENT_NAME:-g1_tracking_standard_no_state_resume2000}"
FLYING_EXPERIMENT_NAME="${MJLAB_FLYING_EXPERIMENT_NAME:-$STANDARD_EXPERIMENT_NAME}"
FLYING_RUN_NAME_PATTERN="${MJLAB_FLYING_RUN_NAME_PATTERN:-*g1_mode15_flying_kick_standard_tracking_resume4999_2000iter_*}"
FLYING_POLICY_FILENAME="${MJLAB_FLYING_POLICY_FILENAME:-flying_kick_standard_tracking_deploy_actor.onnx}"
ROUNDHOUSE_EXPERIMENT_NAME="${MJLAB_ROUNDHOUSE_EXPERIMENT_NAME:-$STANDARD_EXPERIMENT_NAME}"
ROUNDHOUSE_RUN_NAME_PATTERN="${MJLAB_ROUNDHOUSE_RUN_NAME_PATTERN:-*g1_mode15_roundhouse_leading_right_standard_tracking_resume6998_2000iter_*}"
ROUNDHOUSE_POLICY_FILENAME="${MJLAB_ROUNDHOUSE_POLICY_FILENAME:-roundhouse_standard_tracking_deploy_actor.onnx}"

FLYING_SELECTED_RUN_DIR=""
ROUNDHOUSE_SELECTED_RUN_DIR=""
FLYING_POLICY_SRC="${MJLAB_FLYING_POLICY_ONNX:-}"
ROUNDHOUSE_POLICY_SRC="${MJLAB_ROUNDHOUSE_POLICY_ONNX:-}"
FLYING_MOTION_SRC="$WORKTREE/data/motions/g1_flying_kick/mjlab/motion.npz"
ROUNDHOUSE_MOTION_SRC="$WORKTREE/data/motions/g1_roundhouse_leading_right/mjlab/motion.npz"
DEPLOY_YAML_SRC="$DEPLOY/config/policy/mimic/getup/params/deploy.yaml"

ACTIVE_CONFIG="$DEPLOY/config/config.yaml"
FLYING_STATE="Mimic_FlyingKick"
ROUNDHOUSE_STATE="Mimic_RoundhouseLeadingRight"
FLYING_POLICY_DIR="$DEPLOY/config/policy/mimic/flying_kick"
ROUNDHOUSE_POLICY_DIR="$DEPLOY/config/policy/mimic/roundhouse_leading_right"
FLYING_POLICY="$FLYING_POLICY_DIR/exported/policy.onnx"
ROUNDHOUSE_POLICY="$ROUNDHOUSE_POLICY_DIR/exported/policy.onnx"
FLYING_MOTION="$FLYING_POLICY_DIR/params/flying_kick.npz"
ROUNDHOUSE_MOTION="$ROUNDHOUSE_POLICY_DIR/params/roundhouse_leading_right.npz"
FLYING_DEPLOY_YAML="$FLYING_POLICY_DIR/params/deploy.yaml"
ROUNDHOUSE_DEPLOY_YAML="$ROUNDHOUSE_POLICY_DIR/params/deploy.yaml"

LOG_ROOT="$WORKTREE/logs/g1_dual_kicks_real_deploy"
CTRL_SESSION="g1_dual_kicks_real_ctrl"

if [[ -d "$MJLAB_VIRTUAL_ENV" ]]; then
  export VIRTUAL_ENV="$MJLAB_VIRTUAL_ENV"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
fi
export PYTHONPATH="$WORKTREE/src${PYTHONPATH:+:$PYTHONPATH}"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/tools/run_g1_dual_kicks_real_deploy.sh preflight [network-interface]
  bash scripts/tools/run_g1_dual_kicks_real_deploy.sh prepare
  MJLAB_REAL_DEPLOY_CONFIRM=YES bash scripts/tools/run_g1_dual_kicks_real_deploy.sh start <network-interface>
  bash scripts/tools/run_g1_dual_kicks_real_deploy.sh stop
  bash scripts/tools/run_g1_dual_kicks_real_deploy.sh status
  bash scripts/tools/run_g1_dual_kicks_real_deploy.sh restore <prepare-log-dir>

Policy override env vars:
  MJLAB_FLYING_POLICY_ONNX=/path/to/flying_kick_standard_tracking_deploy_actor.onnx
  MJLAB_ROUNDHOUSE_POLICY_ONNX=/path/to/roundhouse_standard_tracking_deploy_actor.onnx

Real-robot FSM:
  Passive -> [X] -> Velocity
  Passive -> [A] -> Mimic_Getup -> Velocity
  Velocity -> [B] -> Passive
  Velocity -> [RB + X] -> Mimic_FlyingKick
  Velocity -> [RB + Y] -> Mimic_RoundhouseLeadingRight
  Mimic_* -> [B] -> Passive
  Mimic_* -> [RB + B] -> Velocity
  Mimic_* ends -> Velocity
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
  local experiment="$1"
  local pattern="$2"
  local root="$WORKTREE/logs/rsl_rl/$experiment"
  if [[ ! -d "$root" ]]; then
    return 1
  fi
  find "$root" -maxdepth 1 -mindepth 1 -type d -name "$pattern" \
    -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-
}

select_policy_srcs() {
  if [[ -z "$FLYING_POLICY_SRC" ]]; then
    FLYING_SELECTED_RUN_DIR="${MJLAB_FLYING_RUN_DIR:-}"
    if [[ -z "$FLYING_SELECTED_RUN_DIR" ]]; then
      FLYING_SELECTED_RUN_DIR="$(latest_run_dir "$FLYING_EXPERIMENT_NAME" "$FLYING_RUN_NAME_PATTERN" || true)"
    fi
    if [[ -z "$FLYING_SELECTED_RUN_DIR" ]]; then
      printf 'No flying-kick run found under logs/rsl_rl/%s.\n' "$FLYING_EXPERIMENT_NAME" >&2
      return 2
    fi
    FLYING_POLICY_SRC="$FLYING_SELECTED_RUN_DIR/$FLYING_POLICY_FILENAME"
  else
    FLYING_SELECTED_RUN_DIR="$(dirname "$FLYING_POLICY_SRC")"
  fi

  if [[ -z "$ROUNDHOUSE_POLICY_SRC" ]]; then
    ROUNDHOUSE_SELECTED_RUN_DIR="${MJLAB_ROUNDHOUSE_RUN_DIR:-}"
    if [[ -z "$ROUNDHOUSE_SELECTED_RUN_DIR" ]]; then
      ROUNDHOUSE_SELECTED_RUN_DIR="$(latest_run_dir "$ROUNDHOUSE_EXPERIMENT_NAME" "$ROUNDHOUSE_RUN_NAME_PATTERN" || true)"
    fi
    if [[ -z "$ROUNDHOUSE_SELECTED_RUN_DIR" ]]; then
      printf 'No roundhouse run found under logs/rsl_rl/%s.\n' "$ROUNDHOUSE_EXPERIMENT_NAME" >&2
      return 2
    fi
    ROUNDHOUSE_POLICY_SRC="$ROUNDHOUSE_SELECTED_RUN_DIR/$ROUNDHOUSE_POLICY_FILENAME"
  else
    ROUNDHOUSE_SELECTED_RUN_DIR="$(dirname "$ROUNDHOUSE_POLICY_SRC")"
  fi
}

stop_ctrl() {
  tmux has-session -t "$CTRL_SESSION" 2>/dev/null && tmux kill-session -t "$CTRL_SESSION"
  true
}

check_real_robot_interface() {
  local iface="$1"
  if [[ "$iface" == "lo" ]]; then
    printf 'Network preflight failed: refusing real robot deploy on loopback interface lo.\n' >&2
    exit 2
  fi
  if [[ ! -d "/sys/class/net/$iface" ]]; then
    printf 'Network preflight failed: interface not found: %s\n' "$iface" >&2
    exit 2
  fi
  if [[ "$(cat "/sys/class/net/$iface/operstate" 2>/dev/null || true)" == "down" ]]; then
    printf 'Network preflight failed: interface is DOWN: %s\n' "$iface" >&2
    exit 2
  fi

  local ipv4s
  ipv4s="$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '{print $4}' || true)"
  if ! grep -q '^192\.168\.123\.' <<<"$ipv4s"; then
    if [[ "${MJLAB_ALLOW_NON_UNITREE_SUBNET:-}" != "YES" ]]; then
      printf 'Network preflight failed: %s has IPv4(s) [%s], not Unitree 192.168.123.x.\n' "$iface" "$ipv4s" >&2
      printf 'Use the wired robot interface, likely enp3s0 on this host, or set MJLAB_ALLOW_NON_UNITREE_SUBNET=YES intentionally.\n' >&2
      exit 2
    fi
    printf 'WARNING: %s is not on 192.168.123.x; continuing only because MJLAB_ALLOW_NON_UNITREE_SUBNET=YES.\n' "$iface" >&2
  fi
  printf 'Network interface ok: %s (%s)\n' "$iface" "$ipv4s"
}

check_policy_contracts() {
  project_python - "$FLYING_POLICY_SRC" "$ROUNDHOUSE_POLICY_SRC" <<'PY'
from pathlib import Path
import sys
import onnx

expected_inputs = {"obs": [1, 154]}
expected_outputs = {"actions": [1, 29]}

for label, path_arg in [("flying", sys.argv[1]), ("roundhouse", sys.argv[2])]:
    path = Path(path_arg)
    model = onnx.load(path)
    inputs = {
        item.name: [dim.dim_value for dim in item.type.tensor_type.shape.dim]
        for item in model.graph.input
    }
    outputs = {
        item.name: [dim.dim_value for dim in item.type.tensor_type.shape.dim]
        for item in model.graph.output
    }
    if inputs != expected_inputs or outputs != expected_outputs:
        raise SystemExit(
            f"{label} ONNX contract mismatch: inputs={inputs}, outputs={outputs}"
        )
    print(f"{label} ONNX contract ok: inputs={inputs}, outputs={outputs}")
PY
}

preflight_bundle() {
  local iface="${1:-}"
  select_policy_srcs
  require_file "$FLYING_POLICY_SRC"
  require_file "$ROUNDHOUSE_POLICY_SRC"
  require_file "$FLYING_MOTION_SRC"
  require_file "$ROUNDHOUSE_MOTION_SRC"
  require_file "$DEPLOY_YAML_SRC"
  require_file "$ACTIVE_CONFIG"
  require_file "$DEPLOY/build/g1_ctrl"
  check_policy_contracts

  if [[ -n "$iface" ]]; then
    check_real_robot_interface "$iface"
  fi

  if pgrep -f 'unitree_mujoco|g1_ctrl' >/dev/null; then
    printf 'Process preflight failed: unitree_mujoco or g1_ctrl is already running.\n' >&2
    pgrep -af 'unitree_mujoco|g1_ctrl' >&2 || true
    exit 2
  fi

  printf 'Selected flying-kick policy: %s\n' "$FLYING_POLICY_SRC"
  printf 'Selected roundhouse policy: %s\n' "$ROUNDHOUSE_POLICY_SRC"
  sha256sum "$FLYING_POLICY_SRC" "$ROUNDHOUSE_POLICY_SRC" "$FLYING_MOTION_SRC" "$ROUNDHOUSE_MOTION_SRC"
  printf 'Dual-kick real deploy preflight passed. This does not start the robot controller.\n'
}

new_prepare_log_dir() {
  mkdir -p "$LOG_ROOT"
  local stamp
  stamp="$(date '+%Y%m%d-%H%M%S')"
  printf '%s/%s-prepare\n' "$LOG_ROOT" "$stamp"
}

backup_existing_deploy_assets() {
  local backup_root="$1"
  mkdir -p "$backup_root"
  cp "$ACTIVE_CONFIG" "$backup_root/config.yaml.before" 2>/dev/null || true
  for path in \
    "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" \
    "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML"
  do
    if [[ -f "$path" ]]; then
      local rel
      rel="${path#"$DEPLOY"/}"
      mkdir -p "$backup_root/$(dirname "$rel")"
      cp "$path" "$backup_root/$rel"
    fi
  done
  {
    printf 'active_config=%s\n' "$ACTIVE_CONFIG"
    printf 'deploy_root=%s\n' "$DEPLOY"
    sha256sum \
      "$ACTIVE_CONFIG" \
      "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" \
      "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" \
      2>/dev/null || true
  } > "$backup_root/hash_before.txt"
}

restore_bundle() {
  local prepare_log_dir="${1:-}"
  if [[ -z "$prepare_log_dir" ]]; then
    printf 'Missing prepare log dir.\n' >&2
    usage
    exit 2
  fi
  local backup_root="$prepare_log_dir/backup"
  require_file "$backup_root/config.yaml.before"
  cp "$backup_root/config.yaml.before" "$ACTIVE_CONFIG"
  for rel in \
    "config/policy/mimic/flying_kick/exported/policy.onnx" \
    "config/policy/mimic/flying_kick/params/flying_kick.npz" \
    "config/policy/mimic/flying_kick/params/deploy.yaml" \
    "config/policy/mimic/roundhouse_leading_right/exported/policy.onnx" \
    "config/policy/mimic/roundhouse_leading_right/params/roundhouse_leading_right.npz" \
    "config/policy/mimic/roundhouse_leading_right/params/deploy.yaml"
  do
    require_file "$backup_root/$rel"
    mkdir -p "$DEPLOY/$(dirname "$rel")"
    cp "$backup_root/$rel" "$DEPLOY/$rel"
  done
  printf 'Restored deploy assets from: %s\n' "$backup_root"
}

prepare_bundle() {
  select_policy_srcs
  require_file "$FLYING_POLICY_SRC"
  require_file "$ROUNDHOUSE_POLICY_SRC"
  require_file "$FLYING_MOTION_SRC"
  require_file "$ROUNDHOUSE_MOTION_SRC"
  require_file "$DEPLOY_YAML_SRC"
  require_file "$ACTIVE_CONFIG"
  require_file "$DEPLOY/build/g1_ctrl"
  check_policy_contracts

  local prepare_log_dir
  prepare_log_dir="$(new_prepare_log_dir)"
  mkdir -p "$prepare_log_dir"
  backup_existing_deploy_assets "$prepare_log_dir/backup"

  mkdir -p \
    "$FLYING_POLICY_DIR/exported" "$FLYING_POLICY_DIR/params" \
    "$ROUNDHOUSE_POLICY_DIR/exported" "$ROUNDHOUSE_POLICY_DIR/params"
  chmod u+w \
    "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" \
    "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" \
    2>/dev/null || true
  cp "$FLYING_POLICY_SRC" "$FLYING_POLICY"
  cp "$ROUNDHOUSE_POLICY_SRC" "$ROUNDHOUSE_POLICY"
  cp "$FLYING_MOTION_SRC" "$FLYING_MOTION"
  cp "$ROUNDHOUSE_MOTION_SRC" "$ROUNDHOUSE_MOTION"
  cp "$DEPLOY_YAML_SRC" "$FLYING_DEPLOY_YAML"
  cp "$DEPLOY_YAML_SRC" "$ROUNDHOUSE_DEPLOY_YAML"
  chmod u+w \
    "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" \
    "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" \
    2>/dev/null || true

  project_python - "$ACTIVE_CONFIG" "$FLYING_STATE" "$ROUNDHOUSE_STATE" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
flying_state = sys.argv[2]
roundhouse_state = sys.argv[3]

data = yaml.safe_load(path.read_text())
fsm = data.setdefault("FSM", {})
fsm["initial_state"] = "Passive"
enabled = fsm.setdefault("_", {})
enabled["Passive"] = {"id": 1}
enabled["FixStand"] = {"id": 2}
enabled["Velocity"] = {"id": 3, "type": "RLBase"}
enabled["Mimic_Getup"] = {"id": 6, "type": "Mimic"}
enabled[flying_state] = {"id": 7, "type": "Mimic"}
enabled[roundhouse_state] = {"id": 8, "type": "Mimic"}
enabled.pop("PreFlyingKick", None)

passive = fsm.setdefault("Passive", {})
passive_transitions = passive.setdefault("transitions", {})
passive_transitions.pop("FixStand", None)
passive_transitions["Velocity"] = "X.on_pressed"
passive_transitions["Mimic_Getup"] = "A.on_pressed"

fixstand = fsm.setdefault("FixStand", {})
fixstand_transitions = fixstand.setdefault("transitions", {})
fixstand_transitions["Passive"] = "B.on_pressed"
fixstand_transitions["Velocity"] = "RT + A.on_pressed"
fixstand_transitions.pop(flying_state, None)
fixstand_transitions.pop(roundhouse_state, None)
fixstand_transitions.pop("PreFlyingKick", None)

velocity = fsm.setdefault("Velocity", {})
velocity_transitions = velocity.setdefault("transitions", {})
velocity_transitions["Passive"] = "B.on_pressed"
velocity_transitions["Mimic_Getup"] = "RB + A.on_pressed"
velocity_transitions[flying_state] = "RB + X.on_pressed"
velocity_transitions[roundhouse_state] = "RB + Y.on_pressed"
velocity_transitions.pop("PreFlyingKick", None)
velocity["policy_dir"] = "config/policy/velocity"

mimic_getup = fsm.setdefault("Mimic_Getup", {})
mimic_getup.setdefault("transitions", {})["Passive"] = "B.on_pressed"
mimic_getup["end_state"] = "Velocity"

common_transitions = {
    "Passive": "B.on_pressed",
    "Velocity": "RB + B.on_pressed",
}
fsm[flying_state] = {
    "transitions": dict(common_transitions),
    "motion_file": "config/policy/mimic/flying_kick/params/flying_kick.npz",
    "policy_dir": "config/policy/mimic/flying_kick/",
    "time_start": 0.0,
    "time_end": 1000.0,
    "end_state": "Velocity",
}
fsm[roundhouse_state] = {
    "transitions": dict(common_transitions),
    "motion_file": "config/policy/mimic/roundhouse_leading_right/params/roundhouse_leading_right.npz",
    "policy_dir": "config/policy/mimic/roundhouse_leading_right/",
    "time_start": 0.0,
    "time_end": 1000.0,
    "end_state": "Velocity",
}
fsm.pop("PreFlyingKick", None)
path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY

  printf 'Selected flying-kick policy: %s\n' "$FLYING_POLICY_SRC"
  printf 'Selected roundhouse policy: %s\n' "$ROUNDHOUSE_POLICY_SRC"
  printf 'Prepared flying-kick motion: %s\n' "$FLYING_MOTION_SRC"
  printf 'Prepared roundhouse motion: %s\n' "$ROUNDHOUSE_MOTION_SRC"
  cp "$ACTIVE_CONFIG" "$prepare_log_dir/config.yaml.after"
  sha256sum \
    "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" \
    "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" \
    > "$prepare_log_dir/hash_selected.txt"
  printf 'Prepare log dir: %s\n' "$prepare_log_dir"
}

start_ctrl() {
  local iface="${1:-}"
  if [[ -z "$iface" ]]; then
    usage
    exit 2
  fi
  if [[ "${MJLAB_REAL_DEPLOY_CONFIRM:-}" != "YES" ]]; then
    printf 'Refusing to start real controller without MJLAB_REAL_DEPLOY_CONFIRM=YES.\n' >&2
    printf 'This command sends commands to the physical robot.\n' >&2
    exit 2
  fi
  check_real_robot_interface "$iface"
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
  sha256sum \
    "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" \
    "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" \
    > "$log_dir/hash_selected.txt"

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
    project_python - "$ACTIVE_CONFIG" "$FLYING_STATE" "$ROUNDHOUSE_STATE" <<'PY'
from pathlib import Path
import sys
import yaml

cfg = yaml.safe_load(Path(sys.argv[1]).read_text())
flying_state = sys.argv[2]
roundhouse_state = sys.argv[3]
fsm = cfg["FSM"]
print("initial_state:", fsm.get("initial_state"))
print("Passive transitions:", fsm.get("Passive", {}).get("transitions"))
print("Velocity transitions:", fsm.get("Velocity", {}).get("transitions"))
print("Mimic_Getup end_state:", fsm.get("Mimic_Getup", {}).get("end_state"))
print(f"{flying_state}:", fsm.get(flying_state))
print(f"{roundhouse_state}:", fsm.get(roundhouse_state))
PY
  fi
  if [[ -f "$FLYING_POLICY" && -f "$FLYING_MOTION" && -f "$FLYING_DEPLOY_YAML" ]]; then
    sha256sum "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML"
  fi
  if [[ -f "$ROUNDHOUSE_POLICY" && -f "$ROUNDHOUSE_MOTION" && -f "$ROUNDHOUSE_DEPLOY_YAML" ]]; then
    sha256sum "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML"
  fi
}

cmd="${1:-}"
case "$cmd" in
  preflight)
    preflight_bundle "${2:-}"
    ;;
  prepare)
    prepare_bundle
    printf 'Prepared dual-kick real deploy bundle and safe FSM config.\n'
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
  restore)
    restore_bundle "${2:-}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
