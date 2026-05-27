#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${MJLAB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
WORKTREE="${MJLAB_WORKTREE:-$ROOT}"
MJLAB_VIRTUAL_ENV="${MJLAB_VIRTUAL_ENV:-$ROOT/.venv}"
UNITREE="$ROOT/.external/unitree_rl_mjlab"
DEPLOY="$UNITREE/deploy/robots/g1"
SIM="$UNITREE/simulate"

EXPERIMENT_NAME="${MJLAB_EXPERIMENT_NAME:-g1_tracking_acrobatics_no_state}"
RUN_NAME_PATTERN="${MJLAB_RUN_NAME_PATTERN:-*g1_mode15_roundhouse_leading_right_4096env_5000iter*}"
SELECTED_RUN_DIR=""
POLICY_SRC="${MJLAB_POLICY_ONNX:-}"
MOTION_SRC="$WORKTREE/data/motions/g1_roundhouse_leading_right/mjlab/motion.npz"
DEPLOY_YAML_SRC="$DEPLOY/config/policy/mimic/getup/params/deploy.yaml"
SIM2SIM_MODE="${MJLAB_SIM2SIM_MODE:-stand}"
CONFIG_SIM2SIM_MODE="$SIM2SIM_MODE"
ENABLE_ELASTIC_BAND="${MJLAB_ENABLE_ELASTIC_BAND:-}"
START_PAUSED="${MJLAB_START_PAUSED:-}"
AUTO_RUN_AFTER_READY="${MJLAB_AUTO_RUN_AFTER_READY:-}"
POLICY_START_GATE_SECONDS="${MJLAB_PHASE1_POLICY_START_GATE_SECONDS:-}"
POLICY_LOWSTATE_TICK_GATE="${MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE:-}"
POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS="${MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS:-}"
VELOCITY_BOOTSTRAP_POSE="${MJLAB_VELOCITY_BOOTSTRAP_POSE:-stand}"
VELOCITY_BOOTSTRAP_ROOT="${MJLAB_VELOCITY_BOOTSTRAP_ROOT:-stand}"

ACTIVE_CONFIG="$DEPLOY/config/config.yaml"
SIM_CONFIG="$SIM/config.yaml"
if [[ -n "${MJLAB_VELOCITY_POLICY_ROOT:-}" ]]; then
  VELOCITY_POLICY_ROOT="$(realpath "$MJLAB_VELOCITY_POLICY_ROOT")"
  VELOCITY_POLICY_DIR_CONFIG="$VELOCITY_POLICY_ROOT"
else
  VELOCITY_POLICY_ROOT="$DEPLOY/config/policy/velocity"
  VELOCITY_POLICY_DIR_CONFIG="config/policy/velocity"
fi
ROUNDHOUSE_POLICY_DIR="$DEPLOY/config/policy/mimic/roundhouse_leading_right"
ROUNDHOUSE_POLICY="$ROUNDHOUSE_POLICY_DIR/exported/policy.onnx"
ROUNDHOUSE_MOTION="$ROUNDHOUSE_POLICY_DIR/params/roundhouse_leading_right.npz"
ROUNDHOUSE_DEPLOY_YAML="$ROUNDHOUSE_POLICY_DIR/params/deploy.yaml"
MODE15_XML="$WORKTREE/src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml"
MODE15_ASSET_DIR="$WORKTREE/src/mjlab/asset_zoo/robots/unitree_g1/xmls/assets"
SIM_XML_DIR="$UNITREE/src/assets/robots/unitree_g1/xmls"
SIM_SCENE_XML="$SIM_XML_DIR/scene_g1.xml"
SIM_G1_XML="$SIM_XML_DIR/g1.xml"
SIM_ASSET_DIR="$SIM_XML_DIR/assets"

STAND_INITIAL_QPOS='[0.0, 0.0, 0.7657805681228638, 1.0, 0.0, 0.0, 0.0, -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, 0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0]'
HOME_ROOT_Z='0.783675'

ARTIFACT_ROOT="$WORKTREE/logs/roundhouse_leading_right_sim2sim"
SIM_SESSION="roundhouse_leading_right_sim"
CTRL_SESSION="roundhouse_leading_right_ctrl"

if [[ -d "$MJLAB_VIRTUAL_ENV" ]]; then
  export VIRTUAL_ENV="$MJLAB_VIRTUAL_ENV"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
fi

usage() {
  cat <<'EOF'
Usage:
  scripts/tools/run_roundhouse_leading_right_sim2sim.sh prepare-lane --official-root <unitree-root> --out-root <lane-root> --policy-root <policy-root> [unitree-sim2sim args...]
  scripts/tools/run_roundhouse_leading_right_sim2sim.sh start
  scripts/tools/run_roundhouse_leading_right_sim2sim.sh stop
  scripts/tools/run_roundhouse_leading_right_sim2sim.sh restore <artifact-dir>
  scripts/tools/run_roundhouse_leading_right_sim2sim.sh status

Productized lane preparation:
  uv run unitree-sim2sim prepare-g1 --official-root <unitree-root> --out-root <lane-root> --action roundhouse_leading_right --policy-root <policy-root>

Before start, export the latest trained actor with:
  bash scripts/tools/run_g1_mode15_roundhouse_leading_right_train.sh export

Environment:
  MJLAB_SIM2SIM_MODE=stand       # default: FixStand from deploy default pose
  MJLAB_SIM2SIM_MODE=play_parity # debug only: start from motion first frame
  MJLAB_SIM2SIM_MODE=prepose     # deploy-style: FixStand interpolates to motion frame 0 before Mimic trigger
  MJLAB_SIM2SIM_MODE=official_bootstrap
                                 # Unitree-style sim bootstrap: stand qpos + elastic band
  MJLAB_SIM2SIM_MODE=official_velocity_bootstrap
                                 # Unitree-style: FixStand bootstrap, then sim-only auto-switch to Velocity
  MJLAB_SIM2SIM_MODE=velocity_bootstrap
                                 # diagnostic: start Velocity from stand qpos + elastic band
  MJLAB_SIM2SIM_MODE=passive_velocity_bootstrap
                                 # diagnostic: start Passive, then sim-only auto-switch to Velocity
  MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default
                                 # optional; velocity bootstrap modes only, default is stand
  MJLAB_VELOCITY_BOOTSTRAP_ROOT=home
                                 # optional; velocity bootstrap modes + policy_default only, default is stand
  MJLAB_VELOCITY_POLICY_ROOT=/path/to/policy_dir
                                 # optional; velocity bootstrap modes policy_dir and policy-default pose source
  MJLAB_ENABLE_ELASTIC_BAND=1    # optional; default is 0
  MJLAB_START_PAUSED=0           # optional; stand mode defaults to 1
  MJLAB_AUTO_RUN_AFTER_READY=1   # optional; default is 0 because stand controller is unstable
  MJLAB_PHASE1_POLICY_START_GATE_SECONDS=5.0
                                 # optional; delays Velocity env->step after state entry
  MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE=1
                                 # optional; waits for fresh lowstate tick before each Velocity env->step
  MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS=1.0
                                 # optional; timeout while waiting for fresh lowstate tick
  MJLAB_ELASTIC_PRETENSION_STEPS=24
                                 # optional; key-8 presses before Run when elastic band is enabled
  MJLAB_OFFICIAL_BOOTSTRAP_PRETENSION_STEPS=24
                                 # legacy alias for MJLAB_ELASTIC_PRETENSION_STEPS
  MJLAB_ELASTIC_DROP_STEPS=0
                                 # optional; extra key-8 presses after Run
  MJLAB_OFFICIAL_BOOTSTRAP_DROP_STEPS=0
                                 # legacy alias for MJLAB_ELASTIC_DROP_STEPS
  MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=1
                                 # optional; send MuJoCo key 9 after Run
  MJLAB_ELASTIC_RELEASE_DELAY_SECONDS=1.0
                                 # optional; delay before automatic key-9 release
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

validate_mode() {
  case "$SIM2SIM_MODE" in
    stand|play_parity|prepose|official_bootstrap|official_velocity_bootstrap|velocity_bootstrap|passive_velocity_bootstrap)
      ;;
    *)
      printf 'Unsupported MJLAB_SIM2SIM_MODE=%s. Use stand, play_parity, prepose, official_bootstrap, official_velocity_bootstrap, velocity_bootstrap, or passive_velocity_bootstrap.\n' "$SIM2SIM_MODE" >&2
      exit 2
      ;;
  esac
  case "$VELOCITY_BOOTSTRAP_POSE" in
    stand|policy_default)
      ;;
    *)
      printf 'Unsupported MJLAB_VELOCITY_BOOTSTRAP_POSE=%s. Use stand or policy_default.\n' "$VELOCITY_BOOTSTRAP_POSE" >&2
      exit 2
      ;;
  esac
  case "$VELOCITY_BOOTSTRAP_ROOT" in
    stand|home)
      ;;
    *)
      printf 'Unsupported MJLAB_VELOCITY_BOOTSTRAP_ROOT=%s. Use stand or home.\n' "$VELOCITY_BOOTSTRAP_ROOT" >&2
      exit 2
      ;;
  esac
}

configure_mode_defaults() {
  CONFIG_SIM2SIM_MODE="$SIM2SIM_MODE"
  if [[ "$SIM2SIM_MODE" == "official_bootstrap" ]]; then
    CONFIG_SIM2SIM_MODE="stand"
    ENABLE_ELASTIC_BAND="${ENABLE_ELASTIC_BAND:-1}"
    START_PAUSED="${START_PAUSED:-1}"
    AUTO_RUN_AFTER_READY="${AUTO_RUN_AFTER_READY:-0}"
  elif [[ "$SIM2SIM_MODE" == "official_velocity_bootstrap" ]]; then
    ENABLE_ELASTIC_BAND="${ENABLE_ELASTIC_BAND:-1}"
    START_PAUSED="${START_PAUSED:-1}"
    AUTO_RUN_AFTER_READY="${AUTO_RUN_AFTER_READY:-0}"
  elif [[ "$SIM2SIM_MODE" == "velocity_bootstrap" || "$SIM2SIM_MODE" == "passive_velocity_bootstrap" ]]; then
    ENABLE_ELASTIC_BAND="${ENABLE_ELASTIC_BAND:-1}"
    START_PAUSED="${START_PAUSED:-1}"
    AUTO_RUN_AFTER_READY="${AUTO_RUN_AFTER_READY:-0}"
  else
    ENABLE_ELASTIC_BAND="${ENABLE_ELASTIC_BAND:-0}"
    START_PAUSED="${START_PAUSED:-1}"
    AUTO_RUN_AFTER_READY="${AUTO_RUN_AFTER_READY:-0}"
  fi
}

configure_policy_start_gate() {
  if [[ "$START_PAUSED" == "1" && "$AUTO_RUN_AFTER_READY" == "1" ]]; then
    case "$SIM2SIM_MODE" in
      official_velocity_bootstrap|velocity_bootstrap|passive_velocity_bootstrap)
        POLICY_START_GATE_SECONDS="${POLICY_START_GATE_SECONDS:-5.0}"
        ;;
    esac
  fi
}

validate_policy_start_gate() {
  if [[ -n "$POLICY_START_GATE_SECONDS" && ! "$POLICY_START_GATE_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    printf 'Unsupported MJLAB_PHASE1_POLICY_START_GATE_SECONDS=%s. Use a non-negative number of seconds.\n' "$POLICY_START_GATE_SECONDS" >&2
    exit 2
  fi
  if [[ -n "$POLICY_LOWSTATE_TICK_GATE" && ! "$POLICY_LOWSTATE_TICK_GATE" =~ ^[01]$ ]]; then
    printf 'Unsupported MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE=%s. Use 0 or 1.\n' "$POLICY_LOWSTATE_TICK_GATE" >&2
    exit 2
  fi
  if [[ -n "$POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS" && ! "$POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    printf 'Unsupported MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS=%s. Use a non-negative number of seconds.\n' "$POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS" >&2
    exit 2
  fi
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
    printf 'Start training with: bash scripts/tools/run_g1_mode15_roundhouse_leading_right_train.sh start\n' >&2
    return 2
  fi
  POLICY_SRC="$SELECTED_RUN_DIR/roundhouse_leading_right_deploy_actor.onnx"
}

hash_manifest() {
  local out="$1"
  {
    date --iso-8601=seconds
    sha256sum "$POLICY_SRC"
    sha256sum "$MOTION_SRC"
    sha256sum "$ROUNDHOUSE_POLICY"
    sha256sum "$ROUNDHOUSE_MOTION"
    sha256sum "$ROUNDHOUSE_DEPLOY_YAML"
    sha256sum "$ACTIVE_CONFIG"
    sha256sum "$SIM_CONFIG"
    stat -c '%y %s %n' \
      "$POLICY_SRC" "$MOTION_SRC" "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" \
      "$ROUNDHOUSE_DEPLOY_YAML" "$ACTIVE_CONFIG" "$SIM_CONFIG"
  } > "$out"
}

stop_sessions() {
  tmux has-session -t "$CTRL_SESSION" 2>/dev/null && tmux kill-session -t "$CTRL_SESSION"
  tmux has-session -t "$SIM_SESSION" 2>/dev/null && tmux kill-session -t "$SIM_SESSION"
  true
}

case_log() {
  local path="$1"
  shift
  printf "$@" | tee -a "$path"
}

log_has() {
  local pattern="$1"
  local path="$2"
  [[ -f "$path" ]] || return 1
  if command -v rg >/dev/null 2>&1; then
    rg -q "$pattern" "$path"
  else
    grep -Eq "$pattern" "$path"
  fi
}

patch_config() {
  project_python - "$ACTIVE_CONFIG" "$CONFIG_SIM2SIM_MODE" "$STAND_INITIAL_QPOS" "$MOTION_SRC" "$VELOCITY_POLICY_DIR_CONFIG" "$VELOCITY_BOOTSTRAP_POSE" "$VELOCITY_POLICY_ROOT" <<'PY'
from ast import literal_eval
from pathlib import Path
import numpy as np
import sys
import yaml

path = Path(sys.argv[1])
mode = sys.argv[2]
stand_qpos = literal_eval(sys.argv[3])
motion_path = Path(sys.argv[4])
velocity_policy_dir_config = sys.argv[5]
velocity_bootstrap_pose = sys.argv[6]
velocity_policy_root = Path(sys.argv[7])
data = yaml.safe_load(path.read_text())


def resolve_policy_dir(path: Path) -> Path:
    if (path / "exported").is_dir():
        return path
    candidates = sorted(child for child in path.iterdir() if child.is_dir())
    for candidate in reversed(candidates):
        if (candidate / "exported").is_dir():
            return candidate
    return path


def velocity_fixstand_target() -> list[float]:
    if velocity_bootstrap_pose != "policy_default":
        return stand_qpos[7:]
    policy_dir = resolve_policy_dir(velocity_policy_root)
    deploy_yaml = yaml.safe_load((policy_dir / "params/deploy.yaml").read_text())
    return [float(value) for value in deploy_yaml["default_joint_pos"]]

fsm = data.setdefault("FSM", {})
enabled = fsm.setdefault("_", {})
enabled["Mimic_RoundhouseLeadingRight"] = {"id": 8, "type": "Mimic"}
enabled["Passive"] = {"id": 1}
enabled["Velocity"] = {"id": 3, "type": "RLBase"}
if mode == "play_parity":
    fsm["initial_state"] = "Mimic_RoundhouseLeadingRight"
elif mode == "velocity_bootstrap":
    fsm["initial_state"] = "Velocity"
elif mode == "passive_velocity_bootstrap":
    fsm["initial_state"] = "Passive"
else:
    fsm["initial_state"] = "FixStand"
passive = fsm.setdefault("Passive", {})
passive_transitions = passive.setdefault("transitions", {})
passive_transitions["Velocity"] = "!A" if mode == "passive_velocity_bootstrap" else "X.on_pressed"
fixstand = fsm.setdefault("FixStand", {})
transitions = fixstand.setdefault("transitions", {})
transitions.pop("Mimic_FlyingKick", None)
transitions.pop("PreFlyingKick", None)
transitions["Mimic_RoundhouseLeadingRight"] = "RB + X.on_pressed"
if mode == "official_velocity_bootstrap":
    transitions["Velocity"] = "!A"
if mode in {"stand", "play_parity", "velocity_bootstrap", "passive_velocity_bootstrap"}:
    fixstand["qs"] = [[], stand_qpos[7:]]
elif mode == "official_velocity_bootstrap":
    fixstand["qs"] = [[], velocity_fixstand_target()]
elif mode == "prepose":
    with np.load(motion_path) as motion:
        fixstand["qs"] = [[], motion["joint_pos"][0].astype(float).tolist()]
velocity = fsm.setdefault("Velocity", {})
velocity_transitions = velocity.setdefault("transitions", {})
velocity_transitions["Passive"] = "B.on_pressed"
velocity_transitions["Mimic_RoundhouseLeadingRight"] = "RB + Y.on_pressed"
velocity["policy_dir"] = velocity_policy_dir_config
fsm["Mimic_RoundhouseLeadingRight"] = {
    "transitions": {
        "Passive": "B.on_pressed",
        "Velocity": "RB + B.on_pressed",
    },
    "motion_file": "config/policy/mimic/roundhouse_leading_right/params/roundhouse_leading_right.npz",
    "policy_dir": "config/policy/mimic/roundhouse_leading_right/",
    "time_start": 0.0,
    "time_end": 1000.0,
    "end_state": "Velocity",
}
path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY
}

patch_sim_config() {
  project_python - "$SIM_CONFIG" "$CONFIG_SIM2SIM_MODE" "$STAND_INITIAL_QPOS" "$MOTION_SRC" "$ENABLE_ELASTIC_BAND" "$START_PAUSED" "$VELOCITY_BOOTSTRAP_POSE" "$VELOCITY_POLICY_ROOT" "$VELOCITY_BOOTSTRAP_ROOT" "$HOME_ROOT_Z" <<'PY'
from ast import literal_eval
from pathlib import Path
import numpy as np
import sys
import yaml

path = Path(sys.argv[1])
mode = sys.argv[2]
stand_qpos = literal_eval(sys.argv[3])
motion_path = Path(sys.argv[4])
enable_elastic_band = int(sys.argv[5])
start_paused = int(sys.argv[6])
velocity_bootstrap_pose = sys.argv[7]
velocity_policy_root = Path(sys.argv[8])
velocity_bootstrap_root = sys.argv[9]
home_root_z = float(sys.argv[10])


def resolve_policy_dir(path: Path) -> Path:
    if (path / "exported").is_dir():
        return path
    candidates = sorted(child for child in path.iterdir() if child.is_dir())
    for candidate in reversed(candidates):
        if (candidate / "exported").is_dir():
            return candidate
    return path

if mode == "play_parity":
    motion = np.load(motion_path)
    root_pos = motion["body_pos_w"][0, 0].astype(float).tolist()
    root_quat = motion["body_quat_w"][0, 0].astype(float).tolist()
    joint_pos = motion["joint_pos"][0].astype(float).tolist()
    qpos = [0.0, 0.0, root_pos[2], *root_quat, *joint_pos]
elif mode in {"official_velocity_bootstrap", "velocity_bootstrap", "passive_velocity_bootstrap"} and velocity_bootstrap_pose == "policy_default":
    policy_dir = resolve_policy_dir(velocity_policy_root)
    deploy_yaml = yaml.safe_load((policy_dir / "params/deploy.yaml").read_text())
    root_qpos = list(stand_qpos[:7])
    if velocity_bootstrap_root == "home":
        root_qpos[2] = home_root_z
    qpos = [*root_qpos, *deploy_yaml["default_joint_pos"]]
else:
    qpos = stand_qpos

data = yaml.safe_load(path.read_text())
data["initial_qpos"] = qpos
data["enable_elastic_band"] = enable_elastic_band
data["start_paused"] = start_paused
path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY
}

unpause_when_ready() {
  local case_dir="$1"
  if [[ "$AUTO_RUN_AFTER_READY" != "1" || "$START_PAUSED" != "1" ]]; then
    return
  fi
  local helper_log="$case_dir/bootstrap_helper.log"
  : > "$helper_log"
  local ready_state="FixStand"
  local transition_from_passive=0
  local transition_from_fixstand=0
  if [[ "$SIM2SIM_MODE" == "velocity_bootstrap" ]]; then
    ready_state="Velocity"
  elif [[ "$SIM2SIM_MODE" == "passive_velocity_bootstrap" ]]; then
    ready_state="Passive"
    transition_from_passive=1
  elif [[ "$SIM2SIM_MODE" == "official_velocity_bootstrap" ]]; then
    ready_state="FixStand"
    transition_from_fixstand=1
  fi
  local window_id=""
  local state_ready=0
  for _ in $(seq 1 120); do
    if [[ "$state_ready" != "1" && "$transition_from_passive" == "1" ]] && log_has "FSM: Change state from Passive to Velocity|FSM: Start Velocity" "$case_dir/g1_ctrl.log"; then
      ready_state="Velocity"
      state_ready=1
    elif [[ "$state_ready" != "1" && "$transition_from_fixstand" == "1" ]] && log_has "FSM: Change state from FixStand to Velocity|FSM: Start Velocity" "$case_dir/g1_ctrl.log"; then
      ready_state="Velocity"
      state_ready=1
    elif [[ "$state_ready" != "1" ]] && log_has "FSM: Start $ready_state" "$case_dir/g1_ctrl.log"; then
      if [[ "$transition_from_passive" == "1" ]]; then
        for _ in $(seq 1 120); do
          if log_has "FSM: Change state from Passive to Velocity|FSM: Start Velocity" "$case_dir/g1_ctrl.log"; then
            ready_state="Velocity"
            state_ready=1
            break
          fi
          sleep 0.1
        done
        if [[ "$ready_state" != "Velocity" ]]; then
          case_log "$helper_log" 'Warning: Passive did not transition to Velocity through the sim-only auto condition; check the selected Passive transition in config.yaml.\n' >&2
          return
        fi
      elif [[ "$transition_from_fixstand" == "1" ]]; then
        for _ in $(seq 1 120); do
          if log_has "FSM: Change state from FixStand to Velocity|FSM: Start Velocity" "$case_dir/g1_ctrl.log"; then
            ready_state="Velocity"
            state_ready=1
            break
          fi
          sleep 0.1
        done
        if [[ "$ready_state" != "Velocity" ]]; then
          case_log "$helper_log" 'Warning: FixStand did not transition to Velocity through the sim-only auto condition; check the selected FixStand transition in config.yaml.\n' >&2
          return
        fi
      else
        state_ready=1
      fi
    fi
    if [[ "$state_ready" == "1" ]]; then
      window_id="$(xdotool search --name 'MuJoCo : scene_g1' 2>/dev/null | tail -n 1 || true)"
      if [[ -n "$window_id" ]]; then
        xdotool windowactivate --sync "$window_id" >/dev/null 2>&1 || xdotool windowfocus "$window_id" >/dev/null 2>&1 || true
        if [[ "$ENABLE_ELASTIC_BAND" == "1" ]]; then
          local pretension_steps="${MJLAB_ELASTIC_PRETENSION_STEPS:-${MJLAB_OFFICIAL_BOOTSTRAP_PRETENSION_STEPS:-24}}"
          if [[ "$pretension_steps" =~ ^[0-9]+$ ]]; then
            for _ in $(seq 1 "$pretension_steps"); do
              xdotool key --window "$window_id" 8 >/dev/null 2>&1 || true
              sleep 0.1
            done
          fi
          case_log "$helper_log" 'Prepared elastic length with MuJoCo key 8 before Run.\n'
        else
          xdotool key --window "$window_id" space >/dev/null 2>&1 || true
        fi
        # On the local GLFW/X11 path, the space hotkey can be swallowed even
        # though xdotool exits successfully. The MuJoCo Run radio is stable in
        # the default left panel, so click it as the deterministic fallback.
        for _ in $(seq 1 4); do
          xdotool mousemove --window "$window_id" 183 428 click 1 >/dev/null 2>&1 || true
          sleep 0.3
        done
        if [[ "$ENABLE_ELASTIC_BAND" == "1" ]]; then
          local drop_steps="${MJLAB_ELASTIC_DROP_STEPS:-${MJLAB_OFFICIAL_BOOTSTRAP_DROP_STEPS:-0}}"
          if [[ "$drop_steps" =~ ^[0-9]+$ ]]; then
            for _ in $(seq 1 "$drop_steps"); do
              xdotool key --window "$window_id" 8 >/dev/null 2>&1 || true
              sleep 0.2
            done
          fi
          if [[ "$drop_steps" != "0" ]]; then
            case_log "$helper_log" 'Requested extra elastic-band lowering with MuJoCo key 8 after Run.\n'
          fi
          if [[ "${MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN:-0}" == "1" ]]; then
            local release_delay="${MJLAB_ELASTIC_RELEASE_DELAY_SECONDS:-1.0}"
            sleep "$release_delay"
            xdotool key --window "$window_id" 9 >/dev/null 2>&1 || true
            case_log "$helper_log" 'Released elastic band with MuJoCo key 9 after Run delay %ss.\n' "$release_delay"
          else
            case_log "$helper_log" 'Release elastic band manually with key 9 only after the policy state is stable.\n'
          fi
        fi
        case_log "$helper_log" 'Requested MuJoCo Run after %s was ready; verify the window is running before accepting evidence.\n' "$ready_state"
        return
      fi
    fi
    sleep 0.1
  done
  case_log "$helper_log" 'Warning: MuJoCo stayed paused; press Run in the MuJoCo window after %s is ready.\n' "$ready_state" >&2
}

sync_mode15_sim_model() {
  require_file "$MODE15_XML"
  require_file "$SIM_SCENE_XML"
  mkdir -p "$SIM_ASSET_DIR"
  cp "$MODE15_ASSET_DIR"/*_5010.STL "$SIM_ASSET_DIR"/
  cp "$MODE15_XML" "$SIM_G1_XML"

  project_python - "$MODE15_XML" "$SIM_SCENE_XML" <<'PY'
from pathlib import Path
import copy
import sys
import xml.etree.ElementTree as ET

mode15_xml = Path(sys.argv[1])
scene_xml = Path(sys.argv[2])

mode15 = ET.parse(mode15_xml).getroot()
current_scene = ET.parse(scene_xml).getroot()

scene = ET.Element("mujoco", {"model": "scene_g1"})
compiler = copy.deepcopy(mode15.find("compiler"))
compiler.set("meshdir", "assets")
scene.append(compiler)
scene.append(copy.deepcopy(mode15.find("default")))

asset = ET.Element("asset")
for child in list(mode15.find("asset")):
  asset.append(copy.deepcopy(child))
existing_names = {child.attrib.get("name") for child in asset if child.attrib.get("name")}
for old_asset in current_scene.findall("asset"):
  for child in list(old_asset):
    name = child.attrib.get("name")
    if child.tag == "mesh" or (name and name in existing_names):
      continue
    asset.append(copy.deepcopy(child))
scene.append(asset)

world = ET.Element("worldbody")
for child in list(mode15.find("worldbody")):
  world.append(copy.deepcopy(child))
for old_world in current_scene.findall("worldbody"):
  for child in list(old_world):
    if child.tag == "body" and child.attrib.get("name") == "pelvis":
      continue
    if child.tag == "light" and child.attrib.get("mode") == "trackcom":
      continue
    world.append(copy.deepcopy(child))
scene.append(world)

contact = mode15.find("contact")
if contact is not None:
  scene.append(copy.deepcopy(contact))

actuator = current_scene.find("actuator")
if actuator is None:
  raise RuntimeError(f"Missing actuator block in {scene_xml}")
scene.append(copy.deepcopy(actuator))

sensor = current_scene.find("sensor")
if sensor is None:
  raise RuntimeError(f"Missing sensor block in {scene_xml}")
sensor = copy.deepcopy(sensor)
for elem in sensor:
  if elem.tag == "framequat" and elem.attrib.get("name") == "imu_quat":
    elem.set("objname", "imu_in_pelvis")
  if elem.tag in {"gyro", "accelerometer"} and elem.attrib.get("name") in {"imu_gyro", "imu_acc"}:
    elem.set("site", "imu_in_pelvis")
  if elem.tag in {"framepos", "framelinvel"} and elem.attrib.get("name") in {"frame_pos", "frame_vel"}:
    elem.set("objname", "imu_in_pelvis")
scene.append(sensor)

for tag in ("statistic", "visual"):
  elem = current_scene.find(tag)
  if elem is not None:
    scene.append(copy.deepcopy(elem))

ET.indent(scene, space="  ")
scene_xml.write_text(ET.tostring(scene, encoding="unicode") + "\n")
PY

  if [[ -x "$SIM/mujoco/bin/compile" ]]; then
    mkdir -p "$ARTIFACT_ROOT/.compile_check"
    (
      cd "$SIM_XML_DIR"
      "$SIM/mujoco/bin/compile" "$SIM_SCENE_XML" "$ARTIFACT_ROOT/.compile_check/scene_g1.mjb" >/dev/null
    )
  fi
}

prepare_product_lane() {
  (
    cd "$ROOT"
    uv run unitree-sim2sim prepare-g1 --action roundhouse_leading_right "$@"
  )
}

start_case() {
  validate_mode
  configure_mode_defaults
  configure_policy_start_gate
  validate_policy_start_gate
  select_policy_src
  if [[ ! -f "$POLICY_SRC" ]]; then
    printf 'Missing exported deploy ONNX: %s\n' "$POLICY_SRC" >&2
    printf 'After the 5000-iter train has a checkpoint, run: bash scripts/tools/run_g1_mode15_roundhouse_leading_right_train.sh export\n' >&2
    exit 2
  fi
  require_file "$POLICY_SRC"
  require_file "$MOTION_SRC"
  require_file "$DEPLOY_YAML_SRC"
  require_file "$ACTIVE_CONFIG"
  require_file "$SIM_CONFIG"
  sync_mode15_sim_model
  require_file "$SIM/build/unitree_mujoco"
  require_file "$DEPLOY/build/g1_ctrl"

  local stamp case_dir
  stamp="$(date '+%Y%m%d-%H%M%S')"
  case_dir="$ARTIFACT_ROOT/$stamp"
  mkdir -p "$case_dir/active_before" "$case_dir/selected" \
    "$ROUNDHOUSE_POLICY_DIR/exported" "$ROUNDHOUSE_POLICY_DIR/params"

  cp "$ACTIVE_CONFIG" "$case_dir/active_before/config.yaml"
  cp "$SIM_CONFIG" "$case_dir/active_before/simulate_config.yaml"
  if [[ -f "$ROUNDHOUSE_POLICY" ]]; then cp "$ROUNDHOUSE_POLICY" "$case_dir/active_before/policy.onnx"; fi
  if [[ -f "$ROUNDHOUSE_MOTION" ]]; then cp "$ROUNDHOUSE_MOTION" "$case_dir/active_before/roundhouse_leading_right.npz"; fi
  if [[ -f "$ROUNDHOUSE_DEPLOY_YAML" ]]; then cp "$ROUNDHOUSE_DEPLOY_YAML" "$case_dir/active_before/deploy.yaml"; fi

  chmod u+w "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" 2>/dev/null || true
  cp "$POLICY_SRC" "$ROUNDHOUSE_POLICY"
  cp "$MOTION_SRC" "$ROUNDHOUSE_MOTION"
  cp "$DEPLOY_YAML_SRC" "$ROUNDHOUSE_DEPLOY_YAML"
  chmod u+w "$ROUNDHOUSE_POLICY" "$ROUNDHOUSE_MOTION" "$ROUNDHOUSE_DEPLOY_YAML" 2>/dev/null || true
  cp "$POLICY_SRC" "$case_dir/selected/policy.onnx"
  cp "$MOTION_SRC" "$case_dir/selected/roundhouse_leading_right.npz"
  cp "$DEPLOY_YAML_SRC" "$case_dir/selected/deploy.yaml"
  patch_config
  patch_sim_config
  cp "$ACTIVE_CONFIG" "$case_dir/selected/config.yaml"
  cp "$SIM_CONFIG" "$case_dir/selected/simulate_config.yaml"
  hash_manifest "$case_dir/hash_selected.txt"

  stop_sessions
  tmux new-session -d -s "$CTRL_SESSION" -c "$DEPLOY/build" \
    "set -o pipefail; MJLAB_PHASE1_POLICY_START_GATE_SECONDS='$POLICY_START_GATE_SECONDS' MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE='$POLICY_LOWSTATE_TICK_GATE' MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS='$POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS' ./g1_ctrl --network=lo 2>&1 | tee '$case_dir/g1_ctrl.log'"
  sleep 0.5
  tmux new-session -d -s "$SIM_SESSION" -c "$UNITREE" \
    "set -o pipefail; ./simulate/build/unitree_mujoco 2>&1 | tee '$case_dir/unitree_mujoco.log'"
  unpause_when_ready "$case_dir"

  printf 'Started roundhouse-leading-right sim2sim\n'
  printf 'Selected policy: %s\n' "$POLICY_SRC"
  printf 'Mode: %s; config mode: %s; velocity pose: %s; velocity root: %s; velocity policy root: %s; elastic band: %s; start paused: %s; auto run: %s; policy start gate: %ss; lowstate tick gate: %s\n' "$SIM2SIM_MODE" "$CONFIG_SIM2SIM_MODE" "$VELOCITY_BOOTSTRAP_POSE" "$VELOCITY_BOOTSTRAP_ROOT" "$VELOCITY_POLICY_ROOT" "$ENABLE_ELASTIC_BAND" "$START_PAUSED" "$AUTO_RUN_AFTER_READY" "${POLICY_START_GATE_SECONDS:-0}" "${POLICY_LOWSTATE_TICK_GATE:-0}"
  printf 'Artifact dir: %s\n' "$case_dir"
  printf 'Attach sim:  tmux attach -t %s\n' "$SIM_SESSION"
  printf 'Attach ctrl: tmux attach -t %s\n' "$CTRL_SESSION"
  if [[ "$SIM2SIM_MODE" == "official_bootstrap" ]]; then
    printf 'Flow: Unitree-style sim bootstrap. Starts from deploy stand qpos with elastic band=%s and stays paused by default. If auto-run is explicitly enabled and elastic band is enabled, the helper pre-tensions it with MuJoCo key 8 before Run. This requires separate evidence before any real-robot gate can unlock.\n' "$ENABLE_ELASTIC_BAND"
  elif [[ "$SIM2SIM_MODE" == "official_velocity_bootstrap" ]]; then
    printf 'Flow: Unitree-style Velocity acceptance bootstrap. Starts in FixStand, uses a sim-only auto transition to enter Velocity from %s joint qpos with %s root qpos, keeps elastic band=%s for MuJoCo key-8 contact preparation, and judges standing only after Velocity is reached. With auto-run, policy start gate=%ss delays initial env->step; lowstate tick gate=%s can additionally require fresh DDS/MuJoCo state before each env->step when the local phase-1 patch is applied.\n' "$VELOCITY_BOOTSTRAP_POSE" "$VELOCITY_BOOTSTRAP_ROOT" "$ENABLE_ELASTIC_BAND" "${POLICY_START_GATE_SECONDS:-0}" "${POLICY_LOWSTATE_TICK_GATE:-0}"
  elif [[ "$SIM2SIM_MODE" == "velocity_bootstrap" ]]; then
    printf 'Flow: diagnostic Velocity-first bootstrap. Starts in Velocity from %s joint qpos with %s root qpos and elastic band=%s, then stays paused by default. Judge standing in Velocity, not FixStand; this still requires separate Mimic evidence before any real-robot gate can unlock. With auto-run, policy start gate=%ss delays initial env->step; lowstate tick gate=%s can additionally require fresh DDS/MuJoCo state before each env->step when the local phase-1 patch is applied.\n' "$VELOCITY_BOOTSTRAP_POSE" "$VELOCITY_BOOTSTRAP_ROOT" "$ENABLE_ELASTIC_BAND" "${POLICY_START_GATE_SECONDS:-0}" "${POLICY_LOWSTATE_TICK_GATE:-0}"
  elif [[ "$SIM2SIM_MODE" == "passive_velocity_bootstrap" ]]; then
    printf 'Flow: diagnostic Passive-to-Velocity bootstrap. Starts in Passive to let sim/controller state settle, uses a sim-only auto transition to enter Velocity from %s joint qpos with %s root qpos and elastic band=%s, then judges standing in Velocity. This still requires separate Mimic evidence before any real-robot gate can unlock. With auto-run, policy start gate=%ss delays initial env->step; lowstate tick gate=%s can additionally require fresh DDS/MuJoCo state before each env->step when the local phase-1 patch is applied.\n' "$VELOCITY_BOOTSTRAP_POSE" "$VELOCITY_BOOTSTRAP_ROOT" "$ENABLE_ELASTIC_BAND" "${POLICY_START_GATE_SECONDS:-0}" "${POLICY_LOWSTATE_TICK_GATE:-0}"
  elif [[ "$SIM2SIM_MODE" == "play_parity" ]]; then
    printf 'Flow: debug parity mode starts from the motion first frame and enters Mimic_RoundhouseLeadingRight immediately.\n'
  elif [[ "$SIM2SIM_MODE" == "prepose" ]]; then
    printf 'Flow: starts in FixStand from deploy stand qpos, interpolates to the motion frame-0 prepose, then waits for the Mimic_RoundhouseLeadingRight trigger.\n'
  else
    printf 'Flow: starts paused in FixStand from deploy default pose. Press Run only for controller debugging; current FixStand is not stable enough for deploy sim2sim.\n'
  fi
}

restore_case() {
  local case_dir="$1"
  require_file "$case_dir/active_before/config.yaml"
  require_file "$case_dir/active_before/simulate_config.yaml"
  stop_sessions
  cp "$case_dir/active_before/config.yaml" "$ACTIVE_CONFIG"
  cp "$case_dir/active_before/simulate_config.yaml" "$SIM_CONFIG"
  if [[ -f "$case_dir/active_before/policy.onnx" ]]; then cp "$case_dir/active_before/policy.onnx" "$ROUNDHOUSE_POLICY"; fi
  if [[ -f "$case_dir/active_before/roundhouse_leading_right.npz" ]]; then cp "$case_dir/active_before/roundhouse_leading_right.npz" "$ROUNDHOUSE_MOTION"; fi
  if [[ -f "$case_dir/active_before/deploy.yaml" ]]; then cp "$case_dir/active_before/deploy.yaml" "$ROUNDHOUSE_DEPLOY_YAML"; fi
  printf 'Restored active deploy config from %s\n' "$case_dir"
}

status() {
  printf 'Productized prepare command: uv run unitree-sim2sim prepare-g1 --action roundhouse_leading_right --official-root <unitree-root> --out-root <lane-root> --policy-root <policy-root>\n'
  tmux ls 2>/dev/null | rg "($SIM_SESSION|$CTRL_SESSION)" || true
  if select_policy_src && [[ -f "$ROUNDHOUSE_POLICY" && -f "$ROUNDHOUSE_MOTION" && -f "$ROUNDHOUSE_DEPLOY_YAML" ]]; then
    hash_manifest /dev/stdout
  fi
}

cmd="${1:-}"
case "$cmd" in
  prepare-lane)
    shift
    prepare_product_lane "$@"
    ;;
  start)
    start_case
    ;;
  stop)
    stop_sessions
    ;;
  restore)
    restore_case "${2:-}"
    ;;
  status)
    status
    ;;
  *)
    usage
    exit 2
    ;;
esac
