#!/usr/bin/env bash
set -euo pipefail

ROOT="${MJLAB_ROOT:-/home/ssy/ssy_files/mjlab}"
WORKTREE="${MJLAB_WORKTREE:-/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main}"
MJLAB_VIRTUAL_ENV="${MJLAB_VIRTUAL_ENV:-/home/ssy/ssy_files/mjlab/.venv}"
UNITREE="$ROOT/.external/unitree_rl_mjlab"
DEPLOY="$UNITREE/deploy/robots/g1"
SIM="$UNITREE/simulate"

EXPERIMENT_NAME="${MJLAB_EXPERIMENT_NAME:-g1_tracking_acrobatics_no_state}"
RUN_NAME_PATTERN="${MJLAB_RUN_NAME_PATTERN:-*g1_mode15_flying_kick_4096env_5000iter*}"
SELECTED_RUN_DIR=""
POLICY_SRC="${MJLAB_POLICY_ONNX:-}"
MOTION_SRC="$WORKTREE/data/motions/g1_flying_kick/mjlab/motion.npz"
DEPLOY_YAML_SRC="$DEPLOY/config/policy/mimic/getup/params/deploy.yaml"
SIM2SIM_MODE="${MJLAB_SIM2SIM_MODE:-stand}"
ENABLE_ELASTIC_BAND="${MJLAB_ENABLE_ELASTIC_BAND:-0}"
START_PAUSED="${MJLAB_START_PAUSED:-1}"
AUTO_RUN_AFTER_READY="${MJLAB_AUTO_RUN_AFTER_READY:-0}"

ACTIVE_CONFIG="$DEPLOY/config/config.yaml"
SIM_CONFIG="$SIM/config.yaml"
FLYING_POLICY_DIR="$DEPLOY/config/policy/mimic/flying_kick"
FLYING_POLICY="$FLYING_POLICY_DIR/exported/policy.onnx"
FLYING_MOTION="$FLYING_POLICY_DIR/params/flying_kick.npz"
FLYING_DEPLOY_YAML="$FLYING_POLICY_DIR/params/deploy.yaml"
MODE15_XML="$WORKTREE/src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml"
MODE15_ASSET_DIR="$WORKTREE/src/mjlab/asset_zoo/robots/unitree_g1/xmls/assets"
SIM_XML_DIR="$UNITREE/src/assets/robots/unitree_g1/xmls"
SIM_SCENE_XML="$SIM_XML_DIR/scene_g1.xml"
SIM_G1_XML="$SIM_XML_DIR/g1.xml"
SIM_ASSET_DIR="$SIM_XML_DIR/assets"

STAND_INITIAL_QPOS='[0.0, 0.0, 0.7657805681228638, 1.0, 0.0, 0.0, 0.0, -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, -0.312, 0.0, 0.0, 0.669, -0.363, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0, 0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0]'

ARTIFACT_ROOT="$WORKTREE/logs/flying_kick_sim2sim"
SIM_SESSION="flying_kick_sim"
CTRL_SESSION="flying_kick_ctrl"

if [[ -d "$MJLAB_VIRTUAL_ENV" ]]; then
  export VIRTUAL_ENV="$MJLAB_VIRTUAL_ENV"
  export PATH="$VIRTUAL_ENV/bin:$PATH"
fi

usage() {
  cat <<'EOF'
Usage:
  scripts/tools/run_flying_kick_sim2sim.sh start
  scripts/tools/run_flying_kick_sim2sim.sh stop
  scripts/tools/run_flying_kick_sim2sim.sh restore <artifact-dir>
  scripts/tools/run_flying_kick_sim2sim.sh status

Before start, export the latest trained actor with:
  bash scripts/tools/run_g1_mode15_flying_kick_train.sh export

Environment:
  MJLAB_SIM2SIM_MODE=stand       # default: FixStand from deploy default pose
  MJLAB_SIM2SIM_MODE=play_parity # debug only: start from motion first frame
  MJLAB_ENABLE_ELASTIC_BAND=1    # optional; default is 0
  MJLAB_START_PAUSED=0           # optional; stand mode defaults to 1
  MJLAB_AUTO_RUN_AFTER_READY=1   # optional; default is 0 because stand controller is unstable
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
    stand|play_parity)
      ;;
    *)
      printf 'Unsupported MJLAB_SIM2SIM_MODE=%s. Use stand or play_parity.\n' "$SIM2SIM_MODE" >&2
      exit 2
      ;;
  esac
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

hash_manifest() {
  local out="$1"
  {
    date --iso-8601=seconds
    sha256sum "$POLICY_SRC"
    sha256sum "$MOTION_SRC"
    sha256sum "$FLYING_POLICY"
    sha256sum "$FLYING_MOTION"
    sha256sum "$FLYING_DEPLOY_YAML"
    sha256sum "$ACTIVE_CONFIG"
    sha256sum "$SIM_CONFIG"
    stat -c '%y %s %n' \
      "$POLICY_SRC" "$MOTION_SRC" "$FLYING_POLICY" "$FLYING_MOTION" \
      "$FLYING_DEPLOY_YAML" "$ACTIVE_CONFIG" "$SIM_CONFIG"
  } > "$out"
}

stop_sessions() {
  tmux has-session -t "$CTRL_SESSION" 2>/dev/null && tmux kill-session -t "$CTRL_SESSION"
  tmux has-session -t "$SIM_SESSION" 2>/dev/null && tmux kill-session -t "$SIM_SESSION"
  true
}

patch_config() {
  project_python - "$ACTIVE_CONFIG" "$SIM2SIM_MODE" "$STAND_INITIAL_QPOS" <<'PY'
from ast import literal_eval
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
mode = sys.argv[2]
stand_qpos = literal_eval(sys.argv[3])
data = yaml.safe_load(path.read_text())
fsm = data.setdefault("FSM", {})
enabled = fsm.setdefault("_", {})
enabled["Mimic_FlyingKick"] = {"id": 7, "type": "Mimic"}
fsm["initial_state"] = "Mimic_FlyingKick" if mode == "play_parity" else "FixStand"
fixstand = fsm.setdefault("FixStand", {})
transitions = fixstand.setdefault("transitions", {})
transitions["Mimic_FlyingKick"] = "RB + X.on_pressed"
if mode == "stand":
    fixstand["qs"] = [[], stand_qpos[7:]]
fsm["Mimic_FlyingKick"] = {
    "transitions": {
        "Passive": "B.on_pressed",
        "FixStand": "RB + B.on_pressed",
    },
    "motion_file": "config/policy/mimic/flying_kick/params/flying_kick.npz",
    "policy_dir": "config/policy/mimic/flying_kick/",
    "time_start": 0.0,
    "time_end": 1000.0,
    "end_state": "FixStand",
}
path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY
}

patch_sim_config() {
  project_python - "$SIM_CONFIG" "$SIM2SIM_MODE" "$STAND_INITIAL_QPOS" "$MOTION_SRC" "$ENABLE_ELASTIC_BAND" "$START_PAUSED" <<'PY'
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

if mode == "play_parity":
    motion = np.load(motion_path)
    root_pos = motion["body_pos_w"][0, 0].astype(float).tolist()
    root_quat = motion["body_quat_w"][0, 0].astype(float).tolist()
    joint_pos = motion["joint_pos"][0].astype(float).tolist()
    qpos = [0.0, 0.0, root_pos[2], *root_quat, *joint_pos]
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
  local window_id=""
  for _ in $(seq 1 120); do
    if rg -q "FSM: Start FixStand" "$case_dir/g1_ctrl.log" 2>/dev/null; then
      window_id="$(xdotool search --name 'MuJoCo : scene_g1' 2>/dev/null | tail -n 1 || true)"
      if [[ -n "$window_id" ]]; then
        xdotool windowfocus "$window_id" key space || true
        printf 'Auto-started paused MuJoCo after FixStand was ready.\n'
        return
      fi
    fi
    sleep 0.1
  done
  printf 'Warning: MuJoCo stayed paused; press Run in the MuJoCo window after FixStand is ready.\n' >&2
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

start_case() {
  validate_mode
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
  require_file "$SIM_CONFIG"
  sync_mode15_sim_model
  require_file "$SIM/build/unitree_mujoco"
  require_file "$DEPLOY/build/g1_ctrl"

  local stamp case_dir
  stamp="$(date '+%Y%m%d-%H%M%S')"
  case_dir="$ARTIFACT_ROOT/$stamp"
  mkdir -p "$case_dir/active_before" "$case_dir/selected" \
    "$FLYING_POLICY_DIR/exported" "$FLYING_POLICY_DIR/params"

  cp "$ACTIVE_CONFIG" "$case_dir/active_before/config.yaml"
  cp "$SIM_CONFIG" "$case_dir/active_before/simulate_config.yaml"
  if [[ -f "$FLYING_POLICY" ]]; then cp "$FLYING_POLICY" "$case_dir/active_before/policy.onnx"; fi
  if [[ -f "$FLYING_MOTION" ]]; then cp "$FLYING_MOTION" "$case_dir/active_before/flying_kick.npz"; fi
  if [[ -f "$FLYING_DEPLOY_YAML" ]]; then cp "$FLYING_DEPLOY_YAML" "$case_dir/active_before/deploy.yaml"; fi

  chmod u+w "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" 2>/dev/null || true
  cp "$POLICY_SRC" "$FLYING_POLICY"
  cp "$MOTION_SRC" "$FLYING_MOTION"
  cp "$DEPLOY_YAML_SRC" "$FLYING_DEPLOY_YAML"
  chmod u+w "$FLYING_POLICY" "$FLYING_MOTION" "$FLYING_DEPLOY_YAML" 2>/dev/null || true
  cp "$POLICY_SRC" "$case_dir/selected/policy.onnx"
  cp "$MOTION_SRC" "$case_dir/selected/flying_kick.npz"
  cp "$DEPLOY_YAML_SRC" "$case_dir/selected/deploy.yaml"
  patch_config
  patch_sim_config
  cp "$ACTIVE_CONFIG" "$case_dir/selected/config.yaml"
  cp "$SIM_CONFIG" "$case_dir/selected/simulate_config.yaml"
  hash_manifest "$case_dir/hash_selected.txt"

  stop_sessions
  tmux new-session -d -s "$CTRL_SESSION" -c "$DEPLOY/build" \
    "set -o pipefail; ./g1_ctrl --network=lo 2>&1 | tee '$case_dir/g1_ctrl.log'"
  sleep 0.5
  tmux new-session -d -s "$SIM_SESSION" -c "$UNITREE" \
    "set -o pipefail; ./simulate/build/unitree_mujoco 2>&1 | tee '$case_dir/unitree_mujoco.log'"
  unpause_when_ready "$case_dir"

  printf 'Started flying-kick sim2sim\n'
  printf 'Selected policy: %s\n' "$POLICY_SRC"
  printf 'Mode: %s; elastic band: %s; start paused: %s; auto run: %s\n' "$SIM2SIM_MODE" "$ENABLE_ELASTIC_BAND" "$START_PAUSED" "$AUTO_RUN_AFTER_READY"
  printf 'Artifact dir: %s\n' "$case_dir"
  printf 'Attach sim:  tmux attach -t %s\n' "$SIM_SESSION"
  printf 'Attach ctrl: tmux attach -t %s\n' "$CTRL_SESSION"
  if [[ "$SIM2SIM_MODE" == "play_parity" ]]; then
    printf 'Flow: debug parity mode starts from the motion first frame and enters Mimic_FlyingKick immediately.\n'
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
  if [[ -f "$case_dir/active_before/policy.onnx" ]]; then cp "$case_dir/active_before/policy.onnx" "$FLYING_POLICY"; fi
  if [[ -f "$case_dir/active_before/flying_kick.npz" ]]; then cp "$case_dir/active_before/flying_kick.npz" "$FLYING_MOTION"; fi
  if [[ -f "$case_dir/active_before/deploy.yaml" ]]; then cp "$case_dir/active_before/deploy.yaml" "$FLYING_DEPLOY_YAML"; fi
  printf 'Restored active deploy config from %s\n' "$case_dir"
}

status() {
  tmux ls 2>/dev/null | rg "($SIM_SESSION|$CTRL_SESSION)" || true
  if select_policy_src && [[ -f "$FLYING_POLICY" && -f "$FLYING_MOTION" && -f "$FLYING_DEPLOY_YAML" ]]; then
    hash_manifest /dev/stdout
  fi
}

cmd="${1:-}"
case "$cmd" in
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
