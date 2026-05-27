#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${MJLAB_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
WORKTREE="${MJLAB_WORKTREE:-$ROOT}"
PHASE1_ROOT="${MJLAB_PHASE1_ROOT:-$WORKTREE/logs/g1_tracking_phase1}"
PYTHON_RUNNER="${MJLAB_PHASE1_PYTHON:-uv run --active --no-sync python}"
CONTRACT_CMD="${MJLAB_PHASE1_CONTRACT_CMD:-}"

FLYING_SCRIPT="$WORKTREE/scripts/tools/run_flying_kick_sim2sim.sh"
ROUNDHOUSE_SCRIPT="$WORKTREE/scripts/tools/run_roundhouse_leading_right_sim2sim.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh prepare-lane --action <flying_kick|roundhouse_leading_right> --official-root <unitree-root> --out-root <lane-root> --policy-root <policy-root> [unitree-sim2sim args...]
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight [--manifest <manifest.json>]
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh entry-gate [--manifest <manifest.json>] [--report-out <report.json>] [--expect-blocked]
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action <flying_kick|roundhouse_leading_right> [--manifest <manifest.json>] [--mode <stand|play_parity|prepose|official_bootstrap|official_velocity_bootstrap|velocity_bootstrap|passive_velocity_bootstrap>] [--start-paused <0|1>]
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh restore --action <flying_kick|roundhouse_leading_right> <artifact-dir>
  scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh status

This wrapper gates both actions with the phase-1 manifest and new-G1 contract
validator before delegating to the existing single-action sim2sim scripts.
For reproducible lane preparation, this compatibility wrapper delegates to:
  uv run unitree-sim2sim prepare-g1 --official-root <unitree-root> --out-root <lane-root> --action <action> --policy-root <policy-root>
Preflight is non-launching: it does not start MuJoCo, g1_ctrl, DDS, or hardware.
For deterministic direct-Mimic evidence collection, use:
  --mode play_parity --start-paused 0
The entry-gate command classifies sim initial_qpos as diagnostic-only unless it
can be reproduced by a controller entry path without teleporting state.
For Velocity bootstrap diagnosis, set MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default
to initialize from the resolved Velocity policy default joint pose.
Set MJLAB_VELOCITY_BOOTSTRAP_ROOT=home with policy_default to test the Velocity
policy default pose at the current G1 HOME_KEYFRAME root height.
EOF
}

die() {
  printf '%s\n' "$*" >&2
  exit 2
}

require_file() {
  local path="$1"
  [[ -f "$path" ]] || die "Missing required file: $path"
}

project_python() {
  (
    cd "$WORKTREE"
    # shellcheck disable=SC2086
    $PYTHON_RUNNER "$@"
  )
}

latest_manifest() {
  if [[ ! -d "$PHASE1_ROOT" ]]; then
    return 1
  fi
  find "$PHASE1_ROOT" -mindepth 2 -maxdepth 2 -type f -name manifest.json \
    -printf '%T@ %p\n' | sort -nr | head -n 1 | cut -d' ' -f2-
}

parse_manifest_arg() {
  local manifest=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --manifest)
        shift
        manifest="${1:-}"
        ;;
      *)
        die "Unsupported argument: $1"
        ;;
    esac
    shift || true
  done
  if [[ -z "$manifest" ]]; then
    manifest="$(latest_manifest || true)"
  fi
  [[ -n "$manifest" ]] || die "No phase-1 manifest found under $PHASE1_ROOT"
  printf '%s\n' "$manifest"
}

parse_action_arg() {
  local action=""
  local manifest_args=()
  MODE_OVERRIDE=""
  START_PAUSED_OVERRIDE=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --action)
        shift
        action="${1:-}"
        ;;
      --mode)
        shift
        MODE_OVERRIDE="${1:-}"
        ;;
      --start-paused)
        shift
        START_PAUSED_OVERRIDE="${1:-}"
        ;;
      --manifest)
        manifest_args+=("$1" "${2:-}")
        shift
        ;;
      *)
        if [[ -z "${RESTORE_DIR:-}" ]]; then
          RESTORE_DIR="$1"
        else
          die "Unsupported argument: $1"
        fi
        ;;
    esac
    shift || true
  done
  case "$action" in
    flying_kick|roundhouse_leading_right)
      ;;
    *)
      die "Use --action flying_kick or --action roundhouse_leading_right"
      ;;
  esac
  case "$MODE_OVERRIDE" in
    ""|stand|play_parity|prepose)
      ;;
    official_bootstrap|official_velocity_bootstrap|velocity_bootstrap|passive_velocity_bootstrap)
      ;;
    *)
      die "Use --mode stand, play_parity, prepose, official_bootstrap, official_velocity_bootstrap, velocity_bootstrap, or passive_velocity_bootstrap"
      ;;
  esac
  case "$START_PAUSED_OVERRIDE" in
    ""|0|1)
      ;;
    *)
      die "Use --start-paused 0 or 1"
      ;;
  esac
  ACTION="$action"
  MANIFEST_ARGS=("${manifest_args[@]}")
}

parse_entry_gate_arg() {
  local manifest_args=()
  ENTRY_GATE_REPORT_OUT=""
  ENTRY_GATE_EXPECT_BLOCKED=0
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --manifest)
        manifest_args+=("$1" "${2:-}")
        shift
        ;;
      --report-out)
        shift
        ENTRY_GATE_REPORT_OUT="${1:-}"
        ;;
      --expect-blocked)
        ENTRY_GATE_EXPECT_BLOCKED=1
        ;;
      *)
        die "Unsupported argument: $1"
        ;;
    esac
    shift || true
  done
  MANIFEST_ARGS=("${manifest_args[@]}")
}

prepare_product_lane() {
  local action=""
  local passthrough=()
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --action)
        shift
        action="${1:-}"
        ;;
      *)
        passthrough+=("$1")
        ;;
    esac
    shift || true
  done
  case "$action" in
    flying_kick|roundhouse_leading_right)
      ;;
    *)
      die "Use --action flying_kick or --action roundhouse_leading_right"
      ;;
  esac
  (
    cd "$ROOT"
    uv run unitree-sim2sim prepare-g1 --action "$action" "${passthrough[@]}"
  )
}

run_contract() {
  local manifest="$1"
  local report_out="$2"
  if [[ -n "$CONTRACT_CMD" ]]; then
    "$CONTRACT_CMD" --manifest "$manifest" --forbid-g1-23dof --report-out "$report_out"
  else
    project_python scripts/tools/g1_tracking_phase1_contract.py \
      --manifest "$manifest" \
      --forbid-g1-23dof \
      --report-out "$report_out"
  fi
}

preflight() {
  local manifest="$1"
  require_file "$manifest"
  require_file "$FLYING_SCRIPT"
  require_file "$ROUNDHOUSE_SCRIPT"

  local evidence_dir contract_report preflight_report
  evidence_dir="$(dirname "$manifest")"
  contract_report="$evidence_dir/contract_report.json"
  preflight_report="$evidence_dir/sim2sim2_preflight.json"
  if ! run_contract "$manifest" "$contract_report" >"$evidence_dir/contract_stdout.log" 2>"$evidence_dir/contract_stderr.log"; then
    cat "$evidence_dir/contract_stderr.log" >&2
    cat "$evidence_dir/contract_stdout.log" >&2
    die "Contract validation failed; see $contract_report"
  fi

  project_python - "$manifest" "$preflight_report" <<'PY'
from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
out = Path(sys.argv[2])
sim_config_path = Path(manifest["deploy_configs"]["sim_config"]["path"])
sim_config = yaml.safe_load(sim_config_path.read_text(encoding="utf-8"))
failures = []
if sim_config.get("interface") != "lo":
  failures.append(f"simulate config interface={sim_config.get('interface')!r}; expected 'lo'")
if sim_config.get("robot_scene") != "src/assets/robots/unitree_g1/xmls/scene_g1.xml":
  failures.append(
    f"simulate config robot_scene={sim_config.get('robot_scene')!r}; expected G1 scene_g1.xml"
  )
if sim_config.get("robot") != "g1":
  failures.append(f"simulate config robot={sim_config.get('robot')!r}; expected 'g1'")
if int(sim_config.get("use_joystick", 0)) != 0:
  failures.append("simulate config use_joystick must be 0 for phase-1 sim2sim2")
report = {
  "passed": not failures,
  "failures": failures,
  "manifest": str(Path(sys.argv[1]).resolve()),
  "sim_config": str(sim_config_path.resolve()),
  "interface": sim_config.get("interface"),
  "domain_id": sim_config.get("domain_id"),
  "robot_scene": sim_config.get("robot_scene"),
  "robot": sim_config.get("robot"),
  "use_joystick": sim_config.get("use_joystick"),
}
out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
if failures:
  print(json.dumps(report, indent=2, sort_keys=True), file=sys.stderr)
  raise SystemExit(2)
print(json.dumps(report, indent=2, sort_keys=True))
PY

  printf 'Phase-1 sim2sim2 preflight passed\n'
  printf 'Manifest: %s\n' "$manifest"
  printf 'Contract report: %s\n' "$contract_report"
  printf 'Preflight report: %s\n' "$preflight_report"
}

entry_gate() {
  MANIFEST_ARGS=()
  parse_entry_gate_arg "$@"
  local manifest report_out args
  manifest="$(parse_manifest_arg "${MANIFEST_ARGS[@]}")"
  report_out="$ENTRY_GATE_REPORT_OUT"
  if [[ -z "$report_out" ]]; then
    report_out="$(dirname "$manifest")/entry_handoff_gate.json"
  fi
  args=(
    scripts/tools/g1_tracking_phase1_handoff_gate.py
    --manifest "$manifest"
    --report-out "$report_out"
  )
  if [[ "$ENTRY_GATE_EXPECT_BLOCKED" == "1" ]]; then
    args+=(--expect-blocked)
  fi
  project_python "${args[@]}"
}

start_case() {
  RESTORE_DIR=""
  MANIFEST_ARGS=()
  ACTION=""
  parse_action_arg "$@"
  local manifest
  manifest="$(parse_manifest_arg "${MANIFEST_ARGS[@]}")"
  preflight "$manifest"
  if [[ -n "$MODE_OVERRIDE" ]]; then
    export MJLAB_SIM2SIM_MODE="$MODE_OVERRIDE"
  fi
  if [[ -n "$START_PAUSED_OVERRIDE" ]]; then
    export MJLAB_START_PAUSED="$START_PAUSED_OVERRIDE"
  fi
  case "$ACTION" in
    flying_kick)
      bash "$FLYING_SCRIPT" start
      ;;
    roundhouse_leading_right)
      bash "$ROUNDHOUSE_SCRIPT" start
      ;;
  esac
}

stop_all() {
  bash "$FLYING_SCRIPT" stop 2>/dev/null || true
  bash "$ROUNDHOUSE_SCRIPT" stop 2>/dev/null || true
}

restore_case() {
  RESTORE_DIR=""
  MANIFEST_ARGS=()
  ACTION=""
  parse_action_arg "$@"
  [[ -n "$RESTORE_DIR" ]] || die "Missing artifact-dir for restore"
  case "$ACTION" in
    flying_kick)
      bash "$FLYING_SCRIPT" restore "$RESTORE_DIR"
      ;;
    roundhouse_leading_right)
      bash "$ROUNDHOUSE_SCRIPT" restore "$RESTORE_DIR"
      ;;
  esac
}

status() {
  printf 'Phase-1 root: %s\n' "$PHASE1_ROOT"
  printf 'Productized prepare command: uv run unitree-sim2sim prepare-g1 --official-root <unitree-root> --out-root <lane-root> --action <action> --policy-root <policy-root>\n'
  local manifest
  manifest="$(latest_manifest || true)"
  if [[ -n "$manifest" ]]; then
    printf 'Latest manifest: %s\n' "$manifest"
  else
    printf 'Latest manifest: none\n'
  fi
  printf 'Single-action scripts:\n'
  [[ -f "$FLYING_SCRIPT" ]] && printf '  flying_kick: %s\n' "$FLYING_SCRIPT" || printf '  flying_kick: missing\n'
  [[ -f "$ROUNDHOUSE_SCRIPT" ]] && printf '  roundhouse_leading_right: %s\n' "$ROUNDHOUSE_SCRIPT" || printf '  roundhouse_leading_right: missing\n'
  tmux ls 2>/dev/null | rg "(flying_kick|roundhouse_leading_right)" || true
}

cmd="${1:-}"
shift || true
case "$cmd" in
  prepare-lane)
    prepare_product_lane "$@"
    ;;
  preflight)
    manifest="$(parse_manifest_arg "$@")"
    preflight "$manifest"
    ;;
  entry-gate)
    entry_gate "$@"
    ;;
  start)
    start_case "$@"
    ;;
  stop)
    stop_all
    ;;
  restore)
    restore_case "$@"
    ;;
  status)
    status
    ;;
  *)
    usage
    exit 2
    ;;
esac
