#!/usr/bin/env bash
set -euo pipefail

root="${1:-/tmp/g1_official_automation_deviation}"
manifest="${root}/AUTOMATION_DEVIATION_MANIFEST.json"
config="${root}/simulate/config.yaml"

printf 'deviation_root: %s\n' "${root}"
if [[ ! -d "${root}" ]]; then
  printf 'deviation_status: missing\n'
  printf 'prepare: scripts/agent/prepare_official_deviation_lane.sh\n'
  exit 1
fi

if [[ -f "${manifest}" ]]; then
  printf 'manifest: %s\n' "${manifest}"
else
  printf 'manifest: missing\n'
  exit 1
fi

if grep -q '"lane": "official_source_plus_automation_deviation"' "${manifest}" && \
  grep -q '"claim": "not_clean_official_baseline"' "${manifest}"; then
  printf 'manifest_claim: explicit_deviation_not_clean\n'
else
  printf 'manifest_claim: invalid\n'
  exit 1
fi

if grep -q '^use_joystick: 0' "${config}"; then
  printf 'use_joystick: 0\n'
else
  printf 'use_joystick: not_disabled\n'
  exit 1
fi

printf 'unitree_mujoco: '
if [[ -x "${root}/simulate/build/unitree_mujoco" ]]; then
  printf 'executable\n'
  sim_ready=1
else
  printf 'missing_or_not_executable\n'
  sim_ready=0
fi

printf 'g1_ctrl: '
if [[ -x "${root}/deploy/robots/g1/build/g1_ctrl" ]]; then
  printf 'executable\n'
  ctrl_ready=1
else
  printf 'missing_or_not_executable\n'
  ctrl_ready=0
fi

if [[ "${sim_ready}" == "1" && "${ctrl_ready}" == "1" ]]; then
  printf 'deviation_status: ready_for_automation_exploration_not_clean_baseline\n'
else
  printf 'deviation_status: needs_build_not_clean_baseline\n'
  exit 1
fi
