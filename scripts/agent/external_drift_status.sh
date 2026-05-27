#!/usr/bin/env bash
set -euo pipefail

official_root="${OFFICIAL_ROOT:-/tmp/unitree_rl_mjlab_official_baseline}"
external_root="${EXTERNAL_ROOT:-/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab}"

printf 'official_root: %s\n' "${official_root}"
printf 'external_root: %s\n' "${external_root}"

if [[ ! -d "${official_root}" ]]; then
  printf 'official_root_status: missing\n'
  exit 1
fi
if [[ ! -d "${external_root}" ]]; then
  printf 'external_root_status: missing\n'
  exit 1
fi

printf 'official_head: '
git -C "${official_root}" rev-parse --short HEAD

printf 'external_git_root: '
git -C "${external_root}" rev-parse --show-toplevel 2>/dev/null || true

critical_files=(
  "simulate/config.yaml"
  "simulate/src/param.h"
  "simulate/src/main.cc"
  "simulate/src/unitree_sdk2_bridge.h"
  "deploy/robots/g1/config/config.yaml"
  "deploy/include/FSM/State_FixStand.h"
  "deploy/include/FSM/State_RLBase.h"
  "deploy/include/isaaclab/envs/mdp/actions/joint_actions.h"
  "deploy/robots/g1/src/State_RLBase.cpp"
  "deploy/robots/g1/src/State_Mimic.cpp"
)

diff_count=0
for rel in "${critical_files[@]}"; do
  official_file="${official_root}/${rel}"
  external_file="${external_root}/${rel}"
  if [[ ! -e "${official_file}" || ! -e "${external_file}" ]]; then
    printf 'critical_file_missing: %s\n' "${rel}"
    diff_count=$((diff_count + 1))
  elif diff -q "${official_file}" "${external_file}" >/dev/null; then
    printf 'critical_file_same: %s\n' "${rel}"
  else
    printf 'critical_file_diff: %s\n' "${rel}"
    diff_count=$((diff_count + 1))
  fi
done

policy_files=(
  "deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx"
  "deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml"
)

for rel in "${policy_files[@]}"; do
  if [[ -e "${official_root}/${rel}" && -e "${external_root}/${rel}" ]] && \
    diff -q "${official_root}/${rel}" "${external_root}/${rel}" >/dev/null; then
    printf 'velocity_asset_same: %s\n' "${rel}"
  else
    printf 'velocity_asset_diff_or_missing: %s\n' "${rel}"
  fi
done

printf 'external_extra_mimic_policies:\n'
comm -13 \
  <(find "${official_root}/deploy/robots/g1/config/policy/mimic" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort) \
  <(find "${external_root}/deploy/robots/g1/config/policy/mimic" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' 2>/dev/null | sort) \
  | sed 's/^/  - /' || true

if [[ "${diff_count}" -eq 0 ]]; then
  printf 'external_drift_status: clean_against_critical_files\n'
else
  printf 'external_drift_status: drifted_critical_files=%s\n' "${diff_count}"
fi
