#!/usr/bin/env bash
set -euo pipefail

stamp="$(date +%Y%m%d-%H%M%S)"
logdir="logs/g1_standard_tracking_resume2000/${stamp}"
mkdir -p "${logdir}"

export UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT:-/home/ssy/ssy_files/mjlab/.venv}"

task_id="Mjlab-Tracking-Flat-Unitree-G1-No-State-Estimation"
experiment_name="g1_tracking_standard_no_state_resume2000"
num_envs=4096
iters=2000
save_interval=500

flying_checkpoint="logs/rsl_rl/g1_tracking_acrobatics_no_state/2026-05-20_10-20-41_g1_mode15_flying_kick_4096env_5000iter_20260520_102036/model_4999.pt"
roundhouse_checkpoint="logs/rsl_rl/g1_tracking_roundhouse_leading_right_no_state/2026-05-21_18-41-05_g1_mode15_roundhouse_leading_right_apexsmooth_resume4999_2000iter_20260521_184100/model_6998.pt"

{
  echo "stamp=${stamp}"
  echo "task_id=${task_id}"
  echo "experiment_name=${experiment_name}"
  echo "num_envs=${num_envs}"
  echo "iters=${iters}"
  echo "flying_checkpoint=${flying_checkpoint}"
  echo "roundhouse_checkpoint=${roundhouse_checkpoint}"
} >"${logdir}/manifest.txt"

run_train() {
  local action_name="$1"
  local motion_file="$2"
  local checkpoint_file="$3"
  local run_name="$4"
  local train_log="${logdir}/${action_name}.log"

  echo "[${action_name}] start $(date -Is)" | tee -a "${logdir}/queue.log"
  UV_PROJECT_ENVIRONMENT="${UV_PROJECT_ENVIRONMENT}" uv run --no-sync train "${task_id}" \
    --env.scene.num-envs "${num_envs}" \
    --env.commands.motion.motion-file "${motion_file}" \
    --agent.resume True \
    --checkpoint-file "${checkpoint_file}" \
    --agent.max-iterations "${iters}" \
    --agent.save-interval "${save_interval}" \
    --agent.experiment-name "${experiment_name}" \
    --agent.run-name "${run_name}" \
    --agent.logger tensorboard \
    --agent.upload-model False \
    >"${train_log}" 2>&1
  echo "[${action_name}] done $(date -Is)" | tee -a "${logdir}/queue.log"
}

run_train \
  "flying_kick" \
  "data/motions/g1_flying_kick/mjlab/motion.npz" \
  "${flying_checkpoint}" \
  "g1_mode15_flying_kick_standard_tracking_resume4999_2000iter_${stamp}"

run_train \
  "roundhouse_leading_right" \
  "data/motions/g1_roundhouse_leading_right/mjlab/motion.npz" \
  "${roundhouse_checkpoint}" \
  "g1_mode15_roundhouse_leading_right_standard_tracking_resume6998_2000iter_${stamp}"

echo "all done $(date -Is)" | tee -a "${logdir}/queue.log"
