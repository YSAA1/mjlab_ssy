#!/usr/bin/env bash
set -euo pipefail

printf '== harness status ==\n'
scripts/agent/status.sh

printf '\n== research status ==\n'
scripts/agent/research_status.sh

printf '\n== environment status ==\n'
set +e
scripts/agent/env_status.sh
env_status=$?
set -e
printf 'env_status_exit: %s\n' "${env_status}"

printf '\n== git status ==\n'
git status --short --branch

printf '\n== protected runtime roots ==\n'
printf 'logs/ wandb/ artifacts/ .external/ .venv/ .worktrees/ problem_result/\n'

printf '\n== report result ==\n'
if [[ "${env_status}" -eq 0 ]]; then
  printf 'harness report: ok\n'
else
  printf 'harness report: static ok; environment probe failed or is sandbox-blocked\n'
fi
