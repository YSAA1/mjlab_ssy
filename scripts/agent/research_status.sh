#!/usr/bin/env bash
set -euo pipefail

scripts/agent/status.sh >/dev/null

printf 'research: none active\n'
printf 'state: .harness/state.md\n'
printf 'harness_doc: docs/agent/harness.md\n'
printf 'closed_g1_archive: docs/research/archive/g1-sim2sim-2026-05-25.md\n'
printf 'closed_g1_real_deploy_runbook: docs/research/archive/g1-dual-kicks-real-deploy.md\n'
printf 'historical_plan: docs/plans/2026-05-22--g1-tracking-sim2real-hardening.md\n'
printf '\n'
printf 'reopen_rule:\n'
printf '  start a new active slice with objective, non-goals, success criteria, and verification path\n'
printf '\n'
printf 'do_not:\n'
printf '  - put active task status in AGENTS.md\n'
printf '  - bulk-delete logs/wandb/artifacts/.external without path-level approval\n'
printf '  - treat archived G1 joystick or deploy blockers as current harness blockers\n'
