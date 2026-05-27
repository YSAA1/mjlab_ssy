#!/usr/bin/env bash
set -euo pipefail

required_files=(
  "AGENTS.md"
  ".harness/state.md"
  ".harness/manifest.yaml"
  ".harness/decisions.md"
  "docs/agent/harness.md"
  "docs/research/archive/g1-sim2sim-2026-05-25.md"
  "docs/research/archive/g1-dual-kicks-real-deploy.md"
  "scripts/agent/check.sh"
  "scripts/agent/env_status.sh"
  "scripts/agent/harness_report.sh"
  "scripts/agent/research_status.sh"
  "scripts/agent/selftest.sh"
)

for path in "${required_files[@]}"; do
  if [[ ! -f "${path}" ]]; then
    printf 'missing required harness file: %s\n' "${path}" >&2
    exit 1
  fi
done

executable_scripts=(
  "scripts/agent/status.sh"
  "scripts/agent/check.sh"
  "scripts/agent/env_status.sh"
  "scripts/agent/harness_report.sh"
  "scripts/agent/research_status.sh"
  "scripts/agent/selftest.sh"
)

for path in "${executable_scripts[@]}"; do
  if [[ ! -x "${path}" ]]; then
    printf 'harness script is not executable: %s\n' "${path}" >&2
    exit 1
  fi
done

legacy_active_docs=(
  "docs/research/current_task_summary.md"
  "docs/research/research_plan.md"
  "docs/research/evidence_log.md"
  "docs/research/iteration_protocol.md"
)

for path in "${legacy_active_docs[@]}"; do
  if [[ -e "${path}" ]]; then
    printf 'legacy active research doc should be archived, not live: %s\n' "${path}" >&2
    exit 1
  fi
done

if [[ -e ".harness/research_manifest.yaml" ]]; then
  printf '.harness/research_manifest.yaml is task-specific and should remain archived/removed\n' >&2
  exit 1
fi

if ! grep -q '\.harness/state\.md' AGENTS.md; then
  printf 'AGENTS.md does not point to .harness/state.md\n' >&2
  exit 1
fi

if ! grep -q 'docs/agent/harness.md' AGENTS.md; then
  printf 'AGENTS.md does not point to docs/agent/harness.md\n' >&2
  exit 1
fi

if grep -q 'current_task_summary\|research_plan\|evidence_log' AGENTS.md; then
  printf 'AGENTS.md still references old active research docs\n' >&2
  exit 1
fi

if ! grep -q 'active_slice: none' .harness/state.md; then
  printf '.harness/state.md must declare no active slice after cleanup\n' >&2
  exit 1
fi

if ! grep -q 'docs/research/archive/g1-sim2sim-2026-05-25.md' .harness/manifest.yaml; then
  printf '.harness/manifest.yaml must point to the G1 archive\n' >&2
  exit 1
fi

if ! grep -q 'Runtime Evidence Policy' docs/agent/harness.md; then
  printf 'docs/agent/harness.md must describe runtime evidence policy\n' >&2
  exit 1
fi

if [[ -d scripts/agent/__pycache__ ]]; then
  printf 'remove generated scripts/agent/__pycache__ before handoff\n' >&2
  exit 1
fi

printf 'harness status: ok\n'
printf 'resume: .harness/state.md\n'
printf 'harness_doc: docs/agent/harness.md\n'
printf 'archive: docs/research/archive/g1-sim2sim-2026-05-25.md\n'
