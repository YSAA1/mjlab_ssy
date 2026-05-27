#!/usr/bin/env bash
set -euo pipefail

scripts/agent/status.sh >/dev/null

grep -q 'current_phase: closed' .harness/state.md
grep -q 'active_slice: none' .harness/state.md
grep -q 'G1 sim2sim / dual-kick deployment research lane is closed' .harness/state.md
grep -q 'Keep `AGENTS.md` thin' .harness/decisions.md
grep -q 'Closed G1 Research Lane' docs/agent/harness.md
grep -q 'Status: closed as active harness work' \
  docs/research/archive/g1-sim2sim-2026-05-25.md

for script in scripts/agent/*.sh; do
  bash -n "${script}"
done

printf 'harness selftest: ok\n'
