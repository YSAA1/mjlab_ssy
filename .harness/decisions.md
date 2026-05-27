# Harness Decisions

Last updated: 2026-05-25

## D1: Keep `AGENTS.md` thin

`AGENTS.md` is for stable project rules, project map, verification commands,
and recovery pointers. It must not contain active task status, one-off research
conclusions, or temporary TODOs.

Current harness state belongs in `.harness/state.md`. Completed research
evidence belongs in `docs/research/archive/`.

## D2: G1 sim2sim research is archived, not active

The G1 clean-official-baseline / automation-deviation lane produced useful
evidence, but the user closed the current task batch on 2026-05-25. Its compact
archive is `docs/research/archive/g1-sim2sim-2026-05-25.md`.

Future G1 work must be reopened as a new active slice instead of silently
continuing from old active research docs.

## D3: Runtime evidence is protected

`logs/`, `wandb/`, `artifacts/`, `.external/`, `.venv/`, `.worktrees/`, and
`problem_result/` may contain evidence, checkpoints, deploy assets, or local
tool state. Do not bulk-delete or commit them without explicit path-level
approval.

## D4: Fast harness checks must stay local and bounded

`scripts/agent/status.sh` and `scripts/agent/check.sh` must not install
dependencies, start training, launch simulators, start real robot controllers,
or write runtime assets.

## D5: Historical plans are lookup surfaces

`docs/plans/2026-05-22--g1-tracking-sim2real-hardening.md` is retained only for
targeted lookup. It is not the active scheduler for this project harness.
