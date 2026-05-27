# mjlab Agent Harness State

Last updated: 2026-05-25

current_phase: closed
active_slice: none

## How To Resume

Read this file first. Then read `docs/agent/harness.md` for the stable project
harness contract.

The G1 sim2sim / dual-kick deployment research lane is closed and archived at
`docs/research/archive/g1-sim2sim-2026-05-25.md`. Do not resume it as the
active task unless the user explicitly reopens that lane.

Do not read the full historical plan by default. Use
`docs/plans/2026-05-22--g1-tracking-sim2real-hardening.md` only for targeted
lookup of a specific evidence path, work item, or commit.

## Current Objective

Keep a small project-local harness that tells future agents:

- where project rules and recovery state live;
- which commands are safe fast checks;
- which local roots are runtime evidence and should not be committed or
  bulk-deleted;
- where completed G1 research evidence was archived.

## Source Of Truth Priority

1. User instruction in the current conversation.
2. `AGENTS.md` for stable agent rules.
3. This file for current harness state.
4. `docs/agent/harness.md` for project harness details.
5. Archived G1 research docs and historical plans for past evidence only.
6. Raw logs, videos, checkpoints, and `.external` runtime state as local
   evidence, not source files.

## Success Criteria

- A fresh agent can run `scripts/agent/status.sh` and find the recovery entry.
- Active task state is not stored in `AGENTS.md`.
- Completed G1 research state is archived and no longer controls the harness.
- Fast checks do not install dependencies or start long training/deploy runs.
- Runtime evidence roots stay out of version control unless explicitly
  requested.

## Verification Commands

Static harness status:

```sh
scripts/agent/status.sh
```

Fast source/environment check:

```sh
scripts/agent/check.sh
```

Environment probe only:

```sh
scripts/agent/env_status.sh
```

One-command harness report:

```sh
scripts/agent/harness_report.sh
```

Broader project gates before commit:

```sh
make format
make type
make test
```

Use targeted `uv run pytest tests/<file>.py` while iterating.

## Protected Local Runtime Roots

Do not bulk-delete or commit these without explicit path-level approval:

- `logs/`
- `wandb/`
- `artifacts/`
- `.external/`
- `.venv/`
- `.worktrees/`
- `problem_result/`

Generated caches such as `__pycache__/`, `.pytest_cache/`, and `.ruff_cache/`
are safe cleanup candidates when they are not tracked.

## Current Blockers

No active harness blocker.

Known residual G1 research risks are archived, not active blockers:

- clean official Unitree baseline remained blocked by missing joystick/input
  capability during the prior lane;
- real robot deployment requires explicit user/hardware confirmation and is
  not a default harness check.

## Next Actions

For a new task:

1. Read `AGENTS.md`.
2. Read this file.
3. Run `scripts/agent/status.sh`.
4. Inspect `git status --short --branch`.
5. Define the new task's objective, success criteria, and verification path
   before editing non-trivial code.

For G1 research follow-up, start from
`docs/research/archive/g1-sim2sim-2026-05-25.md` and treat it as a new active
slice only after the user asks to reopen it.
