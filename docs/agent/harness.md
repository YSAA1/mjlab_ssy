# mjlab Agent Harness

Last updated: 2026-05-25

This file is the stable project harness contract for agents working in this
checkout.

## Entry Points

- Project rules: `AGENTS.md`
- Current harness state: `.harness/state.md`
- Harness decisions: `.harness/decisions.md`
- Harness manifest: `.harness/manifest.yaml`
- Completed G1 research archive:
  `docs/research/archive/g1-sim2sim-2026-05-25.md`

Do not put active task state in `AGENTS.md`. Use `.harness/state.md` for the
current harness state and create task-specific plans only when a new task needs
durable recovery.

## Fast Checks

Run this first when checking the harness:

```sh
scripts/agent/status.sh
```

Run this before claiming harness changes are ready:

```sh
scripts/agent/check.sh
```

The fast check is intentionally bounded. It should not install dependencies,
start training, launch MuJoCo viewers, write deploy assets, or start robot
controllers.

Only scripts named in `.harness/manifest.yaml` are current harness gates.
Additional G1-specific scripts under `scripts/agent/` are historical task
utilities unless the user explicitly reopens that lane.

Before committing code changes, follow `AGENTS.md`:

```sh
make format
make type
make test
```

For narrow iterations, prefer targeted `uv run pytest tests/<file>.py`.

## Runtime Evidence Policy

These roots are local runtime evidence or tool state:

- `logs/`
- `wandb/`
- `artifacts/`
- `.external/`
- `.venv/`
- `.worktrees/`
- `problem_result/`

Do not bulk-delete or commit them without explicit path-level approval.
Generated caches such as `__pycache__/`, `.pytest_cache/`, and `.ruff_cache/`
can be removed when they are untracked.

## Closed G1 Research Lane

The G1 sim2sim / dual-kick deployment work completed its current task batch and
is no longer the active harness state. The compact archive is:

```text
docs/research/archive/g1-sim2sim-2026-05-25.md
```

The old long plan remains a historical lookup surface:

```text
docs/plans/2026-05-22--g1-tracking-sim2real-hardening.md
```

Use targeted `rg` against the long plan only when a specific evidence path,
work item, or commit needs tracing.

## Reopening Research

If the user reopens G1 sim2sim or real deploy work:

1. Treat it as a new active slice.
2. State objective, non-goals, success criteria, and verification path.
3. Read the archive before reading the long historical plan.
4. Preserve raw logs and deploy assets unless the user approves exact cleanup
   paths.
5. Record new evidence in a new task-specific plan or recovery file, not in
   `AGENTS.md`.
