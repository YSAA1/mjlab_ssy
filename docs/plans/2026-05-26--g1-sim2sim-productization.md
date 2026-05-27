# G1 Sim2sim Productization Plan

Created: 2026-05-26

Status: completed

## Spec Source

User request: promote the useful G1 sim2sim work into a durable `mjlab`
feature instead of leaving it as local `.external` experimentation.

Live source checks used for this plan:

- `git status --short --branch`: local `main` is ahead of `fork/main` by one
  squash commit.
- `docs/research/archive/g1-sim2sim-2026-05-25.md`: prior lane is closed, and
  clean official baseline remained blocked by missing joystick/input capability.
- `scripts/agent/prepare_official_deviation_lane.py`: current automation lane
  is useful, but lives as an agent script with local defaults and deviation
  patching mixed together.
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`: current wrapper still
  assumes the deleted worktree path unless `MJLAB_WORKTREE` is overridden.
- `.external/unitree_rl_mjlab`: contains local C++/YAML/runtime diffs that are
  not source-controlled by `mjlab`.

## Confidence Loop

Target: make G1/Unitree sim2sim a repo-owned, reproducible feature with clear
evidence labels and no dependency on mutating `.external` in place.

Assumptions:

- Evidence-backed: `.external` is protected runtime state, not source.
- Evidence-backed: current tracked code has wrapper/tests and archive evidence,
  but not a stable product API or console command for sim2sim lane creation.
- Evidence-backed: prior stable videos/logs were produced from
  `official_source_plus_automation_deviation`, not from a clean official
  baseline.
- Inferred: the durable feature should generate a clean output lane from a
  clean official Unitree checkout, not vendor or rewrite Unitree source inside
  `src/mjlab`.

Loopholes and fixes:

- Loophole: productizing all `.external` diffs would preserve diagnostic noise.
  Fix: classify every delta as `core`, `diagnostic`, `runtime artifact`, or
  `reject` before implementation.
- Loophole: runtime smoke may remain unavailable on a headless/no-joystick host.
  Fix: make library/golden tests the primary runnable path, and keep GUI or
  controller smoke as a separate final runtime gate.
- Loophole: shell wrappers can keep stale absolute paths alive.
  Fix: move path resolution into `mjlab` Python code and keep shell wrappers as
  thin compatibility shims only.
- Loophole: evidence could be mislabeled as clean official sim2sim.
  Fix: require generated manifests to name the source checkout, git SHA,
  changed paths, deviation class, and runtime capabilities used.

Revised plan: first inventory and classify deltas, then build a small
`mjlab.sim2sim.unitree` package and CLI that generates output lanes from a clean
source checkout, then migrate wrappers/tests/docs, and only then run optional
runtime smoke.

Verification: static and unit/golden tests are runnable locally; final visual
or controller smoke is conditional on Unitree/MuJoCo runtime capability.

Remaining risk: this plan can prove reproducible generation and migration, but
it cannot certify real robot safety or clean official baseline behavior without
the missing runtime/input capability.

## Objective

Create a durable `mjlab` G1 sim2sim feature that can:

1. Prepare a labeled Unitree sim2sim working lane from a clean official source
   checkout.
2. Apply only declared, reviewed deviations for automation, model alignment,
   policy assets, and diagnostics.
3. Produce machine-readable manifests and evidence paths.
4. Replace phase-1 hardcoded shell workflow with repo-owned Python entry
   points and tested compatibility wrappers.
5. Keep `.external` as local runtime storage, not the canonical source of the
   feature.

## Active Slice

Productize the `official_source_plus_automation_deviation` G1 sim2sim lane as a
tested `mjlab` CLI/library feature while preserving existing phase-1 evidence
as historical input.

Current unique next item: none; P0-P5 have been implemented or recorded with
runtime-capability evidence.

## Non-goals

- Do not vendor the Unitree upstream repository into `mjlab`.
- Do not commit `.external`, build outputs, videos, logs, checkpoints, or local
  deploy artifacts.
- Do not claim clean official baseline success until the missing joystick/input
  or equivalent runtime capability is available and verified.
- Do not certify real robot deployment safety.
- Do not change PPO, task defaults, or training behavior as part of this
  productization slice.
- Do not keep new durable functionality only in `scripts/agent/`; agent scripts
  may remain as wrappers or archival utilities.

## Success Criteria

- A user can run one repo-owned `uv run` entry point to prepare a G1 sim2sim
  output lane from an official Unitree checkout and a selected action bundle.
- The generated lane includes a manifest with source SHA, output root, action,
  policy assets, changed paths, deviation labels, and evidence directory.
- The implementation is idempotent for an empty output root and refuses to
  overwrite an existing output root unless an explicit force flag is passed.
- Core generation logic is tested without requiring MuJoCo, DDS, Unitree SDK,
  joystick devices, or robot hardware.
- Old phase-1 wrappers either delegate to the new implementation or are clearly
  marked deprecated.
- Documentation distinguishes `clean_official`, `official_source_plus_deviation`,
  and `diagnostic_trace` claims.
- `.external/unitree_rl_mjlab` can be deleted or replaced locally without losing
  the productized sim2sim implementation, aside from user-owned runtime evidence.

## Verification Path

Primary runnable path:

```sh
uv run pytest tests/sim2sim/
uv run pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py
uv run ruff check src/mjlab scripts tests
uv run ty check
```

Plan and harness sanity:

```sh
scripts/agent/status.sh
git status --short --branch
```

Conditional runtime smoke, only when the host has the required Unitree/MuJoCo
capabilities:

```sh
uv run unitree-sim2sim prepare-g1 --official-root <clean-unitree-root> --out-root <scratch-lane> --action flying_kick --automation-sequence full
bash scripts/agent/run_official_deviation_smoke.sh
bash scripts/agent/capture_official_deviation_sim2sim.sh
```

## Verification Path Status

`runnable` for the productization core: manifest generation, patch/deviation
selection, path handling, wrapper delegation, and documentation checks can be
validated with local tests.

`blocked` for final clean official or visual runtime claims until a host exposes
the needed Unitree/MuJoCo runtime, display/input/joystick path, and any required
DDS/controller capability.

## Required Capabilities

- A clean official Unitree source checkout, defaulting to a user-supplied path
  rather than `.external`.
- `uv run` environment for `mjlab` tests and CLI commands.
- Test fixtures that model a small Unitree tree without relying on real build
  artifacts.
- Optional runtime capability for final smoke: MuJoCo simulator build,
  controller build, DDS loopback/network setup, and input automation or joystick
  access.

## Fallback Evidence

Accepted fallback for P0-P4:

- golden manifest tests;
- generated file tree assertions;
- idempotency and overwrite refusal tests;
- path-resolution tests proving no dependency on the deleted worktree path;
- existing archived videos/logs used only as historical comparison evidence.

Not accepted as final runtime integration evidence:

- old `.external` state alone;
- videos without a generated manifest;
- automation-deviation results labeled as clean official baseline;
- real robot deployment claims without explicit hardware gate evidence.

## final_integration_claim

`mjlab` can generate and verify a labeled G1 Unitree sim2sim lane from a clean
source checkout through repo-owned CLI/library code, without mutating
`.external` in place. The feature is regression-tested by static and golden
tests, and runtime evidence is explicitly labeled by capability and deviation
class.

## Final Verification Evidence

Passed:

- `uv run pytest tests/sim2sim/` -> 22 passed.
- `uv run pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py
  tests/tools/test_g1_tracking_phase1_manifest.py
  tests/tools/test_g1_tracking_phase1_contract.py` -> 38 passed.
- `uv run ruff check src/mjlab/sim2sim tests/sim2sim` -> passed.
- `uv run ty check src/mjlab/sim2sim tests/sim2sim` -> passed.
- `uv run unitree-sim2sim --help` and
  `uv run unitree-sim2sim prepare-g1 --help` -> passed.
- `bash -n` on touched sim2sim/smoke shell wrappers -> passed.
- `scripts/agent/status.sh` -> `harness status: ok`.
- `rg` over the productized sim2sim files and compatibility wrappers found no
  deleted `.worktrees/g1-flying-kick-main` defaults.

Broader checks with existing unrelated failures:

- `uv run ruff check src/mjlab scripts tests` fails in existing unrelated
  files such as `src/mjlab/flashsac/reference_ppo.py`,
  `src/mjlab/scripts/debug_robot.py`, `src/mjlab/viewer/viser/debug_panels.py`,
  and existing tests outside this slice.
- `uv run ty check` reports existing diagnostics in FlashSAC, debug robot,
  historical phase-1 analysis scripts, and tests outside this slice.

## Work Items

### P0 - Inventory and classify external deltas

Status: completed

Actions:

- Compare a clean Unitree source checkout with `.external/unitree_rl_mjlab`.
- Classify each changed path as `core_product`, `diagnostic_trace`,
  `runtime_artifact`, `policy_asset`, or `reject`.
- Record the expected durable behavior for model alignment, joystick
  automation, start-paused behavior, manifest labeling, and policy asset
  copying.
- Identify shell/script hardcoded paths that must be removed or wrapped.

acceptance_criteria:

- A checked-in inventory document or fixture names all source-level deltas that
  will be productized.
- Build outputs, backups, logs, and local runtime files are explicitly excluded.
- The stale deleted-worktree defaults are listed as defects to remove.

verification_commands:

```sh
git status --short --branch
diff -qr /tmp/unitree_rl_mjlab_official_baseline .external/unitree_rl_mjlab
rg -n "g1-flying-kick-main|/home/ssy/ssy_files/mjlab/.worktrees" scripts src tests docs
```

success_definition: every `.external` delta has an owner decision before code is
moved into a product package.

result:

- Inventory document:
  `docs/research/g1-sim2sim-delta-inventory-2026-05-26.md`
- Source-level deltas were classified as `core_product`,
  `diagnostic_trace`, `runtime_artifact`, `policy_asset`, or `reject`.
- Build outputs, backups, logs, and local runtime files were explicitly
  excluded.
- Stale deleted-worktree defaults were recorded as defects for P1-P3.

### P1 - Design the `mjlab.sim2sim.unitree` package contract

Status: completed

Actions:

- Add a small internal package for Unitree sim2sim preparation, manifest
  writing, source tree validation, and path resolution.
- Define typed data structures for source checkout, output lane, action bundle,
  deviation options, and manifest records.
- Add a console entry point, tentatively `unitree-sim2sim`, in
  `pyproject.toml`.
- Keep implementation independent of local `.external` paths unless the user
  passes them explicitly.

acceptance_criteria:

- The package has a narrow public entry point and no side effects on import.
- CLI help documents required source/output/action arguments.
- No default path references the deleted worktree.

verification_commands:

```sh
uv run unitree-sim2sim --help
uv run pytest tests/sim2sim/test_unitree_cli_contract.py
uv run ruff check src/mjlab/sim2sim tests/sim2sim
```

success_definition: the repo has a stable product API boundary before porting
the old patching logic.

result:

- Added `src/mjlab/sim2sim/unitree/` with typed source, output, action bundle,
  deviation, request, and manifest contracts.
- Added the `unitree-sim2sim` console entry point with a `prepare-g1`
  subcommand and no local `.external` or deleted-worktree defaults.
- Added P1 contract tests under `tests/sim2sim/`.

### P2 - Port lane generation from agent script to library code

Status: completed

Actions:

- Move reusable logic from `scripts/agent/prepare_official_deviation_lane.py`
  into the new package.
- Generate a fresh output root from a clean official source root.
- Apply productized deviations by option:
  - disable physical joystick and use automation sequence;
  - copy selected policy assets;
  - sync the `mjlab` mode-15 G1 model when requested;
  - align actuator limits when requested;
  - apply official passive joint defaults when requested;
  - emit optional diagnostic traces only when requested.
- Write a manifest before returning success.

acceptance_criteria:

- Generation refuses to mutate the official source root.
- Generation refuses to overwrite an existing output root unless `--force` is
  provided.
- Manifest changed paths match the files actually generated.
- Diagnostic traces are opt-in and cannot be confused with core product
  behavior.

verification_commands:

```sh
uv run pytest tests/sim2sim/test_unitree_lane_generation.py
uv run pytest tests/sim2sim/test_unitree_manifest.py
uv run pytest tests/sim2sim/test_unitree_deviation_labels.py
```

success_definition: a clean fixture source tree can be transformed into a
manifest-labeled output lane without touching `.external`.

result:

- Added `prepare_g1_lane()` in `mjlab.sim2sim.unitree.generator`.
- Generation copies from a caller-supplied official checkout into a fresh
  output root, refuses in-source output roots, and requires `--force` before
  replacing an existing output.
- Generated lanes apply declared automation, policy asset, optional mode-15
  model, optional official passive default, optional actuator limit alignment,
  and optional diagnostic trace deviations.
- Generated `UNITREE_SIM2SIM_MANIFEST.json` records source identity, output
  root, action, policy hashes, changed paths, deviation labels, evidence dir,
  and forbidden claims.
- Added P2 tests for lane generation, manifest integrity, overwrite refusal,
  source immutability, and diagnostic-label opt-in.

### P3 - Productize G1 action bundles and compatibility wrappers

Status: completed

Actions:

- Define first-class action bundle metadata for `flying_kick` and
  `roundhouse_leading_right`.
- Replace hardcoded shell assumptions in dual-kick and single-action sim2sim
  wrappers with delegation to the new CLI/library where practical.
- Preserve existing wrapper names only as compatibility entry points.
- Ensure scripts fail fast when required assets are absent.

acceptance_criteria:

- Existing wrapper tests pass without relying on the deleted worktree path.
- New CLI tests cover action selection, missing assets, and manifest output.
- Compatibility wrappers print the new command path in their status/help.

verification_commands:

```sh
uv run pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py
uv run pytest tests/sim2sim/test_g1_action_bundles.py
uv run unitree-sim2sim prepare-g1 --help
```

success_definition: users can reach the productized path from both new CLI and
old wrapper names.

result:

- First-class action bundle metadata now covers `flying_kick` and
  `roundhouse_leading_right`.
- The single-action and dual-action sim2sim wrappers now default `WORKTREE` to
  the current repo root instead of the deleted
  `.worktrees/g1-flying-kick-main` path.
- Compatibility wrappers expose `prepare-lane`, which delegates to
  `uv run unitree-sim2sim prepare-g1` while preserving existing launch/status
  behavior.
- Wrapper status/help prints the new productized prepare command.
- Added tests for action bundles and wrapper delegation.

### P4 - Documentation and evidence semantics

Status: completed

Actions:

- Add user-facing docs under `docs/source/` for G1 sim2sim preparation,
  evidence labels, and runtime prerequisites.
- Update `docs/agent/harness.md` only if the harness needs to mention the new
  persistent feature; do not store active task state in `AGENTS.md`.
- Link the archived 2026-05-25 research as historical evidence, not current
  instructions.
- Mark old phase-1 agent utilities as archived or compatibility-only.

acceptance_criteria:

- Docs contain the new CLI, required inputs, output manifest semantics, and
  claim labels.
- Docs explicitly state that `.external` is local runtime state.
- Docs do not claim real deploy or clean official baseline certification.

verification_commands:

```sh
rg -n "unitree-sim2sim|official_source_plus|clean_official|diagnostic_trace" docs src scripts tests
scripts/agent/status.sh
```

success_definition: a fresh agent can understand how to run and label the
feature without reading old long research plans first.

result:

- Added `docs/source/debugging/g1_unitree_sim2sim.rst` and linked it from
  `docs/index.rst`.
- Documented the new CLI, compatibility wrapper `prepare-lane` commands,
  manifest semantics, claim labels, runtime prerequisites, and `.external`
  policy.
- Updated `docs/source/debugging/g1_tracking_phase1.rst` to point durable lane
  preparation to the new productized runbook and keep the old page as
  diagnostic/historical context.

### P5 - Optional runtime smoke and final evidence capture

Status: completed_with_runtime_blocker

Actions:

- On a capable host, run the new CLI against a clean official Unitree checkout.
- Build or reuse the generated output lane without writing into `.external`.
- Run a bounded automation-deviation smoke for `flying_kick` and
  `roundhouse_leading_right`.
- Capture logs, manifest, video/contact sheet when available, and record the
  capability label used.

acceptance_criteria:

- Smoke evidence directory contains manifest, command log, simulator/controller
  logs, and any captured video/contact sheet.
- Evidence label matches the actual path, for example
  `official_source_plus_automation_deviation`.
- Failure to run smoke is recorded as a runtime capability blocker, not as a
  productization failure if P0-P4 pass.

verification_commands:

```sh
uv run unitree-sim2sim prepare-g1 --official-root <clean-unitree-root> --out-root <scratch-lane> --action flying_kick --automation-sequence full
bash scripts/agent/run_official_deviation_smoke.sh
bash scripts/agent/capture_official_deviation_sim2sim.sh
```

success_definition: runtime evidence, when available, proves that the generated
lane is runnable and correctly labeled.

result:

- Generated real scratch lanes from clean official source
  `/tmp/unitree_rl_mjlab_official_baseline`:
  - `/tmp/mjlab-g1-unitree-sim2sim-flying-kick`
  - `/tmp/mjlab-g1-unitree-sim2sim-roundhouse`
- Both lanes produced `UNITREE_SIM2SIM_MANIFEST.json` with
  `lane: official_source_plus_automation_deviation` and
  `claim: not_clean_official_baseline`.
- The manifests record selected action metadata, policy/model asset hashes,
  changed paths, deviation labels, source SHA `1425b15`, and forbidden claims.
- Visual/controller runtime smoke was blocked by runtime capability, not by the
  productized generator:
  - `/dev/input/js0` is absent on this host;
  - generated lanes intentionally drop stale `simulate/build` and
    `deploy/robots/g1/build` outputs;
  - `bash scripts/agent/run_official_deviation_smoke.sh
    /tmp/mjlab-g1-unitree-sim2sim-flying-kick` fails fast with
    `missing simulator binary`.
- Updated legacy smoke/capture scripts to accept either the new
  `UNITREE_SIM2SIM_MANIFEST.json` or the historical
  `AUTOMATION_DEVIATION_MANIFEST.json`.

## Commit Units

### CU1 - Productization inventory

scope: plan plus source-delta inventory only.

corresponding phases: P0.

preconditions before commit:

- Review has no Critical findings.
- `scripts/agent/status.sh` passes.
- `git status --short --branch` has only intended docs/inventory changes.

### CU2 - Package and CLI contract

scope: new package skeleton, CLI entry point, fixtures, and contract tests.

corresponding phases: P1.

preconditions before commit:

- Review has no Critical findings.
- `uv run unitree-sim2sim --help` succeeds.
- P1 tests pass.

### CU3 - Lane generation and manifest implementation

scope: productized generator, manifest schema, deviation labels, and golden
tests.

corresponding phases: P2.

preconditions before commit:

- Review has no Critical findings.
- P2 tests pass.
- No generation test mutates `.external`.

### CU4 - Wrappers, action bundles, and docs

scope: wrapper delegation, action bundle metadata, user docs, and harness doc
touch-ups if needed.

corresponding phases: P3, P4.

preconditions before commit:

- Review has no Critical findings.
- Existing phase-1 wrapper tests pass.
- Documentation grep checks pass.

### CU5 - Runtime evidence, if available

scope: evidence references and runtime smoke notes only, without committing raw
videos/logs unless explicitly approved.

corresponding phases: P5.

preconditions before commit:

- Review has no Critical findings.
- Runtime smoke either passes with manifest-labeled evidence, or blocker is
  documented without changing the final productization claim.

## Known Risks and Blockers

- The current automation-deviation lane is valuable but not a clean official
  baseline. Labels must stay strict.
- `.external` includes runtime/build artifacts and backup files mixed with
  source edits; blindly copying it would add entropy.
- Historical phase-1 utilities are compatibility surfaces; the source of truth
  for durable lane generation is `uv run unitree-sim2sim prepare-g1`.
- Runtime smoke can be blocked by missing joystick/input, display, MuJoCo build,
  DDS loopback, or controller build capability.
- The local `main` currently carries one squash commit ahead of `fork/main`; do
  not rewrite it while implementing this plan unless the user explicitly asks.
- Real robot deployment remains a separate gated workflow.

## Handoff

Recommended next skill: `harness-workflow:implement`.

Reason: the plan has a clear active slice, primary verification path is runnable
for productization, and the first implementation step is P0 inventory plus
path-hardening groundwork. Route to `harness-workflow:brainstorm` instead if
the product API name, supported action set, or evidence-label taxonomy needs
user discussion before implementation.
