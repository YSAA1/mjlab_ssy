# G1 Sim2sim Delta Inventory

Created: 2026-05-26

Status: P0 inventory for
`docs/plans/2026-05-26--g1-sim2sim-productization.md`

## Scope

This inventory classifies the local `.external/unitree_rl_mjlab` deltas against
the clean Unitree baseline at `/tmp/unitree_rl_mjlab_official_baseline`.

It is an implementation boundary for productizing the existing
`official_source_plus_automation_deviation` lane. `.external` remains local
runtime storage and is not the canonical source for the feature.

## Evidence Commands

```sh
git status --short --branch
git -C /tmp/unitree_rl_mjlab_official_baseline status --short --branch
diff -qr /tmp/unitree_rl_mjlab_official_baseline .external/unitree_rl_mjlab
rg -n "g1-flying-kick-main|/home/ssy/ssy_files/mjlab/.worktrees" scripts src tests docs
```

Observed state:

- `mjlab` local `main` is ahead of `fork/main` by one commit.
- `/tmp/unitree_rl_mjlab_official_baseline` is a clean git checkout on
  `main...origin/main`.
- `.external/unitree_rl_mjlab` is a plain local runtime tree under `mjlab`, not
  a standalone git checkout in this workspace.
- The clean baseline contains `.git`; `.external/unitree_rl_mjlab` does not.

## Claim Labels

- `clean_official`: unchanged official Unitree source and runtime behavior.
  This was not proven in the prior lane because joystick/input capability was
  missing.
- `official_source_plus_deviation`: official source copied into a fresh output
  lane, then modified by declared product deviations.
- `official_source_plus_automation_deviation`: the first supported
  productization target for no-joystick sim2sim automation.
- `diagnostic_trace`: optional instrumentation for root-cause analysis. It must
  not be enabled by default or confused with core product behavior.

## Source-Level Delta Decisions

| Path | Classification | Decision |
| --- | --- | --- |
| `simulate/config.yaml` | `core_product` | Productize as generated config values: `use_joystick: 0`, `start_paused`, and optional `initial_qpos`. Do not preserve the current local YAML formatting as source. |
| `simulate/src/param.h` | `core_product` | Productize support for optional `start_paused` and `initial_qpos` fields in generated lanes. |
| `simulate/src/main.cc` | `core_product` + `diagnostic_trace` | Productize `initial_qpos` application and `start_paused`; keep `[PHASE1_SIM]` transition trace helpers diagnostic-only and opt-in. |
| `simulate/src/unitree_sdk2_bridge.h` | `diagnostic_trace` | Current delta is lowcmd/control telemetry only. Do not make it a default core deviation. A future automation joystick bridge should be generated from the agent script logic, not copied from this trace-heavy local file. |
| `simulate/src/physics_joystick.h` | `core_product` planned | No current `.external` delta, but the existing agent script injects `AutoSequenceJoystick`. Productize this as generated automation input when `--automation-sequence` is selected. |
| `deploy/include/FSM/CtrlFSM.h` | `core_product` | Productize configurable `FSM.initial_state` lookup for generated lanes. |
| `deploy/include/FSM/State_FixStand.h` | `core_product` + `diagnostic_trace` | Productize reading gains/targets from the concrete state name and initializing from lowstate. Keep stability logging diagnostic-only. |
| `deploy/include/FSM/State_RLBase.h` | `diagnostic_trace` | Current delta is phase-1 start/tick gates controlled by env vars. Keep out of default core path; allow only as an explicit diagnostic option if retained. |
| `deploy/include/isaaclab/envs/manager_based_rl_env.h` | `diagnostic_trace` | Dense policy IO tracing only. Exclude from default product generation. |
| `deploy/include/isaaclab/envs/mdp/actions/joint_actions.h` | `core_product` | Productize reset behavior so zero raw action produces default/offset processed targets. This is a small semantic fix, not trace output. |
| `deploy/robots/g1/CMakeLists.txt` | `core_product` | Productize the build-system adjustment that uses `find_package(unitree_sdk2 REQUIRED)` and avoids hardcoded local DDS include paths. |
| `deploy/robots/g1/main.cpp` | `reject` | Only changes console prompts for a local getup/demo flow. Do not productize as durable behavior. |
| `deploy/robots/g1/config/config.yaml` | `core_product` + `policy_asset` | Generate action-state entries for selected bundles and selected initial state. Do not copy the whole local merged config. |
| `deploy/robots/g1/include/State_Mimic.h` | `diagnostic_trace` | Only trace bookkeeping fields. Exclude from core product path. |
| `deploy/robots/g1/src/State_Mimic.cpp` | `diagnostic_trace` | Motion/policy/q-response logs are diagnostic-only and should be opt-in if retained. |
| `deploy/robots/g1/src/State_RLBase.cpp` | `diagnostic_trace` | Velocity runtime trace and stability samples are diagnostic-only. |
| `src/assets/robots/unitree_g1/xmls/g1.xml` | `core_product` | Productize optional sync to the `mjlab` mode-15/new-G1 model. Do not treat it as clean official. |
| `src/assets/robots/unitree_g1/xmls/scene_g1.xml` | `core_product` | Productize generated scene alignment that preserves official actuator/sensor blocks while applying the `mjlab` body/collision model, optional actuator-limit alignment, and official passive defaults. |
| `src/assets/robots/unitree_g1/xmls/assets/*_5010.STL` | `core_product` | Productize as required model assets copied from the selected `mjlab` model source when mode-15 alignment is enabled. |

## Policy Asset Deltas

These assets are not source code. Productized generation should copy them only
when the selected action bundle asks for them and should record file hashes in
the generated manifest.

| Path | Classification | Decision |
| --- | --- | --- |
| `deploy/robots/g1/config/policy/mimic/flying_kick/exported/policy.onnx` | `policy_asset` | Copy for `flying_kick` bundle. |
| `deploy/robots/g1/config/policy/mimic/flying_kick/params/deploy.yaml` | `policy_asset` | Copy for `flying_kick` bundle. |
| `deploy/robots/g1/config/policy/mimic/flying_kick/params/flying_kick.npz` | `policy_asset` | Copy for `flying_kick` bundle. |
| `deploy/robots/g1/config/policy/mimic/roundhouse_leading_right/exported/policy.onnx` | `policy_asset` | Copy for `roundhouse_leading_right` bundle. |
| `deploy/robots/g1/config/policy/mimic/roundhouse_leading_right/params/deploy.yaml` | `policy_asset` | Copy for `roundhouse_leading_right` bundle. |
| `deploy/robots/g1/config/policy/mimic/roundhouse_leading_right/params/roundhouse_leading_right.npz` | `policy_asset` | Copy for `roundhouse_leading_right` bundle. |
| `deploy/robots/g1/config/policy/mimic/getup/*` | `policy_asset` + `reject` for this slice | Existing local getup assets are not part of the first productized action set. Do not copy unless a later getup bundle is specified. |
| `deploy/robots/g1/config/policy/mimic/getup/*.bak_before_model3500_tailhold` | `runtime_artifact` | Exclude. |

## Explicitly Excluded Artifacts

These are not candidates for product source.

| Path pattern | Classification | Reason |
| --- | --- | --- |
| `.git` under the clean baseline | `runtime_artifact` | Baseline checkout metadata; not part of generated lanes. |
| `simulate/build/**` | `runtime_artifact` | CMake products, object files, binaries, and local build cache. |
| `deploy/robots/g1/build/**` | `runtime_artifact` | CMake products, object files, binaries, local agent state, and local build cache. |
| `MJMODEL.TXT`, `MUJOCO_LOG.TXT` | `runtime_artifact` | MuJoCo runtime output. |
| `*.bak`, `*.before_*`, `*.pre_*` under `.external/unitree_rl_mjlab` | `runtime_artifact` | Local backup/checkpoint files used during diagnostics. |
| `deploy/robots/g1/config/config.yaml.bak_hold_after_end_test` | `runtime_artifact` | Local backup from an experiment. |
| `deploy/robots/g1/src/State_Mimic.cpp.bak_hold_after_end_test` | `runtime_artifact` | Local backup from an experiment. |
| `src/assets/robots/unitree_g1/xmls/scene_g1.xml.before_*` | `runtime_artifact` | Local model-alignment backup snapshots. |

## Expected Durable Behavior

The productized implementation should generate a fresh output lane from a clean
official checkout and a selected action bundle. It should not mutate the source
checkout or `.external` in place.

Durable behaviors to implement in P1-P3:

- Validate source root shape and output root.
- Refuse an existing output root unless `--force` is passed.
- Copy only selected policy assets and model assets.
- Generate config/FSM/action entries for `flying_kick` and
  `roundhouse_leading_right`.
- Generate a manifest containing source root, source SHA, output root, action,
  policy asset hashes, changed paths, deviation labels, and evidence directory.
- Keep diagnostic traces disabled unless explicitly requested.
- Label generated lanes as `official_source_plus_automation_deviation` or a
  narrower declared claim, never as `clean_official`.

## Hardcoded Path Defects

The following references must be removed or wrapped during P1-P3:

| Path | Defect |
| --- | --- |
| `scripts/agent/prepare_official_deviation_lane.py` | Defaults to `/tmp/unitree_rl_mjlab_official_baseline`, `/tmp/g1_official_automation_deviation`, `.external` policy assets, and a deleted `.worktrees/g1-flying-kick-main` mode-15 XML. |
| `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh` | Defaults `MJLAB_WORKTREE` to `/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main`. |
| `scripts/tools/run_flying_kick_sim2sim.sh` | Defaults `MJLAB_WORKTREE` to the deleted worktree path and mutates `.external/unitree_rl_mjlab` in place. |
| `scripts/tools/run_roundhouse_leading_right_sim2sim.sh` | Same stale worktree default and in-place `.external` mutation pattern as the flying-kick wrapper. |
| `src/mjlab/scripts/g1_tracking_phase1_manifest.py` | Contains a default `.worktrees/g1-flying-kick-main` path for phase-1 manifest generation. |

Historical references in `docs/plans/2026-05-22--g1-tracking-sim2real-hardening.md`
are archival evidence and do not need removal as part of the product API.

## P0 Decision

Proceed to P1 with a new repo-owned package/CLI. The first implementation
should port behavior from `scripts/agent/prepare_official_deviation_lane.py`
selectively, using this inventory as the allowed-deviation boundary. Do not
copy `.external/unitree_rl_mjlab` wholesale.
