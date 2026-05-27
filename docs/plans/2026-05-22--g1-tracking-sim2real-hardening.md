# Executable Plan - G1 Tracking Sim2real Hardening

> Superseded on 2026-05-23 by the project harness recovery surface.
> Do not use this file as the active task plan. Resume from
> `.harness/state.md` and `docs/research/archive/g1-sim2sim-2026-05-25.md`.
> This file is retained only as historical evidence for targeted lookup.

> Status: user-approved; amended after phase-1 sim2sim2 blocker and official sim2sim script review
> Date: 2026-05-22
> Spec source: `docs/specs/2026-05-22--g1-tracking-sim2real-hardening.md`
> Planning surface: docs plan
> Active slice: reset to an official-baseline-first diagnosis. The current
> external Unitree checkout and sim/deploy configs are polluted by phase-1
> wrapper state and local diagnostic patches, so the next work must restore or
> isolate a clean upstream Unitree Velocity sim2sim baseline before judging
> mjlab wrapper behavior, Velocity policy quality, or lowcmd handoff fixes.

## Objective

Build a phase-1 deploy-readiness gate that proves the current G1 flying-kick and roundhouse tracking bundles are evaluated against the correct new Unitree G1 29DoF mode 15 contract, then runs the same bundles through a Unitree MuJoCo sim2sim2 path before any real-robot trial.

The plan is diagnosis-first. It does not train, tune rewards, or modify the generic tracking baseline. It exists to separate model quality, robot asset mismatch, deploy timing, FSM handoff, Velocity response, and post-action standing stability.

## Current Evidence Snapshot

The original phase-1 validation stack has been implemented and run. It proved the new-G1 contract and produced sim2sim2 evidence, but the evidence did not pass the 5-second stability gate.

Committed evidence path:

- `2367496a` - baseline manifest command.
- `87582be5` - new-G1 29DoF mode 15 contract validator.
- `71e48c14` - dual-action sim2sim2 preflight wrapper.
- `345c52a2` - timing/stability log parser.
- `df63e4d2` - sim2sim2 evidence classification.
- `e8f8de57` - direct Mimic diagnosis mode.
- `9c869c5f` - in-action instability classifier.
- `18ed1bea` - entry-pose gap checker.

Local evidence paths:

- Manifest: `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json`
- Contract report: `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/contract_report.json`
- Preflight report: `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/sim2sim2_preflight.json`
- Flying-kick sim2sim2: `logs/flying_kick_sim2sim/20260522-143349/`
- Roundhouse sim2sim2: `logs/roundhouse_leading_right_sim2sim/20260522-143755/`
- Entry-gap report: `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/entry_gap_report.json`

Observed blocker:

- Both actions fail before a stable return to `Velocity`; the parser classifies the first visible instability as `policy_action_to_joint_response_mismatch`.
- The narrower follow-up diagnosis classifies the upstream cause as `entry_state_pose_mismatch`.
- Flying kick: frame-0 default-vs-reference gap L2 `1.914434`, max joint gap `1.048041`; the best default-pose frame is still L2 `1.116213` at frame 39 / `0.78s`.
- Roundhouse: frame-0 default-vs-reference gap L2 `1.935909`, max joint gap `0.980142`; the best default-pose frame is still L2 `1.731724` at frame 64 / `1.28s`.
- Handoff gate report: `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/entry_handoff_gate.json`
- The active sim `initial_qpos` is `sim_teleport_only` for flying kick and not the roundhouse reference state. It is cause-isolation evidence only and cannot unlock hardware.
- Work item 6 adds a non-teleport `prepose` candidate where `FixStand` interpolates from stand qpos to each action's motion frame-0 joint pose before the Mimic trigger.
- Work item 7 first found an evidence-harness bug: the original `MJLAB_AUTO_RUN_AFTER_READY=1` path sent a space key to MuJoCo and printed success, but screenshots still showed the window in `PAUSE`. That made `logs/flying_kick_sim2sim/20260522-153100/`, `logs/roundhouse_leading_right_sim2sim/20260522-153225/`, `logs/flying_kick_sim2sim/20260522-154417/`, and `logs/roundhouse_leading_right_sim2sim/20260522-154539/` diagnostic-only paused runs.
- The sim2sim scripts were hardened to click the MuJoCo `Run` radio button after `FixStand` is ready, then both actions were rerun with `MJLAB_AUTO_RUN_AFTER_READY=1`, `--mode prepose`, and `--start-paused 1`.
- Final evidence directories:
  - flying kick: `logs/flying_kick_sim2sim/20260522-155756/`
  - roundhouse: `logs/roundhouse_leading_right_sim2sim/20260522-155950/`
- Both final prepose runs start from the non-teleport stand qpos and visibly enter MuJoCo `Run`, then fail in `FixStand` before any Mimic episode. Parser classification is `insufficient_timing_evidence` because no Mimic episode exists; root evidence still points to entry-state pose mismatch / prepose controller infeasibility.
- A follow-up public script review found that official Unitree G1/H1 sim2sim bootstrap uses `unitree_mujoco` plus `g1_ctrl` with the elastic band enabled, lowers the robot with MuJoCo key `8`, then releases the elastic band with key `9` only after the policy state is stable. That means a free-standing `FixStand` run with `enable_elastic_band=0` is too strict as a bootstrap diagnostic and is not the same as the documented Unitree sim2sim flow.
- The first live `official_bootstrap` smoke for flying kick, `logs/flying_kick_sim2sim/20260522-163437/`, failed immediately: the robot was pulled upward before policy entry. Root cause is the local Unitree elastic band implementation: the band is enabled with anchor `(0, 0, 3)`, stiffness `200`, damping `100`, and default `length=0`, so entering MuJoCo `Run` before pre-tensioning the band applies a large upward force to the torso.
- Therefore the bootstrap path must not auto-run by default. If auto-run is explicitly enabled for operator-controlled smoke evidence, the helper must send MuJoCo key `8` before clicking `Run` to pre-tension the elastic length. Real-robot trial gating remains locked.
- Paused-safe evidence after the fix is `logs/flying_kick_sim2sim/20260522-164258/`. It confirms `official_bootstrap` starts with `elastic band: 1`, `start paused: 1`, and `auto run: 0`; selected sim config has `enable_elastic_band: 1` and `start_paused: 1`; controller logs remain stable in `FixStand` with `q_err_l2=0` while paused; screenshot evidence is `mujoco_paused_safe.png`.
- The explicit auto-run smoke with 24 key-8 pre-tension steps is `logs/flying_kick_sim2sim/20260522-165042/`. It no longer launches upward, but the robot falls in MuJoCo `Run` while still in `FixStand`, before any `Velocity` or `Mimic` judgment. The first unstable sample appears around `2026-05-22 16:50:48.585` with `q_err_l2=4.246`, followed by a clear tip around `2026-05-22 16:50:49.085` with `gravity_b=(0.866,-0.374,0.332)`. Screenshot evidence is `mujoco_run_fall_after_pretension.png`.
- Public Unitree FSM evidence and the local `State_FixStand` source both show that `FixStand` is a PD pose-transition state, not a learned balancing or tracking controller. A free-standing `FixStand` fall is therefore not a valid Velocity/tracking-policy failure. The next gate must use a `Velocity`-first bootstrap and judge standing stability in `Velocity` before triggering `Mimic`.
- Velocity-first paused smoke evidence is `logs/flying_kick_sim2sim/20260522-170654/`. It confirms `velocity_bootstrap` launches with `FSM.initial_state: Velocity`, `enable_elastic_band: 1`, `start_paused: 1`, stand `initial_qpos`, and `Velocity.policy_dir: config/policy/velocity`; the controller resolves the actual policy directory to `config/policy/velocity/v0`; the log reaches `FSM: Start Velocity` and records 117 `stable=1` Velocity samples while paused; screenshot evidence is `mujoco_velocity_bootstrap_paused.png`; restore leaves no `g1_ctrl` or `unitree_mujoco` process active.
- Velocity-first explicit Run evidence is `logs/flying_kick_sim2sim/20260522-171001/`. It waits for `FSM: Start Velocity`, sends 24 key-8 pre-tension steps, clicks MuJoCo `Run`, and does not release key `9`. The run is stable in `Velocity` from `17:10:03.941` through `17:10:06.921`, then first fails at line 33 / `17:10:07.421` with `stable=0`, `q_err_l2=5.614`, `gravity_b=(0.150,-0.963,-0.223)`, and `root_ang_vel_l2=6.364`. Screenshot evidence is `mujoco_velocity_bootstrap_run_fall.png`; restore leaves no `g1_ctrl` or `unitree_mujoco` process active.
- The active blocker has therefore moved upstream: not `FixStand` and not the tracking Mimic policy yet, but the Velocity deploy/sim contract itself. Next diagnosis should inspect the Velocity policy asset, default stand pose versus policy offset, gains/action scale, and whether the new G1 mode-15 sim asset matches the Velocity policy's training/deploy contract.
- Velocity contract diagnosis report: `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_bootstrap_report.json`. The report passes all 29-DoF dimension checks and resolves `Velocity.policy_dir` to `config/policy/velocity/v0` with `policy.onnx` present, but classifies `primary_reason: velocity_default_pose_mismatch`. The stand qpos differs from the Velocity policy `default_joint_pos` and action offset by L2 `0.779069`, max joint gap `0.369`, with largest gaps at knees 3/9 (`0.669` initial vs `0.3` policy default) and arms 18/25 (`0.6` initial vs `0.87` policy default). Runtime evidence then fails at line 33 after 3.5 seconds of stable samples.
- Policy-default Velocity bootstrap evidence is `logs/flying_kick_sim2sim/20260522-172725/`, with report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_default_bootstrap_report.json`. It sets `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default` so selected `initial_qpos[7:]` exactly matches the Velocity `default_joint_pos` / action offset (`gap_l2=0.0`, `gap_max=0.0`), but still fails in `Velocity` before any Mimic trigger at line 34 / `17:27:31.859` with `q_err_l2=6.547`, `q_err_max=5.002`, `gravity_b=(-0.141,-0.822,0.552)`, and `root_ang_vel_l2=3.656`. The extended report classifies `primary_reason: velocity_initial_contact_mismatch`: ONNX/deploy observations match at 98 dims, but the selected root z `0.765781` leaves the Velocity policy-default foot collision surface at `-0.018422` below the floor. Current mjlab G1 source init-state is `KNEES_BENT_KEYFRAME`; the selected root z is `0.017894` below `HOME_KEYFRAME` root z `0.783675`. This falsifies the default-pose-only root cause and makes the next allowed slice a grounded policy-default root-height test.
- Grounded policy-default Velocity evidence is `logs/flying_kick_sim2sim/20260522-175113/`, with report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_bootstrap_report.json`. It sets `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`, making selected root z `0.783675` and improving lowest foot collision surface to `-0.000527`, which passes the contact gate. It still fails in `Velocity` before any Mimic trigger at line 33 / `17:51:20.049` with `q_err_l2=6.767`, `q_err_max=3.929`, `gravity_b=(-0.365,-0.391,-0.845)`, and `root_ang_vel_l2=7.765`; the report classifies `primary_reason: velocity_runtime_instability`. This falsifies initial floor penetration as the sufficient root cause. The active blocker is now the Velocity policy runtime contract itself: stale Velocity policy/export, training robot mismatch, gains/action scale mismatch, observation timing/value mismatch, or dynamics mismatch against the new G1 mode-15 asset.
- No-elastic grounded policy-default Velocity evidence is `logs/flying_kick_sim2sim/20260522-175822/`, with report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_no_elastic_bootstrap_report.json`. It keeps `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default` and `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home` but sets `MJLAB_ENABLE_ELASTIC_BAND=0`. The run still fails in `Velocity` before any Mimic trigger at line 30 / `17:58:26.868` with `q_err_l2=5.438`, `q_err_max=3.809`, `gravity_b=(-0.579,-0.071,-0.812)`, and `root_ang_vel_l2=4.790`; the report again classifies `primary_reason: velocity_runtime_instability`. This falsifies the elastic band as the sole cause. The wrapper output now reports the actual `elastic band=<0|1>` value instead of saying it is always enabled.
- The extended Velocity report now checks policy provenance and current-source deltas. The active `policy.onnx` metadata says `run_path: 2026-03-18_18-40-20`, but no matching local `logs/rsl_rl` source run is found. Current source init is still `KNEES_BENT_KEYFRAME`, with L2 `0.779069` / max `0.369` versus the deploy `v0` default/action offset, while action scale, stiffness, and damping differ only by small rounding-level gaps (`action_scale` max `0.004501`, stiffness max `0.049377`, damping max `0.042110`). This makes a gross gain/action-scale mismatch less likely and makes stale/missing-provenance Velocity policy, observation semantics/timing, or dynamics mismatch the next diagnostic target.
- Local Velocity policy inventory report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_inventory.json` scanned 125 local `g1_velocity*` ONNX candidates under `/home/ssy/ssy_files/mjlab/logs/rsl_rl`. None are directly compatible with active deploy `v0`: the reference policy is `obs[1,98] -> actions[1,29]` with observations `base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel, actions`, while local candidates are mostly `obs[1,99] -> actions[1,29]` with `base_lin_vel, base_ang_vel, projected_gravity, joint_pos, joint_vel, actions, command` or rough-terrain `obs[1,286]`. This means we cannot safely swap in a local April Velocity ONNX without also changing the deploy observation contract.
- Velocity runtime trace evidence is `logs/flying_kick_sim2sim/20260522-183512/`, with screenshot `mujoco_velocity_runtime_trace.png` and report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_trace_report.json`. It keeps policy-default joints, home root, no elastic band, and zero command. It still fails in `Velocity` at line 29 / `18:35:17.211` after only 1.5 seconds of stable samples. First unstable trace: `policy_step=75`, `command_norm=0.0`, `raw_action_l2=14.76`, `raw_action_max=7.827`, `processed_action_l2=4.874`, `processed_action_max=2.241`, `joint_vel_l2=673.649`, `joint_vel_max=576.357`, `gravity_b=(-0.052,-0.643,-0.764)`, `root_ang_vel_l2=8.491`. This points away from delayed/nonzero velocity commands as the primary cause of this no-command fall and toward active Velocity policy output/observation closed-loop mismatch.
- Zero-command ONNX replay report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_zero_command_replay.json` constructs the active deploy `v0` 98-dim observation with zero base angular velocity, gravity `(0,0,-1)`, zero command, zero gait phase output, zero joint position/velocity error, and recurrent `last_action`. The active `policy.onnx` still outputs nonzero targets: over 5 replay steps, max processed target gap from default/action offset is L2 `0.660817`, max joint gap `0.357822`; raw action L2 rises from `1.228195` to `1.679312`. This matches the early runtime trace before the fall (`raw_action_l2≈1.68`, processed target gap/q_err≈0.59) and confirms the active `v0` zero-command closed loop is not a default-pose hold.
- Velocity deploy candidate triage report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy_candidate_triage.json` scanned the same 125 local Velocity ONNX candidates and found no direct replacement for active `velocity/v0`: `direct_swap_ready=0`, `complete_unitree_deploy_package=0`, `active_v0_contract=0`. It did find 122 current-source flat Velocity actor candidates with checkpoints and params (`obs[1,99] -> actions[1,29]`, including `base_lin_vel`), but all require Unitree deploy YAML generation and runtime support for the 99-dim observation contract before sim2sim. This keeps direct ONNX swapping locked and narrows remediation to either a complete 99-dim deploy path or a retrained/re-exported 98-dim Velocity package.
- Velocity runtime observation audit report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_observation_audit.json` proves the 99-dim route is not just missing YAML: current worktree flat Velocity actor is 99 dims with `base_lin_vel`, but the active Unitree deploy YAML and external source flat actor are 98 dims. The C++ deploy runtime registers no `base_lin_vel` observation, `ArticulationData` has no linear-velocity field, and `BaseArticulation::update()` does not populate linear velocity. Therefore `safe_to_generate_99_dim_deploy_yaml_only=false`; generating only a 99-dim deploy YAML would create another invalid sim2sim path. The next viable route is either add a correct runtime `base_lin_vel` source before 99-dim deployment, or train/export a 98-dim Velocity package for the active runtime contract.
- Velocity deploy98 task contract report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_task_contract.json` registers `Mjlab-Velocity-Flat-Unitree-G1-Deploy98` as a separate task ID, leaving existing flat/rough Velocity defaults unchanged. Its actor contract is 98 dims with terms `base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel, actions`; semantic mapping to active deploy YAML is complete (`command -> velocity_commands`, `phase -> gait_phase`, `joint_pos -> joint_pos_rel`, `joint_vel -> joint_vel_rel`, `actions -> last_action`), and `task_contract_matches_active_runtime=true`. This unlocks a 98-dim Velocity training/export path, not a direct ONNX swap or hardware trial.
- Velocity deploy98 package generator smoke reports `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_smoke.json` and `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_write_smoke.json` prove the tooling can convert a compatible 98-dim deploy ONNX plus template deploy YAML into a Unitree policy directory layout. The smoke intentionally used the stale active `velocity/v0` policy only to verify package generation, not to accept that policy. The generated report keeps `safe_to_use_for_sim2sim=false`, `safe_to_swap_without_zero_command_replay=false`, and `real_robot_gate=locked`.
- Manual actor ONNX export now attaches the same base deploy metadata as training-time Velocity exports. This keeps post-hoc checkpoint export compatible with the deploy98 package generator instead of producing a metadata-empty ONNX that cannot be safely packaged.
- Deploy98 training/export smoke `logs/rsl_rl/g1_velocity_deploy98_smoke/2026-05-22_19-40-43_deploy98_smoke_20260522/` proves the new task can run one CPU training iteration and export a metadata-bearing `obs[1,98] -> actions[1,29]` ONNX. Package report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_package.json` confirms it can be converted to a Unitree policy dir. Zero-command replay report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_zero_command_replay.json` keeps it rejected as deploy evidence: `max_processed_target_gap_l2=0.13366`, `zero_command_target_is_default=false`.
- Deploy98 300-iteration GPU pilot `logs/rsl_rl/g1_velocity_deploy98_candidates/2026-05-22_19-45-22_deploy98_v1_300iter_20260522/` proves real training can run on RTX 4090 and export/package the deploy98 ONNX. It is still rejected before GUI: final TensorBoard scalars at step 299 show `Episode/length_seconds=2.692022`, `Episode_Termination/fell_over=29.083334`, `Metrics/twist/error_vel_xy=0.239295`, and zero-command replay report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_zero_command_replay.json` shows `max_processed_target_gap_l2=0.618108`, `zero_command_target_is_default=false`.
- `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst` is now registered as a separate deploy98 training task. It preserves the 98-dim active Unitree runtime actor contract but makes training zero-command standing-first: all sampled velocity ranges are zero, `rel_standing_envs=1.0`, push disturbance and command curriculum are disabled, the training horizon is 5 seconds, and alive/termination shaping is added. This is a zero-command stability remediation entry, not a final nonzero-velocity tracking acceptance.
- StandFirst 300-iteration candidate `logs/rsl_rl/g1_velocity_deploy98_standfirst_candidates/2026-05-22_20-06-38_standfirst_v1_300iter_20260522/` reaches the 5-second training gate: final TensorBoard scalars at step 299 show `Episode/length_seconds=5.0`, `Episode_Termination/fell_over=0.0`, `Metrics/twist/error_vel_xy=0.046835`, `Metrics/twist/error_vel_yaw=0.159242`, and `Episode_Reward/total=30.056957`. Package report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package.json` is compatible and writes a Unitree policy directory. Zero-command replay report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_zero_command_replay.json` shows `max_processed_target_gap_l2=0.595646`, `zero_command_target_is_default=false`; this means the policy is not a default-pose hold policy, but replay alone is not a physics stability verdict. The next gate must be a Velocity-only Unitree MuJoCo smoke with this exact packaged policy and no Mimic trigger.
- The single-action sim2sim wrappers now accept `MJLAB_VELOCITY_POLICY_ROOT`, so a Velocity-only GUI smoke can use an explicit packaged policy directory instead of implicitly resolving the machine-local active `config/policy/velocity/v0`.
- StandFirst Velocity-only GUI smoke evidence is `logs/flying_kick_sim2sim/20260522-202752/`, with screenshot `logs/flying_kick_sim2sim/20260522-202752/mujoco_standfirst_velocity_run_fall.png` and report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_standfirst_bootstrap_report.json`. It confirms `Velocity.policy_dir` resolves to the StandFirst packaged policy directory, selected initial joints match deploy default/action offset (`gap_l2=0.0`, `gap_max=0.0`), root z is HOME (`0.783675`), elastic band is disabled, and initial foot clearance passes. The run still fails in `Velocity` before any Mimic trigger: first unstable sample at line 29 / `20:27:56.728`, after only 1.5s stable, with `policy_step=75`, `command_norm=0.0`, `raw_action_l2=19.253`, `processed_action_l2=7.182`, `joint_vel_l2=328.134`, `q_err_l2=4.855`, and `root_ang_vel_l2=10.997`. This rejects the StandFirst candidate for sim2sim and keeps hardware locked.
- StandFirst policy sensitivity report `logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_standfirst_policy_sensitivity_zero_command_terms.json` probes the exact packaged ONNX/deploy YAML without launching GUI. With zero-command-only terms (`velocity_commands=0`, `gait_phase=0`) and runtime-scale perturbations, the highest sensitivity term is `joint_vel_rel`: a single-axis `100.0 rad/s` perturbation raises processed target gap from the warmup baseline `0.603384` to `3.0931`, with `max_raw_action_l2=7.63456`. This supports a closed-loop diagnosis where the current policy is vulnerable to joint-velocity/attitude transients after small non-default targets, rather than a wrong policy path or FixStand failure.
- `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped` is now registered as the next zero-command deploy98 training entry. It keeps the same active 98-dim Unitree runtime actor contract and StandFirst zero-command setup, but increases pose/body-angular/action-rate costs and adds action-acceleration plus joint-velocity penalties to target the sensitivity report's `joint_vel_rel -> action` amplification loop. It is a training candidate only; no Mimic or hardware gate is unlocked.
- Damped StandFirst 300-iteration GPU pilot `logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-01-53_standfirst_damped_v1_300iter_20260522/` is rejected before package generation or GUI. Final TensorBoard scalars at step 299 show `Episode/length_seconds=4.888494`, `Episode_Termination/fell_over=2.166667`, `Episode_Termination/time_out=16.791668`, `Episode_Reward/total=14.423843`, `Episode_Metrics/mean_action_acc=0.301305`, `Metrics/slip_velocity_mean=0.038145`, `Metrics/twist/error_vel_xy=0.106901`, and `Metrics/twist/error_vel_yaw=0.472875`. This is much closer than the first deploy98 pilot, but it still fails the strict 5-second / zero-fall gate, so no package, Velocity-only MuJoCo smoke, Mimic trigger, or real-robot work is unlocked.
- Resuming that same damped pilot for 300 more iterations produces a better candidate at `logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-12-09_standfirst_damped_v1_resume300_from299_20260522/`. Final TensorBoard scalars at step 598 pass the strict training gate: `Episode/length_seconds=5.0`, `Episode_Termination/fell_over=0.0`, `Episode_Reward/total=38.259392`, `Episode_Metrics/mean_action_acc=0.065160`, `Metrics/slip_velocity_mean=0.003066`, `Metrics/twist/error_vel_xy=0.026822`, and `Metrics/twist/error_vel_yaw=0.070388`. Package report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package.json` is compatible and writes a Unitree policy directory. Zero-command replay improves to `max_processed_target_gap_l2=0.385498`, still with `zero_command_target_is_default=false`. Sensitivity reports keep the real-robot gate locked: the zero-command-focused stress report still finds `joint_vel_rel` can amplify large perturbations, and the default-magnitude report shows nonzero `gait_phase` / `velocity_commands` would produce large actions if the runtime command/phase masking is wrong. This unlocks only the next Velocity-only Unitree MuJoCo smoke with this exact package; it does not unlock Mimic or hardware.
- Damped continuation Velocity-only GUI smoke evidence is `logs/flying_kick_sim2sim/20260522-212636/`, with screenshot `logs/flying_kick_sim2sim/20260522-212636/mujoco_damped_resume300_velocity_run_fall.png` and report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_bootstrap_report.json`. It confirms the controller reached `FSM: Start Velocity`, used the explicit damped package path, selected initial joints matched deploy default/action offset (`gap_l2=0.0`, `gap_max=0.0`), root z was HOME (`0.783675`), elastic band was disabled, and initial foot clearance passed. The run still fails in `Velocity` before any Mimic trigger: first unstable sample at line 30 / `21:26:40.990`, after 2.0s stable, with `policy_step=100`, `command_norm=0.0`, `raw_action_l2=38.483`, `processed_action_l2=15.654`, `joint_vel_l2=269.793`, `q_err_l2=13.591`, and `root_ang_vel_l2=19.108`. This rejects the damped continuation package for sim2sim and keeps hardware locked.
- Passive-to-Velocity GUI smoke evidence is `logs/flying_kick_sim2sim/20260522-215208/`, with screenshot `logs/flying_kick_sim2sim/20260522-215208/mujoco_passive_velocity_damped_resume300_run_fall.png` and report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_passive_bootstrap_report.json`. This run starts in `Passive` and then enters `Velocity` through a sim-only `!A` transition because Unitree FSM transitions read `lowstate->joystick`, not controller stdin. It confirms `FSM: Start Passive` followed by `FSM: Change state from Passive to Velocity`, uses the explicit damped package path, matches deploy default/action offset (`gap_l2=0.0`, `gap_max=0.0`), uses HOME root z `0.783675`, disables elastic band, and passes initial foot clearance (`min_foot_surface_z=0.027211`). The package still fails after MuJoCo physics actually runs: first dynamic unstable sample at line 194 / `2026-05-22 21:53:34.606`, with `policy_step=4175`, `command_norm=0.0`, `raw_action_l2=30.018`, `processed_action_l2=12.254`, `joint_vel_l2=351.815`, `q_err_l2=9.531`, and `root_ang_vel_l2=14.619`. The report's stable-duration field includes paused policy-thread samples with `joint_vel_l2=0`, so it is not accepted as dynamic standing time. This rejects the damped continuation package under a more deploy-faithful `Passive` to `Velocity` entry and keeps Mimic/hardware locked.
- Velocity actuator-force contract report is `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_contract_report.json`. It proves the user-provided `g1_new.xml`, worktree `src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml`, and external Unitree runtime `g1.xml` are semantically equivalent for the robot body/joint/inertial/geometric rows, so the current blocker is not old-G1 or missing local XML replacement. The same report finds a concrete dynamics mismatch in external `scene_g1.xml`: mjlab training expects 50Nm effort limits for `right_ankle_pitch_joint`, `right_ankle_roll_joint`, `waist_roll_joint`, and `waist_pitch_joint`, while Unitree MuJoCo scene motor `ctrlrange` is only `[-25, 25]` for those joints. All other actuator force limits match. The recommended next gate is a reversible local-runtime scene ctrlrange alignment followed by the same `Passive -> Velocity` smoke; real robot remains locked.
- Actuator-aligned Passive-to-Velocity GUI smoke evidence is `logs/flying_kick_sim2sim/20260522-222217/`, with screenshot `logs/flying_kick_sim2sim/20260522-222217/mujoco_actuator_aligned_passive_velocity_run_fall.png`, pre-smoke match report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_contract_after_local_scene_patch_report.json`, and runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_aligned_passive_bootstrap_report.json`. The local runtime scene was patched reversibly from a backup to align the four mismatched motor `ctrlrange` values to `[-50, 50]`, and the post-patch actuator contract passes with `mismatch_count=0`. The same damped package still fails after MuJoCo physics actually runs: first dynamic unstable sample at line 143 / `2026-05-22 22:23:17.944`, with `policy_step=2900`, `command_norm=0.0`, `raw_action_l2=27.663`, `processed_action_l2=12.382`, `joint_vel_l2=415.247`, `q_err_l2=10.440`, and `root_ang_vel_l2=19.636`. The report's stable-duration field again includes paused policy-thread samples with `joint_vel_l2=0`; dynamic acceptance remains failed. This rejects the actuator-aligned smoke and narrows the blocker beyond scene force limits.
- Actuator-aligned start-unpaused Passive-to-Velocity GUI smoke evidence is `logs/flying_kick_sim2sim/20260522-222840/`, with screenshot `logs/flying_kick_sim2sim/20260522-222840/mujoco_actuator_aligned_passive_unpaused_run_fall.png` and report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_aligned_passive_unpaused_report.json`. This run sets `start_paused=0`, starts in `Passive`, immediately enters `Velocity`, and removes the paused-policy-step artifact from acceptance timing. It still fails almost immediately: first unstable sample at line 28 / `2026-05-22 22:28:43.593`, after only one stable sample (`0.5s`), with `policy_step=25`, `command_norm=0.0`, `raw_action_l2=40.939`, `processed_action_l2=16.796`, `joint_vel_l2=425.455`, `q_err_l2=14.955`, and `root_ang_vel_l2=2.401`. This rules out paused stepping as the sole root cause and keeps the blocker on Velocity closed-loop observation/action dynamics.
- Follow-up source and public-flow review corrected the main sim2sim acceptance route: `FixStand` is a PD pose/interpolation state and a free-standing `FixStand` fall is not a Velocity/tracking failure, but `Passive -> Velocity` direct entry is also a diagnostic shortcut rather than the official G1 operator flow. The wrapper now adds `official_velocity_bootstrap`: start in `FixStand`, use a sim-only auto transition into `Velocity`, keep elastic/key-8 preparation available, and judge stability only after `Velocity` is reached. The previous `velocity_bootstrap` and `passive_velocity_bootstrap` modes remain diagnostic lanes, not the primary official-aligned acceptance route.
- Official-aligned auto-run GUI smoke evidence is `logs/flying_kick_sim2sim/20260522-225945/`, with screenshot `logs/flying_kick_sim2sim/20260522-225945/mujoco_official_velocity_bootstrap_auto_run_after_key9.png`, runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_auto_run_after_key9_report.json`, and policy I/O trace report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_auto_run_policy_io_trace_report.json`. This run proves the official-aligned path now reaches `Velocity`, clicks MuJoCo `Run` after `Velocity` is ready, uses the explicit damped package path, aligns default/action offset (`gap_l2=0.0`, `gap_max=0.0`), uses HOME root z `0.783675`, enables elastic band, passes initial foot clearance, and then receives the explicit key-9 release. It still rejects the package in dynamic `Velocity`: first unstable sample at line 74 / `2026-05-22 23:00:06.361`, `policy_step=950`, `command_norm=0.0`, `raw_action_l2=45.429`, `processed_action_l2=19.583`, `joint_vel_l2=263.947`, `q_err_l2=17.164`, and `root_ang_vel_l2=14.925`. The policy I/O trace exists and matches the 98-dim deploy observation schema, but it does not yet capture a trace near the first dynamic unstable sample (`first_unstable_has_nearby_trace=false`). Real-robot work remains locked.
- Dynamic policy I/O trace v2 evidence is `logs/flying_kick_sim2sim/20260522-231619/`, with screenshot `logs/flying_kick_sim2sim/20260522-231619/mujoco_official_velocity_bootstrap_v2_dynamic_trace_after_key9.png`, runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_after_key9_report.json`, and policy I/O trace report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_policy_io_trace_report.json`. The upgraded local-only deploy trace logs early policy steps plus dynamic samples when joint/root motion or attitude drift appears. The report now passes the hard evidence gate: `first_unstable_has_nearby_trace=true`, `selected_step_delta=0`, selected trace `step=925` / log line 73, first unstable `policy_step=925` / line 74. The selected 98-dim observation has `base_ang_vel=(5.589,-4.183,-9.676)`, `projected_gravity=(0.524,0.684,-0.508)`, zero velocity command, `joint_vel_l2=254.265`, and extreme joint velocities at indices 19 (`-198.109`), 28 (`-133.036`), and 20 (`-47.533`). The same trace reports `raw_action_l2=40.260` with largest raw actions at indices 13 (`-18.189`), 20 (`-15.759`), and 14 (`-14.885`), and `processed_action_l2=15.820` with largest processed targets at indices 13 (`-7.985`), 14 (`-6.535`), and 6 (`-6.436`). This proves the policy output at the failure step is now observable, not inferred. The remaining diagnosis is to compare this deploy observation/action state against the mjlab training/play-side observation construction for the same pose/velocity state. Hardware remains locked.
- Dynamic trace ONNX replay report is `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_replay_report.json`. It replays the selected step-925 deploy observation through the same packaged ONNX and reproduces the C++ deploy log: `replay_matches_deploy_log=true`, `raw_action_gap_l2=0.00000685`, and `processed_action_gap_l2=0.00000328`. This rules out a gross C++ ONNX inference/logging mismatch for the selected failure trace. Counterfactuals on the same captured obs show `last_action` is a major contributor at this already-fallen state: zeroing only `last_action` lowers raw action L2 from `40.260` to `15.409`, while zeroing only `joint_vel_rel` lowers it to `35.554`, and resetting base angular velocity plus joint velocity plus upright gravity still leaves raw action L2 `34.658`. The next diagnosis should therefore compare how deploy and mjlab training/play populate `last_action`, `joint_vel_rel`, phase, and projected gravity during the transition into the first few dynamic frames, not just at the already-unstable sample.
- Dynamic trace chronology report is `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_chronology_report.json`. It confirms the early consecutive deploy trace obeys the expected previous-raw-action lag: `last_action(obs_2..obs_5)` exactly matches `raw_action(1..4)` with max L2 gap `0.0`. It also confirms zero-command observations stayed zero and zero-command `gait_phase` was masked to zero across all 216 traces. Source audit of worktree mjlab and external Unitree deploy code confirms equivalent policy-call semantics: deploy computes obs before processing the new action, while mjlab `env.step(action_t)` returns an obs containing `action_t`, so the next policy call sees the previous raw action in both paths. The first logged dynamic threshold crossing is still step 925, the same policy step as the first unstable sample; this rules out a gross early `last_action`/command/phase chronology bug but leaves a sparse evidence gap between stable step 50 and dynamic failure step 925. The next diagnosis should capture denser transition evidence around MuJoCo Run/key-9 release and the first nonzero lowstate/sim motion update.
- Dense dynamic-onset trace v3 evidence is `logs/flying_kick_sim2sim/20260523-000838/`, with screenshot `logs/flying_kick_sim2sim/20260523-000838/mujoco_official_velocity_bootstrap_v3_dense_auto_key9_with_helper_log.png`, helper log `logs/flying_kick_sim2sim/20260523-000838/bootstrap_helper.log`, runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_auto_key9_helper_report.json`, policy I/O report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_helper_policy_io_trace_report.json`, and chronology report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_helper_trace_chronology_report.json`. The helper log records the official-aligned sequence: MuJoCo key `8` before Run, key `9` release after a 1.0s delay, and Run requested only after `Velocity` was ready. The failure is now unambiguously in zero-command `Velocity`, not `FixStand` or Mimic tracking: first unstable sample is policy step `925`, line `83`, timestamp `2026-05-23 00:09:00.045`, with `command_norm=0.0`, `raw_action_l2=46.018`, `processed_action_l2=18.470`, `joint_vel_l2=411.508`, `q_err_l2=16.660`, and `root_ang_vel_l2=7.793`. The dense trace captures the transition before failure: first low dynamic onset appears at step `916` with `joint_vel_rel_l2=172.637`, then step `917` jumps to `joint_vel_rel_l2=678.490` and `root_ang_vel_l2=19.073`, while velocity command and gait phase remain zero. Chronology again passes previous-raw-action, zero-command, zero-phase-mask, and source-contract checks. Real-robot work remains locked.
- Dense onset ordering report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_onset_order_report.json` classifies the first dynamic onset as `observed_motion_before_large_previous_action`. At step `916`, the observation already has `joint_vel_rel_l2=172.637` and `root_ang_vel_l2=0.366`, but the previous action term is still quiet (`last_action_l2=1.002`). The current raw action becomes large in response to that observed motion (`raw_action_l2=11.219`), and the first large `last_action` appears only one policy call later at step `917`. This makes a previous-policy-action jump unlikely as the cause of the first observed motion; the next target is the physical/deploy side before step `916`: contact impulse, key-9/elastic release impulse, lowstate timing, or simulator/controller state handoff.
- No-key9 control evidence is `logs/flying_kick_sim2sim/20260523-002911/`, with screenshot `logs/flying_kick_sim2sim/20260523-002911/mujoco_official_velocity_bootstrap_v3_dense_no_key9.png`, helper log `logs/flying_kick_sim2sim/20260523-002911/bootstrap_helper.log`, runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_report.json`, policy I/O report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_policy_io_trace_report.json`, and dense onset ordering report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_onset_order_report.json`. This run keeps the same official-aligned `FixStand -> Velocity -> Run` path, the same damped package, the same policy-default/home-root entry, and elastic band enabled with key-8 preparation, but does not send key `9`; helper log records `Release elastic band manually with key 9 only after the policy state is stable.` It still fails in zero-command `Velocity` at policy step `925`, line `83`, with `raw_action_l2=42.277`, `processed_action_l2=16.660`, `joint_vel_l2=311.550`, `q_err_l2=15.879`, and `root_ang_vel_l2=14.334`. The first dynamic onset is again step `916`, with the same quiet previous-action ordering (`last_action_l2=1.002`, `raw_action_l2=11.219`). This falsifies automatic key-9 release as the sole trigger. The remaining physical-side suspects are contact/key-8 pre-tension state, paused-to-run state handoff, DDS/lowstate timing, or a broader MuJoCo/deploy dynamics mismatch.
- No-elastic official-aligned control evidence is `logs/flying_kick_sim2sim/20260523-004041/`, with screenshot `logs/flying_kick_sim2sim/20260523-004041/mujoco_official_velocity_bootstrap_v3_dense_no_elastic.png`, helper log `logs/flying_kick_sim2sim/20260523-004041/bootstrap_helper.log`, runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_report.json`, policy I/O report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_policy_io_trace_report.json`, and dense onset ordering report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_onset_order_report.json`. It keeps `FixStand -> Velocity -> Run`, the same damped package, policy-default joints, and HOME root, but sets `enable_elastic_band=0`. It still fails in zero-command `Velocity`, now at policy step `850`, line `81`, after `19.5s` of stable samples, with `raw_action_l2=43.056`, `processed_action_l2=17.724`, `joint_vel_l2=367.571`, `q_err_l2=14.467`, and `root_ang_vel_l2=16.107`. The first dynamic onset appears at step `840` with quiet previous action and quiet current raw action (`last_action_l2=1.002`, `raw_action_l2=0.831`, `joint_vel_rel_l2=11.949`, `root_ang_vel_l2=0.173`), then the policy response grows at step `841`, and large previous action appears at step `842`. This falsifies both key-9 release and enabled elastic band as sufficient explanations, and points back to physical/deploy-side motion onset before a large policy response.
- Repo-side MuJoCo transition trace patch tooling now exists in `src/mjlab/scripts/g1_tracking_phase1_mujoco_transition_trace_patch.py`. The dry-run against the real local Unitree MuJoCo `simulate/src/main.cc` reports `changed=true` and `write=false`, so the next work item can apply it as a reversible `.external` runtime patch and rebuild. No external source, generated log, or rebuilt binary was committed by this tooling-only slice.
- MuJoCo-side transition trace evidence is `logs/flying_kick_sim2sim/20260523-010629/`, with screenshot `logs/flying_kick_sim2sim/20260523-010629/mujoco_official_velocity_bootstrap_mujoco_trace_no_elastic.png`, runtime report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_report.json`, policy I/O report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_policy_io_trace_report.json`, dense onset report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_onset_order_report.json`, and MuJoCo transition report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_mujoco_transition_trace_report.json`. This run applies the local `phase1_mujoco_transition_trace_v1` patch and rebuilds `unitree_mujoco`. It still fails in zero-command `Velocity`, but the MuJoCo trace now localizes the earliest physical-side event: first physics step after Run is already dynamic (`step=1`, `sim_time=0.002`, `qvel_l2=11.950`, `root_ang_vel_l2=0.173`) while there is no contact (`ncon=0`), no elastic force (`elastic_config=0`, `elastic_force_l2=0.0`), and no large ctrl yet (`ctrl_l2=17.644`). First large ctrl appears only at sim step `5` (`ctrl_l2=199.836`), and first contact only at sim step `8` (`ncon=2`). This narrows the blocker away from contact impulse, key-9 release, enabled elastic, or a later large policy response as sufficient causes, and toward paused-to-run handoff, initial floating/support state, or low-level command state at the instant Run begins.
- Run handoff audit report `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_run_handoff_audit.json` combines the latest runtime, policy I/O, MuJoCo transition, and bridge-source evidence. It classifies the run as `paused_policy_with_floating_support_handoff`: `start_paused=true`, `policy_steps_while_physics_paused=true`, `support_gap_before_run=true`, `first_physics_step_is_dynamic=true`, `first_step_no_contact=true`, and `first_step_nonzero_ctrl=true`. The policy thread advanced to frozen step `50` while joint velocity and root angular velocity stayed zero, initial lowest foot surface was still `0.027211m` above the floor, and the first physics step had `ctrl_l2=17.644` with no contact. Therefore this no-elastic paused evidence is not accepted as clean Velocity policy-quality evidence; the next gate must prove contact/settle and align controller start with MuJoCo Run.
- Velocity policy-start gate evidence is `logs/flying_kick_sim2sim/20260523-015505/`, with reports under `logs/g1_tracking_phase1/2026-05-23T01-55-05+08-00/`. The local `State_RLBase.h` patch was applied and rebuilt into `g1_ctrl`, and the controller log proves `MJLAB_PHASE1_POLICY_START_GATE_SECONDS=5.0` delayed policy stepping after `Velocity` entry: `policy_start_gate` starts at `01:55:08.588`, `policy_start_gate_release` occurs at `01:55:13.591`, and `policy_step=0` is logged before release. This prevents the earlier pre-release policy advance, but it is not sufficient acceptance evidence. After release, policy traces still advance on effectively frozen/slow MuJoCo/DDS state through step `50`; the run later fails at policy step `675` after `13.5s` of stable samples with `command_norm=0.0`, `q_err_l2=4.650`, `joint_vel_l2=327.436`, and `root_ang_vel_l2=10.668`. The handoff audit classifies the run as `paused_policy_handoff`, with `policy_steps_while_physics_paused=true`, `first_step_no_contact=false`, and `support_gap_before_run=false`. Therefore the active blocker is no longer simply "entered Velocity before Run"; it is policy-clock versus MuJoCo/DDS update-clock synchronization during and immediately after the GUI Run boundary.
- Lowstate tick gate evidence is `logs/flying_kick_sim2sim/20260523-023332/`, with reports under `logs/g1_tracking_phase1/2026-05-23T02-33-32+08-00/`. The local `State_RLBase.h` tick gate patch was applied and rebuilt into `g1_ctrl`. The controller log proves `LowState.tick()` remains stale after the 5s wall-clock release and `policy_step` stays at `0` until the first fresh tick at `02:33:53.303`. After fresh ticks arrive, cadence is approximately 50Hz and the previous post-wait catch-up burst is gone. The run still rejects Velocity stability: first unstable sample is line `91` / policy step `25`, zero command, `q_err_l2=4.773`, `joint_vel_l2=372.938`, and `root_ang_vel_l2=8.406`. The handoff audit now classifies `lowcmd_ctrl_handoff`, so the next target is early `LowCmd` to MuJoCo `ctrl`.
- Lowcmd ctrl decomposition evidence is `logs/flying_kick_sim2sim/20260523-025122/`, with reports under `logs/g1_tracking_phase1/2026-05-23T02-51-22+08-00/`. The local `unitree_sdk2_bridge.h` trace patch was applied and `unitree_mujoco` rebuilt. The lowcmd report classifies the first nonzero control as `position_pd_ctrl_handoff`: at sample `2` / sim time `0.002`, `ctrl_l2=47.307024`, `tau_l2=0.0`, `pos_term_l2=47.307024`, and `vel_term_l2=0.0`. The dominant joint is index `3`, where `q_cmd=0.0`, `q_sensor=0.3`, and `q_error=-0.3`, producing `top_ctrl=-29.73`. First large control later shifts toward velocity-PD error at sample `5`. This proves the earliest nonzero control is generated by lowcmd target/current mismatch at the bridge, not by commanded torque, contact impulse alone, frozen policy stepping, or a large policy action. Hardware remains locked.
- Baseline reset audit on `2026-05-23` shows the current machine-local Unitree checkout cannot be treated as a clean official baseline. The external tree is not an independent git checkout for status/revert purposes (`git -C .external/unitree_rl_mjlab rev-parse --show-toplevel` resolves to the parent mjlab repo), and it contains active phase-1 patches in `deploy/include/FSM/State_RLBase.h`, `deploy/include/isaaclab/envs/manager_based_rl_env.h`, `deploy/include/isaaclab/envs/mdp/actions/joint_actions.h`, `deploy/robots/g1/src/State_RLBase.cpp`, `deploy/robots/g1/src/State_Mimic.cpp`, `deploy/include/FSM/State_FixStand.h`, `simulate/src/main.cc`, and `simulate/src/unitree_sdk2_bridge.h`. The available phase-1 backups cover most but not all of those files. Current `simulate/config.yaml` is also wrapper-polluted: it includes a custom 36-value `initial_qpos`, `start_paused: 1`, `use_joystick: 0`, and `enable_elastic_band: 0`, while upstream Unitree `simulate/config.yaml` has no `initial_qpos` or `start_paused` and keeps `use_joystick: 1`. Upstream Unitree `deploy/robots/g1/config/config.yaml` also differs from the active local config: it enables only Passive/FixStand/Velocity/Dance1 and FixStand targets the Velocity default/action-offset pose. This confirms the next slice must create a clean official baseline sandbox or reversible restore set before running more GUI evidence.
- Official clone audit on `2026-05-23` uses a clean upstream clone at `/tmp/unitree_rl_mjlab_official_baseline`, HEAD `1425b15`. The active local Velocity `policy.onnx` and `params/deploy.yaml` are byte-identical to upstream, so the first reset target is not the policy file. The behavior-changing drift is in runtime/config: local `simulate/src/param.h` and `simulate/src/main.cc` add `initial_qpos` and `start_paused`; local `deploy/include/FSM/State_FixStand.h` changes FixStand to read per-state config and initializes from sensed joint position instead of the previous lowcmd target; local `deploy/include/isaaclab/envs/mdp/actions/joint_actions.h` changes reset semantics by immediately processing zero raw actions into default/offset targets; local `simulate/config.yaml` and G1 `config/config.yaml` are wrapper-modified. Most other external source deltas are trace/logging helpers, but they still prove the current binary/source tree is not a clean official sim2sim baseline. Therefore the next executable baseline must run from a separate clean clone or from an explicit copy/overlay restore set, not from the already-patched `.external` tree.
- Clean upstream build preflight on `2026-05-23` builds both official clone executables without touching `.external/unitree_rl_mjlab`: `unitree_mujoco` builds after passing `-DCMAKE_PREFIX_PATH=/home/ssy/ssy_files/mjlab/.external/unitree_sdk2/install`, and `g1_ctrl` builds after passing compile/link flags to the same SDK install. The official G1 controller source itself is unchanged. Launch is blocked as a clean baseline in the current environment because upstream README says a gamepad must be connected, upstream `simulate/config.yaml` keeps `use_joystick: 1` and `joystick_device: "/dev/input/js0"`, `/dev/input` does not exist in this session, and `/tmp/unitree_rl_mjlab_official_baseline/simulate/build/jstest` returns `open failed.`. Changing `use_joystick` to `0` would be an automation deviation, not a clean official baseline.
- Repo-side official-baseline preflight tooling now exists at `scripts/tools/g1_tracking_phase1_official_baseline_preflight.py`. It is non-launching and writes a JSON report for the clean upstream clone, expected binaries, upstream joystick config, `/dev/input/js0` status, current user groups, optional `jstest`, and the explicit deviation policy. Fresh real-machine preflight evidence is `logs/g1_tracking_phase1/2026-05-23T-official-baseline-preflight/official_baseline_preflight.json`; it reports `status=blocked`, `official_head=1425b15`, `use_joystick=1`, `has_initial_qpos=false`, `has_start_paused=false`, all three binaries executable, `missing_joystick_device`, `jstest_failed`, and `in_input_group=false`. The clean-baseline route remains blocked until a real joystick device is exposed.

## Non-goals

- No new training or long learning jobs in this phase.
- No mutation of generic `Mjlab-Tracking-Flat-Unitree-G1` defaults.
- No real-robot trial until sim2sim2 passes on the verified new-G1 contract.
- No acceptance based only on mjlab `play`, Viser, kick height, or visual style.
- No generated logs, videos, checkpoints, or `.external` runtime content committed by default.

## Success Criteria

- A baseline manifest records both selected bundles, source checkpoints, ONNX files, motion files, deploy YAMLs, active FSM config, user-provided G1 model files, active mjlab assets, active external sim/deploy assets, and current git status.
- A G1 contract validator proves 29 joints, ordered joint names, ordered actuator names, action dimension, motion joint dimension, ONNX input/output shape, scene XML, default pose, action scale, and deploy gains are consistent with the user's new G1 29DoF mode 15 and do not use old G1 or `g1_23dof`.
- Unitree MuJoCo sim2sim2 can launch both selected bundles through the deploy-style controller path and produce logs plus video or screenshot evidence.
- For both actions, sim2sim2 evidence shows trigger/FSM/motion/policy/action timing and a stable return to `Velocity` or another approved stable state.
- First-version acceptance requires each action to complete and remain stable for 5 seconds after action end.

## Verification Path

Verification path status: `runnable`.

The repo-local validation and artifact-manifest work is runnable through `uv run`. Full sim2sim2 depends on local `.external/unitree_rl_mjlab` GUI/DDS/controller availability, which exists on disk but is machine-local runtime content outside this worktree. Any write or rebuild inside `.external` is a local runtime action, not a source commit.

Required capabilities:

- Project Python tooling via `uv run`.
- YAML/XML/NPZ/ONNX parsing for bundle and robot-contract validation.
- Local Unitree MuJoCo simulator and G1 controller under `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab`.
- Log and video/screenshot capture for sim2sim2.
- Human/operator approval for any later tethered real-robot trial.

Fallback evidence if full sim2sim2 is unavailable:

- Accepted only as a blocker report, not as deploy acceptance.
- The fallback package must include bundle manifest, G1 contract validation output, failed launch command, simulator/controller logs, and the exact missing GUI/DDS/dependency capability.

Final integration claim:

- After all implementation slices pass, the repository provides a reproducible phase-1 gate proving whether the current flying-kick and roundhouse deploy bundles pass or fail the new-G1 Unitree MuJoCo sim2sim2 stability/timing contract, with evidence sufficient to decide whether phase 2 should change training, domain randomization, timing robustness, motion entry, or handoff behavior.

## Work Items

Stage gate rule:

- From this amendment onward, each new work item maps to one commit exactly. Do not combine adjacent work items into one commit, and do not split a work item across multiple behavioral commits unless the plan is updated first.
- Completed work and the post-blocker diagnosis commits are mapped explicitly below so the recovery surface matches the real git history.
- A commit is not ready until its mandatory tests pass and its acceptance gate is satisfied with fresh evidence.
- Do not start the next work item until the previous acceptance gate passes. If a gate fails, stop, record the blocker, and route to `diagnose` instead of continuing.
- Generated logs, videos, checkpoints, and `.external` runtime edits stay out of git unless the user explicitly requests otherwise. Committed evidence should be summaries, validators, scripts, tests, or docs that point to local artifact paths.
- Before each commit, record fresh `git status --short`; after each commit-sized change, run the required tests from that work item and check that only intended source/docs/tests changed.

### 1. Baseline Evidence And Provenance - done

Commit:

- `feat(g1): add phase1 baseline manifest`

Status:

- Done in `2367496a`.

Actions:

- Add a repo-local baseline manifest command or script that collects both actions in one artifact directory under `logs/` or `artifacts/`.
- Capture hashes and paths for flying-kick ONNX, roundhouse ONNX, source run directories, motion `.npz` files, deploy YAMLs, active FSM config, sim config, and user-supplied G1 URDF/XML.
- Include `git status --short`, selected script paths, and symptom-video metadata for `/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/video/2026-05/05e60f0aa3f07a32611272b5beaa9d3a.mp4`.

Mandatory tests:

- `uv run pytest tests/tools/test_g1_tracking_phase1_manifest.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_manifest.py --dry-run`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_manifest.py --output-root logs/g1_tracking_phase1`

Acceptance gate:

- The manifest command creates one timestamped evidence directory and one machine-readable manifest.
- The manifest names both actions, selected policy paths, source run dirs, motion paths, deploy YAMLs, active config paths, user G1 URDF/XML paths, hashes, and symptom-video metadata.
- A missing ONNX, motion, deploy YAML, or config produces a non-zero pre-launch failure.
- No simulator, controller, or real-robot command is launched by this work item.

Advance rule:

- Start work item 2 only after the manifest exists and the tests above pass.

### 2. New-G1 Mode 15 Contract Validator - done

Commit:

- `feat(g1): add phase1 new-g1 contract validator`

Status:

- Done in `87582be5`.

Actions:

- Add a focused validator for the user's new G1 29DoF mode 15 contract across:
  - `src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf`
  - `src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml`
  - `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml`
  - `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/scene_g1.xml`
- Fail on old G1, `g1_23dof`, wrong joint count, wrong ordered joint names, wrong actuator order, wrong scene include, action dimension mismatch, motion dimension mismatch, ONNX shape mismatch, or missing deploy gains.
- Keep byte-for-byte XML equality out of the hard gate; use structural/control-contract equality.

Mandatory tests:

- `uv run pytest tests/tools/test_g1_tracking_phase1_contract.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_contract.py --manifest <latest-phase1-manifest>`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_contract.py --manifest <latest-phase1-manifest> --forbid-g1-23dof`

Acceptance gate:

- Validator passes on the current intended new-G1 assets.
- Negative fixtures or explicit wrong-file inputs fail on `g1_23dof`, wrong scene XML, wrong joint count, wrong actuator order, or ONNX/action dimension mismatch.
- The output includes ordered joint names, ordered actuator names, DOF count, motion joint dimension, ONNX shape, scene include path, default pose source, action scale source, and deploy gain source.
- Byte-for-byte XML differences caused only by comments or line endings do not fail the gate.

Advance rule:

- Start work item 3 only after the current active sim/deploy/mjlab assets pass the new-G1 contract validator.

### 3. Sim2sim2 Wrapper Hardening - done

Commit:

- `feat(g1): add phase1 dual-action sim2sim2 preflight wrapper`

Status:

- Done in `71e48c14`.

Actions:

- Extend or wrap the existing `run_flying_kick_sim2sim.sh` and `run_roundhouse_leading_right_sim2sim.sh` flow so both actions run under one phase-1 gate.
- Preflight must run baseline manifest and G1 contract validation before launching Unitree MuJoCo.
- Preserve the existing single-action scripts as debugging entry points.
- Ensure sim config uses loopback/simulation-safe settings and cannot accidentally target the real robot interface/domain.
- Record simulator logs, controller logs, selected config snapshots, and video/screenshot evidence into the timestamped evidence directory.

Mandatory tests:

- `bash -n scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `uv run pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh status`

Acceptance gate:

- Preflight refuses to run unless work item 1 manifest and work item 2 contract validation are present and current.
- Preflight proves the selected DDS/domain/interface settings are simulation-safe and do not target a real robot interface.
- `status`, `start`, `stop`, and `restore` paths are defined and recoverable.
- Existing single-action scripts remain usable as debugging entry points.
- No GUI launch is required for preflight tests.

Advance rule:

- Start work item 4 only after the dual-action preflight wrapper can prove it will launch the verified bundles and new-G1 assets, without touching hardware.

### 4. Timing And Stability Instrumentation - done

Commit:

- `feat(g1): add phase1 timing and stability evidence parser`

Status:

- Done in `345c52a2`.

Actions:

- Define one minimal timing log schema for trigger time, FSM transition, mimic motion time/frame, policy inference step, processed action, low-level joint target write, q/dq/action error, base velocity, commanded velocity if available, and projected gravity.
- First consume existing controller logs where possible.
- If existing logs are insufficient, add a clearly scoped local-runtime patch path for `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_Mimic.cpp` and `State_RLBase.cpp`; treat this as a machine-local deploy instrumentation step, not a repo source commit.
- Add a parser that extracts post-action 5-second stability windows and timing offsets from logs.

Mandatory tests:

- `uv run pytest tests/tools/test_g1_tracking_phase1_log_parser.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --fixtures tests/fixtures/g1_phase1/pass`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --fixtures tests/fixtures/g1_phase1/fail --expect-failure`

Acceptance gate:

- Parser labels pass/fail for action completion, return state, and 5-second post-action stability.
- Parser reports timing offsets for trigger-to-FSM, FSM-to-motion, motion-to-policy, policy-to-lowcmd, and lowcmd-to-q-response when fields exist.
- Missing required timing fields produce a blocking "insufficient evidence" result, not a pass.
- Any `.external` instrumentation need is documented as a local-runtime patch/rebuild step and is not silently committed as repo source.

Advance rule:

- Start work item 5 only after sim2sim2 logs can be parsed into pass/fail or explicit insufficient-evidence outcomes.

### 5. Sim2sim2 Evidence Run For Both Actions - done, blocker found

Commit:

- `docs(g1): record phase1 sim2sim2 evidence classification`

Status:

- Done in `df63e4d2`.
- Acceptance failed as designed: both actions produced evidence, but neither passed the 5-second post-action stability gate.

Actions:

- Run flying kick and roundhouse through Unitree MuJoCo using the verified new-G1 assets and selected deploy bundles.
- Trigger from the approved entry state, return to `Velocity` or approved stable state, and hold for 5 seconds.
- Package logs, screenshots/video, config snapshots, manifests, and pass/fail summaries.

Mandatory tests:

- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --manifest <latest-phase1-manifest>`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir <latest-sim2sim2-evidence-dir>`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop`

Acceptance gate:

- Both actions produce simulator logs, controller logs, config snapshots, manifest references, and video or screenshot evidence.
- Each action either passes the first-version gate or fails with exactly one primary classified reason:
  - G1 contract mismatch;
  - simulator/deploy startup instability;
  - command/FSM timing mismatch;
  - policy/action-to-joint response mismatch;
  - Velocity response issue;
  - post-motion handoff/standing failure;
  - insufficient timing evidence.
- To pass, each action must complete, return to `Velocity` or an approved stable state, and remain stable for 5 seconds after action end.
- Generated heavy evidence remains under `logs/` or `artifacts/`; the commit records only source/docs summaries and evidence paths.

Advance rule:

- Original work item 6 is locked because both actions failed sim2sim2.
- Continue through the explicit post-blocker diagnosis items below, then enter the new entry-state handoff gate.

### 5a. Direct Mimic Sim2sim Diagnosis Mode - done

Commit:

- `fix(g1): expose direct mimic sim2sim diagnosis mode`

Status:

- Done in `e8f8de57`.

Actions:

- Expose a simulation-only direct-Mimic entry path to remove unrelated Velocity/FSM launch uncertainty during diagnosis.
- Preserve the distinction between diagnostic sim entry and deploy-ready hardware entry.

Mandatory tests:

- `bash -n scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- Focused wrapper/parser tests touched by the change.

Acceptance gate:

- Direct Mimic can be requested only as a diagnostic mode.
- Direct Mimic evidence cannot unlock real-robot trials by itself.

### 5b. In-Action Instability Classification - done

Commit:

- `fix(g1): classify in-action sim2sim instability`

Status:

- Done in `9c869c5f`.

Actions:

- Tighten the parser classification so mid-action projected-gravity failure is not mislabeled as a post-handoff standing failure.
- Keep the primary reason tied to the earliest concrete evidence.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_log_parser.py`
- Real evidence parser rerun for both sim2sim2 directories with `--expect-failure`.

Acceptance gate:

- Flying kick and roundhouse are classified consistently as in-action instability before any stable return-state claim.
- The report preserves the timing and log-line evidence needed for follow-up diagnosis.

### 5c. Entry-Pose Gap Diagnosis - done

Commit:

- `diagnose(g1): add phase1 entry-pose gap checker`

Status:

- Done in `18ed1bea`.

Actions:

- Compare the deploy/default entry joint pose against each motion's frame-0 and best-aligned early-frame pose.
- Cross-check the computed gap against the first logged sim2sim2 `q_err`.
- Record the result as a separate root-cause layer above the in-action instability symptom.

Mandatory tests:

- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_entry_gap.py scripts/tools/g1_tracking_phase1_entry_gap.py tests/tools/test_g1_tracking_phase1_entry_gap.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_entry_gap.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_manifest.py tests/tools/test_g1_tracking_phase1_contract.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_log_parser.py tests/tools/test_g1_tracking_phase1_entry_gap.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_entry_gap.py --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json --log logs/flying_kick_sim2sim/20260522-143349/g1_ctrl.log --log logs/roundhouse_leading_right_sim2sim/20260522-143755/g1_ctrl.log --expect-failure`

Acceptance gate:

- Entry-pose mismatch is reported as `entry_state_pose_mismatch`.
- The report gives per-action frame-0 gap, best early-frame gap, first logged `q_err`, and whether simple `time_start` can plausibly solve the issue.
- Real-robot gate remains locked.

### 6. Entry-State Handoff Gate - done, prepose candidate available

Commit:

- `feat(g1): add phase1 entry-state handoff gate`

Status:

- Implemented in this work item.
- Result is blocked for direct acceptance but provides the next deploy-style sim2sim2 candidate: `--mode prepose`.

Actions:

- Add a repo-local entry-state handoff checker/planner for both actions.
- Compare three entry contracts:
  - deploy/default standing entry;
  - simulation-only reference-frame initialization;
  - deploy-safe pre-pose or transition entry that can be reached without teleporting state.
- If the current Unitree MuJoCo config uses `initial_qpos`, snapshot and validate it as sim-only evidence unless the same entry can be produced by the controller.
- Add explicit metadata that prevents a teleported sim initialization from being counted as deploy or real-robot acceptance.
- Add the minimal wrapper/config path to run the controller-reproducible prepose candidate in sim2sim2.
- If prepose cannot pass work item 7, produce a blocker that routes phase 2 toward a stronger controller transition, deployment entry-state perturbations, or a deployment-aware motion start.

Mandatory tests:

- `bash -n scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `uv run --active --no-sync ruff check <changed-python-files> <changed-tests>`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_entry_gap.py`
- Add and run focused tests for any new entry-state handoff checker or wrapper behavior.
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_entry_gap.py --manifest logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/manifest.json --log logs/flying_kick_sim2sim/20260522-143349/g1_ctrl.log --log logs/roundhouse_leading_right_sim2sim/20260522-143755/g1_ctrl.log --expect-failure`

Acceptance gate:

- The artifact for each action states whether the tested entry is deploy/default, sim-teleport-only, or deploy-safe pre-pose/transition.
- Teleported initial state is allowed only as a cause-isolation test and must not unlock real-robot work.
- The prepose candidate must be reproducible from controller commands and must pass simulation-safe preflight before work item 7 runs it.
- If no deploy-safe entry path passes in phase 1, the workflow exits with a single blocker reason and a phase-2 training recommendation; it does not continue to hardware.

Advance rule:

- Start work item 7 only if work item 6 provides a non-teleport, deploy-safe entry path or an explicitly accepted blocker package.
- Start work item 8 only if work item 7 passes both actions under the 5-second gate.

### 7. Post-Entry-Fix Sim2sim2 Evidence Run - done, blocker found

Commit:

- `docs(g1): record phase1 post-entry sim2sim2 evidence`

Status:

- Evidence collected for both actions.
- Acceptance failed: neither action entered Mimic, and both actions became unstable during deploy-style `FixStand` prepose.
- Evidence-harness bug found and fixed before final classification: the old auto-run path could leave MuJoCo paused while claiming success.

Actions:

- Re-run flying kick and roundhouse through Unitree MuJoCo using the selected entry-state handoff path from work item 6.
- Package logs, screenshots/video, config snapshots, entry-state artifacts, manifests, and pass/fail summaries.
- Compare new evidence against the original failed evidence directories.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh`
- `bash -n scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`
- `MJLAB_AUTO_RUN_AFTER_READY=1 scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action flying_kick --manifest <latest-phase1-manifest> --mode prepose --start-paused 1`
- `MJLAB_AUTO_RUN_AFTER_READY=1 scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh start --action roundhouse_leading_right --manifest <latest-phase1-manifest> --mode prepose --start-paused 1`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_analyze_logs.py --evidence-dir <latest-post-entry-sim2sim2-evidence-dir>`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_entry_gap.py --manifest <latest-phase1-manifest> --log <latest-flying-kick-log> --log <latest-roundhouse-log>`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh stop`

Acceptance gate:

- Both actions produce logs, config snapshots, manifest references, and video or screenshot evidence under the selected non-teleport entry contract.
- Each action either passes the 5-second gate or fails with exactly one primary classified reason.
- Passing requires action completion, stable return to `Velocity` or an approved stable state, and 5 seconds of stability after action end.
- If the only passing result uses a teleported sim state, classify it as `entry_state_pose_mismatch_requires_deploy_or_training_fix`, not as phase-1 deploy acceptance.

Advance rule:

- Do not start work item 8.
- Route next work to `diagnose` because both prepose runs failed before Mimic entry.

### 7c. Official Elastic-Band Bootstrap Mode - done with follow-up safety fix

Commit:

- `feat(g1): add official elastic sim2sim bootstrap mode`
- `fix(g1): make official elastic bootstrap paused-safe`

Status:

- First implementation committed, but live smoke failed by pulling the robot upward before policy entry. Follow-up safety fix makes the default path paused-only and moves optional key-8 pre-tension before `Run`.

Actions:

- Add a `--mode official_bootstrap` path to the dual-action wrapper.
- Map `official_bootstrap` to the same deploy stand qpos as `stand`, but default the sim runtime to `enable_elastic_band=1`, `start_paused=1`, and `MJLAB_AUTO_RUN_AFTER_READY=0`.
- If the operator explicitly sets `MJLAB_AUTO_RUN_AFTER_READY=1`, send MuJoCo key `8` a configurable number of times before clicking `Run`, while keeping key `9` manual so evidence collection can release only after the policy state is stable.
- The default pre-tension is 24 key-8 steps, matching the local `(0,0,3)` anchor and stand-height geometry more closely than the failed length-zero startup.
- Do not send the MuJoCo `space` toggle before pre-tensioning in `official_bootstrap`; pre-tension must happen before any `Run` request.
- Keep the path clearly classified as sim bootstrap evidence; it must not unlock real-robot work without a subsequent evidence run that passes the 5-second gate.

Mandatory tests:

- `bash -n scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `bash -n scripts/tools/run_flying_kick_sim2sim.sh`
- `bash -n scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`

Acceptance gate:

- The dual-action wrapper accepts `--mode official_bootstrap` for both actions and forwards it to the single-action scripts.
- Both single-action scripts force elastic-band bootstrap defaults for that mode unless explicitly overridden by the operator.
- The generated sim config uses stand qpos, `enable_elastic_band=1`, and `start_paused=1` for official bootstrap.
- The auto-run helper still clicks MuJoCo `Run` only when explicitly enabled, and in official bootstrap mode it sends MuJoCo key `8` before `Run`; key `9` remains manual and documented.
- Paused-only smoke evidence proves the default `official_bootstrap` path does not auto-run or apply elastic force before operator action.
- No `.external` source edit, generated log, video, checkpoint, or hardware command is committed by this work item.

Advance rule:

- Start the next evidence run only after this mode passes syntax, unit, and preflight checks. Do not run default `official_bootstrap` with auto-run unless the operator explicitly accepts that smoke risk.
- The `20260522-165042` auto-run smoke proves the robot no longer flies after pre-tension, but it still falls in `FixStand`. Treat that as evidence that the acceptance gate must move to `Velocity`, not as a policy-quality failure.

### 7f. Velocity-First Sim2sim Bootstrap Gate - done, blocker found

Commit:

- `feat(g1): add velocity-first sim2sim bootstrap mode`

Status:

- Implemented. Paused smoke passed; explicit Run smoke failed in `Velocity`
  before any Mimic trigger.

Actions:

- Add a `--mode velocity_bootstrap` path to the dual-action wrapper and both
  single-action sim2sim scripts.
- Start the controller in `Velocity` from the deploy stand qpos, with
  `enable_elastic_band=1`, `start_paused=1`, and no auto-run by default.
- Keep `Passive` and `FixStand` in the generated FSM config for parity with the
  real Unitree flow, but make this mode explicit diagnostic evidence rather
  than a real-robot unlock by itself.
- Preserve action transitions from `Velocity` to the tracking skills:
  `Mimic_FlyingKick` on `RB + X` and `Mimic_RoundhouseLeadingRight` on `RB + Y`.
- In explicit auto-run mode, wait for `FSM: Start Velocity`, pre-tension the
  elastic band before `Run`, and leave key `9` manual until the Velocity state
  is visibly stable.
- Update docs so future agents do not treat `FixStand` stability as the phase-1
  standing gate.

Mandatory tests:

- `bash -n scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `bash -n scripts/tools/run_flying_kick_sim2sim.sh`
- `bash -n scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`
- A paused-only `velocity_bootstrap` GUI smoke for flying kick that records
  selected config snapshots and proves no controller/simulator process remains
  after restore.

Acceptance gate:

- The wrapper accepts and forwards `--mode velocity_bootstrap`. Passed by
  focused wrapper tests.
- Generated selected config has `FSM.initial_state: Velocity`, stand
  `initial_qpos`, `enable_elastic_band: 1`, `start_paused: 1`, and the correct
  `Velocity` to Mimic transitions for each action. Passed by focused tests and
  `logs/flying_kick_sim2sim/20260522-170654/selected/`.
- Default mode is paused-only and does not click `Run`. Passed by
  `logs/flying_kick_sim2sim/20260522-170654/`.
- Explicit auto-run waits for `Velocity`, not `FixStand`, before pre-tension
  and `Run`. Covered by static wrapper tests and
  `logs/flying_kick_sim2sim/20260522-171001/`.
- The first standing-stability evidence is judged in `Velocity`, not
  `FixStand`. Paused smoke log has 117 `Velocity stable=1` samples; explicit
  Run smoke fails in `Velocity` after roughly 3.5 seconds.
- Real-robot gate remains locked until a later evidence run passes the full
  action completion plus 5-second stable-state gate for both actions.

Advance rule:

- Do not start the next tracking evidence run yet. Explicit Run shows Velocity
  cannot stay stable in Unitree MuJoCo, so route to diagnosis of the Velocity
  policy/deploy/sim contract before any Mimic or real-robot work.

### 7g. Velocity Bootstrap Failure Diagnosis - done, blocker found

Commit:

- `diagnose(g1): isolate velocity bootstrap sim2sim failure`

Status:

- Done. Do not trigger Mimic or run hardware before the Velocity default-pose
  mismatch is remediated or falsified by a policy-default-pose bootstrap.

Actions:

- Compare the active `Velocity` policy deploy bundle against the new G1 mode-15
  contract: policy ONNX shape, `deploy.yaml` joint order, action scale, action
  offset/default pose, stiffness, damping, and selected policy directory
  resolution.
- Compare the `velocity_bootstrap` stand `initial_qpos` against the Velocity
  policy action offset/default joint pose.
- Check whether the Velocity policy was trained/exported for the same new G1
  29DoF mode-15 asset now used by Unitree MuJoCo.
- Add a small report command if needed so the Velocity policy/deploy/sim
  contract can be reproduced without launching GUI.
- Keep the finding separate from the flying-kick/roundhouse Mimic policies:
  current evidence fails before Mimic, so the root blocker is upstream.

Mandatory tests:

- Focused unit tests for any new Velocity contract/report command.
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-171001 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_bootstrap_report.json --expect-failure`

Acceptance gate:

- The diagnosis identifies whether the Velocity failure is caused by asset
  mismatch, default-pose/action-offset mismatch, gain/scale mismatch, missing
  or wrong Velocity policy export, or an unknown runtime-only failure. Current
  result: `velocity_default_pose_mismatch`.
- The report names the exact local evidence path and next remediation route.
  Current route: test a Velocity bootstrap from the policy default/action-offset
  pose, or align/retrain/export Velocity so its deploy offset matches the new
  mode-15 stand qpos.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is a policy-default
  Velocity bootstrap check or a Velocity policy/export remediation.

### 7h. Velocity Policy-Default Bootstrap Check - done, blocker found

Commit:

- `fix(g1): test velocity policy-default bootstrap entry`

Status:

- Done. The policy-default entry removes the initial pose/action-offset gap but
  still falls in `Velocity` before any Mimic trigger.

Actions:

- Add a diagnostic-only option that lets `velocity_bootstrap` initialize joints
  from the resolved Velocity policy `default_joint_pos` / action offset instead
  of the motion/deploy stand qpos.
- Keep root qpos and new-G1 mode-15 scene unchanged.
- Run paused and explicit-Run GUI smoke before any Mimic trigger.
- If policy-default bootstrap passes Velocity stability, the root cause is the
  deploy stand qpos versus Velocity policy offset mismatch.
- If it still falls, continue diagnosis toward Velocity policy asset/training
  mismatch, gains/action scale, or new-G1 asset dynamics.

Mandatory tests:

- Focused wrapper tests for the new diagnostic option.
- `bash -n` for all changed sim2sim shell scripts.
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh preflight --manifest <latest-phase1-manifest>`
- Real evidence command for the new policy-default Velocity smoke:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-172725 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_default_bootstrap_report.json --expect-failure`

Acceptance gate:

- The selected config/sim config clearly states whether the entry qpos came from
  deploy stand or Velocity policy default. Current evidence: `policy_default`.
- The result is classified as pass/fail in Velocity before any Mimic trigger.
  Current result after extended diagnosis: `velocity_initial_contact_mismatch`.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is diagnosis of why
  the Velocity policy-default bootstrap fails with the knees-bent/root-low
  initial height, starting with a grounded root-height test.

### 7i. Velocity Initial Contact Contract Diagnosis - done, blocker found

Commit:

- `diagnose(g1): classify velocity initial contact mismatch`

Status:

- Done. The policy-default joint pose is valid, but the selected root height
  starts that pose with foot collision below the floor.

Actions:

- Extend the Velocity contract report beyond joint pose matching to include:
  ONNX input/output shape and metadata, deploy observation term dimensions,
  current mjlab G1 source init-state root height, and MuJoCo foot contact surface
  height for the selected initial qpos.
- Re-run the report on `logs/flying_kick_sim2sim/20260522-172725/`.
- Keep this as diagnosis only: do not trigger Mimic and do not unlock hardware.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-172725 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_default_bootstrap_report.json --expect-failure`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`

Acceptance gate:

- The report must identify whether the policy-default Velocity failure is still
  generic runtime instability or a more specific initial-contact/root-height
  mismatch. Current result: `velocity_initial_contact_mismatch`.
- The report must prove ONNX/deploy observation dimensions before blaming runtime
  dynamics. Current result: `obs[1,98] -> actions[1,29]`, deploy observations
  total `98`, so observation dimension mismatch is falsified.
- The report must state the selected root z and foot surface clearance. Current
  result: selected root z `0.765781`, lowest foot surface `-0.018422`, required
  root lift `0.018422`.
- Real-robot gate remains locked.

Advance rule:

- The next allowed slice is a grounded policy-default root-height sim2sim smoke.
  If grounded policy-default still falls, continue toward policy asset/training
  mismatch, gains/action scale, or observation timing.

### 7j. Grounded Velocity Policy-Default Root Smoke - done, blocker found

Commit:

- `fix(g1): add grounded velocity policy-default bootstrap`

Status:

- Done. Grounding the policy-default pose removes the initial-contact failure but
  does not stabilize `Velocity`.

Actions:

- Add `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home` for `velocity_bootstrap` with
  `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`.
- Keep the default `velocity_bootstrap` root behavior unchanged.
- Run a home-root Velocity-only GUI smoke without triggering Mimic.
- Classify the resulting evidence with the extended Velocity contract report.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh scripts/tools/run_roundhouse_leading_right_sim2sim.sh scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff format --check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-175113 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_bootstrap_report.json --expect-failure`

Acceptance gate:

- The selected sim config must show `initial_qpos[2] == 0.783675` and
  `initial_qpos[7:]` matching Velocity policy default/action offset.
- The report must show initial-contact clearance passes. Current result:
  lowest foot surface `-0.000527`, `floor_clearance_passed: true`.
- The result must be classified before any Mimic trigger. Current result:
  `velocity_runtime_instability`.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is Velocity policy
  runtime contract diagnosis: compare this `v0` policy/export against current
  source/training runs, action scale/gains, observation values/timing, and the
  new G1 mode-15 dynamics.

### 7k. No-Elastic Velocity Runtime Falsification - done, blocker found

Commit:

- `diagnose(g1): falsify velocity elastic-band cause`

Status:

- Done. Disabling the elastic band does not stabilize `Velocity`.

Actions:

- Run the same grounded policy-default Velocity-only GUI smoke with
  `MJLAB_ENABLE_ELASTIC_BAND=0`.
- Fix the sim2sim wrapper flow text so `official_bootstrap` and
  `velocity_bootstrap` report the actual elastic-band value, not a hardcoded
  "enabled" claim.
- Classify the evidence with the extended Velocity contract report.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh scripts/tools/run_roundhouse_leading_right_sim2sim.sh scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff format --check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-175822 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_no_elastic_bootstrap_report.json --expect-failure`

Acceptance gate:

- The selected sim config must show `enable_elastic_band: 0`,
  `initial_qpos[2] == 0.783675`, and `initial_qpos[7:]` matching Velocity
  policy default/action offset.
- The report must still classify the run before any Mimic trigger. Current
  result: `velocity_runtime_instability`.
- The wrapper output must not claim that the elastic band is enabled when
  `MJLAB_ENABLE_ELASTIC_BAND=0`.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. Since default-pose mismatch, root contact,
  and elastic-band force are all falsified as sufficient causes, the next
  allowed slice is deeper Velocity runtime diagnosis: policy provenance,
  deploy YAML versus training config, observation values/timing, action/gain
  semantics, or dynamics mismatch against the new G1 mode-15 asset.

### 7l. Velocity Policy Provenance And Source-Contract Report - done, blocker found

Commit:

- `diagnose(g1): trace velocity policy provenance`

Status:

- Done. The active Velocity policy has incomplete local provenance.

Actions:

- Extend the Velocity contract report to include:
  - `policy.onnx` metadata `run_path`;
  - local `logs/rsl_rl` matching-source-run search results;
  - current-source G1 init pose versus deploy `default_joint_pos` and action
    offset;
  - current-source action scale, stiffness, and damping versus deploy values.
- Re-run the report on the grounded no-elastic Velocity evidence.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-175822 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_home_root_no_elastic_bootstrap_report.json --expect-failure`

Acceptance gate:

- The report must expose the ONNX `run_path`. Current result:
  `2026-03-18_18-40-20`.
- The report must state whether the source run exists locally. Current result:
  `source_run_found: false`.
- The report must compare current-source action scale/gains against deploy.
  Current result: only rounding-level gaps for action scale, stiffness, and
  damping.
- The report must preserve `primary_reason: velocity_runtime_instability`; this
  is provenance diagnosis, not a pass condition.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is to inspect the
  live Velocity observation/action timing in `g1_ctrl` or replace the active
  `velocity/v0` package with a freshly exported Velocity policy whose training
  provenance and current-source contract are known.

### 7m. Local Velocity Policy Inventory - done, blocker found

Commit:

- `diagnose(g1): inventory velocity policy candidates`

Status:

- Done. No local Velocity ONNX candidate is directly compatible with active
  deploy `v0`.

Actions:

- Add a reusable policy inventory command that scans local `g1_velocity*` ONNX
  files and compares each candidate against the active deploy Velocity policy
  input dimension, output dimension, and observation names.
- Run the inventory against `/home/ssy/ssy_files/mjlab/logs/rsl_rl`.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_policy_inventory.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_inventory.py tests/tools/test_g1_tracking_phase1_velocity_policy_inventory.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_inventory.py tests/tools/test_g1_tracking_phase1_velocity_policy_inventory.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_inventory.py --limit 200 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_policy_inventory.json --expect-no-compatible`

Acceptance gate:

- The report must include the active deploy reference policy contract. Current
  result: `obs[1,98] -> actions[1,29]`.
- The report must classify local candidates. Current result: 125 candidates,
  0 directly compatible.
- The report must explain incompatibility. Current dominant reason:
  candidate `input_dim 99 != 98` and `observation_names differ`.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. Since no local ONNX can be swapped into the
  active deploy `v0` observation contract, the next allowed slice is either:
  instrument the current `g1_ctrl` Velocity observation/action values, or
  generate a complete deploy package from a known compatible Velocity training
  config instead of copying ONNX alone.

### 7n. Velocity Runtime Trace Instrumentation - done, blocker found

Commit:

- `diagnose(g1): trace velocity runtime actions`

Status:

- Done. The local runtime trace makes the failure more specific: at zero
  command, the active Velocity policy emits large actions and the simulated
  robot develops extreme joint velocity before falling.

Actions:

- Add a repo-local patch command for the machine-local
  `.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp`.
- Patch and rebuild local `g1_ctrl` so `Velocity` stable samples include
  `policy_step`, command values, raw and processed action norms, joint relative
  position norms, and joint velocity norms.
- Extend the Velocity contract parser so old logs and new runtime-trace logs
  are both accepted.
- Rerun the grounded policy-default no-elastic Velocity-only GUI smoke.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_runtime_trace_patch.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_runtime_trace_patch.py tests/tools/test_g1_tracking_phase1_velocity_runtime_trace_patch.py src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_runtime_trace_patch.py tests/tools/test_g1_tracking_phase1_velocity_runtime_trace_patch.py src/mjlab/scripts/g1_tracking_phase1_velocity_contract.py tests/tools/test_g1_tracking_phase1_velocity_contract.py`
- Local runtime dry-run:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_runtime_trace_patch.py`
- Local runtime patch and rebuild:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_runtime_trace_patch.py --apply`
  then `cmake --build /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/build -j8`.
- Real evidence command:
  `MJLAB_SIM2SIM_MODE=velocity_bootstrap MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default MJLAB_VELOCITY_BOOTSTRAP_ROOT=home MJLAB_ENABLE_ELASTIC_BAND=0 MJLAB_START_PAUSED=1 MJLAB_AUTO_RUN_AFTER_READY=1 bash scripts/tools/run_flying_kick_sim2sim.sh start`
- Report command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-183512 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_trace_report.json --expect-failure`

Acceptance gate:

- The patch tool must be idempotent and must create a backup before writing to
  `.external`. Current backup:
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp.phase1_velocity_runtime_trace_v1.bak`.
- Rebuilt `g1_ctrl` must compile after the patch.
- The new log must include real command/action/runtime fields. Current result:
  first unstable sample has `command_norm=0.0`, `raw_action_l2=14.76`,
  `processed_action_l2=4.874`, and `joint_vel_l2=673.649`.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is to generate a
  deploy package whose observation contract matches the current source Velocity
  training path, or explicitly compare the active `v0` ONNX against a replay of
  its 98-dim zero-command observation sequence to determine why zero command
  produces large destabilizing actions.

### 7o. Velocity Zero-Command ONNX Replay - done, blocker found

Commit:

- `diagnose(g1): replay velocity zero-command policy`

Status:

- Done. The active deploy `velocity/v0` ONNX outputs nonzero joint targets even
  for the nominal zero-command/default-pose observation.

Actions:

- Add a reusable ONNX replay command that builds the deploy `v0` observation
  stream from `deploy.yaml` using:
  zero base angular velocity, projected gravity `(0,0,-1)`, zero velocity
  command, zero gait-phase observation, zero joint position/velocity error, and
  recurrent `last_action`.
- Convert raw ONNX actions through the deploy `JointPositionAction`
  scale/offset and compare the processed target against the deploy default pose.
- Run the replay against the active local `velocity/v0` package.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_zero_command_replay.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_zero_command_replay.py tests/tools/test_g1_tracking_phase1_velocity_zero_command_replay.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_zero_command_replay.py tests/tools/test_g1_tracking_phase1_velocity_zero_command_replay.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_zero_command_replay.py --steps 5 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_zero_command_replay.json --expect-nonzero-target-gap --target-gap-threshold 0.5`

Acceptance gate:

- The report must prove the ONNX input/output contract. Current result:
  input `obs[1,98]`, output `actions[1,29]`.
- The report must quantify zero-command target drift. Current result:
  `max_processed_target_gap_l2=0.660817`, max joint gap `0.357822`.
- The report must connect to runtime trace. Current result: early runtime trace
  has the same scale of raw action and processed target/q error under zero
  command before the fall.
- Real-robot gate remains locked.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is remediation:
  either produce a complete Velocity deploy package whose zero-command replay
  has a much smaller target gap and whose observation contract matches the
  runtime, or retrain/export Velocity for the current new-G1 mode-15 asset and
  then rerun the zero-command replay followed by Velocity-only GUI smoke.

### 7p. Velocity Deploy Candidate Triage - done, blocker found

Commit:

- `diagnose(g1): triage velocity deploy candidates`

Status:

- Done. Local runs contain actor/checkpoint candidates for current-source
  99-dim Velocity, but no complete deploy package can be directly swapped into
  active `velocity/v0`.

Actions:

- Add a read-only triage command that scans local `g1_velocity*` ONNX files and
  groups them by deploy usefulness:
  active-`v0` observation contract, current-source flat 99-dim Velocity actor,
  rough/history Velocity actor, or unknown.
- For each candidate, report ONNX input/output dims, observation names, run
  dir, `params/env.yaml`, `params/agent.yaml`, `params/deploy.yaml`, latest
  checkpoint, complete Unitree deploy-package availability, and blockers.
- Keep this command read-only: no `.external` edits, no GUI launch, no real
  robot access, and no ONNX copying.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_deploy_candidate_triage.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_deploy_candidate_triage.py scripts/tools/g1_tracking_phase1_velocity_deploy_candidate_triage.py tests/tools/test_g1_tracking_phase1_velocity_deploy_candidate_triage.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_deploy_candidate_triage.py scripts/tools/g1_tracking_phase1_velocity_deploy_candidate_triage.py tests/tools/test_g1_tracking_phase1_velocity_deploy_candidate_triage.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy_candidate_triage.py --limit 300 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy_candidate_triage.json --summary-only --expect-no-direct-ready`

Acceptance gate:

- The report must distinguish direct `velocity/v0` replacement readiness from
  merely having a trained actor/checkpoint. Current result:
  `direct_swap_ready=0`, `complete_unitree_deploy_package=0`,
  `active_v0_contract=0`.
- The report must identify whether local current-source runs are reusable as
  actor re-export candidates. Current result: 122 candidates are
  `current_source_flat_velocity_actor` and `actor_reexport_ready`, but each is
  99-dim and blocked by `requires_99_dim_runtime_observation_support`,
  `requires_unitree_deploy_yaml_generation`, and
  `missing_complete_unitree_deploy_package`.
- The decision must keep direct ONNX swap and real robot locked. Current
  decision: `safe_to_swap_local_onnx_into_active_v0: false` and
  `real_robot_gate: locked`.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is to choose and
  implement one Velocity remediation path: generate a complete 99-dim Unitree
  deploy package plus runtime observation support, or train/export a compatible
  98-dim Velocity package and run zero-command replay before GUI smoke.

### 7q. Velocity Runtime Observation Support Audit - done, blocker found

Commit:

- `diagnose(g1): audit velocity runtime observations`

Status:

- Done. The active deploy runtime cannot run the current-source 99-dim
  Velocity actor by YAML generation alone because it lacks a `base_lin_vel`
  observation source.

Actions:

- Add a read-only audit command that compares:
  - current worktree flat Velocity actor terms;
  - external Unitree source flat Velocity actor terms;
  - active deploy `velocity/v0` YAML observations;
  - C++ deploy registered observation functions;
  - `ArticulationData` fields and `BaseArticulation::update()` data sources.
- Use this audit to decide whether 99-dim deploy-package generation is
  currently safe without a runtime patch.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_runtime_observation_audit.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_runtime_observation_audit.py scripts/tools/g1_tracking_phase1_velocity_runtime_observation_audit.py tests/tools/test_g1_tracking_phase1_velocity_runtime_observation_audit.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_runtime_observation_audit.py scripts/tools/g1_tracking_phase1_velocity_runtime_observation_audit.py tests/tools/test_g1_tracking_phase1_velocity_runtime_observation_audit.py`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_runtime_observation_audit.py --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_runtime_observation_audit.json --expect-runtime-missing-base-lin-vel`

Acceptance gate:

- The report must prove the worktree/external/deploy observation split.
  Current result: worktree flat actor is 99 dims with
  `base_lin_vel, base_ang_vel, projected_gravity, joint_pos, joint_vel,
  actions, command`; external source flat actor is 98 dims with
  `base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
  actions`; active deploy YAML is 98 dims with
  `base_ang_vel, projected_gravity, velocity_commands, gait_phase,
  joint_pos_rel, joint_vel_rel, last_action`.
- The report must prove runtime support is missing. Current result:
  `has_base_lin_vel_observation=false`,
  `has_articulation_linear_velocity_field=false`, and
  `unitree_update_mentions_linear_velocity=false`.
- The decision must block YAML-only 99-dim deployment. Current result:
  `can_run_current_source_99_dim_contract_without_runtime_patch=false` and
  `safe_to_generate_99_dim_deploy_yaml_only=false`.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed remediation must either
  implement and verify a correct runtime base-linear-velocity source before
  using 99-dim Velocity actors, or create a 98-dim Velocity training/export path
  for the active deploy runtime contract.

### 7r. Deploy-Compatible 98-Dim Velocity Task - done, remediation path available

Commit:

- `feat(g1): add deploy98 velocity task contract`

Status:

- Done. A separate `Mjlab-Velocity-Flat-Unitree-G1-Deploy98` task now defines a
  98-dim actor contract aligned with the active Unitree deploy runtime.

Actions:

- Add a `phase` observation term for Velocity tasks.
- Add `unitree_g1_flat_deploy98_env_cfg()` that derives from the flat G1
  Velocity config but replaces actor observations with:
  `base_ang_vel, projected_gravity, command, phase, joint_pos, joint_vel,
  actions`.
- Register the new task ID without mutating existing
  `Mjlab-Velocity-Flat-Unitree-G1` or rough Velocity defaults.
- Add a read-only contract command that maps the task actor terms to active
  deploy YAML terms and proves the semantic order/dimension match.

Mandatory tests:

- `uv run --active --no-sync pytest tests/test_velocity_task.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_task_contract.py`
- `uv run --active --no-sync ruff format --check src/mjlab/tasks/velocity/mdp/observations.py src/mjlab/tasks/velocity/config/g1/env_cfgs.py src/mjlab/tasks/velocity/config/g1/__init__.py tests/test_velocity_task.py src/mjlab/scripts/g1_tracking_phase1_velocity_deploy98_task_contract.py scripts/tools/g1_tracking_phase1_velocity_deploy98_task_contract.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_task_contract.py`
- `uv run --active --no-sync ruff check src/mjlab/tasks/velocity/mdp/observations.py src/mjlab/tasks/velocity/config/g1/env_cfgs.py src/mjlab/tasks/velocity/config/g1/__init__.py tests/test_velocity_task.py src/mjlab/scripts/g1_tracking_phase1_velocity_deploy98_task_contract.py scripts/tools/g1_tracking_phase1_velocity_deploy98_task_contract.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_task_contract.py`
- Registry probe:
  `uv run --active --no-sync python -c "import mjlab.tasks; from mjlab.tasks.registry import list_tasks; print('Mjlab-Velocity-Flat-Unitree-G1-Deploy98' in list_tasks())"`
- Real evidence command:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_task_contract.py --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_task_contract.json --expect-compatible`

Acceptance gate:

- Existing G1 flat Velocity must remain 99-dim and include `base_lin_vel`.
- New G1 deploy98 Velocity actor must be 98-dim and exclude `base_lin_vel`.
- New task must be registered separately and keep G1 action scale/sensors/flat
  terrain invariants.
- Contract report must show active runtime semantic mapping is complete and
  ordered. Current result: `task_contract_matches_active_runtime=true`.
- Report must keep direct swapping and hardware locked. Current result:
  `safe_to_swap_without_training=false`, `real_robot_gate=locked`.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is a controlled
  98-dim Velocity export/deploy-package path: train or locate a checkpoint for
  `Mjlab-Velocity-Flat-Unitree-G1-Deploy98`, export ONNX, generate a matching
  deploy YAML, run zero-command replay, then run Velocity-only GUI smoke.

### 7s. Deploy98 Velocity Package Generator - done, tooling ready

Commit:

- `feat(g1): add deploy98 velocity package generator`

Status:

- Done. A compatible 98-dim deploy ONNX can now be converted into the Unitree
  `policy_dir` layout: `exported/policy.onnx`, optional
  `exported/policy.onnx.data`, and `params/deploy.yaml`.

Actions:

- Add a package generator CLI that reads deploy98 ONNX metadata and the active
  Unitree deploy YAML template.
- Require ONNX `obs[1,98] -> actions[1,29]`.
- Require metadata for `observation_names`, `joint_names`,
  `default_joint_pos`, `joint_stiffness`, `joint_damping`, and `action_scale`.
- Map training actor terms to runtime deploy terms:
  `command -> velocity_commands`, `phase -> gait_phase`,
  `joint_pos -> joint_pos_rel`, `joint_vel -> joint_vel_rel`,
  `actions -> last_action`.
- Copy `policy.onnx.data` when present, because Unitree deployment exports can
  use ONNX external data sidecars.
- Keep generated-package evidence locked behind zero-command replay and
  Velocity-only GUI smoke.

Mandatory tests:

- `uv run --active --no-sync ruff format src/mjlab/scripts/g1_tracking_phase1_velocity_deploy98_package.py scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_package.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_deploy98_package.py scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_package.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_deploy98_package.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_task_contract.py`
- Real dry-run smoke:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py --policy-onnx /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx --out-dir logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_smoke/policy_dir --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_smoke.json --dry-run --expect-compatible`
- Real write smoke:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py --policy-onnx /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx --out-dir logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_smoke/policy_dir --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_package_generator_write_smoke.json --expect-compatible`

Acceptance gate:

- Generator must reject wrong observation order or wrong ONNX dimensions.
- Generated YAML must preserve deploy runtime observation terms in active order.
- Generated action scale and offset must come from ONNX metadata, not stale
  template defaults.
- External data sidecar must be copied when present.
- Report must keep `safe_to_use_for_sim2sim=false`,
  `safe_to_swap_without_zero_command_replay=false`, and
  `real_robot_gate=locked`.
- Current write-smoke result: `package_written=true`,
  `safe_to_run_zero_command_replay=true`, `safe_to_use_for_sim2sim=false`.

Advance rule:

- Do not treat the active `velocity/v0` smoke package as accepted policy. The
  next allowed slice is to train or locate a real deploy98 Velocity checkpoint,
  export it with matching metadata, generate its Unitree policy directory, run
  zero-command replay, then run Velocity-only Unitree MuJoCo GUI smoke before
  any Mimic or hardware action.

### 7t. Actor ONNX Export Metadata Gate - done, export path ready

Commit:

- `fix(g1): attach deploy metadata to actor onnx exports`

Status:

- Done. Manual RSL-RL actor ONNX export now attaches base deploy metadata, so a
  checkpoint exported after training can enter the deploy98 package generator.

Actions:

- Update `scripts/tools/export_rsl_rl_actor_onnx.py` to call
  `get_base_metadata()` and `attach_metadata_to_onnx()` after exporting the
  actor.
- Add `--metadata-run-path` for explicit provenance override.
- Default `run_path` to the restored run directory name when checkpoint parity
  restores a run, or the checkpoint parent directory name otherwise.

Mandatory tests:

- `uv run --active --no-sync ruff check scripts/tools/export_rsl_rl_actor_onnx.py tests/tools/test_export_rsl_rl_actor_onnx.py`
- `uv run --active --no-sync pytest tests/tools/test_export_rsl_rl_actor_onnx.py`

Acceptance gate:

- Explicit `--metadata-run-path` must take precedence.
- Restored run directory must take precedence over checkpoint parent.
- Checkpoint parent must be used when no restored run directory exists.
- Export helper must pass the selected `run_path` to base metadata generation
  and attach that metadata to the output ONNX.

Advance rule:

- Do not export/package a deploy98 policy unless the source checkpoint is from
  `Mjlab-Velocity-Flat-Unitree-G1-Deploy98` or has equivalent verified
  98-dim actor metadata. The next allowed slice is a deploy98 training/export
  smoke or an audit proving an existing checkpoint was trained with that task.

### 7u. Deploy98 Training/Export Smoke - done, non-acceptance evidence recorded

Commit:

- `docs(g1): record deploy98 training smoke evidence`

Status:

- Done. The deploy98 task can run a minimal training iteration, export ONNX with
  deploy metadata, generate a Unitree policy directory, and run zero-command
  replay. The smoke policy remains rejected because it is a 1-iteration policy.

Actions:

- Run `Mjlab-Velocity-Flat-Unitree-G1-Deploy98` for one CPU iteration with 8
  environments, TensorBoard logger, no upload, and save interval 1.
- Verify the exported ONNX exists next to `model_0.pt`.
- Package that ONNX with the deploy98 package generator.
- Run zero-command replay on the generated Unitree policy directory.

Mandatory tests:

- Training/export smoke:
  `uv run --active --no-sync train Mjlab-Velocity-Flat-Unitree-G1-Deploy98 --env.scene.num-envs 8 --agent.max-iterations 1 --agent.save-interval 1 --agent.logger tensorboard --agent.upload-model False --agent.experiment-name g1_velocity_deploy98_smoke --agent.run-name deploy98_smoke_20260522 --gpu-ids None`
- Package smoke:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py --policy-onnx logs/rsl_rl/g1_velocity_deploy98_smoke/2026-05-22_19-40-43_deploy98_smoke_20260522/2026-05-22_19-40-43_deploy98_smoke_20260522.onnx --out-dir logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_package/policy_dir --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_package.json --expect-compatible`
- Zero-command replay:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_zero_command_replay.py --policy-root logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_package/policy_dir --steps 5 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_train_smoke_zero_command_replay.json`

Acceptance gate:

- Smoke actor observation shape must be 98 and action shape must be 29.
- Exported ONNX metadata must match deploy98 actor terms.
- Package report must be compatible and written, while keeping
  `safe_to_use_for_sim2sim=false` and `real_robot_gate=locked`.
- Zero-command replay must run to completion and classify whether the policy
  holds default pose. Current smoke result: `max_processed_target_gap_l2=0.13366`,
  `zero_command_target_is_default=false`.

Advance rule:

- This smoke proves the non-model plumbing, not policy quality. Do not run GUI
  smoke or hardware with this 1-iteration policy. The next allowed slice is a
  real deploy98 training run long enough to produce a candidate policy, followed
  by zero-command replay and then Velocity-only Unitree MuJoCo GUI smoke.

### 7v. Deploy98 300-Iteration Velocity Pilot - done, candidate rejected

Commit:

- `docs(g1): record deploy98 velocity pilot rejection`

Status:

- Done. A 300-iteration GPU pilot ran successfully and produced checkpoints plus
  a deploy98 ONNX, but the candidate is rejected before Unitree MuJoCo GUI smoke.

Actions:

- Run deploy98 Velocity training for 300 iterations with 4096 environments on
  RTX 4090.
- Verify checkpoints and ONNX were produced.
- Package the final exported ONNX into a Unitree policy directory.
- Run zero-command replay on the generated package.
- Extract final TensorBoard scalars for durable diagnosis.

Mandatory tests:

- Training pilot:
  `uv run --active --no-sync train Mjlab-Velocity-Flat-Unitree-G1-Deploy98 --env.scene.num-envs 4096 --agent.max-iterations 300 --agent.save-interval 50 --agent.logger tensorboard --agent.upload-model False --agent.experiment-name g1_velocity_deploy98_candidates --agent.run-name deploy98_v1_300iter_20260522 --gpu-ids "[0]"`
- Package pilot ONNX:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py --policy-onnx logs/rsl_rl/g1_velocity_deploy98_candidates/2026-05-22_19-45-22_deploy98_v1_300iter_20260522/2026-05-22_19-45-22_deploy98_v1_300iter_20260522.onnx --out-dir logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_package/policy_dir --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_package.json --expect-compatible`
- Zero-command replay:
  `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_zero_command_replay.py --policy-root logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_package/policy_dir --steps 5 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_v1_300iter_zero_command_replay.json`

Acceptance gate:

- Training must complete without runtime errors and produce `model_299.pt` plus
  an ONNX.
- Package report must show `compatible=true` and `package_written=true`.
- Candidate must not proceed to GUI unless zero-command replay is acceptable.
  Current result is rejected: `max_processed_target_gap_l2=0.618108`,
  `zero_command_target_is_default=false`.
- Final training scalars also do not meet standing stability intent:
  `Episode/length_seconds=2.692022`, `fell_over=29.083334`,
  `error_vel_xy=0.239295`, `error_vel_yaw=0.306789`.

Advance rule:

- Do not spend more cycles only lengthening this exact recipe. The next allowed
  slice is a training objective/reset/reward diagnosis for deploy98 Velocity:
  zero-command standing curriculum, minimum episode/stability gate, or reward
  changes that prevent short falling episodes from looking like improvement.

### 7w. Deploy98 Standing-First Velocity Task - done, training entry ready

Commit:

- `feat(g1): add deploy98 stand-first velocity task`

Status:

- Done. A separate deploy98 standing-first task is available for zero-command
  stability training before any Unitree MuJoCo GUI, Mimic, or hardware gate.

Actions:

- Register `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst` without
  changing `Mjlab-Velocity-Flat-Unitree-G1-Deploy98`.
- Keep the actor observation contract aligned with the active Unitree deploy
  runtime: `base_ang_vel, projected_gravity, command, phase, joint_pos,
  joint_vel, actions` for 98 dims.
- Make the training command distribution zero-command standing-first:
  `rel_standing_envs=1.0`, all velocity ranges zero, no heading command, no
  forward/world/init-velocity command variants.
- Disable `push_robot` and the `command_vel` curriculum for this task so early
  training cannot be judged against full velocity tracking before the standing
  gate exists.
- Add alive and termination shaping and set the training horizon to 5 seconds,
  matching the minimum Velocity standing gate.

Mandatory tests:

- `uv run --active --no-sync ruff format src/mjlab/tasks/velocity/config/g1/env_cfgs.py src/mjlab/tasks/velocity/config/g1/__init__.py tests/test_velocity_task.py`
- `uv run --active --no-sync ruff check src/mjlab/tasks/velocity/config/g1/env_cfgs.py src/mjlab/tasks/velocity/config/g1/__init__.py tests/test_velocity_task.py`
- `uv run --active --no-sync pytest tests/test_velocity_task.py tests/tools/test_g1_tracking_phase1_velocity_deploy98_task_contract.py`
- Registry/config probe proving the new task is registered, keeps the 98-dim
  actor terms, samples zero commands, has no `push_robot`, has no `command_vel`
  curriculum, and uses `episode_length_s=5.0`.

Acceptance gate:

- The original `Mjlab-Velocity-Flat-Unitree-G1-Deploy98` task must retain its
  full velocity command distribution, `push_robot`, command curriculum, 20s
  horizon, and no stand-first shaping rewards.
- The new stand-first task must be a separate task ID, not a mutation of the
  deploy98 baseline.
- The new stand-first task must remain a 98-dim actor contract compatible with
  the active Unitree deploy runtime.
- This work item does not accept any policy for sim2sim. It only creates the
  next training entry.

Advance rule:

- Do not run Mimic or hardware yet. The next allowed slice is a short
  stand-first training/export smoke, package generation, zero-command replay,
  then a Velocity-only Unitree MuJoCo GUI smoke with replay results recorded as
  diagnostic evidence. Do not use a `FixStand` fall, or replay alone, as the
  final Velocity physics verdict.

### 7x. Deploy98 StandFirst 300-Iteration Velocity Candidate - done, GUI gate pending

Commit:

- `docs(g1): record deploy98 standfirst velocity candidate`

Status:

- Done for training/package/replay evidence. The candidate is not accepted for
  hardware or Mimic. It is ready only for a Velocity-only Unitree MuJoCo smoke
  because the training gate passes but the default-pose-hold replay check does
  not.

Actions:

- Train `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst` for 300 iterations
  on RTX 4090.
- Confirm exported `obs[1,98] -> actions[1,29]` ONNX and `model_299.pt` exist.
- Package the ONNX into a Unitree `policy_dir` with deploy metadata.
- Run zero-command replay and record the non-default target gap.
- Reclassify the replay gate as diagnostic rather than sufficient physics
  evidence: a standing RL policy may output a non-default balanced target, so
  the next decisive gate is Velocity-only MuJoCo physics.

Mandatory tests:

- `uv run --active --no-sync train Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst --env.scene.num-envs 4096 --agent.max-iterations 300 --agent.save-interval 50 --agent.logger tensorboard --agent.upload-model False --agent.experiment-name g1_velocity_deploy98_standfirst_candidates --agent.run-name standfirst_v1_300iter_20260522 --gpu-ids "[0]"`
- TensorBoard scalar extraction from
  `logs/rsl_rl/g1_velocity_deploy98_standfirst_candidates/2026-05-22_20-06-38_standfirst_v1_300iter_20260522/events.out.tfevents.1779451600.ssy-LEGION-REN7000K-26IRX.2229676.0`
  shows final `Episode/length_seconds=5.0`,
  `Episode_Termination/fell_over=0.0`,
  `Metrics/twist/error_vel_xy=0.046835`, and
  `Metrics/twist/error_vel_yaw=0.159242`.
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py --policy-onnx logs/rsl_rl/g1_velocity_deploy98_standfirst_candidates/2026-05-22_20-06-38_standfirst_v1_300iter_20260522/2026-05-22_20-06-38_standfirst_v1_300iter_20260522.onnx --out-dir logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package/policy_dir --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package.json --expect-compatible`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_zero_command_replay.py --policy-root logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package/policy_dir --steps 5 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_zero_command_replay.json`

Acceptance gate:

- Training evidence must reach the 5-second zero-command stand gate in mjlab.
- Package generation must report `compatible=true` and `package_written=true`.
- Replay evidence must be recorded. The observed
  `max_processed_target_gap_l2=0.595646` means this policy is not accepted as a
  default-pose hold controller.
- Real-robot gate remains locked.

Advance rule:

- Next allowed slice is a sim-only Velocity policy-root override or equivalent
  reversible runtime preparation, then a Velocity-only Unitree MuJoCo smoke with
  this packaged policy. Do not trigger Mimic or hardware until that Velocity
  smoke is stable.

### 7y. Velocity Policy-Root Override For Sim2Sim Smoke - done

Commit:

- `fix(g1): allow explicit velocity sim2sim policy root`

Status:

- Done. The single-action sim2sim wrappers can point `velocity_bootstrap` at an
  explicit packaged Velocity policy without copying it into the machine-local
  active `velocity/v0` directory.

Actions:

- Add `MJLAB_VELOCITY_POLICY_ROOT` support to both single-action sim2sim
  wrappers.
- Keep the default path unchanged: `$DEPLOY/config/policy/velocity`.
- Include the selected policy root in wrapper stdout for evidence capture.
- Prove `policy_default` initial joint qpos comes from the explicit policy
  root's `params/deploy.yaml`.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `uv run --active --no-sync ruff format tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`

Acceptance gate:

- Existing default `velocity_bootstrap` behavior remains unchanged.
- Setting `MJLAB_VELOCITY_POLICY_ROOT` to a directory containing
  `params/deploy.yaml` and `exported/policy.onnx` makes `policy_default`
  bootstrap use that directory's `default_joint_pos`.
- No external `.external` runtime files are copied or committed by this work
  item.

Advance rule:

- Next allowed slice is the actual Velocity-only Unitree MuJoCo smoke with
  `MJLAB_VELOCITY_POLICY_ROOT` pointing to the StandFirst packaged policy dir.

### 7z. StandFirst Velocity-Only Unitree MuJoCo Smoke - done, rejected

Commit:

- `docs(g1): record standfirst velocity sim2sim rejection`

Status:

- Done. The first StandFirst policy is rejected by the decisive Velocity-only
  Unitree MuJoCo smoke. It must not be used for Mimic or hardware.

Actions:

- Launch `velocity_bootstrap` with:
  - `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`;
  - `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`;
  - `MJLAB_VELOCITY_POLICY_ROOT` pointing to the StandFirst packaged policy;
  - `MJLAB_ENABLE_ELASTIC_BAND=0`;
  - no Mimic trigger.
- Capture controller log, simulator log, selected configs, hash manifest, and
  screenshot.
- Restore the external Unitree config and stop `g1_ctrl` / `unitree_mujoco`.
- Parse the evidence with the Velocity contract checker.

Mandatory tests:

- `MJLAB_SIM2SIM_MODE=velocity_bootstrap MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default MJLAB_VELOCITY_BOOTSTRAP_ROOT=home MJLAB_VELOCITY_POLICY_ROOT=/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main/logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package/policy_dir MJLAB_ENABLE_ELASTIC_BAND=0 MJLAB_START_PAUSED=1 MJLAB_AUTO_RUN_AFTER_READY=1 bash scripts/tools/run_flying_kick_sim2sim.sh start`
- `bash scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-202752`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-202752 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_standfirst_bootstrap_report.json --expect-failure`
- Process check after restore shows no live `g1_ctrl` or `unitree_mujoco`
  process.

Acceptance gate:

- The smoke is accepted only as failure evidence, not as deploy acceptance.
- The report must show the actual StandFirst policy directory was used.
- The report must show initial default/action-offset and floor-contact gates
  passed, so the rejection is not caused by stale `v0`, root-floor mismatch, or
  a `FixStand` fall.
- Real-robot gate remains locked.

Advance rule:

- Route back to Velocity policy/runtime diagnosis. The next candidate must pass
  Velocity-only Unitree MuJoCo stability for at least 5 seconds before any
  Mimic/action or real-robot gate can reopen.

### 7aa. StandFirst Velocity Policy Sensitivity Probe - done, blocker narrowed

Commit:

- `diagnose(g1): add velocity policy sensitivity probe`

Status:

- Done. The probe gives a fast offline feedback loop for the StandFirst
  policy's remaining zero-command closed-loop failure.

Actions:

- Add a CLI that loads a Unitree Velocity `policy_dir`, reconstructs the deploy
  observation slices from `params/deploy.yaml`, runs warmup zero-command ONNX
  replay, then perturbs one observation term/index at a time.
- Preserve deploy observation order, scale, clip, action scale, action offset,
  and default-joint target-gap reporting.
- Keep the probe offline only: it must not launch MuJoCo, `g1_ctrl`, DDS, or
  hardware.
- Run the probe on the StandFirst package with zero-command-only terms:
  `velocity_commands=0`, `gait_phase=0`, `joint_vel_rel=100`,
  `joint_pos_rel=1`, `last_action=5`, `base_ang_vel=10`,
  `projected_gravity=0.5`.

Mandatory tests:

- `uv run --active --no-sync ruff format src/mjlab/scripts/g1_tracking_phase1_velocity_policy_sensitivity.py scripts/tools/g1_tracking_phase1_velocity_policy_sensitivity.py tests/tools/test_g1_tracking_phase1_velocity_policy_sensitivity.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_sensitivity.py scripts/tools/g1_tracking_phase1_velocity_policy_sensitivity.py tests/tools/test_g1_tracking_phase1_velocity_policy_sensitivity.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_policy_sensitivity.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_sensitivity.py --policy-root logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_v1_300iter_package/policy_dir --warmup-steps 5 --magnitude gait_phase=0 --magnitude velocity_commands=0 --magnitude joint_vel_rel=100 --magnitude last_action=5 --magnitude joint_pos_rel=1 --magnitude base_ang_vel=10 --magnitude projected_gravity=0.5 --top-k 20 --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_standfirst_policy_sensitivity_zero_command_terms.json`

Acceptance gate:

- The CLI reports `available=true` and `obs[1,98] -> actions[1,29]` for the
  StandFirst package.
- The zero-command-focused report keeps `velocity_commands` and `gait_phase`
  perturbations at zero so the diagnosis matches the GUI failure's
  `command_norm=0.0` condition.
- The report identifies the strongest remaining zero-command sensitivity term
  and keeps `real_robot_gate=locked`.
- No GUI, simulator, controller, or hardware process is launched.

Advance rule:

- Next candidate training should directly penalize or damp the diagnosed
  joint-velocity/action-amplification loop, then re-run zero-command replay,
  sensitivity, and the Velocity-only Unitree MuJoCo 5-second smoke before any
  Mimic or hardware gate reopens.

### 7ab. Deploy98 StandFirst Damped Velocity Task - done, training entry ready

Commit:

- `feat(g1): add deploy98 standfirst damped velocity task`

Status:

- Done. The next deploy98 training entry is registered without mutating
  `Deploy98`, `Deploy98-StandFirst`, or the current 99-dim G1 flat task.

Actions:

- Register `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped`.
- Preserve the 98-dim deploy actor terms and zero-command StandFirst command
  setup.
- Add targeted damping costs for the diagnosed loop:
  - stronger `pose`;
  - stronger `body_ang_vel`;
  - stronger `action_rate_l2`;
  - new `action_acc_l2`;
  - new `joint_vel_l2`.
- Keep this as a training entry only, not acceptance evidence.

Mandatory tests:

- `uv run --active --no-sync ruff format src/mjlab/tasks/velocity/config/g1/env_cfgs.py src/mjlab/tasks/velocity/config/g1/__init__.py tests/test_velocity_task.py`
- `uv run --active --no-sync ruff check src/mjlab/tasks/velocity/config/g1/env_cfgs.py src/mjlab/tasks/velocity/config/g1/__init__.py tests/test_velocity_task.py`
- `uv run --active --no-sync pytest tests/test_velocity_task.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_task_contract.py --task-id Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped --report-out logs/g1_tracking_phase1/2026-05-22T13-37-34+08-00/velocity_deploy98_standfirst_damped_task_contract.json --expect-compatible`

Acceptance gate:

- The new task reports the same 98-dim actor/runtime mapping as the active
  Unitree deploy contract.
- The damping rewards are present only on the new damped task.
- Existing Deploy98, StandFirst, and current 99-dim flat G1 Velocity contracts
  remain unchanged.
- Real-robot gate remains locked.

Advance rule:

- Next allowed slice is a short GPU pilot for
  `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped`, followed by
  package generation, zero-command replay, policy sensitivity, and only then a
  Velocity-only Unitree MuJoCo 5-second smoke if the offline gates improve.

### 7ac. Deploy98 StandFirst Damped 300-Iteration Pilot - done, candidate rejected before package

Commit:

- `docs(g1): record damped standfirst velocity pilot rejection`

Status:

- Done. The candidate trained and exported, but it did not pass the strict
  5-second zero-command training gate, so the package/replay/sensitivity/GUI
  sequence was not started.

Actions:

- Train `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped` for 300
  iterations on RTX 4090 with 4096 environments.
- Exported ONNX and checkpoints are kept as generated local artifacts only:
  `logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-01-53_standfirst_damped_v1_300iter_20260522/`.
- Extract final TensorBoard scalars and decide whether the candidate is allowed
  to proceed to package generation.

Mandatory tests:

- TensorBoard scalar extraction from
  `events.out.tfevents.1779454915.ssy-LEGION-REN7000K-26IRX.2273190.0`.
- `git diff --check`.

Acceptance gate:

- Reject unless final `Episode/length_seconds == 5.0` and
  `Episode_Termination/fell_over == 0.0`.
- Record the final metrics and local artifact path.
- Do not generate a Unitree policy package, run zero-command replay, launch
  Unitree MuJoCo, trigger Mimic, or touch hardware when the gate fails.
- Real-robot gate remains locked.

Evidence:

- Final step 299 metrics:
  - `Episode/length_seconds=4.888494`
  - `Episode_Termination/fell_over=2.166667`
  - `Episode_Termination/time_out=16.791668`
  - `Episode_Reward/total=14.423843`
  - `Episode_Metrics/mean_action_acc=0.301305`
  - `Metrics/slip_velocity_mean=0.038145`
  - `Metrics/twist/error_vel_xy=0.106901`
  - `Metrics/twist/error_vel_yaw=0.472875`

Advance rule:

- Do not continue to package/GUI for this candidate. The next allowed slice is
  diagnosis or a new training candidate that explicitly addresses the remaining
  near-gate failures without mutating accepted baseline task IDs.

### 7ad. Deploy98 StandFirst Damped Continuation Package - done, Velocity-only GUI gate pending

Commit:

- `docs(g1): record damped standfirst continuation candidate`

Status:

- Done. The resumed damped candidate passes the mjlab 5-second zero-command
  training gate, packages cleanly against the 98-dim Unitree runtime contract,
  and improves zero-command replay. It still needs a Velocity-only Unitree
  MuJoCo smoke before any Mimic or hardware gate can reopen.

Actions:

- Resume
  `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped` from
  `model_299.pt` for 300 more iterations.
- Export and package the resulting `obs[1,98] -> actions[1,29]` ONNX using the
  active Unitree G1 deploy YAML template.
- Run zero-command replay and two sensitivity probes:
  - zero-command-focused stress probe with `velocity_commands=0` and
    `gait_phase=0`;
  - default-magnitude probe to catch command/phase masking hazards.
- Keep real-robot, Mimic, and dual-action gates locked.

Mandatory tests:

- TensorBoard scalar extraction from
  `logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-12-09_standfirst_damped_v1_resume300_from299_20260522/`.
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py --policy-onnx logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-12-09_standfirst_damped_v1_resume300_from299_20260522/2026-05-22_21-12-09_standfirst_damped_v1_resume300_from299_20260522.onnx --out-dir logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package.json --expect-compatible`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_zero_command_replay.py --policy-root logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir --steps 5 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_zero_command_replay.json`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_sensitivity.py --policy-root logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir --warmup-steps 5 --magnitude velocity_commands=0 --magnitude gait_phase=0 --magnitude joint_vel_rel=100 --magnitude last_action=5 --magnitude joint_pos_rel=1 --magnitude base_ang_vel=10 --magnitude projected_gravity=0.5 --top-k 12 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_policy_sensitivity.json --expect-sensitive`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_sensitivity.py --policy-root logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir --warmup-steps 5 --top-k 12 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_policy_sensitivity_default.json --expect-sensitive`
- `git diff --check`.

Acceptance gate:

- Final TensorBoard scalars must show `Episode/length_seconds == 5.0` and
  `Episode_Termination/fell_over == 0.0`.
- Package report must be `compatible=true` and write both `exported/policy.onnx`
  and `params/deploy.yaml`.
- Zero-command replay must improve over the rejected StandFirst package and keep
  the policy-root / deploy-yaml / ONNX dimensions at `98 -> 29`.
- Sensitivity reports must keep `real_robot_gate=locked` and identify remaining
  command/phase/joint-velocity hazards before GUI.
- No Unitree MuJoCo GUI, Mimic trigger, or hardware process is launched in this
  work item.

Evidence:

- Resumed run:
  `logs/rsl_rl/g1_velocity_deploy98_standfirst_damped_candidates/2026-05-22_21-12-09_standfirst_damped_v1_resume300_from299_20260522/`.
- Final step 598 metrics:
  - `Episode/length_seconds=5.0`
  - `Episode_Termination/fell_over=0.0`
  - `Episode_Termination/time_out=16.208334`
  - `Episode_Reward/total=38.259392`
  - `Episode_Metrics/mean_action_acc=0.065160`
  - `Metrics/slip_velocity_mean=0.003066`
  - `Metrics/angular_momentum_mean=0.136680`
  - `Metrics/twist/error_vel_xy=0.026822`
  - `Metrics/twist/error_vel_yaw=0.070388`
- Last-10 TensorBoard window:
  - `Episode/length_seconds` mean/min/max all `5.0`
  - `Episode_Termination/fell_over` mean/min/max all `0.0`
  - `Episode_Reward/total` mean `38.220667`
  - `Episode_Metrics/mean_action_acc` mean `0.065809`
- Package report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package.json`.
- Zero-command replay report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_zero_command_replay.json`.
  Result: `max_processed_target_gap_l2=0.385498`,
  `zero_command_target_is_default=false`.
- Zero-command-focused stress sensitivity report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_policy_sensitivity.json`.
  Result: baseline processed target gap `0.391274`, highest sensitivity
  `joint_vel_rel`, worst processed target gap `3.296666` under a
  `100 rad/s` single-axis joint-velocity perturbation.
- Default-magnitude sensitivity report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_policy_sensitivity_default.json`.
  Result: baseline processed target gap `0.391274`, highest sensitivity
  `gait_phase`, worst processed target gap `3.568449`; this is a masking hazard
  check, because the deploy runtime should zero gait phase when
  `command_norm < 0.1`.

Advance rule:

- The next allowed slice is a Velocity-only Unitree MuJoCo smoke using this exact
  package via `MJLAB_VELOCITY_POLICY_ROOT`, with no Mimic trigger and no hardware.
  A `FixStand` fall alone still does not classify this candidate; acceptance must
  be judged in `Velocity`.

### 7ae. Deploy98 StandFirst Damped Continuation Velocity-Only Smoke - done, candidate rejected

Commit:

- `docs(g1): record damped continuation sim2sim rejection`

Status:

- Done. The candidate is rejected by Unitree MuJoCo before any Mimic trigger.
  This is a `Velocity` runtime failure, not `FixStand`, not stale policy path,
  not initial pose/action-offset mismatch, and not initial foot contact.

Actions:

- Launch `run_flying_kick_sim2sim.sh start` in `velocity_bootstrap` mode with:
  - `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`;
  - `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`;
  - `MJLAB_VELOCITY_POLICY_ROOT` pointing to the damped continuation package;
  - `MJLAB_ENABLE_ELASTIC_BAND=0`;
  - `MJLAB_AUTO_RUN_AFTER_READY=1`;
  - no Mimic trigger.
- Capture GUI screenshot evidence.
- Stop tmux sessions and restore external Unitree runtime config.
- Generate a structured Velocity contract report.

Mandatory tests:

- `scripts/tools/run_flying_kick_sim2sim.sh status`
- `MJLAB_SIM2SIM_MODE=velocity_bootstrap MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default MJLAB_VELOCITY_BOOTSTRAP_ROOT=home MJLAB_VELOCITY_POLICY_ROOT=logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir MJLAB_ENABLE_ELASTIC_BAND=0 MJLAB_START_PAUSED=1 MJLAB_AUTO_RUN_AFTER_READY=1 scripts/tools/run_flying_kick_sim2sim.sh start`
- Screenshot capture:
  `logs/flying_kick_sim2sim/20260522-212636/mujoco_damped_resume300_velocity_run_fall.png`
- `scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-212636`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-212636 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_bootstrap_report.json --expect-failure`
- `pgrep -af 'unitree_mujoco|g1_ctrl|flying_kick_sim|flying_kick_ctrl'`
- `git diff --check`

Acceptance gate:

- Accept only if the controller stays in `Velocity` for at least 5 seconds after
  MuJoCo `Run`, with no first unstable sample and no Mimic trigger.
- Reject if `Velocity` fails before 5 seconds, even if mjlab training and
  zero-command replay improved.
- Restore external Unitree config and leave no simulator/controller process
  active before recording the result.
- Real-robot and Mimic gates remain locked.

Evidence:

- Evidence directory:
  `logs/flying_kick_sim2sim/20260522-212636/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260522-212636/mujoco_damped_resume300_velocity_run_fall.png`.
- Velocity contract report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_bootstrap_report.json`.
- Confirmed setup:
  - `FSM: Start Velocity` reached.
  - `policy_dir_resolved` is the damped continuation package.
  - ONNX/deploy shape is `obs[1,98] -> actions[1,29]`.
  - selected initial joints match deploy `default_joint_pos` and action offset
    with `gap_l2=0.0`, `gap_max=0.0`.
  - selected root z is `0.783675`.
  - initial foot clearance passes with `min_foot_surface_z=0.027211`.
  - `enable_elastic_band=0`.
- First unstable sample:
  - timestamp `2026-05-22 21:26:40.990`;
  - line `30`;
  - stable duration before failure `2.0s`;
  - `policy_step=100`;
  - `command_norm=0.0`;
  - `raw_action_l2=38.483`;
  - `processed_action_l2=15.654`;
  - `joint_vel_l2=269.793`;
  - `q_err_l2=13.591`;
  - `root_ang_vel_l2=19.108`;
  - `gravity_b=(0.945,0.320,-0.068)`.

Advance rule:

- Do not tune kicks or trigger Mimic. The next slice must diagnose the `Velocity`
  closed-loop mismatch between mjlab training/play and Unitree deploy runtime:
  command/phase masking, action target scaling, policy-rate timing, low-state
  joint velocity values, or MuJoCo actuator dynamics on the new G1 mode-15
  asset.

### 7af. Passive-to-Velocity Damped Continuation Smoke - done, candidate rejected

Commit:

- `docs(g1): record passive velocity sim2sim rejection`

Status:

- Done. This validates the deploy-state interpretation: `Passive` is only a
  low-risk entry/settling state, `FixStand` is not the judged state, and the
  candidate is rejected only after the controller reaches `Velocity`.

Actions:

- Add `passive_velocity_bootstrap` mode to the single-action sim2sim wrappers.
- Generate a deploy config with `FSM.initial_state: Passive` and a sim-only
  `Passive -> Velocity` transition.
- Keep `FixStand.qs` valid in the generated config because `CtrlFSM`
  constructs all enabled states, even if the selected initial state is
  `Passive`.
- Launch the damped continuation package through `Passive -> Velocity`, then
  click MuJoCo `Run` and classify the resulting Velocity evidence.
- Restore external Unitree runtime config and confirm no simulator/controller
  process remains active.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh scripts/tools/run_roundhouse_leading_right_sim2sim.sh scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `uv run --active --no-sync ruff check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `MJLAB_SIM2SIM_MODE=passive_velocity_bootstrap MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default MJLAB_VELOCITY_BOOTSTRAP_ROOT=home MJLAB_VELOCITY_POLICY_ROOT=logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir MJLAB_ENABLE_ELASTIC_BAND=0 MJLAB_START_PAUSED=1 MJLAB_AUTO_RUN_AFTER_READY=1 scripts/tools/run_flying_kick_sim2sim.sh start`
- Screenshot capture:
  `logs/flying_kick_sim2sim/20260522-215208/mujoco_passive_velocity_damped_resume300_run_fall.png`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-215208 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_passive_bootstrap_report.json --expect-failure`
- `scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-215208`
- `pgrep -af 'unitree_mujoco|g1_ctrl|flying_kick_sim|flying_kick_ctrl'`

Acceptance gate:

- Accept only if the controller logs `FSM: Start Passive`, then `FSM: Change
  state from Passive to Velocity`, then stays dynamically stable in `Velocity`
  for at least 5 seconds after MuJoCo `Run`.
- Reject if `Velocity` fails before 5 seconds.
- Do not classify a `FixStand` fall as a Velocity policy failure.
- Do not unlock Mimic or real hardware from a paused policy-thread duration.

Evidence:

- Evidence directory:
  `logs/flying_kick_sim2sim/20260522-215208/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260522-215208/mujoco_passive_velocity_damped_resume300_run_fall.png`.
- Velocity contract report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_damped_resume300_passive_bootstrap_report.json`.
- Confirmed setup:
  - `FSM: Start Passive`.
  - `FSM: Change state from Passive to Velocity`.
  - explicit damped continuation package path.
  - ONNX/deploy shape is `obs[1,98] -> actions[1,29]`.
  - selected initial joints match deploy `default_joint_pos` and action offset
    with `gap_l2=0.0`, `gap_max=0.0`.
  - selected root z is `0.783675`.
  - initial foot clearance passes with `min_foot_surface_z=0.027211`.
  - `enable_elastic_band=0`.
- First dynamic unstable sample after MuJoCo `Run`:
  - timestamp `2026-05-22 21:53:34.606`;
  - line `194`;
  - `policy_step=4175`;
  - `command_norm=0.0`;
  - `raw_action_l2=30.018`;
  - `processed_action_l2=12.254`;
  - `joint_vel_l2=351.815`;
  - `q_err_l2=9.531`;
  - `root_ang_vel_l2=14.619`;
  - `gravity_b=(0.537,0.381,-0.753)`.
- Note: report stable-duration includes paused policy-thread samples with
  `joint_vel_l2=0`; do not use that field as dynamic standing acceptance.

Advance rule:

- Continue diagnosing the `Velocity` closed loop. The deploy-state path is now
  clear enough that future smoke tests should use `Passive -> Velocity` for
  deploy parity, but the acceptance state remains `Velocity`, not `Passive` or
  `FixStand`.

### 7ag. Velocity Actuator Force Contract Audit - done, blocker narrowed

Commit:

- `diagnose(g1): audit velocity actuator force contract`

Status:

- Done. The current evidence narrows the `Velocity` runtime mismatch to a
  concrete Unitree MuJoCo scene dynamics mismatch. Robot XML replacement is not
  the active blocker.

Actions:

- Add a repo-local CLI that compares:
  - mjlab G1 training-side `BuiltinPositionActuatorCfg.effort_limit`;
  - Unitree MuJoCo external `scene_g1.xml` motor `ctrlrange`;
  - optional semantic equality between the user `g1_new.xml`, worktree G1 XML,
    and external runtime G1 XML.
- Run it against the active local Unitree runtime scene.
- Keep `.external` unchanged in this commit.

Mandatory tests:

- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_actuator_contract.py scripts/tools/g1_tracking_phase1_velocity_actuator_contract.py tests/tools/test_g1_tracking_phase1_velocity_actuator_contract.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_actuator_contract.py scripts/tools/g1_tracking_phase1_velocity_actuator_contract.py tests/tools/test_g1_tracking_phase1_velocity_actuator_contract.py`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_actuator_contract.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_actuator_contract.py --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_contract_report.json --expect-mismatch`

Acceptance gate:

- The audit must prove whether the active external Unitree MuJoCo scene motor
  force limits match the mjlab training actuator effort limits.
- If there is a mismatch, do not run hardware; route to a reversible local
  scene alignment smoke.
- If all force limits match, continue to policy-rate timing, command/phase
  masking, or observation-value diagnosis.

Evidence:

- Report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_contract_report.json`.
- `decision.actuator_force_contract_matches=false`.
- `decision.primary_mismatch=external_scene_motor_ctrlrange`.
- `decision.right_ankle_limit_mismatch=true`.
- Semantic XML checks:
  - user `g1_new.xml` vs worktree `g1.xml`: `match=true`;
  - worktree `g1.xml` vs external runtime `g1.xml`: `match=true`.
- Force-limit mismatches:
  - `right_ankle_pitch_joint`: expected `50.0`, external `25.0`;
  - `right_ankle_roll_joint`: expected `50.0`, external `25.0`;
  - `waist_roll_joint`: expected `50.0`, external `25.0`;
  - `waist_pitch_joint`: expected `50.0`, external `25.0`.
- All other joints match.

Advance rule:

- The next allowed slice is a reversible local-runtime patch to external
  `scene_g1.xml` only, changing the four mismatched motor `ctrlrange` values to
  `[-50, 50]`, then rerunning the exact damped package through
  `Passive -> Velocity`. Accept only with 5 seconds of dynamic `Velocity`
  stability after MuJoCo `Run`.

### 7ah. Actuator-Aligned Passive Velocity Smoke - done, rejected

Commit:

- `docs(g1): record actuator-aligned velocity smoke rejection`

Status:

- Done. The reversible local scene force-limit patch removes the known external
  `scene_g1.xml` actuator mismatch, but the exact damped Velocity package still
  fails in dynamic `Velocity`.

Actions:

- Back up the local runtime scene to
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/scene_g1.xml.before_velocity_actuator_limit_patch`.
- Patch only the local runtime scene motor `ctrlrange` values for:
  - `right_ankle_pitch_joint`;
  - `right_ankle_roll_joint`;
  - `waist_roll_joint`;
  - `waist_pitch_joint`.
- Re-run the actuator contract audit with `--expect-match`.
- Re-run the exact damped package through the more deploy-faithful
  `Passive -> Velocity` GUI smoke.
- Restore the active deploy/sim config from the smoke artifact directory after
  the run. The local scene patch is intentionally left in place as the next
  diagnostic baseline; the backup above can restore the pre-patch scene.

Mandatory tests:

- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_actuator_contract.py --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_contract_after_local_scene_patch_report.json --expect-match`
- `MJLAB_SIM2SIM_MODE=passive_velocity_bootstrap MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default MJLAB_VELOCITY_BOOTSTRAP_ROOT=home MJLAB_VELOCITY_POLICY_ROOT=logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir MJLAB_ENABLE_ELASTIC_BAND=0 MJLAB_START_PAUSED=1 MJLAB_AUTO_RUN_AFTER_READY=1 bash scripts/tools/run_flying_kick_sim2sim.sh start`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-222217 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_aligned_passive_bootstrap_report.json --expect-failure`
- `bash scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-222217`

Acceptance gate:

- Accept only if `Velocity` remains dynamically stable for at least 5 seconds
  after MuJoCo `Run`, with no first unstable sample in the runtime trace.
- Do not count paused policy-thread samples where `joint_vel_l2=0`.
- Do not unlock Mimic or hardware if the report has
  `primary_reason=velocity_runtime_instability`.

Evidence:

- Scene contract report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_contract_after_local_scene_patch_report.json`.
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_aligned_passive_bootstrap_report.json`.
- GUI evidence directory:
  `logs/flying_kick_sim2sim/20260522-222217/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260522-222217/mujoco_actuator_aligned_passive_velocity_run_fall.png`.
- The actuator contract now matches:
  - `decision.actuator_force_contract_matches=true`;
  - `decision.mismatch_count=0`.
- The smoke still fails:
  - `passed=false`;
  - `primary_reason=velocity_runtime_instability`;
  - first dynamic unstable sample at line `143`;
  - timestamp `2026-05-22 22:23:17.944`;
  - `policy_step=2900`;
  - `command_norm=0.0`;
  - `raw_action_l2=27.663`;
  - `processed_action_l2=12.382`;
  - `joint_vel_l2=415.247`;
  - `q_err_l2=10.440`;
  - `root_ang_vel_l2=19.636`.

Advance rule:

- Continue from an actuator-aligned Unitree MuJoCo scene baseline. The next
  slice should diagnose the remaining `Velocity` closed loop directly: policy
  step timing while paused/running, runtime observation values at the first
  dynamic frame, command/phase masking, or PD target/action scaling under the
  local Unitree bridge.

### 7ai. Actuator-Aligned Start-Unpaused Velocity Smoke - done, rejected

Commit:

- `docs(g1): record start-unpaused velocity smoke rejection`

Status:

- Done. Starting MuJoCo unpaused removes the paused-policy-step timing artifact
  but does not stabilize the damped Velocity package.

Actions:

- Keep the actuator-aligned local Unitree MuJoCo scene from work item 7ah.
- Re-run the exact damped package through `Passive -> Velocity` with:
  - `MJLAB_START_PAUSED=0`;
  - `MJLAB_AUTO_RUN_AFTER_READY=0`;
  - elastic band disabled;
  - policy-default initial joints;
  - HOME root height.
- Generate the runtime contract report with `--expect-failure`.
- Restore active deploy/sim config after the run.

Mandatory tests:

- `MJLAB_SIM2SIM_MODE=passive_velocity_bootstrap MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default MJLAB_VELOCITY_BOOTSTRAP_ROOT=home MJLAB_VELOCITY_POLICY_ROOT=logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir MJLAB_ENABLE_ELASTIC_BAND=0 MJLAB_START_PAUSED=0 MJLAB_AUTO_RUN_AFTER_READY=0 bash scripts/tools/run_flying_kick_sim2sim.sh start`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-222840 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_aligned_passive_unpaused_report.json --expect-failure`
- `bash scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-222840`

Acceptance gate:

- Accept only if start-unpaused `Velocity` reaches at least 5 seconds of
  dynamic stability.
- If the first unstable sample appears within the first second, do not continue
  to Mimic or hardware; diagnose runtime observation/action values.

Evidence:

- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_actuator_aligned_passive_unpaused_report.json`.
- GUI evidence directory:
  `logs/flying_kick_sim2sim/20260522-222840/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260522-222840/mujoco_actuator_aligned_passive_unpaused_run_fall.png`.
- Result:
  - `passed=false`;
  - `primary_reason=velocity_runtime_instability`;
  - `sim_config.start_paused=0`;
  - first unstable sample at line `28`;
  - timestamp `2026-05-22 22:28:43.593`;
  - `stable_duration_before_first_unstable_s=0.5`;
  - `policy_step=25`;
  - `command_norm=0.0`;
  - `raw_action_l2=40.939`;
  - `processed_action_l2=16.796`;
  - `joint_vel_l2=425.455`;
  - `q_err_l2=14.955`;
  - `root_ang_vel_l2=2.401`.

Advance rule:

- Paused policy stepping is a reporting/launch artifact, not the sole cause.
  Continue by logging the full 98-dim deployed observation vector and per-joint
  processed target at the first dynamic frame, then compare it against the
  mjlab training/play observation for the same pose/velocity state.

### 7aj. Official-Aligned FixStand-To-Velocity Bootstrap Mode - done

Commit:

- `fix(g1): add official velocity sim2sim bootstrap`

Status:

- Done. The sim2sim wrappers now have a primary official-aligned Velocity
  acceptance mode instead of treating direct `Passive -> Velocity` as the main
  route.

Actions:

- Add `MJLAB_SIM2SIM_MODE=official_velocity_bootstrap` to both single-action
  sim2sim wrappers and the dual-action phase-1 wrapper.
- Generate a deploy config that starts in `FixStand`, then uses a sim-only
  `FixStand -> Velocity` auto transition so the acceptance state is still
  `Velocity`.
- Reuse `MJLAB_VELOCITY_BOOTSTRAP_POSE`, `MJLAB_VELOCITY_BOOTSTRAP_ROOT`, and
  `MJLAB_VELOCITY_POLICY_ROOT` so the selected Velocity policy default joint
  pose and HOME root can be used without direct `Velocity` startup.
- Keep elastic-band defaults and key-8 pre-tension support aligned with the
  public Unitree G1 sim2sim flow.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh`
- `bash -n scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `bash -n scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff format --check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`

Acceptance gate:

- The dual-action wrapper accepts and forwards
  `--mode official_velocity_bootstrap`.
- The generated single-action config has `FSM.initial_state=FixStand`,
  `FixStand.transitions.Velocity=!A`, and `Velocity.policy_dir` set to the
  selected policy root.
- With `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default` and
  `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`, the sim `initial_qpos` uses the
  selected Velocity policy default joint pose and HOME root height.
- This work item does not unlock Mimic or hardware. It only fixes the
  official-aligned Velocity acceptance launch path; the next work item must run
  GUI evidence through this mode.

Evidence:

- `bash -n` passed for all three wrapper scripts.
- `tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`: `21 passed`.
- `ruff format --check`: `1 file already formatted`.
- `ruff check`: `All checks passed`.

Advance rule:

- Run the actuator-aligned damped package through
  `official_velocity_bootstrap` with policy-default joints, HOME root, and the
  explicit packaged Velocity policy root. Accept only if `Velocity` remains
  dynamically stable for at least 5 seconds after MuJoCo `Run`.

### 7ak. Official-Velocity Auto-Run State Wait Fix - done

Commit:

- `fix(g1): keep waiting after official velocity transition`

Status:

- Done. The first `official_velocity_bootstrap` smoke proved the generated FSM
  was correct, but exposed an automation bug: after `FixStand -> Velocity`, if
  the MuJoCo window was not discoverable on that exact loop iteration, the
  helper resumed waiting for `FSM: Start Velocity`, which is not emitted for a
  transition. This could leave MuJoCo paused until a manual Run click.

Actions:

- Track a separate `state_ready` flag inside `unpause_when_ready`.
- Once `Passive -> Velocity` or `FixStand -> Velocity` has been observed, keep
  searching for the MuJoCo window without requiring a second `FSM: Start
  Velocity` line.
- Add a regression test where the fake controller log emits `FSM: Start
  FixStand` followed by `FSM: Change state from FixStand to Velocity`, and
  assert auto-run proceeds after `Velocity` is ready.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh`
- `bash -n scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff format --check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff check tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`

Acceptance gate:

- `official_velocity_bootstrap` auto-run no longer depends on a nonexistent
  `FSM: Start Velocity` log after the transition path.
- The helper reports `Requested MuJoCo Run after Velocity was ready` in the
  regression test.

Evidence:

- `bash -n` passed for both single-action scripts.
- `tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`: `22 passed`.
- `ruff format --check`: `1 file already formatted`.
- `ruff check`: `All checks passed`.

Advance rule:

- Re-run the official-aligned GUI smoke without manual Run/key-8 intervention
  except for the explicit key-9 release required by the public Unitree flow.

### 7al. Official-Aligned Velocity GUI Smoke - done, candidate rejected

Commit:

- `docs(g1): record official velocity bootstrap rejection`

Status:

- Done. The official-aligned `FixStand -> Velocity` bootstrap now reaches the
  judged `Velocity` state and clicks MuJoCo `Run` automatically, but the damped
  StandFirst package still fails in dynamic `Velocity` before any Mimic or
  hardware gate can reopen.

Actions:

- Use Tavily/source review to confirm the public Unitree sim2sim flow is
  `unitree_mujoco` plus `g1_ctrl`, with stand/contact preparation before policy
  execution. Treat `Passive -> Velocity` as a diagnostic shortcut, not the
  primary official-aligned acceptance route.
- Run the exact damped StandFirst continuation package through
  `official_velocity_bootstrap` with:
  - `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`
  - `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`
  - `MJLAB_VELOCITY_POLICY_ROOT=<damped-resume300 package>/policy_dir`
  - `MJLAB_ENABLE_ELASTIC_BAND=1`
  - `MJLAB_START_PAUSED=1`
  - `MJLAB_AUTO_RUN_AFTER_READY=1`
  - `MJLAB_ELASTIC_PRETENSION_STEPS=24`
- Send MuJoCo key `9` only after the automation reports Run was requested after
  `Velocity` readiness, matching the public elastic-band release order.
- Generate both the Velocity contract report and the policy I/O trace report.
- Restore the generated deploy config and verify no `g1_ctrl` or
  `unitree_mujoco` process remains active.

Mandatory tests:

- `env VIRTUAL_ENV=/home/ssy/ssy_files/mjlab/.venv PATH=/home/ssy/ssy_files/mjlab/.venv/bin:$PATH PYTHONPATH=/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main/src uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-225945 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_auto_run_after_key9_report.json --expect-failure`
- `env VIRTUAL_ENV=/home/ssy/ssy_files/mjlab/.venv PATH=/home/ssy/ssy_files/mjlab/.venv/bin:$PATH PYTHONPATH=/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main/src uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_report.py --evidence-dir logs/flying_kick_sim2sim/20260522-225945 --deploy-yaml logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir/params/deploy.yaml --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_auto_run_policy_io_trace_report.json --expect-trace`
- `bash scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-225945`
- `pgrep -af '[u]nitree_mujoco|[g]1_ctrl|[f]lying_kick_sim|[f]lying_kick_ctrl'`

Acceptance gate:

- Accept only if `Velocity` remains dynamically stable for at least 5 seconds
  after MuJoCo `Run` and after the explicit key-9 release.
- Reject if the first dynamic unstable sample occurs before 5 seconds, even if
  the controller spent longer in a paused policy-thread state.
- Do not classify a `FixStand` fall as a Velocity policy failure.
- Do not unlock Mimic or real hardware from this candidate.

Evidence:

- Tavily/search review found the public Unitree RL Lab sim2sim flow: launch
  `unitree_mujoco`, run `g1_ctrl`, stand the robot up, press MuJoCo key `8` to
  bring feet/contact down, run the policy, then press key `9` to disable the
  elastic band. This supports `official_velocity_bootstrap` as the primary
  acceptance route and demotes direct `Passive -> Velocity` to diagnosis-only.
- Screenshot:
  `logs/flying_kick_sim2sim/20260522-225945/mujoco_official_velocity_bootstrap_auto_run_after_key9.png`.
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_auto_run_after_key9_report.json`.
- Policy I/O trace report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_auto_run_policy_io_trace_report.json`.
- The contract report confirms `Velocity.policy_dir` resolves to the damped
  continuation package, ONNX/deploy observation shape is 98 dims, initial joints
  match deploy default/action offset (`gap_l2=0.0`, `gap_max=0.0`), HOME root z
  is selected (`0.783675`), foot clearance passes, elastic band is enabled, and
  `start_paused=1`.
- The package is still rejected in dynamic `Velocity`: first unstable sample at
  line 74 / `2026-05-22 23:00:06.361`, `policy_step=950`,
  `command_norm=0.0`, `raw_action_l2=45.429`,
  `processed_action_l2=19.583`, `joint_vel_l2=263.947`,
  `q_err_l2=17.164`, and `root_ang_vel_l2=14.925`.
- Policy I/O tracing is present and the 98-dim schema matches, but the selected
  trace is still pre-dynamic; `first_unstable_has_nearby_trace=false`.
- `pgrep -af '[u]nitree_mujoco|[g]1_ctrl|[f]lying_kick_sim|[f]lying_kick_ctrl'`
  returns no active process after restore/cleanup.

Advance rule:

- Continue diagnosing the remaining `Velocity` closed loop. The next slice
  should capture the full 98-dim deployed observation vector, raw action, and
  processed target near the first dynamic unstable sample, then compare that
  against mjlab training/play observation for the same pose/velocity state.
  Hardware remains locked.

### 7am. Dynamic Velocity Policy I/O Trace Gate - done, evidence captured

Commit:

- `diagnose(g1): capture dynamic velocity policy io trace`

Status:

- Done. The local-only policy I/O trace now captures dynamic samples around the
  first unstable Velocity sample, and the report can fail if no trace is near
  first instability.

Actions:

- Upgrade `g1_tracking_phase1_velocity_policy_io_trace_patch.py` from v1 to v2.
- Keep early policy-step trace coverage, but additionally log every 25 policy
  steps after dynamic motion appears, based on `joint_vel_l2`, root angular
  velocity, or projected-gravity drift.
- Add a report gate `--expect-near-first-unstable` so future evidence cannot
  pass with only early pre-dynamic traces.
- Apply the v2 patch to the local Unitree deploy runtime and rebuild `g1_ctrl`.
- Re-run the same official-aligned damped package smoke through
  `official_velocity_bootstrap`, including key-8 preparation and key-9 release.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_patch.py src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_report.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_patch.py src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_report.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `cmake --build /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/build --target g1_ctrl -j4`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260522-231619 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_after_key9_report.json --expect-failure`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_report.py --evidence-dir logs/flying_kick_sim2sim/20260522-231619 --deploy-yaml logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir/params/deploy.yaml --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_policy_io_trace_report.json --expect-trace --expect-near-first-unstable`
- `bash scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260522-231619`
- `pgrep -af '[u]nitree_mujoco|[g]1_ctrl|[f]lying_kick_sim|[f]lying_kick_ctrl'`

Acceptance gate:

- The v2 patch must be idempotent and able to upgrade a v1 patched file.
- The report must expose `selected_step_delta` and fail when
  `--expect-near-first-unstable` is requested but the nearest trace is farther
  than 25 policy steps from first instability.
- The new official-aligned smoke must produce a policy I/O report with
  `first_unstable_has_nearby_trace=true`.
- Do not unlock Mimic or hardware from this evidence; it is diagnostic only.

Evidence:

- Targeted pytest: `7 passed`.
- `ruff check`: `All checks passed`.
- `ruff format --check`: `3 files already formatted`.
- Local runtime patch result:
  `phase1_velocity_policy_io_trace_v2`, backup
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/isaaclab/envs/manager_based_rl_env.h.phase1_velocity_policy_io_trace_v2.bak`.
- `g1_ctrl` rebuild completed: `[100%] Built target g1_ctrl`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260522-231619/mujoco_official_velocity_bootstrap_v2_dynamic_trace_after_key9.png`.
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_after_key9_report.json`.
- Policy I/O trace report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_policy_io_trace_report.json`.
- The trace report now has `trace_count=216`, `selected_step_delta=0`, and
  `first_unstable_has_nearby_trace=true`. The selected trace is `step=925` /
  line 73, matching first unstable `policy_step=925` / line 74.
- Selected trace summary: 98-dim obs, `obs_l2=258.535`,
  `base_ang_vel=(5.589,-4.183,-9.676)`,
  `projected_gravity=(0.524,0.684,-0.508)`, zero velocity command,
  `joint_vel_l2=254.265`, `raw_action_l2=40.260`, and
  `processed_action_l2=15.820`.
- Top dynamic terms at failure: joint velocity index 19 `-198.109`, index 28
  `-133.036`, index 20 `-47.533`; raw action index 13 `-18.189`, index 20
  `-15.759`, index 14 `-14.885`; processed target index 13 `-7.985`, index 14
  `-6.535`, index 6 `-6.436`.
- Restore/cleanup leaves no active `g1_ctrl` or `unitree_mujoco` process.

Advance rule:

- Next work item should compare the selected dynamic deploy trace against the
  mjlab training/play-side observation construction for the same pose/velocity
  state, including term order, scaling, and whether raw action replay from the
  ONNX matches the deploy log. Hardware remains locked.

### 7an. Dynamic Trace ONNX Replay Comparator - done, inference mismatch unlikely

Commit:

- `diagnose(g1): replay dynamic velocity trace through onnx`

Status:

- Done. The captured first-unstable-near deploy observation can be replayed
  offline through the packaged ONNX, and the replay reproduces the C++ deploy
  log action to numerical tolerance.

Actions:

- Add `g1_tracking_phase1_velocity_trace_replay.py`.
- Load a policy I/O trace report, select `selected_trace`, infer the packaged
  `policy_dir`, run the selected 98-dim observation through
  `exported/policy.onnx`, and recompute processed joint targets from
  `params/deploy.yaml`.
- Compare replayed raw action and processed target against the C++ deploy log.
- Add counterfactual probes on the selected trace by zeroing `joint_vel_rel`,
  zeroing `last_action`, and resetting upright/zero-motion terms.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_trace_replay.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_trace_replay.py scripts/tools/g1_tracking_phase1_velocity_trace_replay.py tests/tools/test_g1_tracking_phase1_velocity_trace_replay.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_trace_replay.py scripts/tools/g1_tracking_phase1_velocity_trace_replay.py tests/tools/test_g1_tracking_phase1_velocity_trace_replay.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_trace_replay.py --trace-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_policy_io_trace_report.json --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_replay_report.json --expect-replay-match`

Acceptance gate:

- The tool must fail `--expect-replay-match` if logged deploy raw action differs
  from ONNX replay.
- The real selected dynamic trace must pass replay matching before we use its
  counterfactuals as diagnostic evidence.
- This work item does not unlock Mimic or hardware.

Evidence:

- Targeted pytest: `2 passed`.
- `ruff check`: `All checks passed`.
- `ruff format --check`: `3 files already formatted`.
- Trace replay report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_replay_report.json`.
- Real trace replay result: `replay_matches_deploy_log=true`,
  `raw_action_gap_l2=0.00000685`, `processed_action_gap_l2=0.00000328`,
  selected `step=925`, selected obs L2 `258.535`, replayed raw action L2
  `40.260`, replayed processed target L2 `15.820`.
- Counterfactuals from the same selected obs:
  - zero `joint_vel_rel`: raw action L2 `35.554`, processed target L2 `14.119`;
  - zero `last_action`: raw action L2 `15.409`, processed target L2 `6.737`;
  - reset base angular velocity, joint velocity, and projected gravity to
    upright/zero motion: raw action L2 `34.658`, processed target L2 `13.739`.
- Interpretation: the selected failure-step action is reproduced by the ONNX
  itself, so a gross C++ inference/logging mismatch is unlikely. At this already
  unstable sample, `last_action` dominates the action magnitude more than
  instantaneous joint velocity alone. The next diagnosis should inspect how
  deploy and mjlab training/play update `last_action`, phase, projected gravity,
  and joint velocities during the transition into the first dynamic frames.

Advance rule:

- Compare deploy trace chronology from Velocity entry through first dynamic
  instability against mjlab training/play observation construction, especially
  `last_action` reset/update semantics and whether paused/elastic/bootstrap
  frames feed recurrent previous-action terms differently from training.
  Hardware remains locked.

### 7ao. Dynamic Trace Chronology And Previous-Action Semantics - done, timing bug unlikely

Commit:

- `diagnose(g1): analyze velocity trace chronology`

Status:

- Done. The captured deploy trace now has a chronology report that compares
  early consecutive `last_action` values, zero-command command/phase masking,
  first dynamic threshold crossings, and source-level previous-action semantics
  between mjlab training/play and the C++ Unitree deploy loop.

Actions:

- Add `g1_tracking_phase1_velocity_trace_chronology.py`.
- Load a policy I/O trace report and check early consecutive deploy policy
  calls where exact `last_action(obs_N) == raw_action(N-1)` evidence is
  available.
- Check that zero velocity commands keep the deployed `gait_phase` observation
  masked to zero across the full trace.
- Audit source-order semantics:
  - Unitree deploy computes observation before processing the new ONNX action;
  - mjlab `env.step(action_t)` returns an observation containing `action_t`, so
    the next policy call also sees the previous raw action.
- Report the first logged crossings for `joint_vel_rel`, root angular velocity,
  projected-gravity drift, raw action, and `last_action`.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_trace_chronology.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_trace_chronology.py scripts/tools/g1_tracking_phase1_velocity_trace_chronology.py tests/tools/test_g1_tracking_phase1_velocity_trace_chronology.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_trace_chronology.py scripts/tools/g1_tracking_phase1_velocity_trace_chronology.py tests/tools/test_g1_tracking_phase1_velocity_trace_chronology.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_trace_chronology.py --trace-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_policy_io_trace_report.json --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_chronology_report.json --expect-early-last-action-match --expect-zero-command-phase-mask --expect-source-contract`

Acceptance gate:

- The tool must fail if early consecutive `last_action` values do not match
  the previous raw policy action.
- The real trace must pass zero-command / zero-gait-phase-mask checks before
  command-timing mismatch is ruled out for this run.
- The source audit must confirm deploy and mjlab agree on previous-action
  policy-call semantics.
- This work item does not unlock Mimic or hardware.

Evidence:

- Targeted pytest: `3 passed`.
- `ruff check`: `All checks passed`.
- `ruff format --check`: `3 files already formatted`.
- Real chronology report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v2_dynamic_trace_chronology_report.json`.
- Real trace decisions:
  `early_last_action_matches_previous_raw_action=true`,
  `zero_command_observation=true`,
  `zero_command_gait_phase_masked=true`, and
  `source_contract_matches_previous_action_policy_call_semantics=true`.
- Early consecutive matches: `obs_2..obs_5` `last_action` matches
  `raw_action_1..raw_action_4` with max L2 gap `0.0`.
- First logged dynamic crossings all occur at step `925`, the same policy step
  as the first unstable sample: `joint_vel_rel_l2=254.264691`,
  `root_ang_vel_l2=11.931852`, gravity drift L2 `0.992039`,
  `raw_action_l2=40.260391`, and `last_action_l2=44.553284`.
- Interpretation: a gross `last_action` lag bug, nonzero command leak, or
  gait-phase masking mismatch is unlikely for the early official-aligned
  Velocity trace. The remaining gap is not source semantics but observation
  sparsity: the current dynamic trace jumps from stable step 50 to the first
  already-unstable dynamic threshold at step 925. The next slice should capture
  denser evidence around MuJoCo Run/key-9 release and the first nonzero
  lowstate/sim motion update.

Advance rule:

- Add denser transition instrumentation before changing training, policy, or
  controller logic. The next accepted evidence must distinguish whether the
  first dynamic nonzero state comes from simulator/controller handoff timing,
  action target jump, contact impulse, DDS lowstate latency, or policy response.
  Hardware remains locked.

### 7ap. Dense Dynamic-Onset Velocity Trace Gate - done, blocker narrowed

Commit:

- `diagnose(g1): capture dense velocity onset trace`

Status:

- Done. The official-aligned `FixStand -> Velocity -> MuJoCo Run -> key-9`
  smoke now leaves a durable bootstrap helper log, captures dense policy I/O
  at the first low dynamic onset, and classifies the failure only after
  `Velocity` is reached.

Actions:

- Upgrade the local Unitree deploy trace patch from
  `phase1_velocity_policy_io_trace_v2` to
  `phase1_velocity_policy_io_trace_v3`.
- Keep early fixed trace samples, then start a dense 75-step window when
  `joint_vel_rel_l2 > 1.0`, `root_ang_vel_l2 > 0.05`, or projected gravity
  z drifts by more than `0.01` from upright.
- Add v2-to-v3 patch upgrade coverage so existing local external runtime files
  can be advanced without manual editing.
- Add optional automatic MuJoCo key-9 release after Run:
  `MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=1` plus
  `MJLAB_ELASTIC_RELEASE_DELAY_SECONDS`.
- Write `bootstrap_helper.log` for each single-action GUI smoke so the
  key-8/Run/key-9 sequence is not inferred from terminal scrollback.

Mandatory tests:

- `bash -n scripts/tools/run_flying_kick_sim2sim.sh`
- `bash -n scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `git diff --check`
- External runtime patch dry-run must report `changed=false` with marker
  `phase1_velocity_policy_io_trace_v3`.
- External `g1_ctrl` must rebuild after the v3 trace patch.
- Official-aligned GUI smoke must produce runtime, policy I/O, and chronology
  reports from the same evidence directory.

Acceptance gate:

- `FixStand` evidence is not accepted as a Velocity/tracking policy verdict.
- The official-aligned smoke must start in `FixStand`, transition into
  `Velocity`, request MuJoCo `Run` only after `Velocity` is ready, and record
  the key-8/Run/key-9 sequence in `bootstrap_helper.log`.
- The policy I/O report must match the 98-dim deploy observation schema and
  contain a trace at or near the first unstable `Velocity` step.
- The chronology report must continue to pass previous-raw-action,
  zero-command, zero-gait-phase-mask, and source-contract checks.
- Any failure remains a sim2sim Velocity runtime blocker and does not unlock
  Mimic or hardware.

Evidence:

- Syntax checks passed for both single-action sim2sim wrappers.
- Focused pytest: `31 passed`.
- `ruff check`: `All checks passed`.
- `ruff format --check`: `3 files already formatted`.
- `git diff --check`: passed.
- External patch dry-run reports the v3 marker with no further changes needed.
- External controller rebuild passed: `[100%] Built target g1_ctrl`.
- GUI evidence directory:
  `logs/flying_kick_sim2sim/20260523-000838/`.
- Screenshot evidence:
  `logs/flying_kick_sim2sim/20260523-000838/mujoco_official_velocity_bootstrap_v3_dense_auto_key9_with_helper_log.png`.
- Bootstrap helper log:
  `Prepared elastic length with MuJoCo key 8 before Run.`,
  `Released elastic band with MuJoCo key 9 after Run delay 1.0s.`, and
  `Requested MuJoCo Run after Velocity was ready`.
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_auto_key9_helper_report.json`.
- Policy I/O report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_helper_policy_io_trace_report.json`.
- Chronology report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_helper_trace_chronology_report.json`.
- The runtime report rejects the package with
  `primary_reason=velocity_runtime_instability`; first unstable sample is
  policy step `925`, line `83`, timestamp `2026-05-23 00:09:00.045`,
  `command_norm=0.0`, `raw_action_l2=46.018`,
  `processed_action_l2=18.470`, `joint_vel_l2=411.508`,
  `q_err_l2=16.660`, and `root_ang_vel_l2=7.793`.
- The policy I/O report has `obs_dim_matches=true`,
  `first_unstable_has_nearby_trace=true`, `selected_step_delta=0`, and
  `trace_count=707`.
- The chronology report shows low dynamic onset begins before the first
  unstable sample: step `916` has `joint_vel_rel_l2=172.637` and
  `root_ang_vel_l2=0.366`, then step `917` jumps to
  `joint_vel_rel_l2=678.490` and `root_ang_vel_l2=19.073`.
- Interpretation: the immediate question is no longer `FixStand` stability,
  command timing, or ONNX inference logging. The next diagnosis should focus on
  why zero-command `Velocity` produces large actions during the first physical
  dynamic onset under the new G1 mode-15 MuJoCo/deploy loop.

Advance rule:

- Do not tune Mimic/tracking or run hardware. Next work must isolate the
  zero-command `Velocity` dynamic-onset cause: contact impulse, action target
  jump, policy closed-loop sensitivity, lowstate/DDS timing, or remaining
  sim/deploy dynamics mismatch.

### 7aq. Dense Onset Ordering Analysis - done, previous-action-first cause unlikely

Commit:

- `diagnose(g1): classify velocity onset order`

Status:

- Done. The v3 dense policy I/O trace is now reduced to a machine-checkable
  ordering report that distinguishes first observed physical motion from the
  first large current raw action and first large previous-action term.

Actions:

- Add `g1_tracking_phase1_velocity_dense_onset.py`.
- Load a policy I/O trace report, identify the first low dynamic onset using
  the same v3 thresholds, and compare it with:
  - previous logged quiet trace;
  - first large current raw action;
  - first large `last_action` / previous-action term;
  - zero-command and zero-gait-phase maxima.
- Add CLI gates for zero-command/phase masking and for the expected ordering
  where observed motion appears before a large previous-action term.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_dense_onset.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_dense_onset.py scripts/tools/g1_tracking_phase1_velocity_dense_onset.py tests/tools/test_g1_tracking_phase1_velocity_dense_onset.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_dense_onset.py scripts/tools/g1_tracking_phase1_velocity_dense_onset.py tests/tools/test_g1_tracking_phase1_velocity_dense_onset.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_dense_onset.py --trace-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_helper_policy_io_trace_report.json --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_onset_order_report.json --expect-zero-command-phase-mask --expect-motion-before-large-previous-action`
- `git diff --check`

Acceptance gate:

- The report must fail if zero-command or gait-phase masking leaks into the
  trace.
- The report must fail the ordering gate when first dynamic onset already has
  a large `last_action`, because that would keep previous-action-first as a
  plausible cause.
- The real v3 trace must show first dynamic motion before the first large
  previous-action term before this slice can narrow the blocker.

Evidence:

- Focused pytest: `4 passed`.
- `ruff check`: `All checks passed`.
- `ruff format --check`: `3 files already formatted`.
- Real dense onset ordering report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_onset_order_report.json`.
- Real report decisions:
  `zero_command_observation=true`,
  `zero_command_gait_phase_masked=true`,
  `first_onset_has_quiet_previous_action=true`,
  `first_onset_current_raw_action_is_large=true`, and
  `observed_motion_precedes_large_previous_action=true`.
- First dynamic onset: step `916`, line `73`,
  `joint_vel_rel_l2=172.637`, `root_ang_vel_l2=0.366`,
  `last_action_l2=1.002`, and `raw_action_l2=11.219`.
- First large previous-action term: step `917`, line `74`,
  `last_action_l2=11.219`, `joint_vel_rel_l2=678.490`, and
  `root_ang_vel_l2=19.073`.
- Interpretation: the first observed motion is already present in the
  observation before the newly computed large action can be applied. A large
  previous-action term appears one policy call later. The next diagnostic
  target should be upstream of step `916`: contact impulse, elastic/key-9
  release impulse, lowstate/DDS timing, or MuJoCo/controller state handoff.

Advance rule:

- Do not train or touch Mimic yet. Add physical-side onset instrumentation or
  run controlled bootstrap variants that change only elastic release/contact
  conditions before changing policy rewards or deployment observation code.

### 7ar. No-Key9 Official Velocity Control - done, key9 release not sufficient

Commit:

- `docs(g1): record no-key9 velocity onset control`

Status:

- Done. A controlled official-aligned GUI smoke changed only the automatic
  key-9 release condition. The run still fails at the same first dynamic onset
  step, so automatic key-9 release is not the sole trigger.

Actions:

- Run `official_velocity_bootstrap` with the same explicit damped Velocity
  policy package, policy-default joints, home root, elastic band enabled,
  key-8 preparation, and `MJLAB_AUTO_RUN_AFTER_READY=1`.
- Set `MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=0` so no MuJoCo key `9` is sent.
- Capture screenshot evidence, helper log, runtime report, policy I/O report,
  and dense onset ordering report.
- Restore the external Unitree runtime config and confirm no simulator or
  controller process remains active.

Mandatory tests:

- `scripts/tools/run_flying_kick_sim2sim.sh start` with
  `MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=0` and official Velocity bootstrap
  settings.
- `scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260523-002911`
- `file logs/flying_kick_sim2sim/20260523-002911/mujoco_official_velocity_bootstrap_v3_dense_no_key9.png`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260523-002911 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_report.json --expect-failure`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_report.py --evidence-dir logs/flying_kick_sim2sim/20260523-002911 --deploy-yaml logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir/params/deploy.yaml --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_policy_io_trace_report.json --expect-trace --expect-near-first-unstable`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_dense_onset.py --trace-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_policy_io_trace_report.json --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_onset_order_report.json --expect-zero-command-phase-mask --expect-motion-before-large-previous-action`
- `pgrep -af '^\\./g1_ctrl|^\\./simulate/build/unitree_mujoco'` returns no
  process after restore.
- `git diff --check`

Acceptance gate:

- Helper log must prove key-8 preparation and Run request, while also proving
  key `9` was not sent automatically.
- The runtime report must still classify any failure as zero-command
  `Velocity` instability, not `FixStand`, Mimic, or command tracking.
- The onset ordering report must show whether the first dynamic onset changes
  when key `9` is withheld.

Evidence:

- GUI evidence directory:
  `logs/flying_kick_sim2sim/20260523-002911/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260523-002911/mujoco_official_velocity_bootstrap_v3_dense_no_key9.png`
  (`1706 x 960` PNG).
- Bootstrap helper log records key-8 preparation, manual key-9 instruction, and
  Run after `Velocity` was ready; it does not record automatic key `9`.
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_report.json`.
- Policy I/O report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_policy_io_trace_report.json`.
- Dense onset ordering report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_key9_onset_order_report.json`.
- The run still fails at policy step `925`, line `83`,
  `primary_reason=velocity_runtime_instability`, `command_norm=0.0`,
  `raw_action_l2=42.277`, `processed_action_l2=16.660`,
  `joint_vel_l2=311.550`, `q_err_l2=15.879`, and
  `root_ang_vel_l2=14.334`.
- First dynamic onset is unchanged at step `916`, line `73`:
  `joint_vel_rel_l2=172.637`, `root_ang_vel_l2=0.366`,
  `last_action_l2=1.002`, and `raw_action_l2=11.219`.
- Interpretation: automatic key-9 release is not sufficient to explain the
  first physical onset. The next controlled slice should inspect contact/key-8
  pre-tension state, paused-to-run state handoff, DDS/lowstate timing, or
  MuJoCo/deploy dynamics before the policy response at step `916`.

Advance rule:

- Continue with physical-side instrumentation or one-variable controlled
  bootstraps. Do not unlock Mimic or hardware.

### 7as. No-Elastic Official Velocity Onset Control - done, blocker found

Commit:

- `diagnose(g1): classify no-elastic velocity onset`

Status:

- Implemented. GUI smoke passed as diagnostic evidence and still rejects the
  package in zero-command `Velocity`.

Actions:

- Run the same official-aligned `FixStand -> Velocity -> Run` bootstrap and
  damped StandFirst deploy98 package, but set `MJLAB_ENABLE_ELASTIC_BAND=0`.
- Capture screenshot evidence, helper log, runtime report, policy I/O report,
  and dense onset ordering report.
- Extend the dense onset classifier to distinguish a first observed dynamic
  onset where both previous action and current raw action are still quiet. This
  is stronger evidence that motion appears before a large policy response.
- Restore the external Unitree runtime config and confirm no simulator or
  controller process remains active.

Mandatory tests:

- `scripts/tools/run_flying_kick_sim2sim.sh start` with
  `MJLAB_SIM2SIM_MODE=official_velocity_bootstrap`,
  `MJLAB_ENABLE_ELASTIC_BAND=0`, `MJLAB_AUTO_RUN_AFTER_READY=1`,
  `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`,
  `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`, and the damped StandFirst package root.
- `scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260523-004041`
- `file logs/flying_kick_sim2sim/20260523-004041/mujoco_official_velocity_bootstrap_v3_dense_no_elastic.png`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260523-004041 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_report.json --expect-failure`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_report.py --evidence-dir logs/flying_kick_sim2sim/20260523-004041 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_policy_io_trace_report.json --expect-trace --expect-near-first-unstable`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_dense_onset.py --trace-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_policy_io_trace_report.json --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_onset_order_report.json --expect-zero-command-phase-mask --expect-motion-before-large-previous-action`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_dense_onset.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_dense_onset.py tests/tools/test_g1_tracking_phase1_velocity_dense_onset.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_dense_onset.py tests/tools/test_g1_tracking_phase1_velocity_dense_onset.py`
- `pgrep -af '^\\./g1_ctrl|^\\./simulate/build/unitree_mujoco'` returns no
  process after restore.
- `git diff --check`

Acceptance gate:

- Helper log must prove MuJoCo Run was requested only after `Velocity` was
  ready.
- Selected sim config must show `enable_elastic_band: 0`.
- Runtime report must still classify any failure as zero-command `Velocity`
  instability.
- Dense onset report must classify whether the first dynamic onset appears
  before a large previous action or before a large current policy response.

Evidence:

- GUI evidence directory:
  `logs/flying_kick_sim2sim/20260523-004041/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260523-004041/mujoco_official_velocity_bootstrap_v3_dense_no_elastic.png`
  (`1706 x 960` PNG).
- Bootstrap helper log records Run after `Velocity` was ready. It records no
  key-8 or key-9 actions because elastic band was disabled.
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_report.json`.
- Policy I/O report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_policy_io_trace_report.json`.
- Dense onset ordering report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_v3_dense_no_elastic_onset_order_report.json`.
- The run still fails at policy step `850`, line `81`,
  `primary_reason=velocity_runtime_instability`, `command_norm=0.0`,
  `raw_action_l2=43.056`, `processed_action_l2=17.724`,
  `joint_vel_l2=367.571`, `q_err_l2=14.467`, and
  `root_ang_vel_l2=16.107`. Stable duration before first unstable sample is
  `19.5s`.
- Dense onset report classifies `observed_motion_before_policy_response`.
  First dynamic onset is step `840`: `joint_vel_rel_l2=11.949`,
  `root_ang_vel_l2=0.173`, `last_action_l2=1.002`, and
  `raw_action_l2=0.831`. First large current raw action appears at step `841`,
  and first large previous action appears at step `842`.
- Interpretation: enabled elastic band and key-9 release are not sufficient
  root causes. The remaining target is a physical/deploy-side onset before a
  large policy response: paused-to-run handoff, contact/settling impulse,
  DDS/lowstate timing, or a broader MuJoCo/controller closed-loop mismatch.

Advance rule:

- Continue with physical-side transition instrumentation. Do not unlock Mimic
  or hardware.

### 7at. MuJoCo Physics Transition Trace Patcher - done, ready for runtime application

Commit:

- `diagnose(g1): add mujoco transition trace patcher`

Status:

- Implemented. This is repo-side tooling only; it does not write `.external`,
  rebuild Unitree MuJoCo, launch GUI, or change hardware state.

Actions:

- Add a repo-local patcher for
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/main.cc`.
- Inject a local-only `phase1_mujoco_transition_trace_v1` helper that logs
  `[PHASE1_SIM] event=mujoco_transition_trace` after both `mj_step` paths.
- Capture sim step, sim time, root position, root linear/angular velocity,
  `qvel` L2/max, actuator `ctrl` L2/max, contact count, elastic-band config,
  elastic enabled flag, elastic length, and elastic force.
- Keep the patcher idempotent and strict: it refuses to patch if the expected
  two `mj_step` anchors are not present.
- Add a wrapper CLI under `scripts/tools/` and focused tests for injection,
  idempotence, anchor refusal, and dry-run no-write behavior.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_mujoco_transition_trace_patch.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_mujoco_transition_trace_patch.py scripts/tools/g1_tracking_phase1_mujoco_transition_trace_patch.py tests/tools/test_g1_tracking_phase1_mujoco_transition_trace_patch.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_mujoco_transition_trace_patch.py scripts/tools/g1_tracking_phase1_mujoco_transition_trace_patch.py tests/tools/test_g1_tracking_phase1_mujoco_transition_trace_patch.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_mujoco_transition_trace_patch.py --mujoco-main /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/main.cc`
- `git diff --check`

Acceptance gate:

- Unit tests pass and prove the patcher is idempotent.
- Dry-run on the real local MuJoCo `main.cc` reports `changed=true` and
  `write=false`.
- `.external` remains unmodified by this work item.
- No MuJoCo GUI, controller, or real robot process is launched.
- The next commit-sized work item may apply this patch, rebuild the local
  simulator, and run the same official-aligned no-elastic Velocity smoke with
  synchronized controller and MuJoCo-side traces.

Evidence:

- Targeted pytest: 4 passed.
- Ruff check: all checks passed.
- Ruff format check: 3 files already formatted.
- Real `main.cc` dry-run report:
  `changed=true`, `write=false`,
  `marker=phase1_mujoco_transition_trace_v1`.
- `git diff --check`: passed.

Advance rule:

- Next work may apply the patch to `.external`, rebuild `unitree_mujoco`, run
  the no-elastic official Velocity smoke, and parse `[PHASE1_SIM]` next to the
  existing controller policy I/O trace. Do not unlock Mimic or hardware.

### 7au. MuJoCo Transition Trace Runtime Smoke - done, blocker narrowed

Commit:

- `diagnose(g1): analyze mujoco transition trace`

Status:

- Implemented. The external MuJoCo runtime is currently instrumented locally
  and rebuilt from a backed-up source file. No `.external` source, binary, log,
  screenshot, or generated report is committed.

Actions:

- Apply the `phase1_mujoco_transition_trace_v1` patch to local
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/main.cc`.
- Rebuild `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/build/unitree_mujoco`.
- Run the same official-aligned no-elastic `FixStand -> Velocity -> Run`
  smoke with the damped StandFirst deploy98 package.
- Capture screenshot evidence and restore active deploy/sim configs.
- Add a repo-local MuJoCo trace report parser that extracts first dynamic
  physics step, first contact, first large ctrl, elastic force state, and the
  ordering among them.
- Generate runtime, policy I/O, dense onset, and MuJoCo transition reports for
  the evidence directory.

Mandatory tests:

- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_mujoco_transition_trace_patch.py --apply --mujoco-main /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/main.cc`
- `cmake --build /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/build -- -j8`
- `scripts/tools/run_flying_kick_sim2sim.sh start` with
  `MJLAB_SIM2SIM_MODE=official_velocity_bootstrap`,
  `MJLAB_ENABLE_ELASTIC_BAND=0`, `MJLAB_AUTO_RUN_AFTER_READY=1`,
  `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`,
  `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`, and the damped StandFirst package root.
- Screenshot capture:
  `logs/flying_kick_sim2sim/20260523-010629/mujoco_official_velocity_bootstrap_mujoco_trace_no_elastic.png`
- `scripts/tools/run_flying_kick_sim2sim.sh restore logs/flying_kick_sim2sim/20260523-010629`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_contract.py --evidence-dir logs/flying_kick_sim2sim/20260523-010629 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_report.json --expect-failure`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_report.py --evidence-dir logs/flying_kick_sim2sim/20260523-010629 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_policy_io_trace_report.json --expect-trace --expect-near-first-unstable`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_velocity_dense_onset.py --trace-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_policy_io_trace_report.json --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_onset_order_report.json --expect-zero-command-phase-mask --expect-motion-before-large-previous-action`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_mujoco_transition_trace_report.py --evidence-dir logs/flying_kick_sim2sim/20260523-010629 --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_mujoco_transition_trace_report.json --expect-trace --expect-first-step-dynamic --expect-motion-before-contact --expect-motion-before-large-ctrl`
- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_mujoco_transition_trace_report.py tests/tools/test_g1_tracking_phase1_mujoco_transition_trace_patch.py`
- `uv run --active --no-sync ruff check <changed-python-files> <changed-tests>`
- `uv run --active --no-sync ruff format --check <changed-python-files> <changed-tests>`
- `pgrep -af '^\\./g1_ctrl|^\\./simulate/build/unitree_mujoco'` returns no
  process after restore.
- `git diff --check`

Acceptance gate:

- The local `.external` source patch writes a backup and `unitree_mujoco`
  rebuilds successfully.
- The smoke reaches `Velocity`, requests MuJoCo Run after `Velocity` readiness,
  and leaves no simulator/controller process after restore.
- The MuJoCo report must contain `[PHASE1_SIM]` traces and classify whether
  the first physical motion is before contact and before large ctrl.
- The controller reports must still show zero-command `Velocity` failure, not
  Mimic or hardware evidence.
- If the first physics-step motion precedes both contact and large ctrl, the
  next diagnosis must target Run handoff, initial support/floating state, or
  low-level command state at Run. Do not unlock Mimic or hardware.

Evidence:

- External patch report:
  `backup_written=true`, `changed=true`,
  `marker=phase1_mujoco_transition_trace_v1`.
- Rebuild: `[100%] Built target unitree_mujoco`.
- GUI evidence directory:
  `logs/flying_kick_sim2sim/20260523-010629/`.
- Screenshot:
  `logs/flying_kick_sim2sim/20260523-010629/mujoco_official_velocity_bootstrap_mujoco_trace_no_elastic.png`
  (`1706 x 960` PNG).
- Runtime report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_report.json`.
- Policy I/O report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_policy_io_trace_report.json`.
- Dense onset report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_onset_order_report.json`.
- MuJoCo transition report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_mujoco_transition_trace_report.json`.
- Velocity runtime still fails at policy step `850`, line `80`,
  `primary_reason=velocity_runtime_instability`, `command_norm=0.0`,
  `raw_action_l2=25.397`, `processed_action_l2=11.209`,
  `joint_vel_l2=279.434`, `q_err_l2=9.165`, and
  `root_ang_vel_l2=7.982`.
- Controller dense onset classifies
  `observed_motion_before_large_previous_action`: first logged dynamic onset is
  policy step `841`, with `joint_vel_rel_l2=616.677`,
  `root_ang_vel_l2=0.510`, quiet previous action
  (`last_action_l2=1.002`), and already-large current raw action
  (`raw_action_l2=18.069`).
- MuJoCo transition report has `trace_count=603` and classifies:
  `first_physics_step_is_dynamic=true`, `motion_before_contact=true`,
  `motion_before_large_ctrl=true`, and `elastic_force_disabled=true`.
  First dynamic trace is sim step `1` / `0.002s` with `qvel_l2=11.950`,
  `root_ang_vel_l2=0.173`, `ctrl_l2=17.644`, `ncon=0`, and
  `elastic_force_l2=0.0`. First large ctrl appears at sim step `5`
  (`ctrl_l2=199.836`), and first contact appears at sim step `8`
  (`ncon=2`).
- Interpretation: the earliest physical motion appears immediately after Run
  and before contact or a large actuator command. Contact impulse, enabled
  elastic, key-9 release, and later large policy output are no longer
  sufficient root causes. The next target is Run handoff / initial support
  state / low-level command state at the instant physics starts.

Advance rule:

- Continue with a handoff/support-state probe. Do not tune Mimic/tracking or
  run hardware.

### 7av. Run Handoff Support-State Audit - done, startup evidence contaminated

Commit:

- `diagnose(g1): audit velocity run handoff`

Status:

- Done. The latest no-elastic paused official-aligned evidence is now
  classified as a Run-boundary evidence issue, not clean Velocity policy
  acceptance evidence.

Actions:

- Add a repo-local audit tool that combines the runtime Velocity report, policy
  I/O trace report, MuJoCo transition report, and optional Unitree bridge source.
- Detect whether the policy thread advanced while MuJoCo physics was paused.
- Detect whether the initial support state is floating before Run.
- Detect whether the first MuJoCo physics step is dynamic, contact-free, and
  already using nonzero low-level ctrl.
- Record the bridge-side `ctrl` semantics so the report distinguishes MuJoCo
  actuator torque from raw policy action.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_run_handoff_audit.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_run_handoff_audit.py tests/tools/test_g1_tracking_phase1_run_handoff_audit.py scripts/tools/g1_tracking_phase1_run_handoff_audit.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_run_handoff_audit.py tests/tools/test_g1_tracking_phase1_run_handoff_audit.py scripts/tools/g1_tracking_phase1_run_handoff_audit.py`
- `uv run --active --no-sync python scripts/tools/g1_tracking_phase1_run_handoff_audit.py --evidence-dir logs/flying_kick_sim2sim/20260523-010629 --velocity-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_report.json --policy-io-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_policy_io_trace_report.json --mujoco-transition-report logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_mujoco_transition_trace_report.json --bridge-source /home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/unitree_sdk2_bridge.h --report-out logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_run_handoff_audit.json --expect-paused-policy --expect-support-gap --expect-first-step-dynamic --expect-first-step-no-contact`
- `git diff --check`

Acceptance gate:

- The audit parser must pass focused tests covering positive and negative
  classifications.
- The real evidence report must classify the latest run using current JSON/log
  artifacts, not hard-coded values.
- The report must not claim Velocity acceptance; it must mark hardware and Mimic
  gates locked if startup handoff flags are present.

Evidence:

- Test result: `3 passed` in
  `tests/tools/test_g1_tracking_phase1_run_handoff_audit.py`.
- `ruff check`: all checks passed.
- `ruff format --check`: 3 files already formatted.
- Run handoff report:
  `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_official_bootstrap_mujoco_trace_no_elastic_run_handoff_audit.json`.
- Classification: `paused_policy_with_floating_support_handoff`.
- Decision flags: `start_paused=true`,
  `policy_steps_while_physics_paused=true`, `support_gap_before_run=true`,
  `first_physics_step_is_dynamic=true`, `first_step_no_contact=true`,
  `first_step_nonzero_ctrl=true`, `motion_before_contact=true`,
  `motion_before_large_ctrl=true`, `elastic_force_disabled=true`.
- Frozen policy evidence: policy I/O traces advance to step `50` while
  `joint_vel_l2=0.0`, `root_ang_vel_l2=0.0`, zero command and zero gait phase
  remain true.
- Support evidence: `min_foot_surface_z=0.027211`, above the `0.005m` support
  gap threshold.
- First-step MuJoCo evidence: first physics step has `qvel_l2=11.950`,
  `ctrl_l2=17.644`, and `ncon=0`.
- Bridge source evidence: `formula_found=true`; MuJoCo `ctrl` is the low-level
  motor torque computed from `lowcmd` tau plus `kp/kd` position and velocity
  error.

Interpretation:

- The latest no-elastic paused evidence is contaminated by startup handoff
  state: policy stepping occurred while MuJoCo was paused, the robot started
  with a measurable foot-floor support gap, and the first physics step was
  already dynamic with no contact and nonzero low-level ctrl.
- This does not mean the damped StandFirst Velocity policy is accepted. It means
  this run cannot be used as a clean policy-quality verdict.
- `FixStand` remains a PD pose/interpolation bootstrap, not the acceptance
  policy. Direct `Passive -> Velocity` remains diagnostic. The next primary
  sim2sim gate must follow the official `FixStand -> Velocity` route while
  proving contact/settle and controller start alignment at MuJoCo `Run`.

Advance rule:

- Next slice should change or instrument the official sim2sim runner so policy
  stepping cannot silently advance during paused physics acceptance, or so the
  acceptance gate starts only after a proven contact/settled state. Do not tune
  Mimic/tracking or unlock hardware.

### 7aw. Velocity Policy Start Gate - done, handoff still contaminated

Commit:

- `diagnose(g1): add velocity policy start gate`

Status:

- Done. The local gate delays Velocity policy stepping after state entry, but
  the latest GUI smoke still fails the clean handoff gate.

Actions:

- Add a repo-local patcher for the machine-local Unitree
  `deploy/include/FSM/State_RLBase.h`.
- Inject an opt-in `MJLAB_PHASE1_POLICY_START_GATE_SECONDS` delay after
  `env->reset()` and before the Velocity policy thread begins `env->step()`.
- Pass that environment variable through both single-action sim2sim wrappers.
- Default the gate to `5.0s` only for paused, auto-run Velocity bootstrap modes.
- Harden auto-run log polling so the helper can detect already-completed
  `FixStand/Passive -> Velocity` transitions and does not require `rg`.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_policy_start_gate_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_start_gate_patch.py tests/tools/test_g1_tracking_phase1_velocity_policy_start_gate_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py scripts/tools/g1_tracking_phase1_velocity_policy_start_gate_patch.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_policy_start_gate_patch.py tests/tools/test_g1_tracking_phase1_velocity_policy_start_gate_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py scripts/tools/g1_tracking_phase1_velocity_policy_start_gate_patch.py`
- Apply the local patch to
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/FSM/State_RLBase.h`,
  rebuild
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl`,
  and verify the binary contains the new gate strings.
- Run one official-aligned GUI smoke with:
  `MJLAB_SIM2SIM_MODE=official_velocity_bootstrap`,
  `MJLAB_AUTO_RUN_AFTER_READY=1`,
  `MJLAB_PHASE1_POLICY_START_GATE_SECONDS=5.0`,
  `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`,
  `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`,
  `MJLAB_ENABLE_ELASTIC_BAND=1`,
  `MJLAB_ELASTIC_PRETENSION_STEPS=24`, and
  `MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=0`.
- Stop and restore the sim2sim runtime after the smoke.
- `git diff --check`

Acceptance gate:

- Unit tests cover idempotent patching, missing-anchor rejection, CLI dry-run,
  CLI apply, wrapper default/override behavior, and shell-unsafe gate rejection.
- The external controller binary is rebuilt from the patched header and contains
  `MJLAB_PHASE1_POLICY_START_GATE_SECONDS` plus the two gate log strings.
- The GUI evidence shows the gate starts and releases before policy stepping
  proceeds beyond `policy_step=0`.
- The run handoff audit must explicitly classify whether the gate solved or
  only narrowed the paused-policy handoff problem.
- Hardware and Mimic gates remain locked unless the audit produces clean
  Velocity acceptance evidence.

Evidence:

- Focused pytest: `30 passed` for
  `tests/tools/test_g1_tracking_phase1_velocity_policy_start_gate_patch.py` and
  `tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`.
- `ruff check`: all checks passed for the patcher, wrapper test, and CLI shim.
- `ruff format --check`: 4 files already formatted.
- External patch dry-run after application reports `changed=false` because
  `phase1_velocity_policy_start_gate_v1` is already present.
- External rebuild result: `g1_ctrl` built successfully; binary strings include
  `MJLAB_PHASE1_POLICY_START_GATE_SECONDS`,
  `[PHASE1] event=policy_start_gate delay_seconds={:.3f}`, and
  `[PHASE1] event=policy_start_gate_release running={}`.
- GUI smoke evidence: `logs/flying_kick_sim2sim/20260523-015505/`.
- Reports:
  - `logs/g1_tracking_phase1/2026-05-23T01-55-05+08-00/velocity_official_gate_bootstrap_report.json`
  - `logs/g1_tracking_phase1/2026-05-23T01-55-05+08-00/velocity_official_gate_mujoco_transition_report.json`
  - `logs/g1_tracking_phase1/2026-05-23T01-55-05+08-00/velocity_official_gate_policy_io_report.json`
  - `logs/g1_tracking_phase1/2026-05-23T01-55-05+08-00/velocity_official_gate_run_handoff_audit.json`
- Controller log gate proof: `policy_start_gate` at `01:55:08.588`,
  `policy_step=0` at `01:55:08.589`, and `policy_start_gate_release` at
  `01:55:13.591`.
- Runtime rejection: first unstable sample is line `76` / policy step `675` /
  `2026-05-23 01:55:27.072`, with `command_norm=0.0`, `q_err_l2=4.650`,
  `joint_vel_l2=327.436`, `raw_action_l2=11.720`, and
  `root_ang_vel_l2=10.668`.
- Handoff audit classification: `paused_policy_handoff`.
- Handoff flags: `start_paused=true`,
  `policy_steps_while_physics_paused=true`,
  `support_gap_before_run=false`, `first_step_no_contact=false`,
  `first_step_nonzero_ctrl=true`, `first_physics_step_is_dynamic=true`,
  `motion_before_contact=false`, `motion_before_large_ctrl=true`, and
  `elastic_force_disabled=false`.
- Frozen policy evidence after release: traces advance through step `50` while
  `joint_vel_l2=0.0`, `root_ang_vel_l2=0.0`, zero command, and zero gait phase
  remain true.

Interpretation:

- The gate fixes one concrete issue: the policy thread no longer advances before
  the helper requests MuJoCo `Run` and releases the gate.
- The gate does not prove Velocity policy quality or sim2sim acceptance. After
  release, policy stepping can still advance on frozen or stale lowstate for the
  first few samples, and the run later fails dynamically under zero command.
- The next root target is policy-clock alignment with MuJoCo physics/DDS
  lowstate updates, not `FixStand` and not Mimic/tracking quality.

Advance rule:

- Next slice should add or verify a stronger synchronization condition: policy
  stepping should wait for fresh MuJoCo/DDS state after `Run`, or the runner
  should start acceptance only after physics/contact/lowstate update has been
  proven current. Do not tune Mimic/tracking or unlock hardware.

### 7ax. Velocity Lowstate Tick Gate - done, cadence fixed but stability rejected

Commit:

- `diagnose(g1): gate velocity policy on lowstate tick`

Status:

- Done. The local Velocity policy loop now has an opt-in lowstate tick gate that
  waits for fresh Unitree `LowState.tick()` samples before `env->step()`. The
  GUI smoke proves the gate fixes paused/frozen policy-clock advancement and
  post-wait catch-up bursts, but the Velocity policy still fails the stability
  gate under zero command.

Actions:

- Add a repo-local patcher for the machine-local Unitree
  `deploy/include/FSM/State_RLBase.h`.
- Require the already-applied start-gate marker before injecting the lowstate
  tick gate.
- Add opt-in env vars:
  `MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE` and
  `MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS`.
- Read `lowstate->msg_.tick()` under `lowstate->mutex_`.
- Wait for a fresh tick before each Velocity `env->step()` when the gate is
  enabled.
- Reset `sleepTill = clock::now() + dt` after each successful tick wait so a
  long pause cannot cause accumulated policy-step catch-up.
- Pass the env vars through both single-action sim2sim wrappers and validate
  their shell inputs.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py tests/tools/test_g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py scripts/tools/g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py tests/tools/test_g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py scripts/tools/g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`
- Apply the local patch to
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/FSM/State_RLBase.h`,
  rebuild
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/build/g1_ctrl`,
  and verify the patch is idempotent.
- Run one official-aligned GUI smoke with:
  `MJLAB_SIM2SIM_MODE=official_velocity_bootstrap`,
  `MJLAB_AUTO_RUN_AFTER_READY=1`,
  `MJLAB_PHASE1_POLICY_START_GATE_SECONDS=5.0`,
  `MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE=1`,
  `MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS=0.5`,
  `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`,
  `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`,
  `MJLAB_ENABLE_ELASTIC_BAND=1`,
  `MJLAB_ELASTIC_PRETENSION_STEPS=24`, and
  `MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=0`.
- Stop and restore the sim2sim runtime after the smoke.
- `git diff --check`

Acceptance gate:

- Unit tests cover idempotent patching, missing prerequisite rejection, existing
  old-marker repair, CLI dry-run/apply behavior, and wrapper validation.
- The external controller binary is rebuilt from the patched header and the
  post-apply dry-run reports `changed=false`.
- The GUI evidence shows policy stepping remains at `policy_step=0` while
  `LowState.tick()` is stale, then advances at roughly 50Hz after fresh ticks
  arrive.
- The run handoff audit must no longer classify the evidence as paused policy
  advancement. It must still reject the run if the robot is dynamically unstable
  after clean handoff.
- Hardware and Mimic gates remain locked unless the audit produces clean
  Velocity acceptance evidence.

Evidence:

- Focused pytest: `34 passed` for
  `tests/tools/test_g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py`
  and `tests/tools/test_g1_tracking_phase1_sim2sim_wrapper.py`.
- `ruff check`: all checks passed for the lowstate tick patcher and focused
  tests.
- External patch dry-run after application reports `changed=false`.
- External rebuild result: `g1_ctrl` built successfully.
- GUI smoke evidence: `logs/flying_kick_sim2sim/20260523-023332/`.
- Reports:
  - `logs/g1_tracking_phase1/2026-05-23T02-33-32+08-00/velocity_contract.json`
  - `logs/g1_tracking_phase1/2026-05-23T02-33-32+08-00/mujoco_transition_trace.json`
  - `logs/g1_tracking_phase1/2026-05-23T02-33-32+08-00/policy_io_trace.json`
  - `logs/g1_tracking_phase1/2026-05-23T02-33-32+08-00/run_handoff_audit.json`
- Gate proof: `policy_start_gate` begins at `02:33:35.020`, releases at
  `02:33:40.023`, `lowstate_tick_gate_start tick=0` follows immediately, and
  repeated timeouts keep `policy_step=0` until the first fresh tick release at
  `02:33:53.303`.
- Cadence proof after fresh tick: `policy_step=350` at `02:34:00.304`,
  `policy_step=375` at `02:34:00.806`, and `policy_step=400` at
  `02:34:01.307`, matching roughly 25 policy steps per 0.5s.
- Runtime rejection: first unstable Velocity sample is line `91` /
  `policy_step=25` / `2026-05-23 02:33:53.785`, with `command_norm=0.0`,
  `q_err_l2=4.773`, `joint_vel_l2=372.938`,
  `raw_action_l2=9.300`, and `root_ang_vel_l2=8.406`.
- Handoff audit classification: `lowcmd_ctrl_handoff`.
- Handoff flags: `policy_steps_while_physics_paused=false`,
  `support_gap_before_run=false`, `first_step_no_contact=false`,
  `first_step_nonzero_ctrl=true`, `first_physics_step_is_dynamic=true`, and
  `motion_before_large_ctrl=true`.

Interpretation:

- This commit fixes the non-model clock handoff issue that previously made
  Velocity evidence invalid: the policy no longer advances through frozen
  lowstate while MuJoCo physics is paused, and it no longer catch-up bursts after
  a long wait.
- The run still rejects Velocity stability. The remaining failure is no longer
  explained by `FixStand` alone, paused policy advancement, missing contact, or
  enabled elastic band. The next target is the low-level command/ctrl state at
  the first physics steps and the deploy policy's zero-command stabilization
  behavior under the exact new-G1 MuJoCo scene.

Advance rule:

- Next slice should instrument or reconcile low-level `LowCmd`/MuJoCo `ctrl`
  at the Run boundary and early Velocity samples. Do not treat this as clean
  policy-quality evidence and do not unlock hardware.

### 7ay. Lowcmd Ctrl Handoff Trace - done, position PD handoff proven

Commit:

- `diagnose(g1): trace lowcmd ctrl handoff`

Status:

- Done. The local Unitree MuJoCo bridge now has a reversible trace patcher and
  report parser that decompose `lowcmd` into MuJoCo `ctrl` contributions:
  commanded torque, position-PD term, velocity-PD term, joint errors, and the
  dominant motor. The latest official-aligned smoke proves the first nonzero
  control is a position-PD handoff from mismatched `LowCmd.q` and sensed joint
  position, not policy torque.

Actions:

- Add a repo-local patcher for the machine-local Unitree
  `simulate/src/unitree_sdk2_bridge.h`.
- Inject `[PHASE1_SIM] event=lowcmd_ctrl_trace` lines after the bridge computes
  `ctrl = tau + kp * (q_cmd - q_sensor) + kd * (dq_cmd - dq_sensor)`.
- Add a report parser that classifies first nonzero control source as
  position-PD, velocity-PD, mixed-PD, commanded torque, no trace, or no nonzero
  control.
- Teach the policy I/O report resolver to descend into the latest policy child
  directory when the caller passes a policy root such as `config/policy/velocity`
  instead of a concrete `.../v0`.
- Apply the local bridge patch, rebuild `unitree_mujoco`, run one
  official-aligned Velocity smoke, stop and restore the runtime, and parse all
  reports from that evidence directory.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_lowcmd_ctrl_trace_patch.py tests/tools/test_g1_tracking_phase1_lowcmd_ctrl_trace_report.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_lowcmd_ctrl_trace_patch.py src/mjlab/scripts/g1_tracking_phase1_lowcmd_ctrl_trace_report.py scripts/tools/g1_tracking_phase1_lowcmd_ctrl_trace_patch.py scripts/tools/g1_tracking_phase1_lowcmd_ctrl_trace_report.py src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_report.py tests/tools/test_g1_tracking_phase1_lowcmd_ctrl_trace_patch.py tests/tools/test_g1_tracking_phase1_lowcmd_ctrl_trace_report.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_lowcmd_ctrl_trace_patch.py src/mjlab/scripts/g1_tracking_phase1_lowcmd_ctrl_trace_report.py scripts/tools/g1_tracking_phase1_lowcmd_ctrl_trace_patch.py scripts/tools/g1_tracking_phase1_lowcmd_ctrl_trace_report.py src/mjlab/scripts/g1_tracking_phase1_velocity_policy_io_trace_report.py tests/tools/test_g1_tracking_phase1_lowcmd_ctrl_trace_patch.py tests/tools/test_g1_tracking_phase1_lowcmd_ctrl_trace_report.py tests/tools/test_g1_tracking_phase1_velocity_policy_io_trace.py`
- Apply the local patch to
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/unitree_sdk2_bridge.h`,
  rebuild
  `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/build/unitree_mujoco`,
  and verify the patch is idempotent.
- Run one official-aligned GUI smoke with:
  `MJLAB_SIM2SIM_MODE=official_velocity_bootstrap`,
  `MJLAB_AUTO_RUN_AFTER_READY=1`,
  `MJLAB_PHASE1_POLICY_START_GATE_SECONDS=5.0`,
  `MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE=1`,
  `MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS=0.5`,
  `MJLAB_VELOCITY_BOOTSTRAP_POSE=policy_default`,
  `MJLAB_VELOCITY_BOOTSTRAP_ROOT=home`,
  `MJLAB_ENABLE_ELASTIC_BAND=1`,
  `MJLAB_ELASTIC_PRETENSION_STEPS=24`, and
  `MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN=0`.
- Stop and restore the sim2sim runtime after the smoke.
- `git diff --check`

Acceptance gate:

- Unit tests cover idempotent patching, backup creation, existing-marker
  detection, missing-pattern refusal, wrapper CLI behavior, report
  classification, expectation failures, and policy-root child resolution.
- The external MuJoCo simulator binary is rebuilt from the patched bridge
  header and the post-apply dry-run reports `changed=false`.
- The GUI evidence contains lowcmd ctrl trace lines before the first dynamic
  instability report.
- The lowcmd report must identify whether first nonzero `ctrl` comes from
  commanded torque, position-PD error, velocity-PD error, or mixed PD error.
- Hardware and Mimic gates remain locked unless Velocity acceptance evidence
  later passes.

Evidence:

- Focused pytest: `17 passed` for the lowcmd trace patch/report tests plus the
  policy I/O resolver regression.
- `ruff check`: all checks passed for the lowcmd trace patch/report scripts,
  policy I/O report resolver, and focused tests.
- `ruff format --check`: all checked files were already formatted.
- External bridge patch dry-run after application reports `changed=false`.
- External rebuild result: `unitree_mujoco` built successfully.
- GUI smoke evidence: `logs/flying_kick_sim2sim/20260523-025122/`.
- Reports:
  - `logs/g1_tracking_phase1/2026-05-23T02-51-22+08-00/velocity_contract.json`
  - `logs/g1_tracking_phase1/2026-05-23T02-51-22+08-00/mujoco_transition_trace.json`
  - `logs/g1_tracking_phase1/2026-05-23T02-51-22+08-00/policy_io_trace.json`
  - `logs/g1_tracking_phase1/2026-05-23T02-51-22+08-00/lowcmd_ctrl_trace.json`
- Lowcmd classification: `position_pd_ctrl_handoff`.
- First nonzero control: sample `2` / sim time `0.002`, with
  `ctrl_l2=47.307024`, `ctrl_max=29.73`, `tau_l2=0.0`,
  `pos_term_l2=47.307024`, `vel_term_l2=0.0`, `q_error_l2=1.450379`, and
  `dq_error_l2=0.0`.
- Dominant first nonzero joint: index `3`, `top_q_cmd=0.0`,
  `top_q_sensor=0.3`, `top_q_error=-0.3`, `top_kp=99.099998`,
  `top_pos_term=-29.73`, and `top_ctrl=-29.73`.
- First large control: sample `5` / sim time `0.008`, with
  `ctrl_l2=55.307971`; by then the dominant source has shifted toward
  velocity-PD error (`vel_term_l2=54.693506`, `dq_error_l2=47.921736`).
- Runtime rejection remains: first unstable Velocity sample is line `91` /
  policy step `25`, zero command, `q_err_l2=8.016`,
  `joint_vel_l2=396.306`, `raw_action_l2=23.089`, and
  `root_ang_vel_l2=7.193`.

Interpretation:

- This slice proves the first nonzero MuJoCo `ctrl` is generated by bridge-side
  lowcmd target/current mismatch, specifically position-PD error, before any
  large policy action can be blamed. The first command has zero commanded torque
  and zero velocity-PD term.
- The earlier suspect list is now narrower: not `FixStand` quality, not
  paused/frozen policy stepping, not policy catch-up burst, not key-9 release,
  not enabled elastic band alone, and not contact impulse alone. The remaining
  non-model blocker is how lowcmd target joint positions are initialized,
  carried over, or reconciled when entering the official Velocity Run boundary.

Advance rule:

- Next slice should make the Run/Velocity boundary command-safe: prime or
  reconcile `LowCmd.q` to the intended policy-default/current stand target
  before MuJoCo physics starts applying PD, then rerun the same lowcmd trace and
  official Velocity smoke. Do not tune Mimic/tracking, do not judge policy
  quality, and do not unlock hardware until this lowcmd handoff is clean.

### 7az. Official Unitree Baseline Drift Audit - done, baseline run still pending

Commit:

- `docs(g1): audit official unitree baseline drift`

Status:

- Done as an audit slice. This intentionally does not restore `.external` and
  does not launch another GUI smoke. It corrects the diagnosis route after the
  user challenge that official sim2sim should be mature: before changing
  lowcmd semantics, prove what is official baseline and what is local wrapper
  drift.

Actions:

- Clone upstream `unitreerobotics/unitree_rl_mjlab` into
  `/tmp/unitree_rl_mjlab_official_baseline`.
- Compare upstream HEAD `1425b15` with the active local external runtime tree.
- Classify differences into policy assets, behavior-changing runtime/config
  drift, and diagnostic-only trace drift.
- Record a reversible baseline run plan that avoids using `.external` as a
  clean baseline.

Mandatory checks:

- `git -C /tmp/unitree_rl_mjlab_official_baseline rev-parse --short HEAD`
- `diff -q` upstream vs local active Velocity `policy.onnx`.
- `diff -q` upstream vs local active Velocity `params/deploy.yaml`.
- `diff -u` / focused source inspection for the changed sim/deploy config and
  runtime files.
- `git diff --check`

Evidence:

- Upstream clone HEAD: `1425b15`.
- Active local Velocity `policy.onnx` is byte-identical to upstream.
- Active local Velocity `params/deploy.yaml` is byte-identical to upstream.
- Active local `simulate/config.yaml` differs from upstream by wrapper runtime
  choices: `use_joystick: 0`, `enable_elastic_band: 0`, custom 36-value
  `initial_qpos`, and `start_paused: 1`.
- Active local `deploy/robots/g1/config/config.yaml` differs from upstream by
  adding `initial_state`, Mimic states, sim-only transitions, and a non-upstream
  FixStand target. Upstream only enables Passive/FixStand/Velocity/Dance1 and
  uses the Velocity default/action-offset pose as the FixStand target.
- Behavior-changing source drift found:
  `simulate/src/param.h`, `simulate/src/main.cc`,
  `deploy/include/FSM/State_FixStand.h`, and
  `deploy/include/isaaclab/envs/mdp/actions/joint_actions.h`.
- Diagnostic trace/source drift found:
  `deploy/include/FSM/State_RLBase.h`,
  `deploy/include/isaaclab/envs/manager_based_rl_env.h`,
  `deploy/robots/g1/src/State_RLBase.cpp`,
  `deploy/robots/g1/src/State_Mimic.cpp`, `simulate/src/main.cc`, and
  `simulate/src/unitree_sdk2_bridge.h`.

Interpretation:

- The earlier lowcmd result is still useful as evidence from the patched local
  wrapper, but it is not a clean official baseline result.
- The active local failure should not be used to accuse upstream Unitree
  sim2sim, the upstream Velocity package, or the new training policy until a
  clean official baseline path is run.
- Since the policy assets are identical, a clean official baseline failure would
  implicate operator/bootstrap/runtime assumptions, not a copied wrong ONNX.
  A clean official baseline pass would prove our wrapper patches/config are the
  active failure surface.

Reversible baseline run plan:

- Preferred: build and run the `/tmp/unitree_rl_mjlab_official_baseline` clone
  in place, keeping all generated logs under a new evidence directory and
  leaving `.external/unitree_rl_mjlab` untouched.
- If the official default requires a gamepad and blocks automation, document
  that as a capability blocker; do not silently change the official config and
  call it baseline.
- If an automated comparison is needed, create a separate copy/overlay baseline
  root with an explicit manifest of every deviation from upstream. The only
  allowed deviations must be launch plumbing, log capture, and interface
  selection; no `initial_qpos`, `start_paused`, policy gate, action-reset, or
  FixStand semantic patch may be present in a clean-baseline claim.
- After the clean baseline is proven, either resume the lowcmd handoff fix on
  the wrapper path or delete that direction if the clean baseline already
  identifies a simpler route.

Advance rule:

- Do not apply another behavior patch to `.external` until one of these is true:
  a clean upstream baseline run passes/fails with evidence, or the baseline run
  is blocked by a concrete missing capability such as gamepad/GUI/DDS. Hardware
  remains locked.

### 7ba. Clean Official Baseline Build And Launch Preflight - blocked by missing gamepad

Commit:

- `docs(g1): record official baseline launch blocker`

Status:

- Blocked at launch preflight. The clean upstream source builds in `/tmp`, but
  the official default launch path requires a gamepad at `/dev/input/js0`, which
  is absent in the current session. This is a capability blocker, not a
  simulator-code failure.

Actions:

- Configure and build the upstream `simulate` target from
  `/tmp/unitree_rl_mjlab_official_baseline`.
- Configure and build the upstream G1 deploy controller from
  `/tmp/unitree_rl_mjlab_official_baseline/deploy/robots/g1`.
- Inspect upstream README and config to verify launch requirements.
- Check for joystick devices and run the upstream `jstest` helper.

Mandatory checks:

- `cmake -S /tmp/unitree_rl_mjlab_official_baseline/simulate -B /tmp/unitree_rl_mjlab_official_baseline/simulate/build -DCMAKE_PREFIX_PATH=/home/ssy/ssy_files/mjlab/.external/unitree_sdk2/install`
- `cmake --build /tmp/unitree_rl_mjlab_official_baseline/simulate/build -j8`
- `cmake -S /tmp/unitree_rl_mjlab_official_baseline/deploy/robots/g1 -B /tmp/unitree_rl_mjlab_official_baseline/deploy/robots/g1/build` with SDK include/link flags pointing to `/home/ssy/ssy_files/mjlab/.external/unitree_sdk2/install`.
- `cmake --build /tmp/unitree_rl_mjlab_official_baseline/deploy/robots/g1/build -j8`
- `ls -la /dev/input`
- `/tmp/unitree_rl_mjlab_official_baseline/simulate/build/jstest`
- `git diff --check`

Evidence:

- Upstream `unitree_mujoco` build result: `Built target unitree_mujoco`.
- Upstream `g1_ctrl` build result: `Built target g1_ctrl`.
- The initial clean controller build failed before SDK flags because upstream
  G1 `CMakeLists.txt` lacks the local `.external` build's
  `find_package(unitree_sdk2 REQUIRED)` patch. Rebuilding with command-line SDK
  include/link flags keeps upstream source unchanged and succeeds.
- Upstream README simulation-deployment section says the simulator launch
  requires a connected gamepad.
- Upstream `simulate/config.yaml` uses `use_joystick: 1` and
  `joystick_device: "/dev/input/js0"`.
- `/dev/input` is absent in the current session.
- Upstream `jstest` exits nonzero with `open failed.`.

Interpretation:

- Official Unitree sim2sim is mature enough to build locally from a clean clone.
- This session cannot honestly claim a clean official GUI baseline run because
  the required joystick device is missing.
- The existing wrapper's `use_joystick: 0`, scripted transitions,
  `start_paused`, `initial_qpos`, and phase-1 patches are useful diagnostics,
  but they are not the official default baseline.

Advance rule:

- To complete the clean official baseline, rerun on a session/host exposing the
  gamepad device, or explicitly approve an "official source plus automation
  deviation" lane whose manifest records `use_joystick: 0` as a deviation. Do
  not call that lane clean upstream baseline, and do not unlock hardware.

### 7bb. Official Baseline Preflight Command - done, blocker reproducible

Commit:

- `feat(g1): add official baseline preflight`

Status:

- Done. The clean official baseline blocker is now reproducible through a
  repo-local non-launching CLI instead of being only a session note.

Actions:

- Add `src/mjlab/scripts/g1_tracking_phase1_official_baseline_preflight.py`.
- Add wrapper `scripts/tools/g1_tracking_phase1_official_baseline_preflight.py`.
- Check clean upstream root, upstream HEAD, official simulator config, required
  binaries, `/dev/input/js0`, current user groups, optional `jstest`, and the
  clean-baseline deviation policy.
- Remove the superseded untracked action-reset WIP from this worktree because
  the active route is official-baseline-first, not another behavior patch.

Mandatory tests:

- `uv run --active --no-sync pytest tests/tools/test_g1_tracking_phase1_official_baseline_preflight.py`
- `uv run --active --no-sync ruff check src/mjlab/scripts/g1_tracking_phase1_official_baseline_preflight.py scripts/tools/g1_tracking_phase1_official_baseline_preflight.py tests/tools/test_g1_tracking_phase1_official_baseline_preflight.py`
- `uv run --active --no-sync ruff format --check src/mjlab/scripts/g1_tracking_phase1_official_baseline_preflight.py scripts/tools/g1_tracking_phase1_official_baseline_preflight.py tests/tools/test_g1_tracking_phase1_official_baseline_preflight.py`
- Real preflight:
  `scripts/tools/g1_tracking_phase1_official_baseline_preflight.py --run-jstest --expect-ready --report-out logs/g1_tracking_phase1/2026-05-23T-official-baseline-preflight/official_baseline_preflight.json`
- `git diff --check`

Evidence:

- Focused pytest: `4 passed`.
- `ruff check`: all checks passed.
- `ruff format --check`: 3 files already formatted.
- Real preflight exits `1` under `--expect-ready`, as expected for a blocked
  clean baseline.
- Report path:
  `logs/g1_tracking_phase1/2026-05-23T-official-baseline-preflight/official_baseline_preflight.json`.
- Report facts: official HEAD `1425b15`, upstream `use_joystick=1`, configured
  joystick `/dev/input/js0`, no `initial_qpos`, no `start_paused`, binaries
  `unitree_mujoco`, `g1_ctrl`, and `jstest` executable, blockers
  `missing_joystick_device` and `jstest_failed`, and current user not in the
  `input` group.

Interpretation:

- The clean official source/build path is ready enough to launch once the
  required gamepad device is available.
- The current blocker is environmental: no `/dev/input/js0` is exposed. This is
  different from the earlier wrapper failure and should not trigger another
  behavior patch.

Advance rule:

- Next clean-baseline action is external-state dependent: expose a real
  `/dev/input/js0` and rerun the preflight with `--expect-ready`. If it passes,
  then run the official simulator/controller baseline. If it remains blocked,
  record the exact blocker from the report. Hardware remains locked.

### 8. Real-Robot Trial Gate - locked

Commit:

- `feat(g1): gate tethered real-robot dual-kick trials`

Status:

- Locked. Do not start until work item 7 passes both actions under a deploy-safe entry contract.

Actions:

- Only after both sim2sim2 actions pass, prepare a conservative tethered real-robot checklist.
- Refuse loopback/down/unknown interfaces.
- Verify no stale `g1_ctrl` or Unitree MuJoCo process is active.
- Cap trials at 1-2 attempts per action and require paired video plus controller log.

Mandatory tests:

- `bash -n scripts/tools/run_g1_dual_kicks_real_deploy.sh`
- `uv run pytest tests/tools/test_g1_tracking_phase1_real_gate.py`
- `scripts/tools/run_g1_dual_kicks_real_deploy.sh prepare`
- `scripts/tools/run_g1_dual_kicks_real_deploy.sh status`
- Negative preflight checks that `lo`, down interfaces, missing sim2sim2 evidence, and stale controller/simulator processes are refused.

Acceptance gate:

- Hardware start requires a referenced sim2sim2 evidence directory that passed both actions.
- `lo`, down interfaces, unknown interfaces, and missing evidence produce non-zero refusal before controller launch.
- Real-robot claim requires paired video plus controller log for each attempted action.
- Each attempted action is capped at 1-2 tethered/support-rig attempts.
- Any hardware result is classified with the same timing/stability schema used in sim2sim2.

Advance rule:

- No later training or reward-tuning phase starts until real-robot evidence is either accepted as pass/fail evidence or explicitly deferred by the user.

## Commit Units

Commit units are intentionally identical to the amended work items above:

1. `feat(g1): add phase1 baseline manifest` - done in `2367496a`
2. `feat(g1): add phase1 new-g1 contract validator` - done in `87582be5`
3. `feat(g1): add phase1 dual-action sim2sim2 preflight wrapper` - done in `71e48c14`
4. `feat(g1): add phase1 timing and stability evidence parser` - done in `345c52a2`
5. `docs(g1): record phase1 sim2sim2 evidence classification` - done in `df63e4d2`
5a. `fix(g1): expose direct mimic sim2sim diagnosis mode` - done in `e8f8de57`
5b. `fix(g1): classify in-action sim2sim instability` - done in `9c869c5f`
5c. `diagnose(g1): add phase1 entry-pose gap checker` - done in `18ed1bea`
6. `feat(g1): add phase1 entry-state handoff gate` - done in `3aa42dba`
7a. `fix(g1): harden phase1 sim2sim auto-run` - implemented in this work item
7b. `docs(g1): record phase1 post-entry sim2sim2 evidence` - implemented in this work item
7c. `feat(g1): add official elastic sim2sim bootstrap mode` - smoke failed, follow-up fix required
7d. `fix(g1): make official elastic bootstrap paused-safe` - done
7e. `fix(g1): pretension elastic before any Run request` - done
7f. `feat(g1): add velocity-first sim2sim bootstrap mode` - done, blocker found
7g. `diagnose(g1): isolate velocity bootstrap sim2sim failure` - done, blocker found
7h. `fix(g1): test velocity policy-default bootstrap entry` - done, blocker found
7i. `diagnose(g1): classify velocity initial contact mismatch` - done, blocker found
7j. `fix(g1): add grounded velocity policy-default bootstrap` - done, blocker found
7k. `diagnose(g1): falsify velocity elastic-band cause` - done, blocker found
7l. `diagnose(g1): trace velocity policy provenance` - done, blocker found
7m. `diagnose(g1): inventory velocity policy candidates` - done, blocker found
7n. `diagnose(g1): trace velocity runtime actions` - done, blocker found
7o. `diagnose(g1): replay velocity zero-command policy` - done, blocker found
7p. `diagnose(g1): triage velocity deploy candidates` - done, blocker found
7q. `diagnose(g1): audit velocity runtime observations` - done, blocker found
7r. `feat(g1): add deploy98 velocity task contract` - done, remediation path available
7s. `feat(g1): add deploy98 velocity package generator` - done, tooling ready
7t. `fix(g1): attach deploy metadata to actor onnx exports` - done, export path ready
7u. `docs(g1): record deploy98 training smoke evidence` - done, non-acceptance evidence recorded
7v. `docs(g1): record deploy98 velocity pilot rejection` - done, candidate rejected
7w. `feat(g1): add deploy98 stand-first velocity task` - done, training entry ready
7x. `docs(g1): record deploy98 standfirst velocity candidate` - done, Velocity-only GUI gate pending
7y. `fix(g1): allow explicit velocity sim2sim policy root` - done, GUI smoke unblocked
7z. `docs(g1): record standfirst velocity sim2sim rejection` - done, candidate rejected
7aa. `diagnose(g1): add velocity policy sensitivity probe` - done, blocker narrowed
7ab. `feat(g1): add deploy98 standfirst damped velocity task` - done, training entry ready
7ac. `docs(g1): record damped standfirst velocity pilot rejection` - done, candidate rejected before package
7ad. `docs(g1): record damped standfirst continuation candidate` - done, Velocity-only GUI gate pending
7ae. `docs(g1): record damped continuation sim2sim rejection` - done, candidate rejected in Velocity
7af. `docs(g1): record passive velocity sim2sim rejection` - done, candidate rejected after Passive-to-Velocity entry
7ag. `diagnose(g1): audit velocity actuator force contract` - done, blocker narrowed to external scene ctrlrange mismatch
7ah. `docs(g1): record actuator-aligned velocity smoke rejection` - done, actuator-aligned smoke still fails in Velocity
7ai. `docs(g1): record start-unpaused velocity smoke rejection` - done, start-unpaused smoke fails in first second
7aj. `fix(g1): add official velocity sim2sim bootstrap` - done, official-aligned Velocity acceptance path ready
7ak. `fix(g1): keep waiting after official velocity transition` - done, official_velocity auto-run state wait fixed
7al. `docs(g1): record official velocity bootstrap rejection` - done, official-aligned smoke fails in Velocity after key-9 release
7am. `diagnose(g1): capture dynamic velocity policy io trace` - done, first-unstable trace captured
7an. `diagnose(g1): replay dynamic velocity trace through onnx` - done, deploy replay matches logged action
7ao. `diagnose(g1): analyze velocity trace chronology` - done, gross last-action/command/phase timing mismatch unlikely
7ap. `diagnose(g1): capture dense velocity onset trace` - done, dense onset captured after official key-8/Run/key-9 bootstrap
7aq. `diagnose(g1): classify velocity onset order` - done, previous-action-first cause unlikely
7ar. `docs(g1): record no-key9 velocity onset control` - done, automatic key-9 release not sufficient
7as. `diagnose(g1): classify no-elastic velocity onset` - done, enabled elastic band not sufficient
7at. `diagnose(g1): add mujoco transition trace patcher` - done, tooling ready for runtime application
7au. `diagnose(g1): analyze mujoco transition trace` - done, first physics-step motion precedes contact and large ctrl
7av. `diagnose(g1): audit velocity run handoff` - done, latest paused evidence is startup-contaminated
7aw. `diagnose(g1): add velocity policy start gate` - done, gate works but clock handoff remains contaminated
7ax. `diagnose(g1): gate velocity policy on lowstate tick` - done, policy cadence fixed but Velocity stability still rejected
7ay. `diagnose(g1): trace lowcmd ctrl handoff` - done, first nonzero ctrl source proven
7az. `docs(g1): audit official unitree baseline drift` - done, clean upstream baseline run still pending
7ba. `docs(g1): record official baseline launch blocker` - blocked by missing `/dev/input/js0`
7bb. `feat(g1): add official baseline preflight` - done, clean baseline blocker reproducible
8. `feat(g1): gate tethered real-robot dual-kick trials` - locked until a deploy-safe sim2sim2 evidence run passes both actions

Each commit must carry its own tests and acceptance evidence. If any commit cannot meet its gate, do not continue to the next commit; produce a blocker summary and route to `diagnose`.

## Known Risks And Blockers

- `.external/unitree_rl_mjlab` is machine-local runtime content outside this worktree; modifying or rebuilding it may require explicit approval and should not be committed from this repo.
- Unitree MuJoCo may require GUI, DDS, or controller dependencies that are not available in a headless/sandboxed run.
- The current sim2sim scripts mention that default `FixStand` is not stable enough for deploy sim2sim; if this still reproduces, the first diagnosis target is deploy/sim initial-state contract, not policy quality.
- Current evidence already shows a large deploy/default-entry to motion-start pose gap. A sim-only `initial_qpos` fix can isolate causality, but it is not deploy acceptance unless the controller can reproduce the same entry state without teleporting.
- Real hardware stability cannot be proven by code alone; operator judgment and support-rig safety remain mandatory.
- Passing sim2sim2 does not prove untethered safety; it only unlocks short tethered hardware trials.

## Handoff

Current handoff override:

- `FixStand` is not a Velocity/tracking policy state; it is a PD
  pose/interpolation bootstrap. A free-standing `FixStand` fall is not accepted
  as a Velocity policy failure.
- Direct `Passive -> Velocity` remains a useful diagnostic lane, but the primary
  acceptance lane is now `official_velocity_bootstrap`: `FixStand -> Velocity`,
  key-8/contact preparation, MuJoCo `Run` after `Velocity` readiness, then key-9
  release.
- The latest official-aligned GUI smoke
  `logs/flying_kick_sim2sim/20260522-225945/` rejects the damped StandFirst
  package after it reaches dynamic `Velocity`: first unstable sample at line 74
  / `2026-05-22 23:00:06.361`, `policy_step=950`,
  `command_norm=0.0`, `raw_action_l2=45.429`,
  `processed_action_l2=19.583`, `joint_vel_l2=263.947`,
  `q_err_l2=17.164`, and `root_ang_vel_l2=14.925`.
- Next slice: capture the full 98-dim deployed observation vector, raw action,
  and processed target near the first dynamic unstable sample, then compare
  against mjlab training/play observation for the same pose/velocity state.
  Hardware remains locked.
- Latest update: `logs/flying_kick_sim2sim/20260522-231619/` now captures that
  first-unstable-near trace. The selected policy I/O trace is exactly
  `policy_step=925` / line 73, one line before the first unstable sample at
  line 74. Continue with the comparison/replay slice rather than adding more
  GUI evidence for the same failure.
- Replay update:
  `velocity_official_bootstrap_v2_dynamic_trace_replay_report.json` reproduces
  the logged action from the selected obs (`raw_action_gap_l2=0.00000685`,
  `processed_action_gap_l2=0.00000328`). A gross C++ ONNX/logging mismatch is
  unlikely.
- Chronology update:
  `velocity_official_bootstrap_v2_dynamic_trace_chronology_report.json` shows
  early consecutive `last_action` values exactly match the previous raw action
  (`max_early_last_action_gap_l2=0.0`), zero-command observations stay zero, and
  zero-command `gait_phase` stays masked to zero. Source audit confirms mjlab and
  C++ deploy agree on previous-action policy-call semantics. A gross
  `last_action`, command, or gait-phase timing mismatch is therefore unlikely
  for the early official-aligned Velocity trace.
- Next slice: add denser transition evidence around MuJoCo Run/key-9 release
  and the first nonzero lowstate/sim motion update. The current trace jumps from
  stable step 50 to already-unstable dynamic step 925, so the next gate must
  identify whether the first dynamic nonzero state is caused by
  simulator/controller handoff timing, action target jump, contact impulse, DDS
  lowstate latency, or policy response. Hardware remains locked.
- Latest update:
  `logs/flying_kick_sim2sim/20260523-004041/` repeats the official-aligned
  Velocity bootstrap with `enable_elastic_band=0`. It still rejects the damped
  StandFirst package in zero-command `Velocity`, but the first dynamic onset is
  now classified as `observed_motion_before_policy_response`: step `840` has
  `last_action_l2=1.002` and `raw_action_l2=0.831`, step `841` is the first
  large current raw action, and step `842` is the first large previous action.
  This falsifies enabled elastic band as a sufficient root cause and keeps the
  next target on physical/deploy transition instrumentation before policy
  response.
- Tooling update: `g1_tracking_phase1_mujoco_transition_trace_patch.py` can
  patch the local Unitree MuJoCo `simulate/src/main.cc` and inject
  `[PHASE1_SIM] event=mujoco_transition_trace` after both `mj_step` paths. It
  was first verified as dry-run-only, then applied in the latest evidence
  update below.
- Latest evidence update:
  `logs/flying_kick_sim2sim/20260523-010629/` applied that local MuJoCo trace
  patch and rebuilt `unitree_mujoco`. The MuJoCo report shows first physics
  step after Run is already dynamic (`sim_time=0.002`, `qvel_l2=11.950`,
  `root_ang_vel_l2=0.173`) with `ncon=0`, `elastic_force_l2=0.0`, and
  non-large `ctrl_l2=17.644`. First large ctrl appears at sim step `5`; first
  contact appears at sim step `8`. This narrows the next diagnosis to
  paused-to-run handoff, initial floating/support state, or low-level command
  state at Run. The external MuJoCo source remains locally instrumented with a
  `.phase1_mujoco_transition_trace_v1.bak` backup; source/binary generated
  runtime content is not committed. Hardware remains locked.
- Run handoff audit update:
  `velocity_official_bootstrap_mujoco_trace_no_elastic_run_handoff_audit.json`
  classifies that latest run as `paused_policy_with_floating_support_handoff`.
  The policy thread advanced to frozen step `50` while MuJoCo physics was
  paused, the initial support state still had `min_foot_surface_z=0.027211`,
  and the first physics step had `qvel_l2=11.950`, `ctrl_l2=17.644`, and
  `ncon=0`. Treat this as startup handoff contamination, not clean Velocity
  policy-quality evidence. The next runnable slice should either prevent policy
  advancement during paused acceptance or start acceptance only after proved
  contact/settle and synchronized controller state at MuJoCo `Run`.
- Policy-start gate update:
  `logs/flying_kick_sim2sim/20260523-015505/` applies and rebuilds the local
  `State_RLBase.h` gate patch. The controller log proves the 5s gate starts at
  `01:55:08.588`, logs `policy_step=0` before release, and releases at
  `01:55:13.591`. This removes pre-release policy stepping, but the run remains
  rejected: the audit report
  `velocity_official_gate_run_handoff_audit.json` classifies
  `paused_policy_handoff`, and traces still advance through policy step `50`
  with `joint_vel_l2=0.0` and `root_ang_vel_l2=0.0` after release. The first
  unstable sample is policy step `675` with zero command, `q_err_l2=4.650`,
  `joint_vel_l2=327.436`, and `root_ang_vel_l2=10.668`. Treat this as evidence
  that a fixed wall-clock delay is not enough; the next slice must synchronize
  policy stepping against fresh MuJoCo/DDS lowstate after Run.
- Lowstate tick gate update:
  `logs/flying_kick_sim2sim/20260523-023332/` applies and rebuilds the local
  `State_RLBase.h` lowstate tick gate patch. The controller log proves
  `LowState.tick()` remains stale after the 5s wall-clock release and
  `policy_step` stays at `0` until the first fresh tick at `02:33:53.303`.
  After fresh ticks arrive, cadence is now correct: `policy_step=350` at
  `02:34:00.304`, `375` at `02:34:00.806`, and `400` at `02:34:01.307`, so the
  previous catch-up burst is fixed. The run still rejects Velocity stability:
  first unstable sample is line `91` / policy step `25`, zero command,
  `q_err_l2=4.773`, `joint_vel_l2=372.938`, and `root_ang_vel_l2=8.406`.
  The handoff audit now classifies `lowcmd_ctrl_handoff`, with
  `policy_steps_while_physics_paused=false`, `support_gap_before_run=false`,
  and `first_step_no_contact=false`, but `first_step_nonzero_ctrl=true`.
  Continue by instrumenting/reconciling early `LowCmd` to MuJoCo `ctrl` at Run;
  do not treat this as policy-quality acceptance and do not unlock hardware.
- Lowcmd ctrl trace update:
  `logs/flying_kick_sim2sim/20260523-025122/` applies and rebuilds the local
  `unitree_sdk2_bridge.h` ctrl-decomposition patch. The report
  `logs/g1_tracking_phase1/2026-05-23T02-51-22+08-00/lowcmd_ctrl_trace.json`
  classifies the first nonzero control as `position_pd_ctrl_handoff`. At sample
  `2` / sim time `0.002`, `ctrl_l2=47.307024`, `tau_l2=0.0`,
  `pos_term_l2=47.307024`, and `vel_term_l2=0.0`. The dominant joint is index
  `3`, where `q_cmd=0.0`, `q_sensor=0.3`, and `q_error=-0.3`, producing
  `top_ctrl=-29.73`. This proves the first MuJoCo control spike is a lowcmd
  target/current PD handoff, not policy torque. Continue by making the
  Run/Velocity boundary command-safe: prime or reconcile `LowCmd.q` before
  MuJoCo applies PD. Hardware remains locked.

Next skill: `diagnose`.

Reason: the non-teleport `prepose` candidate failed Unitree MuJoCo sim2sim2 for both actions in `FixStand` before Mimic entry, and the official elastic-band bootstrap smoke now shows a corrected but still falling `FixStand` run. Since `FixStand` is a PD pose-transition state, the active diagnostic path moved to Velocity-first. Paused `velocity_bootstrap` passes, but explicit Run falls in `Velocity` before any Mimic trigger. The first Velocity contract report identified a default-pose/action-offset mismatch; the policy-default follow-up removes that joint mismatch but exposes an initial-contact mismatch from combining policy-default joints with a knees-bent/root-low initial height. The home-root follow-up clears the contact gate but still falls in `Velocity`; the no-elastic follow-up also falls in `Velocity`, so elastic-band force is not the sole cause. The extended report shows the active Velocity policy has `run_path: 2026-03-18_18-40-20` but no matching local source run, while current-source gains/action scale are only rounding-different from deploy. The local inventory found 125 local Velocity ONNX candidates but 0 directly compatible with the active `v0` observation contract. Runtime trace shows zero command at the first unstable sample but large raw/processed Velocity actions and extreme joint velocities. Offline zero-command ONNX replay confirms the active `v0` policy does not hold its deploy default pose even under nominal zero-command/default-pose observation. Deploy-candidate triage found 122 current-source flat Velocity actor/checkpoint candidates, but all are 99-dim and lack complete Unitree deploy packages, so direct ONNX swapping remains unsafe. Runtime observation audit shows YAML-only 99-dim deployment is also unsafe because the C++ deploy runtime has no `base_lin_vel` observation or ArticulationData field. A separate `Mjlab-Velocity-Flat-Unitree-G1-Deploy98` task now provides a 98-dim actor contract aligned with active deploy runtime terms, a package generator can build a Unitree policy directory from a compatible deploy98 ONNX, and manual actor export now writes the metadata required by that package generator. A 1-iteration deploy98 smoke proves the non-model plumbing but remains rejected by zero-command replay as an untrained policy. A 300-iteration pilot also rejects before GUI: it trains and packages, but final episode length is only 2.69s and zero-command replay target gap is 0.618. `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst` trains to the 5-second zero-command mjlab gate and packages successfully, but replay still reports a non-default target gap of 0.595646. The wrapper now supports explicit `MJLAB_VELOCITY_POLICY_ROOT`, and the resulting StandFirst Velocity-only Unitree MuJoCo smoke proves the correct policy was used and initial pose/contact gates passed, but the controller still fails in Velocity after 1.5 seconds with zero command. The policy sensitivity probe narrows the remaining zero-command loop: after excluding impossible zero-command `velocity_commands` and `gait_phase` perturbations, the packaged StandFirst policy is most sensitive to `joint_vel_rel`, where a single-axis `100 rad/s` perturbation raises processed target gap from `0.603384` to `3.0931`. `Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped` was registered to penalize this joint-velocity/action-amplification loop while preserving the deploy98 contract. Its first 300-iteration pilot misses the strict gate (`Episode/length_seconds=4.888494`, `fell_over=2.166667`), but the 300-iteration continuation from `model_299.pt` passes the training gate at step 598 (`Episode/length_seconds=5.0`, `fell_over=0.0`) and packages cleanly as `logs/g1_tracking_phase1/2026-05-22T21-18-00+08-00/velocity_deploy98_standfirst_damped_resume300_package/policy_dir`. Zero-command replay improves to `max_processed_target_gap_l2=0.385498`, while sensitivity still warns that command/phase masking and joint-velocity transients can drive large actions. The Velocity-only Unitree MuJoCo smoke then rejects this package in `Velocity` before any Mimic trigger: `logs/flying_kick_sim2sim/20260522-212636/`, first unstable sample after 2.0s at policy step 100 with `command_norm=0.0`, `raw_action_l2=38.483`, `processed_action_l2=15.654`, `joint_vel_l2=269.793`, `q_err_l2=13.591`, and `root_ang_vel_l2=19.108`. The follow-up `Passive -> Velocity` smoke matches the deploy-state interpretation more closely: `Passive` holds/settles, then a sim-only joystick condition enters `Velocity`; it also rejects the same damped package after MuJoCo Run, with first dynamic unstable sample at line 194 / policy step 4175, `command_norm=0.0`, `raw_action_l2=30.018`, `processed_action_l2=12.254`, `joint_vel_l2=351.815`, `q_err_l2=9.531`, and `root_ang_vel_l2=14.619`. The actuator-force contract audit proves the user, worktree, and external G1 XMLs are semantically the same new G1, but the external Unitree scene halves four training-side effort limits: right ankle pitch/roll and waist roll/pitch are `25Nm` in `scene_g1.xml` versus `50Nm` in mjlab training. A reversible local scene patch aligns those four motor `ctrlrange` values and the post-patch contract report passes, but the actuator-aligned `Passive -> Velocity` smoke still fails dynamically at line 143 / policy step 2900 with `command_norm=0.0`, `raw_action_l2=27.663`, `processed_action_l2=12.382`, `joint_vel_l2=415.247`, `q_err_l2=10.440`, and `root_ang_vel_l2=19.636`. The start-unpaused actuator-aligned control smoke removes the paused-policy-step artifact but fails even faster: first unstable sample at line 28 / policy step 25 with `command_norm=0.0`, `raw_action_l2=40.939`, `processed_action_l2=16.796`, `joint_vel_l2=425.455`, `q_err_l2=14.955`, and `root_ang_vel_l2=2.401`. The official-aligned dynamic trace then captures selected step 925 one line before the first unstable sample, and ONNX replay reproduces the logged action. The chronology report rules out a gross early previous-action lag, command leak, or gait-phase masking mismatch: early `last_action` matches previous raw action with gap `0.0`, command/gait terms remain zero, and source semantics align. The MuJoCo transition report proves first physical motion occurs on the first physics step before contact and before large ctrl; the Run handoff audit classifies that no-elastic paused run as startup-contaminated by paused policy advancement, floating support gap, first-step nonzero ctrl, and no contact. Continue by fixing or instrumenting the official runner's contact/settle and controller start alignment at MuJoCo `Run`; do not treat this run as clean policy-quality evidence. Real-robot work remains locked because no post-entry sim2sim2 gate has passed.
Reason update after 7ax: the lowstate tick gate fixes paused/frozen policy
advancement and post-wait catch-up bursts. The current blocker is now early
`LowCmd` to MuJoCo `ctrl` handoff plus zero-command Velocity instability, not
`FixStand` alone and not policy-clock catch-up.
Reason update after 7ay: the lowcmd trace proves the first nonzero MuJoCo
`ctrl` is position-PD error from `LowCmd.q` mismatch (`q_cmd=0.0` versus
`q_sensor=0.3` on joint index 3), with zero commanded torque. The next fix
should reconcile or prime lowcmd target positions at the official Velocity
Run boundary before any policy-quality or hardware judgment.
Reason update after 7az: the current `.external/unitree_rl_mjlab` runtime is not
a clean official Unitree baseline. Upstream Velocity policy assets match the
local active assets byte-for-byte, but local sim/deploy config and source
semantics have behavior-changing drift. Continue by running or explicitly
blocking a clean upstream baseline before applying more behavior patches.
Reason update after 7ba: clean upstream source builds locally, but the official
baseline launch is blocked by missing joystick device exposure. Upstream
`simulate/config.yaml` requires `/dev/input/js0`, this session has no
`/dev/input`, and upstream `jstest` fails with `open failed.`. Any no-gamepad
run must be labeled as an automation-deviation lane, not clean baseline.
Reason update after 7bb: the joystick blocker is now captured by a reusable
non-launching preflight command. Fresh report confirms clean upstream config and
build products are present, but `/dev/input/js0` is missing and `jstest` fails.
Continue clean baseline only after the joystick device is exposed.
