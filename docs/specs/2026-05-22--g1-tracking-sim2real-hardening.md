# Spec - G1 Tracking Sim2real Hardening

> Status: user-approved
> Owner: user + Codex
> Date: 2026-05-22
> Source request: User confirmed first phase should diagnose the existing flying-kick and roundhouse deployments together, prove the new G1 29DoF mode 15 sim2sim/deploy contract, and use standing stability plus response timing as the hard gate before any new training.

## Background

Two Unitree G1 tracking policies have already been trained and deployed through the external Unitree G1 C++ controller stack:

- flying kick: `g1_tracking_acrobatics_no_state`, exported as `flying_kick_deploy_actor.onnx`
- roundhouse leading right: `g1_tracking_roundhouse_leading_right_no_state`, exported as `roundhouse_leading_right_deploy_actor.onnx`

The real-robot deployment video at `/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/video/2026-05/05e60f0aa3f07a32611272b5beaa9d3a.mp4` shows that the robot can enter the motion, but it is not deployment-ready: it relies on the support rig, has visible lateral instability, and may have a velocity/action-response timing mismatch. The current mjlab play videos and tracking evaluation are useful, but they are not sufficient acceptance evidence for real deployment.

The deployment path is not the mjlab Python runner. It is the external Unitree controller under `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1`, where `config.yaml` selects an FSM state and each mimic state loads `policy_dir/params/deploy.yaml`, `policy_dir/exported/policy.onnx`, and a motion `.npz`. The desired intermediate demo target is `unitreerobotics/unitree_mujoco`, because it runs the Unitree SDK2-style low-level controller against MuJoCo and is closer to the real deployment loop than mjlab `play`.

The robot model must be the user's new Unitree G1 29DoF mode 15, based on:

- `/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/file/2026-04/g1_29dof_mode_15.urdf`
- `/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/file/2026-04/g1_new.xml`

Byte-for-byte XML equality is not required because local files may remove comments or normalize line endings. Structural and control-contract equality is required: 29 joints, matching joint names/order, matching actuator order, matching default pose/action scale/control gains, and no accidental use of old G1 or `g1_23dof`.

## Goals

- Establish a phase-1 deploy-readiness diagnostic gate for both G1 tracking policies together: flying kick and roundhouse leading right.
- Make standing stability and velocity/action-response timing consistency the hard first-version acceptance criteria.
- Preserve current flying-kick and roundhouse baselines so new training or deploy changes can be compared against known exported policies, motions, configs, and videos.
- Prove that the sim2sim/deploy program itself is valid on the user's new G1 29DoF mode 15 before interpreting policy quality.
- Add enough diagnostic evidence to separate these failure modes:
  - policy cannot track the reference motion;
  - policy tracks the motion in mjlab but fails under Unitree deploy timing;
  - FSM handoff or command timing starts the motion at the wrong physical moment;
  - policy action reaches the deploy stack but real or simulated joints respond late or incorrectly;
  - Velocity command or Velocity-state response is inaccurate before or after the mimic motion;
  - the motion ends without a stable balance handoff.
- Use Unitree MuJoCo as the required sim2sim2 demonstration layer before any new real-robot trial.
- Treat kick height, roundhouse visual style, and showcase quality as secondary metrics after stability and timing pass.

## Non-goals

- Do not train, tune rewards, or rerun long learning jobs in phase 1.
- Do not mutate the generic `Mjlab-Tracking-Flat-Unitree-G1` baseline task in place.
- Do not treat mjlab `play` or Viser visual success as sufficient deployment acceptance.
- Do not make motion-file surgery the default first fix.
- Do not replace the external Unitree C++ deploy stack in this spec.
- Do not attempt unsupported untethered or high-risk real-robot trials as part of first-version validation.
- Do not accept any sim2sim result if it used old G1, `g1_23dof`, the wrong scene XML, the wrong joint order, or an unverified initial pose.

## Users / Callers

- Primary user: the operator/researcher deploying G1 tracking policies and judging whether a model is safe enough to test on hardware.
- Codex/agent callers: future implementation agents that will add diagnostics, run Unitree MuJoCo demos, and package phase-1 evidence. Training variants are a later phase after this spec's diagnostic gate.
- Entry points:
  - mjlab play/evaluation and validation commands through `uv run`.
  - local scripts under `scripts/tools/`, especially `run_g1_dual_kicks_real_deploy.sh`, `run_flying_kick_sim2sim.sh`, and `run_roundhouse_leading_right_sim2sim.sh`.
  - external deploy files under `.external/unitree_rl_mjlab/deploy/robots/g1`.
  - Unitree MuJoCo simulator under `.external/unitree_rl_mjlab/simulate` or the upstream `unitreerobotics/unitree_mujoco` layout.

## Behavior Spec

### Happy Path

- Given the existing flying-kick and roundhouse exported policies, motion files, deploy YAMLs, and deploy FSM config, the workflow records one joint baseline manifest with paths, hashes, model checkpoint provenance, motion provenance, robot-model provenance, and current git status.
- Before sim2sim2, the workflow proves that mjlab, external deploy, and Unitree MuJoCo are all using the user's new G1 29DoF mode 15 contract. The check covers XML/URDF provenance, scene XML, DOF count, joint names/order, actuator names/order, action dimension, ONNX input/output shape, motion joint count, default joint pose, action scale, and deploy gains.
- The workflow can run both policy bundles through:
  - mjlab play/evaluation;
  - Unitree MuJoCo sim2sim2 using loopback/simulation DDS settings;
  - real-robot deployment only after sim2sim2 evidence passes.
- During Unitree MuJoCo and real-robot runs, the controller logs enough timing information to align:
  - controller start time;
  - joystick or command trigger event;
  - FSM state transition;
  - mimic motion time/frame;
  - policy inference step;
  - processed action;
  - low-level joint target write;
  - q/dq/action error, base velocity, commanded velocity if available, and projected gravity.
- A real-robot trial is eligible only if both flying kick and roundhouse pass sim2sim2 on the verified new-G1 model.
- A phase-1 run passes only if each action completes, returns to `Velocity` or another approved stable state, and remains stable for 5 seconds after the action ends.

### Edge Cases

- Missing `policy.onnx`, `motion.npz`, `deploy.yaml`, or checkpoint provenance must stop the workflow before deploy or sim2sim2.
- ONNX input/output shape mismatch must fail before launching Unitree MuJoCo or the real robot.
- Any mismatch between the user's new G1 29DoF mode 15 contract and the active mjlab/deploy/sim model must fail before launching Unitree MuJoCo.
- If the simulated robot immediately explodes, flies, or becomes unstable before any intentional command, the result is a sim2sim/deploy contract failure, not a policy-quality judgment.
- A requested real-robot start on loopback, a down interface, or an unknown interface must be refused.
- If Unitree MuJoCo cannot run because of GUI, DDS, or dependency issues, the fallback is a no-hardware deploy-bundle validation plus explicit capability gap; it is not a substitute for sim2sim2 acceptance.
- If a policy falls after the reference ends, the result must be labeled as a standing-handoff failure even if the visible motion looked correct for part of the clip.
- If action amplitude or kick height improves while stability gets worse, the run fails first-version acceptance.

### Interfaces / State

- Source code:
  - `src/mjlab/tasks/tracking/config/g1/env_cfgs.py`
  - `src/mjlab/tasks/tracking/tracking_env_cfg.py`
  - `src/mjlab/tasks/tracking/mdp/commands.py`
  - `src/mjlab/tasks/tracking/scripts/evaluate.py`
  - `scripts/tools/run_g1_dual_kicks_real_deploy.sh`
  - `scripts/tools/run_flying_kick_sim2sim.sh`
  - `scripts/tools/run_roundhouse_leading_right_sim2sim.sh`
  - `src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf`
  - `src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml`
- External deploy source:
  - `.external/unitree_rl_mjlab/deploy/robots/g1/src/State_Mimic.cpp`
  - `.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp`
  - `.external/unitree_rl_mjlab/deploy/robots/g1/config/config.yaml`
  - `.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml`
  - `.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/scene_g1.xml`
  - `.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1_23dof.xml` as a forbidden accidental dependency for this phase
- Runtime artifacts:
  - `logs/rsl_rl/...`
  - `logs/flying_kick_sim2sim/...`
  - `logs/g1_dual_kicks_real_deploy/...`
  - `artifacts/weekly_report_videos/...`
  - new evidence directories should stay under `logs/` or `artifacts/` and should not be committed by default.

## Constraints

- Use `uv run` for project Python tooling.
- Keep existing successful baselines intact; add specialized task IDs or scripts for hardening variants instead of changing generic defaults.
- Do not commit generated logs, videos, checkpoints, or local `.external` contents unless explicitly requested.
- Real-robot tests require human/operator judgment and must remain tethered/support-rig safe for this phase.
- First real-robot pass is conservative: sim2sim2 must pass first; flying kick and roundhouse get at most 1-2 tethered trials each; every trial requires video plus controller log.
- Unitree MuJoCo simulation must use simulation-safe DDS settings and must not collide with the real robot domain/interface.
- The first version optimizes for stability and timing evidence, not peak kick aesthetics.
- Phase 1 does not start new training. It may recommend phase-2 training changes only after the current gap is classified.

## Chosen Approach

Use a diagnosis-first phase-1 gate:

1. Freeze and verify the current flying-kick and roundhouse baseline bundles together.
2. Prove the new-G1 29DoF mode 15 identity and control contract across mjlab, external deploy, and Unitree MuJoCo.
3. Add or require timing/stability diagnostics in the Unitree deploy loop.
4. Run the exact same bundles through Unitree MuJoCo sim2sim2 before touching hardware.
5. If sim2sim2 passes, run only short tethered real-robot trials with video and controller logs.
6. After phase 1, decide whether phase 2 should add observation delay, actuator delay, friction/COM/encoder randomization, command timing perturbations, entry-state perturbations, reward changes, or post-motion standing/`Velocity` handoff criteria.

This approach fits the current problem because the observed failure is in the deployed closed loop, not merely in the visual quality of the reference motion.

## Rejected Options

- Reward tuning first: rejected because it can make a showcase motion look better while leaving timing and handoff gaps unmeasured.
- Motion-file surgery first: rejected because prior get-up work showed that motion edits can degrade policy behavior and obscure whether the root cause is tracking quality or post-reference stability.
- mjlab play-only acceptance: rejected because play/Viser can loop or reset episodes and does not prove real deployment stability.
- Real-robot-first iteration: rejected because Unitree MuJoCo can exercise the same deploy stack with lower risk and better repeatability before hardware trials.
- Single-action pilot: rejected because the first gate must prove the shared deploy/FSM/sim2sim stack for both actions and their return to `Velocity`.
- Strict byte-for-byte XML matching: rejected because line endings and removed comments are not relevant; structural and control-contract equality is the real safety requirement.

## Verification Strategy

### Baseline Evidence

- Record `git status --short` before edits and before final verification.
- Record hash manifests for:
  - flying-kick policy ONNX, checkpoint, motion, deploy YAML, and active config;
  - roundhouse policy ONNX, checkpoint, motion, deploy YAML, and active config.
- Record new-G1 identity evidence:
  - user's source URDF/XML paths;
  - active mjlab URDF/XML paths;
  - active external deploy/sim XML and scene paths;
  - joint count and ordered joint list;
  - actuator count/order;
  - ONNX input/output dimensions;
  - motion `joint_pos` dimensions;
  - default pose and deploy gain source.
- Preserve current real-robot symptom video metadata: duration, FPS, resolution, and file path.
- Collect baseline mjlab play/evaluation evidence for the selected checkpoints.
- Collect baseline Unitree MuJoCo sim2sim2 evidence for both exported bundles before judging policy quality or recommending any training variant.

### Automated Checks

- Run focused tests for any changed Python tracking config or CLI wrapper:
  - `uv run pytest tests/test_tracking_task.py`
  - `uv run pytest tests/tasks/tracking/test_env_contract_controls.py`
  - `uv run pytest tests/test_g1_constants.py`
- Run deploy-bundle validation that checks required files, ONNX input/output shapes, YAML parseability, FSM state presence, and motion file readability.
- Run G1 contract validation that fails on old G1, `g1_23dof`, wrong scene XML, wrong joint/action dimension, or mismatched joint order.
- For broad source changes, run:
  - `make format`
  - `make type`
  - `make test` when the change affects shared task or deploy behavior.

### Smoke / E2E Checks

- Unitree MuJoCo sim2sim2 smoke:
  - launch simulator with G1 scene, loopback interface, and simulation-safe domain;
  - launch G1 controller with the flying-kick and roundhouse bundles;
  - trigger each FSM transition from the approved entry state;
  - confirm action completion and 5-second post-action stability for both actions;
  - record `g1_ctrl.log`, simulator log, screenshot/video, and hash manifest.
- Real-robot smoke, only after sim2sim2 passes:
  - verify network interface is up and not loopback;
  - verify no stale `g1_ctrl` or Unitree MuJoCo process is already active;
  - run a tethered low-risk trial, at most 1-2 attempts per action;
  - confirm action completion and the 5-second post-action stability window;
  - record video and controller logs with enough timing data to align trigger and execution.

### Negative / Boundary Checks

- Missing bundle file fails before launch.
- Mismatched ONNX shape fails before launch.
- G1 identity mismatch fails before launch.
- Sim2sim startup explosion or immediate uncontrolled motion fails the deploy/sim contract.
- Real-robot command refuses `lo` and down interfaces.
- Unitree MuJoCo sim2sim2 and real robot cannot run against the same DDS domain/interface by accident.
- A result that improves kick amplitude but worsens post-motion balance fails acceptance.

### Documentation / State Checks

- Keep this spec as the source of truth until user approval.
- Do not add temporary trial status to `AGENTS.md`.
- If implementation creates new long-running experiments, record recoverable evidence paths in the chosen local recovery surface rather than in this spec.
- Update README or docs only after behavior and commands are stable.

### Fresh Evidence Required Before Completion

- Fresh `git status --short`.
- Fresh baseline hash manifests for both actions.
- Fresh G1 contract validation output.
- Fresh Unitree MuJoCo sim2sim2 logs and video/screenshot for both actions.
- Fresh real-robot log/video pair for any claim about hardware improvement.
- Fresh test output for changed Python code or deploy-wrapper logic.

## Capability Gaps

- Real-robot stability and support-rig load cannot be proven by code alone; the user/operator must judge the physical trial and safety envelope.
- Video-only evidence cannot precisely prove command timing; controller logs need explicit timestamped trigger/FSM/motion/action entries.
- Unitree MuJoCo GUI and DDS availability may fail on a headless or partially configured machine; fallback is to report the blocker and stop before real-robot testing.
- Exact hardware latency may require extra instrumentation or external measurement; if unavailable, approximate it through timestamped controller logs and repeated sim2sim2/real comparisons.
- The user suspects the main timing issue is action-to-joint response and Velocity-command response, but the system must still log the full trigger/FSM/motion/policy/action chain to avoid premature blame.

## Success Criteria

- A baseline report exists for the current flying-kick and roundhouse bundles, including hashes, checkpoint paths, motion paths, deploy config paths, and current failure symptoms.
- G1 contract validation proves the active mjlab, deploy, and Unitree MuJoCo paths use the user's new 29DoF mode 15 model and not old G1 or `g1_23dof`.
- Unitree MuJoCo sim2sim2 can run both selected policy bundles and produce logs/video that show the FSM transition, motion start, policy steps, action writes, response timing, and post-motion state.
- Both actions complete in Unitree MuJoCo and remain stable for 5 seconds after returning to `Velocity` or another explicitly approved stable state.
- Before any real-robot claim, there is a paired video plus controller log showing trigger time, FSM transition time, motion execution window, and post-motion stability window.
- First-version acceptance requires stable entry, stable motion execution, and stable return/hold for 5 seconds after action end; kick height alone cannot pass the spec.

## Residual Risks

- Unitree MuJoCo may still differ from the real robot in actuator response, contact, cable/support effects, and onboard timing.
- A policy that passes a short tethered trial may still fail under longer or untethered operation.
- The current trained models may not contain enough robustness for the required hardening; after diagnostics, additional training budget may be necessary.
- If command timing mismatch comes from joystick/operator behavior rather than code, software-only fixes may not fully solve it.
- If the deploy stack uses a hidden robot asset or generated model not covered by the validation script, manual source inspection may still be needed before hardware trials.

## Plan Handoff

- Active slice: build the baseline evidence, new-G1 contract validation, and Unitree MuJoCo sim2sim2 diagnostic gate for both existing flying-kick and roundhouse bundles.
- Suggested next skill: plan
- Planning notes:
  - Start with evidence and diagnostics, not training.
  - Keep baseline artifacts immutable and compare any later phase-2 candidate by hash.
  - Treat sim2sim startup instability as a deploy/model-contract failure before blaming the policy.
  - Add training variants only after the observed gap is classified in a later phase.
  - Real-robot trials are last, short, tethered, capped at 1-2 attempts per action, and require fresh sim2sim2 evidence first.
