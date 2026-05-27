# G1 Isaac Lab sim2sim

This lane runs an existing G1 mimic deploy bundle in the local Isaac Lab runtime.
It does not install Isaac Lab and it does not mutate `.external`.

Example:

```sh
scripts/tools/run_g1_kick_isaaclab_sim2sim.sh \
  --action roundhouse_leading_right
```

Defaults:

- Isaac Lab checkout: `ISAACLAB_ROOT`, default `/home/ssy/ssy_files/IsaacLab`
- Conda environment: `ISAACLAB_CONDA_ENV`, default `env_isaaclab`
- mjlab checkout: `MJLAB_ROOT`, default current repository root
- external runtime assets: `MJLAB_EXTERNAL_ROOT`, default `<MJLAB_ROOT>/.external`
  - when running from `mjlab/.worktrees/<branch>`, the wrapper falls back to
    the primary checkout's `.external` if the worktree does not have one
- evidence: `logs/g1_isaaclab_sim2sim/<timestamp>-<action>/report.json`

The runner resolves the same deploy bundle used by Unitree sim2sim:

- `params/deploy.yaml`
- `params/<action>.npz`
- `exported/policy.onnx`
- `src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf`

Current local action choices are `dance1_subject2`, `flying_kick`, `getup`, and
`roundhouse_leading_right`.

The default control mode is `policy`: Isaac Lab spawns the local G1 URDF,
applies the deploy YAML gains, builds the deploy observation, evaluates the
exported ONNX actor through a small PyTorch evaluator, and writes the processed
joint position targets back to the articulation. `deploy.yaml` `step_dt` is the
policy/control timestep. The runner uses physics substeps under that timestep
(`--control-decimation 4` by default) so it matches the usual Isaac Lab
train-time `decimation=4` contract instead of treating `step_dt` as a single
coarse PhysX step. The default initial state is `--initial-state motion`, which
sets the floating base pose/velocity and deploy joints from frame 0 of the
motion file before policy playback. Use `--initial-state default` only for
entry-gap diagnostics from the deploy default pose. The fallback
`--control-mode reference` mode directly replays the reference joint positions
and is only for asset/path debugging.

A successful smoke report requires the requested motion steps to complete with
finite joint and root state and without the floating base dropping below
`--fall-root-height-threshold` (default `0.35m`). Policy runs also report
`action_target_delta_norm_*`; non-zero values confirm that the exported policy is
participating instead of sending only default targets.

The acceptance gate for a trained model is a full headless policy run with
`sim2sim_passed=true`. Short `--max-steps` checks only prove startup and policy
I/O; they do not prove the motion is stable.

Useful diagnostics:

- `--trace-every 1` writes `trace.csv` with root height, reference root height,
  joint target error, action target norm, root angular velocity, approximate
  implicit-PD effort ratio, and joint velocity ratio. `support_*` ratio fields
  restrict the same checks to the leg and waist joints, avoiding arm/wrist
  limits masking the support-chain behavior.
- `--settle-steps N --settle-mode velocity` runs the local zero-command G1
  Velocity policy before switching to the mimic policy, matching the Unitree
  `Velocity -> Mimic_*` entry shape more closely than a static pose hold.
- `--training-actuator-limits` writes mjlab training-side per-joint
  effort/velocity limits into PhysX after URDF import.
- `--video-path <file>.mp4` enables the Isaac Lab headless camera and writes
  playable MP4 evidence. The runner also records `video_path`,
  `video_frame_count`, `video_fps`, and `video_resolution` in `report.json`.

Current local evidence:

- All four currently packaged mimic bundles complete their full motion in
  headless Isaac Lab when started from `--initial-state motion`. `getup` needs a
  getup-appropriate low root-height threshold because the reference motion
  starts from the floor.
- `roundhouse_leading_right` passes a full headless policy run in Isaac Lab:
  `logs/g1_isaaclab_sim2sim/roundhouse_policy_default_motion_init_full_trace/report.json`
  has `sim2sim_passed=true`, `fall_detected=false`, and `completed_steps=199`.
- `getup` passes a full headless policy run when started from the motion initial
  state with a getup-appropriate low root-height threshold:
  `logs/g1_isaaclab_sim2sim/getup_policy_motion_init_full_trace/report.json`
  has `sim2sim_passed=true`, `completed_steps=153`, and final root height
  `0.736m` versus reference final root height `0.774m`.
- `flying_kick` passes full headless policy playback with motion root
  initialization:
  `logs/g1_isaaclab_sim2sim/flying_policy_default_motion_init_full_trace/report.json`
  has `sim2sim_passed=true`, `fall_detected=false`, `completed_steps=164`, and
  minimum root height `0.687m`. Earlier default-pose and velocity-settle runs
  fell; those are now treated as entry-gap diagnostics, not the accepted
  trained-model playback.
- Video evidence for the same `flying_kick` policy playback is at
  `logs/g1_isaaclab_sim2sim/flying_policy_video_20260527/flying_kick_isaaclab_headless.mp4`.
  Its paired report has `sim2sim_passed=true`, `fall_detected=false`,
  `completed_steps=164`, `video_frame_count=165`, and `video_resolution=[640, 360]`.
- `dance1_subject2` passes the full 6574-frame policy playback with motion root
  initialization:
  `logs/g1_isaaclab_sim2sim/dance1_policy_motion_root_init_full_trace/report.json`
  has `sim2sim_passed=true`, `fall_detected=false`, `completed_steps=6574`, and
  minimum root height `0.434m`.

For automated headless runs, the runner flushes the report and exits the process
without `SimulationApp.close()` by default because Isaac Sim shutdown can hang
after evidence is already written. Pass `--graceful-close` only when debugging
Kit cleanup behavior.

Avoid launching the full Isaac Sim GUI repeatedly from automation. On this
machine, GUI startup can spend minutes compiling RTX/Vulkan shader pipelines
(`RtPso async group async compilation`) and can make the desktop appear hung.
Prefer headless smoke evidence first, then open GUI once only after confirming no
old Isaac/Kit process is still running.
