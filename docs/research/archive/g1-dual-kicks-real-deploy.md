# G1 Dual Kicks Real Deploy Runbook

Last updated: 2026-05-25

Archive note: this runbook is retained as historical operational guidance from
the closed G1 dual-kick task batch. Revalidate policy paths, network interface,
robot state, and controller process state before any future real-robot start.

This runbook prepares the physical G1 controller for the two standard-tracking
continuation policies that passed labeled sim2sim:

- flying kick: `model_6998.pt`,
  `flying_kick_standard_tracking_deploy_actor.onnx`
- roundhouse leading right: `model_8997.pt`,
  `roundhouse_standard_tracking_deploy_actor.onnx`

This is a real-robot deployment wrapper, not a clean official baseline proof or
hardware-safety certification.

## Prepared State

The active deploy assets have been copied into:

- `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/flying_kick/`
- `/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/roundhouse_leading_right/`

Backup and hash record:

```sh
logs/g1_dual_kicks_real_deploy/20260525-181256-prepare
```

## Check Before Start

```sh
bash scripts/tools/run_g1_dual_kicks_real_deploy.sh preflight enp3s0
bash scripts/tools/run_g1_dual_kicks_real_deploy.sh status
```

The preflight validates:

- both ONNX contracts are `obs (154,) -> actions (29,)`;
- the selected network interface exists and is up;
- no `unitree_mujoco` or `g1_ctrl` process is already running;
- selected policy/motion hashes are printed.

## Start And Stop

Starting requires explicit confirmation:

```sh
MJLAB_REAL_DEPLOY_CONFIRM=YES \
bash scripts/tools/run_g1_dual_kicks_real_deploy.sh start enp3s0
```

On this host, `enp3s0` is the Unitree wired interface
(`192.168.123.222/24`). `wlo1` is Wi-Fi (`192.168.0.105/24`) and will not
receive the robot `rt/lowstate` stream.

Attach logs:

```sh
tmux attach -t g1_dual_kicks_real_ctrl
```

Stop controller:

```sh
bash scripts/tools/run_g1_dual_kicks_real_deploy.sh stop
```

## Controller Buttons

- `X`: Passive -> Velocity
- `A`: Passive -> Getup -> Velocity
- `B`: any active state -> Passive
- `RB + A`: Velocity -> Getup
- `RB + X`: Velocity -> Flying kick
- `RB + Y`: Velocity -> Roundhouse leading right
- `RB + B`: Mimic action -> Velocity

Each mimic action also returns to `Velocity` automatically at motion end.

## Restore Previous Deploy Assets

```sh
bash scripts/tools/run_g1_dual_kicks_real_deploy.sh restore \
  logs/g1_dual_kicks_real_deploy/20260525-181256-prepare
```
