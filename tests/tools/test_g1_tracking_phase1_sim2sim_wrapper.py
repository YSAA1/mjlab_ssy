from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / "scripts/tools/run_g1_dual_kicks_sim2sim_phase1.sh"
FLYING_SINGLE = ROOT / "scripts/tools/run_flying_kick_sim2sim.sh"
ROUNDHOUSE_SINGLE = ROOT / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh"


def _write(path: Path, content: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _fake_contract(path: Path) -> None:
  _write(
    path,
    """#!/usr/bin/env python
from __future__ import annotations

import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--manifest", required=True)
parser.add_argument("--forbid-g1-23dof", action="store_true")
parser.add_argument("--report-out", required=True)
args = parser.parse_args()
report = {"passed": True, "manifest": args.manifest, "forbid_g1_23dof": args.forbid_g1_23dof}
Path(args.report_out).write_text(json.dumps(report), encoding="utf-8")
print(json.dumps(report))
""",
  )
  path.chmod(0o755)


def _fake_executable(
  path: Path, content: str = "#!/usr/bin/env bash\nexit 0\n"
) -> None:
  _write(path, content)
  path.chmod(0o755)


def _workspace(tmp_path: Path, *, interface: str = "lo") -> tuple[Path, Path, Path]:
  worktree = tmp_path / "worktree"
  root = tmp_path / "mjlab-root"
  phase = worktree / "logs/g1_tracking_phase1/2026-05-22T12-00-00+08-00"
  sim_config = root / ".external/unitree_rl_mjlab/simulate/config.yaml"
  _write(
    worktree / "scripts/tools/run_flying_kick_sim2sim.sh",
    "#!/usr/bin/env bash\nexit 0\n",
  )
  _write(
    worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh",
    "#!/usr/bin/env bash\nexit 0\n",
  )
  sim_config.parent.mkdir(parents=True, exist_ok=True)
  sim_config.write_text(
    yaml.safe_dump(
      {
        "robot": "g1",
        "robot_scene": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "domain_id": 0,
        "interface": interface,
        "use_joystick": 0,
      }
    ),
    encoding="utf-8",
  )
  manifest = {
    "deploy_configs": {
      "sim_config": {"path": str(sim_config)},
    }
  }
  _write(phase / "manifest.json", json.dumps(manifest))
  return worktree, root, phase / "manifest.json"


def _single_action_workspace(tmp_path: Path) -> tuple[Path, Path]:
  worktree = tmp_path / "worktree"
  root = tmp_path / "mjlab-root"
  unitree = root / ".external/unitree_rl_mjlab"
  deploy = unitree / "deploy/robots/g1"
  sim = unitree / "simulate"

  run_dir = (
    worktree
    / "logs/rsl_rl/g1_tracking_acrobatics_no_state/2026-05-22_g1_mode15_flying_kick_4096env_5000iter"
  )
  _write(run_dir / "flying_kick_deploy_actor.onnx", "fake onnx\n")
  _write(worktree / "data/motions/g1_flying_kick/mjlab/motion.npz", "fake npz\n")
  _write(deploy / "config/policy/mimic/getup/params/deploy.yaml", "step_dt: 0.02\n")
  _write(
    deploy / "config/policy/velocity/v0/params/deploy.yaml",
    yaml.safe_dump(
      {
        "default_joint_pos": [0.1] * 29,
      },
      sort_keys=False,
    ),
  )
  _write(deploy / "config/policy/velocity/v0/exported/policy.onnx", "fake onnx\n")
  _fake_executable(deploy / "build/g1_ctrl")
  _fake_executable(sim / "build/unitree_mujoco")

  _write(
    deploy / "config/config.yaml",
    yaml.safe_dump(
      {
        "FSM": {
          "initial_state": "Passive",
          "_": {"FixStand": {"id": 2}},
          "FixStand": {
            "transitions": {},
            "kp": [1.0] * 29,
            "kd": [1.0] * 29,
            "ts": [0.0, 2.0],
            "qs": [[], [0.0] * 29],
          },
        }
      },
      sort_keys=False,
    ),
  )
  _write(
    sim / "config.yaml",
    yaml.safe_dump(
      {
        "robot": "g1",
        "robot_scene": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "domain_id": 0,
        "interface": "lo",
        "use_joystick": 0,
        "enable_elastic_band": 0,
        "start_paused": 0,
      },
      sort_keys=False,
    ),
  )

  _write(
    worktree / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml",
    """<mujoco model="g1">
  <compiler angle="radian"/>
  <default/>
  <asset><mesh name="pelvis" file="pelvis_5010.STL"/></asset>
  <worldbody>
    <body name="pelvis"><site name="imu_in_pelvis"/></body>
  </worldbody>
  <contact/>
</mujoco>
""",
  )
  _write(
    worktree / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/assets/pelvis_5010.STL",
    "fake stl\n",
  )
  _write(
    unitree / "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
    """<mujoco model="scene_g1">
  <compiler angle="radian"/>
  <default/>
  <asset><texture name="grid"/></asset>
  <worldbody>
    <light mode="trackcom"/>
    <body name="pelvis"/>
  </worldbody>
  <actuator><motor name="joint0"/></actuator>
  <sensor>
    <framequat name="imu_quat" objname="pelvis"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <framepos name="frame_pos" objname="pelvis"/>
    <framelinvel name="frame_vel" objname="pelvis"/>
  </sensor>
  <statistic/>
  <visual/>
</mujoco>
""",
  )
  return worktree, root


def _fake_runtime_bin(tmp_path: Path) -> Path:
  bin_dir = tmp_path / "bin"
  _fake_executable(
    bin_dir / "uv",
    """#!/usr/bin/env bash
if [[ "$1" == "run" ]]; then
  shift
  while [[ "$1" == --* ]]; do
    shift
  done
fi
exec "$@"
""",
  )
  _fake_executable(
    bin_dir / "tmux",
    """#!/usr/bin/env bash
case "$1" in
  has-session)
    exit 1
    ;;
  new-session)
    cmd="${*: -1}"
    log_path="$(printf '%s\n' "$cmd" | sed -n "s/.*tee '\\([^']*\\)'.*/\\1/p")"
    if [[ -n "$log_path" ]]; then
      mkdir -p "$(dirname "$log_path")"
      printf '%s\n' "$cmd" > "$log_path.cmd"
      printf '[info] FSM: Start FixStand\n[info] FSM: Change state from FixStand to Velocity\n' > "$log_path"
    fi
    exit 0
    ;;
  ls)
    exit 0
    ;;
esac
exit 0
""",
  )
  _fake_executable(
    bin_dir / "xdotool",
    """#!/usr/bin/env bash
if [[ "$1" == "search" ]]; then
  printf '1\n'
fi
exit 0
""",
  )
  return bin_dir


def _env(tmp_path: Path, worktree: Path, root: Path, contract: Path) -> dict[str, str]:
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_PHASE1_ROOT": str(worktree / "logs/g1_tracking_phase1"),
      "MJLAB_PHASE1_PYTHON": sys.executable,
      "MJLAB_PHASE1_CONTRACT_CMD": str(contract),
    }
  )
  return env


def test_preflight_writes_reports_without_launching(tmp_path: Path) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)

  proc = subprocess.run(
    ["bash", str(WRAPPER), "preflight", "--manifest", str(manifest)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  preflight_report = manifest.parent / "sim2sim2_preflight.json"
  contract_report = manifest.parent / "contract_report.json"
  assert json.loads(preflight_report.read_text(encoding="utf-8"))["passed"] is True
  assert json.loads(contract_report.read_text(encoding="utf-8"))["passed"] is True
  assert "Phase-1 sim2sim2 preflight passed" in proc.stdout


def test_preflight_refuses_non_loopback_interface(tmp_path: Path) -> None:
  worktree, root, manifest = _workspace(tmp_path, interface="enp3s0")
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)

  proc = subprocess.run(
    ["bash", str(WRAPPER), "preflight", "--manifest", str(manifest)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 2
  assert "expected 'lo'" in proc.stderr


def test_status_is_non_launching_and_recovers_without_manifest(tmp_path: Path) -> None:
  worktree = tmp_path / "worktree"
  root = tmp_path / "mjlab-root"
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)

  proc = subprocess.run(
    ["bash", str(WRAPPER), "status"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0
  assert "Latest manifest: none" in proc.stdout
  assert "unitree-sim2sim prepare-g1" in proc.stdout


def test_sim2sim_wrappers_do_not_default_to_deleted_worktree() -> None:
  for script in (WRAPPER, FLYING_SINGLE, ROUNDHOUSE_SINGLE):
    content = script.read_text(encoding="utf-8")

    assert 'WORKTREE="${MJLAB_WORKTREE:-$ROOT}"' in content
    assert "/home/ssy/ssy_files/mjlab/.worktrees/g1-flying-kick-main" not in content


def test_single_action_prepare_lane_delegates_to_unitree_sim2sim(
  tmp_path: Path,
) -> None:
  worktree = tmp_path / "worktree"
  root = tmp_path / "mjlab-root"
  bin_dir = tmp_path / "bin"
  _fake_executable(
    bin_dir / "uv",
    """#!/usr/bin/env bash
printf '%s\n' "$@" > "$MJLAB_WORKTREE/prepare_args.txt"
""",
  )
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "PATH": f"{bin_dir}:{env['PATH']}",
    }
  )
  worktree.mkdir(parents=True)
  root.mkdir(parents=True)

  proc = subprocess.run(
    [
      "bash",
      str(FLYING_SINGLE),
      "prepare-lane",
      "--official-root",
      str(tmp_path / "official"),
      "--out-root",
      str(tmp_path / "lane"),
      "--policy-root",
      str(tmp_path / "policy"),
      "--dry-run",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  args = (worktree / "prepare_args.txt").read_text(encoding="utf-8").splitlines()
  assert args[:5] == [
    "run",
    "unitree-sim2sim",
    "prepare-g1",
    "--action",
    "flying_kick",
  ]
  assert "--official-root" in args
  assert "--out-root" in args
  assert "--policy-root" in args
  assert "--dry-run" in args


def test_dual_wrapper_prepare_lane_requires_and_forwards_action(tmp_path: Path) -> None:
  worktree = tmp_path / "worktree"
  root = tmp_path / "mjlab-root"
  bin_dir = tmp_path / "bin"
  _fake_executable(
    bin_dir / "uv",
    """#!/usr/bin/env bash
printf '%s\n' "$@" > "$MJLAB_WORKTREE/prepare_args.txt"
""",
  )
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "PATH": f"{bin_dir}:{env['PATH']}",
    }
  )
  worktree.mkdir(parents=True)
  root.mkdir(parents=True)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "prepare-lane",
      "--action",
      "roundhouse_leading_right",
      "--official-root",
      str(tmp_path / "official"),
      "--out-root",
      str(tmp_path / "lane"),
      "--policy-root",
      str(tmp_path / "policy"),
      "--dry-run",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  args = (worktree / "prepare_args.txt").read_text(encoding="utf-8").splitlines()
  assert args[:5] == [
    "run",
    "unitree-sim2sim",
    "prepare-g1",
    "--action",
    "roundhouse_leading_right",
  ]


def test_start_forwards_direct_mimic_mode_to_single_action_script(
  tmp_path: Path,
) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)
  _write(
    worktree / "scripts/tools/run_flying_kick_sim2sim.sh",
    """#!/usr/bin/env bash
{
  printf 'MJLAB_SIM2SIM_MODE=%s\n' "${MJLAB_SIM2SIM_MODE:-}"
  printf 'MJLAB_START_PAUSED=%s\n' "${MJLAB_START_PAUSED:-}"
} > "$MJLAB_WORKTREE/start_env.txt"
""",
  )
  (worktree / "scripts/tools/run_flying_kick_sim2sim.sh").chmod(0o755)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "start",
      "--action",
      "flying_kick",
      "--manifest",
      str(manifest),
      "--mode",
      "play_parity",
      "--start-paused",
      "0",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  env_report = (worktree / "start_env.txt").read_text(encoding="utf-8")
  assert "MJLAB_SIM2SIM_MODE=play_parity" in env_report
  assert "MJLAB_START_PAUSED=0" in env_report


def test_start_forwards_prepose_mode_to_single_action_script(
  tmp_path: Path,
) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)
  _write(
    worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh",
    """#!/usr/bin/env bash
printf 'MJLAB_SIM2SIM_MODE=%s\n' "${MJLAB_SIM2SIM_MODE:-}" > "$MJLAB_WORKTREE/start_env.txt"
""",
  )
  (worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh").chmod(0o755)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "start",
      "--action",
      "roundhouse_leading_right",
      "--manifest",
      str(manifest),
      "--mode",
      "prepose",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  env_report = (worktree / "start_env.txt").read_text(encoding="utf-8")
  assert "MJLAB_SIM2SIM_MODE=prepose" in env_report


def test_start_forwards_official_bootstrap_mode_to_single_action_script(
  tmp_path: Path,
) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)
  _write(
    worktree / "scripts/tools/run_flying_kick_sim2sim.sh",
    """#!/usr/bin/env bash
printf 'MJLAB_SIM2SIM_MODE=%s\n' "${MJLAB_SIM2SIM_MODE:-}" > "$MJLAB_WORKTREE/start_env.txt"
""",
  )
  (worktree / "scripts/tools/run_flying_kick_sim2sim.sh").chmod(0o755)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "start",
      "--action",
      "flying_kick",
      "--manifest",
      str(manifest),
      "--mode",
      "official_bootstrap",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  env_report = (worktree / "start_env.txt").read_text(encoding="utf-8")
  assert "MJLAB_SIM2SIM_MODE=official_bootstrap" in env_report


def test_start_forwards_official_velocity_bootstrap_mode_to_single_action_script(
  tmp_path: Path,
) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)
  _write(
    worktree / "scripts/tools/run_flying_kick_sim2sim.sh",
    """#!/usr/bin/env bash
printf 'MJLAB_SIM2SIM_MODE=%s\n' "${MJLAB_SIM2SIM_MODE:-}" > "$MJLAB_WORKTREE/start_env.txt"
""",
  )
  (worktree / "scripts/tools/run_flying_kick_sim2sim.sh").chmod(0o755)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "start",
      "--action",
      "flying_kick",
      "--manifest",
      str(manifest),
      "--mode",
      "official_velocity_bootstrap",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  env_report = (worktree / "start_env.txt").read_text(encoding="utf-8")
  assert "MJLAB_SIM2SIM_MODE=official_velocity_bootstrap" in env_report


def test_start_forwards_velocity_bootstrap_mode_to_single_action_script(
  tmp_path: Path,
) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)
  _write(
    worktree / "scripts/tools/run_flying_kick_sim2sim.sh",
    """#!/usr/bin/env bash
printf 'MJLAB_SIM2SIM_MODE=%s\n' "${MJLAB_SIM2SIM_MODE:-}" > "$MJLAB_WORKTREE/start_env.txt"
""",
  )
  (worktree / "scripts/tools/run_flying_kick_sim2sim.sh").chmod(0o755)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "start",
      "--action",
      "flying_kick",
      "--manifest",
      str(manifest),
      "--mode",
      "velocity_bootstrap",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  env_report = (worktree / "start_env.txt").read_text(encoding="utf-8")
  assert "MJLAB_SIM2SIM_MODE=velocity_bootstrap" in env_report


def test_start_forwards_passive_velocity_bootstrap_mode_to_single_action_script(
  tmp_path: Path,
) -> None:
  worktree, root, manifest = _workspace(tmp_path)
  contract = tmp_path / "fake_contract.py"
  _fake_contract(contract)
  _write(
    worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh",
    """#!/usr/bin/env bash
printf 'MJLAB_SIM2SIM_MODE=%s\n' "${MJLAB_SIM2SIM_MODE:-}" > "$MJLAB_WORKTREE/start_env.txt"
""",
  )
  (worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh").chmod(0o755)

  proc = subprocess.run(
    [
      "bash",
      str(WRAPPER),
      "start",
      "--action",
      "roundhouse_leading_right",
      "--manifest",
      str(manifest),
      "--mode",
      "passive_velocity_bootstrap",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=_env(tmp_path, worktree, root, contract),
  )

  assert proc.returncode == 0, proc.stderr
  env_report = (worktree / "start_env.txt").read_text(encoding="utf-8")
  assert "MJLAB_SIM2SIM_MODE=passive_velocity_bootstrap" in env_report


def test_single_action_sim2sim_returns_to_velocity_after_mimic() -> None:
  for script in (FLYING_SINGLE, ROUNDHOUSE_SINGLE):
    content = script.read_text(encoding="utf-8")

    assert '"end_state": "Velocity"' in content
    assert '"end_state": "FixStand"' not in content
    assert "prepose" in content
    assert "official_bootstrap" in content
    assert "official_velocity_bootstrap" in content
    assert "velocity_bootstrap" in content
    assert "passive_velocity_bootstrap" in content


def test_single_action_auto_run_clicks_mujoco_run_without_false_success() -> None:
  for script in (FLYING_SINGLE, ROUNDHOUSE_SINGLE):
    content = script.read_text(encoding="utf-8")

    assert "xdotool windowactivate --sync" in content
    assert 'xdotool key --window "$window_id" space' in content
    assert 'xdotool mousemove --window "$window_id" 183 428 click 1' in content
    assert "Requested MuJoCo Run after %s was ready" in content
    assert 'ready_state="Velocity"' in content
    assert 'xdotool key --window "$window_id" 8' in content
    assert content.index('xdotool key --window "$window_id" 8') < content.index(
      'xdotool key --window "$window_id" space'
    )
    assert "before Run" in content
    assert "Release elastic band manually with key 9" in content
    assert "MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN" in content
    assert "MJLAB_ELASTIC_RELEASE_DELAY_SECONDS" in content
    assert 'xdotool key --window "$window_id" 9' in content
    assert "bootstrap_helper.log" in content
    assert "Auto-started paused MuJoCo" not in content
    assert "MJLAB_PHASE1_POLICY_START_GATE_SECONDS" in content
    assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE" in content
    assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS" in content
    assert "policy start gate" in content
    assert "lowstate tick gate" in content
    assert "log_has()" in content
    assert "grep -Eq" in content


def test_single_action_official_bootstrap_forces_elastic_defaults() -> None:
  for script in (FLYING_SINGLE, ROUNDHOUSE_SINGLE):
    content = script.read_text(encoding="utf-8")

    assert 'CONFIG_SIM2SIM_MODE="stand"' in content
    assert 'ENABLE_ELASTIC_BAND="${ENABLE_ELASTIC_BAND:-1}"' in content
    assert 'START_PAUSED="${START_PAUSED:-1}"' in content
    assert 'AUTO_RUN_AFTER_READY="${AUTO_RUN_AFTER_READY:-0}"' in content
    assert "MJLAB_ELASTIC_PRETENSION_STEPS" in content
    assert "MJLAB_OFFICIAL_BOOTSTRAP_PRETENSION_STEPS:-24" in content
    assert "MJLAB_ELASTIC_DROP_STEPS" in content
    assert "MJLAB_OFFICIAL_BOOTSTRAP_DROP_STEPS:-0" in content
    assert "MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN" in content
    assert "MJLAB_ELASTIC_RELEASE_DELAY_SECONDS" in content


def test_single_action_velocity_bootstrap_forces_elastic_defaults() -> None:
  for script in (FLYING_SINGLE, ROUNDHOUSE_SINGLE):
    content = script.read_text(encoding="utf-8")

    assert (
      '[[ "$SIM2SIM_MODE" == "velocity_bootstrap" || "$SIM2SIM_MODE" == "passive_velocity_bootstrap" ]]'
      in content
    )
    assert "MJLAB_VELOCITY_BOOTSTRAP_POSE" in content
    assert "policy_default" in content
    assert "MJLAB_VELOCITY_BOOTSTRAP_ROOT" in content
    assert "MJLAB_VELOCITY_POLICY_ROOT" in content
    assert "VELOCITY_POLICY_DIR_CONFIG" in content
    assert "velocity root" in content
    assert "velocity policy root" in content
    assert 'ENABLE_ELASTIC_BAND="${ENABLE_ELASTIC_BAND:-1}"' in content
    assert 'START_PAUSED="${START_PAUSED:-1}"' in content
    assert 'AUTO_RUN_AFTER_READY="${AUTO_RUN_AFTER_READY:-0}"' in content
    assert "FSM: Start $ready_state" in content
    assert "FSM: Change state from Passive to Velocity" in content
    assert "FSM: Change state from FixStand to Velocity" in content
    assert 'transitions["Velocity"] = "!A"' in content
    assert "configure_policy_start_gate" in content
    assert 'POLICY_START_GATE_SECONDS="${POLICY_START_GATE_SECONDS:-5.0}"' in content
    assert "MJLAB_PHASE1_POLICY_START_GATE_SECONDS" in content
    assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE" in content
    assert "fresh DDS/MuJoCo state" in content
    assert (
      'passive_transitions["Velocity"] = "!A" if mode == "passive_velocity_bootstrap" else "X.on_pressed"'
      in content
    )
    assert "sim-only auto transition" in content
    assert "Passive-to-Velocity bootstrap" in content

  roundhouse_content = ROUNDHOUSE_SINGLE.read_text(encoding="utf-8")
  assert (
    'velocity_transitions["Mimic_RoundhouseLeadingRight"] = "RB + Y.on_pressed"'
    in (roundhouse_content)
  )


def test_flying_kick_official_bootstrap_generates_elastic_stand_config(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_bootstrap",
      "MJLAB_OFFICIAL_BOOTSTRAP_DROP_STEPS": "1",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "Mode: official_bootstrap; config mode: stand" in proc.stdout
  assert "elastic band: 1" in proc.stdout
  assert "auto run: 0" in proc.stdout
  assert "Unitree-style sim bootstrap" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )
  deploy_config = yaml.safe_load((selected / "config.yaml").read_text(encoding="utf-8"))

  assert sim_config["enable_elastic_band"] == 1
  assert sim_config["start_paused"] == 1
  assert sim_config["initial_qpos"][:7] == [
    0.0,
    0.0,
    0.7657805681228638,
    1.0,
    0.0,
    0.0,
    0.0,
  ]
  assert deploy_config["FSM"]["initial_state"] == "FixStand"
  assert deploy_config["FSM"]["FixStand"]["qs"][1] == sim_config["initial_qpos"][7:]


def test_flying_kick_official_velocity_bootstrap_generates_fixstand_to_velocity_config(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_VELOCITY_BOOTSTRAP_POSE": "policy_default",
      "MJLAB_VELOCITY_BOOTSTRAP_ROOT": "home",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "Mode: official_velocity_bootstrap" in proc.stdout
  assert "Unitree-style Velocity acceptance bootstrap" in proc.stdout
  assert "judges standing only after Velocity is reached" in proc.stdout
  assert "policy start gate: 0s" in proc.stdout
  assert "lowstate tick gate: 0" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )
  deploy_config = yaml.safe_load((selected / "config.yaml").read_text(encoding="utf-8"))

  assert sim_config["enable_elastic_band"] == 1
  assert sim_config["start_paused"] == 1
  assert sim_config["initial_qpos"][:7] == [
    0.0,
    0.0,
    0.783675,
    1.0,
    0.0,
    0.0,
    0.0,
  ]
  assert sim_config["initial_qpos"][7:] == [0.1] * 29
  assert deploy_config["FSM"]["initial_state"] == "FixStand"
  assert deploy_config["FSM"]["FixStand"]["transitions"]["Velocity"] == "!A"
  assert deploy_config["FSM"]["FixStand"]["qs"][1] == [0.1] * 29
  assert deploy_config["FSM"]["Velocity"]["policy_dir"] == "config/policy/velocity"


def test_flying_kick_official_velocity_bootstrap_auto_run_waits_after_transition(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_AUTO_RUN_AFTER_READY": "1",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "Requested MuJoCo Run after Velocity was ready" in proc.stdout
  assert "policy start gate: 5.0s" in proc.stdout
  assert "MuJoCo stayed paused" not in proc.stderr

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  ctrl_cmd = (artifact_dirs[-1] / "g1_ctrl.log.cmd").read_text(encoding="utf-8")
  assert "MJLAB_PHASE1_POLICY_START_GATE_SECONDS='5.0'" in ctrl_cmd
  assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE=''" in ctrl_cmd
  assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS=''" in ctrl_cmd
  assert "./g1_ctrl --network=lo" in ctrl_cmd


def test_flying_kick_official_velocity_bootstrap_can_auto_release_elastic_after_run(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_AUTO_RUN_AFTER_READY": "1",
      "MJLAB_AUTO_RELEASE_ELASTIC_AFTER_RUN": "1",
      "MJLAB_ELASTIC_PRETENSION_STEPS": "0",
      "MJLAB_ELASTIC_RELEASE_DELAY_SECONDS": "0",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "Requested MuJoCo Run after Velocity was ready" in proc.stdout
  assert "policy start gate: 5.0s" in proc.stdout
  assert "Released elastic band with MuJoCo key 9 after Run delay 0s" in proc.stdout
  assert "Release elastic band manually" not in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  helper_log = artifact_dirs[-1] / "bootstrap_helper.log"
  helper_text = helper_log.read_text(encoding="utf-8")
  assert "Prepared elastic length with MuJoCo key 8 before Run" in helper_text
  assert "Released elastic band with MuJoCo key 9 after Run delay 0s" in helper_text
  assert "Requested MuJoCo Run after Velocity was ready" in helper_text


def test_flying_kick_official_velocity_bootstrap_respects_policy_start_gate_override(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_AUTO_RUN_AFTER_READY": "1",
      "MJLAB_PHASE1_POLICY_START_GATE_SECONDS": "2.5",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "policy start gate: 2.5s" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  ctrl_cmd = (artifact_dirs[-1] / "g1_ctrl.log.cmd").read_text(encoding="utf-8")
  assert "MJLAB_PHASE1_POLICY_START_GATE_SECONDS='2.5'" in ctrl_cmd
  assert "./g1_ctrl --network=lo" in ctrl_cmd


def test_flying_kick_official_velocity_bootstrap_passes_lowstate_tick_gate_env(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_AUTO_RUN_AFTER_READY": "1",
      "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE": "1",
      "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS": "0.8",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "policy start gate: 5.0s; lowstate tick gate: 1" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  ctrl_cmd = (artifact_dirs[-1] / "g1_ctrl.log.cmd").read_text(encoding="utf-8")
  assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE='1'" in ctrl_cmd
  assert "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS='0.8'" in ctrl_cmd


def test_flying_kick_policy_start_gate_rejects_shell_unsafe_value(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_AUTO_RUN_AFTER_READY": "1",
      "MJLAB_PHASE1_POLICY_START_GATE_SECONDS": "2;uname",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 2
  assert "Unsupported MJLAB_PHASE1_POLICY_START_GATE_SECONDS" in proc.stderr


def test_flying_kick_lowstate_tick_gate_rejects_shell_unsafe_values(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "official_velocity_bootstrap",
      "MJLAB_AUTO_RUN_AFTER_READY": "1",
      "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE": "1;uname",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 2
  assert "Unsupported MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE" in proc.stderr


def test_flying_kick_velocity_bootstrap_generates_velocity_stand_config(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "velocity_bootstrap",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "Mode: velocity_bootstrap; config mode: velocity_bootstrap" in proc.stdout
  assert "elastic band: 1" in proc.stdout
  assert "auto run: 0" in proc.stdout
  assert "Velocity-first bootstrap" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )
  deploy_config = yaml.safe_load((selected / "config.yaml").read_text(encoding="utf-8"))

  assert sim_config["enable_elastic_band"] == 1
  assert sim_config["start_paused"] == 1
  assert sim_config["initial_qpos"][:7] == [
    0.0,
    0.0,
    0.7657805681228638,
    1.0,
    0.0,
    0.0,
    0.0,
  ]
  assert deploy_config["FSM"]["initial_state"] == "Velocity"
  assert deploy_config["FSM"]["Velocity"]["policy_dir"] == "config/policy/velocity"
  assert (
    deploy_config["FSM"]["Velocity"]["transitions"]["Mimic_FlyingKick"]
    == "RB + X.on_pressed"
  )
  assert deploy_config["FSM"]["FixStand"]["qs"][1] == sim_config["initial_qpos"][7:]


def test_flying_kick_passive_velocity_bootstrap_generates_passive_entry_config(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "passive_velocity_bootstrap",
      "MJLAB_ENABLE_ELASTIC_BAND": "0",
      "MJLAB_VELOCITY_BOOTSTRAP_POSE": "policy_default",
      "MJLAB_VELOCITY_BOOTSTRAP_ROOT": "home",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert (
    "Mode: passive_velocity_bootstrap; config mode: passive_velocity_bootstrap"
    in proc.stdout
  )
  assert "elastic band: 0" in proc.stdout
  assert "Passive-to-Velocity bootstrap" in proc.stdout
  assert "uses a sim-only auto transition to enter Velocity" in proc.stdout
  assert "judges standing in Velocity" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )
  deploy_config = yaml.safe_load((selected / "config.yaml").read_text(encoding="utf-8"))

  assert sim_config["enable_elastic_band"] == 0
  assert sim_config["start_paused"] == 1
  assert sim_config["initial_qpos"][:7] == [
    0.0,
    0.0,
    0.783675,
    1.0,
    0.0,
    0.0,
    0.0,
  ]
  assert sim_config["initial_qpos"][7:] == [0.1] * 29
  assert deploy_config["FSM"]["initial_state"] == "Passive"
  assert deploy_config["FSM"]["Passive"]["transitions"]["Velocity"] == "!A"
  assert deploy_config["FSM"]["Velocity"]["policy_dir"] == "config/policy/velocity"
  assert (
    deploy_config["FSM"]["Velocity"]["transitions"]["Mimic_FlyingKick"]
    == "RB + X.on_pressed"
  )
  assert deploy_config["FSM"]["FixStand"]["qs"][1] == [
    -0.312,
    0.0,
    0.0,
    0.669,
    -0.363,
    0.0,
    -0.312,
    0.0,
    0.0,
    0.669,
    -0.363,
    0.0,
    0.0,
    0.0,
    0.0,
    0.2,
    0.2,
    0.0,
    0.6,
    0.0,
    0.0,
    0.0,
    0.2,
    -0.2,
    0.0,
    0.6,
    0.0,
    0.0,
    0.0,
  ]


def test_flying_kick_velocity_bootstrap_can_use_policy_default_pose(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "velocity_bootstrap",
      "MJLAB_VELOCITY_BOOTSTRAP_POSE": "policy_default",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "velocity pose: policy_default" in proc.stdout
  assert "from policy_default joint qpos" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )

  assert sim_config["initial_qpos"][:7] == [
    0.0,
    0.0,
    0.7657805681228638,
    1.0,
    0.0,
    0.0,
    0.0,
  ]
  assert sim_config["initial_qpos"][7:] == [0.1] * 29


def test_flying_kick_velocity_bootstrap_can_use_explicit_velocity_policy_root(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  custom_policy = tmp_path / "custom_velocity_policy"
  _write(
    custom_policy / "params/deploy.yaml",
    yaml.safe_dump({"default_joint_pos": [0.2] * 29}, sort_keys=False),
  )
  _write(custom_policy / "exported/policy.onnx", "fake onnx\n")
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "velocity_bootstrap",
      "MJLAB_VELOCITY_BOOTSTRAP_POSE": "policy_default",
      "MJLAB_VELOCITY_POLICY_ROOT": str(custom_policy),
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert f"velocity policy root: {custom_policy}" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  deploy_config = yaml.safe_load((selected / "config.yaml").read_text(encoding="utf-8"))
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )

  assert deploy_config["FSM"]["Velocity"]["policy_dir"] == str(custom_policy.resolve())
  assert sim_config["initial_qpos"][7:] == [0.2] * 29


def test_flying_kick_velocity_bootstrap_can_use_home_root_with_policy_default_pose(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "velocity_bootstrap",
      "MJLAB_VELOCITY_BOOTSTRAP_POSE": "policy_default",
      "MJLAB_VELOCITY_BOOTSTRAP_ROOT": "home",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "velocity root: home" in proc.stdout
  assert "with home root qpos" in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )

  assert sim_config["initial_qpos"][:7] == [
    0.0,
    0.0,
    0.783675,
    1.0,
    0.0,
    0.0,
    0.0,
  ]
  assert sim_config["initial_qpos"][7:] == [0.1] * 29


def test_flying_kick_velocity_bootstrap_flow_reports_elastic_override(
  tmp_path: Path,
) -> None:
  worktree, root = _single_action_workspace(tmp_path)
  fake_bin = _fake_runtime_bin(tmp_path)
  env = os.environ.copy()
  env.update(
    {
      "MJLAB_WORKTREE": str(worktree),
      "MJLAB_ROOT": str(root),
      "MJLAB_VIRTUAL_ENV": str(tmp_path / "missing-venv"),
      "MJLAB_SIM2SIM_MODE": "velocity_bootstrap",
      "MJLAB_ENABLE_ELASTIC_BAND": "0",
      "MJLAB_VELOCITY_BOOTSTRAP_POSE": "policy_default",
      "MJLAB_VELOCITY_BOOTSTRAP_ROOT": "home",
      "PATH": f"{fake_bin}:{env['PATH']}",
    }
  )

  proc = subprocess.run(
    ["bash", str(FLYING_SINGLE), "start"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    env=env,
  )

  assert proc.returncode == 0, proc.stderr
  assert "elastic band: 0" in proc.stdout
  assert "elastic band=0" in proc.stdout
  assert "elastic band enabled" not in proc.stdout

  artifact_dirs = sorted((worktree / "logs/flying_kick_sim2sim").iterdir())
  selected = artifact_dirs[-1] / "selected"
  sim_config = yaml.safe_load(
    (selected / "simulate_config.yaml").read_text(encoding="utf-8")
  )

  assert sim_config["enable_elastic_band"] == 0
