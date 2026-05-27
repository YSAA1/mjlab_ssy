from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

from mjlab.scripts.g1_tracking_phase1_velocity_contract import (
  _foot_contact_report,
  _policy_provenance_report,
  analyze_velocity_contract,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_contract.py"


def _write(path: Path, content: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _fake_evidence(
  tmp_path: Path,
  *,
  initial: list[float],
  default: list[float],
  stable: bool = False,
) -> tuple[Path, Path]:
  evidence = tmp_path / "evidence"
  deploy_root = tmp_path / "deploy/robots/g1"
  policy_dir = deploy_root / "config/policy/velocity/v0"

  _write(
    evidence / "selected/simulate_config.yaml",
    yaml.safe_dump(
      {
        "robot": "g1",
        "robot_scene": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "interface": "lo",
        "enable_elastic_band": 1,
        "start_paused": 1,
        "initial_qpos": [0.0, 0.0, 0.76, 1.0, 0.0, 0.0, 0.0, *initial],
      },
      sort_keys=False,
    ),
  )
  _write(
    evidence / "selected/config.yaml",
    yaml.safe_dump(
      {
        "FSM": {
          "initial_state": "Velocity",
          "Velocity": {"policy_dir": "config/policy/velocity"},
        }
      },
      sort_keys=False,
    ),
  )
  _write(
    policy_dir / "params/deploy.yaml",
    yaml.safe_dump(
      {
        "joint_ids_map": list(range(len(default))),
        "stiffness": [1.0] * len(default),
        "damping": [1.0] * len(default),
        "default_joint_pos": default,
        "actions": {
          "JointPositionAction": {
            "offset": default,
            "scale": [0.1] * len(default),
          }
        },
      },
      sort_keys=False,
    ),
  )
  _write(policy_dir / "exported/policy.onnx", "fake onnx\n")
  if stable:
    _write(
      evidence / "g1_ctrl.log",
      "[2026-05-22 17:10:03.941] [info] [PHASE1] event=stable_sample "
      "state=Velocity stable=1 q_err_l2=0.100 q_err_max=0.050 "
      "base_vel_x=0.000 command_vel_x=0.000 gravity_b=(0.000,0.000,-1.000) "
      "root_ang_vel_l2=0.000\n",
    )
  else:
    _write(
      evidence / "g1_ctrl.log",
      "[2026-05-22 17:10:03.941] [info] [PHASE1] event=stable_sample "
      "state=Velocity stable=1 q_err_l2=1.447 q_err_max=0.754 "
      "base_vel_x=0.000 command_vel_x=0.000 gravity_b=(0.000,0.000,-1.000) "
      "root_ang_vel_l2=0.000\n"
      "[2026-05-22 17:10:07.421] [info] [PHASE1] event=stable_sample "
      "state=Velocity stable=0 q_err_l2=5.614 q_err_max=4.030 "
      "base_vel_x=0.000 command_vel_x=0.000 gravity_b=(0.150,-0.963,-0.223) "
      "root_ang_vel_l2=6.364\n",
    )
  return evidence, deploy_root


def test_velocity_contract_flags_default_pose_mismatch(tmp_path: Path) -> None:
  evidence, deploy_root = _fake_evidence(
    tmp_path,
    initial=[0.0] * 29,
    default=[0.0, 0.0, 0.0, 0.8, *([0.0] * 25)],
  )

  report = analyze_velocity_contract(evidence, deploy_root=deploy_root)

  assert report["passed"] is False
  assert report["primary_reason"] == "velocity_default_pose_mismatch"
  assert report["count_passed"] is True
  assert report["initial_vs_default_joint_pos"]["gap_l2"] == 0.8
  assert report["initial_vs_action_offset"]["top_diffs"][0]["index"] == 3


def test_velocity_contract_flags_runtime_instability_after_pose_match(
  tmp_path: Path,
) -> None:
  evidence, deploy_root = _fake_evidence(
    tmp_path,
    initial=[0.0] * 29,
    default=[0.0] * 29,
  )

  report = analyze_velocity_contract(evidence, deploy_root=deploy_root)

  assert report["passed"] is False
  assert report["primary_reason"] == "velocity_runtime_instability"
  first_unstable = report["velocity_stability"]["first_unstable"]
  assert first_unstable["line"] == 2
  assert first_unstable["q_err_l2"] == 5.614
  assert report["deploy_observations"]["available"] is True
  assert report["deploy_observations"]["total_dim"] == 0
  assert report["onnx"]["available"] is False
  assert report["policy_provenance"]["reason"] == "onnx_run_path_missing"
  assert report["current_g1_source_deploy_deltas"]["available"] is True


def test_velocity_contract_parses_runtime_trace_fields(tmp_path: Path) -> None:
  evidence, deploy_root = _fake_evidence(
    tmp_path,
    initial=[0.0] * 29,
    default=[0.0] * 29,
  )
  _write(
    evidence / "g1_ctrl.log",
    "[2026-05-22 18:35:16.211] [info] [PHASE1] event=stable_sample "
    "state=Velocity stable=1 policy_step=25 q_err_l2=0.594 q_err_max=0.288 "
    "base_vel_x=0.000 command_vel_x=0.000 command_vel_y=-0.000 "
    "command_yaw=-0.000 command_norm=0.000 phase=0.867 raw_action_l2=1.682 "
    "raw_action_max=0.825 processed_action_l2=1.590 processed_action_max=0.881 "
    "joint_pos_rel_l2=0.000 joint_pos_rel_max=0.000 joint_vel_l2=0.000 "
    "joint_vel_max=0.000 gravity_b=(0.000,0.000,-1.000) root_ang_vel_l2=0.000\n"
    "[2026-05-22 18:35:17.211] [info] [PHASE1] event=stable_sample "
    "state=Velocity stable=0 policy_step=75 q_err_l2=3.686 q_err_max=1.657 "
    "base_vel_x=0.000 command_vel_x=0.000 command_vel_y=-0.000 "
    "command_yaw=-0.000 command_norm=0.000 phase=0.533 raw_action_l2=14.760 "
    "raw_action_max=7.827 processed_action_l2=4.874 processed_action_max=2.241 "
    "joint_pos_rel_l2=2.711 joint_pos_rel_max=1.359 joint_vel_l2=673.649 "
    "joint_vel_max=576.357 gravity_b=(-0.052,-0.643,-0.764) root_ang_vel_l2=8.491\n",
  )

  report = analyze_velocity_contract(evidence, deploy_root=deploy_root)
  first_unstable = report["velocity_stability"]["first_unstable"]

  assert report["primary_reason"] == "velocity_runtime_instability"
  assert first_unstable["policy_step"] == 75
  assert first_unstable["command_norm"] == 0.0
  assert first_unstable["raw_action_l2"] == 14.76
  assert first_unstable["processed_action_l2"] == 4.874
  assert first_unstable["joint_vel_l2"] == 673.649


def test_velocity_contract_passes_when_pose_and_runtime_match(tmp_path: Path) -> None:
  evidence, deploy_root = _fake_evidence(
    tmp_path,
    initial=[0.0] * 29,
    default=[0.0] * 29,
    stable=True,
  )

  report = analyze_velocity_contract(evidence, deploy_root=deploy_root)

  assert report["passed"] is True
  assert report["primary_reason"] is None


def test_policy_provenance_finds_matching_local_run_dir(tmp_path: Path) -> None:
  evidence = tmp_path / "case/deep/evidence"
  run_dir = tmp_path / "logs/rsl_rl/g1_velocity/2026-03-18_18-40-20"
  run_dir.mkdir(parents=True)

  report = _policy_provenance_report(
    onnx={"metadata": {"run_path": "2026-03-18_18-40-20"}},
    evidence_dir=evidence,
    deploy_root=tmp_path / "external/deploy/robots/g1",
  )

  assert report["source_run_found"] is True
  assert report["matched_run_dirs"] == [str(run_dir)]


def test_foot_contact_report_flags_initial_floor_penetration(
  tmp_path: Path,
) -> None:
  scene = tmp_path / "scene.xml"
  _write(
    scene,
    """
<mujoco model="contact-test">
  <worldbody>
    <body name="pelvis" pos="0 0 0">
      <freejoint/>
      <geom name="left_foot_collision" type="sphere" size="0.01" pos="0 0 0"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
  )

  report = _foot_contact_report(scene, [0.0, 0.0, -0.005, 1.0, 0.0, 0.0, 0.0])

  assert report["available"] is True
  assert report["floor_clearance_passed"] is False
  assert report["min_foot_surface_z"] == -0.015
  assert report["required_root_lift_to_clear_floor"] == 0.015


def test_cli_accepts_expected_velocity_failure(tmp_path: Path) -> None:
  evidence, deploy_root = _fake_evidence(
    tmp_path,
    initial=[0.0] * 29,
    default=[0.0, 0.0, 0.0, 0.8, *([0.0] * 25)],
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--evidence-dir",
      str(evidence),
      "--deploy-root",
      str(deploy_root),
      "--expect-failure",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  report = json.loads(proc.stdout)
  assert report["primary_reason"] == "velocity_default_pose_mismatch"
