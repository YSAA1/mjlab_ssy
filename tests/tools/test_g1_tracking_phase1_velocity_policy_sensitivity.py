from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import onnx
import yaml
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_velocity_policy_sensitivity import (
  probe_policy_sensitivity,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_policy_sensitivity.py"


def _write_joint_vel_sensitive_policy(
  path: Path,
  *,
  input_dim: int,
  output_dim: int,
  joint_vel_start: int,
) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  obs = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, input_dim])
  actions = helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, output_dim])
  weights = [0.0] * (input_dim * output_dim)
  weights[joint_vel_start * output_dim] = 2.0
  weights_tensor = helper.make_tensor(
    "weights",
    TensorProto.FLOAT,
    [input_dim, output_dim],
    weights,
  )
  bias_tensor = helper.make_tensor(
    "bias", TensorProto.FLOAT, [output_dim], [0.0] * output_dim
  )
  node = helper.make_node("Gemm", ["obs", "weights", "bias"], ["actions"])
  graph = helper.make_graph(
    [node],
    "joint-vel-sensitive-policy",
    [obs],
    [actions],
    [weights_tensor, bias_tensor],
  )
  model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
  model.ir_version = 10
  onnx.save(model, path)


def _write_deploy_yaml(path: Path, *, dim: int) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(
    yaml.safe_dump(
      {
        "default_joint_pos": [0.0] * dim,
        "actions": {
          "JointPositionAction": {
            "scale": [0.5] * dim,
            "offset": [0.0] * dim,
          }
        },
        "observations": {
          "base_ang_vel": {"scale": [1.0, 1.0, 1.0]},
          "projected_gravity": {"scale": [1.0, 1.0, 1.0]},
          "velocity_commands": {"scale": [1.0, 1.0, 1.0]},
          "gait_phase": {"scale": [1.0, 1.0]},
          "joint_pos_rel": {"scale": [1.0] * dim},
          "joint_vel_rel": {"scale": [1.0] * dim},
          "last_action": {"scale": [1.0] * dim},
        },
      },
      sort_keys=False,
    ),
    encoding="utf-8",
  )


def test_policy_sensitivity_identifies_joint_velocity_term(tmp_path: Path) -> None:
  policy_root = tmp_path / "policy"
  dim = 29
  _write_deploy_yaml(policy_root / "params/deploy.yaml", dim=dim)
  _write_joint_vel_sensitive_policy(
    policy_root / "exported/policy.onnx",
    input_dim=98,
    output_dim=dim,
    joint_vel_start=40,
  )

  report = probe_policy_sensitivity(
    policy_root=policy_root,
    warmup_steps=1,
    magnitudes={"joint_vel_rel": 10.0, "last_action": 1.0},
    top_k=4,
  )

  assert report["available"] is True
  assert report["decision"]["highest_sensitivity_term"] == "joint_vel_rel"
  assert report["term_summary"][0]["term"] == "joint_vel_rel"
  assert report["term_summary"][0]["worst_index"] == 0
  assert report["top_cases"][0]["raw_action_l2"] == 20.0
  assert report["top_cases"][0]["processed_target_gap_l2"] == 10.0
  assert (
    report["decision"]["policy_can_amplify_deploy_observation_perturbations"] is True
  )


def test_policy_sensitivity_cli_expect_sensitive(tmp_path: Path) -> None:
  policy_root = tmp_path / "policy"
  dim = 29
  _write_deploy_yaml(policy_root / "params/deploy.yaml", dim=dim)
  _write_joint_vel_sensitive_policy(
    policy_root / "exported/policy.onnx",
    input_dim=98,
    output_dim=dim,
    joint_vel_start=40,
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--policy-root",
      str(policy_root),
      "--warmup-steps",
      "1",
      "--magnitude",
      "joint_vel_rel=10",
      "--expect-sensitive",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"highest_sensitivity_term": "joint_vel_rel"' in proc.stdout
