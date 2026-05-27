from __future__ import annotations

import math
import subprocess
import sys
from pathlib import Path

import onnx
import yaml
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_velocity_zero_command_replay import (
  replay_zero_command,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_zero_command_replay.py"


def _write_constant_policy(path: Path, *, input_dim: int, output_dim: int) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  obs = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, input_dim])
  actions = helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, output_dim])
  tensor = helper.make_tensor(
    "constant_actions",
    TensorProto.FLOAT,
    [1, output_dim],
    [1.0] * output_dim,
  )
  node = helper.make_node("Constant", [], ["actions"], value=tensor)
  graph = helper.make_graph([node], "constant-policy", [obs], [actions])
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


def test_zero_command_replay_reports_processed_target_gap(tmp_path: Path) -> None:
  policy_root = tmp_path / "policy"
  dim = 29
  _write_deploy_yaml(policy_root / "params/deploy.yaml", dim=dim)
  _write_constant_policy(
    policy_root / "exported/policy.onnx", input_dim=98, output_dim=dim
  )

  report = replay_zero_command(policy_root=policy_root, steps=2)

  assert report["available"] is True
  assert report["steps"][0]["obs_dim"] == 98
  assert report["steps"][0]["raw_action_l2"] == round(math.sqrt(dim), 6)
  assert report["steps"][0]["processed_target_gap_l2"] == round(math.sqrt(dim) * 0.5, 6)
  assert report["zero_command_target_is_default"] is False


def test_zero_command_cli_expect_nonzero_target_gap(tmp_path: Path) -> None:
  policy_root = tmp_path / "policy"
  dim = 29
  _write_deploy_yaml(policy_root / "params/deploy.yaml", dim=dim)
  _write_constant_policy(
    policy_root / "exported/policy.onnx", input_dim=98, output_dim=dim
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--policy-root",
      str(policy_root),
      "--steps",
      "2",
      "--expect-nonzero-target-gap",
      "--target-gap-threshold",
      "0.5",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"zero_command_target_is_default": false' in proc.stdout
