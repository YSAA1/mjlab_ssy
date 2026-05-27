from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

import onnx
import yaml
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_velocity_trace_replay import (
  replay_selected_trace,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_trace_replay.py"


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


def _write_trace_report(path: Path, policy_root: Path, *, raw_value: float) -> None:
  obs = [0.0] * 98
  obs[0] = 2.0
  obs[3:6] = [0.0, 0.0, -1.0]
  obs[40] = 100.0
  raw = [raw_value] * 29
  processed = [raw_value * 0.5] * 29
  path.write_text(
    json.dumps(
      {
        "deploy_yaml": str(policy_root / "params/deploy.yaml"),
        "observation_terms": [
          {"name": "base_ang_vel", "start": 0, "end": 3, "dim": 3},
          {"name": "projected_gravity", "start": 3, "end": 6, "dim": 3},
          {"name": "velocity_commands", "start": 6, "end": 9, "dim": 3},
          {"name": "gait_phase", "start": 9, "end": 11, "dim": 2},
          {"name": "joint_pos_rel", "start": 11, "end": 40, "dim": 29},
          {"name": "joint_vel_rel", "start": 40, "end": 69, "dim": 29},
          {"name": "last_action", "start": 69, "end": 98, "dim": 29},
        ],
        "selected_trace": {
          "line": 12,
          "step": 25,
          "obs": obs,
          "raw_action": raw,
          "processed_action": processed,
        },
      }
    )
    + "\n",
    encoding="utf-8",
  )


def test_trace_replay_matches_logged_deploy_action(tmp_path: Path) -> None:
  policy_root = tmp_path / "policy"
  _write_deploy_yaml(policy_root / "params/deploy.yaml", dim=29)
  _write_constant_policy(
    policy_root / "exported/policy.onnx", input_dim=98, output_dim=29
  )
  trace_report = tmp_path / "trace.json"
  _write_trace_report(trace_report, policy_root, raw_value=1.0)

  report = replay_selected_trace(trace_report=trace_report)

  assert report["decision"]["replay_matches_deploy_log"] is True
  assert report["replay"]["raw_action_gap_l2"] == 0.0
  assert report["replay"]["processed_action_gap_l2"] == 0.0
  assert abs(report["selected_trace"]["obs"]["l2"] - math.sqrt(10005.0)) < 1e-5
  assert report["counterfactuals"][0]["name"] == "selected_without_joint_vel"


def test_trace_replay_cli_fails_when_logged_action_differs(tmp_path: Path) -> None:
  policy_root = tmp_path / "policy"
  _write_deploy_yaml(policy_root / "params/deploy.yaml", dim=29)
  _write_constant_policy(
    policy_root / "exported/policy.onnx", input_dim=98, output_dim=29
  )
  trace_report = tmp_path / "trace.json"
  _write_trace_report(trace_report, policy_root, raw_value=2.0)

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--trace-report",
      str(trace_report),
      "--expect-replay-match",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 1
  assert '"replay_matches_deploy_log": false' in proc.stdout
