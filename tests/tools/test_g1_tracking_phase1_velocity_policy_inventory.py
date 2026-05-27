from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import onnx
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_velocity_policy_inventory import (
  inventory_velocity_policies,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_policy_inventory.py"


def _write_policy(
  path: Path,
  *,
  input_dim: int,
  output_dim: int,
  observation_names: list[str],
) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  obs = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, input_dim])
  actions = helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, output_dim])
  node = helper.make_node("Identity", ["obs"], ["actions"])
  graph = helper.make_graph([node], "policy", [obs], [actions])
  model = helper.make_model(graph)
  metadata = {
    "run_path": path.parent.name,
    "observation_names": ",".join(observation_names),
    "command_names": "twist",
    "joint_names": ",".join(f"joint_{index}" for index in range(output_dim)),
  }
  for key, value in metadata.items():
    prop = model.metadata_props.add()
    prop.key = key
    prop.value = value
  onnx.save(model, path)


def test_inventory_marks_dimension_and_observation_mismatch(tmp_path: Path) -> None:
  reference = tmp_path / "deploy/policy.onnx"
  candidate = tmp_path / "logs/rsl_rl/g1_velocity/run/policy.onnx"
  _write_policy(
    reference,
    input_dim=98,
    output_dim=29,
    observation_names=["base_ang_vel", "projected_gravity", "gait_phase"],
  )
  _write_policy(
    candidate,
    input_dim=99,
    output_dim=29,
    observation_names=["base_lin_vel", "base_ang_vel", "projected_gravity"],
  )

  report = inventory_velocity_policies(
    reference_policy=reference,
    search_roots=[tmp_path / "logs/rsl_rl"],
  )

  assert report["candidate_count"] == 1
  assert report["compatible_count"] == 0
  item = report["candidates"][0]
  assert item["input_dim"] == 99
  assert item["compatible_with_reference"] is False
  assert "input_dim 99 != 98" in item["incompatibility_reasons"]
  assert "observation_names differ" in item["incompatibility_reasons"]


def test_inventory_accepts_compatible_candidate(tmp_path: Path) -> None:
  reference = tmp_path / "deploy/policy.onnx"
  candidate = tmp_path / "logs/rsl_rl/g1_velocity/run/policy.onnx"
  observation_names = ["base_ang_vel", "projected_gravity", "gait_phase"]
  _write_policy(
    reference,
    input_dim=98,
    output_dim=29,
    observation_names=observation_names,
  )
  _write_policy(
    candidate,
    input_dim=98,
    output_dim=29,
    observation_names=observation_names,
  )

  report = inventory_velocity_policies(
    reference_policy=reference,
    search_roots=[tmp_path / "logs/rsl_rl"],
  )

  assert report["candidate_count"] == 1
  assert report["compatible_count"] == 1
  assert report["compatible_candidates"] == [str(candidate.resolve())]


def test_cli_expect_no_compatible(tmp_path: Path) -> None:
  reference = tmp_path / "deploy/policy.onnx"
  candidate = tmp_path / "logs/rsl_rl/g1_velocity/run/policy.onnx"
  _write_policy(
    reference,
    input_dim=98,
    output_dim=29,
    observation_names=["base_ang_vel", "projected_gravity", "gait_phase"],
  )
  _write_policy(
    candidate,
    input_dim=99,
    output_dim=29,
    observation_names=["base_lin_vel", "base_ang_vel", "projected_gravity"],
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--reference-policy",
      str(reference),
      "--search-root",
      str(tmp_path / "logs/rsl_rl"),
      "--expect-no-compatible",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"compatible_count": 0' in proc.stdout
