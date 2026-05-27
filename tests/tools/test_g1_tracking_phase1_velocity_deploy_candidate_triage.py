from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import onnx
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_velocity_deploy_candidate_triage import (
  CURRENT_SOURCE_FLAT_OBSERVATIONS,
  triage_velocity_deploy_candidates,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_deploy_candidate_triage.py"
ACTIVE_V0_OBS = [
  "base_ang_vel",
  "projected_gravity",
  "command",
  "phase",
  "joint_pos",
  "joint_vel",
  "actions",
]


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


def test_triage_separates_direct_deploy_and_current_source_candidates(
  tmp_path: Path,
) -> None:
  reference = tmp_path / "deploy/policy.onnx"
  _write_policy(
    reference,
    input_dim=98,
    output_dim=29,
    observation_names=ACTIVE_V0_OBS,
  )
  direct_run = tmp_path / "logs/rsl_rl/g1_velocity/direct_ready"
  _write_policy(
    direct_run / "exported/policy.onnx",
    input_dim=98,
    output_dim=29,
    observation_names=ACTIVE_V0_OBS,
  )
  (direct_run / "params").mkdir(parents=True)
  (direct_run / "params/deploy.yaml").write_text("observations: {}\n")

  current_run = tmp_path / "logs/rsl_rl/g1_velocity/current_source"
  _write_policy(
    current_run / "current_source.onnx",
    input_dim=99,
    output_dim=29,
    observation_names=CURRENT_SOURCE_FLAT_OBSERVATIONS,
  )
  (current_run / "params").mkdir(parents=True)
  (current_run / "params/env.yaml").write_text("env: {}\n")
  (current_run / "params/agent.yaml").write_text("agent: {}\n")
  (current_run / "model_42.pt").write_text("checkpoint\n")

  report = triage_velocity_deploy_candidates(
    reference_policy=reference,
    search_roots=[tmp_path / "logs/rsl_rl"],
  )

  assert report["counts"]["direct_swap_ready"] == 1
  assert report["counts"]["current_source_flat_velocity_actor"] == 1
  assert report["counts"]["actor_reexport_ready"] == 1
  assert report["decision"]["direct_replacement_available"] is True
  current_candidates = report["current_source_reexport_candidates"]
  assert len(current_candidates) == 1
  assert current_candidates[0]["checkpoints"]["latest_iteration"] == 42
  assert (
    "requires_99_dim_runtime_observation_support" in current_candidates[0]["blockers"]
  )


def test_triage_expect_no_direct_ready_cli(tmp_path: Path) -> None:
  reference = tmp_path / "deploy/policy.onnx"
  candidate = tmp_path / "logs/rsl_rl/g1_velocity/current_source/policy.onnx"
  _write_policy(
    reference,
    input_dim=98,
    output_dim=29,
    observation_names=ACTIVE_V0_OBS,
  )
  _write_policy(
    candidate,
    input_dim=99,
    output_dim=29,
    observation_names=CURRENT_SOURCE_FLAT_OBSERVATIONS,
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--reference-policy",
      str(reference),
      "--search-root",
      str(tmp_path / "logs/rsl_rl"),
      "--expect-no-direct-ready",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"direct_replacement_available": false' in proc.stdout


def test_triage_flags_missing_deploy_package_as_not_direct_ready(
  tmp_path: Path,
) -> None:
  reference = tmp_path / "deploy/policy.onnx"
  candidate = tmp_path / "logs/rsl_rl/g1_velocity/loose/policy.onnx"
  _write_policy(
    reference,
    input_dim=98,
    output_dim=29,
    observation_names=ACTIVE_V0_OBS,
  )
  _write_policy(
    candidate,
    input_dim=98,
    output_dim=29,
    observation_names=ACTIVE_V0_OBS,
  )

  report = triage_velocity_deploy_candidates(
    reference_policy=reference,
    search_roots=[tmp_path / "logs/rsl_rl"],
  )

  assert report["counts"]["active_v0_contract"] == 1
  assert report["counts"]["direct_swap_ready"] == 0
  item = report["candidates"][0]
  assert item["direct_swap_ready"] is False
  assert "missing_complete_unitree_deploy_package" in item["blockers"]
