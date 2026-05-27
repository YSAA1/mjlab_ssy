from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import onnx
import yaml
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_velocity_deploy98_package import (
  DEPLOY98_OBSERVATIONS,
  build_deploy98_package,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_deploy98_package.py"


def _write_policy(path: Path, *, observation_names: list[str]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  obs = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 98])
  actions = helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 29])
  node = helper.make_node("Identity", ["obs"], ["actions"])
  graph = helper.make_graph([node], "policy", [obs], [actions])
  model = helper.make_model(graph)
  metadata = {
    "run_path": "test-run",
    "observation_names": ",".join(observation_names),
    "joint_names": ",".join(f"joint_{index}" for index in range(29)),
    "default_joint_pos": ",".join("0.1" for _ in range(29)),
    "joint_stiffness": ",".join("40.0" for _ in range(29)),
    "joint_damping": ",".join("2.0" for _ in range(29)),
    "action_scale": ",".join("0.5" for _ in range(29)),
  }
  for key, value in metadata.items():
    prop = model.metadata_props.add()
    prop.key = key
    prop.value = value
  onnx.save(model, path)


def _write_template(path: Path) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(
    yaml.safe_dump(
      {
        "joint_ids_map": list(range(29)),
        "step_dt": 0.02,
        "stiffness": [1.0] * 29,
        "damping": [1.0] * 29,
        "default_joint_pos": [0.0] * 29,
        "commands": {"base_velocity": {"ranges": {"lin_vel_x": [-0.5, 1.0]}}},
        "actions": {"JointPositionAction": {"scale": [0.1] * 29, "offset": [0.0] * 29}},
        "observations": {
          "base_ang_vel": {"params": {}},
          "projected_gravity": {"params": {}},
          "velocity_commands": {"params": {"command_name": "base_velocity"}},
          "gait_phase": {"params": {"period": 0.6}},
          "joint_pos_rel": {"params": {}},
          "joint_vel_rel": {"params": {}},
          "last_action": {"params": {}},
        },
      },
      sort_keys=False,
    ),
    encoding="utf-8",
  )


def test_build_deploy98_package_writes_policy_dir(tmp_path: Path) -> None:
  policy = tmp_path / "policy.onnx"
  template = tmp_path / "template.yaml"
  out_dir = tmp_path / "policy_dir"
  _write_policy(policy, observation_names=DEPLOY98_OBSERVATIONS)
  _write_template(template)

  report = build_deploy98_package(
    policy_onnx=policy,
    template_deploy_yaml=template,
    out_dir=out_dir,
  )

  assert report["decision"]["compatible"] is True
  assert report["decision"]["package_written"] is True
  assert report["decision"]["safe_to_run_zero_command_replay"] is True
  assert report["decision"]["safe_to_use_for_sim2sim"] is False
  assert report["decision"]["real_robot_gate"] == "locked"
  assert (out_dir / "exported/policy.onnx").is_file()
  generated = yaml.safe_load((out_dir / "params/deploy.yaml").read_text())
  assert list(generated["observations"]) == [
    "base_ang_vel",
    "projected_gravity",
    "velocity_commands",
    "gait_phase",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
  ]
  assert generated["actions"]["JointPositionAction"]["scale"] == [0.5] * 29
  assert generated["actions"]["JointPositionAction"]["offset"] == [0.1] * 29


def test_build_deploy98_package_rejects_wrong_observation_contract(
  tmp_path: Path,
) -> None:
  policy = tmp_path / "policy.onnx"
  template = tmp_path / "template.yaml"
  _write_policy(policy, observation_names=["base_lin_vel", *DEPLOY98_OBSERVATIONS])
  _write_template(template)

  report = build_deploy98_package(
    policy_onnx=policy,
    template_deploy_yaml=template,
    out_dir=tmp_path / "policy_dir",
  )

  assert report["decision"]["compatible"] is False
  assert report["decision"]["package_written"] is False
  assert report["decision"]["safe_to_use_for_sim2sim"] is False
  assert report["metadata"]["errors"]


def test_build_deploy98_package_rejects_wrong_onnx_dims(tmp_path: Path) -> None:
  policy = tmp_path / "policy.onnx"
  template = tmp_path / "template.yaml"
  _write_policy(policy, observation_names=DEPLOY98_OBSERVATIONS)
  _write_template(template)

  obs = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 99])
  actions = helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 29])
  graph = helper.make_graph([], "bad_policy", [obs], [actions])
  model = helper.make_model(graph)
  original = onnx.load(policy)
  model.metadata_props.extend(original.metadata_props)
  onnx.save(model, policy)

  report = build_deploy98_package(
    policy_onnx=policy,
    template_deploy_yaml=template,
    out_dir=tmp_path / "policy_dir",
  )

  assert report["decision"]["compatible"] is False
  assert "input_dim 99 != 98" in report["metadata"]["errors"]


def test_build_deploy98_package_copies_external_data_sidecar(tmp_path: Path) -> None:
  policy = tmp_path / "policy.onnx"
  template = tmp_path / "template.yaml"
  out_dir = tmp_path / "policy_dir"
  _write_policy(policy, observation_names=DEPLOY98_OBSERVATIONS)
  policy.with_name(policy.name + ".data").write_bytes(b"sidecar")
  _write_template(template)

  report = build_deploy98_package(
    policy_onnx=policy,
    template_deploy_yaml=template,
    out_dir=out_dir,
  )

  assert report["decision"]["package_written"] is True
  assert (out_dir / "exported/policy.onnx.data").read_bytes() == b"sidecar"
  assert report["written"]["policy_data"] == str(out_dir / "exported/policy.onnx.data")


def test_deploy98_package_cli_dry_run(tmp_path: Path) -> None:
  policy = tmp_path / "policy.onnx"
  template = tmp_path / "template.yaml"
  out_dir = tmp_path / "policy_dir"
  _write_policy(policy, observation_names=DEPLOY98_OBSERVATIONS)
  _write_template(template)

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--policy-onnx",
      str(policy),
      "--template-deploy-yaml",
      str(template),
      "--out-dir",
      str(out_dir),
      "--dry-run",
      "--expect-compatible",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"compatible": true' in proc.stdout
  assert not out_dir.exists()
