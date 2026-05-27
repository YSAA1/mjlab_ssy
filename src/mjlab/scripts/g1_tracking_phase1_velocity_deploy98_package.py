"""Create a Unitree deploy policy_dir from a deploy98 Velocity ONNX."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import yaml

from mjlab.scripts.g1_tracking_phase1_velocity_deploy98_task_contract import (
  DEPLOY98_TO_RUNTIME,
)
from mjlab.scripts.g1_tracking_phase1_velocity_policy_inventory import (
  _metadata_list,
  _onnx_summary,
)

DEPLOY98_OBSERVATIONS = list(DEPLOY98_TO_RUNTIME)
EXPECTED_INPUT_DIM = 98
EXPECTED_OUTPUT_DIM = 29


class Deploy98PackageError(RuntimeError):
  """Raised when a deploy98 package cannot be generated safely."""


def _float_list(value: str | None, *, name: str) -> list[float]:
  if not value:
    raise Deploy98PackageError(f"ONNX metadata missing {name}")
  try:
    return [float(item.strip()) for item in value.split(",") if item.strip()]
  except ValueError as exc:
    raise Deploy98PackageError(f"ONNX metadata {name} must be numeric") from exc


def _raw_metadata(path: Path) -> dict[str, str]:
  try:
    import onnx
  except Exception as exc:  # pragma: no cover - depends on local runtime install.
    raise Deploy98PackageError(f"onnx_import_failed: {exc}") from exc

  model = onnx.load(path)
  return {prop.key: prop.value for prop in model.metadata_props}


def _last_dim(summary: dict[str, Any], key: str) -> int | str | None:
  values = summary.get(key)
  if not isinstance(values, list) or not values:
    return None
  first = values[0]
  if not isinstance(first, dict):
    return None
  dims = first.get("dims")
  if not isinstance(dims, list) or not dims:
    return None
  return dims[-1]


def _mapped_observations(
  template: dict[str, Any], observation_names: list[str]
) -> dict[str, Any]:
  template_observations = template.get("observations")
  if not isinstance(template_observations, dict):
    raise Deploy98PackageError("template deploy.yaml missing observations mapping")

  mapped: dict[str, Any] = {}
  for name in observation_names:
    runtime_name = DEPLOY98_TO_RUNTIME.get(name)
    if runtime_name is None:
      raise Deploy98PackageError(f"unsupported deploy98 observation: {name}")
    if runtime_name not in template_observations:
      raise Deploy98PackageError(f"template deploy.yaml missing {runtime_name}")
    mapped[runtime_name] = template_observations[runtime_name]
  return mapped


def _validate_metadata(
  metadata: dict[str, str],
  *,
  input_dim: int | str | None,
  output_dim: int | str | None,
) -> dict[str, Any]:
  observation_names = _metadata_list(metadata.get("observation_names"))
  joint_names = _metadata_list(metadata.get("joint_names"))
  default_joint_pos = _float_list(
    metadata.get("default_joint_pos"), name="default_joint_pos"
  )
  joint_stiffness = _float_list(metadata.get("joint_stiffness"), name="joint_stiffness")
  joint_damping = _float_list(metadata.get("joint_damping"), name="joint_damping")
  action_scale = _float_list(metadata.get("action_scale"), name="action_scale")
  errors: list[str] = []
  if observation_names != DEPLOY98_OBSERVATIONS:
    errors.append(
      f"observation_names must be {DEPLOY98_OBSERVATIONS}, got {observation_names}"
    )
  if input_dim != EXPECTED_INPUT_DIM:
    errors.append(f"input_dim {input_dim} != {EXPECTED_INPUT_DIM}")
  if output_dim != EXPECTED_OUTPUT_DIM:
    errors.append(f"output_dim {output_dim} != {EXPECTED_OUTPUT_DIM}")
  for name, values in (
    ("joint_names", joint_names),
    ("default_joint_pos", default_joint_pos),
    ("joint_stiffness", joint_stiffness),
    ("joint_damping", joint_damping),
    ("action_scale", action_scale),
  ):
    if len(values) != 29:
      errors.append(f"{name} length {len(values)} != 29")
  return {
    "observation_names": observation_names,
    "joint_names": joint_names,
    "default_joint_pos": default_joint_pos,
    "joint_stiffness": joint_stiffness,
    "joint_damping": joint_damping,
    "action_scale": action_scale,
    "errors": errors,
  }


def build_deploy98_package(
  *,
  policy_onnx: Path,
  template_deploy_yaml: Path,
  out_dir: Path,
  dry_run: bool = False,
) -> dict[str, Any]:
  summary = _onnx_summary(policy_onnx)
  if summary.get("available") is not True:
    raise Deploy98PackageError(str(summary.get("reason", "policy_onnx_unavailable")))

  input_dim = _last_dim(summary, "inputs")
  output_dim = _last_dim(summary, "outputs")
  metadata = _raw_metadata(policy_onnx)
  validated = _validate_metadata(metadata, input_dim=input_dim, output_dim=output_dim)
  template = yaml.safe_load(template_deploy_yaml.read_text(encoding="utf-8"))
  if not isinstance(template, dict):
    raise Deploy98PackageError("template deploy.yaml root must be a mapping")

  compatible = not validated["errors"]
  deploy_yaml = dict(template)
  if compatible:
    deploy_yaml["joint_ids_map"] = list(range(29))
    deploy_yaml["stiffness"] = validated["joint_stiffness"]
    deploy_yaml["damping"] = validated["joint_damping"]
    deploy_yaml["default_joint_pos"] = validated["default_joint_pos"]
    deploy_yaml["observations"] = _mapped_observations(
      template,
      validated["observation_names"],
    )
    actions = deploy_yaml.setdefault("actions", {})
    joint_action = actions.setdefault("JointPositionAction", {})
    joint_action["joint_names"] = [".*"]
    joint_action["joint_ids"] = None
    joint_action["scale"] = validated["action_scale"]
    joint_action["offset"] = validated["default_joint_pos"]

  written: dict[str, str | None] = {
    "policy": None,
    "policy_data": None,
    "deploy_yaml": None,
  }
  if compatible and not dry_run:
    exported_dir = out_dir / "exported"
    params_dir = out_dir / "params"
    exported_dir.mkdir(parents=True, exist_ok=True)
    params_dir.mkdir(parents=True, exist_ok=True)
    policy_out = exported_dir / "policy.onnx"
    yaml_out = params_dir / "deploy.yaml"
    shutil.copy2(policy_onnx, policy_out)
    policy_data = policy_onnx.with_name(policy_onnx.name + ".data")
    policy_data_out = None
    if policy_data.is_file():
      policy_data_out = exported_dir / policy_data.name
      shutil.copy2(policy_data, policy_data_out)
    yaml_out.write_text(
      yaml.safe_dump(deploy_yaml, sort_keys=False),
      encoding="utf-8",
    )
    written = {
      "policy": str(policy_out),
      "policy_data": str(policy_data_out) if policy_data_out else None,
      "deploy_yaml": str(yaml_out),
    }

  return {
    "schema_version": 1,
    "policy_onnx": str(policy_onnx),
    "template_deploy_yaml": str(template_deploy_yaml),
    "out_dir": str(out_dir),
    "dry_run": dry_run,
    "onnx": {
      "input_dim": input_dim,
      "output_dim": output_dim,
      "run_path": metadata.get("run_path"),
    },
    "metadata": {
      "observation_names": validated["observation_names"],
      "joint_names_count": len(validated["joint_names"]),
      "default_joint_pos_count": len(validated["default_joint_pos"]),
      "joint_stiffness_count": len(validated["joint_stiffness"]),
      "joint_damping_count": len(validated["joint_damping"]),
      "action_scale_count": len(validated["action_scale"]),
      "errors": validated["errors"],
    },
    "deploy_yaml": {
      "observation_terms": list(deploy_yaml.get("observations", {}).keys())
      if isinstance(deploy_yaml.get("observations"), dict)
      else [],
      "action_scale_count": len(
        deploy_yaml.get("actions", {}).get("JointPositionAction", {}).get("scale", [])
      )
      if isinstance(deploy_yaml.get("actions"), dict)
      else 0,
    },
    "decision": {
      "compatible": compatible,
      "package_written": bool(written["policy"] and written["deploy_yaml"]),
      "safe_to_run_zero_command_replay": bool(
        written["policy"] and written["deploy_yaml"]
      ),
      "safe_to_use_for_sim2sim": False,
      "sim2sim_gate": "requires_zero_command_replay_and_velocity_gui_smoke",
      "safe_to_swap_without_zero_command_replay": False,
      "real_robot_gate": "locked",
    },
    "written": written,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Create a Unitree deploy policy_dir from a deploy98 Velocity ONNX."
  )
  parser.add_argument("--policy-onnx", type=Path, required=True)
  parser.add_argument(
    "--template-deploy-yaml",
    type=Path,
    default=Path(
      "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/"
      "config/policy/velocity/v0/params/deploy.yaml"
    ),
  )
  parser.add_argument("--out-dir", type=Path, required=True)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--dry-run", action="store_true")
  parser.add_argument("--expect-compatible", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    report = build_deploy98_package(
      policy_onnx=args.policy_onnx,
      template_deploy_yaml=args.template_deploy_yaml,
      out_dir=args.out_dir,
      dry_run=args.dry_run,
    )
  except Deploy98PackageError as exc:
    report = {"schema_version": 1, "decision": {"compatible": False}, "error": str(exc)}

  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_compatible:
    return 0 if report["decision"]["compatible"] else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
