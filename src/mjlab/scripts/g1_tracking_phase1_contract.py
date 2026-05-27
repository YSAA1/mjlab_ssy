"""Validate the new-G1 phase-1 sim/deploy control contract."""

from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import onnx
import yaml

EXPECTED_DOF = 29


class ContractError(RuntimeError):
  """Raised when the contract validator cannot run."""


@dataclass(frozen=True)
class ContractConfig:
  manifest: Path
  forbid_g1_23dof: bool
  report_out: Path | None


def _path(record: dict[str, Any]) -> Path:
  return Path(record["path"]).expanduser().resolve()


def _load_json(path: Path) -> dict[str, Any]:
  return json.loads(path.read_text(encoding="utf-8"))


def _xml_contract(path: Path) -> dict[str, Any]:
  root = ET.parse(path).getroot()
  joints = [joint.attrib["name"] for joint in root.findall(".//joint")]
  motors = root.findall(".//actuator/motor")
  actuator_names = [motor.attrib.get("name", "") for motor in motors]
  actuator_joints = [motor.attrib.get("joint", "") for motor in motors]
  text = path.read_text(encoding="utf-8", errors="replace")
  return {
    "path": str(path),
    "model": root.attrib.get("model") or root.attrib.get("name"),
    "joint_count": len(joints),
    "joint_names": joints,
    "actuator_count": len(motors),
    "actuator_names": actuator_names,
    "actuator_joints": actuator_joints,
    "mentions_g1_23dof": "g1_23dof" in text,
  }


def _urdf_info(path: Path) -> dict[str, Any]:
  root = ET.parse(path).getroot()
  movable_joints = [
    joint.attrib.get("name", "")
    for joint in root.findall(".//joint")
    if joint.attrib.get("type") != "fixed"
  ]
  return {
    "path": str(path),
    "robot_name": root.attrib.get("name"),
    "movable_joint_count": len(movable_joints),
    "movable_joint_names": movable_joints,
  }


def _npz_motion_info(path: Path) -> dict[str, Any]:
  data = np.load(path, allow_pickle=False)
  info: dict[str, Any] = {"path": str(path), "keys": sorted(data.files)}
  for key in ("joint_pos", "joint_vel"):
    if key in data:
      info[key] = {
        "shape": list(data[key].shape),
        "joint_dim": int(data[key].shape[1]) if data[key].ndim >= 2 else None,
      }
    else:
      info[key] = None
  return info


def _onnx_shapes(path: Path) -> dict[str, Any]:
  model = onnx.load(path)

  def tensor_shape(value_info: Any) -> list[int | str]:
    dims: list[int | str] = []
    for dim in value_info.type.tensor_type.shape.dim:
      dims.append(dim.dim_value or dim.dim_param or "?")
    return dims

  return {
    "path": str(path),
    "inputs": {
      value.name: tensor_shape(value)
      for value in model.graph.input
      if value.type.HasField("tensor_type")
    },
    "outputs": {
      value.name: tensor_shape(value)
      for value in model.graph.output
      if value.type.HasField("tensor_type")
    },
  }


def _last_static_dim(shapes: dict[str, list[int | str]]) -> int | None:
  for shape in shapes.values():
    if shape and isinstance(shape[-1], int):
      return shape[-1]
  return None


def _deploy_yaml_info(path: Path) -> dict[str, Any]:
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  action_cfg = (data.get("actions") or {}).get("JointPositionAction") or {}
  return {
    "path": str(path),
    "joint_ids_map_len": len(data.get("joint_ids_map") or []),
    "joint_ids_map": data.get("joint_ids_map"),
    "stiffness_len": len(data.get("stiffness") or []),
    "damping_len": len(data.get("damping") or []),
    "default_joint_pos_len": len(data.get("default_joint_pos") or []),
    "default_pose_source": f"{path}:default_joint_pos",
    "action_scale_len": len(action_cfg.get("scale") or []),
    "action_scale_source": f"{path}:actions.JointPositionAction.scale",
    "action_offset_len": len(action_cfg.get("offset") or []),
    "deploy_gain_source": f"{path}:stiffness,damping",
    "step_dt": data.get("step_dt"),
  }


def _append_if(condition: bool, failures: list[str], message: str) -> None:
  if condition:
    failures.append(message)


def _validate_xmls(
  manifest: dict[str, Any], failures: list[str]
) -> dict[str, dict[str, Any]]:
  sources = manifest["robot_model_sources"]
  external = manifest["external_robot_assets"]
  xml_paths = {
    "user_g1_xml": _path(sources["user_g1_xml"]),
    "mjlab_g1_xml": _path(sources["mjlab_g1_xml"]),
    "external_g1_xml": _path(external["external_g1_xml"]),
    "external_scene_g1_xml": _path(external["external_scene_g1_xml"]),
  }
  xmls = {name: _xml_contract(path) for name, path in xml_paths.items()}
  reference = xmls["user_g1_xml"]
  for name, info in xmls.items():
    _append_if(
      info["joint_count"] != EXPECTED_DOF,
      failures,
      f"{name} joint_count={info['joint_count']} expected {EXPECTED_DOF}",
    )
    _append_if(
      info["joint_names"] != reference["joint_names"],
      failures,
      f"{name} joint order differs from user_g1_xml",
    )
    if info["actuator_count"] not in (0, EXPECTED_DOF):
      failures.append(
        f"{name} actuator_count={info['actuator_count']} expected 0 or {EXPECTED_DOF}"
      )
    if info["actuator_count"] == EXPECTED_DOF:
      _append_if(
        info["actuator_joints"] != info["joint_names"],
        failures,
        f"{name} actuator order does not match joint order",
      )

  scene = xmls["external_scene_g1_xml"]
  _append_if(
    scene["actuator_count"] != EXPECTED_DOF,
    failures,
    f"external_scene_g1_xml actuator_count={scene['actuator_count']} expected {EXPECTED_DOF}",
  )
  _append_if(
    scene["actuator_joints"] != reference["joint_names"],
    failures,
    "external_scene_g1_xml actuator joint order differs from user_g1_xml joint order",
  )
  return xmls


def _validate_motion_and_policy(
  action_name: str,
  action: dict[str, Any],
  failures: list[str],
) -> dict[str, Any]:
  motion_infos = {
    "source_motion_npz": _npz_motion_info(_path(action["source_motion_npz"])),
    "deploy_motion_npz": _npz_motion_info(_path(action["deploy_motion_npz"])),
  }
  for label, info in motion_infos.items():
    for key in ("joint_pos", "joint_vel"):
      entry = info.get(key)
      _append_if(
        entry is None,
        failures,
        f"{action_name}.{label} missing {key}",
      )
      if entry is not None:
        _append_if(
          entry["joint_dim"] != EXPECTED_DOF,
          failures,
          f"{action_name}.{label}.{key} joint_dim={entry['joint_dim']} expected {EXPECTED_DOF}",
        )

  onnx_infos = {
    "source_policy_onnx": _onnx_shapes(_path(action["source_policy_onnx"])),
    "deploy_policy_onnx": _onnx_shapes(_path(action["deploy_policy_onnx"])),
  }
  for label, info in onnx_infos.items():
    action_dim = _last_static_dim(info["outputs"])
    _append_if(
      action_dim != EXPECTED_DOF,
      failures,
      f"{action_name}.{label} action_dim={action_dim} expected {EXPECTED_DOF}",
    )

  deploy = _deploy_yaml_info(_path(action["deploy_yaml"]))
  for key in (
    "joint_ids_map_len",
    "stiffness_len",
    "damping_len",
    "default_joint_pos_len",
    "action_scale_len",
    "action_offset_len",
  ):
    _append_if(
      deploy[key] != EXPECTED_DOF,
      failures,
      f"{action_name}.deploy_yaml {key}={deploy[key]} expected {EXPECTED_DOF}",
    )
  _append_if(
    deploy["joint_ids_map"] != list(range(EXPECTED_DOF)),
    failures,
    f"{action_name}.deploy_yaml joint_ids_map is not 0..{EXPECTED_DOF - 1}",
  )

  return {
    "motion": motion_infos,
    "onnx": onnx_infos,
    "deploy_yaml": deploy,
  }


def validate_contract(
  manifest: dict[str, Any], *, forbid_g1_23dof: bool = False
) -> dict[str, Any]:
  failures: list[str] = []
  xmls = _validate_xmls(manifest, failures)
  urdf = _urdf_info(_path(manifest["robot_model_sources"]["user_g1_urdf"]))
  _append_if(
    urdf["movable_joint_count"] != EXPECTED_DOF,
    failures,
    f"user_g1_urdf movable_joint_count={urdf['movable_joint_count']} expected {EXPECTED_DOF}",
  )

  if forbid_g1_23dof:
    active_paths = [
      manifest["robot_model_sources"]["user_g1_xml"]["path"],
      manifest["robot_model_sources"]["mjlab_g1_xml"]["path"],
      manifest["external_robot_assets"]["external_g1_xml"]["path"],
      manifest["external_robot_assets"]["external_scene_g1_xml"]["path"],
    ]
    for path in active_paths:
      _append_if(
        "g1_23dof" in path,
        failures,
        f"active robot asset points at forbidden g1_23dof path: {path}",
      )
    for name, info in xmls.items():
      _append_if(
        info["mentions_g1_23dof"],
        failures,
        f"{name} mentions forbidden g1_23dof",
      )

  action_reports = {
    name: _validate_motion_and_policy(name, action, failures)
    for name, action in manifest["actions"].items()
  }
  return {
    "schema_version": 1,
    "passed": not failures,
    "expected_dof": EXPECTED_DOF,
    "failures": failures,
    "ordered_joint_names": xmls["user_g1_xml"]["joint_names"],
    "ordered_actuator_names": xmls["external_scene_g1_xml"]["actuator_names"],
    "ordered_actuator_joints": xmls["external_scene_g1_xml"]["actuator_joints"],
    "xml_contracts": xmls,
    "urdf": urdf,
    "actions": action_reports,
  }


def parse_args(argv: list[str] | None = None) -> ContractConfig:
  parser = argparse.ArgumentParser(
    description="Validate the G1 tracking phase-1 robot/control contract."
  )
  parser.add_argument("--manifest", required=True, type=Path)
  parser.add_argument("--forbid-g1-23dof", action="store_true")
  parser.add_argument("--report-out", type=Path, default=None)
  args = parser.parse_args(argv)
  return ContractConfig(
    manifest=args.manifest,
    forbid_g1_23dof=args.forbid_g1_23dof,
    report_out=args.report_out,
  )


def main(argv: list[str] | None = None) -> int:
  config = parse_args(argv)
  try:
    manifest = _load_json(config.manifest)
    report = validate_contract(manifest, forbid_g1_23dof=config.forbid_g1_23dof)
    output = json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True)
    if config.report_out is not None:
      config.report_out.parent.mkdir(parents=True, exist_ok=True)
      config.report_out.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0 if report["passed"] else 2
  except (ContractError, OSError, KeyError, ET.ParseError, yaml.YAMLError) as exc:
    print(f"Contract validation failed to run: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
