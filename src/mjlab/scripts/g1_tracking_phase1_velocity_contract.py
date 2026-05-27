"""Diagnose the G1 phase-1 Velocity bootstrap contract."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml

STABLE_RE = re.compile(
  r"^(?P<prefix>.*?\[(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\].*?)"
  r"\[PHASE1\]\s+event=stable_sample\s+state=Velocity\s+"
  r"stable=(?P<stable>[01])\s+(?:policy_step=(?P<policy_step>\d+)\s+)?"
  r"q_err_l2=(?P<q_err_l2>-?\d+(?:\.\d+)?)\s+"
  r"q_err_max=(?P<q_err_max>-?\d+(?:\.\d+)?)\s+.*?"
  r"gravity_b=\((?P<gravity>[^)]*)\)\s+root_ang_vel_l2=(?P<root_ang_vel_l2>-?\d+(?:\.\d+)?)"
)
NUMERIC_FIELD_RE = re.compile(
  r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>-?\d+(?:\.\d+)?)"
)


class VelocityContractError(RuntimeError):
  """Raised when Velocity contract diagnosis inputs are invalid."""


def _load_yaml(path: Path) -> dict[str, Any]:
  with path.open("r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle)
  if not isinstance(data, dict):
    raise VelocityContractError(f"YAML root must be a mapping: {path}")
  return data


def _resolve_policy_dir(deploy_root: Path, policy_dir: str) -> Path:
  path = Path(policy_dir)
  if not path.is_absolute():
    path = deploy_root / path
  if (path / "exported").is_dir():
    return path
  candidates = sorted(child for child in path.iterdir() if child.is_dir())
  for candidate in reversed(candidates):
    if (candidate / "exported").is_dir():
      return candidate
  return path


def _as_float_list(value: Any, *, name: str) -> list[float]:
  if not isinstance(value, list):
    raise VelocityContractError(f"{name} must be a list")
  return [float(item) for item in value]


def _l2(values: list[float]) -> float:
  return math.sqrt(sum(value * value for value in values))


def _top_diffs(
  left: list[float], right: list[float], *, limit: int = 8
) -> list[dict[str, Any]]:
  diffs = [a - b for a, b in zip(left, right, strict=True)]
  order = sorted(range(len(diffs)), key=lambda idx: abs(diffs[idx]), reverse=True)
  return [
    {
      "index": idx,
      "initial_minus_target": round(diffs[idx], 6),
      "initial": round(left[idx], 6),
      "target": round(right[idx], 6),
    }
    for idx in order[:limit]
  ]


def _vector_report(
  initial: list[float],
  target: list[float],
  *,
  name: str,
  max_l2: float,
  max_abs: float,
) -> dict[str, Any]:
  if len(initial) != len(target):
    raise VelocityContractError(
      f"initial joint dim {len(initial)} does not match {name} dim {len(target)}"
    )
  diffs = [a - b for a, b in zip(initial, target, strict=True)]
  gap_l2 = _l2(diffs)
  gap_max = max(abs(value) for value in diffs) if diffs else 0.0
  return {
    "passed": gap_l2 <= max_l2 and gap_max <= max_abs,
    "gap_l2": round(gap_l2, 6),
    "gap_max": round(gap_max, 6),
    "max_gap_l2": max_l2,
    "max_gap_max": max_abs,
    "top_diffs": _top_diffs(initial, target),
  }


def _xml_joint_names(path: Path) -> list[str]:
  root = ET.parse(path).getroot()
  return [joint.attrib["name"] for joint in root.findall(".//joint")]


def _resolve_pattern_values(
  joint_names: list[str], patterns: dict[str, float], *, default: float = 0.0
) -> list[float]:
  values = [default] * len(joint_names)
  for pattern, value in patterns.items():
    compiled = re.compile(pattern)
    for index, joint_name in enumerate(joint_names):
      if compiled.fullmatch(joint_name):
        values[index] = float(value)
  return values


def _vector_delta(
  left: list[float], right: list[float], *, name: str
) -> dict[str, Any]:
  if len(left) != len(right):
    return {
      "available": False,
      "reason": f"{name}_dim_mismatch",
      "left_dim": len(left),
      "right_dim": len(right),
    }
  diffs = [a - b for a, b in zip(left, right, strict=True)]
  return {
    "available": True,
    "gap_l2": round(_l2(diffs), 6),
    "gap_max": round(max((abs(value) for value in diffs), default=0.0), 6),
    "top_diffs": _top_diffs(left, right),
  }


def _parse_velocity_stability(log_path: Path) -> dict[str, Any]:
  if not log_path.is_file():
    return {
      "log": str(log_path),
      "samples": 0,
      "stable_samples": 0,
      "first_unstable": None,
      "stable_duration_before_first_unstable_s": None,
    }

  first_timestamp: str | None = None
  last_stable_timestamp: str | None = None
  first_unstable: dict[str, Any] | None = None
  samples = 0
  stable_samples = 0
  for line_no, line in enumerate(
    log_path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1
  ):
    match = STABLE_RE.search(line)
    if not match:
      continue
    samples += 1
    timestamp = match.group("timestamp")
    first_timestamp = first_timestamp or timestamp
    stable = match.group("stable") == "1"
    if stable:
      stable_samples += 1
      last_stable_timestamp = timestamp
    elif first_unstable is None:
      gravity = [float(item) for item in match.group("gravity").split(",")]
      numeric_fields = {
        item.group("key"): float(item.group("value"))
        for item in NUMERIC_FIELD_RE.finditer(line)
      }
      first_unstable = {
        "line": line_no,
        "timestamp": timestamp,
        "q_err_l2": float(match.group("q_err_l2")),
        "q_err_max": float(match.group("q_err_max")),
        "gravity_b": gravity,
        "root_ang_vel_l2": float(match.group("root_ang_vel_l2")),
      }
      if match.group("policy_step") is not None:
        first_unstable["policy_step"] = int(match.group("policy_step"))
      for key in (
        "command_vel_x",
        "command_vel_y",
        "command_yaw",
        "command_norm",
        "phase",
        "raw_action_l2",
        "raw_action_max",
        "processed_action_l2",
        "processed_action_max",
        "joint_pos_rel_l2",
        "joint_pos_rel_max",
        "joint_vel_l2",
        "joint_vel_max",
      ):
        if key in numeric_fields:
          first_unstable[key] = numeric_fields[key]

  duration = None
  if first_unstable is not None and first_timestamp is not None:
    # Log samples are emitted at 2 Hz; counting stable samples is robust enough
    # for this diagnostic and avoids timezone parsing noise.
    duration = max(0.0, stable_samples * 0.5)

  return {
    "log": str(log_path),
    "samples": samples,
    "stable_samples": stable_samples,
    "last_stable_timestamp": last_stable_timestamp,
    "first_unstable": first_unstable,
    "stable_duration_before_first_unstable_s": duration,
  }


def _external_root_from_deploy_root(deploy_root: Path) -> Path:
  # deploy_root is normally <unitree_rl_mjlab>/deploy/robots/g1.
  for parent in deploy_root.resolve().parents:
    if (parent / "src").is_dir() and (parent / "deploy").is_dir():
      return parent
  return deploy_root.resolve().parents[2]


def _resolve_scene_path(deploy_root: Path, robot_scene: Any) -> Path | None:
  if not isinstance(robot_scene, str) or not robot_scene:
    return None
  scene_path = Path(robot_scene)
  if scene_path.is_absolute():
    return scene_path
  return _external_root_from_deploy_root(deploy_root) / scene_path


def _geom_lower_z(model: Any, data: Any, geom_id: int) -> float:
  import mujoco

  geom_type = model.geom_type[geom_id]
  if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
    half_height = model.geom_size[geom_id][2]
  else:
    # Sphere/capsule foot collision geoms use size[0] as the radius.
    half_height = model.geom_size[geom_id][0]
  return float(data.geom_xpos[geom_id, 2] - half_height)


def _foot_contact_report(scene_path: Path, initial_qpos: list[float]) -> dict[str, Any]:
  try:
    import mujoco
  except Exception as exc:  # pragma: no cover - depends on local runtime install.
    return {"available": False, "reason": f"mujoco_import_failed: {exc}"}

  if not scene_path.is_file():
    return {"available": False, "reason": f"scene_missing: {scene_path}"}

  model = mujoco.MjModel.from_xml_path(str(scene_path))
  if len(initial_qpos) != model.nq:
    return {
      "available": False,
      "reason": "qpos_dim_mismatch",
      "scene": str(scene_path),
      "model_nq": model.nq,
      "initial_qpos": len(initial_qpos),
    }

  data = mujoco.MjData(model)
  data.qpos[:] = initial_qpos
  data.qvel[:] = 0.0
  mujoco.mj_forward(model, data)

  rows: list[dict[str, Any]] = []
  for geom_id in range(model.ngeom):
    geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
    if "foot" not in geom_name:
      continue
    rows.append(
      {
        "name": geom_name,
        "center_z": round(float(data.geom_xpos[geom_id, 2]), 6),
        "lower_z": round(_geom_lower_z(model, data, geom_id), 6),
      }
    )

  if not rows:
    return {"available": False, "reason": "no_foot_geoms", "scene": str(scene_path)}

  rows = sorted(rows, key=lambda row: row["lower_z"])
  min_lower_z = float(rows[0]["lower_z"])
  return {
    "available": True,
    "scene": str(scene_path),
    "root_z": round(float(initial_qpos[2]), 6),
    "min_foot_surface_z": round(min_lower_z, 6),
    "floor_clearance_passed": min_lower_z >= -0.005,
    "required_root_lift_to_clear_floor": round(max(0.0, -min_lower_z), 6),
    "lowest_foot_geoms": rows[:8],
  }


def _metadata_list(value: str | None) -> list[str]:
  if not value:
    return []
  return [item.strip() for item in value.split(",") if item.strip()]


def _onnx_report(policy_onnx: Path) -> dict[str, Any]:
  if not policy_onnx.is_file():
    return {"available": False, "reason": f"policy_missing: {policy_onnx}"}
  try:
    import onnx
  except Exception as exc:  # pragma: no cover - depends on local runtime install.
    return {"available": False, "reason": f"onnx_import_failed: {exc}"}

  try:
    model = onnx.load(policy_onnx)
  except Exception as exc:
    return {"available": False, "reason": f"onnx_load_failed: {exc}"}

  def dims(value: Any) -> list[int | str]:
    return [
      dim.dim_value if dim.dim_value else dim.dim_param
      for dim in value.type.tensor_type.shape.dim
    ]

  metadata = {prop.key: prop.value for prop in model.metadata_props}
  return {
    "available": True,
    "inputs": [{"name": item.name, "dims": dims(item)} for item in model.graph.input],
    "outputs": [{"name": item.name, "dims": dims(item)} for item in model.graph.output],
    "metadata": {
      "run_path": metadata.get("run_path"),
      "observation_names": _metadata_list(metadata.get("observation_names")),
      "command_names": _metadata_list(metadata.get("command_names")),
      "joint_names_count": len(_metadata_list(metadata.get("joint_names"))),
    },
  }


def _term_dim(term_name: str, term: Any, *, joint_count: int, action_count: int) -> int:
  base_dims = {
    "base_ang_vel": 3,
    "projected_gravity": 3,
    "velocity_commands": 3,
    "keyboard_velocity_commands": 3,
    "gait_phase": 2,
    "joint_pos": joint_count,
    "joint_pos_rel": joint_count,
    "joint_vel_rel": joint_count,
    "last_action": action_count,
  }
  dim = base_dims.get(term_name)
  if dim is None:
    return 0
  if isinstance(term, dict):
    joint_ids = (
      term.get("params", {}).get("asset_cfg", {}).get("joint_ids")
      if isinstance(term.get("params"), dict)
      else None
    )
    if term_name in {"joint_pos", "joint_pos_rel", "joint_vel_rel"} and isinstance(
      joint_ids, list
    ):
      dim = len(joint_ids)
    history_length = int(term.get("history_length") or 1)
    return dim * history_length
  return dim


def _deploy_observation_report(
  policy: dict[str, Any], *, joint_count: int, action_count: int, onnx: dict[str, Any]
) -> dict[str, Any]:
  observations = policy.get("observations", {})
  if not isinstance(observations, dict):
    return {"available": False, "reason": "observations_missing"}

  terms: list[dict[str, Any]] = []
  total_dim = 0
  for term_name, term in observations.items():
    dim = _term_dim(term_name, term, joint_count=joint_count, action_count=action_count)
    total_dim += dim
    terms.append({"name": term_name, "dim": dim})

  onnx_input_dim = None
  if onnx.get("available") and onnx.get("inputs"):
    dims = onnx["inputs"][0].get("dims", [])
    if dims:
      onnx_input_dim = dims[-1]

  return {
    "available": True,
    "terms": terms,
    "total_dim": total_dim,
    "onnx_input_dim": onnx_input_dim,
    "input_dim_passed": onnx_input_dim in (None, total_dim),
  }


def _current_g1_source_report(initial_qpos: list[float]) -> dict[str, Any]:
  try:
    from mjlab.asset_zoo.robots.unitree_g1.g1_constants import (
      HOME_KEYFRAME,
      KNEES_BENT_KEYFRAME,
      get_g1_robot_cfg,
    )
  except Exception as exc:  # pragma: no cover - import can fail outside repo env.
    return {"available": False, "reason": f"current_g1_import_failed: {exc}"}

  current_init = get_g1_robot_cfg().init_state
  current_root_z = float(current_init.pos[2])
  home_root_z = float(HOME_KEYFRAME.pos[2])
  knees_root_z = float(KNEES_BENT_KEYFRAME.pos[2])
  if abs(current_root_z - knees_root_z) < 1e-6:
    current_init_name = "KNEES_BENT_KEYFRAME"
  elif abs(current_root_z - home_root_z) < 1e-6:
    current_init_name = "HOME_KEYFRAME"
  else:
    current_init_name = "unknown"

  return {
    "available": True,
    "current_init_name": current_init_name,
    "current_init_root_z": round(current_root_z, 6),
    "home_root_z": round(home_root_z, 6),
    "knees_bent_root_z": round(knees_root_z, 6),
    "selected_initial_root_z": round(float(initial_qpos[2]), 6),
    "selected_minus_current_init_root_z": round(
      float(initial_qpos[2]) - current_root_z, 6
    ),
    "selected_minus_home_root_z": round(float(initial_qpos[2]) - home_root_z, 6),
  }


def _current_g1_deploy_delta_report(
  *,
  initial_qpos: list[float],
  default_joint_pos: list[float],
  action_offset: list[float],
  action_scale: list[float],
  stiffness: list[float],
  damping: list[float],
) -> dict[str, Any]:
  try:
    from mjlab.actuator import BuiltinPositionActuatorCfg
    from mjlab.asset_zoo.robots.unitree_g1.g1_constants import (
      G1_ACTION_SCALE,
      G1_ARTICULATION,
      G1_XML,
      get_g1_robot_cfg,
    )
  except Exception as exc:  # pragma: no cover - import can fail outside repo env.
    return {"available": False, "reason": f"current_g1_import_failed: {exc}"}

  joint_names = _xml_joint_names(G1_XML)
  current_init = get_g1_robot_cfg().init_state
  current_joint_pos = _resolve_pattern_values(
    joint_names,
    {str(pattern): float(value) for pattern, value in current_init.joint_pos.items()},
  )
  source_action_scale = _resolve_pattern_values(joint_names, G1_ACTION_SCALE)
  source_stiffness = [0.0] * len(joint_names)
  source_damping = [0.0] * len(joint_names)
  for actuator in G1_ARTICULATION.actuators:
    if not isinstance(actuator, BuiltinPositionActuatorCfg):
      continue
    for pattern in actuator.target_names_expr:
      compiled = re.compile(pattern)
      for index, joint_name in enumerate(joint_names):
        if compiled.fullmatch(joint_name):
          source_stiffness[index] = float(actuator.stiffness)
          source_damping[index] = float(actuator.damping)

  return {
    "available": True,
    "joint_names": joint_names,
    "selected_initial_vs_current_source_init": _vector_delta(
      initial_qpos[7:],
      current_joint_pos,
      name="selected_initial_vs_current_source_init",
    ),
    "current_source_init_vs_deploy_default": _vector_delta(
      current_joint_pos,
      default_joint_pos,
      name="current_source_init_vs_deploy_default",
    ),
    "current_source_init_vs_deploy_action_offset": _vector_delta(
      current_joint_pos,
      action_offset,
      name="current_source_init_vs_deploy_action_offset",
    ),
    "current_source_action_scale_vs_deploy": _vector_delta(
      source_action_scale,
      action_scale,
      name="current_source_action_scale_vs_deploy",
    ),
    "current_source_stiffness_vs_deploy": _vector_delta(
      source_stiffness,
      stiffness,
      name="current_source_stiffness_vs_deploy",
    ),
    "current_source_damping_vs_deploy": _vector_delta(
      source_damping,
      damping,
      name="current_source_damping_vs_deploy",
    ),
  }


def _policy_provenance_report(
  *,
  onnx: dict[str, Any],
  evidence_dir: Path,
  deploy_root: Path,
) -> dict[str, Any]:
  metadata = onnx.get("metadata") if isinstance(onnx.get("metadata"), dict) else {}
  run_path = metadata.get("run_path")
  report: dict[str, Any] = {
    "available": bool(run_path),
    "run_path": run_path,
    "matched_run_dirs": [],
  }
  if not isinstance(run_path, str) or not run_path:
    report["reason"] = "onnx_run_path_missing"
    return report

  evidence_parents = evidence_dir.resolve().parents
  evidence_phase_root = evidence_parents[2] if len(evidence_parents) > 2 else Path.cwd()
  candidate_roots = [
    evidence_phase_root / "logs/rsl_rl",
    Path.cwd() / "logs/rsl_rl",
    Path("/home/ssy/ssy_files/mjlab/logs/rsl_rl"),
    _external_root_from_deploy_root(deploy_root) / "logs/rsl_rl",
  ]
  seen: set[Path] = set()
  matches: list[str] = []
  for root in candidate_roots:
    root = root.resolve()
    if root in seen or not root.is_dir():
      continue
    seen.add(root)
    for candidate in root.glob(f"*/{run_path}"):
      if candidate.is_dir():
        matches.append(str(candidate))
    for candidate in root.glob(f"*/*{run_path}*"):
      if candidate.is_dir() and str(candidate) not in matches:
        matches.append(str(candidate))

  report["matched_run_dirs"] = matches[:20]
  report["matched_run_count"] = len(matches)
  report["source_run_found"] = bool(matches)
  if not matches:
    report["reason"] = "onnx_run_path_not_found_in_local_rsl_rl_logs"
  return report


def analyze_velocity_contract(
  evidence_dir: Path,
  *,
  deploy_root: Path,
  max_pose_gap_l2: float = 0.5,
  max_pose_gap_max: float = 0.35,
) -> dict[str, Any]:
  selected = evidence_dir / "selected"
  sim_config = _load_yaml(selected / "simulate_config.yaml")
  fsm_config = _load_yaml(selected / "config.yaml")
  initial_qpos = _as_float_list(sim_config.get("initial_qpos"), name="initial_qpos")
  if len(initial_qpos) < 8:
    raise VelocityContractError("initial_qpos must include root qpos and joints")
  initial_joint_pos = initial_qpos[7:]

  velocity_cfg = fsm_config.get("FSM", {}).get("Velocity", {})
  if not isinstance(velocity_cfg, dict):
    raise VelocityContractError("FSM.Velocity must be a mapping")
  policy_dir_raw = velocity_cfg.get("policy_dir")
  if not isinstance(policy_dir_raw, str):
    raise VelocityContractError("FSM.Velocity.policy_dir must be a string")
  policy_dir = _resolve_policy_dir(deploy_root, policy_dir_raw)
  deploy_yaml = policy_dir / "params/deploy.yaml"
  policy_onnx = policy_dir / "exported/policy.onnx"
  policy = _load_yaml(deploy_yaml)

  default_joint_pos = _as_float_list(
    policy.get("default_joint_pos"), name="default_joint_pos"
  )
  action_cfg = policy.get("actions", {}).get("JointPositionAction", {})
  if not isinstance(action_cfg, dict):
    raise VelocityContractError("actions.JointPositionAction must be a mapping")
  action_offset = _as_float_list(action_cfg.get("offset"), name="action offset")
  action_scale = _as_float_list(action_cfg.get("scale"), name="action scale")
  stiffness = _as_float_list(policy.get("stiffness"), name="stiffness")
  damping = _as_float_list(policy.get("damping"), name="damping")
  joint_ids_map = policy.get("joint_ids_map", [])
  onnx = _onnx_report(policy_onnx)
  observation_report = _deploy_observation_report(
    policy,
    joint_count=len(default_joint_pos),
    action_count=len(action_scale),
    onnx=onnx,
  )
  contact_report = _foot_contact_report(
    _resolve_scene_path(deploy_root, sim_config.get("robot_scene")) or Path(),
    initial_qpos,
  )
  source_report = _current_g1_source_report(initial_qpos)
  source_delta_report = _current_g1_deploy_delta_report(
    initial_qpos=initial_qpos,
    default_joint_pos=default_joint_pos,
    action_offset=action_offset,
    action_scale=action_scale,
    stiffness=stiffness,
    damping=damping,
  )
  provenance_report = _policy_provenance_report(
    onnx=onnx,
    evidence_dir=evidence_dir,
    deploy_root=deploy_root,
  )

  default_report = _vector_report(
    initial_joint_pos,
    default_joint_pos,
    name="default_joint_pos",
    max_l2=max_pose_gap_l2,
    max_abs=max_pose_gap_max,
  )
  offset_report = _vector_report(
    initial_joint_pos,
    action_offset,
    name="action offset",
    max_l2=max_pose_gap_l2,
    max_abs=max_pose_gap_max,
  )
  counts = {
    "initial_joint_pos": len(initial_joint_pos),
    "default_joint_pos": len(default_joint_pos),
    "action_offset": len(action_offset),
    "action_scale": len(action_scale),
    "joint_ids_map": len(joint_ids_map) if isinstance(joint_ids_map, list) else None,
    "stiffness": len(stiffness),
    "damping": len(damping),
  }
  count_passed = all(value == 29 for value in counts.values() if value is not None)
  pose_passed = default_report["passed"] and offset_report["passed"]
  observation_passed = observation_report.get("input_dim_passed", True)
  contact_passed = contact_report.get("floor_clearance_passed", True)
  stability = _parse_velocity_stability(evidence_dir / "g1_ctrl.log")
  runtime_stable = stability["first_unstable"] is None and stability["samples"] > 0

  if not count_passed:
    primary_reason = "velocity_dimension_mismatch"
  elif not observation_passed:
    primary_reason = "velocity_observation_onnx_mismatch"
  elif not pose_passed:
    primary_reason = "velocity_default_pose_mismatch"
  elif not contact_passed:
    primary_reason = "velocity_initial_contact_mismatch"
  elif not runtime_stable:
    primary_reason = "velocity_runtime_instability"
  else:
    primary_reason = None

  return {
    "schema_version": 1,
    "evidence_dir": str(evidence_dir.resolve()),
    "passed": primary_reason is None,
    "primary_reason": primary_reason,
    "policy_dir_config": policy_dir_raw,
    "policy_dir_resolved": str(policy_dir.resolve()),
    "deploy_yaml": str(deploy_yaml.resolve()),
    "policy_onnx": {
      "path": str(policy_onnx.resolve()),
      "exists": policy_onnx.is_file(),
      "size_bytes": policy_onnx.stat().st_size if policy_onnx.is_file() else None,
    },
    "sim_config": {
      "enable_elastic_band": sim_config.get("enable_elastic_band"),
      "start_paused": sim_config.get("start_paused"),
      "robot": sim_config.get("robot"),
      "robot_scene": sim_config.get("robot_scene"),
      "interface": sim_config.get("interface"),
    },
    "counts": counts,
    "count_passed": count_passed,
    "initial_vs_default_joint_pos": default_report,
    "initial_vs_action_offset": offset_report,
    "onnx": onnx,
    "deploy_observations": observation_report,
    "initial_contact": contact_report,
    "current_g1_source": source_report,
    "current_g1_source_deploy_deltas": source_delta_report,
    "policy_provenance": provenance_report,
    "velocity_stability": stability,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Diagnose a G1 phase-1 Velocity bootstrap evidence directory."
  )
  parser.add_argument("--evidence-dir", required=True, type=Path)
  parser.add_argument(
    "--deploy-root",
    default=Path(
      "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1"
    ),
    type=Path,
  )
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--max-pose-gap-l2", default=0.5, type=float)
  parser.add_argument("--max-pose-gap-max", default=0.35, type=float)
  parser.add_argument("--expect-failure", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    report = analyze_velocity_contract(
      args.evidence_dir,
      deploy_root=args.deploy_root,
      max_pose_gap_l2=args.max_pose_gap_l2,
      max_pose_gap_max=args.max_pose_gap_max,
    )
  except VelocityContractError as exc:
    print(str(exc), file=sys.stderr)
    return 2

  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_failure:
    return 0 if not report["passed"] else 1
  return 0 if report["passed"] else 1


if __name__ == "__main__":
  raise SystemExit(main())
