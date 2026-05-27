"""Contracts for running G1 mimic deploy bundles in Isaac Lab."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[4]

ActionName = Literal[
  "dance1_subject2", "flying_kick", "getup", "roundhouse_leading_right"
]

G1_DEPLOY_JOINT_NAMES = (
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
)

ACTION_BUNDLES = {
  "dance1_subject2": {
    "policy_subdir": "dance1_subject2",
    "policy_filename": "policy.onnx",
    "deploy_yaml_filename": "deploy.yaml",
    "motion_filename": "dance1_subject2.npz",
  },
  "flying_kick": {
    "policy_subdir": "flying_kick",
    "policy_filename": "policy.onnx",
    "deploy_yaml_filename": "deploy.yaml",
    "motion_filename": "flying_kick.npz",
  },
  "getup": {
    "policy_subdir": "getup",
    "policy_filename": "policy.onnx",
    "deploy_yaml_filename": "deploy.yaml",
    "motion_filename": "getup.npz",
  },
  "roundhouse_leading_right": {
    "policy_subdir": "roundhouse_leading_right",
    "policy_filename": "policy.onnx",
    "deploy_yaml_filename": "deploy.yaml",
    "motion_filename": "roundhouse_leading_right.npz",
  },
}


class IsaacLabSim2SimError(RuntimeError):
  """Raised when an Isaac Lab sim2sim deployment request is invalid."""


@dataclass(frozen=True)
class G1IsaacLabDeployment:
  """Resolved inputs for one G1 kick Isaac Lab sim2sim run."""

  action: ActionName
  mjlab_root: Path
  external_root: Path
  policy_root: Path
  deploy_yaml: Path
  motion_file: Path
  policy_onnx: Path
  robot_urdf: Path


def _default_policy_root(external_root: Path, action: ActionName) -> Path:
  bundle = ACTION_BUNDLES[action]
  return (
    external_root
    / "unitree_rl_mjlab"
    / "deploy"
    / "robots"
    / "g1"
    / "config"
    / "policy"
    / "mimic"
    / bundle["policy_subdir"]
  )


def resolve_g1_deployment(
  *,
  action: ActionName,
  mjlab_root: Path | None = None,
  external_root: Path | None = None,
  policy_root: Path | None = None,
  deploy_yaml: Path | None = None,
  motion_file: Path | None = None,
  policy_onnx: Path | None = None,
  robot_urdf: Path | None = None,
) -> G1IsaacLabDeployment:
  """Resolve a G1 kick deploy bundle without importing Isaac Lab."""
  root = (mjlab_root or REPO_ROOT).expanduser().resolve()
  resolved_external_root = (
    external_root.expanduser().resolve()
    if external_root is not None
    else root / ".external"
  )
  bundle = ACTION_BUNDLES[action]
  resolved_policy_root = (
    policy_root.expanduser().resolve()
    if policy_root is not None
    else _default_policy_root(resolved_external_root, action).resolve()
  )
  deployment = G1IsaacLabDeployment(
    action=action,
    mjlab_root=root,
    external_root=resolved_external_root,
    policy_root=resolved_policy_root,
    deploy_yaml=(
      deploy_yaml.expanduser().resolve()
      if deploy_yaml is not None
      else resolved_policy_root / "params" / bundle["deploy_yaml_filename"]
    ),
    motion_file=(
      motion_file.expanduser().resolve()
      if motion_file is not None
      else resolved_policy_root / "params" / bundle["motion_filename"]
    ),
    policy_onnx=(
      policy_onnx.expanduser().resolve()
      if policy_onnx is not None
      else resolved_policy_root / "exported" / bundle["policy_filename"]
    ),
    robot_urdf=(
      robot_urdf.expanduser().resolve()
      if robot_urdf is not None
      else root
      / "src"
      / "mjlab"
      / "asset_zoo"
      / "robots"
      / "unitree_g1"
      / "urdf"
      / "g1_29dof_mode_15.urdf"
    ),
  )
  validate_deploy_bundle(deployment)
  return deployment


def validate_deploy_bundle(deployment: G1IsaacLabDeployment) -> None:
  """Validate files and dimensions used by the Isaac Lab runner."""
  required = {
    "policy_root": deployment.policy_root,
    "deploy_yaml": deployment.deploy_yaml,
    "motion_file": deployment.motion_file,
    "policy_onnx": deployment.policy_onnx,
    "robot_urdf": deployment.robot_urdf,
  }
  missing = [f"{name}: {path}" for name, path in required.items() if not path.exists()]
  if missing:
    raise IsaacLabSim2SimError(
      "missing Isaac Lab deployment inputs: " + ", ".join(missing)
    )

  deploy = load_deploy_yaml(deployment.deploy_yaml)
  action_cfg = deploy.get("actions", {}).get("JointPositionAction", {})
  scale = action_cfg.get("scale")
  offset = action_cfg.get("offset")
  default_joint_pos = deploy.get("default_joint_pos")
  for name, value in {
    "actions.JointPositionAction.scale": scale,
    "actions.JointPositionAction.offset": offset,
    "default_joint_pos": default_joint_pos,
  }.items():
    if not isinstance(value, list) or len(value) != len(G1_DEPLOY_JOINT_NAMES):
      raise IsaacLabSim2SimError(
        f"{name} must contain {len(G1_DEPLOY_JOINT_NAMES)} values"
      )

  motion = np.load(deployment.motion_file)
  for key in ("fps", "joint_pos", "joint_vel"):
    if key not in motion.files:
      raise IsaacLabSim2SimError(f"motion file missing {key}: {deployment.motion_file}")
  joint_pos = motion["joint_pos"]
  joint_vel = motion["joint_vel"]
  if joint_pos.ndim != 2 or joint_pos.shape[1] != len(G1_DEPLOY_JOINT_NAMES):
    raise IsaacLabSim2SimError(
      f"motion joint_pos must be [T,{len(G1_DEPLOY_JOINT_NAMES)}], got {joint_pos.shape}"
    )
  if joint_vel.shape != joint_pos.shape:
    raise IsaacLabSim2SimError(
      f"motion joint_vel shape {joint_vel.shape} does not match joint_pos {joint_pos.shape}"
    )
  if joint_pos.shape[0] < 2:
    raise IsaacLabSim2SimError("motion must contain at least two frames")
  if not np.isfinite(joint_pos).all() or not np.isfinite(joint_vel).all():
    raise IsaacLabSim2SimError("motion contains non-finite joint data")


def load_deploy_yaml(path: Path) -> dict[str, Any]:
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise IsaacLabSim2SimError(f"deploy.yaml root must be a mapping: {path}")
  return data


def file_record(path: Path) -> dict[str, Any]:
  digest = hashlib.sha256()
  with path.open("rb") as handle:
    for chunk in iter(lambda: handle.read(1024 * 1024), b""):
      digest.update(chunk)
  return {
    "path": str(path),
    "size_bytes": path.stat().st_size,
    "sha256": digest.hexdigest(),
  }


def deployment_report(deployment: G1IsaacLabDeployment) -> dict[str, Any]:
  """Return static provenance for a resolved Isaac Lab sim2sim deployment."""
  motion = np.load(deployment.motion_file)
  deploy = load_deploy_yaml(deployment.deploy_yaml)
  return {
    "schema_version": 1,
    "simulator": "isaaclab",
    "robot": "unitree_g1_29dof_mode_15",
    "action": deployment.action,
    "joint_names": list(G1_DEPLOY_JOINT_NAMES),
    "step_dt": float(deploy.get("step_dt", 0.02)),
    "motion": {
      "frames": int(motion["joint_pos"].shape[0]),
      "fps": float(np.asarray(motion["fps"]).reshape(-1)[0]),
      "joint_dim": int(motion["joint_pos"].shape[1]),
      "file": file_record(deployment.motion_file),
    },
    "deploy_yaml": file_record(deployment.deploy_yaml),
    "policy_onnx": file_record(deployment.policy_onnx),
    "robot_urdf": file_record(deployment.robot_urdf),
    "external_root": str(deployment.external_root),
    "policy_root": str(deployment.policy_root),
    "mjlab_root": str(deployment.mjlab_root),
  }


def write_json(path: Path, payload: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
  )
