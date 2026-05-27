#!/usr/bin/env python3
"""Run a G1 kick deploy bundle inside Isaac Lab.

This script is intentionally launched by Isaac Lab's python entrypoint:

  isaaclab.sh -p scripts/isaaclab/g1_kick_sim2sim.py --headless ...
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import os
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, cast

from isaaclab.app import AppLauncher


def _load_contracts():
  script_path = Path(__file__).resolve()
  root = (
    Path(os.environ.get("MJLAB_ROOT", script_path.parents[2])).expanduser().resolve()
  )
  module_path = root / "src" / "mjlab" / "sim2sim" / "isaaclab" / "g1_kick.py"
  spec = importlib.util.spec_from_file_location(
    "mjlab_isaaclab_g1_kick_contracts", module_path
  )
  if spec is None or spec.loader is None:
    raise RuntimeError(f"failed to load Isaac Lab sim2sim contracts: {module_path}")
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  return module


CONTRACTS = _load_contracts()
ACTION_BUNDLES = CONTRACTS.ACTION_BUNDLES
G1_DEPLOY_JOINT_NAMES = CONTRACTS.G1_DEPLOY_JOINT_NAMES
deployment_report = CONTRACTS.deployment_report
load_deploy_yaml = CONTRACTS.load_deploy_yaml
resolve_g1_deployment = CONTRACTS.resolve_g1_deployment
write_json = CONTRACTS.write_json
ActionName = CONTRACTS.ActionName


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description="Replay a productized G1 kick deploy bundle in Isaac Lab."
  )
  parser.add_argument(
    "--action",
    choices=sorted(ACTION_BUNDLES),
    default="flying_kick",
    help="G1 deploy action bundle to run.",
  )
  parser.add_argument(
    "--mjlab-root",
    type=Path,
    default=Path(os.environ.get("MJLAB_ROOT", Path.cwd())),
    help="mjlab checkout that owns source assets and runtime policy roots.",
  )
  parser.add_argument(
    "--external-root",
    type=Path,
    default=(
      Path(os.environ["MJLAB_EXTERNAL_ROOT"])
      if os.environ.get("MJLAB_EXTERNAL_ROOT")
      else None
    ),
    help="External runtime asset root. Defaults to <mjlab-root>/.external.",
  )
  parser.add_argument("--policy-root", type=Path, help="Override deploy policy root.")
  parser.add_argument("--deploy-yaml", type=Path, help="Override deploy.yaml path.")
  parser.add_argument("--motion-file", type=Path, help="Override motion npz path.")
  parser.add_argument("--policy-onnx", type=Path, help="Override policy ONNX path.")
  parser.add_argument("--robot-urdf", type=Path, help="Override G1 URDF path.")
  parser.add_argument(
    "--out-dir",
    type=Path,
    help="Evidence directory. Defaults to logs/g1_isaaclab_sim2sim/<timestamp>-<action>.",
  )
  parser.add_argument(
    "--max-steps",
    type=int,
    help="Limit simulated steps. Defaults to the full motion length.",
  )
  parser.add_argument(
    "--root-height",
    type=float,
    default=0.78,
    help="Initial floating-base root height in meters.",
  )
  parser.add_argument(
    "--physics-dt",
    type=float,
    help="Physics timestep. Defaults to deploy step_dt / control_decimation.",
  )
  parser.add_argument(
    "--control-decimation",
    type=int,
    default=4,
    help="Physics substeps per policy action. Isaac Lab training configs commonly use 4.",
  )
  parser.add_argument(
    "--fall-root-height-threshold",
    type=float,
    default=0.35,
    help="Mark the run failed if the floating base drops below this height.",
  )
  parser.add_argument(
    "--fix-base",
    action="store_true",
    help="Import the URDF with a fixed base for asset/path debugging.",
  )
  parser.add_argument(
    "--training-actuator-limits",
    action="store_true",
    help="Apply mjlab training-side per-joint effort/velocity limits after import.",
  )
  parser.add_argument(
    "--control-mode",
    choices=("policy", "reference"),
    default="policy",
    help="Run the exported policy or directly replay reference joint targets.",
  )
  parser.add_argument(
    "--initial-state",
    choices=("default", "motion"),
    default="motion",
    help="Robot joint state used before each playback loop.",
  )
  parser.add_argument(
    "--settle-steps",
    type=int,
    default=0,
    help="Policy-timestep count to hold the initial/default pose before playback.",
  )
  parser.add_argument(
    "--settle-mode",
    choices=("target", "velocity"),
    default="target",
    help="Use a static target or the local zero-command velocity policy while settling.",
  )
  parser.add_argument(
    "--trace-every",
    type=int,
    default=0,
    help="Write trace.csv every N policy steps. Use 1 for detailed diagnostics.",
  )
  parser.add_argument(
    "--video-path",
    type=Path,
    help="Optional MP4 evidence path. Enables Isaac headless camera capture.",
  )
  parser.add_argument(
    "--video-every",
    type=int,
    default=1,
    help="Capture one video frame every N policy steps when --video-path is set.",
  )
  parser.add_argument(
    "--video-width",
    type=int,
    default=640,
    help="Headless camera capture width in pixels.",
  )
  parser.add_argument(
    "--video-height",
    type=int,
    default=360,
    help="Headless camera capture height in pixels.",
  )
  parser.add_argument(
    "--video-fps",
    type=float,
    help="MP4 frame rate. Defaults to 1 / (policy_dt * video_every).",
  )
  parser.add_argument(
    "--render",
    action="store_true",
    help="Render each physics step. Headless evidence normally leaves this off.",
  )
  parser.add_argument(
    "--loop-playback",
    action="store_true",
    help="Repeat the motion until the Isaac Sim window is closed.",
  )
  parser.add_argument(
    "--realtime",
    action="store_true",
    help="Throttle stepping to deploy step_dt for human-viewable playback.",
  )
  parser.add_argument(
    "--playback-speed",
    type=float,
    default=1.0,
    help="Playback speed multiplier used with --realtime.",
  )
  parser.add_argument(
    "--hold-open",
    action="store_true",
    help="Keep the Isaac Sim window open after the requested motion finishes.",
  )
  parser.add_argument(
    "--graceful-close",
    action="store_true",
    help="Call SimulationApp.close() before exiting. Default exits after evidence is flushed.",
  )
  AppLauncher.add_app_launcher_args(parser)
  return parser


args_cli = build_parser().parse_args()
if args_cli.video_path is not None:
  args_cli.enable_cameras = True
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import cv2  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
import numpy as np  # noqa: E402
import onnx  # noqa: E402
import torch  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
from isaaclab.sim import SimulationContext  # noqa: E402
from isaaclab.utils.math import (  # noqa: E402
  matrix_from_quat,
  quat_conjugate,
  quat_from_angle_axis,
  quat_mul,
  yaw_quat,
)
from onnx import numpy_helper  # noqa: E402


def _timestamp() -> str:
  return datetime.now().strftime("%Y%m%d-%H%M%S")


def _default_out_dir(mjlab_root: Path, action: str) -> Path:
  return mjlab_root / "logs" / "g1_isaaclab_sim2sim" / f"{_timestamp()}-{action}"


def _make_robot_cfg(
  *,
  prim_path: str,
  urdf_path: Path,
  fix_base: bool,
  root_height: float,
) -> ArticulationCfg:
  return ArticulationCfg(
    prim_path=prim_path,
    spawn=sim_utils.UrdfFileCfg(
      asset_path=str(urdf_path),
      fix_base=fix_base,
      self_collision=False,
      merge_fixed_joints=True,
      joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
        gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
          stiffness=None,
          damping=None,
        )
      ),
      articulation_props=sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        fix_root_link=fix_base,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=4,
      ),
      rigid_props=sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
      ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
      pos=(0.0, 0.0, root_height),
      joint_pos={".*": 0.0},
      joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
      "all": ImplicitActuatorCfg(
        joint_names_expr=[".*"],
        effort_limit_sim=300.0,
        velocity_limit_sim=100.0,
        stiffness=40.0,
        damping=2.0,
      )
    },
  )


def _design_scene(
  deployment_root: Path, robot_urdf: Path, fix_base: bool, root_height: float
) -> Articulation:
  del deployment_root
  ground = sim_utils.GroundPlaneCfg()
  ground.func("/World/defaultGroundPlane", ground)
  light = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
  light.func("/World/Light", light)
  sim_utils.create_prim("/World/Origin1", "Xform", translation=(0.0, 0.0, 0.0))
  robot_cfg = _make_robot_cfg(
    prim_path="/World/Origin1/Robot",
    urdf_path=robot_urdf,
    fix_base=fix_base,
    root_height=root_height,
  )
  return Articulation(cfg=robot_cfg)


def _make_video_camera() -> Camera:
  camera_cfg = CameraCfg(
    prim_path="/World/VideoCamera",
    update_period=0.0,
    height=max(1, int(args_cli.video_height)),
    width=max(1, int(args_cli.video_width)),
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(
      focal_length=24.0,
      focus_distance=400.0,
      horizontal_aperture=20.955,
      clipping_range=(0.1, 100.0),
    ),
  )
  return Camera(cfg=camera_cfg)


def _camera_frame(camera: Camera) -> np.ndarray:
  frame = camera.data.output["rgb"][0, ..., :3].detach().cpu().numpy()
  if frame.dtype != np.uint8:
    frame = (np.clip(frame, 0.0, 1.0) * 255.0).astype(np.uint8)
  return np.ascontiguousarray(frame)


def _write_video(path: Path, frames: list[np.ndarray], *, fps: float) -> None:
  if not frames:
    return
  height, width = frames[0].shape[:2]
  writer = cv2.VideoWriter(
    str(path),
    cv2.VideoWriter_fourcc(*"mp4v"),
    fps,
    (width, height),
  )
  if not writer.isOpened():
    raise RuntimeError(f"failed to open MP4 writer: {path}")
  try:
    for frame in frames:
      if frame.shape[:2] != (height, width):
        raise RuntimeError("video frames have inconsistent resolution")
      writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
  finally:
    writer.release()


def _capture_video_frame(
  *,
  camera: Camera,
  robot: Articulation,
  sim: SimulationContext,
  frames: list[np.ndarray],
) -> None:
  root = robot.data.root_pos_w[0].detach()
  target = root + torch.tensor((0.0, 0.0, 0.45), dtype=torch.float32, device=root.device)
  eye = root + torch.tensor((2.2, -2.2, 1.2), dtype=torch.float32, device=root.device)
  camera.set_world_poses_from_view(eye.reshape(1, 3), target.reshape(1, 3))
  camera.update(dt=sim.get_physics_dt())
  frames.append(_camera_frame(camera))


def _joint_ids(robot: Articulation) -> list[int]:
  missing = [name for name in G1_DEPLOY_JOINT_NAMES if name not in robot.joint_names]
  if missing:
    raise RuntimeError(
      "Isaac Lab G1 articulation is missing deploy joints: " + ", ".join(missing)
    )
  return [robot.joint_names.index(name) for name in G1_DEPLOY_JOINT_NAMES]


def _training_effort_velocity_limits() -> tuple[list[float], list[float]]:
  effort_limits: list[float] = []
  velocity_limits: list[float] = []
  for name in G1_DEPLOY_JOINT_NAMES:
    if "wrist_pitch" in name or "wrist_yaw" in name:
      effort_limits.append(5.0)
      velocity_limits.append(22.0)
    elif "ankle" in name or name in {"waist_pitch_joint", "waist_roll_joint"}:
      effort_limits.append(50.0)
      velocity_limits.append(37.0)
    elif "hip_roll" in name or "knee" in name:
      effort_limits.append(139.0)
      velocity_limits.append(20.0)
    elif "hip_pitch" in name or "hip_yaw" in name or name == "waist_yaw_joint":
      effort_limits.append(88.0)
      velocity_limits.append(32.0)
    else:
      effort_limits.append(25.0)
      velocity_limits.append(37.0)
  return effort_limits, velocity_limits


def _as_row(values: Any, *, device: str) -> torch.Tensor:
  return torch.tensor(values, dtype=torch.float32, device=device).reshape(1, -1)


def _term_cfg(deploy: dict[str, Any], name: str) -> dict[str, Any]:
  cfg = deploy.get("observations", {}).get(name, {})
  return cfg if isinstance(cfg, dict) else {}


def _maybe_row(values: Any, *, width: int, device: str) -> torch.Tensor | None:
  if values is None:
    return None
  if isinstance(values, int | float):
    return torch.full((1, width), float(values), dtype=torch.float32, device=device)
  if isinstance(values, list) and len(values) == width:
    return _as_row(values, device=device)
  return None


def _scale_and_clip_obs(
  values: torch.Tensor,
  *,
  deploy: dict[str, Any],
  term_name: str,
) -> torch.Tensor:
  cfg = _term_cfg(deploy, term_name)
  width = int(values.shape[-1])
  clip = cfg.get("clip")
  scale = _maybe_row(cfg.get("scale"), width=width, device=str(values.device))
  scale_first = bool(deploy.get("observations", {}).get("scale_first", False))
  if scale_first and scale is not None:
    values = values * scale
  if isinstance(clip, list) and len(clip) >= 2:
    values = torch.clamp(values, float(clip[0]), float(clip[1]))
  if not scale_first and scale is not None:
    values = values * scale
  return values


def _clip_action(target: torch.Tensor, deploy: dict[str, Any]) -> torch.Tensor:
  clip = deploy.get("actions", {}).get("JointPositionAction", {}).get("clip")
  if not isinstance(clip, list):
    return target
  if len(clip) == 2 and all(isinstance(value, int | float) for value in clip):
    return torch.clamp(target, float(clip[0]), float(clip[1]))
  if len(clip) == target.shape[-1] and all(
    isinstance(value, list) and len(value) >= 2 for value in clip
  ):
    lower = _as_row([value[0] for value in clip], device=str(target.device))
    upper = _as_row([value[1] for value in clip], device=str(target.device))
    return torch.maximum(torch.minimum(target, upper), lower)
  return target


def _joint_tracking_diagnostics(
  *,
  robot: Articulation,
  joint_ids: list[int],
  target: torch.Tensor,
  stiffness: torch.Tensor,
  damping: torch.Tensor,
) -> dict[str, float | int]:
  current = robot.data.joint_pos[:, joint_ids]
  velocity = robot.data.joint_vel[:, joint_ids]
  error = torch.abs(current - target)
  effort_limits = torch.clamp(robot.data.joint_effort_limits[:, joint_ids], min=1e-6)
  velocity_limits = torch.clamp(robot.data.joint_vel_limits[:, joint_ids], min=1e-6)
  requested_pd_effort = stiffness * (target - current) - damping * velocity
  pd_effort_ratio = torch.abs(requested_pd_effort) / effort_limits
  joint_vel_ratio = torch.abs(velocity) / velocity_limits
  support_joint_count = 15
  support_pd_effort_ratio = pd_effort_ratio[:, :support_joint_count]
  support_joint_vel_ratio = joint_vel_ratio[:, :support_joint_count]
  return {
    "joint_error_mean": float(torch.mean(error).detach().cpu()),
    "joint_error_max": float(torch.max(error).detach().cpu()),
    "pd_effort_ratio_max": float(torch.max(pd_effort_ratio).detach().cpu()),
    "pd_effort_ratio_argmax": int(torch.argmax(pd_effort_ratio).detach().cpu()),
    "joint_vel_ratio_max": float(torch.max(joint_vel_ratio).detach().cpu()),
    "joint_vel_ratio_argmax": int(torch.argmax(joint_vel_ratio).detach().cpu()),
    "support_pd_effort_ratio_max": float(
      torch.max(support_pd_effort_ratio).detach().cpu()
    ),
    "support_pd_effort_ratio_argmax": int(
      torch.argmax(support_pd_effort_ratio).detach().cpu()
    ),
    "support_joint_vel_ratio_max": float(
      torch.max(support_joint_vel_ratio).detach().cpu()
    ),
    "support_joint_vel_ratio_argmax": int(
      torch.argmax(support_joint_vel_ratio).detach().cpu()
    ),
  }


class OnnxMlpPolicy:
  """Small evaluator for the exported actor ONNX graph used by this deploy lane."""

  def __init__(self, model_path: Path, *, device: str) -> None:
    model = onnx.load(str(model_path))
    self.nodes = list(model.graph.node)
    self.input_name = model.graph.input[0].name
    self.output_name = model.graph.output[0].name
    self.initializers = {
      tensor.name: torch.tensor(
        numpy_helper.to_array(tensor), dtype=torch.float32, device=device
      )
      for tensor in model.graph.initializer
    }

  def __call__(self, obs: torch.Tensor) -> torch.Tensor:
    values = {self.input_name: obs, **self.initializers}
    for node in self.nodes:
      if node.op_type == "Sub":
        values[node.output[0]] = values[node.input[0]] - values[node.input[1]]
      elif node.op_type == "Div":
        values[node.output[0]] = values[node.input[0]] / values[node.input[1]]
      elif node.op_type == "Gemm":
        attrs = {
          attr.name: onnx.helper.get_attribute_value(attr) for attr in node.attribute
        }
        alpha = float(attrs.get("alpha", 1.0))
        beta = float(attrs.get("beta", 1.0))
        trans_a = bool(attrs.get("transA", 0))
        trans_b = bool(attrs.get("transB", 0))
        left = (
          values[node.input[0]].transpose(-2, -1) if trans_a else values[node.input[0]]
        )
        right = (
          values[node.input[1]].transpose(-2, -1) if trans_b else values[node.input[1]]
        )
        bias = values[node.input[2]]
        values[node.output[0]] = alpha * torch.matmul(left, right) + beta * bias
      elif node.op_type == "Elu":
        attrs = {
          attr.name: onnx.helper.get_attribute_value(attr) for attr in node.attribute
        }
        values[node.output[0]] = torch.nn.functional.elu(
          values[node.input[0]], alpha=float(attrs.get("alpha", 1.0))
        )
      else:
        raise RuntimeError(f"unsupported policy ONNX op: {node.op_type}")
    return values[self.output_name]


def _sleep_realtime(start_time: float, step_dt: float) -> None:
  if not args_cli.realtime:
    return
  target_dt = step_dt / max(args_cli.playback_speed, 1e-6)
  delay = target_dt - (time.perf_counter() - start_time)
  if delay > 0:
    time.sleep(delay)


def _axis_angle_quat(
  axis: tuple[float, float, float], angle: torch.Tensor
) -> torch.Tensor:
  axis_tensor = torch.tensor(axis, dtype=torch.float32, device=angle.device).reshape(
    1, 3
  )
  return quat_from_angle_axis(angle.reshape(-1), axis_tensor)


def _torso_quat(root_quat: torch.Tensor, joint_pos: torch.Tensor) -> torch.Tensor:
  waist_yaw = _axis_angle_quat((0.0, 0.0, 1.0), joint_pos[:, 12])
  waist_roll = _axis_angle_quat((1.0, 0.0, 0.0), joint_pos[:, 13])
  waist_pitch = _axis_angle_quat((0.0, 1.0, 0.0), joint_pos[:, 14])
  return quat_mul(quat_mul(quat_mul(root_quat, waist_yaw), waist_roll), waist_pitch)


def _motion_anchor_ori_b(
  *,
  init_quat: torch.Tensor,
  robot: Articulation,
  joint_ids: list[int],
  motion_root_quat: torch.Tensor,
  motion_joint_pos: torch.Tensor,
) -> torch.Tensor:
  real_quat_w = _torso_quat(robot.data.root_quat_w, robot.data.joint_pos[:, joint_ids])
  ref_quat_w = _torso_quat(motion_root_quat, motion_joint_pos)
  aligned_ref = quat_mul(init_quat, ref_quat_w)
  rot_quat = quat_mul(quat_conjugate(aligned_ref), real_quat_w)
  rot = matrix_from_quat(rot_quat).transpose(-2, -1)
  return torch.stack(
    (
      rot[:, 0, 0],
      rot[:, 0, 1],
      rot[:, 1, 0],
      rot[:, 1, 1],
      rot[:, 2, 0],
      rot[:, 2, 1],
    ),
    dim=-1,
  )


def _build_policy_obs(
  *,
  deploy: dict[str, Any],
  init_quat: torch.Tensor,
  robot: Articulation,
  joint_ids: list[int],
  default_joint_pos: torch.Tensor,
  motion_root_quat: torch.Tensor,
  motion_joint_pos: torch.Tensor,
  motion_joint_vel: torch.Tensor,
  last_action: torch.Tensor,
) -> torch.Tensor:
  joint_pos = robot.data.joint_pos[:, joint_ids]
  joint_vel = robot.data.joint_vel[:, joint_ids]
  motion_anchor = _motion_anchor_ori_b(
    init_quat=init_quat,
    robot=robot,
    joint_ids=joint_ids,
    motion_root_quat=motion_root_quat,
    motion_joint_pos=motion_joint_pos,
  )
  motion_command = torch.cat((motion_joint_pos, motion_joint_vel), dim=-1)
  base_ang_vel = robot.data.root_ang_vel_b
  joint_pos_rel = joint_pos - default_joint_pos
  return torch.cat(
    (
      _scale_and_clip_obs(motion_command, deploy=deploy, term_name="motion_command"),
      _scale_and_clip_obs(
        motion_anchor, deploy=deploy, term_name="motion_anchor_ori_b"
      ),
      _scale_and_clip_obs(base_ang_vel, deploy=deploy, term_name="base_ang_vel"),
      _scale_and_clip_obs(joint_pos_rel, deploy=deploy, term_name="joint_pos_rel"),
      _scale_and_clip_obs(joint_vel, deploy=deploy, term_name="joint_vel_rel"),
      _scale_and_clip_obs(last_action, deploy=deploy, term_name="last_action"),
    ),
    dim=-1,
  )


def _projected_gravity_b(robot: Articulation) -> torch.Tensor:
  if hasattr(robot.data, "projected_gravity_b"):
    return robot.data.projected_gravity_b
  gravity_w = torch.tensor(
    (0.0, 0.0, -1.0), dtype=torch.float32, device=robot.data.root_quat_w.device
  ).reshape(1, 3)
  rot = matrix_from_quat(quat_conjugate(robot.data.root_quat_w))
  return torch.bmm(rot, gravity_w.unsqueeze(-1)).squeeze(-1)


def _build_velocity_obs(
  *,
  deploy: dict[str, Any],
  robot: Articulation,
  joint_ids: list[int],
  default_joint_pos: torch.Tensor,
  last_action: torch.Tensor,
  global_phase: float,
) -> torch.Tensor:
  joint_pos = robot.data.joint_pos[:, joint_ids]
  joint_vel = robot.data.joint_vel[:, joint_ids]
  velocity_commands = torch.zeros((1, 3), dtype=torch.float32, device=joint_pos.device)
  gait_phase = torch.zeros((1, 2), dtype=torch.float32, device=joint_pos.device)
  if torch.linalg.vector_norm(velocity_commands) >= 0.1:
    gait_phase[:, 0] = np.sin(global_phase * 2.0 * np.pi)
    gait_phase[:, 1] = np.cos(global_phase * 2.0 * np.pi)
  joint_pos_rel = joint_pos - default_joint_pos
  return torch.cat(
    (
      _scale_and_clip_obs(
        robot.data.root_ang_vel_b, deploy=deploy, term_name="base_ang_vel"
      ),
      _scale_and_clip_obs(
        _projected_gravity_b(robot), deploy=deploy, term_name="projected_gravity"
      ),
      _scale_and_clip_obs(
        velocity_commands, deploy=deploy, term_name="velocity_commands"
      ),
      _scale_and_clip_obs(gait_phase, deploy=deploy, term_name="gait_phase"),
      _scale_and_clip_obs(joint_pos_rel, deploy=deploy, term_name="joint_pos_rel"),
      _scale_and_clip_obs(joint_vel, deploy=deploy, term_name="joint_vel_rel"),
      _scale_and_clip_obs(last_action, deploy=deploy, term_name="last_action"),
    ),
    dim=-1,
  )


def _run() -> dict[str, Any]:
  action = cast(ActionName, args_cli.action)
  deployment = resolve_g1_deployment(
    action=action,
    mjlab_root=args_cli.mjlab_root,
    external_root=args_cli.external_root,
    policy_root=args_cli.policy_root,
    deploy_yaml=args_cli.deploy_yaml,
    motion_file=args_cli.motion_file,
    policy_onnx=args_cli.policy_onnx,
    robot_urdf=args_cli.robot_urdf,
  )
  out_dir = (
    args_cli.out_dir.expanduser().resolve()
    if args_cli.out_dir is not None
    else _default_out_dir(deployment.mjlab_root, deployment.action)
  )
  print(f"ISAACLAB_SIM2SIM_OUT_DIR={out_dir}", flush=True)
  static_report = deployment_report(deployment)
  write_json(out_dir / "deployment_inputs.json", static_report)

  deploy = load_deploy_yaml(deployment.deploy_yaml)
  velocity_policy_root = (
    deployment.external_root
    / "unitree_rl_mjlab"
    / "deploy"
    / "robots"
    / "g1"
    / "config"
    / "policy"
    / "velocity"
    / "v0"
  )
  velocity_deploy = (
    load_deploy_yaml(velocity_policy_root / "params" / "deploy.yaml")
    if args_cli.settle_mode == "velocity" and args_cli.settle_steps > 0
    else None
  )
  motion = np.load(deployment.motion_file)
  motion_joint_pos = torch.tensor(
    motion["joint_pos"],
    dtype=torch.float32,
    device=args_cli.device,
  )
  motion_joint_vel = torch.tensor(
    motion["joint_vel"],
    dtype=torch.float32,
    device=args_cli.device,
  )
  motion_root_quat = torch.tensor(
    motion["body_quat_w"][:, 0, :],
    dtype=torch.float32,
    device=args_cli.device,
  )
  motion_root_pos = torch.tensor(
    motion["body_pos_w"][:, 0, :],
    dtype=torch.float32,
    device=args_cli.device,
  )
  motion_root_lin_vel = (
    torch.tensor(
      motion["body_lin_vel_w"][:, 0, :],
      dtype=torch.float32,
      device=args_cli.device,
    )
    if "body_lin_vel_w" in motion.files
    else torch.zeros(
      (int(motion_joint_pos.shape[0]), 3), dtype=torch.float32, device=args_cli.device
    )
  )
  motion_root_ang_vel = (
    torch.tensor(
      motion["body_ang_vel_w"][:, 0, :],
      dtype=torch.float32,
      device=args_cli.device,
    )
    if "body_ang_vel_w" in motion.files
    else torch.zeros(
      (int(motion_joint_pos.shape[0]), 3), dtype=torch.float32, device=args_cli.device
    )
  )
  motion_root_height = np.asarray(motion["body_pos_w"][:, 0, 2], dtype=np.float32)
  frame_count = int(motion_joint_pos.shape[0])
  step_count = (
    frame_count if args_cli.max_steps is None else min(args_cli.max_steps, frame_count)
  )

  policy_dt = float(deploy.get("step_dt", 0.02))
  control_decimation = max(1, int(args_cli.control_decimation))
  physics_dt = (
    float(args_cli.physics_dt)
    if args_cli.physics_dt is not None
    else policy_dt / control_decimation
  )
  if physics_dt <= 0.0:
    raise ValueError("--physics-dt must be positive")
  if args_cli.physics_dt is not None:
    control_decimation = max(1, int(round(policy_dt / physics_dt)))
  sim_cfg = sim_utils.SimulationCfg(
    dt=physics_dt,
    device=args_cli.device,
  )
  sim = SimulationContext(sim_cfg)
  sim.set_camera_view([2.2, -2.2, 1.5], [0.0, 0.0, 0.7])
  robot = _design_scene(
    deployment.mjlab_root,
    deployment.robot_urdf,
    args_cli.fix_base,
    args_cli.root_height,
  )
  video_camera = _make_video_camera() if args_cli.video_path is not None else None
  sim.reset()

  joint_ids = _joint_ids(robot)
  default_joint_pos = _as_row(deploy["default_joint_pos"], device=sim.device)
  action_scale = _as_row(
    deploy["actions"]["JointPositionAction"]["scale"], device=sim.device
  )
  action_offset = _as_row(
    deploy["actions"]["JointPositionAction"]["offset"], device=sim.device
  )
  stiffness = _as_row(deploy["stiffness"], device=sim.device)
  damping = _as_row(deploy["damping"], device=sim.device)
  robot.write_joint_stiffness_to_sim(stiffness, joint_ids=joint_ids)
  robot.write_joint_damping_to_sim(damping, joint_ids=joint_ids)
  actuator_limits_mode = "import_default"
  if args_cli.training_actuator_limits:
    effort_limits, velocity_limits = _training_effort_velocity_limits()
    robot.write_joint_effort_limit_to_sim(
      _as_row(effort_limits, device=sim.device), joint_ids=joint_ids
    )
    robot.write_joint_velocity_limit_to_sim(
      _as_row(velocity_limits, device=sim.device), joint_ids=joint_ids
    )
    actuator_limits_mode = "mjlab_training"
  policy = (
    OnnxMlpPolicy(deployment.policy_onnx, device=sim.device)
    if args_cli.control_mode == "policy"
    else None
  )
  velocity_policy = (
    OnnxMlpPolicy(velocity_policy_root / "exported" / "policy.onnx", device=sim.device)
    if velocity_deploy is not None
    else None
  )
  velocity_default_joint_pos = (
    _as_row(velocity_deploy["default_joint_pos"], device=sim.device)
    if velocity_deploy is not None
    else None
  )
  velocity_action_scale = (
    _as_row(
      velocity_deploy["actions"]["JointPositionAction"]["scale"], device=sim.device
    )
    if velocity_deploy is not None
    else None
  )
  velocity_action_offset = (
    _as_row(
      velocity_deploy["actions"]["JointPositionAction"]["offset"], device=sim.device
    )
    if velocity_deploy is not None
    else None
  )

  def reset_motion_start() -> None:
    root_state = robot.data.default_root_state.clone()
    root_state[:, :2] = 0.0
    root_state[:, 2] = args_cli.root_height
    robot.write_root_state_to_sim(root_state)
    joint_pos = robot.data.default_joint_pos.clone()
    joint_vel = robot.data.default_joint_vel.clone()
    if args_cli.initial_state == "motion":
      root_state[:, 2] = motion_root_pos[0, 2]
      root_state[:, 3:7] = motion_root_quat[0].reshape(1, -1)
      root_state[:, 7:10] = motion_root_lin_vel[0].reshape(1, -1)
      root_state[:, 10:13] = motion_root_ang_vel[0].reshape(1, -1)
      robot.write_root_state_to_sim(root_state)
      joint_pos[:, joint_ids] = motion_joint_pos[0].reshape(1, -1)
      joint_vel[:, joint_ids] = motion_joint_vel[0].reshape(1, -1)
    else:
      joint_pos[:, joint_ids] = default_joint_pos
      joint_vel[:, joint_ids] = 0.0
    robot.write_joint_state_to_sim(joint_pos, joint_vel)
    robot.reset()

  errors: list[float] = []
  error_maxes: list[float] = []
  root_heights: list[float] = []
  action_norms: list[float] = []
  pd_effort_ratios: list[float] = []
  joint_vel_ratios: list[float] = []
  support_pd_effort_ratios: list[float] = []
  support_joint_vel_ratios: list[float] = []
  trace_rows: list[dict[str, Any]] = []
  video_frames: list[np.ndarray] = []
  video_path = (
    args_cli.video_path.expanduser().resolve()
    if args_cli.video_path is not None
    else None
  )
  video_every = max(1, int(args_cli.video_every))
  video_fps = (
    float(args_cli.video_fps)
    if args_cli.video_fps is not None
    else 1.0 / (policy_dt * video_every)
  )
  max_pd_effort_ratio_observation: dict[str, Any] | None = None
  max_joint_vel_ratio_observation: dict[str, Any] | None = None
  max_support_pd_effort_ratio_observation: dict[str, Any] | None = None
  max_support_joint_vel_ratio_observation: dict[str, Any] | None = None
  nonfinite_step: int | None = None
  first_fall_step: int | None = None
  first_fall_substep: int | None = None
  completed_steps = 0
  render = (
    args_cli.render
    or video_camera is not None
    or not bool(getattr(args_cli, "headless", True))
  )
  while True:
    reset_motion_start()
    if video_camera is not None and not video_frames:
      _capture_video_frame(
        camera=video_camera,
        robot=robot,
        sim=sim,
        frames=video_frames,
      )
    settle_target = (
      motion_joint_pos[0].reshape(1, -1)
      if args_cli.initial_state == "motion"
      else default_joint_pos
    )
    settle_last_action = torch.zeros(
      (1, len(joint_ids)), dtype=torch.float32, device=sim.device
    )
    settle_global_phase = 0.0
    for settle_step in range(max(0, int(args_cli.settle_steps))):
      if args_cli.settle_mode == "velocity":
        if (
          velocity_policy is None
          or velocity_deploy is None
          or velocity_default_joint_pos is None
          or velocity_action_scale is None
          or velocity_action_offset is None
        ):
          raise RuntimeError("velocity settle requested without velocity policy inputs")
        period = float(
          velocity_deploy.get("observations", {})
          .get("gait_phase", {})
          .get("params", {})
          .get("period", 0.6)
        )
        settle_global_phase = (settle_global_phase + policy_dt / period) % 1.0
        settle_obs = _build_velocity_obs(
          deploy=velocity_deploy,
          robot=robot,
          joint_ids=joint_ids,
          default_joint_pos=velocity_default_joint_pos,
          last_action=settle_last_action,
          global_phase=settle_global_phase,
        )
        settle_raw_action = velocity_policy(settle_obs)
        settle_target = _clip_action(
          settle_raw_action * velocity_action_scale + velocity_action_offset,
          velocity_deploy,
        )
        settle_last_action = settle_raw_action.detach()
      for substep in range(control_decimation):
        robot.set_joint_position_target(settle_target, joint_ids=joint_ids)
        robot.write_data_to_sim()
        sim.step(render=render)
        robot.update(sim.get_physics_dt())
        if args_cli.trace_every > 0 and settle_step % args_cli.trace_every == 0:
          diagnostics = _joint_tracking_diagnostics(
            robot=robot,
            joint_ids=joint_ids,
            target=settle_target,
            stiffness=stiffness,
            damping=damping,
          )
          if (
            max_pd_effort_ratio_observation is None
            or diagnostics["pd_effort_ratio_max"]
            > max_pd_effort_ratio_observation["value"]
          ):
            max_pd_effort_ratio_observation = {
              "phase": "settle",
              "step": settle_step,
              "substep": substep,
              "joint": G1_DEPLOY_JOINT_NAMES[
                int(diagnostics["pd_effort_ratio_argmax"])
              ],
              "value": diagnostics["pd_effort_ratio_max"],
            }
          if (
            max_joint_vel_ratio_observation is None
            or diagnostics["joint_vel_ratio_max"]
            > max_joint_vel_ratio_observation["value"]
          ):
            max_joint_vel_ratio_observation = {
              "phase": "settle",
              "step": settle_step,
              "substep": substep,
              "joint": G1_DEPLOY_JOINT_NAMES[
                int(diagnostics["joint_vel_ratio_argmax"])
              ],
              "value": diagnostics["joint_vel_ratio_max"],
            }
          trace_rows.append(
            {
              "phase": "settle",
              "step": settle_step,
              "substep": substep,
              "root_height": float(robot.data.root_state_w[0, 2].detach().cpu()),
              "reference_root_height": float(motion_root_height[0]),
              "joint_error": diagnostics["joint_error_mean"],
              "joint_error_max": diagnostics["joint_error_max"],
              "action_target_delta_norm": 0.0,
              "root_ang_vel_norm": float(
                torch.linalg.vector_norm(robot.data.root_ang_vel_b).detach().cpu()
              ),
              "pd_effort_ratio_max": diagnostics["pd_effort_ratio_max"],
              "joint_vel_ratio_max": diagnostics["joint_vel_ratio_max"],
              "support_pd_effort_ratio_max": diagnostics["support_pd_effort_ratio_max"],
              "support_joint_vel_ratio_max": diagnostics["support_joint_vel_ratio_max"],
            }
          )
    last_action = torch.zeros(
      (1, len(joint_ids)), dtype=torch.float32, device=sim.device
    )
    init_quat = quat_mul(
      yaw_quat(_torso_quat(robot.data.root_quat_w, robot.data.joint_pos[:, joint_ids])),
      quat_conjugate(yaw_quat(motion_root_quat[0].reshape(1, -1))),
    )
    for step in range(step_count):
      step_start = time.perf_counter()
      ref_pos = motion_joint_pos[step].reshape(1, -1)
      ref_vel = motion_joint_vel[step].reshape(1, -1)
      if args_cli.control_mode == "policy":
        if policy is None:
          raise RuntimeError("policy control mode requested without a policy runner")
        obs = _build_policy_obs(
          deploy=deploy,
          init_quat=init_quat,
          robot=robot,
          joint_ids=joint_ids,
          default_joint_pos=default_joint_pos,
          motion_root_quat=motion_root_quat[step].reshape(1, -1),
          motion_joint_pos=ref_pos,
          motion_joint_vel=ref_vel,
          last_action=last_action,
        )
        raw_action = policy(obs)
        target = _clip_action(raw_action * action_scale + action_offset, deploy)
        last_action = raw_action.detach()
      else:
        target = ref_pos
      if not torch.isfinite(target).all():
        nonfinite_step = step
        break
      action_norm = float(
        torch.linalg.vector_norm(target - action_offset).detach().cpu()
      )
      action_norms.append(action_norm)
      for substep in range(control_decimation):
        robot.set_joint_position_target(target, joint_ids=joint_ids)
        robot.write_data_to_sim()
        sim.step(render=render)
        robot.update(sim.get_physics_dt())
        current = robot.data.joint_pos[:, joint_ids]
        root_height = float(robot.data.root_state_w[0, 2].detach().cpu())
        diagnostics = _joint_tracking_diagnostics(
          robot=robot,
          joint_ids=joint_ids,
          target=target,
          stiffness=stiffness,
          damping=damping,
        )
        error = diagnostics["joint_error_mean"]
        root_heights.append(root_height)
        errors.append(error)
        error_maxes.append(diagnostics["joint_error_max"])
        pd_effort_ratios.append(diagnostics["pd_effort_ratio_max"])
        joint_vel_ratios.append(diagnostics["joint_vel_ratio_max"])
        support_pd_effort_ratios.append(diagnostics["support_pd_effort_ratio_max"])
        support_joint_vel_ratios.append(diagnostics["support_joint_vel_ratio_max"])
        if (
          max_pd_effort_ratio_observation is None
          or diagnostics["pd_effort_ratio_max"]
          > max_pd_effort_ratio_observation["value"]
        ):
          max_pd_effort_ratio_observation = {
            "phase": "policy",
            "step": step,
            "substep": substep,
            "joint": G1_DEPLOY_JOINT_NAMES[int(diagnostics["pd_effort_ratio_argmax"])],
            "value": diagnostics["pd_effort_ratio_max"],
          }
        if (
          max_joint_vel_ratio_observation is None
          or diagnostics["joint_vel_ratio_max"]
          > max_joint_vel_ratio_observation["value"]
        ):
          max_joint_vel_ratio_observation = {
            "phase": "policy",
            "step": step,
            "substep": substep,
            "joint": G1_DEPLOY_JOINT_NAMES[int(diagnostics["joint_vel_ratio_argmax"])],
            "value": diagnostics["joint_vel_ratio_max"],
          }
        if (
          max_support_pd_effort_ratio_observation is None
          or diagnostics["support_pd_effort_ratio_max"]
          > max_support_pd_effort_ratio_observation["value"]
        ):
          max_support_pd_effort_ratio_observation = {
            "phase": "policy",
            "step": step,
            "substep": substep,
            "joint": G1_DEPLOY_JOINT_NAMES[
              int(diagnostics["support_pd_effort_ratio_argmax"])
            ],
            "value": diagnostics["support_pd_effort_ratio_max"],
          }
        if (
          max_support_joint_vel_ratio_observation is None
          or diagnostics["support_joint_vel_ratio_max"]
          > max_support_joint_vel_ratio_observation["value"]
        ):
          max_support_joint_vel_ratio_observation = {
            "phase": "policy",
            "step": step,
            "substep": substep,
            "joint": G1_DEPLOY_JOINT_NAMES[
              int(diagnostics["support_joint_vel_ratio_argmax"])
            ],
            "value": diagnostics["support_joint_vel_ratio_max"],
          }
        if args_cli.trace_every > 0 and step % args_cli.trace_every == 0:
          trace_rows.append(
            {
              "phase": "policy",
              "step": step,
              "substep": substep,
              "root_height": root_height,
              "reference_root_height": float(motion_root_height[step]),
              "joint_error": error,
              "joint_error_max": diagnostics["joint_error_max"],
              "action_target_delta_norm": action_norm,
              "root_ang_vel_norm": float(
                torch.linalg.vector_norm(robot.data.root_ang_vel_b).detach().cpu()
              ),
              "pd_effort_ratio_max": diagnostics["pd_effort_ratio_max"],
              "joint_vel_ratio_max": diagnostics["joint_vel_ratio_max"],
              "support_pd_effort_ratio_max": diagnostics["support_pd_effort_ratio_max"],
              "support_joint_vel_ratio_max": diagnostics["support_joint_vel_ratio_max"],
            }
          )
        if first_fall_step is None and root_height < float(
          args_cli.fall_root_height_threshold
        ):
          first_fall_step = step
          first_fall_substep = substep
        if not torch.isfinite(current).all() or not np.isfinite(root_height):
          nonfinite_step = step
          break
        _sleep_realtime(step_start, physics_dt)
      if video_camera is not None and step % video_every == 0:
        _capture_video_frame(
          camera=video_camera,
          robot=robot,
          sim=sim,
          frames=video_frames,
        )
      if nonfinite_step is not None:
        break
      completed_steps += 1
    if nonfinite_step is not None or not args_cli.loop_playback:
      break
    if not simulation_app.is_running():
      break

  while args_cli.hold_open and simulation_app.is_running():
    step_start = time.perf_counter()
    sim.step(render=render)
    robot.update(sim.get_physics_dt())
    _sleep_realtime(step_start, sim.get_physics_dt())

  root_height_min = float(np.min(root_heights)) if root_heights else None
  full_motion_completed = completed_steps >= step_count and nonfinite_step is None
  fall_detected = root_height_min is not None and root_height_min < float(
    args_cli.fall_root_height_threshold
  )
  sim2sim_passed = full_motion_completed and not fall_detected
  if video_path is not None and video_frames:
    video_path.parent.mkdir(parents=True, exist_ok=True)
    _write_video(video_path, video_frames, fps=video_fps)

  final_report = {
    **static_report,
    "evidence_dir": str(out_dir),
    "control_mode": args_cli.control_mode,
    "initial_state": args_cli.initial_state,
    "settle_steps": max(0, int(args_cli.settle_steps)),
    "settle_mode": args_cli.settle_mode,
    "velocity_policy_root": str(velocity_policy_root)
    if velocity_policy is not None
    else None,
    "actuator_limits_mode": actuator_limits_mode,
    "trace_csv": str(out_dir / "trace.csv") if trace_rows else None,
    "video_path": str(video_path) if video_path is not None and video_frames else None,
    "video_frame_count": len(video_frames),
    "video_fps": video_fps if video_frames else None,
    "video_every": video_every if video_path is not None else None,
    "video_resolution": [int(args_cli.video_width), int(args_cli.video_height)]
    if video_path is not None
    else None,
    "fix_base": bool(args_cli.fix_base),
    "requested_steps": step_count,
    "completed_steps": min(completed_steps, step_count),
    "policy_dt": policy_dt,
    "physics_dt": physics_dt,
    "control_decimation": control_decimation,
    "full_motion_completed": full_motion_completed,
    "fall_detected": fall_detected,
    "first_fall_step": first_fall_step,
    "first_fall_substep": first_fall_substep,
    "fall_root_height_threshold": float(args_cli.fall_root_height_threshold),
    "sim2sim_passed": sim2sim_passed,
    "nonfinite_step": nonfinite_step,
    "joint_error_mean": float(np.mean(errors)) if errors else None,
    "joint_error_max": float(np.max(errors)) if errors else None,
    "joint_error_abs_max": float(np.max(error_maxes)) if error_maxes else None,
    "pd_effort_ratio_max": float(np.max(pd_effort_ratios))
    if pd_effort_ratios
    else None,
    "joint_vel_ratio_max": float(np.max(joint_vel_ratios))
    if joint_vel_ratios
    else None,
    "support_pd_effort_ratio_max": float(np.max(support_pd_effort_ratios))
    if support_pd_effort_ratios
    else None,
    "support_joint_vel_ratio_max": float(np.max(support_joint_vel_ratios))
    if support_joint_vel_ratios
    else None,
    "max_pd_effort_ratio_observation": max_pd_effort_ratio_observation,
    "max_joint_vel_ratio_observation": max_joint_vel_ratio_observation,
    "max_support_pd_effort_ratio_observation": max_support_pd_effort_ratio_observation,
    "max_support_joint_vel_ratio_observation": max_support_joint_vel_ratio_observation,
    "diagnostic_effort_model": "stiffness * (target - joint_pos) - damping * joint_vel",
    "action_target_delta_norm_mean": float(np.mean(action_norms))
    if action_norms
    else None,
    "action_target_delta_norm_max": float(np.max(action_norms))
    if action_norms
    else None,
    "root_height_min": root_height_min,
    "root_height_final": root_heights[-1] if root_heights else None,
    "reference_root_height_min": float(np.min(motion_root_height)),
    "reference_root_height_final": float(motion_root_height[-1]),
    "isaaclab_device": sim.device,
    "isaaclab_joint_names": robot.joint_names,
    "shutdown_mode": "graceful_close"
    if args_cli.graceful_close
    else "force_exit_after_report",
  }
  write_json(out_dir / "report.json", final_report)
  if trace_rows:
    trace_path = out_dir / "trace.csv"
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    with trace_path.open("w", newline="", encoding="utf-8") as handle:
      writer = csv.DictWriter(handle, fieldnames=list(trace_rows[0]))
      writer.writeheader()
      writer.writerows(trace_rows)
  return final_report


def main() -> int:
  try:
    report = _run()
  except Exception:
    traceback.print_exc()
    return 1
  print(f"ISAACLAB_SIM2SIM_REPORT={report['evidence_dir']}/report.json", flush=True)
  print(
    "ISAACLAB_SIM2SIM_RESULT "
    f"action={report['action']} "
    f"completed={report['completed_steps']}/{report['requested_steps']} "
    f"full_motion_completed={report['full_motion_completed']} "
    f"fall_detected={report['fall_detected']} "
    f"root_height_min={report['root_height_min']} "
    f"joint_error_mean={report['joint_error_mean']}",
    flush=True,
  )
  return 0 if report["sim2sim_passed"] else 1


exit_code = main()
if args_cli.graceful_close:
  simulation_app.close(wait_for_replicator=False)
  raise SystemExit(exit_code)

sys.stdout.flush()
sys.stderr.flush()
os._exit(exit_code)
