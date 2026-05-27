from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_runtime_observation_audit import (
  audit_velocity_runtime_observations,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_runtime_observation_audit.py"


def _write_fixture_tree(
  tmp_path: Path, *, with_runtime_base_lin_vel: bool
) -> dict[str, Path]:
  worktree_velocity_cfg = tmp_path / "worktree/velocity_env_cfg.py"
  worktree_velocity_cfg.parent.mkdir(parents=True)
  worktree_velocity_cfg.write_text(
    """
actor_terms = {
  "base_lin_vel": object(),
  "base_ang_vel": object(),
  "projected_gravity": object(),
  "joint_pos": object(),
  "joint_vel": object(),
  "actions": object(),
  "command": object(),
  "height_scan": object(),
}
""",
    encoding="utf-8",
  )
  worktree_g1_cfg = tmp_path / "worktree/g1_env_cfgs.py"
  worktree_g1_cfg.write_text(
    'del cfg.observations["actor"].terms["height_scan"]\n',
    encoding="utf-8",
  )
  external_velocity_cfg = tmp_path / "external/velocity_env_cfg.py"
  external_velocity_cfg.parent.mkdir(parents=True)
  external_velocity_cfg.write_text(
    """
actor_terms = {
  "base_ang_vel": object(),
  "projected_gravity": object(),
  "command": object(),
  "phase": object(),
  "joint_pos": object(),
  "joint_vel": object(),
  "actions": object(),
  "height_scan": object(),
}
""",
    encoding="utf-8",
  )
  external_g1_cfg = tmp_path / "external/g1_env_cfgs.py"
  external_g1_cfg.write_text(
    'del cfg.observations["actor"].terms["height_scan"]\n',
    encoding="utf-8",
  )
  deploy_observations_h = tmp_path / "deploy/observations.h"
  deploy_observations_h.parent.mkdir(parents=True)
  registered = [
    "base_ang_vel",
    "projected_gravity",
    "joint_pos_rel",
    "joint_vel_rel",
    "last_action",
    "velocity_commands",
    "gait_phase",
  ]
  if with_runtime_base_lin_vel:
    registered.append("base_lin_vel")
  deploy_observations_h.write_text(
    "\n".join(f"REGISTER_OBSERVATION({name}) {{}}" for name in registered),
    encoding="utf-8",
  )
  articulation_h = tmp_path / "deploy/articulation.h"
  linear_field = "Eigen::Vector3f root_lin_vel_b;" if with_runtime_base_lin_vel else ""
  articulation_h.write_text(
    f"""
struct ArticulationData {{
  Eigen::Vector3f root_ang_vel_b;
  Eigen::Vector3f projected_gravity_b;
  Eigen::VectorXf joint_pos;
  Eigen::VectorXf joint_vel;
  {linear_field}
}};
""",
    encoding="utf-8",
  )
  unitree_articulation_h = tmp_path / "deploy/unitree_articulation.h"
  unitree_articulation_h.write_text(
    "data.root_lin_vel_b[0] = lowstate->msg_.imu_state().accelerometer()[0];\n"
    if with_runtime_base_lin_vel
    else "data.root_ang_vel_b[0] = lowstate->msg_.imu_state().gyroscope()[0];\n",
    encoding="utf-8",
  )
  deploy_yaml = tmp_path / "deploy/deploy.yaml"
  deploy_yaml.write_text(
    """
observations:
  base_ang_vel: {scale: [1, 1, 1]}
  projected_gravity: {scale: [1, 1, 1]}
  velocity_commands: {scale: [1, 1, 1]}
  gait_phase: {scale: [1, 1]}
  joint_pos_rel: {scale: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}
  joint_vel_rel: {scale: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}
  last_action: {scale: [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]}
""",
    encoding="utf-8",
  )
  return {
    "worktree_velocity_cfg": worktree_velocity_cfg,
    "worktree_g1_cfg": worktree_g1_cfg,
    "external_velocity_cfg": external_velocity_cfg,
    "external_g1_cfg": external_g1_cfg,
    "deploy_observations_h": deploy_observations_h,
    "articulation_h": articulation_h,
    "unitree_articulation_h": unitree_articulation_h,
    "deploy_yaml": deploy_yaml,
  }


def test_audit_blocks_99_dim_route_when_runtime_lacks_base_lin_vel(
  tmp_path: Path,
) -> None:
  paths = _write_fixture_tree(tmp_path, with_runtime_base_lin_vel=False)

  report = audit_velocity_runtime_observations(**paths)

  assert report["contracts"]["worktree_flat_actor"]["known_dim"] == 99
  assert report["contracts"]["external_source_flat_actor"]["known_dim"] == 98
  assert report["contracts"]["active_deploy_yaml"]["known_dim"] == 98
  assert report["deploy_runtime"]["has_base_lin_vel_observation"] is False
  assert (
    report["decision"]["can_run_current_source_99_dim_contract_without_runtime_patch"]
    is False
  )
  assert (
    report["decision"]["recommended_next"]
    == "do_not_generate_99_dim_package_until_runtime_base_lin_vel_source_exists"
  )


def test_audit_allows_99_dim_route_when_runtime_has_base_lin_vel(
  tmp_path: Path,
) -> None:
  paths = _write_fixture_tree(tmp_path, with_runtime_base_lin_vel=True)

  report = audit_velocity_runtime_observations(**paths)

  assert report["deploy_runtime"]["has_base_lin_vel_observation"] is True
  assert report["deploy_runtime"]["has_articulation_linear_velocity_field"] is True
  assert (
    report["decision"]["can_run_current_source_99_dim_contract_without_runtime_patch"]
    is True
  )


def test_cli_expect_runtime_missing_base_lin_vel(tmp_path: Path) -> None:
  paths = _write_fixture_tree(tmp_path, with_runtime_base_lin_vel=False)

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--worktree-velocity-cfg",
      str(paths["worktree_velocity_cfg"]),
      "--worktree-g1-cfg",
      str(paths["worktree_g1_cfg"]),
      "--external-velocity-cfg",
      str(paths["external_velocity_cfg"]),
      "--external-g1-cfg",
      str(paths["external_g1_cfg"]),
      "--deploy-observations-h",
      str(paths["deploy_observations_h"]),
      "--articulation-h",
      str(paths["articulation_h"]),
      "--unitree-articulation-h",
      str(paths["unitree_articulation_h"]),
      "--deploy-yaml",
      str(paths["deploy_yaml"]),
      "--expect-runtime-missing-base-lin-vel",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"has_base_lin_vel_observation": false' in proc.stdout
