"""Audit whether the Unitree deploy runtime can serve local Velocity observations."""

from __future__ import annotations

import argparse
import ast
import json
import re
from pathlib import Path
from typing import Any

import yaml

DOF = 29
TERM_DIMS = {
  "base_lin_vel": 3,
  "base_ang_vel": 3,
  "projected_gravity": 3,
  "command": 3,
  "velocity_commands": 3,
  "phase": 2,
  "gait_phase": 2,
  "joint_pos": DOF,
  "joint_pos_rel": DOF,
  "joint_vel": DOF,
  "joint_vel_rel": DOF,
  "actions": DOF,
  "last_action": DOF,
}

REGISTER_OBSERVATION_RE = re.compile(r"REGISTER_OBSERVATION\((?P<name>\w+)\)")


def _actor_terms(path: Path) -> list[str]:
  tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
  for node in ast.walk(tree):
    if not isinstance(node, ast.Assign):
      continue
    if not any(
      isinstance(target, ast.Name) and target.id == "actor_terms"
      for target in node.targets
    ):
      continue
    if not isinstance(node.value, ast.Dict):
      continue
    terms: list[str] = []
    for key in node.value.keys:
      if isinstance(key, ast.Constant) and isinstance(key.value, str):
        terms.append(key.value)
    return terms
  return []


def _flat_removed_actor_terms(path: Path) -> list[str]:
  text = path.read_text(encoding="utf-8")
  return sorted(
    set(
      re.findall(
        r"del\s+cfg\.observations\[[\"']actor[\"']\]\.terms\[[\"']([^\"']+)[\"']\]",
        text,
      )
    )
  )


def _flat_terms(base_terms: list[str], removed_terms: list[str]) -> list[str]:
  removed = set(removed_terms)
  return [term for term in base_terms if term not in removed]


def _dim_report(terms: list[str]) -> dict[str, Any]:
  unknown = [term for term in terms if term not in TERM_DIMS]
  known_dim = sum(TERM_DIMS[term] for term in terms if term in TERM_DIMS)
  return {
    "terms": terms,
    "known_dim": known_dim,
    "unknown_terms": unknown,
    "complete_dim_known": not unknown,
  }


def _registered_observations(path: Path) -> list[str]:
  if not path.is_file():
    return []
  text = path.read_text(encoding="utf-8")
  return sorted(match.group("name") for match in REGISTER_OBSERVATION_RE.finditer(text))


def _struct_fields(path: Path, *, struct_name: str) -> list[str]:
  if not path.is_file():
    return []
  text = path.read_text(encoding="utf-8")
  match = re.search(
    rf"struct\s+{re.escape(struct_name)}\s*\{{(?P<body>.*?)\}};", text, re.S
  )
  if not match:
    return []
  fields: list[str] = []
  for line in match.group("body").splitlines():
    line = line.split("//", 1)[0].strip()
    if not line or "(" in line:
      continue
    field_match = re.search(r"(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*(?:=[^;]*)?;", line)
    if field_match:
      fields.append(field_match.group("name"))
  return fields


def _deploy_observation_terms(path: Path) -> list[str]:
  if not path.is_file():
    return []
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    return []
  observations = data.get("observations")
  if not isinstance(observations, dict):
    return []
  return [str(key) for key in observations.keys()]


def _update_mentions_linear_velocity(path: Path) -> bool:
  if not path.is_file():
    return False
  text = path.read_text(encoding="utf-8")
  return bool(re.search(r"lin(?:ear)?_vel|velocimeter|imu_lin_vel", text))


def audit_velocity_runtime_observations(
  *,
  worktree_velocity_cfg: Path,
  worktree_g1_cfg: Path,
  external_velocity_cfg: Path,
  external_g1_cfg: Path,
  deploy_observations_h: Path,
  articulation_h: Path,
  unitree_articulation_h: Path,
  deploy_yaml: Path,
) -> dict[str, Any]:
  worktree_terms = _flat_terms(
    _actor_terms(worktree_velocity_cfg),
    _flat_removed_actor_terms(worktree_g1_cfg),
  )
  external_terms = _flat_terms(
    _actor_terms(external_velocity_cfg),
    _flat_removed_actor_terms(external_g1_cfg),
  )
  registered = _registered_observations(deploy_observations_h)
  fields = _struct_fields(articulation_h, struct_name="ArticulationData")
  deploy_terms = _deploy_observation_terms(deploy_yaml)

  has_runtime_base_lin_vel = "base_lin_vel" in registered
  has_articulation_linear_velocity = any(
    field in fields
    for field in (
      "root_lin_vel_b",
      "root_link_lin_vel_b",
      "base_lin_vel",
      "imu_lin_vel",
    )
  )
  unitree_update_has_linear_velocity = _update_mentions_linear_velocity(
    unitree_articulation_h
  )
  can_run_current_99 = (
    "base_lin_vel" in worktree_terms
    and has_runtime_base_lin_vel
    and has_articulation_linear_velocity
    and unitree_update_has_linear_velocity
  )

  return {
    "schema_version": 1,
    "paths": {
      "worktree_velocity_cfg": str(worktree_velocity_cfg),
      "worktree_g1_cfg": str(worktree_g1_cfg),
      "external_velocity_cfg": str(external_velocity_cfg),
      "external_g1_cfg": str(external_g1_cfg),
      "deploy_observations_h": str(deploy_observations_h),
      "articulation_h": str(articulation_h),
      "unitree_articulation_h": str(unitree_articulation_h),
      "deploy_yaml": str(deploy_yaml),
    },
    "contracts": {
      "worktree_flat_actor": _dim_report(worktree_terms),
      "external_source_flat_actor": _dim_report(external_terms),
      "active_deploy_yaml": _dim_report(deploy_terms),
    },
    "deploy_runtime": {
      "registered_observations": registered,
      "has_base_lin_vel_observation": has_runtime_base_lin_vel,
      "articulation_data_fields": fields,
      "has_articulation_linear_velocity_field": has_articulation_linear_velocity,
      "unitree_update_mentions_linear_velocity": unitree_update_has_linear_velocity,
    },
    "decision": {
      "can_run_current_source_99_dim_contract_without_runtime_patch": can_run_current_99,
      "safe_to_generate_99_dim_deploy_yaml_only": can_run_current_99,
      "recommended_next": (
        "generate_99_dim_deploy_package_and_smoke"
        if can_run_current_99
        else "do_not_generate_99_dim_package_until_runtime_base_lin_vel_source_exists"
      ),
      "alternative": "train_or_export_a_98_dim_velocity_policy_for_the_active_runtime_contract",
      "real_robot_gate": "locked",
    },
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Audit Velocity observation contract support in the Unitree deploy runtime."
  )
  external_root = Path("/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab")
  parser.add_argument(
    "--worktree-velocity-cfg",
    type=Path,
    default=Path("src/mjlab/tasks/velocity/velocity_env_cfg.py"),
  )
  parser.add_argument(
    "--worktree-g1-cfg",
    type=Path,
    default=Path("src/mjlab/tasks/velocity/config/g1/env_cfgs.py"),
  )
  parser.add_argument(
    "--external-velocity-cfg",
    type=Path,
    default=external_root / "src/tasks/velocity/velocity_env_cfg.py",
  )
  parser.add_argument(
    "--external-g1-cfg",
    type=Path,
    default=external_root / "src/tasks/velocity/config/g1/env_cfgs.py",
  )
  parser.add_argument(
    "--deploy-observations-h",
    type=Path,
    default=external_root
    / "deploy/include/isaaclab/envs/mdp/observations/observations.h",
  )
  parser.add_argument(
    "--articulation-h",
    type=Path,
    default=external_root
    / "deploy/include/isaaclab/assets/articulation/articulation.h",
  )
  parser.add_argument(
    "--unitree-articulation-h",
    type=Path,
    default=external_root / "deploy/include/unitree_articulation.h",
  )
  parser.add_argument(
    "--deploy-yaml",
    type=Path,
    default=external_root
    / "deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml",
  )
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-runtime-missing-base-lin-vel", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = audit_velocity_runtime_observations(
    worktree_velocity_cfg=args.worktree_velocity_cfg,
    worktree_g1_cfg=args.worktree_g1_cfg,
    external_velocity_cfg=args.external_velocity_cfg,
    external_g1_cfg=args.external_g1_cfg,
    deploy_observations_h=args.deploy_observations_h,
    articulation_h=args.articulation_h,
    unitree_articulation_h=args.unitree_articulation_h,
    deploy_yaml=args.deploy_yaml,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_runtime_missing_base_lin_vel:
    missing = not report["deploy_runtime"]["has_base_lin_vel_observation"]
    missing = (
      missing and not report["deploy_runtime"]["has_articulation_linear_velocity_field"]
    )
    return 0 if missing else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
