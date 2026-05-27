"""Validate the 98-dim G1 Velocity task against the Unitree deploy runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

import mjlab.tasks  # noqa: F401
from mjlab.tasks.registry import load_env_cfg

TERM_DIMS = {
  "base_ang_vel": 3,
  "projected_gravity": 3,
  "command": 3,
  "velocity_commands": 3,
  "phase": 2,
  "gait_phase": 2,
  "joint_pos": 29,
  "joint_pos_rel": 29,
  "joint_vel": 29,
  "joint_vel_rel": 29,
  "actions": 29,
  "last_action": 29,
}
DEPLOY98_TO_RUNTIME = {
  "base_ang_vel": "base_ang_vel",
  "projected_gravity": "projected_gravity",
  "command": "velocity_commands",
  "phase": "gait_phase",
  "joint_pos": "joint_pos_rel",
  "joint_vel": "joint_vel_rel",
  "actions": "last_action",
}


def _dim_report(terms: list[str]) -> dict[str, Any]:
  unknown = [term for term in terms if term not in TERM_DIMS]
  return {
    "terms": terms,
    "known_dim": sum(TERM_DIMS[term] for term in terms if term in TERM_DIMS),
    "unknown_terms": unknown,
    "complete_dim_known": not unknown,
  }


def _deploy_observation_terms(path: Path) -> list[str]:
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict) or not isinstance(data.get("observations"), dict):
    return []
  return [str(key) for key in data["observations"].keys()]


def validate_deploy98_task_contract(
  *,
  task_id: str,
  deploy_yaml: Path,
) -> dict[str, Any]:
  cfg = load_env_cfg(task_id)
  actor_terms = list(cfg.observations["actor"].terms)
  runtime_terms = _deploy_observation_terms(deploy_yaml)
  mapped_runtime_terms = [
    DEPLOY98_TO_RUNTIME[term] for term in actor_terms if term in DEPLOY98_TO_RUNTIME
  ]
  missing_mapping_terms = [
    term for term in actor_terms if term not in DEPLOY98_TO_RUNTIME
  ]
  report = {
    "schema_version": 1,
    "task_id": task_id,
    "deploy_yaml": str(deploy_yaml),
    "actor_contract": _dim_report(actor_terms),
    "runtime_contract": _dim_report(runtime_terms),
    "semantic_mapping": {
      "mapping": DEPLOY98_TO_RUNTIME,
      "mapped_runtime_terms": mapped_runtime_terms,
      "missing_mapping_terms": missing_mapping_terms,
      "matches_runtime_terms": mapped_runtime_terms == runtime_terms,
    },
  }
  report["decision"] = {
    "task_contract_matches_active_runtime": (
      report["actor_contract"]["known_dim"] == 98
      and report["runtime_contract"]["known_dim"] == 98
      and not report["actor_contract"]["unknown_terms"]
      and not report["runtime_contract"]["unknown_terms"]
      and mapped_runtime_terms == runtime_terms
      and not missing_mapping_terms
    ),
    "safe_to_train_for_active_98_dim_runtime": True,
    "safe_to_swap_without_training": False,
    "real_robot_gate": "locked",
  }
  return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Validate the G1 deploy98 Velocity task contract."
  )
  parser.add_argument(
    "--task-id",
    default="Mjlab-Velocity-Flat-Unitree-G1-Deploy98",
  )
  parser.add_argument(
    "--deploy-yaml",
    type=Path,
    default=Path(
      "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/"
      "config/policy/velocity/v0/params/deploy.yaml"
    ),
  )
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-compatible", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = validate_deploy98_task_contract(
    task_id=args.task_id,
    deploy_yaml=args.deploy_yaml,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_compatible:
    return 0 if report["decision"]["task_contract_matches_active_runtime"] else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
