"""Triage local Velocity runs as deploy remediation candidates."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

from mjlab.scripts.g1_tracking_phase1_velocity_policy_inventory import (
  _candidate_paths,
  _last_dim,
  _onnx_summary,
)

ACTIVE_V0_OBSERVATIONS = [
  "base_ang_vel",
  "projected_gravity",
  "command",
  "phase",
  "joint_pos",
  "joint_vel",
  "actions",
]
CURRENT_SOURCE_FLAT_OBSERVATIONS = [
  "base_lin_vel",
  "base_ang_vel",
  "projected_gravity",
  "joint_pos",
  "joint_vel",
  "actions",
  "command",
]

CHECKPOINT_RE = re.compile(r"model_(?P<iteration>\d+)\.pt$")


def _find_run_dir(onnx_path: Path, search_roots: list[Path]) -> Path:
  path = onnx_path.resolve()
  root_resolved = [root.resolve() for root in search_roots if root.exists()]
  for parent in [path.parent, *path.parents]:
    if (parent / "params").is_dir():
      return parent
    if any((parent / f"model_{iteration}.pt").is_file() for iteration in (0, 1)):
      return parent
    if any(parent == root for root in root_resolved):
      break
  if path.parent.name == "exported":
    return path.parent.parent
  return path.parent


def _checkpoint_iteration(path: Path) -> int | None:
  match = CHECKPOINT_RE.match(path.name)
  if not match:
    return None
  return int(match.group("iteration"))


def _checkpoint_summary(run_dir: Path) -> dict[str, Any]:
  rows: list[tuple[int, Path]] = []
  for path in sorted(run_dir.glob("model_*.pt")):
    iteration = _checkpoint_iteration(path)
    if iteration is not None:
      rows.append((iteration, path))
  if not rows:
    return {
      "count": 0,
      "latest_iteration": None,
      "latest_path": None,
    }
  latest_iteration, latest_path = max(rows, key=lambda item: item[0])
  return {
    "count": len(rows),
    "latest_iteration": latest_iteration,
    "latest_path": str(latest_path),
  }


def _params_summary(run_dir: Path) -> dict[str, Any]:
  params_dir = run_dir / "params"
  return {
    "env_yaml": str(params_dir / "env.yaml")
    if (params_dir / "env.yaml").is_file()
    else None,
    "agent_yaml": str(params_dir / "agent.yaml")
    if (params_dir / "agent.yaml").is_file()
    else None,
    "deploy_yaml": str(params_dir / "deploy.yaml")
    if (params_dir / "deploy.yaml").is_file()
    else None,
  }


def _has_complete_unitree_deploy_package(run_dir: Path, onnx_path: Path) -> bool:
  has_deploy_yaml = (run_dir / "params" / "deploy.yaml").is_file()
  exported_policy = run_dir / "exported" / "policy.onnx"
  return has_deploy_yaml and (
    exported_policy.is_file() or onnx_path.resolve() == exported_policy.resolve()
  )


def _candidate_kind(
  *,
  input_dim: int | str | None,
  output_dim: int | str | None,
  observation_names: list[str],
  reference_input_dim: int | str | None,
  reference_output_dim: int | str | None,
  reference_observations: list[str],
) -> str:
  if (
    input_dim == reference_input_dim
    and output_dim == reference_output_dim
    and observation_names == reference_observations
  ):
    return "active_v0_contract"
  if (
    input_dim == 99
    and output_dim == 29
    and observation_names == CURRENT_SOURCE_FLAT_OBSERVATIONS
  ):
    return "current_source_flat_velocity_actor"
  if output_dim == 29 and input_dim == 286:
    return "rough_terrain_or_history_velocity_actor"
  if output_dim == 29:
    return "other_velocity_actor"
  return "unusable_or_unknown"


def triage_velocity_deploy_candidates(
  *,
  reference_policy: Path,
  search_roots: list[Path],
  limit: int = 300,
) -> dict[str, Any]:
  reference = _onnx_summary(reference_policy)
  reference_input_dim = _last_dim(reference, "inputs")
  reference_output_dim = _last_dim(reference, "outputs")
  reference_observations = reference.get("metadata", {}).get("observation_names", [])
  if not reference_observations:
    reference_observations = ACTIVE_V0_OBSERVATIONS

  candidates: list[dict[str, Any]] = []
  counts = {
    "active_v0_contract": 0,
    "direct_swap_ready": 0,
    "current_source_flat_velocity_actor": 0,
    "actor_reexport_ready": 0,
    "complete_unitree_deploy_package": 0,
    "rough_terrain_or_history_velocity_actor": 0,
    "other_velocity_actor": 0,
    "unusable_or_unknown": 0,
  }
  for onnx_path in _candidate_paths(search_roots, limit=limit):
    summary = _onnx_summary(onnx_path)
    input_dim = _last_dim(summary, "inputs")
    output_dim = _last_dim(summary, "outputs")
    metadata = summary.get("metadata", {})
    observation_names = metadata.get("observation_names", [])
    run_dir = _find_run_dir(onnx_path, search_roots)
    params = _params_summary(run_dir)
    checkpoints = _checkpoint_summary(run_dir)
    complete_package = _has_complete_unitree_deploy_package(run_dir, onnx_path)
    kind = _candidate_kind(
      input_dim=input_dim,
      output_dim=output_dim,
      observation_names=observation_names,
      reference_input_dim=reference_input_dim,
      reference_output_dim=reference_output_dim,
      reference_observations=reference_observations,
    )
    direct_swap_ready = kind == "active_v0_contract" and complete_package
    actor_reexport_ready = (
      kind == "current_source_flat_velocity_actor"
      and params["env_yaml"] is not None
      and params["agent_yaml"] is not None
      and checkpoints["latest_path"] is not None
    )

    counts[kind] += 1
    if direct_swap_ready:
      counts["direct_swap_ready"] += 1
    if actor_reexport_ready:
      counts["actor_reexport_ready"] += 1
    if complete_package:
      counts["complete_unitree_deploy_package"] += 1

    blockers: list[str] = []
    if kind != "active_v0_contract":
      blockers.append("not_active_v0_observation_contract")
    if kind == "current_source_flat_velocity_actor":
      blockers.append("requires_99_dim_runtime_observation_support")
      blockers.append("requires_unitree_deploy_yaml_generation")
    if not complete_package:
      blockers.append("missing_complete_unitree_deploy_package")
    if kind != "active_v0_contract":
      if params["env_yaml"] is None or params["agent_yaml"] is None:
        blockers.append("missing_training_params")
      if checkpoints["latest_path"] is None:
        blockers.append("missing_checkpoint")

    candidates.append(
      {
        "onnx_path": str(onnx_path),
        "run_dir": str(run_dir),
        "kind": kind,
        "input_dim": input_dim,
        "output_dim": output_dim,
        "observation_names": observation_names,
        "run_path_metadata": metadata.get("run_path"),
        "params": params,
        "checkpoints": checkpoints,
        "complete_unitree_deploy_package": complete_package,
        "direct_swap_ready": direct_swap_ready,
        "actor_reexport_ready": actor_reexport_ready,
        "blockers": blockers,
      }
    )

  def sort_key(item: dict[str, Any]) -> tuple[int, int, str]:
    kind_rank = {
      "active_v0_contract": 0,
      "current_source_flat_velocity_actor": 1,
      "other_velocity_actor": 2,
      "rough_terrain_or_history_velocity_actor": 3,
      "unusable_or_unknown": 4,
    }.get(item["kind"], 9)
    latest = item["checkpoints"]["latest_iteration"]
    latest_rank = -1 if latest is None else int(latest)
    return (kind_rank, -latest_rank, item["onnx_path"])

  candidates.sort(key=sort_key)
  current_source_ready = [
    item
    for item in candidates
    if item["kind"] == "current_source_flat_velocity_actor"
    and item["actor_reexport_ready"]
  ][:20]

  decision = {
    "direct_replacement_available": counts["direct_swap_ready"] > 0,
    "has_current_source_reexport_candidates": bool(current_source_ready),
    "safe_to_swap_local_onnx_into_active_v0": counts["direct_swap_ready"] > 0,
    "remediation": (
      "direct_swap"
      if counts["direct_swap_ready"] > 0
      else "generate_99_dim_deploy_package_or_retrain_98_dim_velocity"
    ),
    "real_robot_gate": "locked",
  }
  if not decision["direct_replacement_available"]:
    decision["reason"] = (
      "No scanned local candidate is both active-v0 observation compatible and "
      "packaged as a complete Unitree deploy policy_dir."
    )

  return {
    "schema_version": 1,
    "reference_policy": {
      "path": str(reference_policy),
      "available": reference.get("available", False),
      "input_dim": reference_input_dim,
      "output_dim": reference_output_dim,
      "observation_names": reference_observations,
      "run_path": reference.get("metadata", {}).get("run_path"),
    },
    "current_source_contract": {
      "input_dim": 99,
      "output_dim": 29,
      "observation_names": CURRENT_SOURCE_FLAT_OBSERVATIONS,
      "deploy_runtime_gap": "active g1_ctrl v0 runtime currently uses 98-dim observations",
    },
    "search_roots": [str(root) for root in search_roots],
    "candidate_count": len(candidates),
    "counts": counts,
    "decision": decision,
    "current_source_reexport_candidates": current_source_ready,
    "candidates": candidates,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description=(
      "Triage local G1 Velocity runs as deploy remediation candidates without "
      "modifying the active Unitree controller."
    )
  )
  parser.add_argument(
    "--reference-policy",
    type=Path,
    default=Path(
      "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/"
      "config/policy/velocity/v0/exported/policy.onnx"
    ),
  )
  parser.add_argument(
    "--search-root",
    type=Path,
    action="append",
    default=[],
    help="Root to scan for g1_velocity ONNX files. Can be passed multiple times.",
  )
  parser.add_argument("--limit", type=int, default=300)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument(
    "--summary-only",
    action="store_true",
    help="Print a compact summary while writing the full report to --report-out.",
  )
  parser.add_argument("--expect-no-direct-ready", action="store_true")
  return parser.parse_args(argv)


def _stdout_report(report: dict[str, Any], *, summary_only: bool) -> dict[str, Any]:
  if not summary_only:
    return report
  return {
    "schema_version": report["schema_version"],
    "reference_policy": report["reference_policy"],
    "current_source_contract": report["current_source_contract"],
    "search_roots": report["search_roots"],
    "candidate_count": report["candidate_count"],
    "counts": report["counts"],
    "decision": report["decision"],
    "current_source_reexport_candidates": report["current_source_reexport_candidates"][
      :5
    ],
  }


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  search_roots = args.search_root or [Path("/home/ssy/ssy_files/mjlab/logs/rsl_rl")]
  report = triage_velocity_deploy_candidates(
    reference_policy=args.reference_policy,
    search_roots=search_roots,
    limit=args.limit,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(
    json.dumps(
      _stdout_report(report, summary_only=args.summary_only),
      indent=2,
      sort_keys=True,
    )
  )
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_no_direct_ready:
    return 0 if not report["decision"]["direct_replacement_available"] else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
