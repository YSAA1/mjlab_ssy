"""Inventory local Velocity ONNX policies against the active deploy contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def _metadata_list(value: str | None) -> list[str]:
  if not value:
    return []
  return [item.strip() for item in value.split(",") if item.strip()]


def _onnx_summary(path: Path) -> dict[str, Any]:
  try:
    import onnx
  except Exception as exc:  # pragma: no cover - depends on local runtime install.
    return {
      "path": str(path),
      "available": False,
      "reason": f"onnx_import_failed: {exc}",
    }

  try:
    model = onnx.load(path)
  except Exception as exc:
    return {"path": str(path), "available": False, "reason": f"onnx_load_failed: {exc}"}

  def dims(value: Any) -> list[int | str]:
    return [
      dim.dim_value if dim.dim_value else dim.dim_param
      for dim in value.type.tensor_type.shape.dim
    ]

  metadata = {prop.key: prop.value for prop in model.metadata_props}
  return {
    "path": str(path),
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


def _last_dim(summary: dict[str, Any], key: str) -> int | str | None:
  values = summary.get(key)
  if not isinstance(values, list) or not values:
    return None
  dims = values[0].get("dims") if isinstance(values[0], dict) else None
  if not isinstance(dims, list) or not dims:
    return None
  return dims[-1]


def _candidate_paths(search_roots: list[Path], *, limit: int) -> list[Path]:
  seen: set[Path] = set()
  paths: list[Path] = []
  for root in search_roots:
    if not root.is_dir():
      continue
    for path in sorted(root.rglob("*.onnx")):
      if "g1_velocity" not in str(path):
        continue
      resolved = path.resolve()
      if resolved in seen:
        continue
      seen.add(resolved)
      paths.append(resolved)
      if len(paths) >= limit:
        return paths
  return paths


def inventory_velocity_policies(
  *,
  reference_policy: Path,
  search_roots: list[Path],
  limit: int = 200,
) -> dict[str, Any]:
  reference = _onnx_summary(reference_policy)
  reference_input_dim = _last_dim(reference, "inputs")
  reference_output_dim = _last_dim(reference, "outputs")
  reference_obs = reference.get("metadata", {}).get("observation_names", [])

  candidates: list[dict[str, Any]] = []
  compatible: list[str] = []
  for path in _candidate_paths(search_roots, limit=limit):
    summary = _onnx_summary(path)
    candidate_input_dim = _last_dim(summary, "inputs")
    candidate_output_dim = _last_dim(summary, "outputs")
    candidate_obs = summary.get("metadata", {}).get("observation_names", [])
    is_compatible = (
      summary.get("available") is True
      and candidate_input_dim == reference_input_dim
      and candidate_output_dim == reference_output_dim
      and candidate_obs == reference_obs
    )
    item = {
      "path": str(path),
      "available": summary.get("available", False),
      "input_dim": candidate_input_dim,
      "output_dim": candidate_output_dim,
      "observation_names": candidate_obs,
      "run_path": summary.get("metadata", {}).get("run_path"),
      "compatible_with_reference": is_compatible,
    }
    if not is_compatible:
      reasons: list[str] = []
      if candidate_input_dim != reference_input_dim:
        reasons.append(f"input_dim {candidate_input_dim} != {reference_input_dim}")
      if candidate_output_dim != reference_output_dim:
        reasons.append(f"output_dim {candidate_output_dim} != {reference_output_dim}")
      if candidate_obs != reference_obs:
        reasons.append("observation_names differ")
      item["incompatibility_reasons"] = reasons
    else:
      compatible.append(str(path))
    candidates.append(item)

  return {
    "schema_version": 1,
    "reference_policy": {
      "path": str(reference_policy),
      "available": reference.get("available", False),
      "input_dim": reference_input_dim,
      "output_dim": reference_output_dim,
      "observation_names": reference_obs,
      "run_path": reference.get("metadata", {}).get("run_path"),
    },
    "search_roots": [str(root) for root in search_roots],
    "candidate_count": len(candidates),
    "compatible_count": len(compatible),
    "compatible_candidates": compatible,
    "candidates": candidates,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Inventory local G1 Velocity ONNX policies against the active deploy contract."
  )
  parser.add_argument(
    "--reference-policy",
    type=Path,
    default=Path(
      "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx"
    ),
  )
  parser.add_argument(
    "--search-root",
    type=Path,
    action="append",
    default=[],
    help="Root to scan for g1_velocity ONNX files. Can be passed multiple times.",
  )
  parser.add_argument("--limit", type=int, default=200)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-no-compatible", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  search_roots = args.search_root or [Path("/home/ssy/ssy_files/mjlab/logs/rsl_rl")]
  report = inventory_velocity_policies(
    reference_policy=args.reference_policy,
    search_roots=search_roots,
    limit=args.limit,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_no_compatible:
    return 0 if report["compatible_count"] == 0 else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
