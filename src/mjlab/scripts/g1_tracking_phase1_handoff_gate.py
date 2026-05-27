"""Gate G1 phase-1 entry-state handoff contracts."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml


class HandoffGateError(RuntimeError):
  """Raised when handoff-gate inputs are invalid."""


def _load_json(path: Path) -> dict[str, Any]:
  with path.open("r", encoding="utf-8") as handle:
    data = json.load(handle)
  if not isinstance(data, dict):
    raise HandoffGateError(f"JSON root must be a mapping: {path}")
  return data


def _load_yaml(path: Path) -> dict[str, Any]:
  with path.open("r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle)
  if not isinstance(data, dict):
    raise HandoffGateError(f"YAML root must be a mapping: {path}")
  return data


def _motion_fps(motion: np.lib.npyio.NpzFile) -> float:
  if "fps" not in motion:
    return 50.0
  value = np.asarray(motion["fps"]).reshape(-1)
  if value.size == 0:
    return 50.0
  return float(value[0])


def _gap(diff: np.ndarray) -> dict[str, float]:
  return {
    "l2": round(float(np.linalg.norm(diff)), 6),
    "max": round(float(np.max(np.abs(diff))), 6),
  }


def _reference_qpos(motion: np.lib.npyio.NpzFile) -> np.ndarray:
  for key in ("body_pos_w", "body_quat_w", "joint_pos"):
    if key not in motion:
      raise HandoffGateError(f"motion npz is missing required key: {key}")
  root_pos = np.asarray(motion["body_pos_w"], dtype=float)
  root_quat = np.asarray(motion["body_quat_w"], dtype=float)
  joint_pos = np.asarray(motion["joint_pos"], dtype=float)
  if root_pos.ndim != 3 or root_pos.shape[0] == 0 or root_pos.shape[2] != 3:
    raise HandoffGateError("body_pos_w must have shape [T, bodies, 3]")
  if root_quat.ndim != 3 or root_quat.shape[0] == 0 or root_quat.shape[2] != 4:
    raise HandoffGateError("body_quat_w must have shape [T, bodies, 4]")
  if joint_pos.ndim != 2 or joint_pos.shape[0] == 0:
    raise HandoffGateError("joint_pos must have shape [T, dof]")
  return np.asarray(
    [
      0.0,
      0.0,
      float(root_pos[0, 0, 2]),
      *root_quat[0, 0].tolist(),
      *joint_pos[0].tolist(),
    ],
    dtype=float,
  )


def _deploy_default_entry(
  *,
  default_joint_pos: np.ndarray,
  joint_pos: np.ndarray,
  fps: float,
  max_l2: float,
  max_max: float,
) -> dict[str, Any]:
  frame0_diff = default_joint_pos - joint_pos[0]
  frame_gaps = np.linalg.norm(joint_pos - default_joint_pos[None, :], axis=1)
  best_frame = int(np.argmin(frame_gaps))
  frame0_gap = _gap(frame0_diff)
  passed = frame0_gap["l2"] <= max_l2 and frame0_gap["max"] <= max_max
  return {
    "entry_type": "deploy_default",
    "deploy_acceptance_candidate": True,
    "passed": passed,
    "reason": None if passed else "entry_state_pose_mismatch",
    "frame0_gap_l2": frame0_gap["l2"],
    "frame0_gap_max": frame0_gap["max"],
    "max_deploy_entry_gap_l2": max_l2,
    "max_deploy_entry_gap_max": max_max,
    "best_default_pose_frame": best_frame,
    "best_default_pose_time_s": round(best_frame / fps, 6),
    "best_default_pose_gap_l2": round(float(frame_gaps[best_frame]), 6),
  }


def _qpos_reference_entry(
  *,
  qpos: np.ndarray | None,
  reference_qpos: np.ndarray,
  max_l2: float,
  max_max: float,
) -> dict[str, Any]:
  expected_len = int(reference_qpos.size)
  if qpos is None:
    return {
      "present": False,
      "expected_qpos_len": expected_len,
      "entry_type": "missing",
      "deploy_acceptance_candidate": False,
      "matches_reference_frame0": False,
    }
  if qpos.size != expected_len:
    return {
      "present": True,
      "qpos_len": int(qpos.size),
      "expected_qpos_len": expected_len,
      "entry_type": "invalid_qpos_length",
      "deploy_acceptance_candidate": False,
      "matches_reference_frame0": False,
    }

  joint_gap = _gap(qpos[7:] - reference_qpos[7:])
  root_z_gap = round(abs(float(qpos[2] - reference_qpos[2])), 6)
  root_quat_gap_l2 = round(float(np.linalg.norm(qpos[3:7] - reference_qpos[3:7])), 6)
  matches_reference = (
    joint_gap["l2"] <= max_l2
    and joint_gap["max"] <= max_max
    and root_z_gap <= max_max
    and root_quat_gap_l2 <= max_l2
  )
  return {
    "present": True,
    "qpos_len": expected_len,
    "expected_qpos_len": expected_len,
    "entry_type": "sim_teleport_only" if matches_reference else "not_this_reference",
    "deploy_acceptance_candidate": False,
    "matches_reference_frame0": matches_reference,
    "joint_gap_to_reference_l2": joint_gap["l2"],
    "joint_gap_to_reference_max": joint_gap["max"],
    "root_z_gap_to_reference": root_z_gap,
    "root_quat_gap_l2_to_reference": root_quat_gap_l2,
    "max_qpos_reference_gap_l2": max_l2,
    "max_qpos_reference_gap_max": max_max,
  }


def _active_sim_qpos(sim_config: dict[str, Any]) -> np.ndarray | None:
  raw = sim_config.get("initial_qpos")
  if raw is None:
    return None
  if not isinstance(raw, list):
    raise HandoffGateError("simulate config initial_qpos must be a list")
  return np.asarray(raw, dtype=float)


def analyze_handoff_gate(
  manifest_path: Path,
  *,
  sim_config_override: Path | None = None,
  max_deploy_entry_gap_l2: float = 0.5,
  max_deploy_entry_gap_max: float = 0.35,
  max_qpos_reference_gap_l2: float = 0.05,
  max_qpos_reference_gap_max: float = 0.05,
) -> dict[str, Any]:
  manifest = _load_json(manifest_path)
  sim_config_path = (
    sim_config_override
    or Path(manifest["deploy_configs"]["sim_config"]["path"]).expanduser()
  )
  sim_config = _load_yaml(sim_config_path)
  sim_qpos = _active_sim_qpos(sim_config)

  action_reports: dict[str, Any] = {}
  for action, bundle in sorted(manifest.get("actions", {}).items()):
    deploy_yaml = Path(bundle["deploy_yaml"]["path"])
    motion_npz = Path(bundle["deploy_motion_npz"]["path"])
    deploy = _load_yaml(deploy_yaml)
    default_joint_pos = np.asarray(deploy.get("default_joint_pos", []), dtype=float)
    with np.load(motion_npz) as motion:
      joint_pos = np.asarray(motion["joint_pos"], dtype=float)
      if joint_pos.ndim != 2:
        raise HandoffGateError(f"motion joint_pos must be 2D: {motion_npz}")
      if default_joint_pos.size != joint_pos.shape[1]:
        raise HandoffGateError(
          f"default_joint_pos length {default_joint_pos.size} does not match "
          f"motion dim {joint_pos.shape[1]}: {deploy_yaml}"
        )
      fps = _motion_fps(motion)
      reference_qpos = _reference_qpos(motion)

    deploy_default = _deploy_default_entry(
      default_joint_pos=default_joint_pos,
      joint_pos=joint_pos,
      fps=fps,
      max_l2=max_deploy_entry_gap_l2,
      max_max=max_deploy_entry_gap_max,
    )
    active_sim = _qpos_reference_entry(
      qpos=sim_qpos,
      reference_qpos=reference_qpos,
      max_l2=max_qpos_reference_gap_l2,
      max_max=max_qpos_reference_gap_max,
    )
    reference_init = _qpos_reference_entry(
      qpos=reference_qpos,
      reference_qpos=reference_qpos,
      max_l2=max_qpos_reference_gap_l2,
      max_max=max_qpos_reference_gap_max,
    )
    reference_init["entry_type"] = "sim_teleport_only"

    deploy_safe_entry_available = bool(deploy_default["passed"])
    action_reports[action] = {
      "passed": deploy_safe_entry_available,
      "primary_reason": None
      if deploy_safe_entry_available
      else "no_deploy_safe_entry_contract",
      "deploy_default_entry": deploy_default,
      "reference_initial_qpos_entry": reference_init,
      "active_sim_initial_qpos_entry": active_sim,
      "deploy_safe_transition_entry": {
        "available": deploy_safe_entry_available,
        "candidate_available": not deploy_safe_entry_available,
        "source": "deploy_default_entry"
        if deploy_safe_entry_available
        else "sim2sim_prepose_mode",
        "reason": None
        if deploy_safe_entry_available
        else (
          "Prepose mode can create a controller-reproducible candidate, but no "
          "post-entry sim2sim2 evidence has passed yet."
        ),
      },
      "real_robot_unlocked": False,
    }

  passed = bool(action_reports) and all(
    report["passed"] for report in action_reports.values()
  )
  any_sim_teleport = any(
    report["active_sim_initial_qpos_entry"]["entry_type"] == "sim_teleport_only"
    for report in action_reports.values()
  )
  return {
    "schema_version": 1,
    "manifest": str(manifest_path.resolve()),
    "sim_config": str(sim_config_path.resolve()),
    "passed": passed,
    "primary_reason": None if passed else "no_deploy_safe_entry_contract",
    "real_robot_unlocked": False,
    "sim_teleport_only_present": any_sim_teleport,
    "sim_teleport_only_note": (
      "A matching initial_qpos is cause-isolation evidence only; it is not "
      "deploy acceptance unless reproduced by the controller without teleporting."
    ),
    "phase2_recommendation": None
    if passed
    else (
      "Run the prepose sim2sim2 candidate first. If it cannot pass, add a stronger "
      "controller transition or train a deployment-aware variant with entry-state "
      "perturbations and handoff robustness."
    ),
    "sim_config_summary": {
      "start_paused": sim_config.get("start_paused"),
      "interface": sim_config.get("interface"),
      "domain_id": sim_config.get("domain_id"),
      "has_initial_qpos": sim_qpos is not None,
    },
    "actions": action_reports,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Gate whether G1 phase-1 entry state is deploy-safe or sim-teleport-only."
  )
  parser.add_argument("--manifest", required=True, type=Path)
  parser.add_argument("--sim-config", type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--max-deploy-entry-gap-l2", default=0.5, type=float)
  parser.add_argument("--max-deploy-entry-gap-max", default=0.35, type=float)
  parser.add_argument("--max-qpos-reference-gap-l2", default=0.05, type=float)
  parser.add_argument("--max-qpos-reference-gap-max", default=0.05, type=float)
  parser.add_argument("--expect-blocked", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    report = analyze_handoff_gate(
      args.manifest,
      sim_config_override=args.sim_config,
      max_deploy_entry_gap_l2=args.max_deploy_entry_gap_l2,
      max_deploy_entry_gap_max=args.max_deploy_entry_gap_max,
      max_qpos_reference_gap_l2=args.max_qpos_reference_gap_l2,
      max_qpos_reference_gap_max=args.max_qpos_reference_gap_max,
    )
  except (HandoffGateError, KeyError) as exc:
    print(str(exc), file=sys.stderr)
    return 2

  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_blocked:
    return 0 if not report["passed"] else 1
  return 0 if report["passed"] else 1


if __name__ == "__main__":
  raise SystemExit(main())
