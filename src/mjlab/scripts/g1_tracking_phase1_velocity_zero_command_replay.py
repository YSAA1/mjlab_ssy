"""Replay the active G1 Velocity ONNX policy on zero-command deploy observations."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

DEFAULT_POLICY_ROOT = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0"
)


class ZeroCommandReplayError(RuntimeError):
  """Raised when the zero-command replay inputs are invalid."""


def _load_yaml(path: Path) -> dict[str, Any]:
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise ZeroCommandReplayError(f"YAML root must be a mapping: {path}")
  return data


def _as_float_array(value: Any, *, name: str) -> np.ndarray:
  if not isinstance(value, list):
    raise ZeroCommandReplayError(f"{name} must be a list")
  return np.asarray(value, dtype=np.float32)


def _obs_term_dim(term_name: str, term_cfg: dict[str, Any], action_dim: int) -> int:
  scale = term_cfg.get("scale")
  if isinstance(scale, list):
    return len(scale)
  if term_name in {"base_ang_vel", "projected_gravity", "velocity_commands"}:
    return 3
  if term_name == "gait_phase":
    return 2
  if term_name in {"joint_pos_rel", "joint_vel_rel", "last_action"}:
    return action_dim
  raise ZeroCommandReplayError(
    f"Unsupported observation term without scale: {term_name}"
  )


def _zero_command_observation(
  deploy_cfg: dict[str, Any],
  *,
  last_action: np.ndarray,
) -> np.ndarray:
  action_dim = int(last_action.size)
  observations = deploy_cfg.get("observations")
  if not isinstance(observations, dict):
    raise ZeroCommandReplayError("deploy.yaml observations must be a mapping")

  values: list[np.ndarray] = []
  for term_name, raw_term_cfg in observations.items():
    if term_name in {"scale_first", "use_gym_history"}:
      continue
    if not isinstance(raw_term_cfg, dict):
      raise ZeroCommandReplayError(f"observation term must be a mapping: {term_name}")
    dim = _obs_term_dim(term_name, raw_term_cfg, action_dim)
    if term_name == "projected_gravity":
      term = np.asarray([0.0, 0.0, -1.0], dtype=np.float32)
    elif term_name == "last_action":
      term = last_action.astype(np.float32, copy=True)
    else:
      # At zero velocity command, deploy gait_phase increments internally but
      # returns [0, 0], so all remaining terms are zero in the nominal default pose.
      term = np.zeros(dim, dtype=np.float32)
    if term.size != dim:
      raise ZeroCommandReplayError(
        f"observation term {term_name} has dim {term.size}, expected {dim}"
      )
    values.append(term)
  if not values:
    raise ZeroCommandReplayError("no observation terms found")
  return np.concatenate(values).astype(np.float32)


def _top_abs(values: np.ndarray, *, limit: int = 8) -> list[dict[str, Any]]:
  order = np.argsort(np.abs(values))[::-1][:limit]
  return [
    {
      "index": int(index),
      "value": round(float(values[index]), 6),
    }
    for index in order
  ]


def replay_zero_command(
  *,
  policy_root: Path,
  steps: int = 5,
) -> dict[str, Any]:
  deploy_yaml = policy_root / "params/deploy.yaml"
  policy_onnx = policy_root / "exported/policy.onnx"
  deploy_cfg = _load_yaml(deploy_yaml)
  default_joint_pos = _as_float_array(
    deploy_cfg.get("default_joint_pos"), name="default_joint_pos"
  )
  action_cfg = deploy_cfg.get("actions", {}).get("JointPositionAction", {})
  if not isinstance(action_cfg, dict):
    raise ZeroCommandReplayError("actions.JointPositionAction must be a mapping")
  action_scale = _as_float_array(action_cfg.get("scale"), name="action scale")
  action_offset = _as_float_array(action_cfg.get("offset"), name="action offset")
  if not (len(default_joint_pos) == len(action_scale) == len(action_offset)):
    raise ZeroCommandReplayError(
      "default_joint_pos, action scale, and offset dims differ"
    )

  try:
    import onnxruntime as ort
  except Exception as exc:  # pragma: no cover - depends on local runtime install.
    return {
      "available": False,
      "reason": f"onnxruntime_import_failed: {exc}",
      "policy_root": str(policy_root),
    }

  session = ort.InferenceSession(str(policy_onnx), providers=["CPUExecutionProvider"])
  input_meta = session.get_inputs()[0]
  output_meta = session.get_outputs()[0]
  last_action = np.zeros(len(action_offset), dtype=np.float32)
  step_reports: list[dict[str, Any]] = []

  for step in range(steps):
    obs = _zero_command_observation(deploy_cfg, last_action=last_action)
    raw_action = session.run(None, {input_meta.name: obs.reshape(1, -1)})[0][0].astype(
      np.float32
    )
    processed_target = raw_action * action_scale + action_offset
    target_gap = processed_target - default_joint_pos
    step_reports.append(
      {
        "step": step,
        "obs_dim": int(obs.size),
        "raw_action_l2": round(float(np.linalg.norm(raw_action)), 6),
        "raw_action_max": round(float(np.max(np.abs(raw_action))), 6),
        "processed_target_gap_l2": round(float(np.linalg.norm(target_gap)), 6),
        "processed_target_gap_max": round(float(np.max(np.abs(target_gap))), 6),
        "top_raw_actions": _top_abs(raw_action),
        "top_processed_target_gaps": _top_abs(target_gap),
      }
    )
    last_action = raw_action

  max_target_gap_l2 = max(
    (item["processed_target_gap_l2"] for item in step_reports), default=0.0
  )
  return {
    "schema_version": 1,
    "available": True,
    "policy_root": str(policy_root),
    "deploy_yaml": str(deploy_yaml),
    "policy_onnx": str(policy_onnx),
    "input_name": input_meta.name,
    "input_shape": list(input_meta.shape),
    "output_name": output_meta.name,
    "output_shape": list(output_meta.shape),
    "steps": step_reports,
    "max_processed_target_gap_l2": round(float(max_target_gap_l2), 6),
    "zero_command_target_is_default": math.isclose(
      max_target_gap_l2, 0.0, abs_tol=1e-6
    ),
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay a G1 Velocity ONNX policy on zero-command default-pose observations."
  )
  parser.add_argument("--policy-root", type=Path, default=DEFAULT_POLICY_ROOT)
  parser.add_argument("--steps", type=int, default=5)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument(
    "--expect-nonzero-target-gap",
    action="store_true",
    help="Exit 0 only when the replay finds a processed target gap above threshold.",
  )
  parser.add_argument("--target-gap-threshold", type=float, default=0.5)
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = replay_zero_command(policy_root=args.policy_root, steps=args.steps)
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_nonzero_target_gap:
    return 0 if report["max_processed_target_gap_l2"] > args.target_gap_threshold else 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
