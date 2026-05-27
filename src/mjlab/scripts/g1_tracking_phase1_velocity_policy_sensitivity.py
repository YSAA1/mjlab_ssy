"""Probe G1 Velocity policy sensitivity to deploy observation terms."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

DEFAULT_POLICY_ROOT = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0"
)
DEFAULT_MAGNITUDES = {
  "base_ang_vel": 1.0,
  "projected_gravity": 0.25,
  "velocity_commands": 0.5,
  "gait_phase": 1.0,
  "joint_pos_rel": 0.25,
  "joint_vel_rel": 10.0,
  "last_action": 1.0,
}


class PolicySensitivityError(RuntimeError):
  """Raised when a policy sensitivity probe cannot be built safely."""


@dataclass(frozen=True)
class ObservationTerm:
  """One deploy observation term and its final model-input slice."""

  name: str
  dim: int
  start: int
  stop: int
  scale: np.ndarray
  clip: tuple[float, float] | None
  scale_first: bool


def _load_yaml(path: Path) -> dict[str, Any]:
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise PolicySensitivityError(f"YAML root must be a mapping: {path}")
  return data


def _as_float_array(value: Any, *, name: str) -> np.ndarray:
  if not isinstance(value, list):
    raise PolicySensitivityError(f"{name} must be a list")
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
  raise PolicySensitivityError(
    f"Unsupported observation term without scale: {term_name}"
  )


def _observation_terms(
  deploy_cfg: dict[str, Any],
  *,
  action_dim: int,
) -> list[ObservationTerm]:
  observations = deploy_cfg.get("observations")
  if not isinstance(observations, dict):
    raise PolicySensitivityError("deploy.yaml observations must be a mapping")

  terms: list[ObservationTerm] = []
  cursor = 0
  scale_first = False
  for term_name, raw_term_cfg in observations.items():
    if term_name == "scale_first":
      scale_first = bool(raw_term_cfg)
      continue
    if term_name == "use_gym_history":
      continue
    if not isinstance(raw_term_cfg, dict):
      raise PolicySensitivityError(f"observation term must be a mapping: {term_name}")
    dim = _obs_term_dim(term_name, raw_term_cfg, action_dim)
    scale = raw_term_cfg.get("scale")
    if isinstance(scale, list):
      scale_array = np.asarray(scale, dtype=np.float32)
      if scale_array.size != dim:
        raise PolicySensitivityError(
          f"observation term {term_name} scale dim {scale_array.size} != {dim}"
        )
    else:
      scale_array = np.ones(dim, dtype=np.float32)
    clip = raw_term_cfg.get("clip")
    clip_pair: tuple[float, float] | None = None
    if isinstance(clip, list) and len(clip) == 2:
      clip_pair = (float(clip[0]), float(clip[1]))
    terms.append(
      ObservationTerm(
        name=term_name,
        dim=dim,
        start=cursor,
        stop=cursor + dim,
        scale=scale_array,
        clip=clip_pair,
        scale_first=scale_first,
      )
    )
    cursor += dim
  if not terms:
    raise PolicySensitivityError("no observation terms found")
  return terms


def _apply_term_transform(values: np.ndarray, term: ObservationTerm) -> np.ndarray:
  out = values.astype(np.float32, copy=True)
  if term.scale_first:
    out = out * term.scale
    if term.clip is not None:
      out = np.clip(out, term.clip[0], term.clip[1])
  else:
    if term.clip is not None:
      out = np.clip(out, term.clip[0], term.clip[1])
    out = out * term.scale
  return out.astype(np.float32)


def _base_term_values(
  terms: list[ObservationTerm],
  *,
  last_action: np.ndarray,
) -> dict[str, np.ndarray]:
  values: dict[str, np.ndarray] = {}
  for term in terms:
    if term.name == "projected_gravity":
      raw = np.asarray([0.0, 0.0, -1.0], dtype=np.float32)
    elif term.name == "last_action":
      raw = last_action.astype(np.float32, copy=True)
    else:
      raw = np.zeros(term.dim, dtype=np.float32)
    if raw.size != term.dim:
      raise PolicySensitivityError(
        f"observation term {term.name} has dim {raw.size}, expected {term.dim}"
      )
    values[term.name] = raw
  return values


def _build_observation(
  terms: list[ObservationTerm],
  values: dict[str, np.ndarray],
) -> np.ndarray:
  pieces = []
  for term in terms:
    raw = values.get(term.name)
    if raw is None:
      raise PolicySensitivityError(f"missing value for observation term {term.name}")
    if raw.size != term.dim:
      raise PolicySensitivityError(
        f"observation term {term.name} value dim {raw.size} != {term.dim}"
      )
    pieces.append(_apply_term_transform(raw, term))
  return np.concatenate(pieces).astype(np.float32)


def _top_abs(values: np.ndarray, *, limit: int = 8) -> list[dict[str, Any]]:
  order = np.argsort(np.abs(values))[::-1][:limit]
  return [
    {
      "index": int(index),
      "value": round(float(values[index]), 6),
    }
    for index in order
  ]


def _policy_output(
  session: Any,
  input_name: str,
  obs: np.ndarray,
) -> np.ndarray:
  return session.run(None, {input_name: obs.reshape(1, -1)})[0][0].astype(np.float32)


def _action_report(
  raw_action: np.ndarray,
  *,
  action_scale: np.ndarray,
  action_offset: np.ndarray,
  default_joint_pos: np.ndarray,
) -> dict[str, Any]:
  processed_target = raw_action * action_scale + action_offset
  target_gap = processed_target - default_joint_pos
  return {
    "raw_action_l2": round(float(np.linalg.norm(raw_action)), 6),
    "raw_action_max": round(float(np.max(np.abs(raw_action))), 6),
    "processed_target_gap_l2": round(float(np.linalg.norm(target_gap)), 6),
    "processed_target_gap_max": round(float(np.max(np.abs(target_gap))), 6),
    "top_raw_actions": _top_abs(raw_action),
    "top_processed_target_gaps": _top_abs(target_gap),
  }


def _parse_magnitude_overrides(items: list[str]) -> dict[str, float]:
  overrides: dict[str, float] = {}
  for item in items:
    if "=" not in item:
      raise PolicySensitivityError(
        f"magnitude override must use TERM=VALUE format: {item}"
      )
    key, raw_value = item.split("=", 1)
    try:
      overrides[key.strip()] = float(raw_value)
    except ValueError as exc:
      raise PolicySensitivityError(f"invalid magnitude override: {item}") from exc
  return overrides


def probe_policy_sensitivity(
  *,
  policy_root: Path,
  warmup_steps: int = 5,
  magnitudes: dict[str, float] | None = None,
  top_k: int = 12,
) -> dict[str, Any]:
  deploy_yaml = policy_root / "params/deploy.yaml"
  policy_onnx = policy_root / "exported/policy.onnx"
  deploy_cfg = _load_yaml(deploy_yaml)
  default_joint_pos = _as_float_array(
    deploy_cfg.get("default_joint_pos"), name="default_joint_pos"
  )
  action_cfg = deploy_cfg.get("actions", {}).get("JointPositionAction", {})
  if not isinstance(action_cfg, dict):
    raise PolicySensitivityError("actions.JointPositionAction must be a mapping")
  action_scale = _as_float_array(action_cfg.get("scale"), name="action scale")
  action_offset = _as_float_array(action_cfg.get("offset"), name="action offset")
  if not (len(default_joint_pos) == len(action_scale) == len(action_offset)):
    raise PolicySensitivityError(
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
  terms = _observation_terms(deploy_cfg, action_dim=len(action_offset))
  term_names = {term.name for term in terms}
  unknown_magnitudes = sorted((magnitudes or {}).keys() - term_names)
  if unknown_magnitudes:
    raise PolicySensitivityError(
      f"magnitude override references unknown observation terms: {unknown_magnitudes}"
    )
  last_action = np.zeros(len(action_offset), dtype=np.float32)

  warmup_reports: list[dict[str, Any]] = []
  for step in range(warmup_steps):
    values = _base_term_values(terms, last_action=last_action)
    obs = _build_observation(terms, values)
    raw_action = _policy_output(session, input_meta.name, obs)
    warmup_reports.append(
      {
        "step": step,
        **_action_report(
          raw_action,
          action_scale=action_scale,
          action_offset=action_offset,
          default_joint_pos=default_joint_pos,
        ),
      }
    )
    last_action = raw_action

  base_values = _base_term_values(terms, last_action=last_action)
  base_obs = _build_observation(terms, base_values)
  base_raw_action = _policy_output(session, input_meta.name, base_obs)
  base_report = _action_report(
    base_raw_action,
    action_scale=action_scale,
    action_offset=action_offset,
    default_joint_pos=default_joint_pos,
  )

  effective_magnitudes = dict(DEFAULT_MAGNITUDES)
  if magnitudes:
    effective_magnitudes.update(magnitudes)

  cases: list[dict[str, Any]] = []
  for term in terms:
    magnitude = effective_magnitudes.get(term.name, 1.0)
    for index in range(term.dim):
      for sign in (-1.0, 1.0):
        values = {
          name: value.astype(np.float32, copy=True)
          for name, value in base_values.items()
        }
        values[term.name][index] += sign * magnitude
        obs = _build_observation(terms, values)
        raw_action = _policy_output(session, input_meta.name, obs)
        delta_action = raw_action - base_raw_action
        action_report = _action_report(
          raw_action,
          action_scale=action_scale,
          action_offset=action_offset,
          default_joint_pos=default_joint_pos,
        )
        cases.append(
          {
            "term": term.name,
            "index": index,
            "signed_magnitude": round(float(sign * magnitude), 6),
            "obs_delta_l2": round(float(np.linalg.norm(obs - base_obs)), 6),
            "delta_raw_action_l2": round(float(np.linalg.norm(delta_action)), 6),
            "delta_raw_action_max": round(float(np.max(np.abs(delta_action))), 6),
            **action_report,
          }
        )

  cases.sort(
    key=lambda item: (
      float(item["processed_target_gap_l2"]),
      float(item["delta_raw_action_l2"]),
    ),
    reverse=True,
  )
  term_summary: list[dict[str, Any]] = []
  for term in terms:
    term_cases = [case for case in cases if case["term"] == term.name]
    if not term_cases:
      continue
    worst = term_cases[0]
    term_summary.append(
      {
        "term": term.name,
        "dim": term.dim,
        "magnitude": round(float(effective_magnitudes.get(term.name, 1.0)), 6),
        "max_processed_target_gap_l2": worst["processed_target_gap_l2"],
        "max_raw_action_l2": worst["raw_action_l2"],
        "max_delta_raw_action_l2": worst["delta_raw_action_l2"],
        "worst_index": worst["index"],
        "worst_signed_magnitude": worst["signed_magnitude"],
      }
    )
  term_summary.sort(
    key=lambda item: (
      float(item["max_processed_target_gap_l2"]),
      float(item["max_delta_raw_action_l2"]),
    ),
    reverse=True,
  )
  highest_term = term_summary[0]["term"] if term_summary else None
  baseline_gap = float(base_report["processed_target_gap_l2"])
  worst_gap = float(cases[0]["processed_target_gap_l2"]) if cases else baseline_gap
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
    "warmup_steps": warmup_reports,
    "baseline_after_warmup": base_report,
    "observation_terms": [
      {
        "name": term.name,
        "dim": term.dim,
        "start": term.start,
        "stop": term.stop,
      }
      for term in terms
    ],
    "term_summary": term_summary,
    "top_cases": cases[:top_k],
    "decision": {
      "highest_sensitivity_term": highest_term,
      "baseline_processed_target_gap_l2": round(baseline_gap, 6),
      "worst_processed_target_gap_l2": round(worst_gap, 6),
      "policy_can_amplify_deploy_observation_perturbations": bool(
        worst_gap > max(baseline_gap * 1.5, baseline_gap + 0.5)
      ),
      "real_robot_gate": "locked",
    },
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Probe a G1 Velocity ONNX policy's sensitivity to deploy observation terms."
  )
  parser.add_argument("--policy-root", type=Path, default=DEFAULT_POLICY_ROOT)
  parser.add_argument("--warmup-steps", type=int, default=5)
  parser.add_argument(
    "--magnitude",
    action="append",
    default=[],
    metavar="TERM=VALUE",
    help="Override a perturbation magnitude, e.g. joint_vel_rel=100.",
  )
  parser.add_argument("--top-k", type=int, default=12)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-sensitive", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = probe_policy_sensitivity(
    policy_root=args.policy_root,
    warmup_steps=args.warmup_steps,
    magnitudes=_parse_magnitude_overrides(args.magnitude),
    top_k=args.top_k,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_sensitive:
    return (
      0
      if report.get("decision", {}).get(
        "policy_can_amplify_deploy_observation_perturbations"
      )
      else 1
    )
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
