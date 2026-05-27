"""Replay a captured G1 Velocity deploy trace through the packaged ONNX."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml


class VelocityTraceReplayError(RuntimeError):
  """Raised when a captured deploy trace cannot be replayed safely."""


def _load_json(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise VelocityTraceReplayError(f"JSON root must be a mapping: {path}")
  return data


def _load_yaml(path: Path) -> dict[str, Any]:
  data = yaml.safe_load(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise VelocityTraceReplayError(f"YAML root must be a mapping: {path}")
  return data


def _as_array(value: Any, *, name: str) -> np.ndarray:
  if not isinstance(value, list):
    raise VelocityTraceReplayError(f"{name} must be a list")
  return np.asarray(value, dtype=np.float32)


def _top_abs(values: np.ndarray, *, limit: int = 8) -> list[dict[str, Any]]:
  order = np.argsort(np.abs(values))[::-1][:limit]
  return [
    {
      "index": int(index),
      "value": round(float(values[index]), 6),
      "abs": round(abs(float(values[index])), 6),
    }
    for index in order
  ]


def _summary(values: np.ndarray) -> dict[str, Any]:
  return {
    "dim": int(values.size),
    "l2": round(float(np.linalg.norm(values)), 6),
    "max_abs": round(float(np.max(np.abs(values))) if values.size else 0.0, 6),
    "top_abs": _top_abs(values),
  }


def _infer_policy_root(trace_report: dict[str, Any], explicit: Path | None) -> Path:
  if explicit is not None:
    return explicit
  deploy_yaml = Path(str(trace_report.get("deploy_yaml", "")))
  if deploy_yaml.name != "deploy.yaml" or deploy_yaml.parent.name != "params":
    raise VelocityTraceReplayError(
      "cannot infer policy root from trace report deploy_yaml"
    )
  return deploy_yaml.parent.parent


def _action_cfg(deploy_cfg: dict[str, Any]) -> dict[str, Any]:
  cfg = deploy_cfg.get("actions", {}).get("JointPositionAction")
  if not isinstance(cfg, dict):
    raise VelocityTraceReplayError("actions.JointPositionAction must be a mapping")
  return cfg


def _processed_action(raw_action: np.ndarray, deploy_cfg: dict[str, Any]) -> np.ndarray:
  cfg = _action_cfg(deploy_cfg)
  scale = _as_array(cfg.get("scale"), name="action scale")
  offset = _as_array(cfg.get("offset"), name="action offset")
  if raw_action.size != scale.size or raw_action.size != offset.size:
    raise VelocityTraceReplayError(
      "raw action, action scale, and action offset dimensions differ"
    )
  processed = raw_action * scale + offset
  clip = cfg.get("clip")
  if isinstance(clip, list):
    clip_array = np.asarray(clip, dtype=np.float32)
    if clip_array.shape != (raw_action.size, 2):
      raise VelocityTraceReplayError("action clip must have shape [action_dim, 2]")
    processed = np.clip(processed, clip_array[:, 0], clip_array[:, 1])
  return processed.astype(np.float32)


def _run_onnx(policy_onnx: Path, obs: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
  try:
    import onnxruntime as ort
  except Exception as exc:  # pragma: no cover - depends on local runtime install.
    raise VelocityTraceReplayError(f"onnxruntime_import_failed: {exc}") from exc

  session = ort.InferenceSession(str(policy_onnx), providers=["CPUExecutionProvider"])
  input_meta = session.get_inputs()[0]
  output_meta = session.get_outputs()[0]
  raw_action = session.run(None, {input_meta.name: obs.reshape(1, -1)})[0][0]
  return raw_action.astype(np.float32), {
    "input_name": input_meta.name,
    "input_shape": list(input_meta.shape),
    "output_name": output_meta.name,
    "output_shape": list(output_meta.shape),
  }


def _term_slice(terms: list[dict[str, Any]], name: str) -> tuple[int, int] | None:
  for term in terms:
    if term.get("name") == name:
      return int(term["start"]), int(term["end"])
  return None


def _variant_obs(
  obs: np.ndarray,
  terms: list[dict[str, Any]],
  *,
  zero_terms: tuple[str, ...] = (),
  upright_gravity: bool = False,
) -> np.ndarray:
  out = obs.astype(np.float32, copy=True)
  for name in zero_terms:
    span = _term_slice(terms, name)
    if span is not None:
      out[span[0] : span[1]] = 0.0
  if upright_gravity:
    span = _term_slice(terms, "projected_gravity")
    if span is not None and span[1] - span[0] == 3:
      out[span[0] : span[1]] = np.asarray([0.0, 0.0, -1.0], dtype=np.float32)
  return out


def _replay_variant(
  *,
  name: str,
  obs: np.ndarray,
  baseline_raw: np.ndarray,
  policy_onnx: Path,
  deploy_cfg: dict[str, Any],
) -> dict[str, Any]:
  raw_action, _ = _run_onnx(policy_onnx, obs)
  processed = _processed_action(raw_action, deploy_cfg)
  return {
    "name": name,
    "obs": _summary(obs),
    "raw_action": _summary(raw_action),
    "processed_action": _summary(processed),
    "raw_delta_from_selected_l2": round(
      float(np.linalg.norm(raw_action - baseline_raw)), 6
    ),
  }


def replay_selected_trace(
  *,
  trace_report: Path,
  policy_root: Path | None = None,
  replay_tolerance: float = 1e-4,
) -> dict[str, Any]:
  report = _load_json(trace_report)
  selected = report.get("selected_trace")
  if not isinstance(selected, dict):
    raise VelocityTraceReplayError("trace report missing selected_trace")
  obs = _as_array(selected.get("obs"), name="selected_trace.obs")
  logged_raw = _as_array(selected.get("raw_action"), name="selected_trace.raw_action")
  logged_processed = _as_array(
    selected.get("processed_action"), name="selected_trace.processed_action"
  )
  terms = report.get("observation_terms")
  if not isinstance(terms, list):
    raise VelocityTraceReplayError("trace report missing observation_terms")

  resolved_policy_root = _infer_policy_root(report, policy_root)
  deploy_yaml = resolved_policy_root / "params/deploy.yaml"
  policy_onnx = resolved_policy_root / "exported/policy.onnx"
  deploy_cfg = _load_yaml(deploy_yaml)
  replayed_raw, onnx_meta = _run_onnx(policy_onnx, obs)
  replayed_processed = _processed_action(replayed_raw, deploy_cfg)
  raw_gap = replayed_raw - logged_raw
  processed_gap = replayed_processed - logged_processed

  variants = [
    _replay_variant(
      name="selected_without_joint_vel",
      obs=_variant_obs(obs, terms, zero_terms=("joint_vel_rel",)),
      baseline_raw=replayed_raw,
      policy_onnx=policy_onnx,
      deploy_cfg=deploy_cfg,
    ),
    _replay_variant(
      name="selected_without_last_action",
      obs=_variant_obs(obs, terms, zero_terms=("last_action",)),
      baseline_raw=replayed_raw,
      policy_onnx=policy_onnx,
      deploy_cfg=deploy_cfg,
    ),
    _replay_variant(
      name="selected_upright_zero_motion",
      obs=_variant_obs(
        obs,
        terms,
        zero_terms=("base_ang_vel", "joint_vel_rel"),
        upright_gravity=True,
      ),
      baseline_raw=replayed_raw,
      policy_onnx=policy_onnx,
      deploy_cfg=deploy_cfg,
    ),
  ]

  raw_gap_l2 = float(np.linalg.norm(raw_gap))
  raw_gap_max = float(np.max(np.abs(raw_gap))) if raw_gap.size else 0.0
  processed_gap_l2 = float(np.linalg.norm(processed_gap))
  processed_gap_max = (
    float(np.max(np.abs(processed_gap))) if processed_gap.size else 0.0
  )
  return {
    "schema_version": 1,
    "trace_report": str(trace_report),
    "policy_root": str(resolved_policy_root),
    "deploy_yaml": str(deploy_yaml),
    "policy_onnx": str(policy_onnx),
    "onnx": onnx_meta,
    "selected_trace": {
      "line": selected.get("line"),
      "step": selected.get("step"),
      "obs_dim": int(obs.size),
      "obs": _summary(obs),
      "logged_raw_action": _summary(logged_raw),
      "logged_processed_action": _summary(logged_processed),
    },
    "replay": {
      "raw_action": _summary(replayed_raw),
      "processed_action": _summary(replayed_processed),
      "raw_action_gap_l2": round(raw_gap_l2, 8),
      "raw_action_gap_max": round(raw_gap_max, 8),
      "processed_action_gap_l2": round(processed_gap_l2, 8),
      "processed_action_gap_max": round(processed_gap_max, 8),
    },
    "counterfactuals": variants,
    "decision": {
      "replay_matches_deploy_log": bool(
        raw_gap_l2 <= replay_tolerance and processed_gap_l2 <= replay_tolerance
      ),
      "replay_tolerance": replay_tolerance,
    },
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Replay a captured G1 Velocity deploy trace through ONNX."
  )
  parser.add_argument("--trace-report", required=True, type=Path)
  parser.add_argument("--policy-root", type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--replay-tolerance", type=float, default=1e-4)
  parser.add_argument("--expect-replay-match", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = replay_selected_trace(
    trace_report=args.trace_report,
    policy_root=args.policy_root,
    replay_tolerance=args.replay_tolerance,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_replay_match and not report["decision"]["replay_matches_deploy_log"]:
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
