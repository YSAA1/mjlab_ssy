"""Summarize G1 Velocity policy I/O trace logs."""

from __future__ import annotations

import argparse
import ast
import json
import math
import re
from pathlib import Path
from typing import Any

import yaml

DEFAULT_DEPLOY_ROOT = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1"
)

POLICY_IO_RE = re.compile(
  r"\[PHASE1\]\s+event=policy_io_trace\s+"
  r"step=(?P<step>\d+)\s+obs_dim=(?P<obs_dim>\d+)\s+"
  r"obs_l2=(?P<obs_l2>[-+0-9.eE]+)\s+obs_max=(?P<obs_max>[-+0-9.eE]+)\s+"
  r"raw_action_l2=(?P<raw_action_l2>[-+0-9.eE]+)\s+"
  r"raw_action_max=(?P<raw_action_max>[-+0-9.eE]+)\s+"
  r"processed_action_l2=(?P<processed_action_l2>[-+0-9.eE]+)\s+"
  r"processed_action_max=(?P<processed_action_max>[-+0-9.eE]+)\s+"
  r"joint_pos_l2=(?P<joint_pos_l2>[-+0-9.eE]+)\s+"
  r"joint_vel_l2=(?P<joint_vel_l2>[-+0-9.eE]+)\s+"
  r"phase=(?P<phase>[-+0-9.eE]+)\s+"
  r"obs=(?P<obs>\[[^\]]*\])\s+"
  r"raw_action=(?P<raw_action>\[[^\]]*\])\s+"
  r"processed_action=(?P<processed_action>\[[^\]]*\])\s+"
  r"joint_pos=(?P<joint_pos>\[[^\]]*\])\s+"
  r"joint_vel=(?P<joint_vel>\[[^\]]*\])\s+"
  r"projected_gravity=(?P<projected_gravity>\[[^\]]*\])\s+"
  r"root_ang_vel=(?P<root_ang_vel>\[[^\]]*\])"
)

STABLE_RE = re.compile(
  r"\[PHASE1\]\s+event=stable_sample\s+state=Velocity\s+"
  r"stable=(?P<stable>[01])\s+policy_step=(?P<policy_step>\d+)"
)

KNOWN_DIMS = {
  "base_ang_vel": 3,
  "projected_gravity": 3,
  "velocity_commands": 3,
  "gait_phase": 2,
  "joint_pos_rel": 29,
  "joint_vel_rel": 29,
  "last_action": 29,
}


class PolicyIoTraceReportError(RuntimeError):
  """Raised when policy I/O trace evidence cannot be parsed."""


def _parse_vector(text: str) -> list[float]:
  value = ast.literal_eval(text)
  if not isinstance(value, list):
    raise PolicyIoTraceReportError(
      f"expected vector literal, got {type(value).__name__}"
    )
  return [float(item) for item in value]


def _l2(values: list[float]) -> float:
  return math.sqrt(sum(value * value for value in values))


def _top_abs(values: list[float], *, limit: int = 8) -> list[dict[str, float | int]]:
  ranked = sorted(
    enumerate(values),
    key=lambda item: abs(item[1]),
    reverse=True,
  )[:limit]
  return [
    {
      "index": int(index),
      "value": round(float(value), 6),
      "abs": round(abs(float(value)), 6),
    }
    for index, value in ranked
  ]


def _summary(values: list[float]) -> dict[str, Any]:
  max_abs = max((abs(value) for value in values), default=0.0)
  return {
    "dim": len(values),
    "l2": round(_l2(values), 6),
    "max_abs": round(max_abs, 6),
    "top_abs": _top_abs(values),
  }


def _term_dim(name: str, cfg: dict[str, Any]) -> int:
  scale = cfg.get("scale")
  if isinstance(scale, list):
    return len(scale)
  if name in KNOWN_DIMS:
    return KNOWN_DIMS[name]
  raise PolicyIoTraceReportError(f"cannot infer observation dim for term {name!r}")


def _deploy_terms(deploy_yaml: Path) -> list[dict[str, Any]]:
  data = yaml.safe_load(deploy_yaml.read_text(encoding="utf-8"))
  if not isinstance(data, dict) or not isinstance(data.get("observations"), dict):
    raise PolicyIoTraceReportError(
      f"deploy.yaml missing observations mapping: {deploy_yaml}"
    )
  terms = []
  cursor = 0
  for name, cfg in data["observations"].items():
    if name in {"scale_first", "use_gym_history"}:
      continue
    if not isinstance(cfg, dict):
      raise PolicyIoTraceReportError(f"observation term {name!r} is not a mapping")
    dim = _term_dim(str(name), cfg)
    terms.append({"name": str(name), "start": cursor, "end": cursor + dim, "dim": dim})
    cursor += dim
  return terms


def _slice_terms(obs: list[float], terms: list[dict[str, Any]]) -> dict[str, Any]:
  out = {}
  for term in terms:
    values = obs[int(term["start"]) : int(term["end"])]
    out[str(term["name"])] = {
      **term,
      **_summary(values),
      "values": [round(float(value), 6) for value in values],
    }
  return out


def _resolve_deploy_yaml(
  evidence_dir: Path, deploy_yaml: Path | None, deploy_root: Path
) -> Path:
  if deploy_yaml is not None:
    return deploy_yaml
  selected_config = evidence_dir / "selected/config.yaml"
  data = yaml.safe_load(selected_config.read_text(encoding="utf-8"))
  policy_dir = data.get("FSM", {}).get("Velocity", {}).get("policy_dir")
  if not isinstance(policy_dir, str):
    raise PolicyIoTraceReportError(
      f"cannot infer Velocity.policy_dir from {selected_config}"
    )
  policy_root = Path(policy_dir)
  if not policy_root.is_absolute():
    policy_root = deploy_root / policy_root
  if policy_root.is_dir() and not (policy_root / "params/deploy.yaml").is_file():
    candidates = sorted(child for child in policy_root.iterdir() if child.is_dir())
    for candidate in reversed(candidates):
      if (candidate / "params/deploy.yaml").is_file():
        policy_root = candidate
        break
  return policy_root / "params/deploy.yaml"


def _parse_trace_line(line: str) -> dict[str, Any] | None:
  match = POLICY_IO_RE.search(line)
  if match is None:
    return None
  parsed: dict[str, Any] = {
    "line": None,
    "step": int(match.group("step")),
    "obs_dim": int(match.group("obs_dim")),
    "obs_l2": float(match.group("obs_l2")),
    "obs_max": float(match.group("obs_max")),
    "raw_action_l2": float(match.group("raw_action_l2")),
    "raw_action_max": float(match.group("raw_action_max")),
    "processed_action_l2": float(match.group("processed_action_l2")),
    "processed_action_max": float(match.group("processed_action_max")),
    "joint_pos_l2": float(match.group("joint_pos_l2")),
    "joint_vel_l2": float(match.group("joint_vel_l2")),
    "phase": float(match.group("phase")),
  }
  for key in (
    "obs",
    "raw_action",
    "processed_action",
    "joint_pos",
    "joint_vel",
    "projected_gravity",
    "root_ang_vel",
  ):
    parsed[key] = _parse_vector(match.group(key))
  return parsed


def _first_unstable(lines: list[str]) -> dict[str, Any] | None:
  for line_no, line in enumerate(lines, start=1):
    match = STABLE_RE.search(line)
    if match is not None and match.group("stable") == "0":
      return {"line": line_no, "policy_step": int(match.group("policy_step"))}
  return None


def analyze_policy_io_trace(
  evidence_dir: Path,
  *,
  deploy_yaml: Path | None = None,
  deploy_root: Path = DEFAULT_DEPLOY_ROOT,
) -> dict[str, Any]:
  log = evidence_dir / "g1_ctrl.log"
  lines = log.read_text(encoding="utf-8").splitlines()
  resolved_deploy_yaml = _resolve_deploy_yaml(evidence_dir, deploy_yaml, deploy_root)
  terms = _deploy_terms(resolved_deploy_yaml)
  expected_obs_dim = sum(int(term["dim"]) for term in terms)
  first_unstable = _first_unstable(lines)

  traces = []
  for line_no, line in enumerate(lines, start=1):
    trace = _parse_trace_line(line)
    if trace is None:
      continue
    trace["line"] = line_no
    trace["obs_terms"] = _slice_terms(trace["obs"], terms)
    trace["raw_action_summary"] = _summary(trace["raw_action"])
    trace["processed_action_summary"] = _summary(trace["processed_action"])
    trace["joint_pos_summary"] = _summary(trace["joint_pos"])
    trace["joint_vel_summary"] = _summary(trace["joint_vel"])
    traces.append(trace)

  selected_trace = None
  if traces and first_unstable is not None:
    selected_trace = min(
      traces,
      key=lambda item: abs(int(item["step"]) - int(first_unstable["policy_step"])),
    )
  elif traces:
    selected_trace = traces[-1]
  selected_step_delta = (
    None
    if selected_trace is None or first_unstable is None
    else abs(int(selected_trace["step"]) - int(first_unstable["policy_step"]))
  )
  first_unstable_has_nearby_trace = (
    selected_step_delta is not None and selected_step_delta <= 25
  )

  return {
    "schema_version": 1,
    "evidence_dir": str(evidence_dir),
    "log": str(log),
    "deploy_yaml": str(resolved_deploy_yaml),
    "expected_obs_dim": expected_obs_dim,
    "observation_terms": terms,
    "first_unstable": first_unstable,
    "trace_count": len(traces),
    "obs_dim_matches": all(trace["obs_dim"] == expected_obs_dim for trace in traces),
    "selected_step_delta": selected_step_delta,
    "selected_trace": selected_trace,
    "traces": traces,
    "decision": {
      "has_policy_io_trace": bool(traces),
      "first_unstable_has_nearby_trace": first_unstable_has_nearby_trace,
    },
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Summarize G1 Velocity policy I/O trace logs from a sim2sim evidence directory."
  )
  parser.add_argument("--evidence-dir", required=True, type=Path)
  parser.add_argument("--deploy-yaml", type=Path)
  parser.add_argument("--deploy-root", type=Path, default=DEFAULT_DEPLOY_ROOT)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-trace", action="store_true")
  parser.add_argument("--expect-near-first-unstable", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = analyze_policy_io_trace(
    args.evidence_dir,
    deploy_yaml=args.deploy_yaml,
    deploy_root=args.deploy_root,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.report_out is not None:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(text + "\n", encoding="utf-8")
  print(text)
  if args.expect_trace and not report["decision"]["has_policy_io_trace"]:
    return 1
  if (
    args.expect_near_first_unstable
    and not report["decision"]["first_unstable_has_nearby_trace"]
  ):
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
