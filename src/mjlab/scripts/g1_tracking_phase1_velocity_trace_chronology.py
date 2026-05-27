"""Analyze chronology in a captured G1 Velocity deploy policy I/O trace."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

DEFAULT_WORKTREE_MANAGER = Path("src/mjlab/envs/manager_based_rl_env.py")
DEFAULT_WORKTREE_OBSERVATIONS = Path("src/mjlab/envs/mdp/observations.py")
DEFAULT_DEPLOY_MANAGER = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/"
  "deploy/include/isaaclab/envs/manager_based_rl_env.h"
)
DEFAULT_DEPLOY_OBSERVATIONS = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/"
  "deploy/include/isaaclab/envs/mdp/observations/observations.h"
)


class VelocityTraceChronologyError(RuntimeError):
  """Raised when trace chronology evidence cannot be analyzed safely."""


def _load_json(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise VelocityTraceChronologyError(f"JSON root must be a mapping: {path}")
  return data


def _as_float_list(value: Any, *, name: str) -> list[float]:
  if not isinstance(value, list):
    raise VelocityTraceChronologyError(f"{name} must be a list")
  return [float(item) for item in value]


def _l2(values: list[float]) -> float:
  return math.sqrt(sum(value * value for value in values))


def _gap_l2(left: list[float], right: list[float]) -> float:
  if len(left) != len(right):
    raise VelocityTraceChronologyError(
      f"dimension mismatch: {len(left)} != {len(right)}"
    )
  return _l2([a - b for a, b in zip(left, right, strict=True)])


def _term_values(trace: dict[str, Any], name: str) -> list[float]:
  terms = trace.get("obs_terms")
  if not isinstance(terms, dict) or not isinstance(terms.get(name), dict):
    raise VelocityTraceChronologyError(f"trace missing obs term {name!r}")
  return _as_float_list(terms[name].get("values"), name=f"obs_terms.{name}.values")


def _term_l2(trace: dict[str, Any], name: str) -> float:
  return _l2(_term_values(trace, name))


def _gravity_drift_l2(trace: dict[str, Any]) -> float:
  gravity = _term_values(trace, "projected_gravity")
  if len(gravity) != 3:
    raise VelocityTraceChronologyError("projected_gravity must have dim 3")
  return _gap_l2(gravity, [0.0, 0.0, -1.0])


def _root_ang_vel_l2(trace: dict[str, Any]) -> float:
  if "root_ang_vel" in trace:
    return _l2(_as_float_list(trace["root_ang_vel"], name="root_ang_vel"))
  return _term_l2(trace, "base_ang_vel")


def _trace_summary(trace: dict[str, Any]) -> dict[str, Any]:
  return {
    "line": trace.get("line"),
    "step": int(trace["step"]),
    "base_ang_vel_l2": round(_term_l2(trace, "base_ang_vel"), 6),
    "root_ang_vel_l2": round(_root_ang_vel_l2(trace), 6),
    "projected_gravity_drift_l2": round(_gravity_drift_l2(trace), 6),
    "velocity_commands_l2": round(_term_l2(trace, "velocity_commands"), 6),
    "gait_phase_l2": round(_term_l2(trace, "gait_phase"), 6),
    "joint_pos_rel_l2": round(_term_l2(trace, "joint_pos_rel"), 6),
    "joint_vel_rel_l2": round(_term_l2(trace, "joint_vel_rel"), 6),
    "last_action_l2": round(_term_l2(trace, "last_action"), 6),
    "raw_action_l2": round(float(trace.get("raw_action_l2", 0.0)), 6),
    "processed_action_l2": round(float(trace.get("processed_action_l2", 0.0)), 6),
  }


def _first_crossing(
  traces: list[dict[str, Any]],
  *,
  name: str,
  threshold: float,
  value_fn,
) -> dict[str, Any] | None:
  for trace in traces:
    value = float(value_fn(trace))
    if value > threshold:
      return {
        "name": name,
        "threshold": threshold,
        "line": trace.get("line"),
        "step": int(trace["step"]),
        "value": round(value, 6),
      }
  return None


def _early_last_action_pairs(
  traces: list[dict[str, Any]], *, tolerance: float
) -> tuple[list[dict[str, Any]], bool, float | None]:
  pairs: list[dict[str, Any]] = []
  for prev, cur in zip(traces, traces[1:], strict=False):
    prev_step = int(prev["step"])
    cur_step = int(cur["step"])
    if cur_step - prev_step != 1:
      continue
    current_last_action = _term_values(cur, "last_action")
    prev_raw_action = _as_float_list(prev.get("raw_action"), name="raw_action")
    gap = _gap_l2(current_last_action, prev_raw_action)
    pairs.append(
      {
        "previous_step": prev_step,
        "current_step": cur_step,
        "current_last_action_l2": round(_l2(current_last_action), 6),
        "previous_raw_action_l2": round(_l2(prev_raw_action), 6),
        "gap_l2": round(gap, 9),
        "matches": gap <= tolerance,
      }
    )
  max_gap = max((float(pair["gap_l2"]) for pair in pairs), default=None)
  return pairs, bool(pairs) and all(bool(pair["matches"]) for pair in pairs), max_gap


def _source_audit(
  *,
  worktree_manager: Path,
  worktree_observations: Path,
  deploy_manager: Path,
  deploy_observations: Path,
) -> dict[str, Any]:
  paths = {
    "worktree_manager": worktree_manager,
    "worktree_observations": worktree_observations,
    "deploy_manager": deploy_manager,
    "deploy_observations": deploy_observations,
  }
  text = {}
  for key, path in paths.items():
    if not path.exists():
      text[key] = ""
    else:
      text[key] = path.read_text(encoding="utf-8")

  deploy_compute = text["deploy_manager"].find(
    "auto obs = observation_manager->compute();"
  )
  deploy_act = text["deploy_manager"].find("auto action = alg->act(obs);")
  deploy_process = text["deploy_manager"].find(
    "action_manager->process_action(action);"
  )
  worktree_process = text["worktree_manager"].find(
    "self.action_manager.process_action(action.to(self.device))"
  )
  worktree_obs = text["worktree_manager"].find(
    "pre_reset_obs = self.observation_manager.compute"
  )

  return {
    "paths": {key: str(path) for key, path in paths.items()},
    "deploy_last_action_reads_raw_action": (
      "REGISTER_OBSERVATION(last_action)" in text["deploy_observations"]
      and "env->action_manager->action()" in text["deploy_observations"]
    ),
    "deploy_policy_call_observes_previous_raw_action": (
      -1 not in {deploy_compute, deploy_act, deploy_process}
      and deploy_compute < deploy_act < deploy_process
    ),
    "worktree_last_action_reads_raw_action": (
      "def last_action" in text["worktree_observations"]
      and "return env.action_manager.action" in text["worktree_observations"]
    ),
    "worktree_returned_obs_contains_just_processed_raw_action": (
      worktree_process != -1 and worktree_obs != -1 and worktree_process < worktree_obs
    ),
    "interpretation": (
      "At a policy call, deploy last_action should contain the previous raw "
      "policy action. In mjlab training/play, env.step(action_t) returns an "
      "observation containing action_t; the next policy call therefore also "
      "sees the previous raw policy action."
    ),
  }


def analyze_trace_chronology(
  *,
  trace_report: Path,
  last_action_tolerance: float = 1e-5,
  command_tolerance: float = 1e-5,
  gait_phase_tolerance: float = 1e-5,
  joint_vel_threshold: float = 20.0,
  root_ang_vel_threshold: float = 1.0,
  gravity_drift_threshold: float = 0.1,
  raw_action_threshold: float = 10.0,
  worktree_manager: Path = DEFAULT_WORKTREE_MANAGER,
  worktree_observations: Path = DEFAULT_WORKTREE_OBSERVATIONS,
  deploy_manager: Path = DEFAULT_DEPLOY_MANAGER,
  deploy_observations: Path = DEFAULT_DEPLOY_OBSERVATIONS,
) -> dict[str, Any]:
  report = _load_json(trace_report)
  raw_traces = report.get("traces")
  if not isinstance(raw_traces, list) or not raw_traces:
    raise VelocityTraceChronologyError("trace report must contain non-empty traces")
  traces = [trace for trace in raw_traces if isinstance(trace, dict)]
  if len(traces) != len(raw_traces):
    raise VelocityTraceChronologyError("all traces must be mappings")

  pairs, last_action_matches, max_last_action_gap = _early_last_action_pairs(
    traces, tolerance=last_action_tolerance
  )
  summaries = [_trace_summary(trace) for trace in traces]
  max_command_l2 = max(float(item["velocity_commands_l2"]) for item in summaries)
  max_gait_phase_l2 = max(float(item["gait_phase_l2"]) for item in summaries)
  max_last_action_l2 = max(float(item["last_action_l2"]) for item in summaries)

  first_crossings = [
    _first_crossing(
      traces,
      name="joint_vel_rel_l2",
      threshold=joint_vel_threshold,
      value_fn=lambda trace: _term_l2(trace, "joint_vel_rel"),
    ),
    _first_crossing(
      traces,
      name="root_ang_vel_l2",
      threshold=root_ang_vel_threshold,
      value_fn=_root_ang_vel_l2,
    ),
    _first_crossing(
      traces,
      name="projected_gravity_drift_l2",
      threshold=gravity_drift_threshold,
      value_fn=_gravity_drift_l2,
    ),
    _first_crossing(
      traces,
      name="raw_action_l2",
      threshold=raw_action_threshold,
      value_fn=lambda trace: float(trace.get("raw_action_l2", 0.0)),
    ),
    _first_crossing(
      traces,
      name="last_action_l2",
      threshold=raw_action_threshold,
      value_fn=lambda trace: _term_l2(trace, "last_action"),
    ),
  ]
  source_audit = _source_audit(
    worktree_manager=worktree_manager,
    worktree_observations=worktree_observations,
    deploy_manager=deploy_manager,
    deploy_observations=deploy_observations,
  )
  source_contract_ok = all(
    bool(source_audit[key])
    for key in (
      "deploy_last_action_reads_raw_action",
      "deploy_policy_call_observes_previous_raw_action",
      "worktree_last_action_reads_raw_action",
      "worktree_returned_obs_contains_just_processed_raw_action",
    )
  )

  selected = report.get("selected_trace")
  first_unstable = report.get("first_unstable")
  return {
    "schema_version": 1,
    "trace_report": str(trace_report),
    "trace_count": len(traces),
    "first_trace_step": int(traces[0]["step"]),
    "last_trace_step": int(traces[-1]["step"]),
    "first_unstable": first_unstable,
    "selected_trace": (
      None
      if not isinstance(selected, dict)
      else {
        "line": selected.get("line"),
        "step": int(selected["step"]),
        **_trace_summary(selected),
      }
    ),
    "early_consecutive_last_action_pairs": pairs,
    "max_early_last_action_gap_l2": max_last_action_gap,
    "first_crossings": [item for item in first_crossings if item is not None],
    "maxima": {
      "velocity_commands_l2": round(max_command_l2, 6),
      "gait_phase_l2": round(max_gait_phase_l2, 6),
      "last_action_l2": round(max_last_action_l2, 6),
    },
    "early_window": summaries[:12],
    "near_unstable_window": _near_unstable_window(summaries, first_unstable),
    "source_audit": source_audit,
    "decision": {
      "early_last_action_matches_previous_raw_action": last_action_matches,
      "zero_command_observation": max_command_l2 <= command_tolerance,
      "zero_command_gait_phase_masked": max_gait_phase_l2 <= gait_phase_tolerance,
      "source_contract_matches_previous_action_policy_call_semantics": (
        source_contract_ok
      ),
    },
  }


def _near_unstable_window(
  summaries: list[dict[str, Any]], first_unstable: Any, *, radius: int = 75
) -> list[dict[str, Any]]:
  if not isinstance(first_unstable, dict) or "policy_step" not in first_unstable:
    return summaries[-12:]
  step = int(first_unstable["policy_step"])
  return [item for item in summaries if abs(int(item["step"]) - step) <= radius]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Analyze temporal consistency in a G1 Velocity policy I/O trace."
  )
  parser.add_argument("--trace-report", required=True, type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--last-action-tolerance", type=float, default=1e-5)
  parser.add_argument("--command-tolerance", type=float, default=1e-5)
  parser.add_argument("--gait-phase-tolerance", type=float, default=1e-5)
  parser.add_argument("--expect-early-last-action-match", action="store_true")
  parser.add_argument("--expect-zero-command-phase-mask", action="store_true")
  parser.add_argument("--expect-source-contract", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = analyze_trace_chronology(
    trace_report=args.trace_report,
    last_action_tolerance=args.last_action_tolerance,
    command_tolerance=args.command_tolerance,
    gait_phase_tolerance=args.gait_phase_tolerance,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.report_out is not None:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(text + "\n", encoding="utf-8")
  print(text)
  decision = report["decision"]
  if (
    args.expect_early_last_action_match
    and not decision["early_last_action_matches_previous_raw_action"]
  ):
    return 1
  if args.expect_zero_command_phase_mask and not (
    decision["zero_command_observation"] and decision["zero_command_gait_phase_masked"]
  ):
    return 1
  if (
    args.expect_source_contract
    and not decision["source_contract_matches_previous_action_policy_call_semantics"]
  ):
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
