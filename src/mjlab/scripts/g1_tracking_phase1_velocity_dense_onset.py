"""Classify dense-onset ordering in a G1 Velocity policy I/O trace."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


class VelocityDenseOnsetError(RuntimeError):
  """Raised when dense-onset evidence cannot be analyzed safely."""


def _load_json(path: Path) -> dict[str, Any]:
  data = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise VelocityDenseOnsetError(f"JSON root must be a mapping: {path}")
  return data


def _as_float_list(value: Any, *, name: str) -> list[float]:
  if not isinstance(value, list):
    raise VelocityDenseOnsetError(f"{name} must be a list")
  return [float(item) for item in value]


def _l2(values: list[float]) -> float:
  return math.sqrt(sum(value * value for value in values))


def _term_values(trace: dict[str, Any], name: str) -> list[float]:
  terms = trace.get("obs_terms")
  if not isinstance(terms, dict) or not isinstance(terms.get(name), dict):
    raise VelocityDenseOnsetError(f"trace missing obs term {name!r}")
  return _as_float_list(terms[name].get("values"), name=f"obs_terms.{name}.values")


def _term_l2(trace: dict[str, Any], name: str) -> float:
  return _l2(_term_values(trace, name))


def _root_ang_vel_l2(trace: dict[str, Any]) -> float:
  if "root_ang_vel" in trace:
    return _l2(_as_float_list(trace["root_ang_vel"], name="root_ang_vel"))
  return _term_l2(trace, "base_ang_vel")


def _gravity_z_drift(trace: dict[str, Any]) -> float:
  gravity = _term_values(trace, "projected_gravity")
  if len(gravity) != 3:
    raise VelocityDenseOnsetError("projected_gravity must have dim 3")
  return abs(gravity[2] + 1.0)


def _summary(trace: dict[str, Any]) -> dict[str, Any]:
  return {
    "line": trace.get("line"),
    "step": int(trace["step"]),
    "joint_vel_rel_l2": round(_term_l2(trace, "joint_vel_rel"), 6),
    "root_ang_vel_l2": round(_root_ang_vel_l2(trace), 6),
    "gravity_z_drift": round(_gravity_z_drift(trace), 6),
    "velocity_commands_l2": round(_term_l2(trace, "velocity_commands"), 6),
    "gait_phase_l2": round(_term_l2(trace, "gait_phase"), 6),
    "joint_pos_rel_l2": round(_term_l2(trace, "joint_pos_rel"), 6),
    "last_action_l2": round(_term_l2(trace, "last_action"), 6),
    "raw_action_l2": round(float(trace.get("raw_action_l2", 0.0)), 6),
    "processed_action_l2": round(float(trace.get("processed_action_l2", 0.0)), 6),
  }


def _low_dynamic_onset(
  trace: dict[str, Any],
  *,
  joint_vel_threshold: float,
  root_ang_vel_threshold: float,
  gravity_z_threshold: float,
) -> bool:
  return (
    _term_l2(trace, "joint_vel_rel") > joint_vel_threshold
    or _root_ang_vel_l2(trace) > root_ang_vel_threshold
    or _gravity_z_drift(trace) > gravity_z_threshold
  )


def _first_matching(traces: list[dict[str, Any]], predicate) -> dict[str, Any] | None:
  for trace in traces:
    if predicate(trace):
      return trace
  return None


def _window_around_step(
  summaries: list[dict[str, Any]], *, step: int, radius: int
) -> list[dict[str, Any]]:
  return [item for item in summaries if abs(int(item["step"]) - step) <= radius]


def analyze_dense_onset(
  *,
  trace_report: Path,
  joint_vel_threshold: float = 1.0,
  root_ang_vel_threshold: float = 0.05,
  gravity_z_threshold: float = 0.01,
  quiet_last_action_threshold: float = 5.0,
  large_action_threshold: float = 10.0,
  command_tolerance: float = 1e-5,
  gait_phase_tolerance: float = 1e-5,
) -> dict[str, Any]:
  report = _load_json(trace_report)
  raw_traces = report.get("traces")
  if not isinstance(raw_traces, list) or not raw_traces:
    raise VelocityDenseOnsetError("trace report must contain non-empty traces")
  traces = [trace for trace in raw_traces if isinstance(trace, dict)]
  if len(traces) != len(raw_traces):
    raise VelocityDenseOnsetError("all traces must be mappings")
  traces = sorted(traces, key=lambda trace: int(trace["step"]))
  summaries = [_summary(trace) for trace in traces]

  first_onset = _first_matching(
    traces,
    lambda trace: _low_dynamic_onset(
      trace,
      joint_vel_threshold=joint_vel_threshold,
      root_ang_vel_threshold=root_ang_vel_threshold,
      gravity_z_threshold=gravity_z_threshold,
    ),
  )
  if first_onset is None:
    raise VelocityDenseOnsetError("no low dynamic onset found in trace report")

  first_onset_step = int(first_onset["step"])
  first_onset_index = traces.index(first_onset)
  previous_logged = traces[first_onset_index - 1] if first_onset_index > 0 else None
  next_logged = (
    traces[first_onset_index + 1] if first_onset_index + 1 < len(traces) else None
  )

  first_large_raw = _first_matching(
    traces,
    lambda trace: float(trace.get("raw_action_l2", 0.0)) > large_action_threshold,
  )
  first_large_last = _first_matching(
    traces,
    lambda trace: _term_l2(trace, "last_action") > large_action_threshold,
  )

  max_command_l2 = max(float(item["velocity_commands_l2"]) for item in summaries)
  max_gait_phase_l2 = max(float(item["gait_phase_l2"]) for item in summaries)
  first_onset_last_action_l2 = _term_l2(first_onset, "last_action")
  first_onset_raw_action_l2 = float(first_onset.get("raw_action_l2", 0.0))

  first_large_raw_step = (
    None if first_large_raw is None else int(first_large_raw["step"])
  )
  first_large_last_step = (
    None if first_large_last is None else int(first_large_last["step"])
  )
  observed_motion_precedes_large_previous_action = (
    first_onset_last_action_l2 <= quiet_last_action_threshold
    and first_large_last_step is not None
    and first_onset_step < first_large_last_step
  )
  observed_motion_before_policy_response = (
    observed_motion_precedes_large_previous_action
    and first_onset_raw_action_l2 <= quiet_last_action_threshold
    and first_large_raw_step is not None
    and first_onset_step < first_large_raw_step < first_large_last_step
  )

  if observed_motion_before_policy_response:
    onset_order = "observed_motion_before_policy_response"
  elif (
    observed_motion_precedes_large_previous_action
    and first_onset_raw_action_l2 > large_action_threshold
    and first_large_raw_step == first_onset_step
  ):
    onset_order = "observed_motion_before_large_previous_action"
  elif first_onset_last_action_l2 > quiet_last_action_threshold:
    onset_order = "large_previous_action_at_first_observed_motion"
  else:
    onset_order = "inconclusive"

  return {
    "schema_version": 1,
    "trace_report": str(trace_report),
    "trace_count": len(traces),
    "thresholds": {
      "joint_vel_l2": joint_vel_threshold,
      "root_ang_vel_l2": root_ang_vel_threshold,
      "gravity_z_drift": gravity_z_threshold,
      "quiet_last_action_l2": quiet_last_action_threshold,
      "large_action_l2": large_action_threshold,
      "command_l2": command_tolerance,
      "gait_phase_l2": gait_phase_tolerance,
    },
    "first_dynamic_onset": _summary(first_onset),
    "previous_logged_trace": None
    if previous_logged is None
    else _summary(previous_logged),
    "next_logged_trace": None if next_logged is None else _summary(next_logged),
    "first_large_current_raw_action": (
      None if first_large_raw is None else _summary(first_large_raw)
    ),
    "first_large_previous_action": (
      None if first_large_last is None else _summary(first_large_last)
    ),
    "onset_window": _window_around_step(summaries, step=first_onset_step, radius=10),
    "maxima": {
      "velocity_commands_l2": round(max_command_l2, 6),
      "gait_phase_l2": round(max_gait_phase_l2, 6),
    },
    "onset_order": onset_order,
    "decision": {
      "zero_command_observation": max_command_l2 <= command_tolerance,
      "zero_command_gait_phase_masked": max_gait_phase_l2 <= gait_phase_tolerance,
      "first_onset_has_quiet_previous_action": (
        first_onset_last_action_l2 <= quiet_last_action_threshold
      ),
      "first_onset_current_raw_action_is_large": (
        first_onset_raw_action_l2 > large_action_threshold
      ),
      "first_onset_current_raw_action_is_quiet": (
        first_onset_raw_action_l2 <= quiet_last_action_threshold
      ),
      "observed_motion_precedes_large_previous_action": (
        observed_motion_precedes_large_previous_action
      ),
      "observed_motion_before_policy_response": (
        observed_motion_before_policy_response
      ),
    },
    "interpretation": (
      "Policy I/O traces are logged after observation computation and policy "
      "inference. If the first dynamic-onset observation has a quiet "
      "last_action, the first observed motion was present before any large "
      "previous-action term could have been applied. If the current raw action "
      "is also quiet, the first observed motion also precedes a large policy "
      "response."
    ),
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Classify ordering in a dense G1 Velocity policy I/O onset trace."
  )
  parser.add_argument("--trace-report", required=True, type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--joint-vel-threshold", type=float, default=1.0)
  parser.add_argument("--root-ang-vel-threshold", type=float, default=0.05)
  parser.add_argument("--gravity-z-threshold", type=float, default=0.01)
  parser.add_argument("--quiet-last-action-threshold", type=float, default=5.0)
  parser.add_argument("--large-action-threshold", type=float, default=10.0)
  parser.add_argument("--command-tolerance", type=float, default=1e-5)
  parser.add_argument("--gait-phase-tolerance", type=float, default=1e-5)
  parser.add_argument("--expect-zero-command-phase-mask", action="store_true")
  parser.add_argument(
    "--expect-motion-before-large-previous-action", action="store_true"
  )
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = analyze_dense_onset(
    trace_report=args.trace_report,
    joint_vel_threshold=args.joint_vel_threshold,
    root_ang_vel_threshold=args.root_ang_vel_threshold,
    gravity_z_threshold=args.gravity_z_threshold,
    quiet_last_action_threshold=args.quiet_last_action_threshold,
    large_action_threshold=args.large_action_threshold,
    command_tolerance=args.command_tolerance,
    gait_phase_tolerance=args.gait_phase_tolerance,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.report_out is not None:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(text + "\n", encoding="utf-8")
  print(text)

  decision = report["decision"]
  if args.expect_zero_command_phase_mask and not (
    decision["zero_command_observation"] and decision["zero_command_gait_phase_masked"]
  ):
    return 1
  if (
    args.expect_motion_before_large_previous_action
    and not decision["observed_motion_precedes_large_previous_action"]
  ):
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
