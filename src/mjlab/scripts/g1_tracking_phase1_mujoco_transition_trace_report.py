"""Analyze Unitree MuJoCo transition traces for G1 phase-1 diagnosis."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any


class MujocoTransitionTraceReportError(RuntimeError):
  """Raised when MuJoCo transition trace evidence is missing or invalid."""


FIELD_RE = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>\([^)]*\)|\S+)")


def _parse_value(value: str) -> int | float | str | list[float]:
  if value.startswith("(") and value.endswith(")"):
    raw_items = value[1:-1].split(",")
    return [float(item) for item in raw_items if item]
  if re.fullmatch(r"[-+]?\d+", value):
    return int(value)
  try:
    return float(value)
  except ValueError:
    return value


def parse_mujoco_transition_traces(log_path: Path) -> list[dict[str, Any]]:
  traces: list[dict[str, Any]] = []
  for line_no, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
    if "[PHASE1_SIM]" not in line or "event=mujoco_transition_trace" not in line:
      continue
    trace: dict[str, Any] = {"line": line_no}
    for match in FIELD_RE.finditer(line):
      trace[match.group("key")] = _parse_value(match.group("value"))
    traces.append(trace)
  return traces


def _first_matching(traces: list[dict[str, Any]], predicate) -> dict[str, Any] | None:
  for trace in traces:
    if predicate(trace):
      return trace
  return None


def _summary(trace: dict[str, Any] | None) -> dict[str, Any] | None:
  if trace is None:
    return None
  keys = [
    "line",
    "step",
    "sim_time",
    "root_pos",
    "root_lin_vel_l2",
    "root_ang_vel_l2",
    "qvel_l2",
    "qvel_max",
    "ctrl_l2",
    "ctrl_max",
    "ncon",
    "elastic_config",
    "elastic_enabled",
    "elastic_length",
    "elastic_force_l2",
  ]
  summary = {key: trace.get(key) for key in keys if key in trace}
  return summary


def analyze_mujoco_transition_trace(
  evidence_dir: Path,
  *,
  qvel_threshold: float = 1.0,
  root_ang_vel_threshold: float = 0.05,
  large_ctrl_threshold: float = 50.0,
) -> dict[str, Any]:
  log_path = evidence_dir / "unitree_mujoco.log"
  if not log_path.is_file():
    raise MujocoTransitionTraceReportError(f"missing MuJoCo log: {log_path}")
  traces = parse_mujoco_transition_traces(log_path)
  if not traces:
    raise MujocoTransitionTraceReportError(f"no PHASE1_SIM traces in {log_path}")
  traces = sorted(traces, key=lambda trace: int(trace["step"]))

  first_trace = traces[0]
  first_dynamic = _first_matching(
    traces,
    lambda trace: float(trace.get("qvel_l2", 0.0)) > qvel_threshold
    or float(trace.get("root_ang_vel_l2", 0.0)) > root_ang_vel_threshold,
  )
  first_contact = _first_matching(traces, lambda trace: int(trace.get("ncon", 0)) > 0)
  first_large_ctrl = _first_matching(
    traces, lambda trace: float(trace.get("ctrl_l2", 0.0)) > large_ctrl_threshold
  )
  max_qvel = max(traces, key=lambda trace: float(trace.get("qvel_l2", 0.0)))
  max_ctrl = max(traces, key=lambda trace: float(trace.get("ctrl_l2", 0.0)))
  max_root_ang_vel = max(
    traces, key=lambda trace: float(trace.get("root_ang_vel_l2", 0.0))
  )

  first_dynamic_step = None if first_dynamic is None else int(first_dynamic["step"])
  first_contact_step = None if first_contact is None else int(first_contact["step"])
  first_large_ctrl_step = (
    None if first_large_ctrl is None else int(first_large_ctrl["step"])
  )
  first_trace_step = int(first_trace["step"])
  root_z_at_first_trace = (
    first_trace.get("root_pos", [None, None, None])[2]
    if isinstance(first_trace.get("root_pos"), list)
    and len(first_trace["root_pos"]) >= 3
    else None
  )
  root_z_at_first_contact = (
    first_contact.get("root_pos", [None, None, None])[2]
    if first_contact is not None
    and isinstance(first_contact.get("root_pos"), list)
    and len(first_contact["root_pos"]) >= 3
    else None
  )

  motion_before_contact = (
    first_dynamic_step is not None
    and first_contact_step is not None
    and first_dynamic_step < first_contact_step
  )
  motion_before_large_ctrl = (
    first_dynamic_step is not None
    and first_large_ctrl_step is not None
    and first_dynamic_step < first_large_ctrl_step
  )
  first_physics_step_is_dynamic = first_dynamic_step == first_trace_step

  return {
    "schema_version": 1,
    "evidence_dir": str(evidence_dir),
    "log_path": str(log_path),
    "trace_count": len(traces),
    "thresholds": {
      "qvel_l2": qvel_threshold,
      "root_ang_vel_l2": root_ang_vel_threshold,
      "large_ctrl_l2": large_ctrl_threshold,
    },
    "first_trace": _summary(first_trace),
    "first_dynamic_trace": _summary(first_dynamic),
    "first_contact_trace": _summary(first_contact),
    "first_large_ctrl_trace": _summary(first_large_ctrl),
    "maxima": {
      "qvel_l2": _summary(max_qvel),
      "ctrl_l2": _summary(max_ctrl),
      "root_ang_vel_l2": _summary(max_root_ang_vel),
    },
    "root_z_drop_before_first_contact": None
    if root_z_at_first_trace is None or root_z_at_first_contact is None
    else root_z_at_first_contact - root_z_at_first_trace,
    "decision": {
      "first_physics_step_is_dynamic": first_physics_step_is_dynamic,
      "motion_before_contact": motion_before_contact,
      "motion_before_large_ctrl": motion_before_large_ctrl,
      "elastic_force_disabled": all(
        int(trace.get("elastic_config", -1)) == 0
        and abs(float(trace.get("elastic_force_l2", 0.0))) <= 1e-9
        for trace in traces
      ),
    },
    "interpretation": (
      "MuJoCo transition traces are logged immediately after mj_step. If the "
      "first dynamic trace precedes both first contact and first large ctrl, "
      "the earliest motion is not explained by contact impulse or a later "
      "large actuator command alone; inspect paused-to-run handoff, initial "
      "floating/support state, and low-level command state at Run."
    ),
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Analyze [PHASE1_SIM] MuJoCo transition traces from a G1 evidence dir."
  )
  parser.add_argument("--evidence-dir", required=True, type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--qvel-threshold", type=float, default=1.0)
  parser.add_argument("--root-ang-vel-threshold", type=float, default=0.05)
  parser.add_argument("--large-ctrl-threshold", type=float, default=50.0)
  parser.add_argument("--expect-trace", action="store_true")
  parser.add_argument("--expect-first-step-dynamic", action="store_true")
  parser.add_argument("--expect-motion-before-contact", action="store_true")
  parser.add_argument("--expect-motion-before-large-ctrl", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = analyze_mujoco_transition_trace(
    args.evidence_dir,
    qvel_threshold=args.qvel_threshold,
    root_ang_vel_threshold=args.root_ang_vel_threshold,
    large_ctrl_threshold=args.large_ctrl_threshold,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.report_out is not None:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(text + "\n", encoding="utf-8")
  print(text)

  decision = report["decision"]
  if args.expect_trace and int(report["trace_count"]) <= 0:
    return 1
  if args.expect_first_step_dynamic and not decision["first_physics_step_is_dynamic"]:
    return 1
  if args.expect_motion_before_contact and not decision["motion_before_contact"]:
    return 1
  if args.expect_motion_before_large_ctrl and not decision["motion_before_large_ctrl"]:
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
