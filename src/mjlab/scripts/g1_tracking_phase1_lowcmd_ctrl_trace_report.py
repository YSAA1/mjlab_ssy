"""Analyze Unitree MuJoCo lowcmd-to-ctrl traces for G1 phase-1 diagnosis."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any


class LowcmdCtrlTraceReportError(RuntimeError):
  """Raised when lowcmd-to-ctrl trace evidence is missing or invalid."""


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


def _as_float(value: Any, *, default: float = 0.0) -> float:
  if isinstance(value, int | float):
    return float(value)
  return default


def parse_lowcmd_ctrl_traces(log_path: Path) -> list[dict[str, Any]]:
  traces: list[dict[str, Any]] = []
  for line_no, line in enumerate(log_path.read_text(encoding="utf-8").splitlines(), 1):
    if "[PHASE1_SIM]" not in line or "event=lowcmd_ctrl_trace" not in line:
      continue
    trace: dict[str, Any] = {"line": line_no}
    for match in FIELD_RE.finditer(line):
      trace[match.group("key")] = _parse_value(match.group("value"))
    trace["pd_term_l2"] = math.sqrt(
      _as_float(trace.get("pos_term_l2")) ** 2
      + _as_float(trace.get("vel_term_l2")) ** 2
    )
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
    "sample",
    "sim_time",
    "ctrl_l2",
    "ctrl_max",
    "tau_l2",
    "tau_max",
    "pos_term_l2",
    "pos_term_max",
    "vel_term_l2",
    "vel_term_max",
    "pd_term_l2",
    "q_error_l2",
    "q_error_max",
    "dq_error_l2",
    "dq_error_max",
    "kp_l2",
    "kd_l2",
    "top_index",
    "top_ctrl",
    "top_tau",
    "top_pos_term",
    "top_vel_term",
    "top_q_cmd",
    "top_q_sensor",
    "top_q_error",
    "top_dq_cmd",
    "top_dq_sensor",
    "top_dq_error",
    "top_kp",
    "top_kd",
  ]
  return {key: trace.get(key) for key in keys if key in trace}


def _dominant_ctrl_source(
  trace: dict[str, Any] | None,
  *,
  nonzero_ctrl_threshold: float,
  pd_dominance_ratio: float,
) -> str:
  if trace is None or _as_float(trace.get("ctrl_l2")) <= nonzero_ctrl_threshold:
    return "zero_ctrl"
  tau_l2 = _as_float(trace.get("tau_l2"))
  pos_l2 = _as_float(trace.get("pos_term_l2"))
  vel_l2 = _as_float(trace.get("vel_term_l2"))
  pd_l2 = math.sqrt(pos_l2 * pos_l2 + vel_l2 * vel_l2)
  if pd_l2 >= nonzero_ctrl_threshold and pd_l2 >= tau_l2 * pd_dominance_ratio:
    if pos_l2 >= vel_l2 * 1.25:
      return "position_pd_error"
    if vel_l2 >= pos_l2 * 1.25:
      return "velocity_pd_error"
    return "mixed_pd_error"
  if tau_l2 > nonzero_ctrl_threshold:
    return "tau_command"
  return "insufficient_component_evidence"


def _classification(
  first_nonzero_ctrl: dict[str, Any] | None,
  *,
  nonzero_ctrl_threshold: float,
  pd_dominance_ratio: float,
) -> str:
  dominant = _dominant_ctrl_source(
    first_nonzero_ctrl,
    nonzero_ctrl_threshold=nonzero_ctrl_threshold,
    pd_dominance_ratio=pd_dominance_ratio,
  )
  if dominant == "zero_ctrl":
    return "zero_ctrl"
  if dominant == "position_pd_error":
    return "position_pd_ctrl_handoff"
  if dominant == "velocity_pd_error":
    return "velocity_pd_ctrl_handoff"
  if dominant == "mixed_pd_error":
    return "mixed_pd_ctrl_handoff"
  if dominant == "tau_command":
    return "tau_ctrl_handoff"
  return "insufficient_component_evidence"


def analyze_lowcmd_ctrl_trace(
  evidence_dir: Path,
  *,
  nonzero_ctrl_threshold: float = 1.0,
  large_ctrl_threshold: float = 50.0,
  pd_dominance_ratio: float = 1.0,
  tau_near_zero_threshold: float = 1e-6,
) -> dict[str, Any]:
  log_path = evidence_dir / "unitree_mujoco.log"
  if not log_path.is_file():
    raise LowcmdCtrlTraceReportError(f"missing MuJoCo log: {log_path}")
  traces = parse_lowcmd_ctrl_traces(log_path)
  if not traces:
    raise LowcmdCtrlTraceReportError(f"no lowcmd_ctrl_trace lines in {log_path}")
  traces = sorted(traces, key=lambda trace: (int(trace["sample"]), int(trace["line"])))

  first_trace = traces[0]
  first_nonzero_ctrl = _first_matching(
    traces, lambda trace: _as_float(trace.get("ctrl_l2")) > nonzero_ctrl_threshold
  )
  first_large_ctrl = _first_matching(
    traces, lambda trace: _as_float(trace.get("ctrl_l2")) > large_ctrl_threshold
  )
  max_ctrl = max(traces, key=lambda trace: _as_float(trace.get("ctrl_l2")))
  max_pos_term = max(traces, key=lambda trace: _as_float(trace.get("pos_term_l2")))
  max_vel_term = max(traces, key=lambda trace: _as_float(trace.get("vel_term_l2")))
  max_tau = max(traces, key=lambda trace: _as_float(trace.get("tau_l2")))

  first_nonzero_pd_l2 = _as_float(
    None if first_nonzero_ctrl is None else first_nonzero_ctrl.get("pd_term_l2")
  )
  first_nonzero_tau_l2 = _as_float(
    None if first_nonzero_ctrl is None else first_nonzero_ctrl.get("tau_l2")
  )
  classification = _classification(
    first_nonzero_ctrl,
    nonzero_ctrl_threshold=nonzero_ctrl_threshold,
    pd_dominance_ratio=pd_dominance_ratio,
  )

  return {
    "schema_version": 1,
    "evidence_dir": str(evidence_dir),
    "log_path": str(log_path),
    "trace_count": len(traces),
    "thresholds": {
      "nonzero_ctrl_l2": nonzero_ctrl_threshold,
      "large_ctrl_l2": large_ctrl_threshold,
      "pd_dominance_ratio": pd_dominance_ratio,
      "tau_near_zero_l2": tau_near_zero_threshold,
    },
    "classification": classification,
    "first_trace": _summary(first_trace),
    "first_nonzero_ctrl_trace": _summary(first_nonzero_ctrl),
    "first_large_ctrl_trace": _summary(first_large_ctrl),
    "maxima": {
      "ctrl_l2": _summary(max_ctrl),
      "pos_term_l2": _summary(max_pos_term),
      "vel_term_l2": _summary(max_vel_term),
      "tau_l2": _summary(max_tau),
    },
    "decision": {
      "first_sample_nonzero_ctrl": _as_float(first_trace.get("ctrl_l2"))
      > nonzero_ctrl_threshold,
      "has_nonzero_ctrl": first_nonzero_ctrl is not None,
      "has_large_ctrl": first_large_ctrl is not None,
      "first_nonzero_has_pd_source": (
        first_nonzero_ctrl is not None
        and first_nonzero_pd_l2 > nonzero_ctrl_threshold
        and first_nonzero_pd_l2 >= first_nonzero_tau_l2 * pd_dominance_ratio
      ),
      "first_nonzero_tau_near_zero": first_nonzero_tau_l2 <= tau_near_zero_threshold,
      "dominant_first_nonzero_source": _dominant_ctrl_source(
        first_nonzero_ctrl,
        nonzero_ctrl_threshold=nonzero_ctrl_threshold,
        pd_dominance_ratio=pd_dominance_ratio,
      ),
    },
    "interpretation": (
      "The bridge trace decomposes the exact Unitree MuJoCo formula "
      "ctrl = tau + kp * (q_cmd - q_sensor) + kd * (dq_cmd - dq_sensor). "
      "A PD-source classification means the early actuator command is explained "
      "by the lowcmd target/current-state error at the bridge boundary, not by "
      "a nonzero tau command alone."
    ),
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Analyze [PHASE1_SIM] lowcmd-to-ctrl bridge traces."
  )
  parser.add_argument("--evidence-dir", required=True, type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--nonzero-ctrl-threshold", type=float, default=1.0)
  parser.add_argument("--large-ctrl-threshold", type=float, default=50.0)
  parser.add_argument("--pd-dominance-ratio", type=float, default=1.0)
  parser.add_argument("--tau-near-zero-threshold", type=float, default=1e-6)
  parser.add_argument("--expect-trace", action="store_true")
  parser.add_argument("--expect-nonzero-ctrl", action="store_true")
  parser.add_argument("--expect-pd-source", action="store_true")
  parser.add_argument("--expect-tau-near-zero", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = analyze_lowcmd_ctrl_trace(
    args.evidence_dir,
    nonzero_ctrl_threshold=args.nonzero_ctrl_threshold,
    large_ctrl_threshold=args.large_ctrl_threshold,
    pd_dominance_ratio=args.pd_dominance_ratio,
    tau_near_zero_threshold=args.tau_near_zero_threshold,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.report_out is not None:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(text + "\n", encoding="utf-8")
  print(text)

  decision = report["decision"]
  if args.expect_trace and int(report["trace_count"]) <= 0:
    return 1
  if args.expect_nonzero_ctrl and not decision["has_nonzero_ctrl"]:
    return 1
  if args.expect_pd_source and not decision["first_nonzero_has_pd_source"]:
    return 1
  if args.expect_tau_near_zero and not decision["first_nonzero_tau_near_zero"]:
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
