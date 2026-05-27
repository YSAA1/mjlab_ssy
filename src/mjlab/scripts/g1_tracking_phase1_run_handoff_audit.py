"""Audit paused-to-run handoff evidence for G1 phase-1 Velocity sim2sim."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


class RunHandoffAuditError(RuntimeError):
  """Raised when run-handoff audit inputs are missing or invalid."""


def _load_json(path: Path, name: str) -> dict[str, Any]:
  if not path.is_file():
    raise RunHandoffAuditError(f"missing {name}: {path}")
  data = json.loads(path.read_text(encoding="utf-8"))
  if not isinstance(data, dict):
    raise RunHandoffAuditError(f"{name} JSON root must be a mapping: {path}")
  return data


def _as_float(value: Any, *, default: float = 0.0) -> float:
  if isinstance(value, int | float):
    return float(value)
  return default


def _as_int(value: Any, *, default: int = 0) -> int:
  if isinstance(value, int):
    return value
  if isinstance(value, float):
    return int(value)
  return default


def _l2(values: Any) -> float:
  if not isinstance(values, list):
    return 0.0
  return math.sqrt(
    sum(
      float(value) * float(value) for value in values if isinstance(value, int | float)
    )
  )


def _trace_metric(trace: dict[str, Any], key: str) -> float:
  value = trace.get(key)
  if isinstance(value, int | float):
    return float(value)
  if key == "root_ang_vel_l2":
    return _l2(trace.get("root_ang_vel"))
  return 0.0


def _summarize_policy_trace(trace: dict[str, Any] | None) -> dict[str, Any] | None:
  if trace is None:
    return None
  obs_terms = trace.get("obs_terms")
  command_norm = None
  gait_phase_norm = None
  last_action_l2 = None
  if isinstance(obs_terms, dict):
    command = obs_terms.get("velocity_commands")
    gait_phase = obs_terms.get("gait_phase")
    last_action = obs_terms.get("last_action")
    if isinstance(command, dict):
      command_norm = command.get("l2")
    if isinstance(gait_phase, dict):
      gait_phase_norm = gait_phase.get("l2")
    if isinstance(last_action, dict):
      last_action_l2 = last_action.get("l2")
  return {
    "line": trace.get("line"),
    "step": trace.get("step"),
    "joint_pos_l2": trace.get("joint_pos_l2"),
    "joint_vel_l2": trace.get("joint_vel_l2"),
    "root_ang_vel_l2": _trace_metric(trace, "root_ang_vel_l2"),
    "raw_action_l2": trace.get("raw_action_l2"),
    "processed_action_l2": trace.get("processed_action_l2"),
    "command_l2": command_norm,
    "gait_phase_l2": gait_phase_norm,
    "last_action_l2": last_action_l2,
  }


def _find_frozen_policy_traces(
  traces: list[Any],
  *,
  frozen_joint_vel_threshold: float,
  frozen_root_ang_vel_threshold: float,
) -> list[dict[str, Any]]:
  frozen = []
  for trace in traces:
    if not isinstance(trace, dict):
      continue
    if (
      _as_float(trace.get("joint_vel_l2")) <= frozen_joint_vel_threshold
      and _trace_metric(trace, "root_ang_vel_l2") <= frozen_root_ang_vel_threshold
    ):
      frozen.append(trace)
  return sorted(frozen, key=lambda trace: _as_int(trace.get("step")))


def _policy_frozen_summary(
  policy_report: dict[str, Any],
  *,
  frozen_joint_vel_threshold: float,
  frozen_root_ang_vel_threshold: float,
  paused_advance_step_threshold: int,
) -> dict[str, Any]:
  traces = policy_report.get("traces")
  if not isinstance(traces, list):
    raise RunHandoffAuditError("policy I/O report missing traces list")
  frozen = _find_frozen_policy_traces(
    traces,
    frozen_joint_vel_threshold=frozen_joint_vel_threshold,
    frozen_root_ang_vel_threshold=frozen_root_ang_vel_threshold,
  )
  first = frozen[0] if frozen else None
  last = frozen[-1] if frozen else None
  max_step = max((_as_int(trace.get("step")) for trace in frozen), default=0)
  raw_l2_values = [_as_float(trace.get("raw_action_l2")) for trace in frozen]
  processed_l2_values = [
    _as_float(trace.get("processed_action_l2")) for trace in frozen
  ]
  return {
    "trace_count": len(traces),
    "frozen_trace_count": len(frozen),
    "max_frozen_policy_step": max_step,
    "paused_policy_thread_advanced": max_step >= paused_advance_step_threshold,
    "first_frozen_trace": _summarize_policy_trace(first),
    "last_frozen_trace": _summarize_policy_trace(last),
    "raw_action_l2_range_while_frozen": {
      "min": min(raw_l2_values, default=0.0),
      "max": max(raw_l2_values, default=0.0),
    },
    "processed_action_l2_range_while_frozen": {
      "min": min(processed_l2_values, default=0.0),
      "max": max(processed_l2_values, default=0.0),
    },
  }


def _mujoco_summary(
  mujoco_report: dict[str, Any],
  *,
  nonzero_ctrl_threshold: float,
) -> dict[str, Any]:
  first_trace = mujoco_report.get("first_trace")
  first_dynamic = mujoco_report.get("first_dynamic_trace")
  first_contact = mujoco_report.get("first_contact_trace")
  first_large_ctrl = mujoco_report.get("first_large_ctrl_trace")
  decision = mujoco_report.get("decision")
  if not isinstance(first_trace, dict):
    raise RunHandoffAuditError("MuJoCo transition report missing first_trace")
  if not isinstance(decision, dict):
    raise RunHandoffAuditError("MuJoCo transition report missing decision")
  return {
    "trace_count": mujoco_report.get("trace_count"),
    "first_trace": first_trace,
    "first_dynamic_trace": first_dynamic,
    "first_contact_trace": first_contact,
    "first_large_ctrl_trace": first_large_ctrl,
    "root_z_drop_before_first_contact": mujoco_report.get(
      "root_z_drop_before_first_contact"
    ),
    "first_step_nonzero_ctrl": _as_float(first_trace.get("ctrl_l2"))
    > nonzero_ctrl_threshold,
    "first_step_no_contact": _as_int(first_trace.get("ncon")) == 0,
    "decision": {
      "first_physics_step_is_dynamic": bool(
        decision.get("first_physics_step_is_dynamic")
      ),
      "motion_before_contact": bool(decision.get("motion_before_contact")),
      "motion_before_large_ctrl": bool(decision.get("motion_before_large_ctrl")),
      "elastic_force_disabled": bool(decision.get("elastic_force_disabled")),
    },
  }


def _support_summary(
  velocity_report: dict[str, Any],
  *,
  support_gap_threshold: float,
) -> dict[str, Any]:
  sim_config = velocity_report.get("sim_config")
  initial_contact = velocity_report.get("initial_contact")
  if not isinstance(sim_config, dict):
    raise RunHandoffAuditError("Velocity report missing sim_config")
  if not isinstance(initial_contact, dict):
    raise RunHandoffAuditError("Velocity report missing initial_contact")
  min_foot_surface_z = initial_contact.get("min_foot_surface_z")
  support_gap = _as_float(min_foot_surface_z)
  return {
    "sim_config": sim_config,
    "start_paused": sim_config.get("start_paused"),
    "enable_elastic_band": sim_config.get("enable_elastic_band"),
    "root_z": initial_contact.get("root_z"),
    "min_foot_surface_z": min_foot_surface_z,
    "floor_clearance_passed": initial_contact.get("floor_clearance_passed"),
    "support_gap_before_run": support_gap > support_gap_threshold,
    "support_gap_threshold": support_gap_threshold,
  }


def _bridge_summary(bridge_source: Path | None) -> dict[str, Any]:
  if bridge_source is None:
    return {
      "source": None,
      "ctrl_semantics": (
        "MuJoCo ctrl is interpreted as the low-level motor torque produced from "
        "lowcmd tau plus kp/kd position and velocity error."
      ),
      "formula_found": None,
    }
  if not bridge_source.is_file():
    raise RunHandoffAuditError(f"missing bridge source: {bridge_source}")
  text = bridge_source.read_text(encoding="utf-8")
  formula_found = (
    "mj_data_->ctrl[i]" in text
    and ".kp()" in text
    and ".kd()" in text
    and ".tau()" in text
  )
  return {
    "source": str(bridge_source),
    "ctrl_semantics": (
      "MuJoCo ctrl is the Unitree bridge motor torque computed from lowcmd "
      "tau + kp * (q_target - q_sensor) + kd * (dq_target - dq_sensor)."
    ),
    "formula_found": formula_found,
  }


def _classify(decision: dict[str, Any]) -> str:
  if (
    decision["policy_steps_while_physics_paused"]
    and decision["support_gap_before_run"]
    and decision["first_physics_step_is_dynamic"]
    and decision["first_step_no_contact"]
  ):
    return "paused_policy_with_floating_support_handoff"
  if (
    decision["support_gap_before_run"]
    and decision["first_physics_step_is_dynamic"]
    and decision["first_step_no_contact"]
  ):
    return "floating_support_handoff"
  if decision["policy_steps_while_physics_paused"]:
    return "paused_policy_handoff"
  if decision["first_step_nonzero_ctrl"] and decision["first_physics_step_is_dynamic"]:
    return "lowcmd_ctrl_handoff"
  return "insufficient_handoff_evidence"


def analyze_run_handoff_audit(
  *,
  evidence_dir: Path,
  velocity_report_path: Path,
  policy_io_report_path: Path,
  mujoco_transition_report_path: Path,
  bridge_source: Path | None = None,
  frozen_joint_vel_threshold: float = 1e-6,
  frozen_root_ang_vel_threshold: float = 1e-6,
  paused_advance_step_threshold: int = 10,
  support_gap_threshold: float = 0.005,
  nonzero_ctrl_threshold: float = 1.0,
) -> dict[str, Any]:
  velocity_report = _load_json(velocity_report_path, "Velocity report")
  policy_report = _load_json(policy_io_report_path, "policy I/O report")
  mujoco_report = _load_json(mujoco_transition_report_path, "MuJoCo transition report")
  policy = _policy_frozen_summary(
    policy_report,
    frozen_joint_vel_threshold=frozen_joint_vel_threshold,
    frozen_root_ang_vel_threshold=frozen_root_ang_vel_threshold,
    paused_advance_step_threshold=paused_advance_step_threshold,
  )
  support = _support_summary(
    velocity_report, support_gap_threshold=support_gap_threshold
  )
  mujoco = _mujoco_summary(mujoco_report, nonzero_ctrl_threshold=nonzero_ctrl_threshold)
  decision = {
    "start_paused": support["start_paused"] == 1,
    "policy_steps_while_physics_paused": support["start_paused"] == 1
    and bool(policy["paused_policy_thread_advanced"]),
    "support_gap_before_run": bool(support["support_gap_before_run"]),
    "first_physics_step_is_dynamic": bool(
      mujoco["decision"]["first_physics_step_is_dynamic"]
    ),
    "first_step_no_contact": bool(mujoco["first_step_no_contact"]),
    "first_step_nonzero_ctrl": bool(mujoco["first_step_nonzero_ctrl"]),
    "motion_before_contact": bool(mujoco["decision"]["motion_before_contact"]),
    "motion_before_large_ctrl": bool(mujoco["decision"]["motion_before_large_ctrl"]),
    "elastic_force_disabled": bool(mujoco["decision"]["elastic_force_disabled"]),
  }
  classification = _classify(decision)
  return {
    "schema_version": 1,
    "evidence_dir": str(evidence_dir),
    "inputs": {
      "velocity_report": str(velocity_report_path),
      "policy_io_report": str(policy_io_report_path),
      "mujoco_transition_report": str(mujoco_transition_report_path),
      "bridge_source": None if bridge_source is None else str(bridge_source),
    },
    "thresholds": {
      "frozen_joint_vel_l2": frozen_joint_vel_threshold,
      "frozen_root_ang_vel_l2": frozen_root_ang_vel_threshold,
      "paused_advance_policy_step": paused_advance_step_threshold,
      "support_gap_m": support_gap_threshold,
      "nonzero_ctrl_l2": nonzero_ctrl_threshold,
    },
    "classification": classification,
    "decision": decision,
    "policy_frozen_state": policy,
    "support_state": support,
    "mujoco_transition": mujoco,
    "lowcmd_ctrl_source": _bridge_summary(bridge_source),
    "interpretation": (
      "This audit does not accept the Velocity policy. It checks whether the "
      "captured run is contaminated by controller steps while MuJoCo physics is "
      "paused, an initial floating support gap, or nonzero low-level ctrl at "
      "the first physics step. If these flags are true, the next sim2sim gate "
      "must explicitly settle/contact the robot and align controller start with "
      "MuJoCo Run before using the run as policy-quality evidence."
    ),
    "recommended_next_probe": (
      "Run an official FixStand-to-Velocity gate that logs lowcmd/ctrl at the "
      "Run boundary and proves contact/settling before policy acceptance; do "
      "not unlock Mimic or hardware from this evidence."
      if classification != "insufficient_handoff_evidence"
      else "Collect stronger Run-boundary evidence before changing policy or hardware gates."
    ),
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Audit G1 Velocity paused-to-run handoff evidence."
  )
  parser.add_argument("--evidence-dir", required=True, type=Path)
  parser.add_argument("--velocity-report", required=True, type=Path)
  parser.add_argument("--policy-io-report", required=True, type=Path)
  parser.add_argument("--mujoco-transition-report", required=True, type=Path)
  parser.add_argument("--bridge-source", type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--frozen-joint-vel-threshold", type=float, default=1e-6)
  parser.add_argument("--frozen-root-ang-vel-threshold", type=float, default=1e-6)
  parser.add_argument("--paused-advance-step-threshold", type=int, default=10)
  parser.add_argument("--support-gap-threshold", type=float, default=0.005)
  parser.add_argument("--nonzero-ctrl-threshold", type=float, default=1.0)
  parser.add_argument("--expect-paused-policy", action="store_true")
  parser.add_argument("--expect-support-gap", action="store_true")
  parser.add_argument("--expect-first-step-dynamic", action="store_true")
  parser.add_argument("--expect-first-step-no-contact", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = analyze_run_handoff_audit(
    evidence_dir=args.evidence_dir,
    velocity_report_path=args.velocity_report,
    policy_io_report_path=args.policy_io_report,
    mujoco_transition_report_path=args.mujoco_transition_report,
    bridge_source=args.bridge_source,
    frozen_joint_vel_threshold=args.frozen_joint_vel_threshold,
    frozen_root_ang_vel_threshold=args.frozen_root_ang_vel_threshold,
    paused_advance_step_threshold=args.paused_advance_step_threshold,
    support_gap_threshold=args.support_gap_threshold,
    nonzero_ctrl_threshold=args.nonzero_ctrl_threshold,
  )
  text = json.dumps(report, indent=2, sort_keys=True)
  if args.report_out is not None:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(text + "\n", encoding="utf-8")
  print(text)
  decision = report["decision"]
  if args.expect_paused_policy and not decision["policy_steps_while_physics_paused"]:
    return 1
  if args.expect_support_gap and not decision["support_gap_before_run"]:
    return 1
  if args.expect_first_step_dynamic and not decision["first_physics_step_is_dynamic"]:
    return 1
  if args.expect_first_step_no_contact and not decision["first_step_no_contact"]:
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
