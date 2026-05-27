from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_dense_onset import (
  analyze_dense_onset,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_dense_onset.py"


def _l2(values: list[float]) -> float:
  return math.sqrt(sum(value * value for value in values))


def _term(values: list[float]) -> dict:
  return {
    "dim": len(values),
    "l2": _l2(values),
    "max_abs": max((abs(value) for value in values), default=0.0),
    "values": values,
  }


def _trace(
  step: int,
  *,
  raw: list[float],
  last_action: list[float],
  command: list[float] | None = None,
  gait_phase: list[float] | None = None,
  joint_vel: list[float] | None = None,
  root_ang_vel: list[float] | None = None,
  gravity: list[float] | None = None,
) -> dict:
  command = command or [0.0, 0.0, 0.0]
  gait_phase = gait_phase or [0.0, 0.0]
  joint_vel = joint_vel or [0.0] * 29
  root_ang_vel = root_ang_vel or [0.0, 0.0, 0.0]
  gravity = gravity or [0.0, 0.0, -1.0]
  processed = [value * 0.5 for value in raw]
  return {
    "line": step + 10,
    "step": step,
    "raw_action": raw,
    "processed_action": processed,
    "raw_action_l2": _l2(raw),
    "processed_action_l2": _l2(processed),
    "obs_terms": {
      "base_ang_vel": _term(root_ang_vel),
      "projected_gravity": _term(gravity),
      "velocity_commands": _term(command),
      "gait_phase": _term(gait_phase),
      "joint_pos_rel": _term([0.0] * 29),
      "joint_vel_rel": _term(joint_vel),
      "last_action": _term(last_action),
    },
    "root_ang_vel": root_ang_vel,
  }


def _write_report(path: Path, traces: list[dict]) -> None:
  path.write_text(
    json.dumps({"trace_count": len(traces), "traces": traces}) + "\n",
    encoding="utf-8",
  )


def test_dense_onset_classifies_observed_motion_before_large_previous_action(
  tmp_path: Path,
) -> None:
  quiet_raw = [0.2] * 29
  large_raw = [3.0] * 29
  traces = [
    _trace(1, raw=quiet_raw, last_action=[0.0] * 29),
    _trace(50, raw=quiet_raw, last_action=quiet_raw),
    _trace(
      916,
      raw=large_raw,
      last_action=quiet_raw,
      joint_vel=[2.0] + [0.0] * 28,
      root_ang_vel=[0.1, 0.0, 0.0],
    ),
    _trace(
      917,
      raw=[4.0] * 29,
      last_action=large_raw,
      joint_vel=[5.0] + [0.0] * 28,
      root_ang_vel=[1.0, 0.0, 0.0],
    ),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)

  report = analyze_dense_onset(trace_report=trace_report)

  assert report["onset_order"] == "observed_motion_before_large_previous_action"
  assert report["first_dynamic_onset"]["step"] == 916
  assert report["first_large_current_raw_action"]["step"] == 916
  assert report["first_large_previous_action"]["step"] == 917
  assert report["decision"]["first_onset_has_quiet_previous_action"] is True
  assert report["decision"]["first_onset_current_raw_action_is_large"] is True
  assert report["decision"]["observed_motion_precedes_large_previous_action"] is True
  assert report["decision"]["observed_motion_before_policy_response"] is False


def test_dense_onset_classifies_motion_before_policy_response(
  tmp_path: Path,
) -> None:
  quiet_raw = [0.1] * 29
  large_raw = [3.0] * 29
  traces = [
    _trace(1, raw=quiet_raw, last_action=[0.0] * 29),
    _trace(50, raw=quiet_raw, last_action=quiet_raw),
    _trace(
      840,
      raw=quiet_raw,
      last_action=quiet_raw,
      joint_vel=[12.0] + [0.0] * 28,
      root_ang_vel=[0.17, 0.0, 0.0],
    ),
    _trace(
      841,
      raw=large_raw,
      last_action=quiet_raw,
      joint_vel=[20.0] + [0.0] * 28,
      root_ang_vel=[2.0, 0.0, 0.0],
    ),
    _trace(
      842,
      raw=[4.0] * 29,
      last_action=large_raw,
      joint_vel=[25.0] + [0.0] * 28,
      root_ang_vel=[3.0, 0.0, 0.0],
    ),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)

  report = analyze_dense_onset(trace_report=trace_report)

  assert report["onset_order"] == "observed_motion_before_policy_response"
  assert report["first_dynamic_onset"]["step"] == 840
  assert report["first_large_current_raw_action"]["step"] == 841
  assert report["first_large_previous_action"]["step"] == 842
  assert report["decision"]["first_onset_has_quiet_previous_action"] is True
  assert report["decision"]["first_onset_current_raw_action_is_quiet"] is True
  assert report["decision"]["observed_motion_precedes_large_previous_action"] is True
  assert report["decision"]["observed_motion_before_policy_response"] is True


def test_dense_onset_rejects_large_previous_action_at_onset(tmp_path: Path) -> None:
  large_raw = [3.0] * 29
  traces = [
    _trace(1, raw=[0.2] * 29, last_action=[0.0] * 29),
    _trace(
      2,
      raw=large_raw,
      last_action=large_raw,
      joint_vel=[2.0] + [0.0] * 28,
    ),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)

  report = analyze_dense_onset(trace_report=trace_report)

  assert report["onset_order"] == "large_previous_action_at_first_observed_motion"
  assert report["decision"]["first_onset_has_quiet_previous_action"] is False
  assert report["decision"]["observed_motion_precedes_large_previous_action"] is False


def test_dense_onset_cli_fails_when_expected_order_is_not_met(
  tmp_path: Path,
) -> None:
  traces = [
    _trace(1, raw=[0.2] * 29, last_action=[0.0] * 29),
    _trace(
      2,
      raw=[3.0] * 29,
      last_action=[3.0] * 29,
      joint_vel=[2.0] + [0.0] * 28,
    ),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--trace-report",
      str(trace_report),
      "--expect-motion-before-large-previous-action",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 1
  assert '"observed_motion_precedes_large_previous_action": false' in proc.stdout


def test_dense_onset_cli_fails_on_command_or_phase_leak(tmp_path: Path) -> None:
  traces = [
    _trace(1, raw=[0.2] * 29, last_action=[0.0] * 29),
    _trace(
      2,
      raw=[3.0] * 29,
      last_action=[0.2] * 29,
      command=[1.0, 0.0, 0.0],
      gait_phase=[1.0, 0.0],
      joint_vel=[2.0] + [0.0] * 28,
    ),
    _trace(3, raw=[4.0] * 29, last_action=[3.0] * 29),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--trace-report",
      str(trace_report),
      "--expect-zero-command-phase-mask",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 1
  assert '"zero_command_observation": false' in proc.stdout
