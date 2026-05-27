from __future__ import annotations

import json
import math
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_trace_chronology import (
  analyze_trace_chronology,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_trace_chronology.py"


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
    json.dumps(
      {
        "trace_count": len(traces),
        "first_unstable": {
          "line": traces[-1]["line"] + 1,
          "policy_step": traces[-1]["step"],
        },
        "selected_trace": traces[-1],
        "traces": traces,
      }
    )
    + "\n",
    encoding="utf-8",
  )


def _write_source_audit_files(root: Path) -> dict[str, Path]:
  worktree_manager = root / "manager_based_rl_env.py"
  worktree_observations = root / "observations.py"
  deploy_manager = root / "manager_based_rl_env.h"
  deploy_observations = root / "observations.h"
  worktree_manager.write_text(
    "self.action_manager.process_action(action.to(self.device))\n"
    "pre_reset_obs = self.observation_manager.compute(update_history=True)\n",
    encoding="utf-8",
  )
  worktree_observations.write_text(
    "def last_action(env):\n  return env.action_manager.action\n",
    encoding="utf-8",
  )
  deploy_manager.write_text(
    "auto obs = observation_manager->compute();\n"
    "auto action = alg->act(obs);\n"
    "action_manager->process_action(action);\n",
    encoding="utf-8",
  )
  deploy_observations.write_text(
    "REGISTER_OBSERVATION(last_action)\nauto data = env->action_manager->action();\n",
    encoding="utf-8",
  )
  return {
    "worktree_manager": worktree_manager,
    "worktree_observations": worktree_observations,
    "deploy_manager": deploy_manager,
    "deploy_observations": deploy_observations,
  }


def test_trace_chronology_confirms_early_last_action_lag(tmp_path: Path) -> None:
  raw1 = [1.0] * 29
  raw2 = [2.0] * 29
  raw3 = [3.0] * 29
  traces = [
    _trace(1, raw=raw1, last_action=[0.0] * 29),
    _trace(2, raw=raw2, last_action=raw1),
    _trace(3, raw=raw3, last_action=raw2),
    _trace(
      25,
      raw=[4.0] * 29,
      last_action=raw3,
      joint_vel=[25.0] + [0.0] * 28,
      root_ang_vel=[0.0, 1.5, 0.0],
      gravity=[0.2, 0.0, -0.95],
    ),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)
  source_paths = _write_source_audit_files(tmp_path)

  report = analyze_trace_chronology(trace_report=trace_report, **source_paths)

  assert report["decision"]["early_last_action_matches_previous_raw_action"] is True
  assert report["decision"]["zero_command_observation"] is True
  assert report["decision"]["zero_command_gait_phase_masked"] is True
  assert (
    report["decision"]["source_contract_matches_previous_action_policy_call_semantics"]
    is True
  )
  assert report["early_consecutive_last_action_pairs"][0]["gap_l2"] == 0.0
  assert report["first_crossings"][0]["name"] == "joint_vel_rel_l2"


def test_trace_chronology_cli_fails_on_last_action_mismatch(tmp_path: Path) -> None:
  traces = [
    _trace(1, raw=[1.0] * 29, last_action=[0.0] * 29),
    _trace(2, raw=[2.0] * 29, last_action=[9.0] * 29),
  ]
  trace_report = tmp_path / "trace.json"
  _write_report(trace_report, traces)

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--trace-report",
      str(trace_report),
      "--expect-early-last-action-match",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 1
  assert '"early_last_action_matches_previous_raw_action": false' in proc.stdout


def test_trace_chronology_cli_fails_on_unmasked_zero_command_phase(
  tmp_path: Path,
) -> None:
  traces = [
    _trace(1, raw=[1.0] * 29, last_action=[0.0] * 29, gait_phase=[1.0, 0.0]),
    _trace(2, raw=[2.0] * 29, last_action=[1.0] * 29, gait_phase=[0.0, 1.0]),
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
  assert '"zero_command_gait_phase_masked": false' in proc.stdout
