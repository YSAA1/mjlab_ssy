from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_run_handoff_audit import (
  analyze_run_handoff_audit,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_run_handoff_audit.py"


def _write_json(path: Path, data: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def _policy_report() -> dict:
  return {
    "traces": [
      {
        "line": 10,
        "step": 1,
        "joint_pos_l2": 0.0,
        "joint_vel_l2": 0.0,
        "root_ang_vel": [0.0, 0.0, 0.0],
        "raw_action_l2": 1.0,
        "processed_action_l2": 1.4,
        "obs_terms": {
          "velocity_commands": {"l2": 0.0},
          "gait_phase": {"l2": 0.0},
          "last_action": {"l2": 0.0},
        },
      },
      {
        "line": 12,
        "step": 50,
        "joint_pos_l2": 0.0,
        "joint_vel_l2": 0.0,
        "root_ang_vel": [0.0, 0.0, 0.0],
        "raw_action_l2": 1.8,
        "processed_action_l2": 1.2,
        "obs_terms": {
          "velocity_commands": {"l2": 0.0},
          "gait_phase": {"l2": 0.0},
          "last_action": {"l2": 1.1},
        },
      },
      {
        "line": 20,
        "step": 850,
        "joint_pos_l2": 7.6,
        "joint_vel_l2": 279.0,
        "root_ang_vel": [0.0, 7.8, 1.5],
        "raw_action_l2": 25.0,
        "processed_action_l2": 11.0,
      },
    ]
  }


def _velocity_report(
  *, start_paused: int = 1, min_foot_surface_z: float = 0.027
) -> dict:
  return {
    "sim_config": {"start_paused": start_paused, "enable_elastic_band": 0},
    "initial_contact": {
      "root_z": 0.783675,
      "min_foot_surface_z": min_foot_surface_z,
      "floor_clearance_passed": True,
    },
  }


def _mujoco_report(*, first_ncon: int = 0, dynamic: bool = True) -> dict:
  return {
    "trace_count": 3,
    "first_trace": {
      "step": 1,
      "sim_time": 0.002,
      "qvel_l2": 11.9 if dynamic else 0.0,
      "root_ang_vel_l2": 0.17 if dynamic else 0.0,
      "ctrl_l2": 17.6,
      "ncon": first_ncon,
    },
    "first_dynamic_trace": {
      "step": 1,
      "sim_time": 0.002,
      "qvel_l2": 11.9,
      "ctrl_l2": 17.6,
      "ncon": first_ncon,
    }
    if dynamic
    else None,
    "first_contact_trace": {"step": 8, "ncon": 2},
    "first_large_ctrl_trace": {"step": 5, "ctrl_l2": 120.0},
    "root_z_drop_before_first_contact": -0.0008,
    "decision": {
      "first_physics_step_is_dynamic": dynamic,
      "motion_before_contact": dynamic,
      "motion_before_large_ctrl": dynamic,
      "elastic_force_disabled": True,
    },
  }


def test_run_handoff_audit_classifies_paused_floating_handoff(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  velocity = tmp_path / "velocity.json"
  policy = tmp_path / "policy.json"
  mujoco = tmp_path / "mujoco.json"
  bridge = tmp_path / "unitree_sdk2_bridge.h"
  _write_json(velocity, _velocity_report())
  _write_json(policy, _policy_report())
  _write_json(mujoco, _mujoco_report())
  bridge.write_text(
    "mj_data_->ctrl[i] = m.tau() + m.kp() + m.kd();\n", encoding="utf-8"
  )

  report = analyze_run_handoff_audit(
    evidence_dir=evidence,
    velocity_report_path=velocity,
    policy_io_report_path=policy,
    mujoco_transition_report_path=mujoco,
    bridge_source=bridge,
  )

  assert report["classification"] == "paused_policy_with_floating_support_handoff"
  assert report["decision"]["policy_steps_while_physics_paused"] is True
  assert report["decision"]["support_gap_before_run"] is True
  assert report["decision"]["first_physics_step_is_dynamic"] is True
  assert report["decision"]["first_step_no_contact"] is True
  assert report["lowcmd_ctrl_source"]["formula_found"] is True


def test_run_handoff_audit_does_not_overclassify_without_handoff_flags(
  tmp_path: Path,
) -> None:
  velocity = tmp_path / "velocity.json"
  policy = tmp_path / "policy.json"
  mujoco = tmp_path / "mujoco.json"
  _write_json(velocity, _velocity_report(start_paused=0, min_foot_surface_z=0.0))
  _write_json(policy, {"traces": [_policy_report()["traces"][0]]})
  _write_json(mujoco, _mujoco_report(first_ncon=2, dynamic=False))

  report = analyze_run_handoff_audit(
    evidence_dir=tmp_path / "evidence",
    velocity_report_path=velocity,
    policy_io_report_path=policy,
    mujoco_transition_report_path=mujoco,
  )

  assert report["classification"] == "insufficient_handoff_evidence"
  assert report["decision"]["policy_steps_while_physics_paused"] is False
  assert report["decision"]["support_gap_before_run"] is False
  assert report["decision"]["first_physics_step_is_dynamic"] is False
  assert report["decision"]["first_step_no_contact"] is False


def test_run_handoff_audit_cli_writes_report(tmp_path: Path) -> None:
  velocity = tmp_path / "velocity.json"
  policy = tmp_path / "policy.json"
  mujoco = tmp_path / "mujoco.json"
  report_out = tmp_path / "report.json"
  _write_json(velocity, _velocity_report())
  _write_json(policy, _policy_report())
  _write_json(mujoco, _mujoco_report())

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--evidence-dir",
      str(tmp_path / "evidence"),
      "--velocity-report",
      str(velocity),
      "--policy-io-report",
      str(policy),
      "--mujoco-transition-report",
      str(mujoco),
      "--report-out",
      str(report_out),
      "--expect-paused-policy",
      "--expect-support-gap",
      "--expect-first-step-dynamic",
      "--expect-first-step-no-contact",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  saved = json.loads(report_out.read_text(encoding="utf-8"))
  assert saved["classification"] == "paused_policy_with_floating_support_handoff"
