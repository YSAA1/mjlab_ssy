from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_mujoco_transition_trace_report import (
  analyze_mujoco_transition_trace,
  parse_mujoco_transition_traces,
)

ROOT = Path(__file__).resolve().parents[2]
REPORT_CLI = ROOT / "scripts/tools/g1_tracking_phase1_mujoco_transition_trace_report.py"


def _write_log(path: Path) -> None:
  path.write_text(
    "\n".join(
      [
        "ignored",
        "[PHASE1_SIM] event=mujoco_transition_trace step=1 sim_time=0.002000 "
        "root_pos=(0.000000,0.000000,0.783000) root_lin_vel=(0.000000,0.000000,-0.002000) "
        "root_ang_vel=(0.000000,-0.160000,0.060000) root_lin_vel_l2=0.004000 "
        "root_ang_vel_l2=0.171000 qvel_l2=11.950000 qvel_max=6.900000 "
        "ctrl_l2=17.600000 ctrl_max=12.800000 ncon=0 elastic_config=0 "
        "elastic_enabled=1 elastic_length=0.000000 elastic_force=(0.000000,0.000000,0.000000) "
        "elastic_force_l2=0.000000",
        "[PHASE1_SIM] event=mujoco_transition_trace step=5 sim_time=0.010000 "
        "root_pos=(0.000100,0.000000,0.783100) root_lin_vel=(0.000000,0.000000,-0.100000) "
        "root_ang_vel=(0.000000,0.000000,0.100000) root_lin_vel_l2=0.100000 "
        "root_ang_vel_l2=0.100000 qvel_l2=20.000000 qvel_max=10.000000 "
        "ctrl_l2=120.000000 ctrl_max=90.000000 ncon=0 elastic_config=0 "
        "elastic_enabled=1 elastic_length=0.000000 elastic_force=(0.000000,0.000000,0.000000) "
        "elastic_force_l2=0.000000",
        "[PHASE1_SIM] event=mujoco_transition_trace step=8 sim_time=0.016000 "
        "root_pos=(0.000200,0.000000,0.782800) root_lin_vel=(0.000000,0.000000,-0.040000) "
        "root_ang_vel=(0.000000,0.000000,0.800000) root_lin_vel_l2=0.040000 "
        "root_ang_vel_l2=0.800000 qvel_l2=30.000000 qvel_max=20.000000 "
        "ctrl_l2=130.000000 ctrl_max=95.000000 ncon=2 elastic_config=0 "
        "elastic_enabled=1 elastic_length=0.000000 elastic_force=(0.000000,0.000000,0.000000) "
        "elastic_force_l2=0.000000",
      ]
    )
    + "\n",
    encoding="utf-8",
  )


def test_parse_mujoco_transition_traces_extracts_vectors(tmp_path: Path) -> None:
  log_path = tmp_path / "unitree_mujoco.log"
  _write_log(log_path)

  traces = parse_mujoco_transition_traces(log_path)

  assert len(traces) == 3
  assert traces[0]["line"] == 2
  assert traces[0]["step"] == 1
  assert traces[0]["root_pos"] == [0.0, 0.0, 0.783]
  assert traces[0]["qvel_l2"] == 11.95


def test_mujoco_transition_trace_report_classifies_order(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  _write_log(evidence / "unitree_mujoco.log")

  report = analyze_mujoco_transition_trace(evidence)

  assert report["trace_count"] == 3
  assert report["first_dynamic_trace"]["step"] == 1
  assert report["first_contact_trace"]["step"] == 8
  assert report["first_large_ctrl_trace"]["step"] == 5
  assert report["decision"]["first_physics_step_is_dynamic"] is True
  assert report["decision"]["motion_before_contact"] is True
  assert report["decision"]["motion_before_large_ctrl"] is True
  assert report["decision"]["elastic_force_disabled"] is True


def test_mujoco_transition_trace_report_cli_writes_json(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  _write_log(evidence / "unitree_mujoco.log")
  report_out = tmp_path / "report.json"

  proc = subprocess.run(
    [
      sys.executable,
      str(REPORT_CLI),
      "--evidence-dir",
      str(evidence),
      "--report-out",
      str(report_out),
      "--expect-trace",
      "--expect-first-step-dynamic",
      "--expect-motion-before-contact",
      "--expect-motion-before-large-ctrl",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert json.loads(report_out.read_text(encoding="utf-8"))["trace_count"] == 3
