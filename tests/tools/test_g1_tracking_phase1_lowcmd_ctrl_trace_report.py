from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_lowcmd_ctrl_trace_report import (
  analyze_lowcmd_ctrl_trace,
  parse_lowcmd_ctrl_traces,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_lowcmd_ctrl_trace_report.py"


def _write_log(path: Path) -> None:
  path.write_text(
    "\n".join(
      [
        "ignored",
        "[PHASE1_SIM] event=lowcmd_ctrl_trace sample=1 sim_time=0.000000 "
        "ctrl_l2=0.000000 ctrl_max=0.000000 tau_l2=0.000000 tau_max=0.000000 "
        "pos_term_l2=0.000000 pos_term_max=0.000000 vel_term_l2=0.000000 "
        "vel_term_max=0.000000 q_error_l2=0.000000 q_error_max=0.000000 "
        "dq_error_l2=0.000000 dq_error_max=0.000000 q_cmd_l2=1.000000 "
        "q_sensor_l2=1.000000 dq_cmd_l2=0.000000 dq_sensor_l2=0.000000 "
        "kp_l2=200.000000 kd_l2=10.000000 top_index=-1 top_ctrl=0.000000 "
        "top_tau=0.000000 top_pos_term=0.000000 top_vel_term=0.000000 "
        "top_q_cmd=0.000000 top_q_sensor=0.000000 top_q_error=0.000000 "
        "top_dq_cmd=0.000000 top_dq_sensor=0.000000 top_dq_error=0.000000 "
        "top_kp=0.000000 top_kd=0.000000",
        "[PHASE1_SIM] event=lowcmd_ctrl_trace sample=2 sim_time=0.002000 "
        "ctrl_l2=64.000000 ctrl_max=60.000000 tau_l2=0.000000 tau_max=0.000000 "
        "pos_term_l2=63.000000 pos_term_max=59.000000 vel_term_l2=8.000000 "
        "vel_term_max=8.000000 q_error_l2=0.300000 q_error_max=0.250000 "
        "dq_error_l2=2.000000 dq_error_max=1.800000 q_cmd_l2=1.500000 "
        "q_sensor_l2=1.450000 dq_cmd_l2=0.000000 dq_sensor_l2=2.000000 "
        "kp_l2=220.000000 kd_l2=12.000000 top_index=3 top_ctrl=60.000000 "
        "top_tau=0.000000 top_pos_term=59.000000 top_vel_term=1.000000 "
        "top_q_cmd=0.600000 top_q_sensor=0.350000 top_q_error=0.250000 "
        "top_dq_cmd=0.000000 top_dq_sensor=-0.200000 top_dq_error=0.200000 "
        "top_kp=236.000000 top_kd=5.000000",
        "[PHASE1_SIM] event=lowcmd_ctrl_trace sample=3 sim_time=0.004000 "
        "ctrl_l2=80.000000 ctrl_max=75.000000 tau_l2=0.000000 tau_max=0.000000 "
        "pos_term_l2=20.000000 pos_term_max=18.000000 vel_term_l2=77.000000 "
        "vel_term_max=70.000000 q_error_l2=0.100000 q_error_max=0.080000 "
        "dq_error_l2=10.000000 dq_error_max=9.000000 q_cmd_l2=1.500000 "
        "q_sensor_l2=1.450000 dq_cmd_l2=0.000000 dq_sensor_l2=10.000000 "
        "kp_l2=220.000000 kd_l2=12.000000 top_index=5 top_ctrl=-75.000000 "
        "top_tau=0.000000 top_pos_term=-5.000000 top_vel_term=-70.000000 "
        "top_q_cmd=0.100000 top_q_sensor=0.180000 top_q_error=-0.080000 "
        "top_dq_cmd=0.000000 top_dq_sensor=9.000000 top_dq_error=-9.000000 "
        "top_kp=62.500000 top_kd=7.777778",
      ]
    )
    + "\n",
    encoding="utf-8",
  )


def test_parse_lowcmd_ctrl_trace_extracts_pd_term(tmp_path: Path) -> None:
  log_path = tmp_path / "unitree_mujoco.log"
  _write_log(log_path)

  traces = parse_lowcmd_ctrl_traces(log_path)

  assert len(traces) == 3
  assert traces[1]["line"] == 3
  assert traces[1]["sample"] == 2
  assert traces[1]["top_index"] == 3
  assert round(traces[1]["pd_term_l2"], 3) == 63.506


def test_lowcmd_ctrl_trace_report_classifies_position_pd_handoff(
  tmp_path: Path,
) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  _write_log(evidence / "unitree_mujoco.log")

  report = analyze_lowcmd_ctrl_trace(evidence)

  assert report["trace_count"] == 3
  assert report["classification"] == "position_pd_ctrl_handoff"
  assert report["first_nonzero_ctrl_trace"]["sample"] == 2
  assert report["decision"]["first_nonzero_has_pd_source"] is True
  assert report["decision"]["first_nonzero_tau_near_zero"] is True
  assert report["maxima"]["vel_term_l2"]["sample"] == 3


def test_lowcmd_ctrl_trace_report_cli_writes_json(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  _write_log(evidence / "unitree_mujoco.log")
  report_out = tmp_path / "lowcmd_ctrl_trace.json"

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--evidence-dir",
      str(evidence),
      "--report-out",
      str(report_out),
      "--expect-trace",
      "--expect-nonzero-ctrl",
      "--expect-pd-source",
      "--expect-tau-near-zero",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  report = json.loads(report_out.read_text(encoding="utf-8"))
  assert report["classification"] == "position_pd_ctrl_handoff"
