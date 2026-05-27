"""Patch local Unitree MuJoCo lowcmd-to-ctrl tracing for phase-1 diagnosis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DEFAULT_BRIDGE_HEADER = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/unitree_sdk2_bridge.h"
)

TRACE_MARKER = "phase1_lowcmd_ctrl_trace_v1"

INCLUDE_ANCHOR = "#include <iostream>\n"
INCLUDE_PATCH = "#include <cmath>\n#include <cstdio>\n#include <iostream>\n"

HELPER_ANCHOR = "#define MOTOR_SENSOR_NUM 3\n\nclass UnitreeSDK2BridgeBase"
HELPER_PATCH = f"""#define MOTOR_SENSOR_NUM 3

// {TRACE_MARKER}: local-only bridge trace for the exact lowcmd -> ctrl formula.
struct Phase1LowcmdCtrlSummary
{{
    double tau_l2 = 0.0;
    double tau_max = 0.0;
    double pos_term_l2 = 0.0;
    double pos_term_max = 0.0;
    double vel_term_l2 = 0.0;
    double vel_term_max = 0.0;
    double ctrl_l2 = 0.0;
    double ctrl_max = 0.0;
    double q_error_l2 = 0.0;
    double q_error_max = 0.0;
    double dq_error_l2 = 0.0;
    double dq_error_max = 0.0;
    double q_cmd_l2 = 0.0;
    double q_sensor_l2 = 0.0;
    double dq_cmd_l2 = 0.0;
    double dq_sensor_l2 = 0.0;
    double kp_l2 = 0.0;
    double kd_l2 = 0.0;
    int top_index = -1;
    double top_ctrl = 0.0;
    double top_tau = 0.0;
    double top_pos_term = 0.0;
    double top_vel_term = 0.0;
    double top_q_cmd = 0.0;
    double top_q_sensor = 0.0;
    double top_q_error = 0.0;
    double top_dq_cmd = 0.0;
    double top_dq_sensor = 0.0;
    double top_dq_error = 0.0;
    double top_kp = 0.0;
    double top_kd = 0.0;
}};

inline double phase1_square(double value)
{{
    return value * value;
}}

inline void phase1_lowcmd_ctrl_accumulate(
    Phase1LowcmdCtrlSummary & summary,
    int index,
    double tau,
    double kp,
    double kd,
    double q_cmd,
    double q_sensor,
    double dq_cmd,
    double dq_sensor,
    double pos_term,
    double vel_term,
    double ctrl)
{{
    const double q_error = q_cmd - q_sensor;
    const double dq_error = dq_cmd - dq_sensor;
    summary.tau_l2 += phase1_square(tau);
    summary.pos_term_l2 += phase1_square(pos_term);
    summary.vel_term_l2 += phase1_square(vel_term);
    summary.ctrl_l2 += phase1_square(ctrl);
    summary.q_error_l2 += phase1_square(q_error);
    summary.dq_error_l2 += phase1_square(dq_error);
    summary.q_cmd_l2 += phase1_square(q_cmd);
    summary.q_sensor_l2 += phase1_square(q_sensor);
    summary.dq_cmd_l2 += phase1_square(dq_cmd);
    summary.dq_sensor_l2 += phase1_square(dq_sensor);
    summary.kp_l2 += phase1_square(kp);
    summary.kd_l2 += phase1_square(kd);

    const double abs_tau = std::abs(tau);
    const double abs_pos_term = std::abs(pos_term);
    const double abs_vel_term = std::abs(vel_term);
    const double abs_ctrl = std::abs(ctrl);
    const double abs_q_error = std::abs(q_error);
    const double abs_dq_error = std::abs(dq_error);
    if (abs_tau > summary.tau_max) summary.tau_max = abs_tau;
    if (abs_pos_term > summary.pos_term_max) summary.pos_term_max = abs_pos_term;
    if (abs_vel_term > summary.vel_term_max) summary.vel_term_max = abs_vel_term;
    if (abs_q_error > summary.q_error_max) summary.q_error_max = abs_q_error;
    if (abs_dq_error > summary.dq_error_max) summary.dq_error_max = abs_dq_error;
    if (abs_ctrl > summary.ctrl_max)
    {{
        summary.ctrl_max = abs_ctrl;
        summary.top_index = index;
        summary.top_ctrl = ctrl;
        summary.top_tau = tau;
        summary.top_pos_term = pos_term;
        summary.top_vel_term = vel_term;
        summary.top_q_cmd = q_cmd;
        summary.top_q_sensor = q_sensor;
        summary.top_q_error = q_error;
        summary.top_dq_cmd = dq_cmd;
        summary.top_dq_sensor = dq_sensor;
        summary.top_dq_error = dq_error;
        summary.top_kp = kp;
        summary.top_kd = kd;
    }}
}}

inline void phase1_lowcmd_ctrl_trace(
    const mjData * data,
    const Phase1LowcmdCtrlSummary & summary)
{{
    if (!data)
    {{
        return;
    }}

    static double last_time = -1.0;
    static long sample = 0;
    static bool dense_started = false;
    static long dense_until = -1;

    if (data->time < last_time)
    {{
        sample = 0;
        dense_started = false;
        dense_until = -1;
    }}
    const bool time_changed = data->time != last_time;
    if (!time_changed && sample > 0)
    {{
        return;
    }}
    last_time = data->time;
    ++sample;

    const double ctrl_l2 = std::sqrt(summary.ctrl_l2);
    if (ctrl_l2 > 1.0 && !dense_started)
    {{
        dense_started = true;
        dense_until = sample + 500;
    }}

    bool should_log = sample <= 10 || sample == 25 || sample == 50 ||
        sample == 100 || sample == 250 || sample == 500 || sample % 1000 == 0;
    if (dense_started && sample <= dense_until)
    {{
        should_log = true;
    }}
    if (ctrl_l2 > 50.0)
    {{
        should_log = true;
    }}
    if (!should_log)
    {{
        return;
    }}

    std::printf(
        "[PHASE1_SIM] event=lowcmd_ctrl_trace sample=%ld sim_time=%.6f "
        "ctrl_l2=%.6f ctrl_max=%.6f tau_l2=%.6f tau_max=%.6f "
        "pos_term_l2=%.6f pos_term_max=%.6f vel_term_l2=%.6f "
        "vel_term_max=%.6f q_error_l2=%.6f q_error_max=%.6f "
        "dq_error_l2=%.6f dq_error_max=%.6f q_cmd_l2=%.6f "
        "q_sensor_l2=%.6f dq_cmd_l2=%.6f dq_sensor_l2=%.6f "
        "kp_l2=%.6f kd_l2=%.6f top_index=%d top_ctrl=%.6f "
        "top_tau=%.6f top_pos_term=%.6f top_vel_term=%.6f "
        "top_q_cmd=%.6f top_q_sensor=%.6f top_q_error=%.6f "
        "top_dq_cmd=%.6f top_dq_sensor=%.6f top_dq_error=%.6f "
        "top_kp=%.6f top_kd=%.6f\\n",
        sample,
        static_cast<double>(data->time),
        ctrl_l2,
        summary.ctrl_max,
        std::sqrt(summary.tau_l2),
        summary.tau_max,
        std::sqrt(summary.pos_term_l2),
        summary.pos_term_max,
        std::sqrt(summary.vel_term_l2),
        summary.vel_term_max,
        std::sqrt(summary.q_error_l2),
        summary.q_error_max,
        std::sqrt(summary.dq_error_l2),
        summary.dq_error_max,
        std::sqrt(summary.q_cmd_l2),
        std::sqrt(summary.q_sensor_l2),
        std::sqrt(summary.dq_cmd_l2),
        std::sqrt(summary.dq_sensor_l2),
        std::sqrt(summary.kp_l2),
        std::sqrt(summary.kd_l2),
        summary.top_index,
        summary.top_ctrl,
        summary.top_tau,
        summary.top_pos_term,
        summary.top_vel_term,
        summary.top_q_cmd,
        summary.top_q_sensor,
        summary.top_q_error,
        summary.top_dq_cmd,
        summary.top_dq_sensor,
        summary.top_dq_error,
        summary.top_kp,
        summary.top_kd);
    std::fflush(stdout);
}}

class UnitreeSDK2BridgeBase"""

LOWCMD_LOOP_ANCHOR = """            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
"""

LOWCMD_LOOP_PATCH = """            Phase1LowcmdCtrlSummary phase1_lowcmd_ctrl_summary;
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                const double tau = static_cast<double>(m.tau());
                const double kp = static_cast<double>(m.kp());
                const double kd = static_cast<double>(m.kd());
                const double q_cmd = static_cast<double>(m.q());
                const double q_sensor = static_cast<double>(mj_data_->sensordata[i]);
                const double dq_cmd = static_cast<double>(m.dq());
                const double dq_sensor = static_cast<double>(mj_data_->sensordata[i + num_motor_]);
                const double pos_term = kp * (q_cmd - q_sensor);
                const double vel_term = kd * (dq_cmd - dq_sensor);
                const double ctrl = tau + pos_term + vel_term;
                mj_data_->ctrl[i] = ctrl;
                phase1_lowcmd_ctrl_accumulate(
                    phase1_lowcmd_ctrl_summary,
                    i,
                    tau,
                    kp,
                    kd,
                    q_cmd,
                    q_sensor,
                    dq_cmd,
                    dq_sensor,
                    pos_term,
                    vel_term,
                    ctrl);
            }
            phase1_lowcmd_ctrl_trace(mj_data_, phase1_lowcmd_ctrl_summary);
"""


class LowcmdCtrlTracePatchError(RuntimeError):
  """Raised when the local Unitree MuJoCo bridge source cannot be patched safely."""


def patch_unitree_sdk2_bridge_h(text: str) -> tuple[str, bool]:
  """Return patched ``unitree_sdk2_bridge.h`` text and whether it changed."""

  if TRACE_MARKER in text:
    return text, False

  if INCLUDE_PATCH not in text:
    if INCLUDE_ANCHOR not in text:
      raise LowcmdCtrlTracePatchError("missing include anchor")
    text = text.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)

  if HELPER_ANCHOR not in text:
    raise LowcmdCtrlTracePatchError("missing helper insertion anchor")
  text = text.replace(HELPER_ANCHOR, HELPER_PATCH, 1)

  if LOWCMD_LOOP_ANCHOR not in text:
    raise LowcmdCtrlTracePatchError("missing lowcmd ctrl loop anchor")
  text = text.replace(LOWCMD_LOOP_ANCHOR, LOWCMD_LOOP_PATCH, 1)
  return text, True


def apply_patch(path: Path, *, write: bool, backup: bool = True) -> dict[str, Any]:
  original = path.read_text(encoding="utf-8")
  patched, changed = patch_unitree_sdk2_bridge_h(original)
  backup_path = path.with_suffix(path.suffix + f".{TRACE_MARKER}.bak")
  backup_written = False
  if write and changed:
    if backup and not backup_path.exists():
      backup_path.write_text(original, encoding="utf-8")
      backup_written = True
    path.write_text(patched, encoding="utf-8")
  return {
    "path": str(path),
    "changed": changed,
    "write": write,
    "marker": TRACE_MARKER,
    "backup_path": str(backup_path) if backup else None,
    "backup_written": backup_written,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Patch local Unitree MuJoCo bridge with lowcmd-to-ctrl trace logs."
  )
  parser.add_argument("--bridge-header", type=Path, default=DEFAULT_BRIDGE_HEADER)
  parser.add_argument(
    "--apply",
    action="store_true",
    help="Write the patch. Without this flag, only reports whether a patch is needed.",
  )
  parser.add_argument(
    "--no-backup",
    action="store_true",
    help="Do not create a sibling .bak file before writing the local runtime patch.",
  )
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = apply_patch(args.bridge_header, write=args.apply, backup=not args.no_backup)
  print(json.dumps(report, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
