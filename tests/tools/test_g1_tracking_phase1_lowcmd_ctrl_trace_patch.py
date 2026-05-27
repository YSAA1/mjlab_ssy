from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from mjlab.scripts.g1_tracking_phase1_lowcmd_ctrl_trace_patch import (
  TRACE_MARKER,
  LowcmdCtrlTracePatchError,
  patch_unitree_sdk2_bridge_h,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_lowcmd_ctrl_trace_patch.py"

BRIDGE_FIXTURE = """#pragma once

#include <mujoco/mujoco.h>

#include <iostream>

#include "param.h"

#define MOTOR_SENSOR_NUM 3

class UnitreeSDK2BridgeBase
{
public:
    virtual void run()
    {
        if(!mj_data_) return;
        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }
    }
};
"""


def test_lowcmd_ctrl_trace_patch_injects_formula_decomposition() -> None:
  patched, changed = patch_unitree_sdk2_bridge_h(BRIDGE_FIXTURE)

  assert changed is True
  assert TRACE_MARKER in patched
  assert "#include <cmath>" in patched
  assert "event=lowcmd_ctrl_trace" in patched
  assert "pos_term_l2=%.6f" in patched
  assert "vel_term_l2=%.6f" in patched
  assert "top_q_error=%.6f" in patched
  assert "const double ctrl = tau + pos_term + vel_term;" in patched
  assert "phase1_lowcmd_ctrl_trace(mj_data_, phase1_lowcmd_ctrl_summary);" in patched


def test_lowcmd_ctrl_trace_patch_is_idempotent() -> None:
  patched, changed = patch_unitree_sdk2_bridge_h(BRIDGE_FIXTURE)
  patched_again, changed_again = patch_unitree_sdk2_bridge_h(patched)

  assert changed is True
  assert changed_again is False
  assert patched_again == patched


def test_lowcmd_ctrl_trace_patch_rejects_missing_loop_anchor() -> None:
  fixture = BRIDGE_FIXTURE.replace("            for(int i(0); i<num_motor_; i++)", "")

  with pytest.raises(LowcmdCtrlTracePatchError, match="missing lowcmd ctrl loop"):
    patch_unitree_sdk2_bridge_h(fixture)


def test_lowcmd_ctrl_trace_patch_cli_dry_run_does_not_write(tmp_path: Path) -> None:
  source = tmp_path / "unitree_sdk2_bridge.h"
  source.write_text(BRIDGE_FIXTURE, encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(CLI), "--bridge-header", str(source)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert TRACE_MARKER not in source.read_text(encoding="utf-8")


def test_lowcmd_ctrl_trace_patch_cli_apply_writes_backup(tmp_path: Path) -> None:
  source = tmp_path / "unitree_sdk2_bridge.h"
  source.write_text(BRIDGE_FIXTURE, encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(CLI), "--bridge-header", str(source), "--apply"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert TRACE_MARKER in source.read_text(encoding="utf-8")
  backup = source.with_suffix(source.suffix + f".{TRACE_MARKER}.bak")
  assert backup.read_text(encoding="utf-8") == BRIDGE_FIXTURE
