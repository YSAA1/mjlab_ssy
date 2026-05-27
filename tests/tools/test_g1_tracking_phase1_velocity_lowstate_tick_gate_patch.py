from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from mjlab.scripts.g1_tracking_phase1_velocity_lowstate_tick_gate_patch import (
  TICK_GATE_ENV_VAR,
  TICK_GATE_MARKER,
  TICK_GATE_TIMEOUT_ENV_VAR,
  LowstateTickGatePatchError,
  patch_state_rlbase_h,
)
from mjlab.scripts.g1_tracking_phase1_velocity_policy_start_gate_patch import (
  GATE_MARKER,
)
from mjlab.scripts.g1_tracking_phase1_velocity_policy_start_gate_patch import (
  patch_state_rlbase_h as patch_start_gate,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_lowstate_tick_gate_patch.py"

STATE_RLBASE_FIXTURE = """// Copyright (c) 2025, Unitree Robotics Co., Ltd.
#pragma once

#include "FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <cmath>
#include <spdlog/spdlog.h>

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);

    void enter()
    {
        env->robot->update();
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();

            while (policy_thread_running)
            {
                env->step();

                // Sleep
                std::this_thread::sleep_until(sleepTill);
                sleepTill += dt;
            }
        });
    }

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::thread policy_thread;
    bool policy_thread_running = false;
};
"""


def _start_gate_fixture() -> str:
  patched, changed = patch_start_gate(STATE_RLBASE_FIXTURE)
  assert changed is True
  assert GATE_MARKER in patched
  return patched


def test_patch_injects_lowstate_tick_gate_after_start_gate() -> None:
  patched, changed = patch_state_rlbase_h(_start_gate_fixture())

  assert changed is True
  assert TICK_GATE_MARKER in patched
  assert TICK_GATE_ENV_VAR in patched
  assert TICK_GATE_TIMEOUT_ENV_VAR in patched
  assert "#include <mutex>" in patched
  assert "phase1_lowstate_tick_gate_enabled()" in patched
  assert "phase1_read_lowstate_tick" in patched
  assert "[PHASE1] event=lowstate_tick_gate_start" in patched
  assert "[PHASE1] event=lowstate_tick_gate_release" in patched
  assert "[PHASE1] event=lowstate_tick_gate_timeout" in patched
  assert patched.index("phase1_wait_for_lowstate_tick") < patched.index("env->step();")
  tick_wait_index = patched.index("if(!phase1_wait_for_lowstate_tick())")
  reset_index = patched.index("sleepTill = clock::now() + dt;", tick_wait_index)
  step_index = patched.index("env->step();", reset_index)
  assert reset_index < step_index


def test_patch_is_idempotent() -> None:
  patched, changed = patch_state_rlbase_h(_start_gate_fixture())
  patched_again, changed_again = patch_state_rlbase_h(patched)

  assert changed is True
  assert changed_again is False
  assert patched_again == patched


def test_patch_repairs_existing_tick_gate_without_catchup_reset() -> None:
  patched, changed = patch_state_rlbase_h(_start_gate_fixture())
  assert changed is True
  stale = patched.replace(
    """                // Waiting for a lowstate tick can consume longer than dt;
                // anchor the next policy period to the fresh state sample.
                sleepTill = clock::now() + dt;
                env->step();
""",
    """                env->step();
""",
    1,
  )

  repaired, changed_again = patch_state_rlbase_h(stale)

  assert changed_again is True
  tick_wait_index = repaired.index("if(!phase1_wait_for_lowstate_tick())")
  reset_index = repaired.index("sleepTill = clock::now() + dt;", tick_wait_index)
  step_index = repaired.index("env->step();", reset_index)
  assert reset_index < step_index


def test_patch_rejects_missing_start_gate_prerequisite() -> None:
  with pytest.raises(LowstateTickGatePatchError, match="prerequisite"):
    patch_state_rlbase_h(STATE_RLBASE_FIXTURE)


def test_patch_rejects_missing_policy_loop_anchor() -> None:
  source = _start_gate_fixture().replace("env->step();", "env->step_now();")

  with pytest.raises(LowstateTickGatePatchError, match="policy loop anchor"):
    patch_state_rlbase_h(source)


def test_cli_dry_run_does_not_write(tmp_path: Path) -> None:
  source = tmp_path / "State_RLBase.h"
  source.write_text(_start_gate_fixture(), encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(CLI), "--state-rlbase", str(source)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert TICK_GATE_ENV_VAR in proc.stdout
  assert TICK_GATE_MARKER not in source.read_text(encoding="utf-8")


def test_cli_apply_writes_patch(tmp_path: Path) -> None:
  source = tmp_path / "State_RLBase.h"
  source.write_text(_start_gate_fixture(), encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(CLI), "--state-rlbase", str(source), "--apply"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert TICK_GATE_MARKER in source.read_text(encoding="utf-8")
  backup = source.with_suffix(source.suffix + f".{TICK_GATE_MARKER}.bak")
  assert GATE_MARKER in backup.read_text(encoding="utf-8")
