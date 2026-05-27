from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from mjlab.scripts.g1_tracking_phase1_velocity_policy_start_gate_patch import (
  GATE_ENV_VAR,
  GATE_MARKER,
  PolicyStartGatePatchError,
  patch_state_rlbase_h,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_policy_start_gate_patch.py"

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
        phase1_last_stable_log_slot_ = -1;

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
    int phase1_last_stable_log_slot_ = -1;
};
"""


def test_patch_injects_policy_start_gate() -> None:
  patched, changed = patch_state_rlbase_h(STATE_RLBASE_FIXTURE)

  assert changed is True
  assert GATE_MARKER in patched
  assert GATE_ENV_VAR in patched
  assert "#include <cstdlib>" in patched
  assert "phase1_policy_start_gate_seconds()" in patched
  assert "[PHASE1] event=policy_start_gate delay_seconds" in patched
  assert "[PHASE1] event=policy_start_gate_release" in patched
  assert patched.index("env->reset();") < patched.index("phase1_gate_seconds")
  assert patched.index("phase1_gate_seconds") < patched.index("env->step();")


def test_patch_is_idempotent() -> None:
  patched, changed = patch_state_rlbase_h(STATE_RLBASE_FIXTURE)
  patched_again, changed_again = patch_state_rlbase_h(patched)

  assert changed is True
  assert changed_again is False
  assert patched_again == patched


def test_patch_rejects_missing_policy_thread_anchor() -> None:
  with pytest.raises(PolicyStartGatePatchError, match="policy thread anchor"):
    patch_state_rlbase_h(
      STATE_RLBASE_FIXTURE.replace("env->reset();", "env->reset_now();")
    )


def test_cli_dry_run_does_not_write(tmp_path: Path) -> None:
  source = tmp_path / "State_RLBase.h"
  source.write_text(STATE_RLBASE_FIXTURE, encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(CLI), "--state-rlbase", str(source)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert GATE_ENV_VAR in proc.stdout
  assert GATE_MARKER not in source.read_text(encoding="utf-8")


def test_cli_apply_writes_patch(tmp_path: Path) -> None:
  source = tmp_path / "State_RLBase.h"
  source.write_text(STATE_RLBASE_FIXTURE, encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(CLI), "--state-rlbase", str(source), "--apply"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert GATE_MARKER in source.read_text(encoding="utf-8")
  backup = source.with_suffix(source.suffix + f".{GATE_MARKER}.bak")
  assert backup.read_text(encoding="utf-8") == STATE_RLBASE_FIXTURE
