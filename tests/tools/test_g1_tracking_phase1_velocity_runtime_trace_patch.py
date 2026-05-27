from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_runtime_trace_patch import (
  TRACE_MARKER,
  patch_state_rlbase_cpp,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_runtime_trace_patch.py"

STATE_RLBASE_FIXTURE = """#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    return {0.0f, 0.0f, 0.0f};
}

}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }

    const float t = env->episode_length * env->step_dt;
    const int stable_log_slot = static_cast<int>(t * 2.0f);
    if(stable_log_slot != phase1_last_stable_log_slot_)
    {
        env->robot->update();
        auto q_real = env->robot->data.joint_pos;
        float q_err_l2 = 0.0f;
        float q_err_max = 0.0f;
        for(int i(0); i < q_real.size() && i < action.size(); ++i) {
            const float q_err = q_real[i] - action[i];
            q_err_l2 += q_err * q_err;
            q_err_max = std::max(q_err_max, std::abs(q_err));
        }
        const auto gravity = env->robot->data.projected_gravity_b;
        const auto root_ang_vel = env->robot->data.root_ang_vel_b;
        const float root_ang_vel_l2 = root_ang_vel.norm();
        const bool stable = gravity[2] < -0.75f && root_ang_vel_l2 < 2.0f;
        spdlog::info(
            "[PHASE1] event=stable_sample state={} stable={} q_err_l2={:.3f} "
            "q_err_max={:.3f} base_vel_x=0.000 command_vel_x=0.000 "
            "gravity_b=({:.3f},{:.3f},{:.3f}) root_ang_vel_l2={:.3f}",
            getStateString(),
            stable ? 1 : 0,
            std::sqrt(q_err_l2),
            q_err_max,
            gravity[0],
            gravity[1],
            gravity[2],
            root_ang_vel_l2
        );
        phase1_last_stable_log_slot_ = stable_log_slot;
    }
}
"""


def test_patch_injects_velocity_runtime_trace_fields() -> None:
  patched, changed = patch_state_rlbase_cpp(STATE_RLBASE_FIXTURE)

  assert changed is True
  assert TRACE_MARKER in patched
  assert "#include <algorithm>" in patched
  assert "policy_step={}" in patched
  assert "command_vel_y={:.3f}" in patched
  assert "raw_action_l2={:.3f}" in patched
  assert "processed_action_l2={:.3f}" in patched
  assert "joint_pos_rel_l2={:.3f}" in patched
  assert "joint_vel_l2={:.3f}" in patched
  assert "command_vel_x=0.000" not in patched


def test_patch_is_idempotent() -> None:
  patched, changed = patch_state_rlbase_cpp(STATE_RLBASE_FIXTURE)
  patched_again, changed_again = patch_state_rlbase_cpp(patched)

  assert changed is True
  assert changed_again is False
  assert patched_again == patched


def test_cli_dry_run_does_not_write(tmp_path: Path) -> None:
  source = tmp_path / "State_RLBase.cpp"
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
  assert TRACE_MARKER not in source.read_text(encoding="utf-8")


def test_cli_apply_writes_patch(tmp_path: Path) -> None:
  source = tmp_path / "State_RLBase.cpp"
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
  assert TRACE_MARKER in source.read_text(encoding="utf-8")
  backup = source.with_suffix(source.suffix + f".{TRACE_MARKER}.bak")
  assert backup.read_text(encoding="utf-8") == STATE_RLBASE_FIXTURE
