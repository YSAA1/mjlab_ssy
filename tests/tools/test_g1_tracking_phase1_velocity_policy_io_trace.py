from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_policy_io_trace_patch import (
  HELPER_ANCHOR,
  INCLUDE_ANCHOR,
  INCLUDE_PATCH,
  OLD_HELPER_PATCH,
  OLD_STEP_PATCH,
  OLD_TRACE_MARKER,
  PREVIOUS_HELPER_PATCH,
  PREVIOUS_TRACE_MARKER,
  STEP_ANCHOR,
  STEP_PATCH,
  TRACE_MARKER,
  patch_manager_based_rl_env_h,
)
from mjlab.scripts.g1_tracking_phase1_velocity_policy_io_trace_report import (
  analyze_policy_io_trace,
)

ROOT = Path(__file__).resolve().parents[2]
PATCH_CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_patch.py"
REPORT_CLI = (
  ROOT / "scripts/tools/g1_tracking_phase1_velocity_policy_io_trace_report.py"
)

MANAGER_ENV_FIXTURE = """#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include "isaaclab/utils/utils.h"

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    void step()
    {
        episode_length += 1;
        robot->update();
        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
    }

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
};

};
"""


def _vector(values: list[float]) -> str:
  return "[" + ",".join(f"{value:.6f}" for value in values) + "]"


def _write_deploy_yaml(path: Path) -> None:
  path.write_text(
    "\n".join(
      [
        "observations:",
        "  base_ang_vel:",
        "    scale: [1, 1, 1]",
        "  projected_gravity:",
        "    scale: [1, 1, 1]",
        "  velocity_commands:",
        "    scale: [1, 1, 1]",
        "  gait_phase:",
        "    scale: [1, 1]",
        "  joint_pos_rel:",
        "    scale: [" + ", ".join(["1"] * 29) + "]",
        "  joint_vel_rel:",
        "    scale: [" + ", ".join(["1"] * 29) + "]",
        "  last_action:",
        "    scale: [" + ", ".join(["1"] * 29) + "]",
      ]
    )
    + "\n",
    encoding="utf-8",
  )


def test_policy_io_patch_injects_step_trace() -> None:
  patched, changed = patch_manager_based_rl_env_h(MANAGER_ENV_FIXTURE)

  assert changed is True
  assert TRACE_MARKER in patched
  assert "#include <spdlog/spdlog.h>" in patched
  assert "event=policy_io_trace" in patched
  assert "obs_dim={}" in patched
  assert "phase1_trace_should_log(episode_length, joint_vel_l2" in patched
  assert "low_dynamic_onset" in patched
  assert "dense_until = step + 75" in patched
  assert "joint_vel_l2 > 20.0f" in patched
  assert "phase1_trace_format(processed_action)" in patched


def test_policy_io_patch_upgrades_v1_trace() -> None:
  v1_patch = MANAGER_ENV_FIXTURE.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)
  v1_patch = v1_patch.replace(HELPER_ANCHOR, OLD_HELPER_PATCH, 1)
  v1_patch = v1_patch.replace(STEP_ANCHOR, OLD_STEP_PATCH, 1)

  upgraded, upgraded_changed = patch_manager_based_rl_env_h(v1_patch)

  assert upgraded_changed is True
  assert TRACE_MARKER in upgraded
  assert OLD_TRACE_MARKER not in upgraded
  assert "phase1_trace_should_log(episode_length, joint_vel_l2" in upgraded
  assert "low_dynamic_onset" in upgraded


def test_policy_io_patch_upgrades_v2_trace() -> None:
  v2_patch = MANAGER_ENV_FIXTURE.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)
  v2_patch = v2_patch.replace(HELPER_ANCHOR, PREVIOUS_HELPER_PATCH, 1)
  v2_patch = v2_patch.replace(STEP_ANCHOR, STEP_PATCH, 1)

  upgraded, upgraded_changed = patch_manager_based_rl_env_h(v2_patch)

  assert upgraded_changed is True
  assert TRACE_MARKER in upgraded
  assert PREVIOUS_TRACE_MARKER not in upgraded
  assert "low_dynamic_onset" in upgraded
  assert "dense_until = step + 75" in upgraded


def test_policy_io_patch_is_idempotent() -> None:
  patched, changed = patch_manager_based_rl_env_h(MANAGER_ENV_FIXTURE)
  patched_again, changed_again = patch_manager_based_rl_env_h(patched)

  assert changed is True
  assert changed_again is False
  assert patched_again == patched


def test_policy_io_patch_cli_dry_run_does_not_write(tmp_path: Path) -> None:
  source = tmp_path / "manager_based_rl_env.h"
  source.write_text(MANAGER_ENV_FIXTURE, encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(PATCH_CLI), "--manager-env", str(source)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert TRACE_MARKER not in source.read_text(encoding="utf-8")


def test_policy_io_report_slices_first_unstable_trace(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  deploy_yaml = tmp_path / "deploy.yaml"
  _write_deploy_yaml(deploy_yaml)
  obs = [0.0] * 98
  obs[0] = 0.1
  obs[7] = 1.0
  obs[40] = 123.0
  raw_action = [0.0] * 29
  raw_action[10] = 12.5
  processed_action = [0.0] * 29
  processed_action[10] = 5.0
  joint_pos = [0.0] * 29
  joint_vel = [0.0] * 29
  joint_vel[10] = 200.0
  (evidence / "g1_ctrl.log").write_text(
    "\n".join(
      [
        "[2026-05-22 22:28:43.593] [info] [PHASE1] event=stable_sample state=Velocity stable=0 policy_step=25 q_err_l2=1.0",
        "[2026-05-22 22:28:43.593] [info] [PHASE1] event=policy_io_trace step=25 obs_dim=98 obs_l2=123.004065 obs_max=123.000000 raw_action_l2=12.500000 raw_action_max=12.500000 processed_action_l2=5.000000 processed_action_max=5.000000 joint_pos_l2=0.000000 joint_vel_l2=200.000000 phase=0.866667 "
        f"obs={_vector(obs)} raw_action={_vector(raw_action)} processed_action={_vector(processed_action)} "
        f"joint_pos={_vector(joint_pos)} joint_vel={_vector(joint_vel)} "
        "projected_gravity=[0.000000,0.000000,-1.000000] root_ang_vel=[0.000000,0.000000,0.000000]",
      ]
    )
    + "\n",
    encoding="utf-8",
  )

  report = analyze_policy_io_trace(evidence, deploy_yaml=deploy_yaml)

  assert report["trace_count"] == 1
  assert report["obs_dim_matches"] is True
  assert report["selected_step_delta"] == 0
  assert report["decision"]["first_unstable_has_nearby_trace"] is True
  selected = report["selected_trace"]
  assert selected["step"] == 25
  assert selected["obs_terms"]["joint_vel_rel"]["values"][0] == 123.0
  assert selected["raw_action_summary"]["top_abs"][0]["index"] == 10
  assert selected["joint_vel_summary"]["top_abs"][0]["value"] == 200.0


def test_policy_io_report_resolves_policy_subdirectory(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  deploy_root = tmp_path / "deploy"
  policy_root = deploy_root / "config/policy/velocity"
  deploy_yaml = policy_root / "v0/params/deploy.yaml"
  deploy_yaml.parent.mkdir(parents=True)
  _write_deploy_yaml(deploy_yaml)
  (evidence / "selected").mkdir()
  (evidence / "selected/config.yaml").write_text(
    "FSM:\n  Velocity:\n    policy_dir: config/policy/velocity\n",
    encoding="utf-8",
  )
  obs = [0.0] * 98
  (evidence / "g1_ctrl.log").write_text(
    "[info] [PHASE1] event=policy_io_trace step=1 obs_dim=98 obs_l2=0.000000 obs_max=0.000000 raw_action_l2=0.000000 raw_action_max=0.000000 processed_action_l2=0.000000 processed_action_max=0.000000 joint_pos_l2=0.000000 joint_vel_l2=0.000000 phase=0.000000 "
    f"obs={_vector(obs)} raw_action={_vector([0.0] * 29)} processed_action={_vector([0.0] * 29)} "
    f"joint_pos={_vector([0.0] * 29)} joint_vel={_vector([0.0] * 29)} "
    "projected_gravity=[0.000000,0.000000,-1.000000] root_ang_vel=[0.000000,0.000000,0.000000]\n",
    encoding="utf-8",
  )

  report = analyze_policy_io_trace(evidence, deploy_root=deploy_root)

  assert report["deploy_yaml"] == str(deploy_yaml)
  assert report["trace_count"] == 1


def test_policy_io_report_cli_writes_json(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  deploy_yaml = tmp_path / "deploy.yaml"
  _write_deploy_yaml(deploy_yaml)
  obs = [0.0] * 98
  (evidence / "g1_ctrl.log").write_text(
    "[info] [PHASE1] event=policy_io_trace step=1 obs_dim=98 obs_l2=0.000000 obs_max=0.000000 raw_action_l2=0.000000 raw_action_max=0.000000 processed_action_l2=0.000000 processed_action_max=0.000000 joint_pos_l2=0.000000 joint_vel_l2=0.000000 phase=0.000000 "
    f"obs={_vector(obs)} raw_action={_vector([0.0] * 29)} processed_action={_vector([0.0] * 29)} "
    f"joint_pos={_vector([0.0] * 29)} joint_vel={_vector([0.0] * 29)} "
    "projected_gravity=[0.000000,0.000000,-1.000000] root_ang_vel=[0.000000,0.000000,0.000000]\n",
    encoding="utf-8",
  )
  report_out = tmp_path / "report.json"

  proc = subprocess.run(
    [
      sys.executable,
      str(REPORT_CLI),
      "--evidence-dir",
      str(evidence),
      "--deploy-yaml",
      str(deploy_yaml),
      "--report-out",
      str(report_out),
      "--expect-trace",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert json.loads(report_out.read_text(encoding="utf-8"))["trace_count"] == 1


def test_policy_io_report_cli_can_require_near_unstable_trace(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  deploy_yaml = tmp_path / "deploy.yaml"
  _write_deploy_yaml(deploy_yaml)
  obs = [0.0] * 98
  (evidence / "g1_ctrl.log").write_text(
    "\n".join(
      [
        "[info] [PHASE1] event=stable_sample state=Velocity stable=0 policy_step=950 q_err_l2=1.0",
        "[info] [PHASE1] event=policy_io_trace step=50 obs_dim=98 obs_l2=0.000000 obs_max=0.000000 raw_action_l2=0.000000 raw_action_max=0.000000 processed_action_l2=0.000000 processed_action_max=0.000000 joint_pos_l2=0.000000 joint_vel_l2=0.000000 phase=0.000000 "
        f"obs={_vector(obs)} raw_action={_vector([0.0] * 29)} processed_action={_vector([0.0] * 29)} "
        f"joint_pos={_vector([0.0] * 29)} joint_vel={_vector([0.0] * 29)} "
        "projected_gravity=[0.000000,0.000000,-1.000000] root_ang_vel=[0.000000,0.000000,0.000000]",
      ]
    )
    + "\n",
    encoding="utf-8",
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(REPORT_CLI),
      "--evidence-dir",
      str(evidence),
      "--deploy-yaml",
      str(deploy_yaml),
      "--expect-trace",
      "--expect-near-first-unstable",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 1
  assert '"selected_step_delta": 900' in proc.stdout
