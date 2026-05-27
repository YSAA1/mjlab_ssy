from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_deploy98_task_contract import (
  validate_deploy98_task_contract,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_deploy98_task_contract.py"


def _write_deploy_yaml(path: Path, terms: list[str]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  observations = "\n".join(f"  {term}: {{params: {{}}}}" for term in terms)
  path.write_text(f"observations:\n{observations}\n", encoding="utf-8")


def test_deploy98_task_contract_matches_runtime_terms(tmp_path: Path) -> None:
  deploy_yaml = tmp_path / "deploy.yaml"
  _write_deploy_yaml(
    deploy_yaml,
    [
      "base_ang_vel",
      "projected_gravity",
      "velocity_commands",
      "gait_phase",
      "joint_pos_rel",
      "joint_vel_rel",
      "last_action",
    ],
  )

  report = validate_deploy98_task_contract(
    task_id="Mjlab-Velocity-Flat-Unitree-G1-Deploy98",
    deploy_yaml=deploy_yaml,
  )

  assert report["actor_contract"]["known_dim"] == 98
  assert report["runtime_contract"]["known_dim"] == 98
  assert report["semantic_mapping"]["matches_runtime_terms"] is True
  assert report["decision"]["task_contract_matches_active_runtime"] is True
  assert report["decision"]["safe_to_swap_without_training"] is False


def test_deploy98_task_contract_detects_wrong_runtime_order(tmp_path: Path) -> None:
  deploy_yaml = tmp_path / "deploy.yaml"
  _write_deploy_yaml(
    deploy_yaml,
    [
      "projected_gravity",
      "base_ang_vel",
      "velocity_commands",
      "gait_phase",
      "joint_pos_rel",
      "joint_vel_rel",
      "last_action",
    ],
  )

  report = validate_deploy98_task_contract(
    task_id="Mjlab-Velocity-Flat-Unitree-G1-Deploy98",
    deploy_yaml=deploy_yaml,
  )

  assert report["semantic_mapping"]["matches_runtime_terms"] is False
  assert report["decision"]["task_contract_matches_active_runtime"] is False


def test_deploy98_task_contract_cli_expect_compatible(tmp_path: Path) -> None:
  deploy_yaml = tmp_path / "deploy.yaml"
  _write_deploy_yaml(
    deploy_yaml,
    [
      "base_ang_vel",
      "projected_gravity",
      "velocity_commands",
      "gait_phase",
      "joint_pos_rel",
      "joint_vel_rel",
      "last_action",
    ],
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--deploy-yaml",
      str(deploy_yaml),
      "--expect-compatible",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"task_contract_matches_active_runtime": true' in proc.stdout
