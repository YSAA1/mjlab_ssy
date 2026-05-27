from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

from mjlab.scripts.g1_tracking_phase1_handoff_gate import analyze_handoff_gate

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_handoff_gate.py"


def _write(path: Path, content: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _reference_qpos(joint: list[float]) -> list[float]:
  return [0.0, 0.0, 0.75, 1.0, 0.0, 0.0, 0.0, *joint]


def _fake_manifest(
  tmp_path: Path,
  *,
  default_joint_pos: list[float],
  frame0_joint_pos: list[float],
  sim_initial_qpos: list[float] | None,
) -> Path:
  deploy_yaml = tmp_path / "deploy.yaml"
  _write(deploy_yaml, yaml.safe_dump({"default_joint_pos": default_joint_pos}))
  motion = tmp_path / "motion.npz"
  np.savez(
    motion,
    fps=np.array([50.0], dtype=np.float32),
    joint_pos=np.asarray(
      [frame0_joint_pos, default_joint_pos],
      dtype=np.float32,
    ),
    joint_vel=np.zeros((2, 3), dtype=np.float32),
    body_pos_w=np.asarray(
      [
        [[0.2, -0.1, 0.75]],
        [[0.2, -0.1, 0.75]],
      ],
      dtype=np.float32,
    ),
    body_quat_w=np.asarray(
      [
        [[1.0, 0.0, 0.0, 0.0]],
        [[1.0, 0.0, 0.0, 0.0]],
      ],
      dtype=np.float32,
    ),
  )
  sim_config = tmp_path / "simulate/config.yaml"
  sim_data = {
    "robot": "g1",
    "robot_scene": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
    "domain_id": 0,
    "interface": "lo",
    "use_joystick": 0,
    "start_paused": 1,
  }
  if sim_initial_qpos is not None:
    sim_data["initial_qpos"] = sim_initial_qpos
  _write(sim_config, yaml.safe_dump(sim_data))
  manifest = {
    "deploy_configs": {"sim_config": {"path": str(sim_config)}},
    "actions": {
      "flying_kick": {
        "deploy_yaml": {"path": str(deploy_yaml)},
        "deploy_motion_npz": {"path": str(motion)},
      }
    },
  }
  manifest_path = tmp_path / "manifest.json"
  manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
  return manifest_path


def test_handoff_gate_blocks_sim_teleport_only_entry(tmp_path: Path) -> None:
  frame0 = [1.0, 0.0, 0.0]
  manifest = _fake_manifest(
    tmp_path,
    default_joint_pos=[0.0, 0.0, 0.0],
    frame0_joint_pos=frame0,
    sim_initial_qpos=_reference_qpos(frame0),
  )

  report = analyze_handoff_gate(manifest, max_deploy_entry_gap_l2=0.5)

  assert report["passed"] is False
  assert report["primary_reason"] == "no_deploy_safe_entry_contract"
  assert report["real_robot_unlocked"] is False
  assert report["sim_teleport_only_present"] is True
  action = report["actions"]["flying_kick"]
  assert action["deploy_default_entry"]["reason"] == "entry_state_pose_mismatch"
  assert action["active_sim_initial_qpos_entry"]["entry_type"] == "sim_teleport_only"
  assert action["deploy_safe_transition_entry"]["available"] is False
  assert action["deploy_safe_transition_entry"]["candidate_available"] is True
  assert action["deploy_safe_transition_entry"]["source"] == "sim2sim_prepose_mode"


def test_handoff_gate_passes_when_deploy_default_matches_reference(
  tmp_path: Path,
) -> None:
  frame0 = [0.1, 0.0, 0.0]
  manifest = _fake_manifest(
    tmp_path,
    default_joint_pos=frame0,
    frame0_joint_pos=frame0,
    sim_initial_qpos=None,
  )

  report = analyze_handoff_gate(manifest, max_deploy_entry_gap_l2=0.5)

  assert report["passed"] is True
  assert report["primary_reason"] is None
  assert report["actions"]["flying_kick"]["deploy_default_entry"]["passed"] is True
  assert (
    report["actions"]["flying_kick"]["deploy_safe_transition_entry"]["available"]
    is True
  )


def test_cli_accepts_expected_blocked_handoff_gate(tmp_path: Path) -> None:
  frame0 = [1.0, 0.0, 0.0]
  manifest = _fake_manifest(
    tmp_path,
    default_joint_pos=[0.0, 0.0, 0.0],
    frame0_joint_pos=frame0,
    sim_initial_qpos=_reference_qpos(frame0),
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--manifest",
      str(manifest),
      "--expect-blocked",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  report = json.loads(proc.stdout)
  assert report["passed"] is False
  assert (
    report["actions"]["flying_kick"]["active_sim_initial_qpos_entry"][
      "deploy_acceptance_candidate"
    ]
    is False
  )
