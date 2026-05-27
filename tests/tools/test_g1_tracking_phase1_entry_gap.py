from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

from mjlab.scripts.g1_tracking_phase1_entry_gap import analyze_entry_gaps

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_entry_gap.py"


def _write(path: Path, content: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _fake_manifest(tmp_path: Path, *, q0: list[float]) -> Path:
  xml = tmp_path / "g1.xml"
  _write(
    xml,
    """
<mujoco>
  <worldbody>
    <body>
      <joint name="joint_a"/>
      <joint name="joint_b"/>
      <joint name="joint_c"/>
    </body>
  </worldbody>
</mujoco>
""".strip(),
  )
  deploy_yaml = tmp_path / "deploy.yaml"
  _write(
    deploy_yaml,
    yaml.safe_dump({"default_joint_pos": [0.0, 0.0, 0.0]}),
  )
  motion = tmp_path / "motion.npz"
  np.savez(
    motion,
    fps=np.array([50.0], dtype=np.float32),
    joint_pos=np.asarray([q0, [0.2, 0.0, 0.0]], dtype=np.float32),
    joint_vel=np.zeros((2, 3), dtype=np.float32),
  )
  manifest = {
    "robot_model_sources": {"mjlab_g1_xml": {"path": str(xml)}},
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


def test_entry_gap_flags_pose_mismatch_and_best_frame(tmp_path: Path) -> None:
  manifest = _fake_manifest(tmp_path, q0=[1.0, 0.0, 0.0])

  report = analyze_entry_gaps(manifest, max_entry_gap_l2=0.5)

  assert report["passed"] is False
  assert report["primary_reason"] == "entry_state_pose_mismatch"
  action = report["actions"]["flying_kick"]
  assert action["reason"] == "entry_state_pose_mismatch"
  assert action["frame0_gap_l2"] == 1.0
  assert action["best_default_pose_frame"] == 1
  assert action["top_joint_gaps"][0]["joint"] == "joint_a"


def test_entry_gap_passes_when_default_pose_matches_reference(tmp_path: Path) -> None:
  manifest = _fake_manifest(tmp_path, q0=[0.1, 0.0, 0.0])

  report = analyze_entry_gaps(manifest, max_entry_gap_l2=0.5)

  assert report["passed"] is True
  assert report["primary_reason"] is None


def test_cli_accepts_expected_entry_gap_failure(tmp_path: Path) -> None:
  manifest = _fake_manifest(tmp_path, q0=[1.0, 0.0, 0.0])
  log = tmp_path / "g1_ctrl.log"
  _write(
    log,
    "[2026-05-22 14:33:52.226] [info] [PHASE1] event=q_response "
    "action=Mimic_FlyingKick step=1 q_err_l2=1.000 dq_err_l2=0.0 "
    "base_vel_x=0.000 command_vel_x=0.000 gravity_b=(0.000,0.000,-1.000)\n",
  )

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--manifest",
      str(manifest),
      "--log",
      str(log),
      "--expect-failure",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  report = json.loads(proc.stdout)
  assert report["actions"]["flying_kick"]["first_logged_q_err_l2"] == 1.0
