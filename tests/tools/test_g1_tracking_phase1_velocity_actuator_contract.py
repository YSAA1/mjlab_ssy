from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_velocity_actuator_contract import (
  audit_velocity_actuator_contract,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_velocity_actuator_contract.py"

JOINT_NAMES = [
  "left_hip_pitch_joint",
  "left_hip_roll_joint",
  "left_hip_yaw_joint",
  "left_knee_joint",
  "left_ankle_pitch_joint",
  "left_ankle_roll_joint",
  "right_hip_pitch_joint",
  "right_hip_roll_joint",
  "right_hip_yaw_joint",
  "right_knee_joint",
  "right_ankle_pitch_joint",
  "right_ankle_roll_joint",
  "waist_yaw_joint",
  "waist_roll_joint",
  "waist_pitch_joint",
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_pitch_joint",
  "left_wrist_yaw_joint",
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint",
]


def _write(path: Path, content: str) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _xml(joint_names: list[str] = JOINT_NAMES, *, actuators: dict[str, float] | None):
  joints = "\n".join(f'<joint name="{name}" />' for name in joint_names)
  motors = ""
  if actuators is not None:
    motors = "\n".join(
      f'<motor name="{joint.removesuffix("_joint")}" joint="{joint}" ctrlrange="{-limit:g} {limit:g}" />'
      for joint, limit in actuators.items()
    )
    motors = f"<actuator>{motors}</actuator>"
  return f"""<mujoco model="g1_29dof_mode_15_aligned">
  <worldbody><body>{joints}</body></worldbody>
  {motors}
</mujoco>
"""


def _matching_limits() -> dict[str, float]:
  limits = {}
  for joint in JOINT_NAMES:
    if "hip_roll" in joint or "knee" in joint:
      limits[joint] = 139.0
    elif "hip_pitch" in joint or "hip_yaw" in joint or joint == "waist_yaw_joint":
      limits[joint] = 88.0
    elif "wrist_pitch" in joint or "wrist_yaw" in joint:
      limits[joint] = 5.0
    elif "ankle" in joint or joint in {"waist_roll_joint", "waist_pitch_joint"}:
      limits[joint] = 50.0
    else:
      limits[joint] = 25.0
  return limits


def test_actuator_contract_detects_external_right_ankle_limit_mismatch(
  tmp_path: Path,
) -> None:
  mjlab_xml = tmp_path / "mjlab_g1.xml"
  user_xml = tmp_path / "user_g1.xml"
  scene_xml = tmp_path / "scene_g1.xml"
  limits = _matching_limits()
  limits["right_ankle_pitch_joint"] = 25.0
  limits["right_ankle_roll_joint"] = 25.0
  _write(mjlab_xml, _xml(actuators=None))
  _write(user_xml, _xml(actuators=None))
  _write(scene_xml, _xml(actuators=limits))

  report = audit_velocity_actuator_contract(
    external_scene=scene_xml,
    user_g1_xml=user_xml,
    mjlab_g1_xml=mjlab_xml,
  )

  assert report["decision"]["actuator_force_contract_matches"] is False
  assert report["decision"]["right_ankle_limit_mismatch"] is True
  mismatch_joints = {row["joint"] for row in report["mismatches"]}
  assert mismatch_joints == {
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
  }


def test_actuator_contract_passes_for_matching_scene_limits(tmp_path: Path) -> None:
  mjlab_xml = tmp_path / "mjlab_g1.xml"
  scene_xml = tmp_path / "scene_g1.xml"
  _write(mjlab_xml, _xml(actuators=None))
  _write(scene_xml, _xml(actuators=_matching_limits()))

  report = audit_velocity_actuator_contract(
    external_scene=scene_xml,
    user_g1_xml=None,
    mjlab_g1_xml=mjlab_xml,
  )

  assert report["decision"]["actuator_force_contract_matches"] is True
  assert report["mismatches"] == []


def test_actuator_contract_cli_expect_mismatch(tmp_path: Path) -> None:
  mjlab_xml = tmp_path / "mjlab_g1.xml"
  scene_xml = tmp_path / "scene_g1.xml"
  limits = _matching_limits()
  limits["right_ankle_pitch_joint"] = 25.0
  _write(mjlab_xml, _xml(actuators=None))
  _write(scene_xml, _xml(actuators=limits))
  report_out = tmp_path / "report.json"

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--mjlab-g1-xml",
      str(mjlab_xml),
      "--external-scene",
      str(scene_xml),
      "--user-g1-xml",
      str(tmp_path / "missing_user.xml"),
      "--report-out",
      str(report_out),
      "--expect-mismatch",
    ],
    cwd=ROOT,
    text=True,
    capture_output=True,
    check=False,
  )

  assert proc.returncode == 0, proc.stderr
  report = json.loads(report_out.read_text(encoding="utf-8"))
  assert report["decision"]["primary_mismatch"] == "external_scene_motor_ctrlrange"
