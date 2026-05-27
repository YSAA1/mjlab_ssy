from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import onnx
import yaml
from onnx import TensorProto, helper

from mjlab.scripts.g1_tracking_phase1_contract import validate_contract
from mjlab.scripts.g1_tracking_phase1_manifest import (
  ManifestConfig,
  build_manifest,
)

JOINT_NAMES = [f"joint_{idx}" for idx in range(29)]


def _write(path: Path, content: str | bytes) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  if isinstance(content, bytes):
    path.write_bytes(content)
  else:
    path.write_text(content, encoding="utf-8")


def _robot_xml(
  joint_names: list[str] = JOINT_NAMES,
  *,
  include_actuators: bool = False,
) -> str:
  joints = "\n".join(f'<joint name="{name}" />' for name in joint_names)
  motors = "\n".join(
    f'<motor name="{name.removesuffix("_joint")}" joint="{name}" />'
    for name in joint_names
  )
  actuator = f"<actuator>{motors}</actuator>" if include_actuators else ""
  return f"""<mujoco model="g1_29dof_mode_15_aligned">
  <worldbody><body>{joints}</body></worldbody>
  {actuator}
</mujoco>
"""


def _urdf() -> str:
  joints = "\n".join(f'<joint name="{name}" type="revolute" />' for name in JOINT_NAMES)
  return f'<robot name="g1_29dof_mode_15">{joints}</robot>'


def _write_motion(path: Path, dof: int = 29) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  np.savez(
    path,
    fps=np.array([30.0]),
    joint_pos=np.zeros((2, dof), dtype=np.float32),
    joint_vel=np.zeros((2, dof), dtype=np.float32),
  )


def _write_onnx(path: Path, action_dim: int = 29) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  graph = helper.make_graph(
    nodes=[
      helper.make_node(
        "Constant",
        inputs=[],
        outputs=["actions"],
        value=helper.make_tensor(
          "actions_value",
          TensorProto.FLOAT,
          [1, action_dim],
          [0.0] * action_dim,
        ),
      )
    ],
    name="policy",
    inputs=[
      helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 154]),
    ],
    outputs=[
      helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, action_dim])
    ],
  )
  onnx.save(helper.make_model(graph), path)


def _write_deploy_yaml(path: Path, dof: int = 29) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(
    yaml.safe_dump(
      {
        "joint_ids_map": list(range(dof)),
        "step_dt": 0.02,
        "stiffness": [1.0] * dof,
        "damping": [0.1] * dof,
        "default_joint_pos": [0.0] * dof,
        "actions": {
          "JointPositionAction": {
            "scale": [1.0] * dof,
            "offset": [0.0] * dof,
          }
        },
      }
    ),
    encoding="utf-8",
  )


def _workspace(tmp_path: Path) -> tuple[ManifestConfig, Path]:
  worktree = tmp_path / "worktree"
  mjlab_root = tmp_path / "mjlab-root"
  deploy = mjlab_root / ".external/unitree_rl_mjlab/deploy/robots/g1"
  simulate = mjlab_root / ".external/unitree_rl_mjlab/simulate"
  external_xml_dir = (
    mjlab_root / ".external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls"
  )

  _write(worktree / ".git/HEAD", "ref: refs/heads/test\n")
  _write(
    worktree / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml",
    _robot_xml(),
  )
  _write(
    worktree / "src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf",
    _urdf(),
  )
  _write(external_xml_dir / "g1.xml", _robot_xml())
  _write(external_xml_dir / "scene_g1.xml", _robot_xml(include_actuators=True))
  _write(
    external_xml_dir / "g1_23dof.xml",
    _robot_xml(JOINT_NAMES[:23], include_actuators=True),
  )
  _write(deploy / "config/config.yaml", "FSM: {}\n")
  _write(simulate / "config.yaml", "robot: g1\n")
  _write_deploy_yaml(deploy / "config/policy/mimic/getup/params/deploy.yaml")

  user_urdf = tmp_path / "user/g1_29dof_mode_15.urdf"
  user_xml = tmp_path / "user/g1_new.xml"
  symptom_video = tmp_path / "video/symptom.mp4"
  _write(user_urdf, _urdf())
  _write(user_xml, _robot_xml())
  _write(symptom_video, b"not-a-real-mp4")

  actions = [
    (
      "g1_tracking_acrobatics_no_state",
      "2026-01-01_g1_mode15_flying_kick_4096env_5000iter",
      "flying_kick_deploy_actor.onnx",
      "g1_flying_kick",
      "flying_kick",
      "flying_kick.npz",
    ),
    (
      "g1_tracking_roundhouse_leading_right_no_state",
      "2026-01-02_g1_mode15_roundhouse_leading_right",
      "roundhouse_leading_right_deploy_actor.onnx",
      "g1_roundhouse_leading_right",
      "roundhouse_leading_right",
      "roundhouse_leading_right.npz",
    ),
  ]
  for (
    experiment,
    run_name,
    policy_name,
    motion_dir,
    deploy_dir,
    deploy_motion,
  ) in actions:
    _write_onnx(worktree / f"logs/rsl_rl/{experiment}/{run_name}/{policy_name}")
    _write_motion(worktree / f"data/motions/{motion_dir}/mjlab/motion.npz")
    _write_onnx(deploy / f"config/policy/mimic/{deploy_dir}/exported/policy.onnx")
    _write_motion(deploy / f"config/policy/mimic/{deploy_dir}/params/{deploy_motion}")
    _write_deploy_yaml(deploy / f"config/policy/mimic/{deploy_dir}/params/deploy.yaml")

  config = ManifestConfig(
    worktree=worktree,
    mjlab_root=mjlab_root,
    output_root=tmp_path / "out",
    timestamp="2026-05-22T12:00:00+08:00",
    dry_run=False,
    flying_policy_onnx=None,
    roundhouse_policy_onnx=None,
    flying_run_dir=None,
    roundhouse_run_dir=None,
    flying_experiment_name="g1_tracking_acrobatics_no_state",
    roundhouse_experiment_name="g1_tracking_roundhouse_leading_right_no_state",
    flying_run_name_pattern="*g1_mode15_flying_kick_4096env_5000iter*",
    roundhouse_run_name_pattern="*g1_mode15_roundhouse_leading_right*",
    user_g1_urdf=user_urdf,
    user_g1_xml=user_xml,
    symptom_video=symptom_video,
  )
  return config, external_xml_dir


def test_contract_passes_for_matching_29dof_assets(tmp_path: Path) -> None:
  config, _ = _workspace(tmp_path)
  manifest = build_manifest(config)

  report = validate_contract(manifest, forbid_g1_23dof=True)

  assert report["passed"] is True
  assert report["expected_dof"] == 29
  assert report["ordered_joint_names"] == JOINT_NAMES
  assert report["actions"]["flying_kick"]["deploy_yaml"][
    "action_scale_source"
  ].endswith("actions.JointPositionAction.scale")


def test_contract_fails_on_wrong_scene_or_old_g1_path(tmp_path: Path) -> None:
  config, external_xml_dir = _workspace(tmp_path)
  manifest = build_manifest(config)
  manifest["external_robot_assets"]["external_scene_g1_xml"]["path"] = str(
    external_xml_dir / "g1_23dof.xml"
  )

  report = validate_contract(manifest, forbid_g1_23dof=True)

  assert report["passed"] is False
  assert any("g1_23dof" in failure for failure in report["failures"])
  assert any("joint_count=23" in failure for failure in report["failures"])


def test_contract_fails_on_actuator_order_mismatch(tmp_path: Path) -> None:
  config, external_xml_dir = _workspace(tmp_path)
  _write(
    external_xml_dir / "scene_g1.xml",
    _robot_xml(list(reversed(JOINT_NAMES)), include_actuators=True),
  )
  manifest = build_manifest(config)

  report = validate_contract(manifest)

  assert report["passed"] is False
  assert any("joint order differs" in failure for failure in report["failures"])


def test_contract_fails_on_motion_or_onnx_action_dim_mismatch(
  tmp_path: Path,
) -> None:
  config, _ = _workspace(tmp_path)
  bad_policy = (
    config.worktree
    / "logs/rsl_rl/g1_tracking_acrobatics_no_state/2026-01-01_g1_mode15_flying_kick_4096env_5000iter/flying_kick_deploy_actor.onnx"
  )
  _write_onnx(bad_policy, action_dim=28)
  bad_motion = config.worktree / "data/motions/g1_flying_kick/mjlab/motion.npz"
  _write_motion(bad_motion, dof=28)
  manifest = build_manifest(config)

  report = validate_contract(manifest)

  assert report["passed"] is False
  assert any("action_dim=28" in failure for failure in report["failures"])
  assert any("joint_dim=28" in failure for failure in report["failures"])


def test_cli_report_json_round_trip(tmp_path: Path) -> None:
  config, _ = _workspace(tmp_path)
  manifest = build_manifest(config)
  manifest_path = tmp_path / "manifest.json"
  manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

  loaded = json.loads(manifest_path.read_text(encoding="utf-8"))
  report = validate_contract(loaded, forbid_g1_23dof=True)

  assert report["passed"] is True
