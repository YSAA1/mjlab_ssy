from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from mjlab.sim2sim.isaaclab import (
  ACTION_BUNDLES,
  G1_DEPLOY_JOINT_NAMES,
  IsaacLabSim2SimError,
  deployment_report,
  resolve_g1_deployment,
)


def _write(path: Path, content: str | bytes = "content\n") -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  if isinstance(content, bytes):
    path.write_bytes(content)
  else:
    path.write_text(content, encoding="utf-8")


def _fake_root(tmp_path: Path) -> Path:
  root = tmp_path / "mjlab"
  for bundle in ACTION_BUNDLES.values():
    policy = (
      root
      / ".external"
      / "unitree_rl_mjlab"
      / "deploy"
      / "robots"
      / "g1"
      / "config"
      / "policy"
      / "mimic"
      / bundle["policy_subdir"]
    )
    _write(policy / f"exported/{bundle['policy_filename']}", b"onnx")
    _write(
      policy / f"params/{bundle['deploy_yaml_filename']}",
      "\n".join(
        [
          "step_dt: 0.02",
          f"default_joint_pos: {[0.0] * len(G1_DEPLOY_JOINT_NAMES)}",
          f"stiffness: {[1.0] * len(G1_DEPLOY_JOINT_NAMES)}",
          f"damping: {[0.1] * len(G1_DEPLOY_JOINT_NAMES)}",
          "actions:",
          "  JointPositionAction:",
          f"    scale: {[1.0] * len(G1_DEPLOY_JOINT_NAMES)}",
          f"    offset: {[0.0] * len(G1_DEPLOY_JOINT_NAMES)}",
        ]
      )
      + "\n",
    )
    np.savez(
      policy / f"params/{bundle['motion_filename']}",
      fps=np.asarray([50.0]),
      joint_pos=np.zeros((3, len(G1_DEPLOY_JOINT_NAMES)), dtype=np.float32),
      joint_vel=np.zeros((3, len(G1_DEPLOY_JOINT_NAMES)), dtype=np.float32),
    )
  _write(
    root
    / "src"
    / "mjlab"
    / "asset_zoo"
    / "robots"
    / "unitree_g1"
    / "urdf"
    / "g1_29dof_mode_15.urdf",
    "<robot name='g1'/>\n",
  )
  return root


@pytest.mark.parametrize("action,bundle", sorted(ACTION_BUNDLES.items()))
def test_resolve_g1_deployment_supports_registered_actions(
  tmp_path: Path, action: str, bundle: dict[str, str]
) -> None:
  root = _fake_root(tmp_path)

  deployment = resolve_g1_deployment(action=action, mjlab_root=root)

  assert deployment.action == action
  assert deployment.policy_root.name == bundle["policy_subdir"]
  assert deployment.deploy_yaml.name == bundle["deploy_yaml_filename"]
  assert deployment.motion_file.name == bundle["motion_filename"]
  assert deployment.policy_onnx.name == bundle["policy_filename"]


def test_resolve_g1_deployment_uses_action_bundle_defaults(tmp_path: Path) -> None:
  root = _fake_root(tmp_path)

  deployment = resolve_g1_deployment(action="flying_kick", mjlab_root=root)

  assert deployment.action == "flying_kick"
  assert deployment.external_root == root / ".external"
  assert deployment.policy_root.name == "flying_kick"
  assert deployment.deploy_yaml.name == "deploy.yaml"
  assert deployment.motion_file.name == "flying_kick.npz"
  assert deployment.policy_onnx.name == "policy.onnx"


def test_resolve_g1_deployment_accepts_external_root_override(tmp_path: Path) -> None:
  root = _fake_root(tmp_path)
  external_root = tmp_path / "shared-external"
  source = root / ".external"
  target = external_root
  target.parent.mkdir(parents=True, exist_ok=True)
  source.rename(target)

  deployment = resolve_g1_deployment(
    action="flying_kick",
    mjlab_root=root,
    external_root=external_root,
  )

  assert deployment.external_root == external_root
  assert deployment.policy_root.is_relative_to(external_root)


def test_deployment_report_records_static_evidence(tmp_path: Path) -> None:
  root = _fake_root(tmp_path)
  deployment = resolve_g1_deployment(action="flying_kick", mjlab_root=root)

  report = deployment_report(deployment)

  assert report["simulator"] == "isaaclab"
  assert report["robot"] == "unitree_g1_29dof_mode_15"
  assert report["motion"]["frames"] == 3
  assert report["motion"]["joint_dim"] == len(G1_DEPLOY_JOINT_NAMES)
  assert len(report["policy_onnx"]["sha256"]) == 64
  assert report["external_root"] == str(root / ".external")
  json.dumps(report)


def test_resolve_g1_deployment_rejects_bad_motion_shape(tmp_path: Path) -> None:
  root = _fake_root(tmp_path)
  motion = (
    root
    / ".external"
    / "unitree_rl_mjlab"
    / "deploy"
    / "robots"
    / "g1"
    / "config"
    / "policy"
    / "mimic"
    / "flying_kick"
    / "params"
    / "flying_kick.npz"
  )
  np.savez(
    motion,
    fps=np.asarray([50.0]),
    joint_pos=np.zeros((3, len(G1_DEPLOY_JOINT_NAMES) - 1), dtype=np.float32),
    joint_vel=np.zeros((3, len(G1_DEPLOY_JOINT_NAMES) - 1), dtype=np.float32),
  )

  with pytest.raises(IsaacLabSim2SimError, match="motion joint_pos"):
    resolve_g1_deployment(action="flying_kick", mjlab_root=root)


def test_isaaclab_package_does_not_embed_machine_absolute_paths() -> None:
  package_root = Path(__file__).resolve().parents[2] / "src/mjlab/sim2sim/isaaclab"
  combined = "\n".join(
    path.read_text(encoding="utf-8") for path in package_root.rglob("*.py")
  )

  assert "/home/ssy" not in combined
  assert "g1-flying-kick-main" not in combined
