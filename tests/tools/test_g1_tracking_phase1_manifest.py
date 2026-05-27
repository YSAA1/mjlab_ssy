from __future__ import annotations

import json
from pathlib import Path

import pytest

from mjlab.scripts.g1_tracking_phase1_manifest import (
  ManifestConfig,
  ManifestError,
  build_manifest,
  write_manifest,
)


def _write(path: Path, content: bytes = b"content") -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(content)


def _fake_workspace(tmp_path: Path) -> tuple[Path, Path, ManifestConfig]:
  worktree = tmp_path / "worktree"
  mjlab_root = tmp_path / "mjlab-root"
  deploy = mjlab_root / ".external/unitree_rl_mjlab/deploy/robots/g1"
  simulate = mjlab_root / ".external/unitree_rl_mjlab/simulate"

  _write(worktree / ".git/HEAD", b"ref: refs/heads/test\n")
  _write(
    worktree
    / "logs/rsl_rl/g1_tracking_acrobatics_no_state/2026-01-01_g1_mode15_flying_kick_4096env_5000iter/flying_kick_deploy_actor.onnx",
    b"flying-source-policy",
  )
  _write(
    worktree
    / "logs/rsl_rl/g1_tracking_roundhouse_leading_right_no_state/2026-01-02_g1_mode15_roundhouse_leading_right/roundhouse_leading_right_deploy_actor.onnx",
    b"roundhouse-source-policy",
  )
  _write(
    worktree / "data/motions/g1_flying_kick/mjlab/motion.npz",
    b"flying-motion",
  )
  _write(
    worktree / "data/motions/g1_roundhouse_leading_right/mjlab/motion.npz",
    b"roundhouse-motion",
  )
  _write(
    worktree / "src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf",
    b"mjlab-urdf",
  )
  _write(
    worktree / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml",
    b"mjlab-xml",
  )
  _write(worktree / "scripts/tools/g1_tracking_phase1_manifest.py", b"wrapper")
  _write(worktree / "scripts/tools/run_g1_dual_kicks_real_deploy.sh", b"real")
  _write(worktree / "scripts/tools/run_flying_kick_sim2sim.sh", b"flying-sim")
  _write(
    worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh",
    b"roundhouse-sim",
  )

  _write(deploy / "config/config.yaml", b"active-config")
  _write(deploy / "config/policy/mimic/getup/params/deploy.yaml", b"shared-deploy")
  _write(
    deploy / "config/policy/mimic/flying_kick/exported/policy.onnx",
    b"flying-deploy-policy",
  )
  _write(
    deploy / "config/policy/mimic/flying_kick/params/flying_kick.npz",
    b"flying-deploy-motion",
  )
  _write(
    deploy / "config/policy/mimic/flying_kick/params/deploy.yaml",
    b"flying-deploy-yaml",
  )
  _write(
    deploy / "config/policy/mimic/roundhouse_leading_right/exported/policy.onnx",
    b"roundhouse-deploy-policy",
  )
  _write(
    deploy
    / "config/policy/mimic/roundhouse_leading_right/params/roundhouse_leading_right.npz",
    b"roundhouse-deploy-motion",
  )
  _write(
    deploy / "config/policy/mimic/roundhouse_leading_right/params/deploy.yaml",
    b"roundhouse-deploy-yaml",
  )
  _write(simulate / "config.yaml", b"sim-config")
  _write(
    mjlab_root / ".external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml",
    b"external-g1-xml",
  )
  _write(
    mjlab_root
    / ".external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/scene_g1.xml",
    b"external-scene-g1-xml",
  )
  _write(
    mjlab_root
    / ".external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1_23dof.xml",
    b"forbidden-old-g1",
  )

  user_urdf = tmp_path / "user/g1_29dof_mode_15.urdf"
  user_xml = tmp_path / "user/g1_new.xml"
  symptom_video = tmp_path / "video/symptom.mp4"
  _write(user_urdf, b"user-urdf")
  _write(user_xml, b"user-xml")
  _write(symptom_video, b"not-a-real-mp4")

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
  return worktree, mjlab_root, config


def test_build_manifest_records_both_actions_and_required_sources(
  tmp_path: Path,
) -> None:
  _, _, config = _fake_workspace(tmp_path)

  manifest = build_manifest(config)

  assert manifest["non_launching"] is True
  assert sorted(manifest["actions"]) == ["flying_kick", "roundhouse_leading_right"]
  assert manifest["actions"]["flying_kick"]["state_name"] == "Mimic_FlyingKick"
  assert (
    manifest["actions"]["roundhouse_leading_right"]["state_name"]
    == "Mimic_RoundhouseLeadingRight"
  )
  assert len(manifest["actions"]["flying_kick"]["source_policy_onnx"]["sha256"]) == 64
  assert "active_fsm_config" in manifest["deploy_configs"]
  assert "user_g1_urdf" in manifest["robot_model_sources"]
  assert "external_g1_xml" in manifest["external_robot_assets"]
  assert "external_scene_g1_xml" in manifest["external_robot_assets"]
  assert "forbidden_g1_23dof_xml" in manifest["external_robot_assets"]
  assert "status_short" in manifest["git"]
  assert manifest["symptom_video"]["size_bytes"] == len(b"not-a-real-mp4")


def test_write_manifest_creates_timestamped_json(tmp_path: Path) -> None:
  _, _, config = _fake_workspace(tmp_path)
  manifest = build_manifest(config)

  manifest_path = write_manifest(manifest, config.output_root, config.timestamp)

  assert manifest_path.name == "manifest.json"
  assert manifest_path.parent.name == "2026-05-22T12-00-00+08-00"
  written = json.loads(manifest_path.read_text(encoding="utf-8"))
  assert written["objective"] == "g1_tracking_phase1_baseline_manifest"


def test_missing_required_bundle_file_fails_before_manifest(tmp_path: Path) -> None:
  _, _, config = _fake_workspace(tmp_path)
  missing = (
    config.mjlab_root
    / ".external/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/flying_kick/exported/policy.onnx"
  )
  missing.unlink()

  with pytest.raises(ManifestError, match="flying_kick.deploy_policy_onnx"):
    build_manifest(config)
