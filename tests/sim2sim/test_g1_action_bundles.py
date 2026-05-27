from __future__ import annotations

from pathlib import Path

from mjlab.sim2sim.unitree import ACTION_BUNDLES


def test_g1_action_bundles_cover_productized_dual_kick_actions() -> None:
  assert sorted(ACTION_BUNDLES) == ["flying_kick", "roundhouse_leading_right"]


def test_flying_kick_bundle_contract() -> None:
  bundle = ACTION_BUNDLES["flying_kick"]

  assert bundle.state_name == "Mimic_FlyingKick"
  assert bundle.policy_subdir == "flying_kick"
  assert bundle.trigger == "RB + X"
  assert bundle.required_policy_files(Path("/policy")) == (
    Path("/policy/exported/policy.onnx"),
    Path("/policy/params/deploy.yaml"),
    Path("/policy/params/flying_kick.npz"),
  )


def test_roundhouse_bundle_contract() -> None:
  bundle = ACTION_BUNDLES["roundhouse_leading_right"]

  assert bundle.state_name == "Mimic_RoundhouseLeadingRight"
  assert bundle.policy_subdir == "roundhouse_leading_right"
  assert bundle.trigger == "RB + Y"
  assert bundle.required_policy_files(Path("/policy")) == (
    Path("/policy/exported/policy.onnx"),
    Path("/policy/params/deploy.yaml"),
    Path("/policy/params/roundhouse_leading_right.npz"),
  )
