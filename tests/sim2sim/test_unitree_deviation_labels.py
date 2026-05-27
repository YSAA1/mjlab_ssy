from __future__ import annotations

from pathlib import Path

from test_unitree_lane_generation import _request

from mjlab.sim2sim.unitree import prepare_g1_lane


def test_default_generation_labels_automation_and_policy_only(tmp_path: Path) -> None:
  request = _request(tmp_path)

  manifest = prepare_g1_lane(request)

  assert manifest["deviation_labels"] == ["automation_input", "policy_asset"]


def test_diagnostic_label_requires_diagnostic_trace_option(tmp_path: Path) -> None:
  default_manifest = prepare_g1_lane(_request(tmp_path, out_name="default"))
  trace_manifest = prepare_g1_lane(
    _request(tmp_path, out_name="trace", diagnostic_trace=True)
  )

  assert "diagnostic_trace" not in default_manifest["deviation_labels"]
  assert "diagnostic_trace" in trace_manifest["deviation_labels"]


def test_model_labels_require_mode15_option(tmp_path: Path) -> None:
  manifest = prepare_g1_lane(_request(tmp_path, use_mjlab_mode15_model=True))

  assert "mjlab_mode15_model" in manifest["deviation_labels"]
  assert "official_joint_passive_defaults" in manifest["deviation_labels"]
  assert "clean_official" not in manifest["deviation_labels"]
