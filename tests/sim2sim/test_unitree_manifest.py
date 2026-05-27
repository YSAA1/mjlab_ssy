from __future__ import annotations

import json
from pathlib import Path

from test_unitree_lane_generation import _request

from mjlab.sim2sim.unitree import MANIFEST_NAME, prepare_g1_lane


def test_manifest_changed_paths_exist_under_output_root(tmp_path: Path) -> None:
  request = _request(tmp_path)

  manifest = prepare_g1_lane(request)

  for relative_path in manifest["changed_paths"]:
    assert (request.output.root / relative_path).exists(), relative_path


def test_manifest_records_source_output_action_and_policy_hashes(
  tmp_path: Path,
) -> None:
  request = _request(tmp_path)

  manifest = prepare_g1_lane(request)
  written = json.loads((request.output.root / MANIFEST_NAME).read_text())

  assert written["source_root"] == str(request.source.root.resolve())
  assert written["out_root"] == str(request.output.root.resolve())
  assert written["manifest_path"] == str(
    (request.output.root / MANIFEST_NAME).resolve()
  )
  assert written["action"]["name"] == "flying_kick"
  assert written["policy_assets"]
  assert all(len(asset["sha256"]) == 64 for asset in written["policy_assets"])
  assert written["changed_paths"] == manifest["changed_paths"]
