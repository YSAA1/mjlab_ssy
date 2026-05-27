from __future__ import annotations

import json
from importlib import import_module
from pathlib import Path
from typing import Any, cast

import pytest

from mjlab.sim2sim.unitree import ACTION_BUNDLES, missing_required_source_paths
from mjlab.sim2sim.unitree.cli import main

ROOT = Path(__file__).resolve().parents[2]


def test_pyproject_exposes_unitree_sim2sim_entrypoint() -> None:
  tomllib = cast(Any, import_module("tomllib"))
  pyproject = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))

  assert (
    pyproject["project"]["scripts"]["unitree-sim2sim"]
    == "mjlab.sim2sim.unitree.cli:main"
  )


def test_top_level_help_lists_prepare_g1(capsys: pytest.CaptureFixture[str]) -> None:
  with pytest.raises(SystemExit) as exc:
    main(["--help"])

  assert exc.value.code == 0
  out = capsys.readouterr().out
  assert "prepare-g1" in out
  assert "Unitree G1 sim2sim" in out


def test_prepare_g1_help_documents_required_contract(
  capsys: pytest.CaptureFixture[str],
) -> None:
  with pytest.raises(SystemExit) as exc:
    main(["prepare-g1", "--help"])

  assert exc.value.code == 0
  out = capsys.readouterr().out
  assert "--official-root" in out
  assert "--out-root" in out
  assert "--action" in out
  assert "--automation-sequence" in out
  assert "flying_kick" in out
  assert "roundhouse_leading_right" in out


def test_prepare_g1_requires_source_output_and_action(
  capsys: pytest.CaptureFixture[str],
) -> None:
  with pytest.raises(SystemExit) as exc:
    main(["prepare-g1"])

  assert exc.value.code == 2
  err = capsys.readouterr().err
  assert "--official-root" in err
  assert "--out-root" in err
  assert "--action" in err


def test_prepare_g1_dry_run_has_no_local_external_defaults(
  tmp_path: Path,
  capsys: pytest.CaptureFixture[str],
) -> None:
  official_root = tmp_path / "official"
  out_root = tmp_path / "lane"

  code = main(
    [
      "prepare-g1",
      "--official-root",
      str(official_root),
      "--out-root",
      str(out_root),
      "--action",
      "flying_kick",
      "--dry-run",
    ]
  )

  assert code == 0
  payload = json.loads(capsys.readouterr().out)
  request = payload["request"]
  assert request["official_root"] == str(official_root.resolve())
  assert request["out_root"] == str(out_root.resolve())
  assert request["action"] == "flying_kick"
  assert request["state_name"] == ACTION_BUNDLES["flying_kick"].state_name
  assert request["deviation_labels"] == ["automation_input"]


def test_source_checkout_shape_validation_names_missing_paths(tmp_path: Path) -> None:
  missing = missing_required_source_paths(tmp_path)

  assert Path("simulate/config.yaml") in missing
  assert Path("deploy/robots/g1/config/config.yaml") in missing
  assert Path("src/assets/robots/unitree_g1/xmls/scene_g1.xml") in missing


def test_new_unitree_package_does_not_embed_deleted_worktree_defaults() -> None:
  package_root = ROOT / "src/mjlab/sim2sim"
  texts = [path.read_text(encoding="utf-8") for path in package_root.rglob("*.py")]
  combined = "\n".join(texts)

  assert "g1-flying-kick-main" not in combined
  assert "/home/ssy/ssy_files/mjlab/.worktrees" not in combined
  assert ".external/unitree_rl_mjlab" not in combined
