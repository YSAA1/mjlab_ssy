from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/tools/export_rsl_rl_actor_onnx.py"


def _load_script() -> ModuleType:
  spec = importlib.util.spec_from_file_location("export_rsl_rl_actor_onnx", SCRIPT)
  assert spec is not None
  assert spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def test_resolve_metadata_run_path_prefers_explicit_override() -> None:
  module = _load_script()

  result = module.resolve_metadata_run_path(
    checkpoint_file="/logs/rsl_rl/g1_velocity/run/model_50.pt",
    run_dir=Path("/logs/rsl_rl/g1_velocity/restored_run"),
    metadata_run_path="manual-run-path",
  )

  assert result == "manual-run-path"


def test_resolve_metadata_run_path_uses_restored_run_dir() -> None:
  module = _load_script()

  result = module.resolve_metadata_run_path(
    checkpoint_file="/logs/rsl_rl/g1_velocity/local_run/model_50.pt",
    run_dir=Path("/logs/rsl_rl/g1_velocity/restored_run"),
    metadata_run_path=None,
  )

  assert result == "restored_run"


def test_resolve_metadata_run_path_falls_back_to_checkpoint_parent() -> None:
  module = _load_script()

  result = module.resolve_metadata_run_path(
    checkpoint_file="/logs/rsl_rl/g1_velocity/local_run/model_50.pt",
    run_dir=None,
    metadata_run_path=None,
  )

  assert result == "local_run"


def test_attach_actor_export_metadata_attaches_base_metadata(
  tmp_path: Path,
  monkeypatch,
) -> None:
  module = _load_script()
  calls: dict[str, Any] = {}
  env = object()
  onnx_path = tmp_path / "policy.onnx"

  def fake_get_base_metadata(fake_env: object, run_path: str) -> dict[str, Any]:
    calls["env"] = fake_env
    calls["run_path"] = run_path
    return {"run_path": run_path, "joint_names": ["joint_a"]}

  def fake_attach_metadata_to_onnx(path: str, metadata: dict[str, Any]) -> None:
    calls["path"] = path
    calls["metadata"] = metadata

  monkeypatch.setattr(module, "get_base_metadata", fake_get_base_metadata)
  monkeypatch.setattr(module, "attach_metadata_to_onnx", fake_attach_metadata_to_onnx)

  run_path = module.attach_actor_export_metadata(
    onnx_path=onnx_path,
    env=env,
    checkpoint_file="/logs/rsl_rl/g1_velocity/local_run/model_50.pt",
    run_dir=Path("/logs/rsl_rl/g1_velocity/restored_run"),
    metadata_run_path=None,
  )

  assert run_path == "restored_run"
  assert calls == {
    "env": env,
    "run_path": "restored_run",
    "path": str(onnx_path),
    "metadata": {"run_path": "restored_run", "joint_names": ["joint_a"]},
  }
