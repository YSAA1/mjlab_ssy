"""Build the phase-1 baseline manifest for G1 tracking deploy diagnostics."""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

DEFAULT_MJLAB_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_WORKTREE = DEFAULT_MJLAB_ROOT
DEFAULT_USER_G1_URDF = Path(
  "/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/file/2026-04/g1_29dof_mode_15.urdf"
)
DEFAULT_USER_G1_XML = Path(
  "/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/file/2026-04/g1_new.xml"
)
DEFAULT_SYMPTOM_VIDEO = Path(
  "/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/video/2026-05/05e60f0aa3f07a32611272b5beaa9d3a.mp4"
)


@dataclass(frozen=True)
class ActionPaths:
  name: str
  state_name: str
  source_run_dir: Path
  source_policy_onnx: Path
  source_motion_npz: Path
  deploy_policy_onnx: Path
  deploy_motion_npz: Path
  deploy_yaml: Path


@dataclass(frozen=True)
class ManifestConfig:
  worktree: Path
  mjlab_root: Path
  output_root: Path
  timestamp: str | None
  dry_run: bool
  flying_policy_onnx: Path | None
  roundhouse_policy_onnx: Path | None
  flying_run_dir: Path | None
  roundhouse_run_dir: Path | None
  flying_experiment_name: str
  roundhouse_experiment_name: str
  flying_run_name_pattern: str
  roundhouse_run_name_pattern: str
  user_g1_urdf: Path
  user_g1_xml: Path
  symptom_video: Path


class ManifestError(RuntimeError):
  """Raised when the manifest cannot be created safely."""


def _resolve(path: Path) -> Path:
  return path.expanduser().resolve()


def _repo_relative(path: Path, root: Path) -> str | None:
  try:
    return str(path.resolve().relative_to(root.resolve()))
  except ValueError:
    return None


def _run_git(worktree: Path, args: list[str]) -> str:
  try:
    proc = subprocess.run(
      ["git", *args],
      cwd=worktree,
      check=False,
      text=True,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
    )
  except FileNotFoundError:
    return "git-not-found"
  if proc.returncode != 0:
    return proc.stderr.strip() or f"git failed: {' '.join(args)}"
  return proc.stdout.rstrip("\n")


def _latest_run_dir(logs_root: Path, pattern: str) -> Path | None:
  if not logs_root.is_dir():
    return None
  candidates = [
    child
    for child in logs_root.iterdir()
    if child.is_dir() and fnmatch.fnmatch(child.name, pattern)
  ]
  if not candidates:
    return None
  return max(candidates, key=lambda path: path.stat().st_mtime)


def _select_action_paths(
  *,
  worktree: Path,
  deploy: Path,
  action_name: str,
  state_name: str,
  experiment_name: str,
  run_name_pattern: str,
  override_policy: Path | None,
  override_run_dir: Path | None,
  policy_filename: str,
  source_motion: Path,
  deploy_subdir: str,
  deploy_motion_filename: str,
) -> ActionPaths:
  if override_policy is not None:
    source_policy = _resolve(override_policy)
    source_run_dir = source_policy.parent
  else:
    source_run_dir = _resolve(override_run_dir) if override_run_dir else None
    if source_run_dir is None:
      source_run_dir = _latest_run_dir(
        worktree / "logs/rsl_rl" / experiment_name,
        run_name_pattern,
      )
    if source_run_dir is None:
      raise ManifestError(
        f"No run directory found for {action_name} under "
        f"{worktree / 'logs/rsl_rl' / experiment_name} matching "
        f"{run_name_pattern!r}."
      )
    source_policy = source_run_dir / policy_filename

  deploy_dir = deploy / "config/policy/mimic" / deploy_subdir
  return ActionPaths(
    name=action_name,
    state_name=state_name,
    source_run_dir=source_run_dir,
    source_policy_onnx=source_policy,
    source_motion_npz=source_motion,
    deploy_policy_onnx=deploy_dir / "exported/policy.onnx",
    deploy_motion_npz=deploy_dir / "params" / deploy_motion_filename,
    deploy_yaml=deploy_dir / "params/deploy.yaml",
  )


def _sha256(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as handle:
    for chunk in iter(lambda: handle.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def _file_record(path: Path, *, root: Path | None = None) -> dict[str, Any]:
  resolved = path.resolve()
  stat = resolved.stat()
  record: dict[str, Any] = {
    "path": str(resolved),
    "sha256": _sha256(resolved),
    "size_bytes": stat.st_size,
    "mtime_ns": stat.st_mtime_ns,
  }
  if root is not None:
    record["repo_relative_path"] = _repo_relative(resolved, root)
  return record


def _optional_ffprobe(video: Path) -> dict[str, Any]:
  if shutil.which("ffprobe") is None:
    return {"available": False}
  proc = subprocess.run(
    [
      "ffprobe",
      "-v",
      "error",
      "-select_streams",
      "v:0",
      "-show_entries",
      "stream=width,height,avg_frame_rate,nb_frames,duration",
      "-show_entries",
      "format=duration",
      "-of",
      "json",
      str(video),
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )
  if proc.returncode != 0:
    return {
      "available": True,
      "error": proc.stderr.strip(),
    }
  try:
    data = json.loads(proc.stdout)
  except json.JSONDecodeError as exc:
    return {
      "available": True,
      "error": f"invalid ffprobe json: {exc}",
    }
  stream = (data.get("streams") or [{}])[0]
  return {
    "available": True,
    "width": stream.get("width"),
    "height": stream.get("height"),
    "avg_frame_rate": stream.get("avg_frame_rate"),
    "nb_frames": stream.get("nb_frames"),
    "stream_duration_s": stream.get("duration"),
    "format_duration_s": (data.get("format") or {}).get("duration"),
  }


def _video_record(path: Path) -> dict[str, Any]:
  record = _file_record(path)
  record["ffprobe"] = _optional_ffprobe(path)
  return record


def _missing_required(paths: dict[str, Path]) -> list[str]:
  missing = []
  for label, path in paths.items():
    if not path.is_file():
      missing.append(f"{label}: {path}")
  return missing


def _action_record(action: ActionPaths, worktree: Path) -> dict[str, Any]:
  return {
    "name": action.name,
    "state_name": action.state_name,
    "source_run_dir": str(action.source_run_dir.resolve()),
    "source_policy_onnx": _file_record(action.source_policy_onnx, root=worktree),
    "source_motion_npz": _file_record(action.source_motion_npz, root=worktree),
    "deploy_policy_onnx": _file_record(action.deploy_policy_onnx),
    "deploy_motion_npz": _file_record(action.deploy_motion_npz),
    "deploy_yaml": _file_record(action.deploy_yaml),
  }


def build_manifest(config: ManifestConfig) -> dict[str, Any]:
  worktree = _resolve(config.worktree)
  mjlab_root = _resolve(config.mjlab_root)
  unitree = mjlab_root / ".external/unitree_rl_mjlab"
  deploy = unitree / "deploy/robots/g1"
  simulate = unitree / "simulate"
  timestamp = config.timestamp or datetime.now().astimezone().isoformat(
    timespec="seconds"
  )

  flying = _select_action_paths(
    worktree=worktree,
    deploy=deploy,
    action_name="flying_kick",
    state_name="Mimic_FlyingKick",
    experiment_name=config.flying_experiment_name,
    run_name_pattern=config.flying_run_name_pattern,
    override_policy=config.flying_policy_onnx,
    override_run_dir=config.flying_run_dir,
    policy_filename="flying_kick_deploy_actor.onnx",
    source_motion=worktree / "data/motions/g1_flying_kick/mjlab/motion.npz",
    deploy_subdir="flying_kick",
    deploy_motion_filename="flying_kick.npz",
  )
  roundhouse = _select_action_paths(
    worktree=worktree,
    deploy=deploy,
    action_name="roundhouse_leading_right",
    state_name="Mimic_RoundhouseLeadingRight",
    experiment_name=config.roundhouse_experiment_name,
    run_name_pattern=config.roundhouse_run_name_pattern,
    override_policy=config.roundhouse_policy_onnx,
    override_run_dir=config.roundhouse_run_dir,
    policy_filename="roundhouse_leading_right_deploy_actor.onnx",
    source_motion=worktree
    / "data/motions/g1_roundhouse_leading_right/mjlab/motion.npz",
    deploy_subdir="roundhouse_leading_right",
    deploy_motion_filename="roundhouse_leading_right.npz",
  )

  required_files: dict[str, Path] = {
    "active_fsm_config": deploy / "config/config.yaml",
    "sim_config": simulate / "config.yaml",
    "shared_deploy_yaml_source": deploy
    / "config/policy/mimic/getup/params/deploy.yaml",
    "external_g1_xml": unitree / "src/assets/robots/unitree_g1/xmls/g1.xml",
    "external_scene_g1_xml": unitree / "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
    "user_g1_urdf": config.user_g1_urdf,
    "user_g1_xml": config.user_g1_xml,
    "mjlab_g1_urdf": worktree
    / "src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf",
    "mjlab_g1_xml": worktree / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml",
    "symptom_video": config.symptom_video,
  }
  for action in (flying, roundhouse):
    prefix = action.name
    required_files.update(
      {
        f"{prefix}.source_policy_onnx": action.source_policy_onnx,
        f"{prefix}.source_motion_npz": action.source_motion_npz,
        f"{prefix}.deploy_policy_onnx": action.deploy_policy_onnx,
        f"{prefix}.deploy_motion_npz": action.deploy_motion_npz,
        f"{prefix}.deploy_yaml": action.deploy_yaml,
      }
    )

  missing = _missing_required(required_files)
  if missing:
    raise ManifestError("Missing required files:\n" + "\n".join(missing))

  manifest: dict[str, Any] = {
    "schema_version": 1,
    "created_at": timestamp,
    "dry_run": config.dry_run,
    "objective": "g1_tracking_phase1_baseline_manifest",
    "non_launching": True,
    "worktree": str(worktree),
    "mjlab_root": str(mjlab_root),
    "git": {
      "head": _run_git(worktree, ["rev-parse", "HEAD"]),
      "status_short": _run_git(worktree, ["status", "--short"]).splitlines(),
    },
    "scripts": {
      "manifest": str(
        (worktree / "scripts/tools/g1_tracking_phase1_manifest.py").resolve()
      ),
      "dual_real_deploy": str(
        (worktree / "scripts/tools/run_g1_dual_kicks_real_deploy.sh").resolve()
      ),
      "flying_sim2sim": str(
        (worktree / "scripts/tools/run_flying_kick_sim2sim.sh").resolve()
      ),
      "roundhouse_sim2sim": str(
        (worktree / "scripts/tools/run_roundhouse_leading_right_sim2sim.sh").resolve()
      ),
    },
    "robot_model_sources": {
      "user_g1_urdf": _file_record(config.user_g1_urdf),
      "user_g1_xml": _file_record(config.user_g1_xml),
      "mjlab_g1_urdf": _file_record(
        worktree / "src/mjlab/asset_zoo/robots/unitree_g1/urdf/g1_29dof_mode_15.urdf",
        root=worktree,
      ),
      "mjlab_g1_xml": _file_record(
        worktree / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml",
        root=worktree,
      ),
    },
    "deploy_configs": {
      "active_fsm_config": _file_record(deploy / "config/config.yaml"),
      "sim_config": _file_record(simulate / "config.yaml"),
      "shared_deploy_yaml_source": _file_record(
        deploy / "config/policy/mimic/getup/params/deploy.yaml"
      ),
    },
    "external_robot_assets": {
      "external_g1_xml": _file_record(
        unitree / "src/assets/robots/unitree_g1/xmls/g1.xml"
      ),
      "external_scene_g1_xml": _file_record(
        unitree / "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
      ),
    },
    "actions": {
      flying.name: _action_record(flying, worktree),
      roundhouse.name: _action_record(roundhouse, worktree),
    },
    "symptom_video": _video_record(config.symptom_video),
  }
  forbidden_g1_23dof = unitree / "src/assets/robots/unitree_g1/xmls/g1_23dof.xml"
  if forbidden_g1_23dof.is_file():
    manifest["external_robot_assets"]["forbidden_g1_23dof_xml"] = _file_record(
      forbidden_g1_23dof
    )
  return manifest


def write_manifest(
  manifest: dict[str, Any], output_root: Path, timestamp: str | None
) -> Path:
  safe_timestamp = (
    timestamp or datetime.now().astimezone().isoformat(timespec="seconds")
  ).replace(":", "-")
  out_dir = output_root / safe_timestamp
  out_dir.mkdir(parents=True, exist_ok=False)
  manifest_path = out_dir / "manifest.json"
  manifest_path.write_text(
    json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
    encoding="utf-8",
  )
  return manifest_path


def parse_args(argv: list[str] | None = None) -> ManifestConfig:
  parser = argparse.ArgumentParser(
    description="Create the G1 tracking phase-1 baseline manifest."
  )
  parser.add_argument(
    "--worktree",
    type=Path,
    default=Path(os.environ.get("MJLAB_WORKTREE", DEFAULT_WORKTREE)),
  )
  parser.add_argument(
    "--mjlab-root",
    type=Path,
    default=Path(os.environ.get("MJLAB_ROOT", DEFAULT_MJLAB_ROOT)),
  )
  parser.add_argument(
    "--output-root",
    type=Path,
    default=None,
    help="Directory for timestamped manifest output. Defaults to WORKTREE/logs/g1_tracking_phase1.",
  )
  parser.add_argument("--timestamp", default=None)
  parser.add_argument("--dry-run", action="store_true")
  parser.add_argument("--flying-policy-onnx", type=Path, default=None)
  parser.add_argument("--roundhouse-policy-onnx", type=Path, default=None)
  parser.add_argument("--flying-run-dir", type=Path, default=None)
  parser.add_argument("--roundhouse-run-dir", type=Path, default=None)
  parser.add_argument(
    "--flying-experiment-name",
    default=os.environ.get(
      "MJLAB_FLYING_EXPERIMENT_NAME", "g1_tracking_acrobatics_no_state"
    ),
  )
  parser.add_argument(
    "--roundhouse-experiment-name",
    default=os.environ.get(
      "MJLAB_ROUNDHOUSE_EXPERIMENT_NAME",
      "g1_tracking_roundhouse_leading_right_no_state",
    ),
  )
  parser.add_argument(
    "--flying-run-name-pattern",
    default=os.environ.get(
      "MJLAB_FLYING_RUN_NAME_PATTERN",
      "*g1_mode15_flying_kick_4096env_5000iter*",
    ),
  )
  parser.add_argument(
    "--roundhouse-run-name-pattern",
    default=os.environ.get(
      "MJLAB_ROUNDHOUSE_RUN_NAME_PATTERN",
      "*g1_mode15_roundhouse_leading_right*",
    ),
  )
  parser.add_argument("--user-g1-urdf", type=Path, default=DEFAULT_USER_G1_URDF)
  parser.add_argument("--user-g1-xml", type=Path, default=DEFAULT_USER_G1_XML)
  parser.add_argument("--symptom-video", type=Path, default=DEFAULT_SYMPTOM_VIDEO)
  args = parser.parse_args(argv)
  output_root = args.output_root or args.worktree / "logs/g1_tracking_phase1"
  return ManifestConfig(
    worktree=args.worktree,
    mjlab_root=args.mjlab_root,
    output_root=output_root,
    timestamp=args.timestamp,
    dry_run=args.dry_run,
    flying_policy_onnx=args.flying_policy_onnx,
    roundhouse_policy_onnx=args.roundhouse_policy_onnx,
    flying_run_dir=args.flying_run_dir,
    roundhouse_run_dir=args.roundhouse_run_dir,
    flying_experiment_name=args.flying_experiment_name,
    roundhouse_experiment_name=args.roundhouse_experiment_name,
    flying_run_name_pattern=args.flying_run_name_pattern,
    roundhouse_run_name_pattern=args.roundhouse_run_name_pattern,
    user_g1_urdf=args.user_g1_urdf,
    user_g1_xml=args.user_g1_xml,
    symptom_video=args.symptom_video,
  )


def main(argv: list[str] | None = None) -> int:
  try:
    config = parse_args(argv)
    manifest = build_manifest(config)
    if config.dry_run:
      print(json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True))
      return 0
    manifest_path = write_manifest(manifest, config.output_root, config.timestamp)
    print(f"Wrote G1 tracking phase-1 manifest: {manifest_path}")
    return 0
  except ManifestError as exc:
    print(str(exc), file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
