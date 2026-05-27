"""Diagnose G1 phase-1 deploy entry pose gaps."""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
import yaml

ACTION_STATES = {
  "Mimic_FlyingKick": "flying_kick",
  "Mimic_RoundhouseLeadingRight": "roundhouse_leading_right",
}
Q_RESPONSE_RE = re.compile(
  r"\[PHASE1\]\s+event=q_response\s+action=(?P<state>\S+)\s+"
  r"step=(?P<step>\d+)\s+q_err_l2=(?P<q_err_l2>-?\d+(?:\.\d+)?)"
)


class EntryGapError(RuntimeError):
  """Raised when the entry-gap inputs are invalid."""


def _load_json(path: Path) -> dict[str, Any]:
  with path.open("r", encoding="utf-8") as handle:
    return json.load(handle)


def _load_yaml(path: Path) -> dict[str, Any]:
  with path.open("r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle)
  if not isinstance(data, dict):
    raise EntryGapError(f"YAML root must be a mapping: {path}")
  return data


def _joint_names_from_xml(path: Path) -> list[str]:
  root = ET.parse(path).getroot()
  names = [
    joint.get("name")
    for joint in root.findall(".//joint")
    if joint.get("type") != "free"
  ]
  return [name for name in names if name]


def _motion_fps(motion: np.lib.npyio.NpzFile) -> float:
  if "fps" not in motion:
    return 50.0
  value = np.asarray(motion["fps"]).reshape(-1)
  if value.size == 0:
    return 50.0
  return float(value[0])


def _top_joint_gaps(
  diff: np.ndarray, joint_names: list[str], *, limit: int = 8
) -> list[dict[str, Any]]:
  order = np.argsort(np.abs(diff))[::-1][:limit]
  return [
    {
      "index": int(idx),
      "joint": joint_names[idx] if idx < len(joint_names) else f"joint_{idx}",
      "default_minus_motion": round(float(diff[idx]), 6),
    }
    for idx in order
  ]


def _first_logged_q_err(log_path: Path) -> dict[str, float]:
  first: dict[str, float] = {}
  if not log_path.is_file():
    return first
  for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
    match = Q_RESPONSE_RE.search(line)
    if not match:
      continue
    action = ACTION_STATES.get(match.group("state"))
    if action is None or action in first:
      continue
    if int(match.group("step")) != 1:
      continue
    first[action] = float(match.group("q_err_l2"))
  return first


def _log_q_errs(logs: list[Path]) -> dict[str, float]:
  merged: dict[str, float] = {}
  for path in logs:
    source = path / "g1_ctrl.log" if path.is_dir() else path
    for action, q_err in _first_logged_q_err(source).items():
      merged.setdefault(action, q_err)
  return merged


def analyze_entry_gaps(
  manifest_path: Path,
  *,
  max_entry_gap_l2: float = 0.5,
  max_entry_gap_max: float = 0.35,
  logs: list[Path] | None = None,
) -> dict[str, Any]:
  manifest = _load_json(manifest_path)
  xml_info = manifest.get("robot_model_sources", {}).get("mjlab_g1_xml", {})
  xml_path = Path(xml_info.get("path", ""))
  if not xml_path.is_file():
    raise EntryGapError(f"Missing mjlab_g1_xml path in manifest: {manifest_path}")
  joint_names = _joint_names_from_xml(xml_path)
  logged_q_errs = _log_q_errs(logs or [])

  action_reports: dict[str, Any] = {}
  for action, bundle in sorted(manifest.get("actions", {}).items()):
    deploy_yaml = Path(bundle["deploy_yaml"]["path"])
    motion_npz = Path(bundle["deploy_motion_npz"]["path"])
    deploy = _load_yaml(deploy_yaml)
    default = np.asarray(deploy.get("default_joint_pos", []), dtype=float)
    with np.load(motion_npz) as motion:
      joint_pos = np.asarray(motion["joint_pos"], dtype=float)
      fps = _motion_fps(motion)

    if joint_pos.ndim != 2:
      raise EntryGapError(f"motion joint_pos must be 2D: {motion_npz}")
    if len(default) != joint_pos.shape[1]:
      raise EntryGapError(
        f"default_joint_pos length {len(default)} does not match motion dim "
        f"{joint_pos.shape[1]}: {deploy_yaml}"
      )

    frame0_diff = default - joint_pos[0]
    frame_gaps = np.linalg.norm(joint_pos - default[None, :], axis=1)
    best_frame = int(np.argmin(frame_gaps))
    frame0_gap_l2 = float(np.linalg.norm(frame0_diff))
    frame0_gap_max = float(np.max(np.abs(frame0_diff)))
    passed = frame0_gap_l2 <= max_entry_gap_l2 and frame0_gap_max <= max_entry_gap_max
    first_logged_q_err = logged_q_errs.get(action)
    log_delta = (
      round(abs(first_logged_q_err - frame0_gap_l2), 6)
      if first_logged_q_err is not None
      else None
    )
    action_reports[action] = {
      "passed": passed,
      "reason": None if passed else "entry_state_pose_mismatch",
      "frame0_gap_l2": round(frame0_gap_l2, 6),
      "frame0_gap_max": round(frame0_gap_max, 6),
      "max_entry_gap_l2": max_entry_gap_l2,
      "max_entry_gap_max": max_entry_gap_max,
      "best_default_pose_frame": best_frame,
      "best_default_pose_time_s": round(best_frame / fps, 6),
      "best_default_pose_gap_l2": round(float(frame_gaps[best_frame]), 6),
      "first_logged_q_err_l2": first_logged_q_err,
      "first_logged_q_err_delta_l2": log_delta,
      "top_joint_gaps": _top_joint_gaps(frame0_diff, joint_names),
    }

  passed = bool(action_reports) and all(
    report["passed"] for report in action_reports.values()
  )
  return {
    "schema_version": 1,
    "manifest": str(manifest_path.resolve()),
    "passed": passed,
    "primary_reason": None if passed else "entry_state_pose_mismatch",
    "actions": action_reports,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Diagnose whether G1 phase-1 deploy starts from the trained reference pose."
  )
  parser.add_argument("--manifest", required=True, type=Path)
  parser.add_argument("--log", action="append", default=[], type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--max-entry-gap-l2", default=0.5, type=float)
  parser.add_argument("--max-entry-gap-max", default=0.35, type=float)
  parser.add_argument("--expect-failure", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    report = analyze_entry_gaps(
      args.manifest,
      max_entry_gap_l2=args.max_entry_gap_l2,
      max_entry_gap_max=args.max_entry_gap_max,
      logs=args.log,
    )
  except EntryGapError as exc:
    print(str(exc), file=sys.stderr)
    return 2

  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")
  if args.expect_failure:
    return 0 if not report["passed"] else 1
  return 0 if report["passed"] else 1


if __name__ == "__main__":
  raise SystemExit(main())
