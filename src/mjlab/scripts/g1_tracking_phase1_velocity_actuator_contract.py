"""Audit G1 Velocity actuator limits between mjlab training and Unitree sim."""

from __future__ import annotations

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

DEFAULT_EXTERNAL_SCENE = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/scene_g1.xml"
)
DEFAULT_USER_G1_XML = Path(
  "/home/ssy/文档/xwechat_files/wxid_k9vao1u7f11t22_a132/msg/file/2026-04/g1_new.xml"
)


class ActuatorContractError(RuntimeError):
  """Raised when the actuator contract audit cannot run."""


def _xml_joint_names(path: Path) -> list[str]:
  root = ET.parse(path).getroot()
  return [joint.attrib["name"] for joint in root.findall(".//joint")]


def _semantic_xml_rows(path: Path) -> list[tuple[str, tuple[tuple[str, str], ...]]]:
  root = ET.parse(path).getroot()
  rows: list[tuple[str, tuple[tuple[str, str], ...]]] = []
  for elem in root.iter():
    if elem.tag not in {
      "compiler",
      "default",
      "asset",
      "mesh",
      "visual",
      "worldbody",
      "body",
      "joint",
      "geom",
      "site",
      "inertial",
    }:
      continue
    rows.append((elem.tag, tuple(sorted(elem.attrib.items()))))
  return rows


def _semantic_xml_match(left: Path, right: Path) -> dict[str, Any]:
  left_rows = _semantic_xml_rows(left)
  right_rows = _semantic_xml_rows(right)
  first_diff = None
  for index, (left_row, right_row) in enumerate(
    zip(left_rows, right_rows, strict=False)
  ):
    if left_row != right_row:
      first_diff = {
        "index": index,
        "left": left_row,
        "right": right_row,
      }
      break
  return {
    "left": str(left),
    "right": str(right),
    "match": left_rows == right_rows,
    "left_rows": len(left_rows),
    "right_rows": len(right_rows),
    "first_diff": first_diff,
  }


def _expected_mjlab_effort_limits(
  *,
  mjlab_g1_xml: Path | None = None,
) -> dict[str, float]:
  from mjlab.actuator import BuiltinPositionActuatorCfg
  from mjlab.asset_zoo.robots.unitree_g1.g1_constants import (
    G1_ARTICULATION,
    G1_XML,
  )

  joint_names = _xml_joint_names(mjlab_g1_xml or G1_XML)
  expected: dict[str, float] = {}
  for actuator in G1_ARTICULATION.actuators:
    if not isinstance(actuator, BuiltinPositionActuatorCfg):
      continue
    if actuator.effort_limit is None:
      continue
    for pattern in actuator.target_names_expr:
      compiled = re.compile(pattern)
      matches = [
        joint_name for joint_name in joint_names if compiled.fullmatch(joint_name)
      ]
      if not matches:
        raise ActuatorContractError(
          f"G1 actuator pattern did not match any joint: {pattern}"
        )
      for joint_name in matches:
        if joint_name in expected:
          raise ActuatorContractError(
            f"duplicate expected actuator limit for joint: {joint_name}"
          )
        expected[joint_name] = float(actuator.effort_limit)
  missing = [joint_name for joint_name in joint_names if joint_name not in expected]
  if missing:
    raise ActuatorContractError(
      f"missing expected actuator effort limits for joints: {missing}"
    )
  return expected


def _parse_ctrlrange(value: str, *, joint_name: str) -> tuple[float, float]:
  parts = value.split()
  if len(parts) != 2:
    raise ActuatorContractError(
      f"motor ctrlrange for {joint_name} must have two values: {value!r}"
    )
  return (float(parts[0]), float(parts[1]))


def _external_scene_ctrlranges(scene_xml: Path) -> dict[str, dict[str, Any]]:
  root = ET.parse(scene_xml).getroot()
  motors = root.findall(".//actuator/motor")
  if not motors:
    raise ActuatorContractError(f"no <actuator><motor> entries found: {scene_xml}")
  rows: dict[str, dict[str, Any]] = {}
  for motor in motors:
    joint_name = motor.attrib.get("joint")
    if not joint_name:
      continue
    low, high = _parse_ctrlrange(
      motor.attrib.get("ctrlrange", ""), joint_name=joint_name
    )
    rows[joint_name] = {
      "motor_name": motor.attrib.get("name"),
      "ctrlrange": [low, high],
      "abs_limit": max(abs(low), abs(high)),
      "symmetric": abs(abs(low) - abs(high)) <= 1e-9,
    }
  return rows


def audit_velocity_actuator_contract(
  *,
  external_scene: Path = DEFAULT_EXTERNAL_SCENE,
  user_g1_xml: Path | None = DEFAULT_USER_G1_XML,
  mjlab_g1_xml: Path | None = None,
  tolerance: float = 1e-6,
) -> dict[str, Any]:
  from mjlab.asset_zoo.robots.unitree_g1.g1_constants import G1_XML

  mjlab_xml = mjlab_g1_xml or G1_XML
  expected = _expected_mjlab_effort_limits(mjlab_g1_xml=mjlab_xml)
  external = _external_scene_ctrlranges(external_scene)
  joint_names = _xml_joint_names(mjlab_xml)
  mismatches: list[dict[str, Any]] = []
  rows: list[dict[str, Any]] = []
  for joint_name in joint_names:
    expected_limit = expected[joint_name]
    external_row = external.get(joint_name)
    if external_row is None:
      row = {
        "joint": joint_name,
        "expected_effort_limit": expected_limit,
        "external_abs_limit": None,
        "status": "missing_external_motor",
      }
      mismatches.append(row)
      rows.append(row)
      continue
    actual_limit = float(external_row["abs_limit"])
    gap = actual_limit - expected_limit
    status = "match" if abs(gap) <= tolerance else "mismatch"
    row = {
      "joint": joint_name,
      "expected_effort_limit": round(expected_limit, 6),
      "external_abs_limit": round(actual_limit, 6),
      "gap_external_minus_expected": round(gap, 6),
      "external_ctrlrange": external_row["ctrlrange"],
      "external_ctrlrange_symmetric": external_row["symmetric"],
      "status": status,
    }
    if status != "match" or not external_row["symmetric"]:
      mismatches.append(row)
    rows.append(row)

  semantic_asset_checks: list[dict[str, Any]] = []
  if user_g1_xml is not None and user_g1_xml.is_file():
    semantic_asset_checks.append(_semantic_xml_match(user_g1_xml, mjlab_xml))
  external_g1_xml = external_scene.with_name("g1.xml")
  if external_g1_xml.is_file():
    semantic_asset_checks.append(_semantic_xml_match(mjlab_xml, external_g1_xml))

  mismatched_joints = [row["joint"] for row in mismatches]
  right_ankle_mismatch = any(
    joint in {"right_ankle_pitch_joint", "right_ankle_roll_joint"}
    for joint in mismatched_joints
  )
  return {
    "schema_version": 1,
    "external_scene": str(external_scene),
    "mjlab_g1_xml": str(mjlab_xml),
    "user_g1_xml": str(user_g1_xml) if user_g1_xml is not None else None,
    "joint_count": len(joint_names),
    "semantic_asset_checks": semantic_asset_checks,
    "rows": rows,
    "mismatches": mismatches,
    "decision": {
      "actuator_force_contract_matches": not mismatches,
      "mismatch_count": len(mismatches),
      "right_ankle_limit_mismatch": right_ankle_mismatch,
      "primary_mismatch": "external_scene_motor_ctrlrange" if mismatches else None,
      "recommended_next_step": (
        "align_external_scene_ctrlrange_with_mjlab_training_effort_limits_then_rerun_passive_velocity_smoke"
        if mismatches
        else "continue_velocity_closed_loop_diagnosis"
      ),
      "real_robot_gate": "locked",
    },
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description=(
      "Compare mjlab G1 training actuator force limits against Unitree MuJoCo "
      "scene motor ctrlrange."
    )
  )
  parser.add_argument("--external-scene", type=Path, default=DEFAULT_EXTERNAL_SCENE)
  parser.add_argument("--user-g1-xml", type=Path, default=DEFAULT_USER_G1_XML)
  parser.add_argument("--mjlab-g1-xml", type=Path, default=None)
  parser.add_argument("--tolerance", type=float, default=1e-6)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-mismatch", action="store_true")
  parser.add_argument("--expect-match", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = audit_velocity_actuator_contract(
    external_scene=args.external_scene,
    user_g1_xml=args.user_g1_xml,
    mjlab_g1_xml=args.mjlab_g1_xml,
    tolerance=args.tolerance,
  )
  payload = json.dumps(report, indent=2, sort_keys=True)
  print(payload)
  if args.report_out:
    args.report_out.parent.mkdir(parents=True, exist_ok=True)
    args.report_out.write_text(payload + "\n", encoding="utf-8")

  matches = bool(report["decision"]["actuator_force_contract_matches"])
  if args.expect_mismatch and matches:
    return 1
  if args.expect_match and not matches:
    return 1
  return 0


if __name__ == "__main__":
  try:
    raise SystemExit(main())
  except ActuatorContractError as exc:
    print(f"error: {exc}", file=sys.stderr)
    raise SystemExit(2) from exc
