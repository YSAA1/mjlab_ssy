#!/usr/bin/env python3
"""Stdlib-only official Unitree G1 baseline capability preflight."""

from __future__ import annotations

import argparse
import grp
import json
import os
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any

DEFAULT_OFFICIAL_ROOT = Path("/tmp/unitree_rl_mjlab_official_baseline")
DEFAULT_REPORT_OUT = Path(
  "logs/g1_tracking_phase1/"
  "2026-05-23T-official-baseline-preflight/official_baseline_preflight.json"
)


def _parse_flat_yaml(path: Path) -> dict[str, Any]:
  data: dict[str, Any] = {}
  try:
    lines = path.read_text(encoding="utf-8").splitlines()
  except FileNotFoundError as exc:
    raise RuntimeError(f"missing YAML file: {path}") from exc

  for raw_line in lines:
    line = raw_line.split("#", 1)[0].strip()
    if not line or ":" not in line:
      continue
    key, value = line.split(":", 1)
    key = key.strip()
    value = value.strip().strip('"').strip("'")
    if value in {"0", "1"}:
      data[key] = int(value)
    elif value.lower() in {"true", "false"}:
      data[key] = value.lower() == "true"
    else:
      data[key] = value
  return data


def _git_head(root: Path) -> str | None:
  proc = subprocess.run(
    ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )
  if proc.returncode != 0:
    return None
  return proc.stdout.strip()


def _group_names() -> list[str]:
  names: list[str] = []
  for gid in os.getgroups():
    try:
      names.append(grp.getgrgid(gid).gr_name)
    except KeyError:
      names.append(str(gid))
  return sorted(set(names))


def _path_status(path: Path) -> dict[str, Any]:
  exists = path.exists()
  record: dict[str, Any] = {
    "path": str(path),
    "exists": exists,
    "is_file": path.is_file() if exists else False,
    "is_executable": os.access(path, os.X_OK) if exists else False,
  }
  if exists:
    st = path.stat()
    record["mode_octal"] = oct(stat.S_IMODE(st.st_mode))
    record["is_character_device"] = stat.S_ISCHR(st.st_mode)
    record["readable"] = os.access(path, os.R_OK)
    record["writable"] = os.access(path, os.W_OK)
  return record


def _run_jstest(jstest: Path, timeout_s: float) -> dict[str, Any]:
  if not jstest.exists():
    return {
      "ran": False,
      "status": "missing_jstest_binary",
      "returncode": None,
      "stdout": "",
      "stderr": "",
    }
  try:
    proc = subprocess.run(
      [str(jstest)],
      check=False,
      text=True,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      timeout=timeout_s,
    )
  except subprocess.TimeoutExpired as exc:
    stdout = exc.stdout.decode() if isinstance(exc.stdout, bytes) else exc.stdout
    stderr = exc.stderr.decode() if isinstance(exc.stderr, bytes) else exc.stderr
    return {
      "ran": True,
      "status": "timeout_after_open",
      "returncode": None,
      "stdout": (stdout or "").strip(),
      "stderr": (stderr or "").strip(),
      "timeout_s": timeout_s,
    }
  return {
    "ran": True,
    "status": "ok" if proc.returncode == 0 else "failed",
    "returncode": proc.returncode,
    "stdout": proc.stdout.strip(),
    "stderr": proc.stderr.strip(),
    "timeout_s": timeout_s,
  }


def build_report(root: Path, report_device: Path | None, timeout_s: float) -> dict[str, Any]:
  root = root.expanduser().resolve()
  simulate = root / "simulate"
  deploy_g1 = root / "deploy/robots/g1"
  simulate_config_path = simulate / "config.yaml"
  simulate_config = _parse_flat_yaml(simulate_config_path)

  configured_device = Path(str(simulate_config.get("joystick_device", "/dev/input/js0")))
  joystick_device = report_device or configured_device
  jstest = _run_jstest(simulate / "build/jstest", timeout_s)
  binaries = {
    "unitree_mujoco": _path_status(simulate / "build/unitree_mujoco"),
    "g1_ctrl": _path_status(deploy_g1 / "build/g1_ctrl"),
    "jstest": _path_status(simulate / "build/jstest"),
  }
  joystick = _path_status(joystick_device)

  blockers: list[str] = []
  if not root.is_dir():
    blockers.append("missing_official_root")
  if _git_head(root) is None:
    blockers.append("missing_official_git_head")
  if int(simulate_config.get("use_joystick", 0)) != 1:
    blockers.append("official_config_not_using_joystick")
  if str(configured_device) != str(joystick_device):
    blockers.append("joystick_device_override_is_not_clean_default")
  for name, record in binaries.items():
    if not record["exists"]:
      blockers.append(f"missing_{name}_binary")
    elif not record["is_executable"]:
      blockers.append(f"{name}_not_executable")
  if not joystick["exists"]:
    blockers.append("missing_joystick_device")
  elif not joystick["is_character_device"]:
    blockers.append("joystick_device_not_character_device")
  elif not joystick["readable"]:
    blockers.append("joystick_device_not_readable_by_current_user")
  if jstest["status"] == "failed":
    blockers.append("jstest_failed")

  return {
    "objective": "g1_tracking_phase1_clean_official_baseline_preflight",
    "stdlib_agent_preflight": True,
    "non_launching": True,
    "official_root": str(root),
    "official_head": _git_head(root),
    "simulate_config": {
      "path": str(simulate_config_path),
      "use_joystick": simulate_config.get("use_joystick"),
      "joystick_type": simulate_config.get("joystick_type"),
      "configured_joystick_device": str(configured_device),
      "effective_joystick_device": str(joystick_device),
      "robot": simulate_config.get("robot"),
      "robot_scene": simulate_config.get("robot_scene"),
      "interface": simulate_config.get("interface"),
      "enable_elastic_band": simulate_config.get("enable_elastic_band"),
      "has_initial_qpos": "initial_qpos" in simulate_config,
      "has_start_paused": "start_paused" in simulate_config,
    },
    "binaries": binaries,
    "joystick": joystick,
    "current_user": {
      "uid": os.getuid(),
      "gid": os.getgid(),
      "groups": _group_names(),
      "in_input_group": "input" in _group_names(),
    },
    "jstest": jstest,
    "status": "ready" if not blockers else "blocked",
    "blockers": blockers,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
  parser.add_argument("--joystick-device", type=Path)
  parser.add_argument("--jstest-timeout-s", type=float, default=1.0)
  parser.add_argument("--expect-ready", action="store_true")
  parser.add_argument("--report-out", type=Path, default=DEFAULT_REPORT_OUT)
  args = parser.parse_args()

  try:
    report = build_report(args.official_root, args.joystick_device, args.jstest_timeout_s)
  except RuntimeError as exc:
    print(f"official baseline preflight failed: {exc}", file=sys.stderr)
    return 2

  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  args.report_out.parent.mkdir(parents=True, exist_ok=True)
  args.report_out.write_text(output + "\n", encoding="utf-8")
  if args.expect_ready and report["status"] != "ready":
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
