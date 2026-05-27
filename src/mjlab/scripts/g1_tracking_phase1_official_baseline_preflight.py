"""Preflight the clean upstream Unitree G1 sim2sim baseline launch."""

from __future__ import annotations

import argparse
import grp
import json
import os
import stat
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

DEFAULT_OFFICIAL_ROOT = Path("/tmp/unitree_rl_mjlab_official_baseline")
DEFAULT_SDK_INSTALL = Path("/home/ssy/ssy_files/mjlab/.external/unitree_sdk2/install")


@dataclass(frozen=True)
class OfficialBaselinePreflightConfig:
  official_root: Path
  sdk_install: Path
  joystick_device: Path | None
  run_jstest: bool
  jstest_timeout_s: float
  expect_ready: bool
  report_out: Path | None


class OfficialBaselinePreflightError(RuntimeError):
  """Raised when the preflight cannot inspect the requested baseline."""


def _load_yaml(path: Path) -> dict[str, Any]:
  try:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
  except FileNotFoundError as exc:
    raise OfficialBaselinePreflightError(f"missing YAML file: {path}") from exc
  if not isinstance(data, dict):
    raise OfficialBaselinePreflightError(f"YAML root must be a mapping: {path}")
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

  status_name = "ok" if proc.returncode == 0 else "failed"
  return {
    "ran": True,
    "status": status_name,
    "returncode": proc.returncode,
    "stdout": proc.stdout.strip(),
    "stderr": proc.stderr.strip(),
    "timeout_s": timeout_s,
  }


def build_preflight_report(config: OfficialBaselinePreflightConfig) -> dict[str, Any]:
  root = config.official_root.expanduser().resolve()
  simulate = root / "simulate"
  deploy_g1 = root / "deploy/robots/g1"
  simulate_config_path = simulate / "config.yaml"
  simulate_config = _load_yaml(simulate_config_path)

  configured_device = Path(
    str(simulate_config.get("joystick_device", "/dev/input/js0"))
  )
  joystick_device = config.joystick_device or configured_device

  binaries = {
    "unitree_mujoco": _path_status(simulate / "build/unitree_mujoco"),
    "g1_ctrl": _path_status(deploy_g1 / "build/g1_ctrl"),
    "jstest": _path_status(simulate / "build/jstest"),
  }
  joystick = _path_status(joystick_device)
  jstest_result = (
    _run_jstest(simulate / "build/jstest", config.jstest_timeout_s)
    if config.run_jstest
    else {"ran": False, "status": "not_requested"}
  )

  blockers: list[str] = []
  if not root.is_dir():
    blockers.append("missing_official_root")
  if _git_head(root) is None:
    blockers.append("missing_official_git_head")
  if not bool(int(simulate_config.get("use_joystick", 0))):
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
  if config.run_jstest and jstest_result["status"] == "failed":
    blockers.append("jstest_failed")

  return {
    "objective": "g1_tracking_phase1_clean_official_baseline_preflight",
    "non_launching": True,
    "official_root": str(root),
    "official_head": _git_head(root),
    "sdk_install": str(config.sdk_install.expanduser().resolve()),
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
    "jstest": jstest_result,
    "deviation_policy": {
      "clean_baseline": "requires upstream source/config semantics",
      "disallowed_clean_baseline_deviations": [
        "use_joystick=0",
        "initial_qpos",
        "start_paused",
        "policy_start_gate",
        "lowstate_tick_gate",
        "action_reset_semantics_patch",
        "FixStand_semantics_patch",
      ],
      "automation_lane_label": "official_source_plus_automation_deviation",
    },
    "status": "ready" if not blockers else "blocked",
    "blockers": blockers,
  }


def write_report(report: dict[str, Any], path: Path) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv: list[str] | None = None) -> OfficialBaselinePreflightConfig:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
  parser.add_argument("--sdk-install", type=Path, default=DEFAULT_SDK_INSTALL)
  parser.add_argument("--joystick-device", type=Path)
  parser.add_argument("--run-jstest", action="store_true")
  parser.add_argument("--jstest-timeout-s", type=float, default=1.0)
  parser.add_argument("--expect-ready", action="store_true")
  parser.add_argument("--report-out", type=Path)
  args = parser.parse_args(argv)
  return OfficialBaselinePreflightConfig(
    official_root=args.official_root,
    sdk_install=args.sdk_install,
    joystick_device=args.joystick_device,
    run_jstest=args.run_jstest,
    jstest_timeout_s=args.jstest_timeout_s,
    expect_ready=args.expect_ready,
    report_out=args.report_out,
  )


def main(argv: list[str] | None = None) -> int:
  config = parse_args(argv)
  try:
    report = build_preflight_report(config)
  except OfficialBaselinePreflightError as exc:
    print(f"official baseline preflight failed: {exc}", file=sys.stderr)
    return 2

  output = json.dumps(report, indent=2, sort_keys=True)
  print(output)
  if config.report_out is not None:
    write_report(report, config.report_out)
  if config.expect_ready and report["status"] != "ready":
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
