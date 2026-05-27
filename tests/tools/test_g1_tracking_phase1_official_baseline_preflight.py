from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import yaml

from mjlab.scripts.g1_tracking_phase1_official_baseline_preflight import (
  OfficialBaselinePreflightConfig,
  build_preflight_report,
)

ROOT = Path(__file__).resolve().parents[2]
CLI = ROOT / "scripts/tools/g1_tracking_phase1_official_baseline_preflight.py"


def _write(path: Path, content: str = "") -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text(content, encoding="utf-8")


def _make_executable(path: Path, content: str = "#!/bin/sh\nexit 0\n") -> None:
  _write(path, content)
  path.chmod(path.stat().st_mode | 0o111)


def _fake_official_root(tmp_path: Path) -> Path:
  root = tmp_path / "official"
  _write(
    root / "simulate/config.yaml",
    yaml.safe_dump(
      {
        "robot": "g1",
        "robot_scene": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "interface": "lo",
        "use_joystick": 1,
        "joystick_type": "xbox",
        "joystick_device": "/dev/input/js0",
        "enable_elastic_band": 0,
      }
    ),
  )
  _make_executable(root / "simulate/build/unitree_mujoco")
  _make_executable(root / "deploy/robots/g1/build/g1_ctrl")
  _make_executable(
    root / "simulate/build/jstest",
    "#!/bin/sh\nprintf 'open failed.\\n'\nexit 1\n",
  )
  return root


def _config(root: Path, **kwargs: object) -> OfficialBaselinePreflightConfig:
  values = {
    "official_root": root,
    "sdk_install": root / "sdk",
    "joystick_device": None,
    "run_jstest": False,
    "jstest_timeout_s": 0.1,
    "expect_ready": False,
    "report_out": None,
  }
  values.update(kwargs)
  return OfficialBaselinePreflightConfig(**values)


def test_official_baseline_preflight_blocks_without_js0(tmp_path: Path) -> None:
  root = _fake_official_root(tmp_path)

  report = build_preflight_report(_config(root))

  assert report["status"] == "blocked"
  assert report["simulate_config"]["use_joystick"] == 1
  assert report["simulate_config"]["configured_joystick_device"] == "/dev/input/js0"
  assert "missing_joystick_device" in report["blockers"]
  assert report["binaries"]["unitree_mujoco"]["is_executable"] is True
  assert report["binaries"]["g1_ctrl"]["is_executable"] is True
  assert (
    "use_joystick=0"
    in report["deviation_policy"]["disallowed_clean_baseline_deviations"]
  )


def test_official_baseline_preflight_flags_device_override_as_deviation(
  tmp_path: Path,
) -> None:
  root = _fake_official_root(tmp_path)
  fake_device = tmp_path / "fake-js0"
  _write(fake_device)

  report = build_preflight_report(_config(root, joystick_device=fake_device))

  assert report["status"] == "blocked"
  assert "joystick_device_override_is_not_clean_default" in report["blockers"]
  assert "joystick_device_not_character_device" in report["blockers"]
  assert report["joystick"]["exists"] is True


def test_official_baseline_preflight_cli_writes_report_and_expect_ready_fails(
  tmp_path: Path,
) -> None:
  root = _fake_official_root(tmp_path)
  report_out = tmp_path / "report.json"

  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--official-root",
      str(root),
      "--report-out",
      str(report_out),
      "--expect-ready",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 1, proc.stderr
  stdout_report = json.loads(proc.stdout)
  file_report = json.loads(report_out.read_text(encoding="utf-8"))
  assert stdout_report["status"] == "blocked"
  assert file_report == stdout_report


def test_official_baseline_preflight_can_record_jstest_failure(
  tmp_path: Path,
) -> None:
  root = _fake_official_root(tmp_path)

  report = build_preflight_report(_config(root, run_jstest=True))

  assert report["jstest"]["ran"] is True
  assert report["jstest"]["status"] == "failed"
  assert report["jstest"]["stdout"] == "open failed."
  assert "jstest_failed" in report["blockers"]
