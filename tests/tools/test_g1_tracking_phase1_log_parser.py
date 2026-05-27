from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from mjlab.scripts.g1_tracking_phase1_analyze_logs import analyze_log

ROOT = Path(__file__).resolve().parents[2]
PASS_FIXTURE = ROOT / "tests/fixtures/g1_phase1/pass/g1_ctrl.log"
FAIL_FIXTURE = ROOT / "tests/fixtures/g1_phase1/fail/g1_ctrl.log"
ACTION_MISMATCH_FIXTURE = ROOT / "tests/fixtures/g1_phase1/action_mismatch/g1_ctrl.log"
CLI = ROOT / "scripts/tools/g1_tracking_phase1_analyze_logs.py"


def test_pass_fixture_reports_offsets_and_5s_stability() -> None:
  report = analyze_log(PASS_FIXTURE)

  assert report["passed"] is True
  assert report["primary_reason"] is None
  assert len(report["episodes"]) == 2
  for episode in report["episodes"]:
    assert episode["returned_to_approved_state"] is True
    assert episode["stable_window"]["passed"] is True
    assert episode["stable_window"]["duration_s"] >= 5.0
    assert all(value is not None for value in episode["offsets_s"].values())
  assert report["episodes"][0]["stable_window"]["duration_s"] == 5.18
  assert report["episodes"][0]["stable_window"]["samples"] == 3


def test_fail_fixture_blocks_on_missing_timing_and_stability_evidence() -> None:
  report = analyze_log(FAIL_FIXTURE)

  assert report["passed"] is False
  assert report["primary_reason"] == "insufficient_timing_evidence"
  assert report["instrumentation_needed"]
  assert any(
    "lowcmd_to_q_response_s" in episode["missing_evidence"]
    for episode in report["episodes"]
  )
  assert any(
    episode["stable_window"]["reason"] == "missing_stable_samples"
    for episode in report["episodes"]
  )


def test_action_instability_takes_precedence_over_later_stable_window() -> None:
  report = analyze_log(ACTION_MISMATCH_FIXTURE)

  assert report["passed"] is False
  assert report["primary_reason"] == "policy_action_to_joint_response_mismatch"
  episode = report["episodes"][0]
  assert episode["missing_evidence"] == []
  assert episode["stable_window"]["passed"] is True
  assert episode["action_health"]["passed"] is False
  assert episode["action_health"]["reason"] == "bad_gravity_during_action"


def test_cli_expect_failure_returns_success_for_failing_fixture() -> None:
  proc = subprocess.run(
    [
      sys.executable,
      str(CLI),
      "--fixtures",
      str(FAIL_FIXTURE.parent),
      "--expect-failure",
    ],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert json.loads(proc.stdout)["passed"] is False


def test_cli_writes_evidence_report(tmp_path: Path) -> None:
  evidence = tmp_path / "evidence"
  evidence.mkdir()
  (evidence / "g1_ctrl.log").write_text(PASS_FIXTURE.read_text(encoding="utf-8"))

  proc = subprocess.run(
    [sys.executable, str(CLI), "--evidence-dir", str(evidence)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  report_path = evidence / "phase1_log_analysis.json"
  assert report_path.is_file()
  assert json.loads(report_path.read_text(encoding="utf-8"))["passed"] is True
