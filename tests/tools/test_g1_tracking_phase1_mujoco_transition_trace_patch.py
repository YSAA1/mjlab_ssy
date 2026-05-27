from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from mjlab.scripts.g1_tracking_phase1_mujoco_transition_trace_patch import (
  INCLUDE_PATCH,
  TRACE_MARKER,
  MujocoTransitionTracePatchError,
  patch_mujoco_main_cc,
)

ROOT = Path(__file__).resolve().parents[2]
PATCH_CLI = ROOT / "scripts/tools/g1_tracking_phase1_mujoco_transition_trace_patch.py"

MUJOCO_MAIN_FIXTURE = """#include <chrono>
#include <cstdint>
#include <cstdio>

#include <mujoco/mujoco.h>
#include "param.h"

class ElasticBand
{
public:
  bool enable_ = true;
  double length_ = 0.0;
  std::vector<double> f_ = {0, 0, 0};
};
inline ElasticBand elastic_band;


namespace
{
  void PhysicsLoop()
  {
            if (true)
            {
              mj_step(m, d);
              stepped = true;
            }
            else
            {
              while (true)
              {
                mj_step(m, d);
                stepped = true;
              }
            }
  }
}
"""


def test_mujoco_transition_trace_patch_injects_two_step_traces() -> None:
  patched, changed = patch_mujoco_main_cc(MUJOCO_MAIN_FIXTURE)

  assert changed is True
  assert TRACE_MARKER in patched
  assert INCLUDE_PATCH in patched
  assert "event=mujoco_transition_trace" in patched
  assert "ctrl_l2=%.6f" in patched
  assert "elastic_config=%d elastic_enabled=%d" in patched
  assert "dense_until = step + 500" in patched
  assert patched.count("phase1_mujoco_trace_after_step(m, d);") == 2


def test_mujoco_transition_trace_patch_is_idempotent() -> None:
  patched, changed = patch_mujoco_main_cc(MUJOCO_MAIN_FIXTURE)
  patched_again, changed_again = patch_mujoco_main_cc(patched)

  assert changed is True
  assert changed_again is False
  assert patched_again == patched


def test_mujoco_transition_trace_patch_rejects_missing_step_anchor() -> None:
  fixture = MUJOCO_MAIN_FIXTURE.replace("              mj_step(m, d);\n", "", 1)

  with pytest.raises(MujocoTransitionTracePatchError, match="expected 2 mj_step"):
    patch_mujoco_main_cc(fixture)


def test_mujoco_transition_trace_patch_cli_dry_run_does_not_write(
  tmp_path: Path,
) -> None:
  source = tmp_path / "main.cc"
  source.write_text(MUJOCO_MAIN_FIXTURE, encoding="utf-8")

  proc = subprocess.run(
    [sys.executable, str(PATCH_CLI), "--mujoco-main", str(source)],
    check=False,
    text=True,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
  )

  assert proc.returncode == 0, proc.stderr
  assert '"changed": true' in proc.stdout
  assert TRACE_MARKER not in source.read_text(encoding="utf-8")
