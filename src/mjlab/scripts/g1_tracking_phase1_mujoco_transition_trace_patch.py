"""Patch local Unitree MuJoCo physics transition logging for phase-1 diagnosis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DEFAULT_MUJOCO_MAIN = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/simulate/src/main.cc"
)

TRACE_MARKER = "phase1_mujoco_transition_trace_v1"

INCLUDE_ANCHOR = "#include <chrono>\n#include <cstdint>\n"
INCLUDE_PATCH = "#include <chrono>\n#include <cmath>\n#include <cstdint>\n"

HELPER_ANCHOR = "inline ElasticBand elastic_band;\n\n\nnamespace\n{"
HELPER_PATCH = f"""inline ElasticBand elastic_band;

// {TRACE_MARKER}: local-only MuJoCo physics transition trace helpers.
inline double phase1_sim_array_l2(const mjtNum* values, int start, int count)
{{
  double total = 0.0;
  for (int i = 0; i < count; ++i)
  {{
    const double value = static_cast<double>(values[start + i]);
    total += value * value;
  }}
  return std::sqrt(total);
}}

inline double phase1_sim_array_max_abs(const mjtNum* values, int start, int count)
{{
  double max_value = 0.0;
  for (int i = 0; i < count; ++i)
  {{
    const double value = std::abs(static_cast<double>(values[start + i]));
    if (value > max_value)
    {{
      max_value = value;
    }}
  }}
  return max_value;
}}

inline double phase1_sim_value(const mjtNum* values, int count, int index)
{{
  if (index < 0 || index >= count)
  {{
    return 0.0;
  }}
  return static_cast<double>(values[index]);
}}

inline void phase1_mujoco_trace_after_step(const mjModel* model, const mjData* data)
{{
  if (!model || !data)
  {{
    return;
  }}

  static long step = 0;
  static double last_time = -1.0;
  static bool dense_started = false;
  static long dense_until = -1;

  if (data->time < last_time)
  {{
    step = 0;
    dense_started = false;
    dense_until = -1;
  }}

  last_time = data->time;
  ++step;

  const double qvel_l2 = phase1_sim_array_l2(data->qvel, 0, model->nv);
  const double qvel_max = phase1_sim_array_max_abs(data->qvel, 0, model->nv);
  const int root_lin_count = model->nv >= 3 ? 3 : model->nv;
  const int root_ang_count = model->nv >= 6 ? 3 : (model->nv > 3 ? model->nv - 3 : 0);
  const double root_lin_vel_l2 = phase1_sim_array_l2(data->qvel, 0, root_lin_count);
  const double root_ang_vel_l2 = phase1_sim_array_l2(data->qvel, 3, root_ang_count);
  const double ctrl_l2 = phase1_sim_array_l2(data->ctrl, 0, model->nu);
  const double ctrl_max = phase1_sim_array_max_abs(data->ctrl, 0, model->nu);
  const double elastic_force_l2 = std::sqrt(
      elastic_band.f_[0] * elastic_band.f_[0] +
      elastic_band.f_[1] * elastic_band.f_[1] +
      elastic_band.f_[2] * elastic_band.f_[2]);

  bool should_log = step <= 5 || step == 10 || step == 25 || step == 50;
  const bool dynamic_onset = qvel_l2 > 1.0 || root_ang_vel_l2 > 0.05;
  if (dynamic_onset && !dense_started)
  {{
    dense_started = true;
    dense_until = step + 500;
    should_log = true;
  }}
  if (dense_started && step <= dense_until)
  {{
    should_log = true;
  }}
  if (step % 500 == 0)
  {{
    should_log = true;
  }}
  if (!should_log)
  {{
    return;
  }}

  std::printf(
      "[PHASE1_SIM] event=mujoco_transition_trace step=%ld sim_time=%.6f "
      "root_pos=(%.6f,%.6f,%.6f) root_lin_vel=(%.6f,%.6f,%.6f) "
      "root_ang_vel=(%.6f,%.6f,%.6f) root_lin_vel_l2=%.6f "
      "root_ang_vel_l2=%.6f qvel_l2=%.6f qvel_max=%.6f ctrl_l2=%.6f "
      "ctrl_max=%.6f ncon=%d elastic_config=%d elastic_enabled=%d "
      "elastic_length=%.6f elastic_force=(%.6f,%.6f,%.6f) "
      "elastic_force_l2=%.6f\\n",
      step,
      static_cast<double>(data->time),
      phase1_sim_value(data->qpos, model->nq, 0),
      phase1_sim_value(data->qpos, model->nq, 1),
      phase1_sim_value(data->qpos, model->nq, 2),
      phase1_sim_value(data->qvel, model->nv, 0),
      phase1_sim_value(data->qvel, model->nv, 1),
      phase1_sim_value(data->qvel, model->nv, 2),
      phase1_sim_value(data->qvel, model->nv, 3),
      phase1_sim_value(data->qvel, model->nv, 4),
      phase1_sim_value(data->qvel, model->nv, 5),
      root_lin_vel_l2,
      root_ang_vel_l2,
      qvel_l2,
      qvel_max,
      ctrl_l2,
      ctrl_max,
      data->ncon,
      param::config.enable_elastic_band,
      elastic_band.enable_ ? 1 : 0,
      elastic_band.length_,
      elastic_band.f_[0],
      elastic_band.f_[1],
      elastic_band.f_[2],
      elastic_force_l2);
  std::fflush(stdout);
}}


namespace
{{"""

FIRST_STEP_ANCHOR = """              mj_step(m, d);
              stepped = true;
"""
FIRST_STEP_PATCH = """              mj_step(m, d);
              phase1_mujoco_trace_after_step(m, d);
              stepped = true;
"""
LOOP_STEP_ANCHOR = """                mj_step(m, d);
                stepped = true;
"""
LOOP_STEP_PATCH = """                mj_step(m, d);
                phase1_mujoco_trace_after_step(m, d);
                stepped = true;
"""


class MujocoTransitionTracePatchError(RuntimeError):
  """Raised when the local Unitree MuJoCo source cannot be patched safely."""


def patch_mujoco_main_cc(text: str) -> tuple[str, bool]:
  """Return patched ``simulate/src/main.cc`` text and whether it changed."""

  if TRACE_MARKER in text:
    return text, False

  if INCLUDE_PATCH not in text:
    if INCLUDE_ANCHOR not in text:
      raise MujocoTransitionTracePatchError("missing include anchor")
    text = text.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)

  if HELPER_ANCHOR not in text:
    raise MujocoTransitionTracePatchError("missing helper insertion anchor")
  text = text.replace(HELPER_ANCHOR, HELPER_PATCH, 1)

  step_count = text.count(FIRST_STEP_ANCHOR) + text.count(LOOP_STEP_ANCHOR)
  if step_count != 2:
    raise MujocoTransitionTracePatchError(
      f"expected 2 mj_step anchors, found {step_count}"
    )
  text = text.replace(FIRST_STEP_ANCHOR, FIRST_STEP_PATCH, 1)
  text = text.replace(LOOP_STEP_ANCHOR, LOOP_STEP_PATCH, 1)
  return text, True


def apply_patch(path: Path, *, write: bool, backup: bool = True) -> dict[str, Any]:
  original = path.read_text(encoding="utf-8")
  patched, changed = patch_mujoco_main_cc(original)
  backup_path = path.with_suffix(path.suffix + f".{TRACE_MARKER}.bak")
  backup_written = False
  if write and changed:
    if backup and not backup_path.exists():
      backup_path.write_text(original, encoding="utf-8")
      backup_written = True
    path.write_text(patched, encoding="utf-8")
  return {
    "path": str(path),
    "changed": changed,
    "write": write,
    "marker": TRACE_MARKER,
    "backup_path": str(backup_path) if backup else None,
    "backup_written": backup_written,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Patch local Unitree MuJoCo main.cc with phase-1 transition trace logs."
  )
  parser.add_argument("--mujoco-main", type=Path, default=DEFAULT_MUJOCO_MAIN)
  parser.add_argument(
    "--apply",
    action="store_true",
    help="Write the patch. Without this flag, only reports whether a patch is needed.",
  )
  parser.add_argument(
    "--no-backup",
    action="store_true",
    help="Do not create a sibling .bak file before writing the local runtime patch.",
  )
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  report = apply_patch(args.mujoco_main, write=args.apply, backup=not args.no_backup)
  print(json.dumps(report, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
