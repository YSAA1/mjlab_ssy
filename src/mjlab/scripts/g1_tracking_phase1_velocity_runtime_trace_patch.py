"""Patch local Unitree G1 Velocity runtime logging for phase-1 diagnosis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DEFAULT_STATE_RLBASE = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp"
)

TRACE_MARKER = "phase1_velocity_runtime_trace_v1"

INCLUDE_ANCHOR = "#include <unordered_map>\n"
INCLUDE_PATCH = "#include <algorithm>\n#include <unordered_map>\n"

HELPER_ANCHOR = "}\n\nState_RLBase::State_RLBase"
HELPER_PATCH = f"""}}
namespace
{{

// {TRACE_MARKER}: helpers for local-only deploy diagnostics.
float phase1_l2(const std::vector<float>& values)
{{
    float total = 0.0f;
    for(const auto value : values)
    {{
        total += value * value;
    }}
    return std::sqrt(total);
}}

float phase1_max_abs(const std::vector<float>& values)
{{
    float max_value = 0.0f;
    for(const auto value : values)
    {{
        max_value = std::max(max_value, std::abs(value));
    }}
    return max_value;
}}

float phase1_l2(const Eigen::VectorXf& values)
{{
    return values.norm();
}}

float phase1_max_abs(const Eigen::VectorXf& values)
{{
    float max_value = 0.0f;
    for(int i = 0; i < values.size(); ++i)
    {{
        max_value = std::max(max_value, std::abs(values[i]));
    }}
    return max_value;
}}

}}  // namespace

State_RLBase::State_RLBase"""

LOG_ANCHOR = """        spdlog::info(
            "[PHASE1] event=stable_sample state={} stable={} q_err_l2={:.3f} "
            "q_err_max={:.3f} base_vel_x=0.000 command_vel_x=0.000 "
            "gravity_b=({:.3f},{:.3f},{:.3f}) root_ang_vel_l2={:.3f}",
            getStateString(),
            stable ? 1 : 0,
            std::sqrt(q_err_l2),
            q_err_max,
            gravity[0],
            gravity[1],
            gravity[2],
            root_ang_vel_l2
        );"""

LOG_PATCH = """        const auto command = isaaclab::mdp::velocity_commands(env.get(), YAML::Node());
        const auto raw_action = env->action_manager->action();
        const Eigen::VectorXf joint_pos_rel = q_real - env->robot->data.default_joint_pos;
        const auto joint_vel = env->robot->data.joint_vel;
        const float command_vel_x = command.size() > 0 ? command[0] : 0.0f;
        const float command_vel_y = command.size() > 1 ? command[1] : 0.0f;
        const float command_yaw = command.size() > 2 ? command[2] : 0.0f;
        const float command_norm = phase1_l2(command);
        const float raw_action_l2 = phase1_l2(raw_action);
        const float raw_action_max = phase1_max_abs(raw_action);
        const float processed_action_l2 = phase1_l2(action);
        const float processed_action_max = phase1_max_abs(action);
        const float joint_pos_rel_l2 = phase1_l2(joint_pos_rel);
        const float joint_pos_rel_max = phase1_max_abs(joint_pos_rel);
        const float joint_vel_l2 = phase1_l2(joint_vel);
        const float joint_vel_max = phase1_max_abs(joint_vel);
        spdlog::info(
            "[PHASE1] event=stable_sample state={} stable={} policy_step={} q_err_l2={:.3f} "
            "q_err_max={:.3f} base_vel_x=0.000 command_vel_x={:.3f} "
            "command_vel_y={:.3f} command_yaw={:.3f} command_norm={:.3f} "
            "phase={:.3f} raw_action_l2={:.3f} raw_action_max={:.3f} "
            "processed_action_l2={:.3f} processed_action_max={:.3f} "
            "joint_pos_rel_l2={:.3f} joint_pos_rel_max={:.3f} "
            "joint_vel_l2={:.3f} joint_vel_max={:.3f} "
            "gravity_b=({:.3f},{:.3f},{:.3f}) root_ang_vel_l2={:.3f}",
            getStateString(),
            stable ? 1 : 0,
            env->episode_length,
            std::sqrt(q_err_l2),
            q_err_max,
            command_vel_x,
            command_vel_y,
            command_yaw,
            command_norm,
            env->global_phase,
            raw_action_l2,
            raw_action_max,
            processed_action_l2,
            processed_action_max,
            joint_pos_rel_l2,
            joint_pos_rel_max,
            joint_vel_l2,
            joint_vel_max,
            gravity[0],
            gravity[1],
            gravity[2],
            root_ang_vel_l2
        );"""


class RuntimeTracePatchError(RuntimeError):
  """Raised when the local runtime source cannot be patched safely."""


def patch_state_rlbase_cpp(text: str) -> tuple[str, bool]:
  """Return patched ``State_RLBase.cpp`` text and whether it changed."""

  if TRACE_MARKER in text:
    return text, False

  if INCLUDE_PATCH not in text:
    if INCLUDE_ANCHOR not in text:
      raise RuntimeTracePatchError("missing include anchor")
    text = text.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)

  if HELPER_ANCHOR not in text:
    raise RuntimeTracePatchError("missing helper insertion anchor")
  text = text.replace(HELPER_ANCHOR, HELPER_PATCH, 1)

  if LOG_ANCHOR not in text:
    raise RuntimeTracePatchError("missing stable_sample log anchor")
  text = text.replace(LOG_ANCHOR, LOG_PATCH, 1)
  return text, True


def apply_patch(path: Path, *, write: bool, backup: bool = True) -> dict[str, Any]:
  original = path.read_text(encoding="utf-8")
  patched, changed = patch_state_rlbase_cpp(original)
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
    description="Patch local Unitree G1 State_RLBase.cpp with Velocity runtime trace logs."
  )
  parser.add_argument("--state-rlbase", type=Path, default=DEFAULT_STATE_RLBASE)
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
  report = apply_patch(args.state_rlbase, write=args.apply, backup=not args.no_backup)
  print(json.dumps(report, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
