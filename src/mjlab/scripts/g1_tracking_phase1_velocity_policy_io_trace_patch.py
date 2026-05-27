"""Patch local Unitree Velocity policy I/O logging for phase-1 diagnosis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DEFAULT_MANAGER_ENV = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/isaaclab/envs/manager_based_rl_env.h"
)

OLD_TRACE_MARKER = "phase1_velocity_policy_io_trace_v1"
PREVIOUS_TRACE_MARKER = "phase1_velocity_policy_io_trace_v2"
TRACE_MARKER = "phase1_velocity_policy_io_trace_v3"

INCLUDE_ANCHOR = '#include <iostream>\n#include "isaaclab/utils/utils.h"\n'
INCLUDE_PATCH = """#include <iostream>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <spdlog/spdlog.h>
#include "isaaclab/utils/utils.h"
"""

HELPER_ANCHOR = """class ObservationManager;
class ActionManager;
"""

OLD_HELPER_PATCH = f"""class ObservationManager;
class ActionManager;

// {OLD_TRACE_MARKER}: local-only policy input/output trace helpers.
inline float phase1_trace_l2(const std::vector<float>& values)
{{
    float total = 0.0f;
    for(const auto value : values)
    {{
        total += value * value;
    }}
    return std::sqrt(total);
}}

inline float phase1_trace_max_abs(const std::vector<float>& values)
{{
    float max_value = 0.0f;
    for(const auto value : values)
    {{
        max_value = std::max(max_value, std::abs(value));
    }}
    return max_value;
}}

inline std::vector<float> phase1_trace_vector(const Eigen::VectorXf& values)
{{
    std::vector<float> out;
    out.reserve(values.size());
    for(int i = 0; i < values.size(); ++i)
    {{
        out.push_back(values[i]);
    }}
    return out;
}}

inline std::vector<float> phase1_trace_vector(const Eigen::Vector3f& values)
{{
    return {{values[0], values[1], values[2]}};
}}

inline std::string phase1_trace_format(const std::vector<float>& values)
{{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << "[";
    for(size_t i = 0; i < values.size(); ++i)
    {{
        if(i > 0)
        {{
            stream << ",";
        }}
        stream << values[i];
    }}
    stream << "]";
    return stream.str();
}}

inline bool phase1_trace_should_log(long step)
{{
    return step <= 5 || step == 10 || step == 25 || step == 50;
}}
"""

PREVIOUS_HELPER_PATCH = f"""class ObservationManager;
class ActionManager;

// {PREVIOUS_TRACE_MARKER}: local-only policy input/output trace helpers.
inline float phase1_trace_l2(const std::vector<float>& values)
{{
    float total = 0.0f;
    for(const auto value : values)
    {{
        total += value * value;
    }}
    return std::sqrt(total);
}}

inline float phase1_trace_max_abs(const std::vector<float>& values)
{{
    float max_value = 0.0f;
    for(const auto value : values)
    {{
        max_value = std::max(max_value, std::abs(value));
    }}
    return max_value;
}}

inline std::vector<float> phase1_trace_vector(const Eigen::VectorXf& values)
{{
    std::vector<float> out;
    out.reserve(values.size());
    for(int i = 0; i < values.size(); ++i)
    {{
        out.push_back(values[i]);
    }}
    return out;
}}

inline std::vector<float> phase1_trace_vector(const Eigen::Vector3f& values)
{{
    return {{values[0], values[1], values[2]}};
}}

inline std::string phase1_trace_format(const std::vector<float>& values)
{{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << "[";
    for(size_t i = 0; i < values.size(); ++i)
    {{
        if(i > 0)
        {{
            stream << ",";
        }}
        stream << values[i];
    }}
    stream << "]";
    return stream.str();
}}

inline bool phase1_trace_should_log(
    long step,
    float joint_vel_l2,
    float root_ang_vel_l2,
    const std::vector<float>& gravity
)
{{
    if(step <= 5 || step == 10 || step == 25 || step == 50)
    {{
        return true;
    }}
    if(step % 25 != 0)
    {{
        return false;
    }}
    const float gravity_z = gravity.size() >= 3 ? gravity[2] : -1.0f;
    return joint_vel_l2 > 20.0f || root_ang_vel_l2 > 1.0f || std::abs(gravity_z + 1.0f) > 0.1f;
}}
"""

HELPER_PATCH = f"""class ObservationManager;
class ActionManager;

// {TRACE_MARKER}: local-only dense dynamic-onset policy input/output trace helpers.
inline float phase1_trace_l2(const std::vector<float>& values)
{{
    float total = 0.0f;
    for(const auto value : values)
    {{
        total += value * value;
    }}
    return std::sqrt(total);
}}

inline float phase1_trace_max_abs(const std::vector<float>& values)
{{
    float max_value = 0.0f;
    for(const auto value : values)
    {{
        max_value = std::max(max_value, std::abs(value));
    }}
    return max_value;
}}

inline std::vector<float> phase1_trace_vector(const Eigen::VectorXf& values)
{{
    std::vector<float> out;
    out.reserve(values.size());
    for(int i = 0; i < values.size(); ++i)
    {{
        out.push_back(values[i]);
    }}
    return out;
}}

inline std::vector<float> phase1_trace_vector(const Eigen::Vector3f& values)
{{
    return {{values[0], values[1], values[2]}};
}}

inline std::string phase1_trace_format(const std::vector<float>& values)
{{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(6) << "[";
    for(size_t i = 0; i < values.size(); ++i)
    {{
        if(i > 0)
        {{
            stream << ",";
        }}
        stream << values[i];
    }}
    stream << "]";
    return stream.str();
}}

inline bool phase1_trace_should_log(
    long step,
    float joint_vel_l2,
    float root_ang_vel_l2,
    const std::vector<float>& gravity
)
{{
    static bool dense_started = false;
    static long dense_until = -1;

    if(step == 1)
    {{
        dense_started = false;
        dense_until = -1;
    }}

    if(step <= 5 || step == 10 || step == 25 || step == 50)
    {{
        return true;
    }}

    const float gravity_z = gravity.size() >= 3 ? gravity[2] : -1.0f;
    const bool low_dynamic_onset =
        joint_vel_l2 > 1.0f ||
        root_ang_vel_l2 > 0.05f ||
        std::abs(gravity_z + 1.0f) > 0.01f;

    if(low_dynamic_onset && !dense_started)
    {{
        dense_started = true;
        dense_until = step + 75;
        return true;
    }}

    if(dense_started && step <= dense_until)
    {{
        return true;
    }}

    if(step % 25 != 0)
    {{
        return false;
    }}

    return joint_vel_l2 > 20.0f || root_ang_vel_l2 > 1.0f || std::abs(gravity_z + 1.0f) > 0.1f;
}}
"""

STEP_ANCHOR = """        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
"""

OLD_STEP_PATCH = """        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
        if(phase1_trace_should_log(episode_length))
        {
            std::vector<float> obs_flat;
            if(obs.find("obs") != obs.end())
            {
                obs_flat = obs.at("obs");
            }
            const auto processed_action = action_manager->processed_actions();
            const auto joint_pos = phase1_trace_vector(robot->data.joint_pos);
            const auto joint_vel = phase1_trace_vector(robot->data.joint_vel);
            const auto gravity = phase1_trace_vector(robot->data.projected_gravity_b);
            const auto root_ang_vel = phase1_trace_vector(robot->data.root_ang_vel_b);
            spdlog::info(
                "[PHASE1] event=policy_io_trace step={} obs_dim={} obs_l2={:.6f} obs_max={:.6f} "
                "raw_action_l2={:.6f} raw_action_max={:.6f} "
                "processed_action_l2={:.6f} processed_action_max={:.6f} "
                "joint_pos_l2={:.6f} joint_vel_l2={:.6f} phase={:.6f} "
                "obs={} raw_action={} processed_action={} joint_pos={} joint_vel={} "
                "projected_gravity={} root_ang_vel={}",
                episode_length,
                obs_flat.size(),
                phase1_trace_l2(obs_flat),
                phase1_trace_max_abs(obs_flat),
                phase1_trace_l2(action),
                phase1_trace_max_abs(action),
                phase1_trace_l2(processed_action),
                phase1_trace_max_abs(processed_action),
                phase1_trace_l2(joint_pos),
                phase1_trace_l2(joint_vel),
                global_phase,
                phase1_trace_format(obs_flat),
                phase1_trace_format(action),
                phase1_trace_format(processed_action),
                phase1_trace_format(joint_pos),
                phase1_trace_format(joint_vel),
                phase1_trace_format(gravity),
                phase1_trace_format(root_ang_vel)
            );
        }
"""

STEP_PATCH = """        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
        std::vector<float> obs_flat;
        if(obs.find("obs") != obs.end())
        {
            obs_flat = obs.at("obs");
        }
        const auto processed_action = action_manager->processed_actions();
        const auto joint_pos = phase1_trace_vector(robot->data.joint_pos);
        const auto joint_vel = phase1_trace_vector(robot->data.joint_vel);
        const auto gravity = phase1_trace_vector(robot->data.projected_gravity_b);
        const auto root_ang_vel = phase1_trace_vector(robot->data.root_ang_vel_b);
        const float joint_vel_l2 = phase1_trace_l2(joint_vel);
        const float root_ang_vel_l2 = phase1_trace_l2(root_ang_vel);
        if(phase1_trace_should_log(episode_length, joint_vel_l2, root_ang_vel_l2, gravity))
        {
            spdlog::info(
                "[PHASE1] event=policy_io_trace step={} obs_dim={} obs_l2={:.6f} obs_max={:.6f} "
                "raw_action_l2={:.6f} raw_action_max={:.6f} "
                "processed_action_l2={:.6f} processed_action_max={:.6f} "
                "joint_pos_l2={:.6f} joint_vel_l2={:.6f} phase={:.6f} "
                "obs={} raw_action={} processed_action={} joint_pos={} joint_vel={} "
                "projected_gravity={} root_ang_vel={}",
                episode_length,
                obs_flat.size(),
                phase1_trace_l2(obs_flat),
                phase1_trace_max_abs(obs_flat),
                phase1_trace_l2(action),
                phase1_trace_max_abs(action),
                phase1_trace_l2(processed_action),
                phase1_trace_max_abs(processed_action),
                phase1_trace_l2(joint_pos),
                joint_vel_l2,
                global_phase,
                phase1_trace_format(obs_flat),
                phase1_trace_format(action),
                phase1_trace_format(processed_action),
                phase1_trace_format(joint_pos),
                phase1_trace_format(joint_vel),
                phase1_trace_format(gravity),
                phase1_trace_format(root_ang_vel)
            );
        }
"""


class PolicyIoTracePatchError(RuntimeError):
  """Raised when the local Unitree runtime source cannot be patched safely."""


def patch_manager_based_rl_env_h(text: str) -> tuple[str, bool]:
  """Return patched ``manager_based_rl_env.h`` text and whether it changed."""

  if TRACE_MARKER in text:
    return text, False

  if OLD_TRACE_MARKER in text:
    if OLD_HELPER_PATCH not in text:
      raise PolicyIoTracePatchError("missing old helper patch block")
    if OLD_STEP_PATCH not in text:
      raise PolicyIoTracePatchError("missing old step patch block")
    text = text.replace(OLD_HELPER_PATCH, HELPER_PATCH, 1)
    text = text.replace(OLD_STEP_PATCH, STEP_PATCH, 1)
    return text, True

  if PREVIOUS_TRACE_MARKER in text:
    if PREVIOUS_HELPER_PATCH not in text:
      raise PolicyIoTracePatchError("missing previous helper patch block")
    text = text.replace(PREVIOUS_HELPER_PATCH, HELPER_PATCH, 1)
    return text, True

  if INCLUDE_PATCH not in text:
    if INCLUDE_ANCHOR not in text:
      raise PolicyIoTracePatchError("missing include anchor")
    text = text.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)

  if HELPER_ANCHOR not in text:
    raise PolicyIoTracePatchError("missing helper insertion anchor")
  text = text.replace(HELPER_ANCHOR, HELPER_PATCH, 1)

  if STEP_ANCHOR not in text:
    raise PolicyIoTracePatchError("missing step insertion anchor")
  text = text.replace(STEP_ANCHOR, STEP_PATCH, 1)
  return text, True


def apply_patch(path: Path, *, write: bool, backup: bool = True) -> dict[str, Any]:
  original = path.read_text(encoding="utf-8")
  patched, changed = patch_manager_based_rl_env_h(original)
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
    description="Patch local Unitree manager_based_rl_env.h with Velocity policy I/O trace logs."
  )
  parser.add_argument("--manager-env", type=Path, default=DEFAULT_MANAGER_ENV)
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
  report = apply_patch(args.manager_env, write=args.apply, backup=not args.no_backup)
  print(json.dumps(report, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
