"""Patch local Unitree G1 Velocity policy loop to wait for fresh lowstate ticks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DEFAULT_STATE_RLBASE = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/FSM/State_RLBase.h"
)

TICK_GATE_MARKER = "phase1_velocity_lowstate_tick_gate_v1"
START_GATE_MARKER = "phase1_velocity_policy_start_gate_v1"
TICK_GATE_ENV_VAR = "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE"
TICK_GATE_TIMEOUT_ENV_VAR = "MJLAB_PHASE1_POLICY_LOWSTATE_TICK_GATE_TIMEOUT_SECONDS"

MUTEX_INCLUDE_ANCHOR = "#include <cstdlib>\n#include <thread>\n"
MUTEX_INCLUDE_PATCH = "#include <cstdlib>\n#include <mutex>\n#include <thread>\n"

HELPER_ANCHOR = "\n}  // namespace\n\nclass State_RLBase : public FSMState\n"
HELPER_PATCH = f"""// {TICK_GATE_MARKER}: local-only sim2sim lowstate freshness gate.
bool phase1_lowstate_tick_gate_enabled()
{{
    const char* raw = std::getenv("{TICK_GATE_ENV_VAR}");
    return raw != nullptr && raw[0] == '1' && raw[1] == '\\0';
}}

double phase1_lowstate_tick_gate_timeout_seconds()
{{
    const char* raw = std::getenv("{TICK_GATE_TIMEOUT_ENV_VAR}");
    if(raw == nullptr || raw[0] == '\\0')
    {{
        return 1.0;
    }}

    char* end = nullptr;
    const double seconds = std::strtod(raw, &end);
    if(end == raw || seconds <= 0.0)
    {{
        return 1.0;
    }}
    return seconds;
}}

}}  // namespace

class State_RLBase : public FSMState
"""

LOOP_ANCHOR = """            while (policy_thread_running)
            {
                env->step();
"""
OLD_TICK_GATED_STEP = """                if(!phase1_wait_for_lowstate_tick())
                {
                    sleepTill = clock::now() + dt;
                    continue;
                }
                env->step();
"""
TICK_GATED_STEP = """                if(!phase1_wait_for_lowstate_tick())
                {
                    sleepTill = clock::now() + dt;
                    continue;
                }
                // Waiting for a lowstate tick can consume longer than dt;
                // anchor the next policy period to the fresh state sample.
                sleepTill = clock::now() + dt;
                env->step();
"""
LOOP_PATCH = (
  """            const bool phase1_tick_gate_enabled = phase1_lowstate_tick_gate_enabled();
            const double phase1_tick_gate_timeout_s =
                phase1_lowstate_tick_gate_timeout_seconds();
            const std::chrono::duration<double> phase1_tick_gate_timeout_duration(
                phase1_tick_gate_timeout_s
            );
            auto phase1_read_lowstate_tick = [this]() -> long long
            {
                std::lock_guard<std::mutex> lock(lowstate->mutex_);
                return static_cast<long long>(lowstate->msg_.tick());
            };
            long long phase1_last_lowstate_tick = phase1_read_lowstate_tick();
            int phase1_tick_gate_log_count = 0;
            auto phase1_wait_for_lowstate_tick = [&]() -> bool
            {
                if(!phase1_tick_gate_enabled)
                {
                    return true;
                }
                const auto deadline = clock::now()
                    + std::chrono::duration_cast<clock::duration>(
                        phase1_tick_gate_timeout_duration
                    );
                while(policy_thread_running)
                {
                    const long long tick = phase1_read_lowstate_tick();
                    if(tick != phase1_last_lowstate_tick)
                    {
                        phase1_last_lowstate_tick = tick;
                        if(phase1_tick_gate_log_count < 8)
                        {
                            spdlog::info(
                                "[PHASE1] event=lowstate_tick_gate_release tick={} policy_step={}",
                                tick,
                                env->episode_length
                            );
                            phase1_tick_gate_log_count += 1;
                        }
                        return true;
                    }
                    if(clock::now() >= deadline)
                    {
                        spdlog::warn(
                            "[PHASE1] event=lowstate_tick_gate_timeout last_tick={} policy_step={}",
                            phase1_last_lowstate_tick,
                            env->episode_length
                        );
                        return false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                return false;
            };
            if(phase1_tick_gate_enabled)
            {
                spdlog::info(
                    "[PHASE1] event=lowstate_tick_gate_start tick={} timeout_seconds={:.3f}",
                    phase1_last_lowstate_tick,
                    phase1_tick_gate_timeout_s
                );
            }

            while (policy_thread_running)
            {
"""
  + TICK_GATED_STEP
)


class LowstateTickGatePatchError(RuntimeError):
  """Raised when the local runtime header cannot be patched safely."""


def patch_state_rlbase_h(text: str) -> tuple[str, bool]:
  """Return patched ``State_RLBase.h`` text and whether it changed."""

  if TICK_GATE_MARKER in text:
    patched = text.replace(OLD_TICK_GATED_STEP, TICK_GATED_STEP, 1)
    return patched, patched != text

  if START_GATE_MARKER not in text:
    raise LowstateTickGatePatchError(f"missing prerequisite marker {START_GATE_MARKER}")

  if MUTEX_INCLUDE_PATCH not in text:
    if MUTEX_INCLUDE_ANCHOR not in text:
      raise LowstateTickGatePatchError("missing mutex include anchor")
    text = text.replace(MUTEX_INCLUDE_ANCHOR, MUTEX_INCLUDE_PATCH, 1)

  if HELPER_ANCHOR not in text:
    raise LowstateTickGatePatchError("missing helper insertion anchor")
  text = text.replace(HELPER_ANCHOR, "\n" + HELPER_PATCH, 1)

  if LOOP_ANCHOR not in text:
    raise LowstateTickGatePatchError("missing policy loop anchor")
  text = text.replace(LOOP_ANCHOR, LOOP_PATCH, 1)
  return text, True


def apply_patch(path: Path, *, write: bool, backup: bool = True) -> dict[str, Any]:
  original = path.read_text(encoding="utf-8")
  patched, changed = patch_state_rlbase_h(original)
  backup_path = path.with_suffix(path.suffix + f".{TICK_GATE_MARKER}.bak")
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
    "marker": TICK_GATE_MARKER,
    "prerequisite_marker": START_GATE_MARKER,
    "env_var": TICK_GATE_ENV_VAR,
    "timeout_env_var": TICK_GATE_TIMEOUT_ENV_VAR,
    "backup_path": str(backup_path) if backup else None,
    "backup_written": backup_written,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description=(
      "Patch local Unitree G1 State_RLBase.h so Velocity policy stepping can "
      f"wait for fresh lowstate tick updates when {TICK_GATE_ENV_VAR}=1."
    )
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
