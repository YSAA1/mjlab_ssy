"""Patch local Unitree G1 Velocity policy start gate for phase-1 sim2sim."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

DEFAULT_STATE_RLBASE = Path(
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/FSM/State_RLBase.h"
)

GATE_MARKER = "phase1_velocity_policy_start_gate_v1"
GATE_ENV_VAR = "MJLAB_PHASE1_POLICY_START_GATE_SECONDS"

INCLUDE_ANCHOR = (
  '#include "isaaclab/envs/mdp/terminations.h"\n'
  "#include <cmath>\n"
  "#include <spdlog/spdlog.h>\n"
)
INCLUDE_PATCH = (
  '#include "isaaclab/envs/mdp/terminations.h"\n'
  "#include <chrono>\n"
  "#include <cmath>\n"
  "#include <cstdlib>\n"
  "#include <thread>\n"
  "#include <spdlog/spdlog.h>\n"
)

HELPER_ANCHOR = "class State_RLBase : public FSMState\n"
HELPER_PATCH = f"""namespace
{{

// {GATE_MARKER}: local-only sim2sim diagnostic gate.
double phase1_policy_start_gate_seconds()
{{
    const char* raw = std::getenv("{GATE_ENV_VAR}");
    if(raw == nullptr || raw[0] == '\\0')
    {{
        return 0.0;
    }}

    char* end = nullptr;
    const double seconds = std::strtod(raw, &end);
    if(end == raw || seconds <= 0.0)
    {{
        return 0.0;
    }}
    return seconds;
}}

}}  // namespace

class State_RLBase : public FSMState
"""

THREAD_ANCHOR = """            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();

            while (policy_thread_running)
"""
THREAD_PATCH = """            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();

            const double phase1_gate_seconds = phase1_policy_start_gate_seconds();
            if(phase1_gate_seconds > 0.0)
            {
                spdlog::info(
                    "[PHASE1] event=policy_start_gate delay_seconds={:.3f}",
                    phase1_gate_seconds
                );
                const auto gate_until = clock::now()
                    + std::chrono::duration_cast<clock::duration>(
                        std::chrono::duration<double>(phase1_gate_seconds)
                    );
                while(policy_thread_running && clock::now() < gate_until)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                }
                sleepTill = clock::now() + dt;
                spdlog::info(
                    "[PHASE1] event=policy_start_gate_release running={}",
                    policy_thread_running ? 1 : 0
                );
            }

            while (policy_thread_running)
"""


class PolicyStartGatePatchError(RuntimeError):
  """Raised when the local runtime header cannot be patched safely."""


def patch_state_rlbase_h(text: str) -> tuple[str, bool]:
  """Return patched ``State_RLBase.h`` text and whether it changed."""

  if GATE_MARKER in text:
    return text, False

  if INCLUDE_PATCH not in text:
    if INCLUDE_ANCHOR not in text:
      raise PolicyStartGatePatchError("missing include anchor")
    text = text.replace(INCLUDE_ANCHOR, INCLUDE_PATCH, 1)

  if HELPER_ANCHOR not in text:
    raise PolicyStartGatePatchError("missing helper insertion anchor")
  text = text.replace(HELPER_ANCHOR, HELPER_PATCH, 1)

  if THREAD_ANCHOR not in text:
    raise PolicyStartGatePatchError("missing policy thread anchor")
  text = text.replace(THREAD_ANCHOR, THREAD_PATCH, 1)
  return text, True


def apply_patch(path: Path, *, write: bool, backup: bool = True) -> dict[str, Any]:
  original = path.read_text(encoding="utf-8")
  patched, changed = patch_state_rlbase_h(original)
  backup_path = path.with_suffix(path.suffix + f".{GATE_MARKER}.bak")
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
    "marker": GATE_MARKER,
    "env_var": GATE_ENV_VAR,
    "backup_path": str(backup_path) if backup else None,
    "backup_written": backup_written,
  }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description=(
      "Patch local Unitree G1 State_RLBase.h so Velocity policy stepping can "
      f"be delayed by {GATE_ENV_VAR} during paused sim2sim handoff checks."
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
