"""Analyze phase-1 G1 tracking timing and post-action stability logs."""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

APPROVED_STABLE_STATES = {"Velocity", "FixStand"}
ACTION_STATES = {
  "Mimic_FlyingKick": "flying_kick",
  "Mimic_RoundhouseLeadingRight": "roundhouse_leading_right",
}
REQUIRED_OFFSETS = (
  "trigger_to_fsm_s",
  "fsm_to_motion_s",
  "motion_to_policy_s",
  "policy_to_lowcmd_s",
  "lowcmd_to_q_response_s",
)
INSTRUMENTATION_TARGETS = [
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_Mimic.cpp",
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/robots/g1/src/State_RLBase.cpp",
  "/home/ssy/ssy_files/mjlab/.external/unitree_rl_mjlab/deploy/include/FSM/State_FixStand.h",
]

TS_RE = re.compile(r"^\[(?P<ts>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d+)\]")
FSM_START_RE = re.compile(r"FSM: Start (?P<state>\S+)")
FSM_CHANGE_RE = re.compile(r"FSM: Change state from (?P<from>\S+) to (?P<to>\S+)")
DIAG_RE = re.compile(r"\[GETUP-DIAG\]\s+(?P<body>.*)$")
PHASE1_RE = re.compile(r"\[PHASE1\]\s+(?P<body>.*)$")
KV_RE = re.compile(r"(?P<key>[A-Za-z0-9_]+)=(?P<value>\([^)]*\)|\"[^\"]*\"|\S+)")


@dataclass(frozen=True)
class TimedLine:
  timestamp_s: float
  line_no: int
  line: str


def _timestamp_s(line: str) -> float | None:
  match = TS_RE.search(line)
  if not match:
    return None
  dt = datetime.strptime(match.group("ts"), "%Y-%m-%d %H:%M:%S.%f")
  return dt.timestamp()


def _parse_kv(body: str) -> dict[str, str]:
  return {
    match.group("key"): match.group("value").strip('"')
    for match in KV_RE.finditer(body)
  }


def _float(value: str | None) -> float | None:
  if value is None:
    return None
  try:
    return float(value)
  except ValueError:
    return None


def _vector3(value: str | None) -> tuple[float, float, float] | None:
  if value is None:
    return None
  parts = value.strip("()").split(",")
  if len(parts) != 3:
    return None
  parsed = [_float(part) for part in parts]
  if any(part is None for part in parsed):
    return None
  return parsed[0], parsed[1], parsed[2]


def _action_name(value: str | None) -> str | None:
  if value is None:
    return None
  if value in ACTION_STATES:
    return ACTION_STATES[value]
  if value.startswith("Mimic_"):
    return value.removeprefix("Mimic_").lower()
  return value


def _load_lines(log_path: Path) -> list[TimedLine]:
  lines: list[TimedLine] = []
  for idx, line in enumerate(
    log_path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1
  ):
    ts = _timestamp_s(line)
    if ts is None:
      continue
    lines.append(TimedLine(timestamp_s=ts, line_no=idx, line=line))
  return lines


def _collect_events(lines: list[TimedLine]) -> dict[str, list[dict[str, Any]]]:
  events: dict[str, list[dict[str, Any]]] = {
    "fsm": [],
    "phase1": [],
    "diag": [],
  }
  for item in lines:
    if match := FSM_START_RE.search(item.line):
      state = match.group("state")
      events["fsm"].append(
        {
          "timestamp_s": item.timestamp_s,
          "line_no": item.line_no,
          "event": "fsm_start",
          "from": None,
          "to": state,
        }
      )
    if match := FSM_CHANGE_RE.search(item.line):
      events["fsm"].append(
        {
          "timestamp_s": item.timestamp_s,
          "line_no": item.line_no,
          "event": "fsm_transition",
          "from": match.group("from"),
          "to": match.group("to"),
        }
      )
    if match := DIAG_RE.search(item.line):
      fields = _parse_kv(match.group("body"))
      fields.update(
        {
          "timestamp_s": item.timestamp_s,
          "line_no": item.line_no,
          "event": "q_response",
          "source": "GETUP-DIAG",
        }
      )
      events["diag"].append(fields)
    if match := PHASE1_RE.search(item.line):
      fields = _parse_kv(match.group("body"))
      fields.update(
        {
          "timestamp_s": item.timestamp_s,
          "line_no": item.line_no,
          "source": "PHASE1",
        }
      )
      fields["event"] = fields.get("event", "unknown")
      if "action" in fields:
        fields["action"] = _action_name(fields["action"])
      if "state" in fields:
        fields["state"] = fields["state"]
      events["phase1"].append(fields)
  events["fsm"].sort(key=lambda event: event["timestamp_s"])
  events["phase1"].sort(key=lambda event: event["timestamp_s"])
  events["diag"].sort(key=lambda event: event["timestamp_s"])
  return events


def _build_episodes(events: dict[str, list[dict[str, Any]]]) -> list[dict[str, Any]]:
  fsm = events["fsm"]
  episodes: list[dict[str, Any]] = []
  for idx, event in enumerate(fsm):
    state = event["to"]
    if state not in ACTION_STATES:
      continue
    end_event = None
    for later in fsm[idx + 1 :]:
      if later["from"] == state:
        end_event = later
        break
    next_action_event = None
    for later in fsm[idx + 1 :]:
      if later["to"] in ACTION_STATES:
        next_action_event = later
        break
    action = ACTION_STATES[state]
    episodes.append(
      {
        "action": action,
        "state": state,
        "start_time_s": event["timestamp_s"],
        "start_line": event["line_no"],
        "end_time_s": end_event["timestamp_s"] if end_event else None,
        "end_line": end_event["line_no"] if end_event else None,
        "return_state": end_event["to"] if end_event else None,
        "next_action_start_time_s": next_action_event["timestamp_s"]
        if next_action_event
        else None,
        "next_action_start_line": next_action_event["line_no"]
        if next_action_event
        else None,
      }
    )
  return episodes


def _events_for_episode(
  events: list[dict[str, Any]],
  episode: dict[str, Any],
  *,
  name: str,
) -> list[dict[str, Any]]:
  start = episode["start_time_s"]
  end = episode["end_time_s"]
  action = episode["action"]
  filtered = []
  for event in events:
    ts = event["timestamp_s"]
    if name == "trigger":
      if ts <= start and (event.get("action") in {None, action}):
        filtered.append(event)
      continue
    if end is None:
      in_window = ts >= start
    else:
      in_window = start <= ts <= end
    if in_window and event.get("action", action) == action:
      filtered.append(event)
  return filtered


def _first_after(
  events: list[dict[str, Any]],
  timestamp_s: float,
  event_name: str,
) -> dict[str, Any] | None:
  for event in events:
    if event.get("event") == event_name and event["timestamp_s"] >= timestamp_s:
      return event
  return None


def _latest_before_or_at(
  events: list[dict[str, Any]],
  timestamp_s: float,
  event_name: str,
) -> dict[str, Any] | None:
  candidates = [
    event
    for event in events
    if event.get("event") == event_name and event["timestamp_s"] <= timestamp_s
  ]
  return candidates[-1] if candidates else None


def _stability_report(
  phase1_events: list[dict[str, Any]], episode: dict[str, Any]
) -> dict[str, Any]:
  end = episode["end_time_s"]
  state = episode["return_state"]
  if end is None or state is None:
    return {
      "passed": False,
      "reason": "missing_action_end",
      "duration_s": 0.0,
      "samples": 0,
    }
  samples = [
    event
    for event in phase1_events
    if event.get("event") == "stable_sample"
    and event.get("state") == state
    and event["timestamp_s"] >= end
    and (
      episode["next_action_start_time_s"] is None
      or event["timestamp_s"] < episode["next_action_start_time_s"]
    )
  ]
  if not samples:
    return {
      "passed": False,
      "reason": "missing_stable_samples",
      "duration_s": 0.0,
      "samples": 0,
    }
  unstable = [
    event
    for event in samples
    if event.get("stable", "1").lower() in {"0", "false", "no"}
  ]
  last_sample = samples[-1]
  duration = last_sample["timestamp_s"] - end
  if unstable:
    return {
      "passed": False,
      "reason": "unstable_sample",
      "duration_s": round(duration, 3),
      "samples": len(samples),
      "first_unstable_line": unstable[0]["line_no"],
    }
  if duration < 5.0:
    return {
      "passed": False,
      "reason": "stability_window_too_short",
      "duration_s": round(duration, 3),
      "samples": len(samples),
    }
  return {
    "passed": True,
    "reason": "stable_for_5s",
    "duration_s": round(duration, 3),
    "samples": len(samples),
  }


def _action_health_report(
  action_events: list[dict[str, Any]], diagnostics: list[dict[str, Any]]
) -> dict[str, Any]:
  responses = [
    event
    for event in [*action_events, *diagnostics]
    if event.get("event") == "q_response"
  ]
  if not responses:
    return {
      "passed": True,
      "reason": "no_action_health_samples",
      "samples": 0,
    }

  max_q_err_l2 = 0.0
  max_dq_err_l2 = 0.0
  first_bad_gravity = None
  last_gravity = None
  for event in responses:
    q_err_l2 = _float(event.get("q_err_l2")) or 0.0
    dq_err_l2 = _float(event.get("dq_err_l2")) or 0.0
    max_q_err_l2 = max(max_q_err_l2, q_err_l2)
    max_dq_err_l2 = max(max_dq_err_l2, dq_err_l2)
    gravity = _vector3(event.get("gravity_b"))
    if gravity is None:
      continue
    last_gravity = gravity
    if first_bad_gravity is None and gravity[2] > -0.5:
      first_bad_gravity = {
        "line_no": event["line_no"],
        "gravity_b": [round(axis, 3) for axis in gravity],
      }

  report: dict[str, Any] = {
    "passed": first_bad_gravity is None,
    "reason": "action_health_ok"
    if first_bad_gravity is None
    else "bad_gravity_during_action",
    "samples": len(responses),
    "max_q_err_l2": round(max_q_err_l2, 3),
    "max_dq_err_l2": round(max_dq_err_l2, 3),
  }
  if last_gravity is not None:
    report["last_gravity_b"] = [round(axis, 3) for axis in last_gravity]
  if first_bad_gravity is not None:
    report["first_bad_gravity"] = first_bad_gravity
  return report


def _episode_report(
  episode: dict[str, Any], events: dict[str, list[dict[str, Any]]]
) -> dict[str, Any]:
  phase1 = events["phase1"]
  action_events = _events_for_episode(phase1, episode, name="action")
  triggers = _events_for_episode(phase1, episode, name="trigger")
  diagnostics = _events_for_episode(events["diag"], episode, name="action")

  trigger = _latest_before_or_at(triggers, episode["start_time_s"], "trigger")
  motion = _first_after(action_events, episode["start_time_s"], "motion_frame")
  policy = _first_after(
    action_events,
    motion["timestamp_s"] if motion else episode["start_time_s"],
    "policy_step",
  )
  lowcmd = _first_after(
    action_events,
    policy["timestamp_s"] if policy else episode["start_time_s"],
    "lowcmd_write",
  )
  q_response = _first_after(
    action_events,
    lowcmd["timestamp_s"] if lowcmd else episode["start_time_s"],
    "q_response",
  )
  if q_response is None and lowcmd is not None:
    q_response = _first_after(diagnostics, lowcmd["timestamp_s"], "q_response")

  offsets: dict[str, float | None] = {
    "trigger_to_fsm_s": round(episode["start_time_s"] - trigger["timestamp_s"], 3)
    if trigger
    else None,
    "fsm_to_motion_s": round(motion["timestamp_s"] - episode["start_time_s"], 3)
    if motion
    else None,
    "motion_to_policy_s": round(policy["timestamp_s"] - motion["timestamp_s"], 3)
    if policy and motion
    else None,
    "policy_to_lowcmd_s": round(lowcmd["timestamp_s"] - policy["timestamp_s"], 3)
    if lowcmd and policy
    else None,
    "lowcmd_to_q_response_s": round(
      q_response["timestamp_s"] - lowcmd["timestamp_s"], 3
    )
    if q_response and lowcmd
    else None,
  }
  missing = [name for name in REQUIRED_OFFSETS if offsets[name] is None]
  if episode["end_time_s"] is None:
    missing.append("action_end_fsm_transition")
  action_health = _action_health_report(action_events, diagnostics)
  stable = _stability_report(phase1, episode)
  returned_to_approved = episode["return_state"] in APPROVED_STABLE_STATES

  primary_reason = None
  passed = True
  if episode["end_time_s"] is None:
    primary_reason = "incomplete_action"
    passed = False
  elif not returned_to_approved:
    primary_reason = "post_motion_handoff_failure"
    passed = False
  elif missing:
    primary_reason = "insufficient_timing_evidence"
    passed = False
  elif not action_health["passed"]:
    primary_reason = "policy_action_to_joint_response_mismatch"
    passed = False
  elif not stable["passed"]:
    primary_reason = (
      "insufficient_stability_evidence"
      if stable["reason"] in {"missing_stable_samples", "stability_window_too_short"}
      else "post_motion_handoff_failure"
    )
    passed = False

  return {
    **episode,
    "passed": passed,
    "primary_reason": primary_reason,
    "returned_to_approved_state": returned_to_approved,
    "offsets_s": offsets,
    "missing_evidence": missing,
    "action_health": action_health,
    "stable_window": stable,
    "diagnostic_samples": len(diagnostics),
  }


def analyze_log(log_path: Path) -> dict[str, Any]:
  lines = _load_lines(log_path)
  events = _collect_events(lines)
  episodes = _build_episodes(events)
  episode_reports = [_episode_report(episode, events) for episode in episodes]
  missing_required = sorted(
    {missing for episode in episode_reports for missing in episode["missing_evidence"]}
  )
  passed = bool(episode_reports) and all(
    episode["passed"] for episode in episode_reports
  )
  if not episode_reports:
    primary_reason = "insufficient_timing_evidence"
  elif passed:
    primary_reason = None
  elif any(
    ep["primary_reason"] == "policy_action_to_joint_response_mismatch"
    for ep in episode_reports
  ):
    primary_reason = "policy_action_to_joint_response_mismatch"
  elif any(
    ep["primary_reason"] == "post_motion_handoff_failure" for ep in episode_reports
  ):
    primary_reason = "post_motion_handoff_failure"
  elif any(ep["primary_reason"] == "incomplete_action" for ep in episode_reports):
    primary_reason = "incomplete_action"
  elif any(
    ep["primary_reason"] == "insufficient_stability_evidence" for ep in episode_reports
  ):
    primary_reason = "insufficient_stability_evidence"
  else:
    primary_reason = "insufficient_timing_evidence"
  return {
    "schema_version": 1,
    "log_path": str(log_path.resolve()),
    "passed": passed,
    "primary_reason": primary_reason,
    "episodes": episode_reports,
    "event_counts": {key: len(value) for key, value in events.items()},
    "missing_required_evidence": missing_required,
    "instrumentation_needed": INSTRUMENTATION_TARGETS
    if primary_reason and primary_reason.startswith("insufficient")
    else [],
  }


def _find_log(input_dir: Path) -> Path:
  candidate = input_dir / "g1_ctrl.log"
  if candidate.is_file():
    return candidate
  matches = sorted(input_dir.glob("**/g1_ctrl.log"))
  if matches:
    return matches[0]
  raise FileNotFoundError(f"No g1_ctrl.log found under {input_dir}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Analyze G1 phase-1 timing/stability logs."
  )
  group = parser.add_mutually_exclusive_group(required=True)
  group.add_argument("--fixtures", type=Path)
  group.add_argument("--evidence-dir", type=Path)
  group.add_argument("--log", type=Path)
  parser.add_argument("--report-out", type=Path)
  parser.add_argument("--expect-failure", action="store_true")
  return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
  args = parse_args(argv)
  try:
    if args.log is not None:
      log_path = args.log
    else:
      log_path = _find_log(args.fixtures or args.evidence_dir)
    report = analyze_log(log_path)
    output = json.dumps(report, indent=2, sort_keys=True)
    report_out = args.report_out
    if report_out is None and args.evidence_dir is not None:
      report_out = args.evidence_dir / "phase1_log_analysis.json"
    if report_out is not None:
      report_out.parent.mkdir(parents=True, exist_ok=True)
      report_out.write_text(output + "\n", encoding="utf-8")
    print(output)
    if args.expect_failure:
      return 0 if not report["passed"] else 2
    return 0 if report["passed"] else 2
  except OSError as exc:
    print(f"Log analysis failed: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
