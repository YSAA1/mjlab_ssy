"""Command line interface for Unitree sim2sim lane preparation."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Sequence, cast

from mjlab.sim2sim.unitree.contracts import (
  ACTION_BUNDLES,
  ActionName,
  AutomationSequence,
  DeviationOptions,
  OutputLane,
  PrepareG1Request,
  SourceCheckout,
  UnitreeSim2SimError,
)
from mjlab.sim2sim.unitree.generator import prepare_g1_lane


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    prog="unitree-sim2sim",
    description="Prepare labeled Unitree G1 sim2sim lanes from official source.",
  )
  subparsers = parser.add_subparsers(dest="command", required=True)

  prepare = subparsers.add_parser(
    "prepare-g1",
    help="prepare a G1 sim2sim lane",
    description=(
      "Prepare a labeled G1 sim2sim output lane from a caller-supplied "
      "official Unitree checkout and a selected mjlab action bundle."
    ),
  )
  prepare.add_argument(
    "--official-root",
    type=Path,
    required=True,
    help="clean official Unitree source checkout to copy from",
  )
  prepare.add_argument(
    "--out-root",
    type=Path,
    required=True,
    help="empty output root for the generated sim2sim lane",
  )
  prepare.add_argument(
    "--action",
    choices=sorted(ACTION_BUNDLES),
    required=True,
    help="G1 action bundle to install into the generated lane",
  )
  prepare.add_argument(
    "--automation-sequence",
    choices=("full", "fixstand_only"),
    default="full",
    help="synthetic input sequence for no-joystick automation",
  )
  prepare.add_argument(
    "--unpause-delay-seconds",
    type=float,
    default=8.0,
    help="controller warmup delay before automation unpauses MuJoCo",
  )
  prepare.add_argument(
    "--policy-root",
    type=Path,
    help="optional action policy root containing exported/ and params/",
  )
  prepare.add_argument(
    "--mjlab-model-xml",
    type=Path,
    help="optional mjlab G1 XML used when mode-15 model sync is enabled",
  )
  prepare.add_argument(
    "--evidence-dir",
    type=Path,
    help="optional directory where runtime evidence will be recorded",
  )
  prepare.add_argument(
    "--use-mjlab-mode15-model",
    action="store_true",
    help="sync the selected mjlab mode-15 G1 model into the generated lane",
  )
  prepare.add_argument(
    "--align-mjlab-actuator-limits",
    action="store_true",
    help="align generated scene actuator limits to mjlab training limits",
  )
  prepare.add_argument(
    "--apply-official-joint-passive-defaults",
    action="store_true",
    help="apply official passive joint defaults after mode-15 scene sync",
  )
  prepare.add_argument(
    "--diagnostic-trace",
    action="store_true",
    help="include opt-in diagnostic trace deviations in the generated lane",
  )
  prepare.add_argument(
    "--force",
    action="store_true",
    help="replace an existing output root instead of refusing it",
  )
  prepare.add_argument(
    "--dry-run",
    action="store_true",
    help="print the parsed request without generating an output lane",
  )
  prepare.set_defaults(handler=_prepare_g1)
  return parser


def request_from_args(args: argparse.Namespace) -> PrepareG1Request:
  action_name = cast(ActionName, args.action)
  automation_sequence = cast(AutomationSequence, args.automation_sequence)
  return PrepareG1Request(
    source=SourceCheckout(args.official_root),
    output=OutputLane(args.out_root, force=args.force),
    action=ACTION_BUNDLES[action_name],
    deviations=DeviationOptions(
      automation_sequence=automation_sequence,
      unpause_delay_seconds=args.unpause_delay_seconds,
      use_mjlab_mode15_model=args.use_mjlab_mode15_model,
      align_mjlab_actuator_limits=args.align_mjlab_actuator_limits,
      apply_official_joint_passive_defaults=args.apply_official_joint_passive_defaults,
      diagnostic_trace=args.diagnostic_trace,
    ),
    policy_root=args.policy_root,
    mjlab_model_xml=args.mjlab_model_xml,
    evidence_dir=args.evidence_dir,
  )


def _path_to_str(path: Path | None) -> str | None:
  if path is None:
    return None
  return str(path.expanduser().resolve())


def request_to_dict(request: PrepareG1Request) -> dict[str, Any]:
  return {
    "official_root": _path_to_str(request.source.root),
    "out_root": _path_to_str(request.output.root),
    "force": request.output.force,
    "action": request.action.name,
    "state_name": request.action.state_name,
    "policy_subdir": request.action.policy_subdir,
    "trigger": request.action.trigger,
    "automation_sequence": request.deviations.automation_sequence,
    "unpause_delay_seconds": request.deviations.unpause_delay_seconds,
    "deviation_labels": list(request.deviations.labels()),
    "policy_root": _path_to_str(request.policy_root),
    "mjlab_model_xml": _path_to_str(request.mjlab_model_xml),
    "evidence_dir": _path_to_str(request.evidence_dir),
  }


def _prepare_g1(args: argparse.Namespace) -> int:
  request = request_from_args(args)
  if args.dry_run:
    print(json.dumps({"request": request_to_dict(request)}, indent=2, sort_keys=True))
    return 0
  manifest = prepare_g1_lane(request)
  print(json.dumps(manifest, indent=2, sort_keys=True))
  return 0


def main(argv: Sequence[str] | None = None) -> int:
  parser = build_parser()
  args = parser.parse_args(argv)
  try:
    return args.handler(args)
  except UnitreeSim2SimError as exc:
    print(f"unitree-sim2sim: {exc}", file=sys.stderr)
    return 2


if __name__ == "__main__":
  raise SystemExit(main())
