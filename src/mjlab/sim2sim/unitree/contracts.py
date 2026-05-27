"""Typed contracts for Unitree sim2sim lane preparation."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

ActionName = Literal["flying_kick", "roundhouse_leading_right"]
AutomationSequence = Literal["full", "fixstand_only"]
ClaimLabel = Literal[
  "clean_official",
  "not_clean_official_baseline",
  "official_source_plus_deviation",
  "official_source_plus_automation_deviation",
  "diagnostic_trace",
]


class UnitreeSim2SimError(RuntimeError):
  """Raised when a Unitree sim2sim lane request is invalid."""


@dataclass(frozen=True)
class ActionBundle:
  """Metadata for one productized G1 action bundle."""

  name: ActionName
  state_name: str
  policy_subdir: str
  trigger: str
  policy_filename: str
  motion_filename: str
  deploy_yaml_filename: str = "deploy.yaml"

  def required_policy_files(self, policy_root: Path) -> tuple[Path, ...]:
    """Return required files below a caller-supplied policy root."""
    return (
      policy_root / "exported" / self.policy_filename,
      policy_root / "params" / self.deploy_yaml_filename,
      policy_root / "params" / self.motion_filename,
    )


ACTION_BUNDLES: dict[ActionName, ActionBundle] = {
  "flying_kick": ActionBundle(
    name="flying_kick",
    state_name="Mimic_FlyingKick",
    policy_subdir="flying_kick",
    trigger="RB + X",
    policy_filename="policy.onnx",
    motion_filename="flying_kick.npz",
  ),
  "roundhouse_leading_right": ActionBundle(
    name="roundhouse_leading_right",
    state_name="Mimic_RoundhouseLeadingRight",
    policy_subdir="roundhouse_leading_right",
    trigger="RB + Y",
    policy_filename="policy.onnx",
    motion_filename="roundhouse_leading_right.npz",
  ),
}


REQUIRED_SOURCE_PATHS = (
  Path("simulate/config.yaml"),
  Path("simulate/src/main.cc"),
  Path("simulate/src/param.h"),
  Path("simulate/src/physics_joystick.h"),
  Path("simulate/src/unitree_sdk2_bridge.h"),
  Path("deploy/robots/g1/config/config.yaml"),
  Path("deploy/robots/g1/CMakeLists.txt"),
  Path("src/assets/robots/unitree_g1/xmls/scene_g1.xml"),
)


@dataclass(frozen=True)
class SourceCheckout:
  """A caller-supplied clean official Unitree source checkout."""

  root: Path

  @property
  def resolved_root(self) -> Path:
    return self.root.expanduser().resolve()

  def missing_required_paths(self) -> tuple[Path, ...]:
    return missing_required_source_paths(self.resolved_root)


@dataclass(frozen=True)
class OutputLane:
  """Destination for a generated sim2sim lane."""

  root: Path
  force: bool = False

  @property
  def resolved_root(self) -> Path:
    return self.root.expanduser().resolve()


@dataclass(frozen=True)
class DeviationOptions:
  """Declared deviations from a clean official Unitree checkout."""

  automation_sequence: AutomationSequence = "full"
  unpause_delay_seconds: float = 8.0
  use_mjlab_mode15_model: bool = False
  align_mjlab_actuator_limits: bool = False
  apply_official_joint_passive_defaults: bool = False
  diagnostic_trace: bool = False

  def labels(self) -> tuple[str, ...]:
    labels = ["automation_input"]
    if self.use_mjlab_mode15_model:
      labels.append("mjlab_mode15_model")
    if self.align_mjlab_actuator_limits:
      labels.append("mjlab_actuator_limit_alignment")
    if self.apply_official_joint_passive_defaults:
      labels.append("official_joint_passive_defaults")
    if self.diagnostic_trace:
      labels.append("diagnostic_trace")
    return tuple(labels)


@dataclass(frozen=True)
class PrepareG1Request:
  """Parsed request for `unitree-sim2sim prepare-g1`."""

  source: SourceCheckout
  output: OutputLane
  action: ActionBundle
  deviations: DeviationOptions = field(default_factory=DeviationOptions)
  policy_root: Path | None = None
  mjlab_model_xml: Path | None = None
  evidence_dir: Path | None = None


@dataclass(frozen=True)
class ManifestRecord:
  """Machine-readable provenance fields every generated lane must emit."""

  schema_version: int
  claim: ClaimLabel
  source_root: Path
  source_sha: str | None
  output_root: Path
  action: ActionName
  automation_sequence: AutomationSequence
  deviation_labels: tuple[str, ...]
  changed_paths: tuple[str, ...]
  evidence_dir: Path | None


def missing_required_source_paths(root: Path) -> tuple[Path, ...]:
  """Return required official-source paths missing from `root`."""
  resolved_root = root.expanduser().resolve()
  return tuple(
    path for path in REQUIRED_SOURCE_PATHS if not (resolved_root / path).exists()
  )


def validate_source_checkout(source: SourceCheckout) -> None:
  """Fail if the source checkout does not look like Unitree sim2sim source."""
  missing = source.missing_required_paths()
  if missing:
    missing_display = ", ".join(str(path) for path in missing)
    raise UnitreeSim2SimError(
      f"official source checkout is missing required paths: {missing_display}"
    )
