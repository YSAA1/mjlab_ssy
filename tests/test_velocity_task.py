"""Tests specific to velocity tasks."""

import pytest

from mjlab.asset_zoo.robots import G1_ACTION_SCALE, GO1_ACTION_SCALE
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.tasks.registry import list_tasks, load_env_cfg
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg

DEPLOY98_ACTOR_DIMS = {
  "base_ang_vel": 3,
  "projected_gravity": 3,
  "command": 3,
  "phase": 2,
  "joint_pos": 29,
  "joint_vel": 29,
  "actions": 29,
}


@pytest.fixture(scope="module")
def velocity_task_ids() -> list[str]:
  """Get all velocity task IDs."""
  return [t for t in list_tasks() if "Velocity" in t]


@pytest.fixture(scope="module")
def g1_velocity_task_ids(velocity_task_ids: list[str]) -> list[str]:
  """Get all G1 velocity task IDs."""
  return [t for t in velocity_task_ids if "G1" in t]


@pytest.fixture(scope="module")
def go1_velocity_task_ids(velocity_task_ids: list[str]) -> list[str]:
  """Get all Go1 velocity task IDs."""
  return [t for t in velocity_task_ids if "Go1" in t]


@pytest.fixture(scope="module")
def rough_velocity_task_ids(velocity_task_ids: list[str]) -> list[str]:
  """Get all rough terrain velocity task IDs."""
  return [t for t in velocity_task_ids if "Rough" in t]


@pytest.fixture(scope="module")
def flat_velocity_task_ids(velocity_task_ids: list[str]) -> list[str]:
  """Get all flat terrain velocity task IDs."""
  return [t for t in velocity_task_ids if "Flat" in t]


def test_velocity_tasks_have_twist_command(velocity_task_ids: list[str]) -> None:
  """All velocity tasks should have a velocity command."""
  for task_id in velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert "twist" in cfg.commands, f"Task {task_id} missing 'twist' command"

    twist_cmd = cfg.commands["twist"]
    assert isinstance(twist_cmd, UniformVelocityCommandCfg), (
      f"Task {task_id} twist command is not UniformVelocityCommandCfg"
    )


def test_g1_velocity_has_required_sensors(g1_velocity_task_ids: list[str]) -> None:
  """G1 velocity tasks should have feet/ground and self collision sensors."""
  for task_id in g1_velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert cfg.scene.sensors is not None, f"Task {task_id} has no sensors"

    sensor_names = {s.name for s in cfg.scene.sensors}
    assert "feet_ground_contact" in sensor_names, (
      f"Task {task_id} missing feet_ground_contact sensor"
    )
    assert "self_collision" in sensor_names, (
      f"Task {task_id} missing self_collision sensor"
    )


def test_go1_velocity_has_required_sensors(go1_velocity_task_ids: list[str]) -> None:
  """Go1 velocity tasks should have feet/ground and collision sensors."""
  for task_id in go1_velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert cfg.scene.sensors is not None, f"Task {task_id} has no sensors"

    sensor_names = {s.name for s in cfg.scene.sensors}
    assert "feet_ground_contact" in sensor_names, (
      f"Task {task_id} missing feet_ground_contact sensor"
    )
    if "Rough" in task_id:
      for name in (
        "self_collision",
        "thigh_ground_touch",
        "shank_ground_touch",
        "trunk_ground_touch",
      ):
        assert name in sensor_names, f"Task {task_id} missing {name} sensor"


def test_flat_velocity_tasks_have_plane_terrain(
  flat_velocity_task_ids: list[str],
) -> None:
  """Flat velocity tasks should have terrain_type='plane' and no terrain_generator."""
  for task_id in flat_velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert cfg.scene.terrain is not None, f"Task {task_id} has no terrain config"
    assert cfg.scene.terrain.terrain_type == "plane", (
      f"Task {task_id} terrain_type={cfg.scene.terrain.terrain_type}, expected 'plane'"
    )
    assert cfg.scene.terrain.terrain_generator is None, (
      f"Task {task_id} has terrain_generator, expected None for flat terrain"
    )


def test_rough_velocity_tasks_have_generator_terrain(
  rough_velocity_task_ids: list[str],
) -> None:
  """Rough velocity tasks should have generator terrain."""
  for task_id in rough_velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert cfg.scene.terrain is not None, f"Task {task_id} has no terrain config"
    assert cfg.scene.terrain.terrain_type == "generator", (
      f"Task {task_id} terrain_type={cfg.scene.terrain.terrain_type}, "
      "expected 'generator'"
    )
    assert cfg.scene.terrain.terrain_generator is not None, (
      f"Task {task_id} has no terrain_generator, expected one for rough terrain"
    )


def test_rough_velocity_training_has_curriculum_enabled() -> None:
  """Rough velocity training tasks should have terrain curriculum enabled."""
  rough_training_tasks = [
    "Mjlab-Velocity-Rough-Unitree-G1",
    "Mjlab-Velocity-Rough-Unitree-Go1",
  ]

  for task_id in rough_training_tasks:
    cfg = load_env_cfg(task_id)

    assert cfg.scene.terrain is not None, f"Task {task_id} has no terrain config"
    assert cfg.scene.terrain.terrain_generator is not None, (
      f"Task {task_id} has no terrain_generator"
    )
    assert cfg.scene.terrain.terrain_generator.curriculum is True, (
      f"Task {task_id} curriculum={cfg.scene.terrain.terrain_generator.curriculum}, "
      "expected True"
    )


def test_rough_velocity_play_has_curriculum_disabled() -> None:
  """Rough velocity play tasks should have terrain curriculum disabled."""
  rough_training_tasks = [
    "Mjlab-Velocity-Rough-Unitree-G1",
    "Mjlab-Velocity-Rough-Unitree-Go1",
  ]

  for task_id in rough_training_tasks:
    cfg = load_env_cfg(task_id, play=True)

    assert cfg.scene.terrain is not None, (
      f"Task {task_id} (play mode) has no terrain config"
    )
    assert cfg.scene.terrain.terrain_generator is not None, (
      f"Task {task_id} (play mode) has no terrain_generator"
    )
    assert cfg.scene.terrain.terrain_generator.curriculum is False, (
      f"Task {task_id} (play mode) curriculum={cfg.scene.terrain.terrain_generator.curriculum}, "
      "expected False"
    )


def test_g1_velocity_has_correct_action_scale(g1_velocity_task_ids: list[str]) -> None:
  """G1 velocity tasks should use G1_ACTION_SCALE."""
  for task_id in g1_velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert "joint_pos" in cfg.actions, f"Task {task_id} missing 'joint_pos' action"

    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg), (
      f"Task {task_id} joint_pos action is not JointPositionActionCfg"
    )

    assert joint_pos_action.scale == G1_ACTION_SCALE, (
      f"Task {task_id} action scale mismatch, expected G1_ACTION_SCALE"
    )


def test_g1_deploy98_velocity_actor_matches_unitree_runtime_contract() -> None:
  cfg = load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1-Deploy98")

  actor_terms = cfg.observations["actor"].terms
  assert list(actor_terms) == list(DEPLOY98_ACTOR_DIMS)
  assert sum(DEPLOY98_ACTOR_DIMS.values()) == 98
  assert "base_lin_vel" not in actor_terms
  assert "height_scan" not in actor_terms

  phase_cfg = actor_terms["phase"]
  assert phase_cfg.params == {"period": 0.6, "command_name": "twist"}


def test_g1_deploy98_stand_first_zero_command_training_gate() -> None:
  cfg = load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst")

  actor_terms = cfg.observations["actor"].terms
  assert list(actor_terms) == list(DEPLOY98_ACTOR_DIMS)
  assert sum(DEPLOY98_ACTOR_DIMS.values()) == 98

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  assert twist_cmd.rel_standing_envs == 1.0
  assert twist_cmd.rel_heading_envs == 0.0
  assert twist_cmd.rel_forward_envs == 0.0
  assert twist_cmd.rel_world_envs == 0.0
  assert twist_cmd.init_velocity_prob == 0.0
  assert twist_cmd.heading_command is False
  assert twist_cmd.ranges.lin_vel_x == (0.0, 0.0)
  assert twist_cmd.ranges.lin_vel_y == (0.0, 0.0)
  assert twist_cmd.ranges.ang_vel_z == (0.0, 0.0)
  assert twist_cmd.ranges.heading is None

  assert "push_robot" not in cfg.events
  assert "command_vel" not in cfg.curriculum
  assert cfg.episode_length_s == 5.0
  assert "alive" in cfg.rewards
  assert "termination_penalty" in cfg.rewards


def test_g1_deploy98_stand_first_keeps_base_deploy98_unchanged() -> None:
  cfg = load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1-Deploy98")

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  assert twist_cmd.rel_standing_envs == 0.1
  assert twist_cmd.heading_command is True
  assert twist_cmd.ranges.lin_vel_x == (-1.0, 1.0)
  assert "push_robot" in cfg.events
  assert "command_vel" in cfg.curriculum
  assert cfg.episode_length_s == 20.0
  assert "alive" not in cfg.rewards
  assert "termination_penalty" not in cfg.rewards


def test_g1_deploy98_stand_first_damped_targets_sensitivity_findings() -> None:
  cfg = load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst-Damped")

  actor_terms = cfg.observations["actor"].terms
  assert list(actor_terms) == list(DEPLOY98_ACTOR_DIMS)
  assert sum(DEPLOY98_ACTOR_DIMS.values()) == 98

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  assert twist_cmd.rel_standing_envs == 1.0
  assert twist_cmd.ranges.lin_vel_x == (0.0, 0.0)
  assert twist_cmd.ranges.lin_vel_y == (0.0, 0.0)
  assert twist_cmd.ranges.ang_vel_z == (0.0, 0.0)

  assert cfg.rewards["pose"].weight == 2.0
  assert cfg.rewards["body_ang_vel"].weight == -0.2
  assert cfg.rewards["action_rate_l2"].weight == -0.5
  assert cfg.rewards["action_acc_l2"].weight == -0.25
  assert cfg.rewards["joint_vel_l2"].weight == -0.02
  assert "push_robot" not in cfg.events
  assert "command_vel" not in cfg.curriculum
  assert cfg.episode_length_s == 5.0


def test_g1_deploy98_stand_first_damped_keeps_stand_first_unchanged() -> None:
  cfg = load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1-Deploy98-StandFirst")

  assert cfg.rewards["pose"].weight == 1.0
  assert cfg.rewards["body_ang_vel"].weight == -0.05
  assert cfg.rewards["action_rate_l2"].weight == -0.1
  assert "action_acc_l2" not in cfg.rewards
  assert "joint_vel_l2" not in cfg.rewards


def test_g1_flat_velocity_keeps_current_99_dim_actor_contract() -> None:
  cfg = load_env_cfg("Mjlab-Velocity-Flat-Unitree-G1")

  actor_terms = cfg.observations["actor"].terms
  assert list(actor_terms) == [
    "base_lin_vel",
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "actions",
    "command",
  ]
  assert "phase" not in actor_terms


def test_go1_velocity_has_correct_action_scale(
  go1_velocity_task_ids: list[str],
) -> None:
  """Go1 velocity tasks should use GO1_ACTION_SCALE."""
  for task_id in go1_velocity_task_ids:
    cfg = load_env_cfg(task_id)

    assert "joint_pos" in cfg.actions, f"Task {task_id} missing 'joint_pos' action"

    joint_pos_action = cfg.actions["joint_pos"]
    assert isinstance(joint_pos_action, JointPositionActionCfg), (
      f"Task {task_id} joint_pos action is not JointPositionActionCfg"
    )

    assert joint_pos_action.scale == GO1_ACTION_SCALE, (
      f"Task {task_id} action scale mismatch, expected GO1_ACTION_SCALE"
    )
