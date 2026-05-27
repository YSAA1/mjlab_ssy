from __future__ import annotations

import json
from pathlib import Path
from typing import Literal

import pytest

from mjlab.sim2sim.unitree import (
  ACTION_BUNDLES,
  MANIFEST_NAME,
  DeviationOptions,
  OutputLane,
  PrepareG1Request,
  SourceCheckout,
  UnitreeSim2SimError,
  prepare_g1_lane,
)


def _write(path: Path, content: str | bytes = "content\n") -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  if isinstance(content, bytes):
    path.write_bytes(content)
  else:
    path.write_text(content, encoding="utf-8")


def _official_source(tmp_path: Path) -> Path:
  root = tmp_path / "official"
  _write(
    root / "simulate/config.yaml",
    """robot: "g1"
robot_scene: "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
domain_id: 0
interface: "lo"
use_joystick: 1
joystick_type: "xbox"
joystick_device: "/dev/input/js0"
joystick_bits: 16
print_scene_information: 1
enable_elastic_band: 0
""",
  )
  _write(
    root / "simulate/src/physics_joystick.h",
    """#pragma once
#include <iostream>
""",
  )
  _write(
    root / "simulate/src/unitree_sdk2_bridge.h",
    """#pragma once
#include <iostream>

        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        if(!mj_data_) return;
""",
  )
  _write(root / "simulate/src/param.h", "#pragma once\n")
  _write(
    root / "simulate/src/main.cc",
    """#include <chrono>
#include <thread>

  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);
""",
  )
  _write(root / "simulate/build/unitree_mujoco", "runtime binary\n")
  _write(root / "deploy/robots/g1/build/g1_ctrl", "runtime binary\n")
  _write(root / "deploy/robots/g1/CMakeLists.txt", "add_executable(g1_ctrl main.cpp)\n")
  _write(
    root / "deploy/robots/g1/config/config.yaml",
    """FSM:
  _:
    Passive:
      id: 1
    Velocity:
      id: 3
      type: RLBase
  Passive:
    transitions:
      FixStand: LT + up.on_pressed
  Velocity:
    transitions:
      Passive: LT + B.on_pressed
""",
  )
  _write(
    root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
    """<mujoco model="scene_g1">
  <compiler angle="radian" meshdir="assets"/>
  <default><joint damping="0.05"/></default>
  <asset><mesh name="pelvis" file="pelvis.STL"/></asset>
  <worldbody><body name="pelvis"><site name="imu"/></body></worldbody>
  <actuator><motor name="joint0" joint="joint0" ctrlrange="-1 1"/></actuator>
  <sensor>
    <framequat name="imu_quat" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <framepos name="frame_pos" objname="imu"/>
    <framelinvel name="frame_vel" objname="imu"/>
  </sensor>
  <statistic/>
  <visual/>
</mujoco>
""",
  )
  return root


ActionName = Literal["flying_kick", "roundhouse_leading_right"]


def _policy_root(tmp_path: Path, action: ActionName = "flying_kick") -> Path:
  root = tmp_path / f"{action}_policy"
  bundle = ACTION_BUNDLES[action]
  _write(root / "exported" / bundle.policy_filename, b"policy")
  _write(root / "params" / bundle.deploy_yaml_filename, "default_joint_pos: []\n")
  _write(root / "params" / bundle.motion_filename, b"motion")
  return root


def _mode15_xml(tmp_path: Path) -> Path:
  xml = tmp_path / "mjlab_model/g1.xml"
  _write(
    xml,
    """<mujoco model="g1">
  <compiler angle="radian"/>
  <default><default class="g1"><joint damping="0.1"/></default></default>
  <asset><mesh name="pelvis" file="pelvis_5010.STL"/></asset>
  <worldbody><body name="pelvis"><site name="imu_in_pelvis"/></body></worldbody>
  <contact/>
</mujoco>
""",
  )
  _write(xml.parent / "assets/pelvis_5010.STL", b"stl")
  return xml


def _request(
  tmp_path: Path,
  *,
  out_name: str = "lane",
  diagnostic_trace: bool = False,
  use_mjlab_mode15_model: bool = False,
) -> PrepareG1Request:
  official = _official_source(tmp_path)
  policy = _policy_root(tmp_path)
  return PrepareG1Request(
    source=SourceCheckout(official),
    output=OutputLane(tmp_path / out_name),
    action=ACTION_BUNDLES["flying_kick"],
    deviations=DeviationOptions(
      diagnostic_trace=diagnostic_trace,
      use_mjlab_mode15_model=use_mjlab_mode15_model,
      apply_official_joint_passive_defaults=use_mjlab_mode15_model,
    ),
    policy_root=policy,
    mjlab_model_xml=_mode15_xml(tmp_path) if use_mjlab_mode15_model else None,
  )


def test_prepare_g1_lane_generates_manifest_and_declared_files(tmp_path: Path) -> None:
  request = _request(tmp_path)
  source_config_before = (request.source.root / "simulate/config.yaml").read_text(
    encoding="utf-8"
  )

  manifest = prepare_g1_lane(request)
  out_root = request.output.root

  assert (out_root / MANIFEST_NAME).is_file()
  written = json.loads((out_root / MANIFEST_NAME).read_text(encoding="utf-8"))
  assert written == manifest
  assert manifest["lane"] == "official_source_plus_automation_deviation"
  assert manifest["source_sha"]
  assert manifest["action"]["name"] == "flying_kick"
  assert "automation_input" in manifest["deviation_labels"]
  assert "policy_asset" in {row["label"] for row in manifest["allowed_deviations"]}
  assert not (out_root / "simulate/build").exists()
  assert not (out_root / "deploy/robots/g1/build").exists()

  for relative_path in manifest["changed_paths"]:
    assert (out_root / relative_path).exists(), relative_path

  assert "use_joystick: 0" in (out_root / "simulate/config.yaml").read_text(
    encoding="utf-8"
  )
  assert "AutoSequenceJoystick" in (
    out_root / "simulate/src/physics_joystick.h"
  ).read_text(encoding="utf-8")
  joystick_text = (out_root / "simulate/src/physics_joystick.h").read_text(
    encoding="utf-8"
  )
  assert "const bool rt_window" in joystick_text
  assert "const bool rt_a_window" in joystick_text
  assert "RT(rt_window ? 1.0f : 0.0f)" in joystick_text
  assert "Mimic_FlyingKick" in (
    out_root / "deploy/robots/g1/config/config.yaml"
  ).read_text(encoding="utf-8")
  assert "SIM_TELEMETRY" not in (
    out_root / "simulate/src/unitree_sdk2_bridge.h"
  ).read_text(encoding="utf-8")
  assert (request.source.root / "simulate/config.yaml").read_text(
    encoding="utf-8"
  ) == source_config_before
  assert not (request.source.root / MANIFEST_NAME).exists()


def test_prepare_g1_lane_refuses_existing_output_without_force(tmp_path: Path) -> None:
  request = _request(tmp_path)
  request.output.root.mkdir()

  with pytest.raises(UnitreeSim2SimError, match="output root already exists"):
    prepare_g1_lane(request)


def test_prepare_g1_lane_force_replaces_existing_output(tmp_path: Path) -> None:
  request = _request(tmp_path)
  request.output.root.mkdir()
  _write(request.output.root / "stale.txt", "stale\n")
  forced = PrepareG1Request(
    source=request.source,
    output=OutputLane(request.output.root, force=True),
    action=request.action,
    deviations=request.deviations,
    policy_root=request.policy_root,
  )

  prepare_g1_lane(forced)

  assert not (request.output.root / "stale.txt").exists()
  assert (request.output.root / MANIFEST_NAME).is_file()


def test_prepare_g1_lane_refuses_output_inside_source(tmp_path: Path) -> None:
  request = _request(tmp_path)
  inside_source = PrepareG1Request(
    source=request.source,
    output=OutputLane(request.source.root / "generated-lane"),
    action=request.action,
    deviations=request.deviations,
    policy_root=request.policy_root,
  )

  with pytest.raises(UnitreeSim2SimError, match="official source root"):
    prepare_g1_lane(inside_source)


def test_diagnostic_trace_is_opt_in(tmp_path: Path) -> None:
  default_request = _request(tmp_path, out_name="default-lane")
  trace_request = _request(tmp_path, out_name="trace-lane", diagnostic_trace=True)

  prepare_g1_lane(default_request)
  trace_manifest = prepare_g1_lane(trace_request)

  assert "SIM_TELEMETRY" not in (
    default_request.output.root / "simulate/src/unitree_sdk2_bridge.h"
  ).read_text(encoding="utf-8")
  assert "SIM_TELEMETRY" in (
    trace_request.output.root / "simulate/src/unitree_sdk2_bridge.h"
  ).read_text(encoding="utf-8")
  assert "diagnostic_trace" in trace_manifest["deviation_labels"]


def test_mode15_model_sync_is_labeled_and_copies_assets(tmp_path: Path) -> None:
  request = _request(tmp_path, use_mjlab_mode15_model=True)

  manifest = prepare_g1_lane(request)

  assert "mjlab_mode15_model" in manifest["deviation_labels"]
  assert "official_joint_passive_defaults" in manifest["deviation_labels"]
  assert (
    request.output.root / "src/assets/robots/unitree_g1/xmls/assets/pelvis_5010.STL"
  ).is_file()
  scene = (
    request.output.root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
  ).read_text(encoding="utf-8")
  assert "imu_in_pelvis" in scene
  assert 'frictionloss="0.2"' in scene


def test_missing_policy_root_fails_before_copy(tmp_path: Path) -> None:
  request = _request(tmp_path)
  missing_policy = PrepareG1Request(
    source=request.source,
    output=request.output,
    action=request.action,
    deviations=request.deviations,
    policy_root=None,
  )

  with pytest.raises(UnitreeSim2SimError, match="--policy-root is required"):
    prepare_g1_lane(missing_policy)
