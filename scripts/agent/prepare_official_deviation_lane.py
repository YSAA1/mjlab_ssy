#!/usr/bin/env python3
"""Prepare an official-source automation-deviation copy without touching .external."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_OFFICIAL_ROOT = Path("/tmp/unitree_rl_mjlab_official_baseline")
DEFAULT_OUT_ROOT = Path("/tmp/g1_official_automation_deviation")
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_FLYING_KICK_POLICY_ROOT = (
  REPO_ROOT
  / ".external/unitree_rl_mjlab/deploy/robots/g1/config/policy/mimic/flying_kick"
)
DEFAULT_MJLAB_MODE15_XML = (
  REPO_ROOT / "src/mjlab/asset_zoo/robots/unitree_g1/xmls/g1.xml"
)


def _git_head(root: Path) -> str:
  proc = subprocess.run(
    ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
    check=True,
    text=True,
    stdout=subprocess.PIPE,
  )
  return proc.stdout.strip()


def _file_record(path: Path, *, root: Path | None = None) -> dict[str, object]:
  digest = hashlib.sha256()
  with path.open("rb") as f:
    for chunk in iter(lambda: f.read(1024 * 1024), b""):
      digest.update(chunk)
  display_path = path
  if root is not None:
    try:
      display_path = path.relative_to(root)
    except ValueError:
      display_path = path
  return {
    "path": str(display_path),
    "size_bytes": path.stat().st_size,
    "sha256": digest.hexdigest(),
  }


def _replace_use_joystick(config_path: Path) -> None:
  lines = config_path.read_text(encoding="utf-8").splitlines()
  replaced = False
  output: list[str] = []
  for line in lines:
    stripped = line.strip()
    if stripped.startswith("use_joystick:"):
      output.append("use_joystick: 0 # DEVIATION: disable joystick for automation lane")
      replaced = True
    else:
      output.append(line)
  if not replaced:
    raise RuntimeError(f"did not find use_joystick in {config_path}")
  config_path.write_text("\n".join(output) + "\n", encoding="utf-8")


def _drop_stale_simulator_build(out_root: Path) -> None:
  shutil.rmtree(out_root / "simulate/build", ignore_errors=True)


def _inject_auto_sequence_joystick(
  physics_joystick_path: Path, *, include_flying_kick: bool, automation_sequence: str
) -> None:
  text = physics_joystick_path.read_text(encoding="utf-8")
  if "#include <chrono>" not in text:
    text = text.replace(
      "#include <iostream>\n", "#include <iostream>\n#include <chrono>\n"
    )
  if "class AutoSequenceJoystick" in text:
    physics_joystick_path.write_text(text, encoding="utf-8")
    return

  mimic_button = "X" if include_flying_kick else "A"
  if automation_sequence == "full":
    rt_window = "t >= 7.0 && t < 10.0"
    rt_a_window = (
      "(t >= 7.4 && t < 7.7) ||\n"
      "            (t >= 8.4 && t < 8.7) ||\n"
      "            (t >= 9.4 && t < 9.7)"
    )
    rb_window = "t >= 11.0 && t < 14.0"
    rb_a_window = (
      "(t >= 11.4 && t < 11.7) ||\n"
      "            (t >= 12.4 && t < 12.7) ||\n"
      "            (t >= 13.4 && t < 13.7)"
    )
  elif automation_sequence == "fixstand_only":
    rt_window = "false"
    rt_a_window = "false"
    rb_window = "false"
    rb_a_window = "false"
  else:
    raise RuntimeError(f"unsupported automation sequence: {automation_sequence}")
  auto_sequence = """


class AutoSequenceJoystick : public unitree::common::UnitreeJoystick
{
public:
    AutoSequenceJoystick()
    : unitree::common::UnitreeJoystick(),
      start_(std::chrono::steady_clock::now())
    {}

    void update() override
    {
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - start_).count();

        const bool lt_window = t >= 0.2 && t < 6.8;
        const bool up_window =
            (t >= 0.8 && t < 1.1) ||
            (t >= 1.8 && t < 2.1) ||
            (t >= 2.8 && t < 3.1) ||
            (t >= 3.8 && t < 4.1) ||
            (t >= 4.8 && t < 5.1) ||
            (t >= 5.8 && t < 6.1);
        const bool rt_window = __RT_WINDOW__;
        const bool rt_a_window =
            __RT_A_WINDOW__;
        const bool rb_window = __RB_WINDOW__;
        const bool rb_a_window =
            __RB_A_WINDOW__;

        back(0);
        start(0);
        LS(0);
        RS(0);
        LB(0);
        RB(rb_window ? 1 : 0);
        A((rt_a_window || (rb_a_window && std::string("__MIMIC_BUTTON__") == "A")) ? 1 : 0);
        B(0);
        X((rb_a_window && std::string("__MIMIC_BUTTON__") == "X") ? 1 : 0);
        Y(0);
        up(up_window ? 1 : 0);
        down(0);
        left(0);
        right(0);
        F1(0);
        F2(0);
        LT(lt_window ? 1.0f : 0.0f);
        RT(rt_window ? 1.0f : 0.0f);
        lx(0.0f);
        ly(0.0f);
        rx(0.0f);
        ry(0.0f);
    }

private:
    std::chrono::steady_clock::time_point start_;
};
"""
  text += (
    auto_sequence.replace("__MIMIC_BUTTON__", mimic_button)
    .replace("__RT_WINDOW__", rt_window)
    .replace("__RT_A_WINDOW__", rt_a_window)
    .replace("__RB_WINDOW__", rb_window)
    .replace("__RB_A_WINDOW__", rb_a_window)
  )
  physics_joystick_path.write_text(text, encoding="utf-8")


def _inject_auto_sequence_bridge(bridge_path: Path) -> None:
  text = bridge_path.read_text(encoding="utf-8")
  if "AutoSequenceJoystick" in text:
    bridge_path.write_text(text, encoding="utf-8")
    return

  needle = """        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        }
"""
  replacement = """        if(param::config.use_joystick == 1) {
            if(param::config.joystick_type == "xbox") {
                joystick = std::make_shared<XBoxJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else if(param::config.joystick_type == "switch") {
                joystick  = std::make_shared<SwitchJoystick>(param::config.joystick_device, param::config.joystick_bits);
            } else {
                std::cerr << "Unsupported joystick type: " << param::config.joystick_type << std::endl;
                exit(EXIT_FAILURE);
            }
        } else {
            joystick = std::make_shared<AutoSequenceJoystick>();
            std::cout << "DEVIATION: AutoSequenceJoystick enabled for automation lane." << std::endl;
        }
"""
  if needle not in text:
    raise RuntimeError(f"did not find joystick construction block in {bridge_path}")
  bridge_path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")


def _inject_sim_telemetry(bridge_path: Path) -> None:
  text = bridge_path.read_text(encoding="utf-8")
  if "#include <chrono>" not in text:
    text = text.replace(
      "#include <iostream>\n", "#include <chrono>\n#include <iostream>\n"
    )
  if "SIM_TELEMETRY" in text:
    bridge_path.write_text(text, encoding="utf-8")
    return

  needle = """        if(!mj_data_) return;
"""
  replacement = """        if(!mj_data_) return;
        static auto telemetry_start = std::chrono::steady_clock::now();
        static auto telemetry_last = telemetry_start;
        const auto telemetry_now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(telemetry_now - telemetry_last).count() >= 0.25) {
            telemetry_last = telemetry_now;
            const double wall_t = std::chrono::duration<double>(telemetry_now - telemetry_start).count();
            std::cout << "SIM_TELEMETRY"
                      << " wall_t=" << wall_t
                      << " sim_t=" << mj_data_->time
                      << " base_x=" << mj_data_->qpos[0]
                      << " base_y=" << mj_data_->qpos[1]
                      << " base_z=" << mj_data_->qpos[2]
                      << " quat_w=" << mj_data_->qpos[3]
                      << " quat_x=" << mj_data_->qpos[4]
                      << " quat_y=" << mj_data_->qpos[5]
                      << " quat_z=" << mj_data_->qpos[6]
                      << std::endl;
        }
"""
  if needle not in text:
    raise RuntimeError(f"did not find bridge run guard in {bridge_path}")
  bridge_path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")


def _inject_control_telemetry(bridge_path: Path) -> None:
  text = bridge_path.read_text(encoding="utf-8")
  if "#include <cmath>" not in text:
    text = text.replace(
      "#include <mujoco/mujoco.h>\n", "#include <mujoco/mujoco.h>\n#include <cmath>\n"
    )
  if "SIM_CTRL_TELEMETRY" in text:
    bridge_path.write_text(text, encoding="utf-8")
    return

  needle = """        // lowcmd
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * (m.q() - mj_data_->sensordata[i]) +
                                    m.kd() * (m.dq() - mj_data_->sensordata[i + num_motor_]);
            }
        }
"""
  replacement = """        // lowcmd
        static auto control_telemetry_start = std::chrono::steady_clock::now();
        static auto control_telemetry_last = control_telemetry_start;
        const auto control_telemetry_now = std::chrono::steady_clock::now();
        const bool print_control_telemetry =
            std::chrono::duration<double>(control_telemetry_now - control_telemetry_last).count() >= 0.25;
        double max_abs_ctrl = 0.0;
        double max_ctrl_qcmd = 0.0;
        double max_ctrl_q = 0.0;
        double max_ctrl_dq = 0.0;
        double max_ctrl_kp = 0.0;
        double max_ctrl_kd = 0.0;
        int max_ctrl_i = -1;
        double max_abs_qerr = 0.0;
        double max_qerr_qcmd = 0.0;
        double max_qerr_q = 0.0;
        int max_qerr_i = -1;
        {
            std::lock_guard<std::mutex> lock(lowcmd->mutex_);
            for(int i(0); i<num_motor_; i++) {
                auto & m = lowcmd->msg_.motor_cmd()[i];
                const double q = mj_data_->sensordata[i];
                const double dq = mj_data_->sensordata[i + num_motor_];
                const double qerr = m.q() - q;
                mj_data_->ctrl[i] = m.tau() +
                                    m.kp() * qerr +
                                    m.kd() * (m.dq() - dq);
                if(print_control_telemetry) {
                    const double abs_ctrl = std::fabs(mj_data_->ctrl[i]);
                    const double abs_qerr = std::fabs(qerr);
                    if(abs_ctrl > max_abs_ctrl) {
                        max_abs_ctrl = abs_ctrl;
                        max_ctrl_i = i;
                        max_ctrl_qcmd = m.q();
                        max_ctrl_q = q;
                        max_ctrl_dq = dq;
                        max_ctrl_kp = m.kp();
                        max_ctrl_kd = m.kd();
                    }
                    if(abs_qerr > max_abs_qerr) {
                        max_abs_qerr = abs_qerr;
                        max_qerr_i = i;
                        max_qerr_qcmd = m.q();
                        max_qerr_q = q;
                    }
                }
            }
        }
        if(print_control_telemetry) {
            control_telemetry_last = control_telemetry_now;
            const double wall_t = std::chrono::duration<double>(control_telemetry_now - control_telemetry_start).count();
            std::cout << "SIM_CTRL_TELEMETRY"
                      << " wall_t=" << wall_t
                      << " sim_t=" << mj_data_->time
                      << " max_abs_ctrl=" << max_abs_ctrl
                      << " max_ctrl_i=" << max_ctrl_i
                      << " max_ctrl_qcmd=" << max_ctrl_qcmd
                      << " max_ctrl_q=" << max_ctrl_q
                      << " max_ctrl_dq=" << max_ctrl_dq
                      << " max_ctrl_kp=" << max_ctrl_kp
                      << " max_ctrl_kd=" << max_ctrl_kd
                      << " max_abs_qerr=" << max_abs_qerr
                      << " max_qerr_i=" << max_qerr_i
                      << " max_qerr_qcmd=" << max_qerr_qcmd
                      << " max_qerr_q=" << max_qerr_q
                      << std::endl;
        }
"""
  if needle not in text:
    raise RuntimeError(f"did not find lowcmd control block in {bridge_path}")
  bridge_path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")


def _inject_start_paused_warmup(
  main_path: Path, *, unpause_delay_seconds: float
) -> None:
  text = main_path.read_text(encoding="utf-8")
  if "automation unpaused simulator after 8s controller warmup" in text:
    main_path.write_text(text, encoding="utf-8")
    return

  needle = """  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);
"""
  replacement = """  auto sim = std::make_unique<mj::Simulate>(
    std::make_unique<mj::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false);
  sim->run = 0;
  std::thread automation_unpause_thread([sim_ptr = sim.get()]() {
    std::this_thread::sleep_for(std::chrono::duration<double>(__UNPAUSE_DELAY_SECONDS__));
    sim_ptr->run = 1;
    std::cout << "DEVIATION: automation unpaused simulator after __UNPAUSE_DELAY_SECONDS__s controller warmup." << std::endl;
  });
  automation_unpause_thread.detach();

  std::thread unitree_thread(UnitreeSdk2BridgeThread, nullptr);
"""
  if needle not in text:
    raise RuntimeError(f"did not find simulator construction block in {main_path}")
  main_path.write_text(
    text.replace(
      needle,
      replacement.replace("__UNPAUSE_DELAY_SECONDS__", f"{unpause_delay_seconds:g}"),
      1,
    ),
    encoding="utf-8",
  )


def _copy_flying_kick_policy(
  source_root: Path, out_root: Path
) -> list[dict[str, object]]:
  if not source_root.is_dir():
    raise RuntimeError(f"missing flying-kick policy root: {source_root}")
  required = [
    source_root / "exported/policy.onnx",
    source_root / "params/deploy.yaml",
    source_root / "params/flying_kick.npz",
  ]
  missing = [str(path) for path in required if not path.is_file()]
  if missing:
    raise RuntimeError("missing flying-kick policy files: " + ", ".join(missing))

  target_root = out_root / "deploy/robots/g1/config/policy/mimic/flying_kick"
  if target_root.exists():
    shutil.rmtree(target_root)
  shutil.copytree(source_root, target_root)
  return [_file_record(path, root=source_root.parent) for path in required]


def _inject_flying_kick_config(config_path: Path) -> None:
  text = config_path.read_text(encoding="utf-8")
  if "Mimic_FlyingKick:" in text:
    config_path.write_text(text, encoding="utf-8")
    return

  enabled_anchor = """    Mimic_Dance1_subject2:
      id: 5
      type: Mimic
"""
  enabled_replacement = (
    enabled_anchor
    + """
    Mimic_FlyingKick:
      id: 6
      type: Mimic
"""
  )
  if enabled_anchor not in text:
    raise RuntimeError(f"did not find enabled Mimic_Dance1 block in {config_path}")
  text = text.replace(enabled_anchor, enabled_replacement, 1)

  transition_anchor = """      Mimic_Dance1_subject2: RB + A.on_pressed
"""
  transition_replacement = (
    transition_anchor
    + """      Mimic_FlyingKick: RB + X.on_pressed
"""
  )
  if transition_anchor not in text:
    raise RuntimeError(f"did not find Velocity mimic transition block in {config_path}")
  text = text.replace(transition_anchor, transition_replacement, 1)

  state_anchor = """  Mimic_Dance1_subject2:
    transitions:
      Passive: LT + B.on_pressed
      Velocity: RT + A.on_pressed

    motion_file: config/policy/mimic/dance1_subject2/params/dance1_subject2.npz
    policy_dir: config/policy/mimic/dance1_subject2/
    time_start: 0.0
    time_end: 1000.0
"""
  state_replacement = (
    state_anchor
    + """

  Mimic_FlyingKick:
    transitions:
      Passive: LT + B.on_pressed
      Velocity: RT + A.on_pressed

    motion_file: config/policy/mimic/flying_kick/params/flying_kick.npz
    policy_dir: config/policy/mimic/flying_kick/
    time_start: 0.0
    time_end: 1000.0
"""
  )
  if state_anchor not in text:
    raise RuntimeError(f"did not find Mimic_Dance1 state block in {config_path}")
  config_path.write_text(
    text.replace(state_anchor, state_replacement, 1), encoding="utf-8"
  )


def _sync_mjlab_mode15_model(
  out_root: Path, mode15_xml: Path
) -> list[dict[str, object]]:
  if not mode15_xml.is_file():
    raise RuntimeError(f"missing mjlab mode-15 XML: {mode15_xml}")
  mode15_asset_dir = mode15_xml.parent / "assets"
  if not mode15_asset_dir.is_dir():
    raise RuntimeError(f"missing mjlab mode-15 asset dir: {mode15_asset_dir}")

  sim_xml_dir = out_root / "src/assets/robots/unitree_g1/xmls"
  scene_xml = sim_xml_dir / "scene_g1.xml"
  sim_g1_xml = sim_xml_dir / "g1.xml"
  sim_asset_dir = sim_xml_dir / "assets"
  if not scene_xml.is_file():
    raise RuntimeError(f"missing official scene XML: {scene_xml}")

  copied_assets: list[dict[str, object]] = []
  sim_asset_dir.mkdir(parents=True, exist_ok=True)
  for asset in sorted(mode15_asset_dir.glob("*_5010.STL")):
    target = sim_asset_dir / asset.name
    shutil.copy2(asset, target)
    copied_assets.append(_file_record(target, root=out_root))
  shutil.copy2(mode15_xml, sim_g1_xml)

  mode15 = ET.parse(mode15_xml).getroot()
  current_scene = ET.parse(scene_xml).getroot()

  scene = ET.Element("mujoco", {"model": "scene_g1"})
  compiler = mode15.find("compiler")
  if compiler is None:
    raise RuntimeError(f"missing compiler block in {mode15_xml}")
  compiler = copy.deepcopy(compiler)
  compiler.set("meshdir", "assets")
  scene.append(compiler)

  default = mode15.find("default")
  if default is not None:
    scene.append(copy.deepcopy(default))

  asset = ET.Element("asset")
  mode15_asset = mode15.find("asset")
  if mode15_asset is None:
    raise RuntimeError(f"missing asset block in {mode15_xml}")
  for child in list(mode15_asset):
    asset.append(copy.deepcopy(child))
  existing_names = {
    child.attrib.get("name") for child in asset if child.attrib.get("name")
  }
  for old_asset in current_scene.findall("asset"):
    for child in list(old_asset):
      name = child.attrib.get("name")
      if child.tag == "mesh" or (name and name in existing_names):
        continue
      asset.append(copy.deepcopy(child))
  scene.append(asset)

  world = ET.Element("worldbody")
  mode15_world = mode15.find("worldbody")
  if mode15_world is None:
    raise RuntimeError(f"missing worldbody block in {mode15_xml}")
  for child in list(mode15_world):
    world.append(copy.deepcopy(child))
  for old_world in current_scene.findall("worldbody"):
    for child in list(old_world):
      if child.tag == "body" and child.attrib.get("name") == "pelvis":
        continue
      if child.tag == "light" and child.attrib.get("mode") == "trackcom":
        continue
      world.append(copy.deepcopy(child))
  scene.append(world)

  contact = mode15.find("contact")
  if contact is not None:
    scene.append(copy.deepcopy(contact))

  actuator = current_scene.find("actuator")
  if actuator is None:
    raise RuntimeError(f"missing actuator block in {scene_xml}")
  scene.append(copy.deepcopy(actuator))

  sensor = current_scene.find("sensor")
  if sensor is None:
    raise RuntimeError(f"missing sensor block in {scene_xml}")
  sensor = copy.deepcopy(sensor)
  for elem in sensor:
    if elem.tag == "framequat" and elem.attrib.get("name") == "imu_quat":
      elem.set("objname", "imu_in_pelvis")
    if elem.tag in {"gyro", "accelerometer"} and elem.attrib.get("name") in {
      "imu_gyro",
      "imu_acc",
    }:
      elem.set("site", "imu_in_pelvis")
    if elem.tag in {"framepos", "framelinvel"} and elem.attrib.get("name") in {
      "frame_pos",
      "frame_vel",
    }:
      elem.set("objname", "imu_in_pelvis")
  scene.append(sensor)

  for tag in ("statistic", "visual"):
    elem = current_scene.find(tag)
    if elem is not None:
      scene.append(copy.deepcopy(elem))

  ET.indent(scene, space="  ")
  scene_xml.write_text(ET.tostring(scene, encoding="unicode") + "\n", encoding="utf-8")

  records = [_file_record(mode15_xml), _file_record(sim_g1_xml, root=out_root)]
  records.extend(copied_assets)
  records.append(_file_record(scene_xml, root=out_root))
  return records


def _align_mjlab_actuator_limits(
  scene_xml: Path, mode15_xml: Path
) -> dict[str, object]:
  from mjlab.scripts.g1_tracking_phase1_velocity_actuator_contract import (
    _expected_mjlab_effort_limits,
  )

  expected = _expected_mjlab_effort_limits(mjlab_g1_xml=mode15_xml)
  tree = ET.parse(scene_xml)
  root = tree.getroot()
  changed: list[dict[str, object]] = []
  for motor in root.findall(".//actuator/motor"):
    joint_name = motor.attrib.get("joint")
    if joint_name not in expected:
      continue
    limit = expected[joint_name]
    new_ctrlrange = f"-{limit:g} {limit:g}"
    old_ctrlrange = motor.attrib.get("ctrlrange")
    if old_ctrlrange == new_ctrlrange:
      continue
    motor.set("ctrlrange", new_ctrlrange)
    changed.append(
      {
        "joint": joint_name,
        "old_ctrlrange": old_ctrlrange,
        "new_ctrlrange": new_ctrlrange,
      }
    )
  if changed:
    ET.indent(root, space="  ")
    tree.write(scene_xml, encoding="unicode", xml_declaration=False)
    with scene_xml.open("a", encoding="utf-8") as f:
      f.write("\n")
  return {"changed": changed}


def _apply_official_joint_passive_defaults(scene_xml: Path) -> dict[str, object]:
  tree = ET.parse(scene_xml)
  root = tree.getroot()
  default = root.find("default")
  if default is None:
    default = ET.Element("default")
    root.insert(1, default)
  target_default = default.find("./default[@class='g1']")
  if target_default is None:
    target_default = default

  joint = target_default.find("joint")
  old_attrib = dict(joint.attrib) if joint is not None else None
  if joint is None:
    joint = ET.Element("joint")
    target_default.insert(0, joint)
  joint.set("damping", "0.05")
  joint.set("armature", "0.01")
  joint.set("frictionloss", "0.2")

  ET.indent(root, space="  ")
  tree.write(scene_xml, encoding="unicode", xml_declaration=False)
  with scene_xml.open("a", encoding="utf-8") as f:
    f.write("\n")
  return {
    "old": old_attrib,
    "new": {"damping": "0.05", "armature": "0.01", "frictionloss": "0.2"},
  }


def _set_fixstand_target(out_root: Path, target_name: str) -> dict[str, object]:
  if target_name == "official":
    return {"target": target_name, "changed": False}

  import yaml

  config_path = out_root / "deploy/robots/g1/config/config.yaml"
  if target_name == "zero":
    target = [0.0] * 29
  elif target_name == "velocity_default":
    deploy_path = (
      out_root / "deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml"
    )
    target = yaml.safe_load(deploy_path.read_text(encoding="utf-8"))[
      "default_joint_pos"
    ]
  elif target_name == "flying_default":
    deploy_path = (
      out_root / "deploy/robots/g1/config/policy/mimic/flying_kick/params/deploy.yaml"
    )
    if not deploy_path.is_file():
      raise RuntimeError(
        "--fixstand-target flying_default requires --include-flying-kick"
      )
    target = yaml.safe_load(deploy_path.read_text(encoding="utf-8"))[
      "default_joint_pos"
    ]
  else:
    raise RuntimeError(f"unsupported FixStand target: {target_name}")

  config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
  old_target = config["FSM"]["FixStand"]["qs"][1]
  config["FSM"]["FixStand"]["qs"][1] = target
  config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
  return {
    "target": target_name,
    "changed": True,
    "old_target": old_target,
    "new_target": target,
  }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--official-root", type=Path, default=DEFAULT_OFFICIAL_ROOT)
  parser.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
  parser.add_argument("--include-flying-kick", action="store_true")
  parser.add_argument("--use-mjlab-mode15-model", action="store_true")
  parser.add_argument("--align-mjlab-actuator-limits", action="store_true")
  parser.add_argument("--apply-official-joint-passive-defaults", action="store_true")
  parser.add_argument(
    "--automation-sequence",
    choices=("full", "fixstand_only"),
    default="full",
  )
  parser.add_argument("--unpause-delay-seconds", type=float, default=8.0)
  parser.add_argument("--debug-control-telemetry", action="store_true")
  parser.add_argument(
    "--fixstand-target",
    choices=("official", "zero", "velocity_default", "flying_default"),
    default="official",
  )
  parser.add_argument("--mjlab-mode15-xml", type=Path, default=DEFAULT_MJLAB_MODE15_XML)
  parser.add_argument(
    "--flying-kick-policy-root",
    type=Path,
    default=DEFAULT_FLYING_KICK_POLICY_ROOT,
  )
  parser.add_argument("--force", action="store_true")
  args = parser.parse_args()

  official_root = args.official_root.resolve()
  out_root = args.out_root.resolve()
  if not official_root.is_dir():
    print(f"missing official root: {official_root}", file=sys.stderr)
    return 2
  if out_root.exists():
    if not args.force:
      print(f"output root already exists: {out_root}", file=sys.stderr)
      print("rerun with --force to replace it", file=sys.stderr)
      return 2
    shutil.rmtree(out_root)

  shutil.copytree(official_root, out_root, symlinks=True)
  _drop_stale_simulator_build(out_root)
  _replace_use_joystick(out_root / "simulate/config.yaml")
  _inject_auto_sequence_joystick(
    out_root / "simulate/src/physics_joystick.h",
    include_flying_kick=args.include_flying_kick,
    automation_sequence=args.automation_sequence,
  )
  _inject_auto_sequence_bridge(out_root / "simulate/src/unitree_sdk2_bridge.h")
  _inject_sim_telemetry(out_root / "simulate/src/unitree_sdk2_bridge.h")
  if args.debug_control_telemetry:
    _inject_control_telemetry(out_root / "simulate/src/unitree_sdk2_bridge.h")
  _inject_start_paused_warmup(
    out_root / "simulate/src/main.cc",
    unpause_delay_seconds=args.unpause_delay_seconds,
  )
  mode15_model_assets = None
  actuator_limit_alignment = None
  official_joint_passive_defaults = None
  if args.use_mjlab_mode15_model:
    mode15_xml = args.mjlab_mode15_xml.resolve()
    mode15_model_assets = _sync_mjlab_mode15_model(out_root, mode15_xml)
    if args.apply_official_joint_passive_defaults:
      official_joint_passive_defaults = _apply_official_joint_passive_defaults(
        out_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
      )
    if args.align_mjlab_actuator_limits:
      actuator_limit_alignment = _align_mjlab_actuator_limits(
        out_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        mode15_xml,
      )
  elif args.align_mjlab_actuator_limits:
    raise RuntimeError(
      "--align-mjlab-actuator-limits requires --use-mjlab-mode15-model"
    )
  elif args.apply_official_joint_passive_defaults:
    raise RuntimeError(
      "--apply-official-joint-passive-defaults requires --use-mjlab-mode15-model"
    )
  flying_kick_assets = None
  if args.include_flying_kick:
    flying_kick_source = args.flying_kick_policy_root.resolve()
    flying_kick_assets = _copy_flying_kick_policy(flying_kick_source, out_root)
    _inject_flying_kick_config(out_root / "deploy/robots/g1/config/config.yaml")
  fixstand_target = _set_fixstand_target(out_root, args.fixstand_target)

  manifest = {
    "lane": "official_source_plus_automation_deviation",
    "claim": "not_clean_official_baseline",
    "official_root": str(official_root),
    "out_root": str(out_root),
    "official_head": _git_head(official_root),
    "automation_sequence": args.automation_sequence,
    "unpause_delay_seconds": args.unpause_delay_seconds,
    "fixstand_target": fixstand_target,
    "allowed_deviations": [
      {
        "path": "simulate/config.yaml",
        "change": "use_joystick: 1 -> 0",
        "reason": "current host has no /dev/input/js0; selects synthetic automation input",
      },
      {
        "path": "simulate/src/physics_joystick.h",
        "change": f"add AutoSequenceJoystick sequence={args.automation_sequence}",
        "reason": "exercise the official FSM transition path without claiming clean official baseline",
      },
      {
        "path": "simulate/src/unitree_sdk2_bridge.h",
        "change": "instantiate AutoSequenceJoystick when use_joystick is disabled",
        "reason": "publish joystick state through the same LowState path used by the controller",
      },
      {
        "path": "simulate/src/unitree_sdk2_bridge.h",
        "change": "add SIM_TELEMETRY base pose logging",
        "reason": "make stand/fall behavior observable without relying on a single screenshot",
      },
      {
        "path": "simulate/src/main.cc",
        "change": f"start paused and unpause after {args.unpause_delay_seconds:g}s controller warmup",
        "reason": "avoid the robot falling before the controller reaches FixStand in no-joystick automation",
      },
    ],
    "changed_paths": [
      "simulate/config.yaml",
      "simulate/src/physics_joystick.h",
      "simulate/src/unitree_sdk2_bridge.h",
      "simulate/src/main.cc",
      "AUTOMATION_DEVIATION_MANIFEST.json",
    ],
    "forbidden_claims": [
      "clean official baseline",
      "hardware readiness",
      "policy quality acceptance",
    ],
  }
  if args.include_flying_kick:
    manifest["flying_kick_source_root"] = str(flying_kick_source)
    manifest["flying_kick_assets"] = flying_kick_assets
    manifest["allowed_deviations"].extend(
      [
        {
          "path": "deploy/robots/g1/config/policy/mimic/flying_kick",
          "change": "copy local flying-kick deploy policy assets into the official-source deviation lane",
          "reason": "observe the requested flying-kick policy behavior in sim2sim without mutating the clean official clone",
        },
        {
          "path": "deploy/robots/g1/config/config.yaml",
          "change": "add Mimic_FlyingKick state and Velocity RB+X transition",
          "reason": "make the copied flying-kick policy reachable through the existing FSM",
        },
        {
          "path": "simulate/src/physics_joystick.h",
          "change": "use RB+X as the automation mimic trigger",
          "reason": "select Mimic_FlyingKick instead of the official dance1_subject2 demo",
        },
      ]
    )
    manifest["changed_paths"].extend(
      [
        "deploy/robots/g1/config/config.yaml",
        "deploy/robots/g1/config/policy/mimic/flying_kick",
      ]
    )
  if args.use_mjlab_mode15_model:
    manifest["mjlab_mode15_xml"] = str(args.mjlab_mode15_xml.resolve())
    manifest["mjlab_mode15_model_assets"] = mode15_model_assets
    manifest["allowed_deviations"].append(
      {
        "path": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "change": "replace official G1 body/collision/mesh model with mjlab new-G1 mode-15 model while preserving official actuator and sensor blocks",
        "reason": "test whether the flying-kick failure is caused by training/deploy robot model mismatch",
      }
    )
    manifest["changed_paths"].extend(
      [
        "src/assets/robots/unitree_g1/xmls/g1.xml",
        "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "src/assets/robots/unitree_g1/xmls/assets/*_5010.STL",
      ]
    )
  if args.align_mjlab_actuator_limits:
    manifest["mjlab_actuator_limit_alignment"] = actuator_limit_alignment
    manifest["allowed_deviations"].append(
      {
        "path": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "change": "align motor ctrlrange values to mjlab G1 training effort limits",
        "reason": "separate robot body/collision mismatch from actuator-limit mismatch",
      }
    )
  if args.apply_official_joint_passive_defaults:
    manifest["official_joint_passive_defaults"] = official_joint_passive_defaults
    manifest["allowed_deviations"].append(
      {
        "path": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "change": "apply official joint damping/armature/frictionloss defaults to the mode-15 scene",
        "reason": "test whether new-G1 collapse is caused by missing passive joint defaults in the deployment simulator",
      }
    )
  if args.debug_control_telemetry:
    manifest["allowed_deviations"].append(
      {
        "path": "simulate/src/unitree_sdk2_bridge.h",
        "change": "add SIM_CTRL_TELEMETRY lowcmd/control logging",
        "reason": "diagnose whether stand collapse is caused by target, sensor, gain, or torque contract mismatch",
      }
    )
  if args.fixstand_target != "official":
    manifest["allowed_deviations"].append(
      {
        "path": "deploy/robots/g1/config/config.yaml",
        "change": f"set FixStand final target to {args.fixstand_target}",
        "reason": "test whether new-G1 collapse is caused by the official FixStand/default pose contract",
      }
    )
    manifest["changed_paths"].append("deploy/robots/g1/config/config.yaml")
  manifest_path = out_root / "AUTOMATION_DEVIATION_MANIFEST.json"
  manifest_path.write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
  )
  print(json.dumps(manifest, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
