"""Generate labeled Unitree G1 sim2sim output lanes."""

from __future__ import annotations

import copy
import hashlib
import json
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from mjlab.sim2sim.unitree.contracts import (
  PrepareG1Request,
  UnitreeSim2SimError,
  validate_source_checkout,
)

MANIFEST_NAME = "UNITREE_SIM2SIM_MANIFEST.json"


def prepare_g1_lane(request: PrepareG1Request) -> dict[str, Any]:
  """Generate a G1 sim2sim lane and return its manifest."""
  source_root = request.source.resolved_root
  out_root = request.output.resolved_root
  _validate_request(request, source_root, out_root)

  if out_root.exists():
    shutil.rmtree(out_root)
  shutil.copytree(
    source_root,
    out_root,
    symlinks=True,
    ignore=shutil.ignore_patterns(".git"),
  )
  _drop_runtime_outputs(out_root)

  changed_paths: list[str] = []
  deviations: list[dict[str, str]] = []

  _replace_use_joystick(out_root / "simulate/config.yaml")
  _record(changed_paths, "simulate/config.yaml")
  deviations.append(
    {
      "label": "automation_input",
      "path": "simulate/config.yaml",
      "change": "disable physical joystick for synthetic automation",
    }
  )

  _inject_auto_sequence_joystick(
    out_root / "simulate/src/physics_joystick.h",
    mimic_button=_mimic_button(request.action.trigger),
    automation_sequence=request.deviations.automation_sequence,
  )
  _record(changed_paths, "simulate/src/physics_joystick.h")
  deviations.append(
    {
      "label": "automation_input",
      "path": "simulate/src/physics_joystick.h",
      "change": f"add AutoSequenceJoystick sequence={request.deviations.automation_sequence}",
    }
  )

  _inject_auto_sequence_bridge(out_root / "simulate/src/unitree_sdk2_bridge.h")
  _record(changed_paths, "simulate/src/unitree_sdk2_bridge.h")
  deviations.append(
    {
      "label": "automation_input",
      "path": "simulate/src/unitree_sdk2_bridge.h",
      "change": "instantiate AutoSequenceJoystick when use_joystick is disabled",
    }
  )

  _inject_start_paused_warmup(
    out_root / "simulate/src/main.cc",
    unpause_delay_seconds=request.deviations.unpause_delay_seconds,
  )
  _record(changed_paths, "simulate/src/main.cc")
  deviations.append(
    {
      "label": "automation_input",
      "path": "simulate/src/main.cc",
      "change": (
        "start paused and unpause after "
        f"{request.deviations.unpause_delay_seconds:g}s controller warmup"
      ),
    }
  )

  policy_assets = _copy_action_policy(request, out_root)
  for record in policy_assets:
    _record(changed_paths, str(record["path"]))
  deviations.append(
    {
      "label": "policy_asset",
      "path": f"deploy/robots/g1/config/policy/mimic/{request.action.policy_subdir}",
      "change": f"copy selected {request.action.name} policy assets",
    }
  )

  _inject_action_config(out_root / "deploy/robots/g1/config/config.yaml", request)
  _record(changed_paths, "deploy/robots/g1/config/config.yaml")
  deviations.append(
    {
      "label": "policy_asset",
      "path": "deploy/robots/g1/config/config.yaml",
      "change": f"add {request.action.state_name} state and transition",
    }
  )

  model_assets: list[dict[str, Any]] = []
  actuator_limit_alignment: dict[str, Any] | None = None
  passive_defaults: dict[str, Any] | None = None
  if request.deviations.use_mjlab_mode15_model:
    if request.mjlab_model_xml is None:
      raise UnitreeSim2SimError("--use-mjlab-mode15-model requires --mjlab-model-xml")
    model_assets = _sync_mjlab_mode15_model(
      out_root,
      request.mjlab_model_xml.expanduser().resolve(),
    )
    for record in model_assets:
      _record(changed_paths, str(record["path"]))
    deviations.append(
      {
        "label": "mjlab_mode15_model",
        "path": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
        "change": "sync mjlab mode-15 body/collision model into Unitree scene",
      }
    )
    scene_xml = out_root / "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
    if request.deviations.apply_official_joint_passive_defaults:
      passive_defaults = _apply_official_joint_passive_defaults(scene_xml)
      _record(changed_paths, "src/assets/robots/unitree_g1/xmls/scene_g1.xml")
      deviations.append(
        {
          "label": "official_joint_passive_defaults",
          "path": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
          "change": "apply official passive joint defaults",
        }
      )
    if request.deviations.align_mjlab_actuator_limits:
      actuator_limit_alignment = _align_mjlab_actuator_limits(
        scene_xml,
        request.mjlab_model_xml.expanduser().resolve(),
      )
      _record(changed_paths, "src/assets/robots/unitree_g1/xmls/scene_g1.xml")
      deviations.append(
        {
          "label": "mjlab_actuator_limit_alignment",
          "path": "src/assets/robots/unitree_g1/xmls/scene_g1.xml",
          "change": "align motor ctrlrange values to mjlab training limits",
        }
      )
  elif request.deviations.align_mjlab_actuator_limits:
    raise UnitreeSim2SimError(
      "--align-mjlab-actuator-limits requires --use-mjlab-mode15-model"
    )
  elif request.deviations.apply_official_joint_passive_defaults:
    raise UnitreeSim2SimError(
      "--apply-official-joint-passive-defaults requires --use-mjlab-mode15-model"
    )

  if request.deviations.diagnostic_trace:
    _inject_sim_telemetry(out_root / "simulate/src/unitree_sdk2_bridge.h")
    _record(changed_paths, "simulate/src/unitree_sdk2_bridge.h")
    deviations.append(
      {
        "label": "diagnostic_trace",
        "path": "simulate/src/unitree_sdk2_bridge.h",
        "change": "emit SIM_TELEMETRY base pose logging",
      }
    )

  manifest_path = out_root / MANIFEST_NAME
  _record(changed_paths, MANIFEST_NAME)
  deviation_labels = list(dict.fromkeys([*request.deviations.labels(), "policy_asset"]))
  manifest = {
    "schema_version": 1,
    "lane": "official_source_plus_automation_deviation",
    "claim": "not_clean_official_baseline",
    "source_root": str(source_root),
    "source_sha": _source_sha(source_root),
    "out_root": str(out_root),
    "manifest_path": str(manifest_path),
    "action": {
      "name": request.action.name,
      "state_name": request.action.state_name,
      "policy_subdir": request.action.policy_subdir,
      "trigger": request.action.trigger,
    },
    "automation_sequence": request.deviations.automation_sequence,
    "unpause_delay_seconds": request.deviations.unpause_delay_seconds,
    "deviation_labels": deviation_labels,
    "allowed_deviations": deviations,
    "changed_paths": changed_paths,
    "policy_assets": policy_assets,
    "model_assets": model_assets,
    "actuator_limit_alignment": actuator_limit_alignment,
    "official_joint_passive_defaults": passive_defaults,
    "evidence_dir": (
      str(request.evidence_dir.expanduser().resolve())
      if request.evidence_dir is not None
      else None
    ),
    "forbidden_claims": [
      "clean official baseline",
      "hardware readiness",
      "real robot safety certification",
    ],
  }
  manifest_path.write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
  )
  return manifest


def _validate_request(
  request: PrepareG1Request,
  source_root: Path,
  out_root: Path,
) -> None:
  validate_source_checkout(request.source)
  if source_root == out_root or source_root in out_root.parents:
    raise UnitreeSim2SimError("output root must not be the official source root")
  if out_root.exists() and not request.output.force:
    raise UnitreeSim2SimError(
      f"output root already exists: {out_root}; rerun with --force to replace it"
    )
  if request.policy_root is None:
    raise UnitreeSim2SimError("--policy-root is required for selected action assets")
  missing_policy = [
    path
    for path in request.action.required_policy_files(
      request.policy_root.expanduser().resolve()
    )
    if not path.is_file()
  ]
  if missing_policy:
    missing_display = ", ".join(str(path) for path in missing_policy)
    raise UnitreeSim2SimError(f"missing action policy files: {missing_display}")


def _record(paths: list[str], path: str) -> None:
  if path not in paths:
    paths.append(path)


def _drop_runtime_outputs(out_root: Path) -> None:
  shutil.rmtree(out_root / "simulate/build", ignore_errors=True)
  shutil.rmtree(out_root / "deploy/robots/g1/build", ignore_errors=True)


def _source_sha(root: Path) -> str:
  try:
    proc = subprocess.run(
      ["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
      check=True,
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      text=True,
    )
  except (FileNotFoundError, subprocess.CalledProcessError):
    return "sha256:" + _tree_hash(root)
  return proc.stdout.strip()


def _tree_hash(root: Path) -> str:
  digest = hashlib.sha256()
  for path in sorted(child for child in root.rglob("*") if child.is_file()):
    if ".git" in path.relative_to(root).parts:
      continue
    relative = path.relative_to(root)
    digest.update(str(relative).encode("utf-8"))
    digest.update(b"\0")
    digest.update(path.read_bytes())
    digest.update(b"\0")
  return digest.hexdigest()


def _sha256_file(path: Path) -> str:
  digest = hashlib.sha256()
  with path.open("rb") as handle:
    for chunk in iter(lambda: handle.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest()


def _file_record(path: Path, *, root: Path) -> dict[str, Any]:
  resolved = path.resolve()
  return {
    "path": str(resolved.relative_to(root.resolve())),
    "sha256": _sha256_file(resolved),
    "size_bytes": resolved.stat().st_size,
  }


def _replace_use_joystick(config_path: Path) -> None:
  lines = config_path.read_text(encoding="utf-8").splitlines()
  output: list[str] = []
  replaced = False
  for line in lines:
    if line.strip().startswith("use_joystick:"):
      output.append("use_joystick: 0 # DEVIATION: synthetic automation input")
      replaced = True
    else:
      output.append(line)
  if not replaced:
    raise UnitreeSim2SimError(f"did not find use_joystick in {config_path}")
  config_path.write_text("\n".join(output) + "\n", encoding="utf-8")


def _mimic_button(trigger: str) -> str:
  button = trigger.split("+")[-1].strip()
  if button not in {"A", "B", "X", "Y"}:
    raise UnitreeSim2SimError(f"unsupported automation trigger: {trigger}")
  return button


def _inject_auto_sequence_joystick(
  physics_joystick_path: Path,
  *,
  mimic_button: str,
  automation_sequence: str,
) -> None:
  text = physics_joystick_path.read_text(encoding="utf-8")
  if "class AutoSequenceJoystick" in text:
    return
  if "#include <chrono>" not in text:
    text = text.replace(
      "#include <iostream>\n",
      "#include <iostream>\n#include <chrono>\n",
    )
  if automation_sequence == "full":
    rt_window = "t >= 7.0 && t < 10.0"
    rt_a_window = (
      "(t >= 7.4 && t < 7.7) ||\n"
      "            (t >= 8.4 && t < 8.7) ||\n"
      "            (t >= 9.4 && t < 9.7)"
    )
    rb_window = "t >= 11.0 && t < 14.0"
    rb_press_window = (
      "(t >= 11.4 && t < 11.7) ||\n"
      "            (t >= 12.4 && t < 12.7) ||\n"
      "            (t >= 13.4 && t < 13.7)"
    )
  elif automation_sequence == "fixstand_only":
    rt_window = "false"
    rt_a_window = "false"
    rb_window = "false"
    rb_press_window = "false"
  else:
    raise UnitreeSim2SimError(f"unsupported automation sequence: {automation_sequence}")

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
        const bool rb_window = __RB_WINDOW__;
        const bool rt_window = __RT_WINDOW__;
        const bool rt_a_window =
            __RT_A_WINDOW__;
        const bool mimic_press_window =
            __RB_PRESS_WINDOW__;

        back(0);
        start(0);
        LS(0);
        RS(0);
        LB(0);
        RB(rb_window ? 1 : 0);
        A((rt_a_window || (mimic_press_window && __BUTTON_A__)) ? 1 : 0);
        B((mimic_press_window && __BUTTON_B__) ? 1 : 0);
        X((mimic_press_window && __BUTTON_X__) ? 1 : 0);
        Y((mimic_press_window && __BUTTON_Y__) ? 1 : 0);
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
  replacements = {
    "__RB_WINDOW__": rb_window,
    "__RT_WINDOW__": rt_window,
    "__RT_A_WINDOW__": rt_a_window,
    "__RB_PRESS_WINDOW__": rb_press_window,
    "__BUTTON_A__": "true" if mimic_button == "A" else "false",
    "__BUTTON_B__": "true" if mimic_button == "B" else "false",
    "__BUTTON_X__": "true" if mimic_button == "X" else "false",
    "__BUTTON_Y__": "true" if mimic_button == "Y" else "false",
  }
  for needle, replacement in replacements.items():
    auto_sequence = auto_sequence.replace(needle, replacement)
  physics_joystick_path.write_text(text + auto_sequence, encoding="utf-8")


def _inject_auto_sequence_bridge(bridge_path: Path) -> None:
  text = bridge_path.read_text(encoding="utf-8")
  if "AutoSequenceJoystick" in text:
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
    raise UnitreeSim2SimError(
      f"did not find joystick construction block in {bridge_path}"
    )
  bridge_path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")


def _inject_start_paused_warmup(
  main_path: Path,
  *,
  unpause_delay_seconds: float,
) -> None:
  text = main_path.read_text(encoding="utf-8")
  if "automation unpaused simulator after" in text:
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
    raise UnitreeSim2SimError(
      f"did not find simulator construction block in {main_path}"
    )
  delay = f"{unpause_delay_seconds:g}"
  main_path.write_text(
    text.replace(
      needle,
      replacement.replace("__UNPAUSE_DELAY_SECONDS__", delay),
      1,
    ),
    encoding="utf-8",
  )


def _copy_action_policy(
  request: PrepareG1Request,
  out_root: Path,
) -> list[dict[str, Any]]:
  if request.policy_root is None:
    raise UnitreeSim2SimError("--policy-root is required")
  source_root = request.policy_root.expanduser().resolve()
  target_root = (
    out_root / "deploy/robots/g1/config/policy/mimic" / request.action.policy_subdir
  )
  if target_root.exists():
    shutil.rmtree(target_root)
  shutil.copytree(source_root, target_root)
  return [
    _file_record(
      target_root / "exported" / request.action.policy_filename, root=out_root
    ),
    _file_record(
      target_root / "params" / request.action.deploy_yaml_filename, root=out_root
    ),
    _file_record(
      target_root / "params" / request.action.motion_filename, root=out_root
    ),
  ]


def _inject_action_config(config_path: Path, request: PrepareG1Request) -> None:
  import yaml

  config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
  fsm = config.setdefault("FSM", {})
  fsm.setdefault("initial_state", "Passive")
  enabled = fsm.setdefault("_", {})
  enabled[request.action.state_name] = {
    "id": 7 if request.action.name == "flying_kick" else 8,
    "type": "Mimic",
  }
  velocity = fsm.setdefault("Velocity", {})
  transitions = velocity.setdefault("transitions", {})
  transitions[request.action.state_name] = f"{request.action.trigger}.on_pressed"
  fsm[request.action.state_name] = {
    "transitions": {
      "Passive": "B.on_pressed",
      "Velocity": "RB + B.on_pressed",
    },
    "motion_file": (
      "config/policy/mimic/"
      f"{request.action.policy_subdir}/params/{request.action.motion_filename}"
    ),
    "policy_dir": f"config/policy/mimic/{request.action.policy_subdir}/",
    "time_start": 0.0,
    "time_end": 1000.0,
    "end_state": "Velocity",
  }
  config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")


def _sync_mjlab_mode15_model(out_root: Path, mode15_xml: Path) -> list[dict[str, Any]]:
  if not mode15_xml.is_file():
    raise UnitreeSim2SimError(f"missing mjlab mode-15 XML: {mode15_xml}")
  mode15_asset_dir = mode15_xml.parent / "assets"
  if not mode15_asset_dir.is_dir():
    raise UnitreeSim2SimError(f"missing mjlab mode-15 asset dir: {mode15_asset_dir}")

  sim_xml_dir = out_root / "src/assets/robots/unitree_g1/xmls"
  scene_xml = sim_xml_dir / "scene_g1.xml"
  sim_g1_xml = sim_xml_dir / "g1.xml"
  sim_asset_dir = sim_xml_dir / "assets"
  if not scene_xml.is_file():
    raise UnitreeSim2SimError(f"missing official scene XML: {scene_xml}")

  copied_assets: list[dict[str, Any]] = []
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
    raise UnitreeSim2SimError(f"missing compiler block in {mode15_xml}")
  compiler = copy.deepcopy(compiler)
  compiler.set("meshdir", "assets")
  scene.append(compiler)

  default = mode15.find("default")
  if default is not None:
    scene.append(copy.deepcopy(default))

  asset = ET.Element("asset")
  mode15_asset = mode15.find("asset")
  if mode15_asset is None:
    raise UnitreeSim2SimError(f"missing asset block in {mode15_xml}")
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
    raise UnitreeSim2SimError(f"missing worldbody block in {mode15_xml}")
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
    raise UnitreeSim2SimError(f"missing actuator block in {scene_xml}")
  scene.append(copy.deepcopy(actuator))

  sensor = current_scene.find("sensor")
  if sensor is None:
    raise UnitreeSim2SimError(f"missing sensor block in {scene_xml}")
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

  records = [_file_record(sim_g1_xml, root=out_root)]
  records.extend(copied_assets)
  records.append(_file_record(scene_xml, root=out_root))
  return records


def _align_mjlab_actuator_limits(scene_xml: Path, mode15_xml: Path) -> dict[str, Any]:
  from mjlab.scripts.g1_tracking_phase1_velocity_actuator_contract import (
    _expected_mjlab_effort_limits,
  )

  expected = _expected_mjlab_effort_limits(mjlab_g1_xml=mode15_xml)
  tree = ET.parse(scene_xml)
  root = tree.getroot()
  changed: list[dict[str, Any]] = []
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
    with scene_xml.open("a", encoding="utf-8") as handle:
      handle.write("\n")
  return {"changed": changed}


def _apply_official_joint_passive_defaults(scene_xml: Path) -> dict[str, Any]:
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
  with scene_xml.open("a", encoding="utf-8") as handle:
    handle.write("\n")
  return {
    "old": old_attrib,
    "new": {"damping": "0.05", "armature": "0.01", "frictionloss": "0.2"},
  }


def _inject_sim_telemetry(bridge_path: Path) -> None:
  text = bridge_path.read_text(encoding="utf-8")
  if "SIM_TELEMETRY" in text:
    return
  if "#include <chrono>" not in text:
    text = text.replace(
      "#include <iostream>\n",
      "#include <chrono>\n#include <iostream>\n",
    )
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
                      << std::endl;
        }
"""
  if needle not in text:
    raise UnitreeSim2SimError(f"did not find bridge run guard in {bridge_path}")
  bridge_path.write_text(text.replace(needle, replacement, 1), encoding="utf-8")
