from __future__ import annotations

import argparse
from pathlib import Path

import mjlab.tasks  # noqa: F401
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.rl.checkpoint_restore import load_rsl_rl_runtime_configs
from mjlab.rl.exporter_utils import attach_metadata_to_onnx, get_base_metadata
from mjlab.tasks.registry import load_runner_cls
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.utils.torch import configure_torch_backends


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
    description="Export an RSL-RL actor-only ONNX for deploy stacks."
  )
  parser.add_argument("task_id")
  parser.add_argument("--checkpoint-file", required=True)
  parser.add_argument("--motion-file")
  parser.add_argument("--output-file", required=True)
  parser.add_argument("--device", default="cpu")
  parser.add_argument(
    "--use-train-env-config",
    action="store_true",
    help="Restore checkpoint params against the training env config instead of play config.",
  )
  parser.add_argument(
    "--metadata-run-path",
    help="Optional run_path metadata override. Defaults to restored run dir name.",
  )
  return parser.parse_args()


def resolve_metadata_run_path(
  *,
  checkpoint_file: str,
  run_dir: Path | None,
  metadata_run_path: str | None,
) -> str:
  if metadata_run_path:
    return metadata_run_path
  if run_dir is not None:
    return run_dir.name
  return Path(checkpoint_file).expanduser().resolve().parent.name


def attach_actor_export_metadata(
  *,
  onnx_path: Path,
  env: ManagerBasedRlEnv,
  checkpoint_file: str,
  run_dir: Path | None,
  metadata_run_path: str | None,
) -> str:
  run_path = resolve_metadata_run_path(
    checkpoint_file=checkpoint_file,
    run_dir=run_dir,
    metadata_run_path=metadata_run_path,
  )
  metadata = get_base_metadata(env, run_path)
  attach_metadata_to_onnx(str(onnx_path), metadata)
  return run_path


def main() -> None:
  args = parse_args()
  configure_torch_backends()

  env_cfg, agent_cfg, restored, run_dir = load_rsl_rl_runtime_configs(
    args.task_id,
    checkpoint_file=args.checkpoint_file,
    play=not args.use_train_env_config,
  )
  if restored and run_dir is not None:
    print(f"[INFO] Restored RSL-RL config from {run_dir / 'params'}")

  if args.motion_file is not None:
    motion_cmd = env_cfg.commands.get("motion")
    if not isinstance(motion_cmd, MotionCommandCfg):
      raise TypeError("Expected tracking task with a MotionCommandCfg named 'motion'.")
    motion_cmd.motion_file = args.motion_file

  env_cfg.scene.num_envs = 1
  env = ManagerBasedRlEnv(cfg=env_cfg, device=args.device)
  wrapped_env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.get("clip_actions"))

  runner_cls = load_runner_cls(args.task_id) or MjlabOnPolicyRunner
  runner = runner_cls(wrapped_env, agent_cfg, device=args.device)
  runner.load(
    args.checkpoint_file,
    load_cfg={"actor": True},
    strict=True,
    map_location=args.device,
  )

  output = Path(args.output_file).expanduser().resolve()
  MjlabOnPolicyRunner.export_policy_to_onnx(
    runner,
    str(output.parent),
    output.name,
  )
  print(f"[INFO] Exported actor-only ONNX: {output}")
  metadata_run_path = attach_actor_export_metadata(
    onnx_path=output,
    env=env,
    checkpoint_file=args.checkpoint_file,
    run_dir=run_dir,
    metadata_run_path=args.metadata_run_path,
  )
  print(f"[INFO] Attached ONNX deploy metadata: run_path={metadata_run_path}")
  env.close()


if __name__ == "__main__":
  main()
