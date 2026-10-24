"""Command-line runner for the packaged MuJoCo scene."""

import argparse
from pathlib import Path

import mujoco

from .config import apply_config, load_config
from .simulation import build_scene, validate_robot_placement


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the MuJoCo demo scene, optionally with a Panda robot."
    )
    parser.add_argument(
        "--robot",
        action="store_true",
        help="attach the Franka Panda model to the demo scene",
    )
    parser.add_argument(
        "--robot-model",
        type=Path,
        help="path to a locally available Panda panda.xml (used with --robot)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="step the simulation without opening a viewer",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=1000,
        help="number of physics steps in headless mode (default: 1000)",
    )
    parser.add_argument(
        "--config",
        help="path to a TOML configuration file (defaults to the packaged config)",
    )
    args = parser.parse_args(argv)

    if args.robot_model is not None and not args.robot:
        parser.error("--robot-model can only be used with --robot")
    if args.steps <= 0:
        parser.error("--steps must be a positive integer")

    try:
        config = load_config(args.config)
    except ValueError as error:
        parser.error(str(error))

    if args.robot:
        try:
            model, data = build_scene(args.robot_model) if args.robot_model else build_scene()
        except ValueError as error:
            parser.error(str(error))
    else:
        try:
            model = mujoco.MjModel.from_xml_path(str(config.model_path))
        except ValueError as error:
            parser.error(f"Cannot load MuJoCo model {config.model_path}: {error}")
        data = mujoco.MjData(model)

    try:
        apply_config(model, data, config)
        if args.robot:
            validate_robot_placement(model, data)
    except ValueError as error:
        parser.error(str(error))

    if args.headless:
        for _ in range(args.steps):
            mujoco.mj_step(model, data)

        ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
        ball_position = data.xpos[ball_id]
        print(
            f"Completed {args.steps} steps; "
            f"ball position: ({ball_position[0]:.3f}, "
            f"{ball_position[1]:.3f}, {ball_position[2]:.3f}) m"
        )
        return 0

    from mujoco import viewer

    viewer.launch(model, data)
    return 0
