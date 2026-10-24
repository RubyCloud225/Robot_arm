"""Load and validate simulation configuration."""

import math
from dataclasses import dataclass
from pathlib import Path
import tomllib

import mujoco

from .simulation import validate_ball_spawn

_DEFAULT_CONFIG = Path(__file__).parent / "assets" / "default.toml"


@dataclass(frozen=True)
class SimulationConfig:
    model_path: Path
    seed: int
    timestep: float
    controller_period: float
    default_joint_pose: dict[str, float]
    default_gripper_command: dict[str, float]
    ball_spawn: tuple[float, float, float]
    target_position: tuple[float, float, float]


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    if positive and result <= 0:
        raise ValueError(f"{name} must be greater than zero")
    return result


def _named_values(value: object, name: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a table of names and numeric values")
    result: dict[str, float] = {}
    for item_name, item_value in value.items():
        if not isinstance(item_name, str) or not item_name:
            raise ValueError(f"{name} names must be non-empty strings")
        result[item_name] = _number(item_value, f"{name}.{item_name}")
    return result


def _vector3(value: object, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must contain exactly three numbers")
    return (
        _number(value[0], f"{name}[0]"),
        _number(value[1], f"{name}[1]"),
        _number(value[2], f"{name}[2]"),
    )


def load_config(config_path: str | Path | None = None) -> SimulationConfig:
    """Load TOML configuration; relative model paths use the TOML file's directory."""
    path = Path(config_path) if config_path is not None else _DEFAULT_CONFIG
    path = path.resolve()
    try:
        with path.open("rb") as config_file:
            raw = tomllib.load(config_file)
    except OSError as error:
        raise ValueError(f"Cannot read configuration file {path}: {error}") from error
    except tomllib.TOMLDecodeError as error:
        raise ValueError(f"Invalid TOML in configuration file {path}: {error}") from error

    if not isinstance(raw, dict):
        raise ValueError("Configuration root must be a TOML table")
    simulation = raw.get("simulation")
    robot = raw.get("robot")
    task = raw.get("task")
    if not isinstance(simulation, dict):
        raise ValueError("Configuration must contain a [simulation] table")
    if not isinstance(robot, dict):
        raise ValueError("Configuration must contain a [robot] table")
    if not isinstance(task, dict):
        raise ValueError("Configuration must contain a [task] table")

    model_value = simulation.get("model_path")
    if not isinstance(model_value, str) or not model_value.strip():
        raise ValueError("simulation.model_path must be a non-empty relative path")
    model_relative_path = Path(model_value)
    if model_relative_path.is_absolute():
        raise ValueError("simulation.model_path must be relative to the configuration file")
    model_path = (path.parent / model_relative_path).resolve()
    if not model_path.is_file():
        raise ValueError(f"Configured model file does not exist: {model_path}")

    seed = simulation.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("simulation.seed must be a non-negative integer")

    timestep = _number(simulation.get("timestep"), "simulation.timestep", positive=True)
    controller_period = _number(
        simulation.get("controller_period"),
        "simulation.controller_period",
        positive=True,
    )
    steps_per_control = controller_period / timestep
    if steps_per_control < 1 or not math.isclose(
        steps_per_control, round(steps_per_control), rel_tol=1e-9, abs_tol=1e-9
    ):
        raise ValueError(
            "simulation.controller_period must be an integer multiple of "
            "simulation.timestep"
        )

    return SimulationConfig(
        model_path=model_path,
        seed=seed,
        timestep=timestep,
        controller_period=controller_period,
        default_joint_pose=_named_values(
            robot.get("default_joint_pose"), "robot.default_joint_pose"
        ),
        default_gripper_command=_named_values(
            robot.get("default_gripper_command"),
            "robot.default_gripper_command",
        ),
        ball_spawn=_vector3(task.get("ball_spawn"), "task.ball_spawn"),
        target_position=_vector3(task.get("target_position"), "task.target_position"),
    )


def apply_config(
    model: mujoco.MjModel, data: mujoco.MjData, config: SimulationConfig
) -> None:
    """Apply initial positions and controls, checking configured names and limits."""
    model.opt.timestep = config.timestep
    for joint_name, position in config.default_joint_pose.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"Configured joint does not exist in the model: {joint_name}")
        joint_type = int(model.jnt_type[joint_id])
        if joint_type not in (
            int(mujoco.mjtJoint.mjJNT_HINGE),
            int(mujoco.mjtJoint.mjJNT_SLIDE),
        ):
            raise ValueError(f"Configured joint must be hinge or slide: {joint_name}")
        if model.jnt_limited[joint_id]:
            lower, upper = model.jnt_range[joint_id]
            if not lower <= position <= upper:
                raise ValueError(
                    f"Default position for joint {joint_name!r} is outside "
                    f"its range [{lower}, {upper}]"
                )
        data.qpos[model.jnt_qposadr[joint_id]] = position

    for actuator_id in range(model.nu):
        if model.actuator_ctrllimited[actuator_id]:
            lower, upper = model.actuator_ctrlrange[actuator_id]
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError(f"Actuator {actuator_id} has an invalid control range")

    for actuator_name, command in config.default_gripper_command.items():
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
        )
        if actuator_id < 0:
            raise ValueError(
                f"Configured gripper actuator does not exist in the model: "
                f"{actuator_name}"
            )
        if model.actuator_ctrllimited[actuator_id]:
            lower, upper = model.actuator_ctrlrange[actuator_id]
            if not lower <= command <= upper:
                raise ValueError(
                    f"Default command for actuator {actuator_name!r} is outside "
                    f"its range [{lower}, {upper}]"
                )
        data.ctrl[actuator_id] = command

    for body_name, position in (
        ("ball", config.ball_spawn),
        ("target", config.target_position),
    ):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise ValueError(f"Configured body does not exist in the model: {body_name}")
        model.body_pos[body_id] = position
        if body_name == "ball":
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, "ball_free"
            )
            if (
                joint_id < 0
                or int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE)
            ):
                raise ValueError("The ball body must have a free joint named 'ball_free'")
            qpos_address = model.jnt_qposadr[joint_id]
            data.qpos[qpos_address : qpos_address + 3] = position

    mujoco.mj_forward(model, data)
    validate_ball_spawn(model, data)
