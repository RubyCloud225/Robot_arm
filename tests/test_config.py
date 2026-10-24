from pathlib import Path
import shutil

import mujoco
import pytest

from robot_arm_sim.config import apply_config, load_config


def _write_config(
    directory: Path,
    *,
    timestep: str = "0.002",
    period: str = "0.04",
    model_path: str = '"scene.xml"',
    joint_pose: str = "{}",
    gripper_command: str = "{}",
) -> Path:
    source_model = Path(__file__).parents[1] / "src/robot_arm_sim/assets/demo.xml"
    shutil.copy(source_model, directory / "scene.xml")
    config_path = directory / "custom.toml"
    config_path.write_text(
        f"""
[simulation]
model_path = {model_path}
seed = 7
timestep = {timestep}
controller_period = {period}

[robot]
default_joint_pose = {joint_pose}
default_gripper_command = {gripper_command}

[task]
ball_spawn = [0.1, 0.0, 0.7]
target_position = [0.2, 0.0, 0.502]
""",
        encoding="utf-8",
    )
    return config_path


def _write_actuated_model(directory: Path) -> None:
    (directory / "scene.xml").write_text(
        """
<mujoco>
  <worldbody>
    <body name="finger">
      <joint name="finger_joint" type="slide" axis="1 0 0" range="0 1"/>
      <geom type="box" size="0.02 0.02 0.02"/>
    </body>
    <body name="ball" pos="0 0 0.62">
      <freejoint name="ball_free"/>
      <geom type="sphere" size="0.04" mass="0.1"/>
    </body>
    <body name="target" pos="0.2 0 0.502">
      <geom type="cylinder" size="0.06 0.002"/>
    </body>
  </worldbody>
  <actuator>
    <position name="gripper" joint="finger_joint" kp="1"
              ctrllimited="true" ctrlrange="0 1"/>
  </actuator>
</mujoco>
""",
        encoding="utf-8",
    )


def test_default_config_resolves_packaged_model_and_values() -> None:
    config = load_config()

    assert config.model_path.is_file()
    assert config.model_path.name == "demo.xml"
    assert config.timestep == 0.002
    assert config.controller_period == 0.04
    assert config.seed == 0


@pytest.mark.parametrize(
    ("timestep", "period", "message"),
    [
        ("0", "0.04", "timestep must be greater than zero"),
        ("true", "0.04", "timestep must be a number"),
        ("0.002", "0.031", "integer multiple"),
    ],
)
def test_load_config_rejects_invalid_timing(
    tmp_path: Path, timestep: str, period: str, message: str
) -> None:
    path = _write_config(tmp_path, timestep=timestep, period=period)

    with pytest.raises(ValueError, match=message):
        load_config(path)


def test_load_config_resolves_relative_model_path(tmp_path: Path) -> None:
    path = _write_config(tmp_path)

    assert load_config(path).model_path == (tmp_path / "scene.xml").resolve()


def test_load_config_rejects_absolute_model_path(tmp_path: Path) -> None:
    path = _write_config(tmp_path, model_path=f'"{tmp_path / "scene.xml"}"')

    with pytest.raises(ValueError, match="must be relative"):
        load_config(path)


def test_load_config_rejects_missing_model(tmp_path: Path) -> None:
    path = _write_config(tmp_path, model_path='"missing.xml"')

    with pytest.raises(ValueError, match="does not exist"):
        load_config(path)


def test_apply_config_applies_timestep_and_scene_positions() -> None:
    config = load_config()
    model = mujoco.MjModel.from_xml_path(str(config.model_path))
    data = mujoco.MjData(model)

    apply_config(model, data, config)

    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    target_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
    assert model.opt.timestep == config.timestep
    assert data.xpos[ball_id].tolist() == pytest.approx(config.ball_spawn)
    assert model.body_pos[target_id].tolist() == pytest.approx(config.target_position)


@pytest.mark.parametrize(
    ("joint_pose", "gripper_command", "message"),
    [
        ('{"finger_joint" = 1.5}', "{}", "outside its range"),
        ("{}", '{"gripper" = 1.5}', "outside its range"),
        ('{"unknown_joint" = 0.5}', "{}", "joint does not exist"),
        ("{}", '{"unknown_actuator" = 0.5}', "actuator does not exist"),
    ],
)
def test_apply_config_checks_joint_and_actuator_limits(
    tmp_path: Path, joint_pose: str, gripper_command: str, message: str
) -> None:
    config_path = _write_config(
        tmp_path,
        joint_pose=joint_pose,
        gripper_command=gripper_command,
    )
    _write_actuated_model(tmp_path)
    config = load_config(config_path)
    model = mujoco.MjModel.from_xml_path(str(config.model_path))
    data = mujoco.MjData(model)

    with pytest.raises(ValueError, match=message):
        apply_config(model, data, config)
