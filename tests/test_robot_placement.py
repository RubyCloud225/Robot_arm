import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_arm_sim.cli import main
from robot_arm_sim.simulation import (
    ASSET_DIR,
    DEFAULT_PANDA_XML,
    ROBOT_BASE_POSITION,
    build_scene,
    validate_robot_placement,
)


def write_robot(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "robot.xml"
    path.write_text(
        f'<mujoco><worldbody>{body}</worldbody>'
        '<keyframe><key name="home"/></keyframe></mujoco>'
    )
    return path


def test_base_and_pedestal_stay_rigid(tmp_path: Path) -> None:
    path = write_robot(tmp_path, '''
      <body name="base">
        <geom type="box" pos="0 0 0.05" size="0.05 0.05 0.05"/>
      </body>''')
    model, data = build_scene(path)
    base = model.body("base").id
    assert model.body_weldid[base] == 0
    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    assert data.xpos[base] == pytest.approx(ROBOT_BASE_POSITION)
    assert data.xmat[base] == pytest.approx(np.eye(3).ravel())
    assert data.geom_xpos[model.geom("robot_pedestal").id] == pytest.approx([-0.65, 0, 0.25])


@pytest.mark.parametrize("joint", ['<freejoint/>', '<joint type="hinge"/>'])
def test_movable_base_is_rejected(tmp_path: Path, joint: str) -> None:
    path = write_robot(tmp_path, f'''
      <body name="base">{joint}
        <geom type="sphere" size="0.05" mass="1"/>
      </body>''')
    with pytest.raises(ValueError, match="fixed body"):
        build_scene(path)


@pytest.mark.parametrize("position, support", [
    ("0.65 0 -0.025", "table_top"),
    ("0.26 -0.29 -0.275", "table_leg_front_left"),
    ("1.5 0 -0.5", "floor"),
    ("0 0 -0.1", "robot_pedestal"),
])
def test_fixed_geometry_penetration_is_rejected(tmp_path, position, support):
    # World-welded pairs are filtered from contacts, so this also checks that
    # validation uses actual geometry distances rather than only data.contact.
    path = write_robot(tmp_path, f'''
      <body name="base">
        <geom name="bad_geometry" type="sphere" pos="{position}" size="0.02"/>
      </body>''')
    with pytest.raises(ValueError, match=f"overlaps {support}"):
        build_scene(path)


def test_visual_only_geometry_does_not_block_reset(tmp_path: Path) -> None:
    path = write_robot(tmp_path, '''
      <body name="base">
        <geom type="sphere" pos="0.65 0 -0.025" size="0.02"
              contype="0" conaffinity="0"/>
      </body>''')
    build_scene(path)


def test_changed_reset_pose_is_checked(tmp_path: Path, capsys) -> None:
    path = write_robot(tmp_path, '''
      <body name="base">
        <body name="arm" pos="0.75 0 0.1">
          <joint name="slide" type="slide" axis="0 0 1" range="-0.2 0.2"/>
          <geom name="arm_collision" type="sphere" size="0.02" mass="1"/>
        </body>
      </body>''')
    model, data = build_scene(path)
    data.qpos[model.joint("slide").qposadr[0]] = -0.125
    mujoco.mj_forward(model, data)
    with pytest.raises(ValueError, match="overlaps table_top"):
        validate_robot_placement(model, data)

    # Exercise the actual CLI path: the imported home is valid but the TOML
    # override moves the link into the tabletop, so stepping must not start.
    (tmp_path / "demo.xml").write_text((ASSET_DIR / "demo.xml").read_text())
    config = tmp_path / "reset.toml"
    config.write_text((ASSET_DIR / "default.toml").read_text().replace(
        "default_joint_pose = {}", 'default_joint_pose = {slide = -0.125}'
    ))
    with pytest.raises(SystemExit, match="2"):
        main(["--robot", "--robot-model", str(path), "--config", str(config),
              "--headless"])
    assert "overlaps table_top" in capsys.readouterr().err


@pytest.fixture
def panda_scene():
    path = Path(os.environ.get("ROBOT_ARM_PANDA_XML", str(DEFAULT_PANDA_XML)))
    if not path.is_file():
        pytest.skip("Set ROBOT_ARM_PANDA_XML to run real Panda clearance checks")
    return build_scene(path)


def test_panda_home_and_nearby_sweeps_clear_supports(panda_scene):
    model, data = panda_scene
    home = data.qpos.copy()
    # Bounded sweeps around home, one joint at a time. These samples do not
    # certify arbitrary simultaneous joint motion or the full workspace.
    for name, lower, upper in [
        ("joint1", -0.5, 0.5), ("joint2", -0.4, 0.4),
        ("joint4", -1.9, -1.2), ("joint6", 1.3, 1.9),
    ]:
        for angle in np.linspace(lower, upper, 21):
            data.qpos[:] = home
            data.qpos[model.joint(name).qposadr[0]] = angle
            mujoco.mj_forward(model, data)
            validate_robot_placement(model, data)
    data.qpos[:] = home
    mujoco.mj_forward(model, data)
    assert data.xpos[model.body("link0").id] == pytest.approx(ROBOT_BASE_POSITION)


def test_panda_base_stays_fixed_and_home_remains_clear(panda_scene):
    model, data = panda_scene
    base = model.body("link0").id
    initial_position = data.xpos[base].copy()
    initial_orientation = data.xmat[base].copy()
    for _ in range(1000):
        mujoco.mj_step(model, data)
        validate_robot_placement(model, data)
    mujoco.mj_forward(model, data)
    assert data.xpos[base] == pytest.approx(initial_position)
    assert data.xmat[base] == pytest.approx(initial_orientation)
    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()
