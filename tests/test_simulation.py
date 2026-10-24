from pathlib import Path

import mujoco
import pytest

from robot_arm_sim.simulation import build_scene


def test_build_scene_attaches_robot_model(tmp_path: Path) -> None:
    panda_path = tmp_path / "panda.xml"
    panda_path.write_text(
        """\
<mujoco model="panda_stub">
  <worldbody>
    <body name="panda_base">
      <body name="arm" pos="0 0 0.1">
      <joint name="joint1" type="hinge" range="-2 2"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="actuator1" joint="joint1" kp="1"
              ctrllimited="true" ctrlrange="-2 2"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0.5" ctrl="0.25"/>
  </keyframe>
</mujoco>
""",
        encoding="utf-8",
    )

    model, data = build_scene(panda_path)

    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "panda_base")
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint1")
    actuator_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator1"
    )
    assert base_id >= 0
    assert ball_id >= 0
    assert data.qpos[model.jnt_qposadr[joint_id]] == 0.5
    assert data.ctrl[actuator_id] == 0.25
    assert data.xpos[ball_id] == pytest.approx([0.0, 0.0, 0.62])


def test_build_scene_rejects_missing_model(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="Panda model not found"):
        build_scene(tmp_path / "missing.xml")


def test_build_scene_rejects_invalid_xml(tmp_path: Path) -> None:
    invalid_xml = tmp_path / "panda.xml"
    invalid_xml.write_text("<mujoco>", encoding="utf-8")

    with pytest.raises(ValueError):
        build_scene(invalid_xml)


def test_build_scene_requires_home_keyframe(tmp_path: Path) -> None:
    panda_path = tmp_path / "panda.xml"
    panda_path.write_text(
        """\
<mujoco>
  <worldbody>
    <body name="panda_base">
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="missing its 'home' keyframe"):
        build_scene(panda_path)


def test_build_scene_copies_articulated_joint_types_from_home_keyframe(
    tmp_path: Path,
) -> None:
    panda_path = tmp_path / "panda.xml"
    panda_path.write_text(
        """\
<mujoco>
  <worldbody>
    <body name="panda_base">
    <body name="hinge_body" pos="0 0 0.2">
      <joint name="hinge_joint" type="hinge" range="-2 2"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
      <body name="slide_body">
        <joint name="slide_joint" type="slide" axis="1 0 0" range="-1 1"/>
        <geom type="box" size="0.05 0.05 0.05" mass="1"/>
        <body name="ball_body">
          <joint name="ball_joint" type="ball"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </body>
    </body>
    </body>
  </worldbody>
  <keyframe>
    <key name="home"
         qpos="0.2 0.3 0 0 0 1"
         qvel="0.7 0.8 0.1 0.2 0.3"/>
  </keyframe>
</mujoco>
""",
        encoding="utf-8",
    )

    model, data = build_scene(panda_path)

    expected_state = {
        "hinge_joint": ([0.2], [0.7]),
        "slide_joint": ([0.3], [0.8]),
        "ball_joint": ([0.0, 0.0, 0.0, 1.0], [0.1, 0.2, 0.3]),
    }
    for joint_name, (expected_qpos, expected_qvel) in expected_state.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_address = model.jnt_qposadr[joint_id]
        assert data.qpos[
            qpos_address : qpos_address + len(expected_qpos)
        ] == pytest.approx(
            expected_qpos
        )
        qvel_address = model.jnt_dofadr[joint_id]
        assert data.qvel[
            qvel_address : qvel_address + len(expected_qvel)
        ] == pytest.approx(expected_qvel)
