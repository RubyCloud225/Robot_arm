from pathlib import Path

import mujoco
import pytest

from robot_arm_sim.simulation import build_scene

DEMO_XML = Path(__file__).parents[1] / "src/robot_arm_sim/assets/demo.xml"
ROBOT_STUB = """\
<mujoco>
  <worldbody>
    <body name="panda_base">
      <geom type="box" pos="0 0 0.05" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="home"/>
  </keyframe>
</mujoco>
"""


def test_gravity_is_earth_like_and_points_down() -> None:
    model = mujoco.MjModel.from_xml_path(str(DEMO_XML))

    assert model.opt.gravity.tolist() == pytest.approx([0.0, 0.0, -9.81])


def test_scene_uses_radians_and_si_scale() -> None:
    model = mujoco.MjModel.from_xml_path(str(DEMO_XML))
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ball_geom")
    table_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")

    # Ball: 4 cm radius sphere of 0.1 kg; table top: 0.9 m x 0.7 m footprint.
    assert model.geom_size[ball_id][0] == pytest.approx(0.04)
    assert model.body_mass[model.geom_bodyid[ball_id]] == pytest.approx(0.1)
    assert model.geom_size[table_id][:2].tolist() == pytest.approx([0.45, 0.35])


def test_robot_scene_keeps_gravity(tmp_path: Path) -> None:
    robot_path = tmp_path / "panda.xml"
    robot_path.write_text(ROBOT_STUB, encoding="utf-8")

    model, _ = build_scene(robot_path)

    assert model.opt.gravity.tolist() == pytest.approx([0.0, 0.0, -9.81])


def test_free_fall_acceleration_matches_gravity() -> None:
    model = mujoco.MjModel.from_xml_path(str(DEMO_XML))
    data = mujoco.MjData(model)
    ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    start_height = data.qpos[2]

    # Ball is ~8 cm above the table, so it falls freely for ~0.128 s; stop at 0.1 s.
    duration = 0.1
    for _ in range(round(duration / model.opt.timestep)):
        mujoco.mj_step(model, data)

    mujoco.mj_forward(model, data)
    expected_drop = 0.5 * 9.81 * duration**2
    assert start_height - data.xpos[ball_id][2] == pytest.approx(
        expected_drop, rel=0.05
    )
    assert data.qvel[2] == pytest.approx(-9.81 * duration, rel=0.02)
