"""Reset clearance and a goal marker that cannot alter physical motion."""

from dataclasses import replace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from robot_arm_sim.config import apply_config, load_config
from robot_arm_sim.simulation import ASSET_DIR, build_scene, validate_ball_spawn


@pytest.fixture(params=["demo", "composed"])
def scene(request, tmp_path):
    if request.param == "demo":
        model = mujoco.MjModel.from_xml_path(str(ASSET_DIR / "demo.xml"))
        data = mujoco.MjData(model)
    else:
        path = tmp_path / "robot.xml"
        path.write_text('''<mujoco>
          <worldbody><body name="base">
            <geom name="robot_base_geom" type="box" pos="0 0 0.05"
                  size="0.05 0.05 0.05"/>
          </body></worldbody>
          <keyframe><key name="home"/></keyframe>
        </mujoco>''')
        model, data = build_scene(path)
    apply_config(model, data, load_config())
    return model, data


def test_default_spawn_has_clearance_and_settles_without_explosive_correction(scene):
    model, data = scene
    ball = model.geom("ball_geom").id
    table = model.geom("table_top").id
    assert data.ncon == 0
    assert mujoco.mj_geomDistance(model, data, ball, table, 1, None) == pytest.approx(0.08)
    assert data.qvel == pytest.approx(np.zeros(model.nv))
    dof = model.joint("ball_free").dofadr[0]
    peak_speed = 0.0
    for _ in range(1000):
        mujoco.mj_step(model, data)
        peak_speed = max(peak_speed, np.linalg.norm(data.qvel[dof:dof + 3]))
        assert np.isfinite(data.qpos).all()
        assert np.isfinite(data.qvel).all()
    mujoco.mj_forward(model, data)
    # Falling through 8 cm predicts ~1.25 m/s at impact; allow solver transients.
    assert peak_speed < 1.5
    assert data.body("ball").xpos == pytest.approx([0, 0, 0.54], abs=0.001)
    assert data.qvel[dof:dof + 6] == pytest.approx(np.zeros(6), abs=0.001)


@pytest.mark.parametrize("height", [0.54, 0.540001, 0.62])
def test_touching_or_clear_spawn_is_accepted(scene, height):
    model, data = scene
    apply_config(model, data, replace(load_config(), ball_spawn=(0, 0, height)))
    validate_ball_spawn(model, data)


@pytest.mark.parametrize("position, support", [
    ((0, 0, 0.539), "table_top"),
    ((0, 0, 0.475), "table_top"),
    ((0.8, 0, 0.039), "floor"),
    ((-0.39, -0.29, 0.225), "table_leg_front_left"),
    ((0.39, -0.29, 0.225), "table_leg_front_right"),
    ((-0.39, 0.29, 0.225), "table_leg_back_left"),
    ((0.39, 0.29, 0.225), "table_leg_back_right"),
])
def test_overlapping_configured_spawn_is_rejected(scene, position, support):
    model, data = scene
    with pytest.raises(ValueError, match=f"Ball spawn overlaps {support}"):
        apply_config(model, data, replace(load_config(), ball_spawn=position))


def test_spawn_inside_robot_is_rejected(tmp_path):
    path = tmp_path / "robot.xml"
    path.write_text('''<mujoco>
      <worldbody><body name="base">
        <geom name="robot_base_geom" type="box" pos="0 0 0.05"
              size="0.05 0.05 0.05"/>
      </body></worldbody>
      <keyframe><key name="home"/></keyframe>
    </mujoco>''')
    model, data = build_scene(path)
    with pytest.raises(ValueError, match="Ball spawn overlaps robot_base_geom"):
        apply_config(model, data, replace(load_config(), ball_spawn=(-0.65, 0, 0.55)))


def test_target_is_visible_fixed_site_without_collision_geometry(scene):
    model, data = scene
    target = model.body("target").id
    site = model.site("target_marker").id
    assert model.site_bodyid[site] == target
    assert model.site_type[site] == mujoco.mjtGeom.mjGEOM_CYLINDER
    assert model.site_size[site, :2] == pytest.approx([0.06, 0.002])
    assert data.site_xpos[site] == pytest.approx([0.25, 0, 0.502])
    assert model.site_rgba[site, 1] > model.site_rgba[site, 0]  # Distinct green.
    assert 0 < model.site_rgba[site, 3] < 1
    assert model.body_jntnum[target] == 0
    assert model.body_geomnum[target] == 0
    assert model.body_parentid[target] == 0


def test_target_tracks_configured_position(scene):
    model, data = scene
    position = (0.1, -0.15, 0.502)
    apply_config(model, data, replace(load_config(), target_position=position))
    assert data.site("target_marker").xpos == pytest.approx(position)


@pytest.mark.parametrize("position", [(0, 0, 0.62), (0, 0, 0.54), (0, 0, 0.502)])
def test_target_overlap_cannot_change_ball_trajectory(position):
    # Compare with the marker body removed entirely. Place the visual marker
    # at the spawn, resting centre and tabletop so the ball crosses it.
    xml = ET.parse(ASSET_DIR / "demo.xml").getroot()
    world = xml.find("worldbody")
    target = world.find("body[@name='target']")
    target.set("pos", " ".join(map(str, position)))
    marked = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    world.remove(target)
    unmarked = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    marked_data, unmarked_data = mujoco.MjData(marked), mujoco.MjData(unmarked)
    assert marked.neq == unmarked.neq == 0

    for _ in range(1000):
        mujoco.mj_step(marked, marked_data)
        mujoco.mj_step(unmarked, unmarked_data)
        np.testing.assert_allclose(marked_data.qpos, unmarked_data.qpos, atol=1e-12, rtol=0)
        np.testing.assert_allclose(marked_data.qvel, unmarked_data.qvel, atol=1e-12, rtol=0)
        assert marked_data.ncon == unmarked_data.ncon


def test_composition_rejects_robot_overlapping_default_ball_spawn(tmp_path):
    path = tmp_path / "overlapping_robot.xml"
    path.write_text('''<mujoco>
      <worldbody><body name="base">
        <geom name="robot_overlap" type="sphere" size="0.02" pos="0.65 0 0.12"/>
      </body></worldbody>
      <keyframe><key name="home"/></keyframe>
    </mujoco>''')
    with pytest.raises(ValueError, match="Ball spawn overlaps robot_overlap"):
        build_scene(path)
