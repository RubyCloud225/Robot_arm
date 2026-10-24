"""Scene framing and explicit physics choices survive model composition."""

import mujoco
import numpy as np
import pytest

from robot_arm_sim.config import apply_config, load_config
from robot_arm_sim.simulation import ASSET_DIR, build_scene


@pytest.fixture(params=["demo", "composed"])
def scene(request, tmp_path):
    if request.param == "demo":
        model = mujoco.MjModel.from_xml_path(str(ASSET_DIR / "demo.xml"))
        data = mujoco.MjData(model)
    else:
        robot = tmp_path / "robot.xml"
        robot.write_text('''<mujoco>
          <option integrator="implicitfast"/>
          <worldbody><body name="base"/></worldbody>
          <keyframe><key name="home"/></keyframe>
        </mujoco>''')
        model, data = build_scene(robot)
    apply_config(model, data, load_config())
    mujoco.mj_forward(model, data)
    return model, data


def test_explicit_physics_options_survive_configuration(scene):
    model, _ = scene
    assert model.opt.timestep == pytest.approx(0.002)
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert model.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.iterations == 100
    assert model.opt.tolerance == pytest.approx(1e-8)
    assert model.opt.noslip_iterations == 0
    assert not model.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CONTACT
    ball = model.geom("ball_geom").id
    assert model.geom_solref[ball] == pytest.approx([0.02, 1])
    assert model.geom_solimp[ball, :3] == pytest.approx([0.9, 0.95, 0.001])


@pytest.mark.parametrize("camera_name", ["overview", "top_down"])
def test_fixed_cameras_frame_table_ball_goal_and_robot_envelope(scene, camera_name):
    model, data = scene
    camera = model.camera(camera_name).id
    assert model.cam_bodyid[camera] == 0
    assert model.cam_mode[camera] == mujoco.mjtCamLight.mjCAMLIGHT_FIXED
    rotation = data.cam_xmat[camera].reshape(3, 3)
    vertical = np.tan(np.deg2rad(model.cam_fovy[camera]) / 2)
    horizontal = vertical * 4 / 3  # Minimum documented viewport aspect ratio.
    points = [[x, y, z] for x in [-0.45, 0.45] for y in [-0.35, 0.35]
              for z in [0, 0.5]]
    points += [[-0.77, 0, 0], [-0.65, 0, 1.4], [0, 0, 0.62], [0.25, 0, 0.502]]
    for point in points:
        local = rotation.T @ (np.array(point) - data.cam_xpos[camera])
        depth = -local[2]
        assert depth > 0
        assert abs(local[0]) < depth * horizontal
        assert abs(local[1]) < depth * vertical


def test_debug_lights_and_colours_are_distinct(scene):
    model, _ = scene
    key, fill = model.light("key_light").id, model.light("fill_light").id
    assert model.light_castshadow[key]
    assert not model.light_castshadow[fill]
    assert np.all(model.light_diffuse[[key, fill]] > 0)
    assert model.light_bodyid[[key, fill]].tolist() == [0, 0]
    assert model.material("ball_material").rgba[0] > 0.8
    assert model.site("target_marker").rgba[1] > 0.7
    assert model.geom("table_top").rgba[0] > model.geom("table_top").rgba[2]
