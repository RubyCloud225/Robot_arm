from pathlib import Path

import mujoco
import pytest


SCENE = Path(__file__).parents[1] / "src/robot_arm_sim/assets/demo.xml"


def test_floor_is_fixed_at_world_zero_with_debug_material() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    floor = model.geom("floor").id

    assert model.geom_type[floor] == mujoco.mjtGeom.mjGEOM_PLANE
    assert model.geom_bodyid[floor] == 0
    assert data.geom_xpos[floor] == pytest.approx([0, 0, 0])
    assert data.geom_xmat[floor].reshape(3, 3)[:, 2] == pytest.approx([0, 0, 1])
    assert model.geom_contype[floor] == 1
    assert model.geom_conaffinity[floor] == 1
    assert model.geom_friction[floor] == pytest.approx([1, 0.005, 0.0001])
    assert model.geom_matid[floor] == model.material("floor_material").id
    assert model.texture("floor_checker").id >= 0


@pytest.mark.parametrize("gap", [-0.001, 0.001], ids=["overlap", "clearance"])
def test_floor_contact_boundary(gap: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    address = model.joint("ball_free").qposadr[0]
    data.qpos[address:address + 3] = [0.8, 0, 0.04 + gap]
    mujoco.mj_forward(model, data)
    pair = {model.geom("floor").id, model.geom("ball_geom").id}

    assert any({c.geom1, c.geom2} == pair for c in data.contact) == (gap < 0)


@pytest.mark.parametrize("x, y", [(0.8, 0), (-0.8, 0), (0, 0.8), (0, -0.8),
                                        (2.5, 0), (-2.5, 0), (0, 2.5), (0, -2.5)])
def test_floor_catches_ball_even_beyond_visible_patch(x: float, y: float) -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    address = model.joint("ball_free").qposadr[0]
    data.qpos[address:address + 3] = [x, y, 0.62]
    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    assert data.xpos[model.body("ball").id] == pytest.approx([x, y, 0.04], abs=0.001)
    assert data.qvel == pytest.approx([0] * model.nv, abs=0.001)
    pair = {model.geom("floor").id, model.geom("ball_geom").id}
    assert any({c.geom1, c.geom2} == pair for c in data.contact)
