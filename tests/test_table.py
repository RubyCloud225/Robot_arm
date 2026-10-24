from pathlib import Path

import mujoco
import pytest


SCENE = Path(__file__).parents[1] / "src/robot_arm_sim/assets/demo.xml"
LEGS = [
    "table_leg_front_left",
    "table_leg_front_right",
    "table_leg_back_left",
    "table_leg_back_right",
]


def test_table_is_fixed_and_supports_ball() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    table = model.body("table").id
    top = model.geom("table_top").id
    ball = model.geom("ball_geom").id
    assert model.body_jntnum[table] == 0
    assert model.body_parentid[table] == 0
    initial_position = model.body_pos[table].copy()

    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    assert data.xpos[table] == pytest.approx(initial_position)
    assert data.geom_xpos[top][2] + model.geom_size[top][2] == pytest.approx(0.5)
    assert data.geom_xpos[ball][2] == pytest.approx(0.54, abs=0.001)
    assert any({c.geom1, c.geom2} == {top, ball} for c in data.contact)


@pytest.mark.parametrize("leg_name", LEGS)
def test_leg_contacts_ball_below_tabletop(leg_name: str) -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    leg = model.geom(leg_name).id
    ball = model.geom("ball_geom").id
    ball_joint = model.joint("ball_free").qposadr[0]
    x, y, _ = data.geom_xpos[leg]
    # A slight overlap at mid-leg height exercises real narrow-phase contacts.
    data.qpos[ball_joint:ball_joint + 3] = [x + 0.069, y, 0.225]
    mujoco.mj_forward(model, data)

    assert any({c.geom1, c.geom2} == {leg, ball} for c in data.contact)


def test_space_under_table_is_open() -> None:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    ball_joint = model.joint("ball_free").qposadr[0]
    data.qpos[ball_joint:ball_joint + 3] = [0, 0, 0.225]
    mujoco.mj_forward(model, data)

    assert data.ncon == 0


@pytest.fixture
def table_scene() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def place_ball(model: mujoco.MjModel, data: mujoco.MjData, position) -> None:
    address = model.joint("ball_free").qposadr[0]
    data.qpos[address:address + 3] = position
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


def has_ball_contact(
    model: mujoco.MjModel, data: mujoco.MjData, geom_name: str
) -> bool:
    pair = {model.geom("ball_geom").id, model.geom(geom_name).id}
    return any({c.geom1, c.geom2} == pair for c in data.contact)


@pytest.mark.parametrize("gap", [-0.001, 0.001], ids=["overlap", "clearance"])
@pytest.mark.parametrize(
    "surface, normal",
    [
        ((0.45, 0, 0.475), (1, 0, 0)),
        ((-0.45, 0, 0.475), (-1, 0, 0)),
        ((0, 0.35, 0.475), (0, 1, 0)),
        ((0, -0.35, 0.475), (0, -1, 0)),
        ((0, 0, 0.45), (0, 0, -1)),
        # Diagonal approaches exercise the actual corner rather than its AABB.
        ((0.45, 0.35, 0.5), (1 / 3**0.5,) * 3),
        ((-0.45, 0.35, 0.5), (-1 / 3**0.5, 1 / 3**0.5, 1 / 3**0.5)),
        ((0.45, -0.35, 0.5), (1 / 3**0.5, -1 / 3**0.5, 1 / 3**0.5)),
        ((-0.45, -0.35, 0.5), (-1 / 3**0.5, -1 / 3**0.5, 1 / 3**0.5)),
    ],
    ids=["right", "left", "back", "front", "underside", "corner-ne", "corner-nw", "corner-se", "corner-sw"],
)
def test_tabletop_contact_boundary(table_scene, surface, normal, gap):
    model, data = table_scene
    radius = model.geom("ball_geom").size[0]
    position = [s + n * (radius + gap) for s, n in zip(surface, normal)]
    place_ball(model, data, position)

    assert has_ball_contact(model, data, "table_top") == (gap < 0)


@pytest.mark.parametrize("leg_name", LEGS)
@pytest.mark.parametrize("axis, sign", [(0, -1), (0, 1), (1, -1), (1, 1)])
@pytest.mark.parametrize("gap", [-0.001, 0.001], ids=["overlap", "clearance"])
def test_leg_contact_boundaries(table_scene, leg_name, axis, sign, gap):
    model, data = table_scene
    leg = model.geom(leg_name).id
    position = data.geom_xpos[leg].copy()
    radius = model.geom("ball_geom").size[0]
    position[axis] += sign * (model.geom_size[leg, axis] + radius + gap)
    place_ball(model, data, position)

    assert has_ball_contact(model, data, leg_name) == (gap < 0)


@pytest.mark.parametrize(
    "x, y",
    [(0.44, 0), (-0.44, 0), (0, 0.34), (0, -0.34),
     (0.44, 0.34), (-0.44, -0.34), (-0.44, 0.34), (0.44, -0.34)],
)
def test_ball_settles_near_table_edges(table_scene, x, y):
    model, data = table_scene
    place_ball(model, data, [x, y, 0.62])
    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)

    assert data.xpos[model.body("ball").id] == pytest.approx([x, y, 0.54], abs=0.001)
    assert has_ball_contact(model, data, "table_top")


@pytest.mark.parametrize("x, y", [(0.51, 0), (-0.51, 0), (0, 0.41), (0, -0.41)])
def test_ball_outside_table_falls_to_floor(table_scene, x, y):
    model, data = table_scene
    place_ball(model, data, [x, y, 0.62])
    touched_table = False
    for _ in range(1000):
        mujoco.mj_step(model, data)
        touched_table |= any(has_ball_contact(model, data, name)
                             for name in ["table_top", *LEGS])
    mujoco.mj_forward(model, data)

    assert not touched_table
    assert data.xpos[model.body("ball").id][2] == pytest.approx(0.04, abs=0.001)
    assert has_ball_contact(model, data, "floor")
