"""Real-Panda collision probes; these verify filters, not grasp success."""

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_arm_sim.simulation import DEFAULT_PANDA_XML, build_scene


@pytest.fixture
def panda_scene():
    path = Path(os.environ.get("ROBOT_ARM_PANDA_XML", str(DEFAULT_PANDA_XML)))
    if not path.is_file():
        pytest.skip("Set ROBOT_ARM_PANDA_XML to check actual Panda contact filters")
    return build_scene(path)


def collision_geoms(model, body_name):
    body = model.body(body_name).id
    return [g for g in range(model.ngeom) if model.geom_bodyid[g] == body
            and (model.geom_contype[g] or model.geom_conaffinity[g])]


def contact_exists(data, first, second):
    return any({c.geom1, c.geom2} == {first, second} for c in data.contact)


def test_robot_collision_masks_allow_scene_contacts_and_visuals_are_disabled(panda_scene):
    model, _ = panda_scene
    supports = [model.geom(name).id for name in [
        "floor", "table_top", "table_leg_front_left", "table_leg_front_right",
        "table_leg_back_left", "table_leg_back_right", "ball_geom", "robot_pedestal",
    ]]
    for body_name in ["hand", "left_finger", "right_finger"]:
        geoms = collision_geoms(model, body_name)
        assert geoms
        for geom in geoms:
            for support in supports:
                assert ((model.geom_contype[geom] & model.geom_conaffinity[support])
                        or (model.geom_contype[support] & model.geom_conaffinity[geom]))
    visual = np.flatnonzero(model.geom_group == 2)
    assert len(visual) > 0
    assert np.all(model.geom_contype[visual] == 0)
    assert np.all(model.geom_conaffinity[visual] == 0)
    for finger in ["left_finger", "right_finger"]:
        assert sum(model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
                   for g in collision_geoms(model, finger)) == 5


@pytest.mark.parametrize("body_name", ["hand", "left_finger", "right_finger"])
def test_ball_contacts_every_hand_and_finger_collider(panda_scene, body_name):
    model, data = panda_scene
    ball = model.geom("ball_geom").id
    qpos = model.joint("ball_free").qposadr[0]
    for geom in collision_geoms(model, body_name):
        # Use the local AABB centre: a mesh's frame origin may lie outside it.
        centre = data.geom_xpos[geom] + data.geom_xmat[geom].reshape(3, 3) @ model.geom_aabb[geom, :3]
        data.qpos[qpos:qpos + 3] = centre
        mujoco.mj_forward(model, data)
        assert contact_exists(data, geom, ball), (body_name, geom)


@pytest.mark.parametrize("body_name", ["hand", "left_finger", "right_finger"])
@pytest.mark.parametrize("support_name", ["floor", "table_top"])
def test_supports_contact_hand_and_fingers(panda_scene, body_name, support_name):
    model, data = panda_scene
    support = model.geom(support_name).id
    mount = model.body("robot_mount").id
    original_mount = model.body_pos[mount].copy()
    for geom in collision_geoms(model, body_name):
        model.body_pos[mount] = original_mount
        mujoco.mj_forward(model, data)
        centre = data.geom_xpos[geom] + data.geom_xmat[geom].reshape(3, 3) @ model.geom_aabb[geom, :3]
        # Deliberately shift the robot so this collider meets the support.
        # Forward kinematics updates broad-phase bounds as well as contacts.
        # Never inject contacts or step these deliberately invalid probe poses.
        model.body_pos[mount] += data.geom_xpos[support] - centre
        mujoco.mj_forward(model, data)
        assert contact_exists(data, geom, support), (body_name, geom, support_name)


def test_original_base_link_exclusion_is_preserved(panda_scene):
    model, _ = panda_scene
    link0, link1 = model.body("link0").id, model.body("link1").id
    signature = (min(link0, link1) << 16) + max(link0, link1)
    assert signature in model.exclude_signature
