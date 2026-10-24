"""Physical invariants for the uniform solid task sphere."""

import mujoco
import numpy as np
import pytest

from robot_arm_sim.simulation import ASSET_DIR, build_scene


@pytest.fixture(params=["demo", "composed"])
def ball_scene(request, tmp_path):
    if request.param == "demo":
        model = mujoco.MjModel.from_xml_path(str(ASSET_DIR / "demo.xml"))
        data = mujoco.MjData(model)
    else:
        # Composition must preserve the object's mass and contact properties.
        robot = tmp_path / "robot.xml"
        robot.write_text('''<mujoco>
          <worldbody><body name="base">
            <geom type="box" pos="0 0 0.05" size="0.05 0.05 0.05"/>
          </body></worldbody>
          <keyframe><key name="home"/></keyframe>
        </mujoco>''')
        model, data = build_scene(robot)
    mujoco.mj_forward(model, data)
    return model, data


def test_ball_has_uniform_solid_sphere_inertia(ball_scene):
    model, _ = ball_scene
    body = model.body("ball").id
    geom = model.geom("ball_geom").id
    radius, mass = 0.04, 0.1

    assert model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_SPHERE
    assert model.geom_size[geom, 0] == pytest.approx(radius)
    assert model.body_mass[body] == pytest.approx(mass)
    assert model.body_ipos[body] == pytest.approx([0, 0, 0])
    assert model.body_inertia[body] == pytest.approx([2 / 5 * mass * radius**2] * 3)
    assert model.geom_matid[geom] == model.material("ball_material").id


@pytest.mark.parametrize("axis", range(3), ids=["x", "y", "z"])
def test_ball_acceleration_matches_force_and_mass(ball_scene, axis):
    model, data = ball_scene
    body = model.body("ball").id
    dof = model.joint("ball_free").dofadr[0]
    force = 0.2
    assert data.ncon == 0
    data.xfrc_applied[body, axis] = force
    mujoco.mj_forward(model, data)
    expected = model.opt.gravity.copy()
    expected[axis] += force / 0.1

    assert data.qacc[dof:dof + 3] == pytest.approx(expected)


@pytest.mark.parametrize("axis", range(3), ids=["x", "y", "z"])
def test_ball_angular_acceleration_matches_torque_and_inertia(ball_scene, axis):
    model, data = ball_scene
    body = model.body("ball").id
    dof = model.joint("ball_free").dofadr[0]
    torque = 0.001
    assert data.ncon == 0
    data.xfrc_applied[body, 3 + axis] = torque
    mujoco.mj_forward(model, data)
    expected = np.zeros(3)
    expected[axis] = torque / (2 / 5 * 0.1 * 0.04**2)

    assert data.qacc[dof + 3:dof + 6] == pytest.approx(expected)


def test_ball_uses_specified_sliding_friction_in_table_contact(ball_scene):
    model, data = ball_scene
    ball = model.geom("ball_geom").id
    top = model.geom("table_top").id
    assert model.geom_contype[ball] == 1
    assert model.geom_conaffinity[ball] == 1
    assert model.geom_condim[ball] == 3
    assert model.geom_friction[ball] == pytest.approx([1, 0.005, 0.0001])

    for _ in range(1000):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    contacts = [c for c in data.contact if {c.geom1, c.geom2} == {ball, top}]
    assert contacts
    for contact in contacts:
        assert contact.dim == 3
        assert contact.friction[:2] == pytest.approx([1, 1])


def test_ball_free_joint_has_six_unconstrained_velocity_dofs(ball_scene):
    model, _ = ball_scene
    body = model.body("ball").id
    joint = model.joint("ball_free").id
    dof = model.jnt_dofadr[joint]

    assert model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE
    assert model.jnt_bodyid[joint] == body
    assert model.body_parentid[body] == 0
    assert model.body_jntnum[body] == 1
    assert model.body_mocapid[body] == -1
    assert not model.jnt_limited[joint]
    assert model.dof_damping[dof:dof + 6] == pytest.approx([0] * 6)
    assert model.dof_frictionloss[dof:dof + 6] == pytest.approx([0] * 6)
    assert model.dof_armature[dof:dof + 6] == pytest.approx([0] * 6)
    # Both fixture models intentionally have no equalities, tendons or
    # actuators that could secretly attach or drive the ball.
    assert model.neq == model.ntendon == model.nu == 0


@pytest.mark.parametrize("velocity", [
    (0, 0, 0), (0.3, -0.2, 0), (0.1, 0.2, 0.3), (-0.1, -0.2, -0.3),
], ids=["rest", "horizontal", "upward", "downward"])
def test_free_ball_follows_ballistic_motion(ball_scene, velocity):
    model, data = ball_scene
    joint = model.joint("ball_free").id
    qpos = model.jnt_qposadr[joint]
    dof = model.jnt_dofadr[joint]
    initial_position = data.qpos[qpos:qpos + 3].copy()
    initial_velocity = np.array(velocity)
    data.qvel[dof:dof + 3] = initial_velocity
    steps = 25
    duration = steps * model.opt.timestep

    for _ in range(steps):
        mujoco.mj_step(model, data)
        assert data.ncon == 0
    mujoco.mj_forward(model, data)

    # Compare with continuous Newtonian motion; allow one integration step's
    # accumulated position error rather than assuming a particular integrator.
    expected = initial_position + initial_velocity * duration
    expected += 0.5 * model.opt.gravity * duration**2
    position_tolerance = np.linalg.norm(model.opt.gravity) * duration * model.opt.timestep
    assert data.qpos[qpos:qpos + 3] == pytest.approx(expected, abs=position_tolerance)
    assert data.qvel[dof:dof + 3] == pytest.approx(
        initial_velocity + model.opt.gravity * duration
    )


@pytest.mark.parametrize("axis", range(3), ids=["x", "y", "z"])
def test_free_ball_keeps_spinning_without_applied_torque(ball_scene, axis):
    model, data = ball_scene
    joint = model.joint("ball_free").id
    qpos = model.jnt_qposadr[joint]
    dof = model.jnt_dofadr[joint]
    angular_velocity = np.zeros(3)
    angular_velocity[axis] = 2.0
    data.qvel[dof + 3:dof + 6] = angular_velocity
    steps = 25
    duration = steps * model.opt.timestep
    for _ in range(steps):
        mujoco.mj_step(model, data)
        assert data.ncon == 0

    # For an isotropic solid sphere, constant spin gives an analytic
    # axis-angle quaternion. Positive scalar part removes sign ambiguity here.
    expected_quaternion = np.zeros(4)
    expected_quaternion[0] = np.cos(duration)
    expected_quaternion[1 + axis] = np.sin(duration)
    assert data.qpos[qpos + 3:qpos + 7] == pytest.approx(expected_quaternion, abs=1e-6)
    assert data.qvel[dof + 3:dof + 6] == pytest.approx(angular_velocity)
    assert np.linalg.norm(data.qpos[qpos + 3:qpos + 7]) == pytest.approx(1.0)


@pytest.mark.parametrize("side", [-1, 1], ids=["left", "right"])
def test_table_side_collision_changes_free_ball_momentum(ball_scene, side):
    model, data = ball_scene
    joint = model.joint("ball_free").id
    qpos = model.jnt_qposadr[joint]
    dof = model.jnt_dofadr[joint]
    data.qpos[qpos:qpos + 3] = [side * 0.55, 0, 0.475]
    data.qvel[dof:dof + 3] = [-side, 0, 0]
    pair = {model.geom("ball_geom").id, model.geom("table_top").id}
    peak_contact_force = 0.0
    for _ in range(40):
        mujoco.mj_step(model, data)
        for index, contact in enumerate(data.contact):
            if {contact.geom1, contact.geom2} == pair:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, index, force)
                peak_contact_force = max(peak_contact_force, force[0])

    assert peak_contact_force > 0
    assert side * data.qpos[qpos] > 0.48
    # The incoming speed is 1 m/s; the wall must oppose this motion.
    assert side * data.qvel[dof] > -0.5
    assert np.isfinite(data.qpos).all()
    assert np.isfinite(data.qvel).all()
