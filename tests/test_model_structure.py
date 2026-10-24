
"""P0-005.05 — Panda structural validation and failure tests.

Synthetic MJCF fixtures are deliberately small and self-contained.
The real Panda integration test uses the repository's local assets.

Tests must not modify production model files.
"""

import os
from pathlib import Path

import mujoco
import pytest

from robot_arm_sim.model_structure import (
    ModelStructureValidationError,
    validate_panda_structure,
)


def make_test_xml(
    *,
    arm_count: int = 7,
    finger_type: str = "slide",
    ball_type: str = "free",
    ball_joint_body: str = "ball",
    extra_joint: bool = False,
    extra_actuator: bool = False,
    missing_actuator: str | None = None,
) -> str:
    """Generate test MJCF with controlled structural defects.

    This synthetic model verifies structural checks only.
    It does not reproduce Panda actuation dynamics or geometry.
    """
    arm_bodies = []
    actuators = []

    for index in range(1, arm_count + 1):
        arm_bodies.append(
            f"""
            <body name="link{index}" pos="0 0 1">
              <joint name="joint{index}" type="hinge"/>
              <geom type="sphere" size=".03" mass="1"/>
            </body>
            """
        )
        actuators.append(
            f'<motor name="actuator{index}" joint="joint{index}"/>'
        )

    fingers = []
    for index in (1, 2):
        fingers.append(
            f"""
            <body name="finger{index}" pos="0 0 1">
              <joint name="finger_joint{index}"
                     type="{finger_type}"/>
              <geom type="sphere" size=".02" mass=".1"/>
            </body>
            """
        )

    if ball_type == "free":
        ball_joint = '<freejoint name="ball_free"/>'
    else:
        ball_joint = (
            f'<joint name="ball_free" type="{ball_type}"/>'
        )

    if ball_joint_body == "ball":
        ball_body = f"""
        <body name="ball" pos="0 0 2">
          {ball_joint}
          <geom type="sphere" size=".04" mass=".1"/>
        </body>
        """
    else:
        ball_body = f"""
        <body name="ball" pos="0 0 2">
          <geom type="sphere" size=".04" mass=".1"/>
        </body>
        <body name="wrong_ball_holder" pos="0 1 2">
          {ball_joint}
          <geom type="sphere" size=".04" mass=".1"/>
        </body>
        """

    # The synthetic gripper motor is only a structural stand-in.
    # The production Panda's actuator8 is tendon-driven.
    actuators.append(
        '<motor name="actuator8" joint="finger_joint1"/>'
    )

    extra_joint_xml = ""
    if extra_joint:
        extra_joint_xml = """
        <body name="unexpected_link" pos="1 0 1">
          <joint name="unexpected_joint" type="hinge"/>
          <geom type="sphere" size=".03" mass="1"/>
        </body>
        """

    if extra_actuator:
        actuators.append(
            '<motor name="unexpected_actuator" joint="joint1"/>'
        )

    if missing_actuator is not None:
        actuators = [
            entry for entry in actuators
            if f'name="{missing_actuator}"' not in entry
        ]

    return f"""
    <mujoco model="structural_fixture">
      <worldbody>
        {''.join(arm_bodies)}
        {''.join(fingers)}
        {ball_body}
        {extra_joint_xml}
      </worldbody>
      <actuator>
        {''.join(actuators)}
      </actuator>
    </mujoco>
    """


def compile_fixture(**kwargs) -> mujoco.MjModel:
    """Compile a synthetic model with the requested variation."""
    return mujoco.MjModel.from_xml_string(
        make_test_xml(**kwargs)
    )


def test_valid_structure_passes() -> None:
    model = compile_fixture()
    report = validate_panda_structure(model)

    assert report.arm_joint_count == 7
    assert report.finger_joint_count == 2
    assert report.arm_actuator_count == 7
    assert report.gripper_actuator_count == 1
    assert report.total_joint_count == 10
    assert report.total_actuator_count == 8


def test_correct_total_state_dimensions() -> None:
    model = compile_fixture()

    assert model.njnt == 10
    assert model.nu == 8
    assert model.nq == 16
    assert model.nv == 15

    report = validate_panda_structure(model)

    assert report.ball_qpos_width == 7
    assert report.ball_qvel_width == 6


def test_missing_arm_joint_rejected() -> None:
    model = compile_fixture(arm_count=6)

    with pytest.raises(
        ModelStructureValidationError,
        match="joint7",
    ):
        validate_panda_structure(model)


def test_extra_joint_rejected() -> None:
    model = compile_fixture(extra_joint=True)

    with pytest.raises(
        ModelStructureValidationError,
        match="total joints",
    ):
        validate_panda_structure(model)


@pytest.mark.parametrize(
    "finger_type",
    ["hinge", "ball"],
)
def test_wrong_finger_joint_type_rejected(
    finger_type: str,
) -> None:
    model = compile_fixture(finger_type=finger_type)

    with pytest.raises(
        ModelStructureValidationError,
        match="finger_joint",
    ):
        validate_panda_structure(model)


@pytest.mark.parametrize(
    "ball_type",
    ["hinge", "slide", "ball"],
)
def test_ball_must_be_free_joint(ball_type: str) -> None:
    model = compile_fixture(ball_type=ball_type)

    with pytest.raises(
        ModelStructureValidationError,
        match="ball_free",
    ):
        validate_panda_structure(model)


def test_free_joint_attached_to_wrong_body_rejected() -> None:
    model = compile_fixture(
        ball_joint_body="wrong_ball_holder"
    )

    with pytest.raises(
        ModelStructureValidationError,
        match="wrong body",
    ):
        validate_panda_structure(model)


@pytest.mark.parametrize(
    "missing",
    ["actuator1", "actuator4", "actuator7", "actuator8"],
)
def test_missing_actuator_rejected(missing: str) -> None:
    model = compile_fixture(missing_actuator=missing)

    with pytest.raises(
        ModelStructureValidationError,
        match=missing,
    ):
        validate_panda_structure(model)


def test_extra_actuator_rejected() -> None:
    model = compile_fixture(extra_actuator=True)

    with pytest.raises(
        ModelStructureValidationError,
        match="actuators",
    ):
        validate_panda_structure(model)


def test_wrong_model_argument_rejected() -> None:
    with pytest.raises(TypeError):
        validate_panda_structure(None)


def test_validation_is_read_only() -> None:
    model = compile_fixture()
    data = mujoco.MjData(model)

    original_qpos = data.qpos.copy()
    original_qvel = data.qvel.copy()
    original_ctrl = data.ctrl.copy()
    original_time = float(data.time)
    original_timestep = float(model.opt.timestep)

    validate_panda_structure(model)

    assert (data.qpos == original_qpos).all()
    assert (data.qvel == original_qvel).all()
    assert (data.ctrl == original_ctrl).all()
    assert data.time == original_time
    assert model.opt.timestep == original_timestep


def test_failure_does_not_modify_state() -> None:
    model = compile_fixture(extra_joint=True)
    data = mujoco.MjData(model)

    original_qpos = data.qpos.copy()
    original_time = float(data.time)

    with pytest.raises(ModelStructureValidationError):
        validate_panda_structure(model)

    assert (data.qpos == original_qpos).all()
    assert data.time == original_time


# ---------------------------------------------------------
# Real Franka Panda integration
# ---------------------------------------------------------

@pytest.mark.skipif(
    not os.environ.get("ROBOT_ARM_PANDA_XML"),
    reason="Panda model not configured",
)
def test_real_panda_structure() -> None:
    """Validate the complete production Panda task scene."""
    from robot_arm_sim.simulation import build_scene

    path = Path(os.environ["ROBOT_ARM_PANDA_XML"])
    model, data = build_scene(path)

    report = validate_panda_structure(model)

    assert report.total_joint_count == 10
    assert report.total_actuator_count == 8
    assert report.ball_qpos_width == 7
    assert report.ball_qvel_width == 6

    assert model.nq == 16
    assert model.nv == 15

    # Structural validation must not advance the simulation.
    assert data.time == pytest.approx(0.0)

def test_challenge_extra_joint_with_valid_required_names() -> None:
    """Reject an extra joint despite all required names existing.

    Challenge:
        All expected Panda joints remain present.
        An additional independently movable joint is introduced.

    Expected:
        MuJoCo compilation succeeds.
        Name resolution alone would find the required joints.
        Structural validation rejects the unexpected joint count.
    """
    model = compile_fixture(extra_joint=True)

    # Confirm that all required names still exist.
    required_joints = [
        *(f"joint{i}" for i in range(1, 8)),
        "finger_joint1",
        "finger_joint2",
        "ball_free",
    ]

    for name in required_joints:
        joint_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )
        assert joint_id >= 0, f"Required joint missing: {name}"

    # The malicious structural change adds one more joint.
    assert model.njnt == 11

    # It also adds one qpos and one qvel coordinate.
    assert model.nq == 17
    assert model.nv == 16

    # Even though all expected names exist, validation must fail.
    with pytest.raises(
        ModelStructureValidationError
    ) as exc:
        validate_panda_structure(model)

    message = str(exc.value)

    assert "Expected 10 total joints" in message
    assert "found 11" in message
    assert "Expected nq=16" in message
    assert "Expected nv=15" in message


def test_challenge_extra_joint_failure_is_read_only() -> None:
    """A rejected structural change must leave state untouched."""
    model = compile_fixture(extra_joint=True)
    data = mujoco.MjData(model)

    before_qpos = data.qpos.copy()
    before_qvel = data.qvel.copy()
    before_ctrl = data.ctrl.copy()
    before_time = float(data.time)

    with pytest.raises(ModelStructureValidationError):
        validate_panda_structure(model)

    # No state should have been changed by validation.
    assert (data.qpos == before_qpos).all()
    assert (data.qvel == before_qvel).all()
    assert (data.ctrl == before_ctrl).all()
    assert data.time == before_time


def test_challenge_ball_free_joint_on_wrong_body() -> None:
    """Reject a correctly named free joint attached to another body.

    Challenge:
        Joint name is correct.
        Joint type is correct.
        Joint and actuator counts are correct.
        State dimensions are correct.
        Only the joint-to-body relationship is wrong.

    Expected:
        Validation fails on joint ownership.
    """
    model = compile_fixture(
        ball_joint_body="wrong_ball_holder"
    )

    # All structural counts are deliberately correct.
    assert model.njnt == 10
    assert model.nu == 8
    assert model.nq == 16
    assert model.nv == 15

    ball_joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "ball_free",
    )

    ball_body_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "ball",
    )

    assert ball_joint_id >= 0
    assert ball_body_id >= 0

    # Correct name and correct free-joint type.
    assert int(model.jnt_type[ball_joint_id]) == int(
        mujoco.mjtJoint.mjJNT_FREE
    )

    # But the free joint is attached to the wrong body.
    assert int(model.jnt_bodyid[ball_joint_id]) != ball_body_id

    with pytest.raises(
        ModelStructureValidationError,
        match="ball_free is attached to the wrong body",
    ):
        validate_panda_structure(model)


def test_challenge_wrong_ball_joint_does_not_advance_physics() -> None:
    """Reject invalid ownership without advancing simulation time."""
    model = compile_fixture(
        ball_joint_body="wrong_ball_holder"
    )
    data = mujoco.MjData(model)

    initial_time = float(data.time)
    original_qpos = data.qpos.copy()
    original_qvel = data.qvel.copy()

    with pytest.raises(ModelStructureValidationError):
        validate_panda_structure(model)

    assert data.time == initial_time
    assert (data.qpos == original_qpos).all()
    assert (data.qvel == original_qvel).all()


def test_challenge_arm_joint_type_changed_with_same_dimensions() -> None:
    """Reject a slide joint substituted for an expected arm hinge.

    A slide and a hinge joint each contribute one qpos and one
    qvel coordinate, so total dimensions alone cannot detect it.
    """
    xml = make_test_xml()

    # Modify only the synthetic fixture in memory.
    xml = xml.replace(
        '<joint name="joint4" type="hinge"/>',
        '<joint name="joint4" type="slide"/>',
    )

    model = mujoco.MjModel.from_xml_string(xml)

    assert model.njnt == 10
    assert model.nu == 8
    assert model.nq == 16
    assert model.nv == 15

    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "joint4",
    )

    assert int(model.jnt_type[joint_id]) == int(
        mujoco.mjtJoint.mjJNT_SLIDE
    )

    with pytest.raises(
        ModelStructureValidationError,
        match="joint4",
    ):
        validate_panda_structure(model)
