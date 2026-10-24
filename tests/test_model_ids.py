
"""P0-005.03 — Structural name-resolution regression tests."""

import mujoco
import pytest
import numpy as np

from robot_arm_sim.model_ids import (
    ModelStructureError,
    RequiredModelNames,
    resolve_id,
    validate_model_names,
)


@pytest.fixture
def simple_model() -> mujoco.MjModel:
    """Small valid MJCF model without external assets."""
    xml = """
    <mujoco model="name_validation">
      <worldbody>
        <body name="test_body">
          <joint name="test_joint" type="hinge"/>
          <geom type="sphere" size="0.05" mass="1"/>
          <site name="test_site" size="0.01"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="test_actuator" joint="test_joint"/>
      </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


@pytest.mark.parametrize(
    "object_type,name",
    [
        (mujoco.mjtObj.mjOBJ_JOINT, "test_joint"),
        (mujoco.mjtObj.mjOBJ_ACTUATOR, "test_actuator"),
        (mujoco.mjtObj.mjOBJ_BODY, "test_body"),
        (mujoco.mjtObj.mjOBJ_SITE, "test_site"),
    ],
)
def test_resolves_existing_names(
    simple_model, object_type, name
):
    identifier = resolve_id(simple_model, object_type, name)
    assert identifier >= 0
    assert mujoco.mj_id2name(
        simple_model, object_type, identifier
    ) == name


@pytest.mark.parametrize(
    "object_type",
    [
        mujoco.mjtObj.mjOBJ_JOINT,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        mujoco.mjtObj.mjOBJ_BODY,
        mujoco.mjtObj.mjOBJ_SITE,
    ],
)
def test_missing_name_is_rejected(
    simple_model, object_type
):
    with pytest.raises(
        ModelStructureError, match="does_not_exist"
    ):
        resolve_id(
            simple_model, object_type, "does_not_exist"
        )


def test_wrong_object_category_rejected(simple_model):
    """A joint name must not resolve as an actuator."""
    with pytest.raises(ModelStructureError):
        resolve_id(
            simple_model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            "test_joint",
        )


@pytest.mark.parametrize("name", ["", " ", "\t"])
def test_blank_names_rejected(simple_model, name):
    with pytest.raises(ValueError):
        resolve_id(
            simple_model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
        )


def test_invalid_model_type_rejected():
    with pytest.raises(TypeError):
        resolve_id(
            None,
            mujoco.mjtObj.mjOBJ_JOINT,
            "test_joint",
        )


def test_invalid_object_type_rejected(simple_model):
    with pytest.raises(TypeError):
        resolve_id(simple_model, 999, "test_joint")


def test_unsupported_object_category_rejected(simple_model):
    with pytest.raises(TypeError):
        resolve_id(
            simple_model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "test_joint",
        )


def test_complete_custom_schema(simple_model):
    """The resolver must work for non-Panda models too."""
    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=(),
        arm_actuators=("test_actuator",),
        gripper_actuators=(),
        robot_bodies=("test_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("test_site",),
    )

    result = validate_model_names(simple_model, required)

    assert "test_joint" in result.joints
    assert "test_actuator" in result.actuators
    assert "test_body" in result.bodies
    assert "test_site" in result.sites


def test_missing_components_reported_together(simple_model):
    required = RequiredModelNames(
        arm_joints=("missing_joint",),
        finger_joints=(),
        arm_actuators=("missing_actuator",),
        gripper_actuators=(),
        robot_bodies=("missing_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("missing_site",),
    )

    with pytest.raises(ModelStructureError) as exc:
        validate_model_names(simple_model, required)

    message = str(exc.value)

    for name in (
        "missing_joint",
        "missing_actuator",
        "missing_body",
        "missing_site",
    ):
        assert name in message


def test_duplicate_names_rejected(simple_model):
    required = RequiredModelNames(
        arm_joints=("test_joint", "test_joint"),
        finger_joints=(),
        arm_actuators=(),
        gripper_actuators=(),
        robot_bodies=(),
        scene_bodies=(),
        scene_joints=(),
        sites=(),
    )

    with pytest.raises(ModelStructureError, match="Duplicate"):
        validate_model_names(simple_model, required)


def test_no_physics_step_occurs(simple_model):
    data = mujoco.MjData(simple_model)
    before = data.time

    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=(),
        arm_actuators=("test_actuator",),
        gripper_actuators=(),
        robot_bodies=("test_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("test_site",),
    )

    validate_model_names(simple_model, required)

    assert data.time == before


def test_default_panda_schema_rejects_incomplete_model(
    simple_model,
):
    """A synthetic model must not masquerade as the Panda scene."""
    with pytest.raises(ModelStructureError) as exc:
        validate_model_names(simple_model)

    assert "joint7" in str(exc.value)
    assert "actuator8" in str(exc.value)
    assert "ball_free" in str(exc.value)
    assert "target_marker" in str(exc.value)


# P0-005.03 — Additional adversarial validation tests.
#
# These tests deliberately use malformed schemas, ambiguous names,
# incomplete models and reordered model definitions.
#
# They do not modify production assets or advance simulation physics.


def test_same_name_in_different_object_categories() -> None:
    """Object type must be part of the name-resolution lookup.

    MuJoCo has separate namespaces for different object types.
    A joint and actuator can therefore share a name without
    having the same underlying identity.
    """
    xml = """
    <mujoco model="shared_names">
      <worldbody>
        <body name="robot">
          <joint name="shared" type="hinge"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="shared" joint="shared"/>
      </actuator>
    </mujoco>
    """

    model = mujoco.MjModel.from_xml_string(xml)

    joint_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "shared"
    )
    actuator_id = resolve_id(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, "shared"
    )

    assert mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
    ) == "shared"

    assert mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
    ) == "shared"


@pytest.mark.parametrize(
    "wrong_name",
    [
        "Test_Joint",
        "TEST_JOINT",
        "test_Joint",
        "test_joint ",
        " test_joint",
        "test_joint\n",
    ],
)
def test_name_matching_is_exact(
    simple_model,
    wrong_name: str,
) -> None:
    """Name lookup must not perform implicit normalisation."""
    with pytest.raises(ModelStructureError):
        resolve_id(
            simple_model,
            mujoco.mjtObj.mjOBJ_JOINT,
            wrong_name,
        )


def test_duplicate_names_across_joint_groups_rejected(
    simple_model,
) -> None:
    """Duplicates must be rejected across merged schema groups."""
    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=("test_joint",),
        arm_actuators=("test_actuator",),
        gripper_actuators=(),
        robot_bodies=("test_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("test_site",),
    )

    with pytest.raises(
        ModelStructureError, match="Duplicate"
    ):
        validate_model_names(simple_model, required)


def test_duplicate_names_across_actuator_groups_rejected(
    simple_model,
) -> None:
    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=(),
        arm_actuators=("test_actuator",),
        gripper_actuators=("test_actuator",),
        robot_bodies=("test_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("test_site",),
    )

    with pytest.raises(
        ModelStructureError, match="Duplicate"
    ):
        validate_model_names(simple_model, required)


def test_duplicate_names_across_body_groups_rejected(
    simple_model,
) -> None:
    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=(),
        arm_actuators=("test_actuator",),
        gripper_actuators=(),
        robot_bodies=("test_body",),
        scene_bodies=("test_body",),
        scene_joints=(),
        sites=("test_site",),
    )

    with pytest.raises(
        ModelStructureError, match="Duplicate"
    ):
        validate_model_names(simple_model, required)


def test_empty_schema_is_rejected_for_full_validation(
    simple_model,
) -> None:
    """A completely empty schema must not yield a false success.

    This requires the proposed empty-schema guard to be added
    to validate_model_names().
    """
    required = RequiredModelNames(
        arm_joints=(),
        finger_joints=(),
        arm_actuators=(),
        gripper_actuators=(),
        robot_bodies=(),
        scene_bodies=(),
        scene_joints=(),
        sites=(),
    )

    with pytest.raises(
        ModelStructureError, match="empty"
    ):
        validate_model_names(simple_model, required)


def test_model_ids_recomputed_when_joint_order_changes() -> None:
    """Names, not hard-coded numeric indices, identify joints."""
    first_xml = """
    <mujoco>
      <worldbody>
        <body>
          <joint name="alpha" type="hinge"/>
          <joint name="beta" type="hinge"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    second_xml = """
    <mujoco>
      <worldbody>
        <body>
          <joint name="beta" type="hinge"/>
          <joint name="alpha" type="hinge"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    first = mujoco.MjModel.from_xml_string(first_xml)
    second = mujoco.MjModel.from_xml_string(second_xml)

    first_alpha = resolve_id(
        first, mujoco.mjtObj.mjOBJ_JOINT, "alpha"
    )
    second_alpha = resolve_id(
        second, mujoco.mjtObj.mjOBJ_JOINT, "alpha"
    )

    assert first_alpha != second_alpha

    assert mujoco.mj_id2name(
        first,
        mujoco.mjtObj.mjOBJ_JOINT,
        first_alpha,
    ) == "alpha"

    assert mujoco.mj_id2name(
        second,
        mujoco.mjtObj.mjOBJ_JOINT,
        second_alpha,
    ) == "alpha"


def test_validation_does_not_modify_compiled_model(
    simple_model,
) -> None:
    """Structural validation must not mutate model parameters."""
    original_mass = simple_model.body_mass.copy()
    original_range = simple_model.jnt_range.copy()
    original_timestep = float(simple_model.opt.timestep)

    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=(),
        arm_actuators=("test_actuator",),
        gripper_actuators=(),
        robot_bodies=("test_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("test_site",),
    )

    validate_model_names(simple_model, required)

    np.testing.assert_array_equal(
        simple_model.body_mass, original_mass
    )
    np.testing.assert_array_equal(
        simple_model.jnt_range, original_range
    )
    assert simple_model.opt.timestep == original_timestep


def test_failed_validation_does_not_mutate_model(
    simple_model,
) -> None:
    """Failed validation must also be read-only."""
    original_mass = simple_model.body_mass.copy()
    original_range = simple_model.jnt_range.copy()

    with pytest.raises(ModelStructureError):
        validate_model_names(simple_model)

    np.testing.assert_array_equal(
        simple_model.body_mass, original_mass
    )
    np.testing.assert_array_equal(
        simple_model.jnt_range, original_range
    )


def test_resolved_result_does_not_depend_on_later_lookups(
    simple_model,
) -> None:
    """Completed results must retain their original mappings."""
    required = RequiredModelNames(
        arm_joints=("test_joint",),
        finger_joints=(),
        arm_actuators=("test_actuator",),
        gripper_actuators=(),
        robot_bodies=("test_body",),
        scene_bodies=(),
        scene_joints=(),
        sites=("test_site",),
    )

    first = validate_model_names(simple_model, required)

    original_id = first.joints["test_joint"]

    second = validate_model_names(simple_model, required)

    assert first.joints["test_joint"] == original_id
    assert second.joints["test_joint"] == original_id


def test_missing_site_does_not_fallback_to_body(
    simple_model,
) -> None:
    """Body names must never silently substitute for site names."""
    with pytest.raises(ModelStructureError):
        resolve_id(
            simple_model,
            mujoco.mjtObj.mjOBJ_SITE,
            "test_body",
        )


def test_unexpected_schema_type_is_rejected(
    simple_model,
) -> None:
    with pytest.raises(TypeError):
        validate_model_names(
            simple_model,
            required={"arm_joints": ["test_joint"]},
        )
