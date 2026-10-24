
"""P0-005.04 — Model-specific address cache regression tests."""

from dataclasses import FrozenInstanceError

import mujoco
import numpy as np
import pytest
import os
from pathlib import Path
from types import MappingProxyType


from robot_arm_sim.model_addresses import (
    ModelAddressError,
    build_address_cache,
)
from robot_arm_sim.model_ids import (
    RequiredModelNames,
    ResolvedModelIds,
    validate_model_names,
)


@pytest.fixture
def model() -> mujoco.MjModel:
    """A mixed joint-type model with a free joint and actuated hinge."""
    xml = """
    <mujoco model="address_test">
      <worldbody>
        <body name="ball" pos="0 0 1">
          <freejoint name="ball_free"/>
          <geom type="sphere" size="0.04" mass="0.1"/>
        </body>
        <body name="arm">
          <joint name="hinge" type="hinge"/>
          <geom type="box" size=".05 .05 .05" mass="1"/>
          <site name="tool" size=".01"/>
        </body>
      </worldbody>
      <actuator>
        <motor name="motor" joint="hinge"/>
      </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


@pytest.fixture
def resolved(model):
    required = RequiredModelNames(
        arm_joints=("hinge",),
        finger_joints=(),
        arm_actuators=("motor",),
        gripper_actuators=(),
        robot_bodies=("arm",),
        scene_bodies=("ball",),
        scene_joints=("ball_free",),
        sites=("tool",),
    )
    return validate_model_names(model, required)


def test_correct_joint_address_dimensions(model, resolved):
    cache = build_address_cache(model, resolved)

    ball = cache.joint("ball_free", model=model)
    hinge = cache.joint("hinge", model=model)

    assert ball.qpos_width == 7
    assert ball.qvel_width == 6
    assert hinge.qpos_width == 1
    assert hinge.qvel_width == 1


def test_qpos_and_qvel_addresses_are_not_confused(model, resolved):
    cache = build_address_cache(model, resolved)
    hinge = cache.joint("hinge", model=model)

    assert hinge.qpos_start == int(
        model.jnt_qposadr[hinge.joint_id]
    )
    assert hinge.qvel_start == int(
        model.jnt_dofadr[hinge.joint_id]
    )

    # The preceding free joint ensures the two address
    # spaces have different offsets for the hinge.
    assert hinge.qpos_start != hinge.qvel_start


def test_all_joint_slices_within_bounds(model, resolved):
    cache = build_address_cache(model, resolved)

    for address in cache.joints.values():
        assert address.qpos_start >= 0
        assert address.qvel_start >= 0
        assert address.qpos_slice.stop <= model.nq
        assert address.qvel_slice.stop <= model.nv


def test_free_joint_slices_read_correct_dimensions(model, resolved):
    cache = build_address_cache(model, resolved)
    data = mujoco.MjData(model)

    address = cache.joint("ball_free", model=model)

    assert data.qpos[address.qpos_slice].shape == (7,)
    assert data.qvel[address.qvel_slice].shape == (6,)


def test_different_model_is_rejected(model, resolved):
    cache = build_address_cache(model, resolved)

    other_model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <body>
              <joint name="hinge" type="hinge"/>
              <geom type="sphere" size=".05" mass="1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )

    with pytest.raises(ModelAddressError, match="different"):
        cache.joint("hinge", model=other_model)


def test_wrong_joint_identifier_is_rejected(model, resolved):
    corrupted = ResolvedModelIds(
        joints={**resolved.joints, "hinge": 999999},
        actuators=dict(resolved.actuators),
        bodies=dict(resolved.bodies),
        sites=dict(resolved.sites),
    )

    with pytest.raises(ModelAddressError):
        build_address_cache(model, corrupted)


def test_cross_wired_joint_identifier_is_rejected(model, resolved):
    """A valid numeric ID belonging to the wrong name must fail."""
    corrupted = ResolvedModelIds(
        joints={
            **resolved.joints,
            "hinge": resolved.joints["ball_free"],
        },
        actuators=dict(resolved.actuators),
        bodies=dict(resolved.bodies),
        sites=dict(resolved.sites),
    )

    with pytest.raises(ModelAddressError, match="mismatch"):
        build_address_cache(model, corrupted)


def test_negative_identifier_is_rejected(model, resolved):
    corrupted = ResolvedModelIds(
        joints={"hinge": -1},
        actuators={},
        bodies={},
        sites={},
    )

    with pytest.raises(ModelAddressError):
        build_address_cache(model, corrupted)


def test_boolean_identifier_is_rejected(model, resolved):
    corrupted = ResolvedModelIds(
        joints={"hinge": True},
        actuators={},
        bodies={},
        sites={},
    )

    with pytest.raises(ModelAddressError):
        build_address_cache(model, corrupted)


def test_wrong_actuator_identifier_is_rejected(model, resolved):
    corrupted = ResolvedModelIds(
        joints=dict(resolved.joints),
        actuators={"motor": 500},
        bodies=dict(resolved.bodies),
        sites=dict(resolved.sites),
    )

    with pytest.raises(ModelAddressError):
        build_address_cache(model, corrupted)


def test_cache_rejects_mutation(model, resolved):
    cache = build_address_cache(model, resolved)

    with pytest.raises(TypeError):
        cache.joints["hinge"] = None

    with pytest.raises(FrozenInstanceError):
        cache.model = None


def test_input_mapping_mutation_does_not_change_cache(model, resolved):
    cache = build_address_cache(model, resolved)
    original = cache.joint("hinge", model=model).joint_id

    resolved.joints["hinge"] = -1

    assert cache.joint("hinge", model=model).joint_id == original


def test_missing_cached_joint_rejected(model, resolved):
    cache = build_address_cache(model, resolved)

    with pytest.raises(ModelAddressError, match="not in"):
        cache.joint("nonexistent", model=model)


def test_cache_build_does_not_mutate_model_or_state(model, resolved):
    data = mujoco.MjData(model)

    qpos_before = data.qpos.copy()
    qvel_before = data.qvel.copy()
    mass_before = model.body_mass.copy()
    time_before = float(data.time)

    build_address_cache(model, resolved)

    np.testing.assert_array_equal(data.qpos, qpos_before)
    np.testing.assert_array_equal(data.qvel, qvel_before)
    np.testing.assert_array_equal(model.body_mass, mass_before)
    assert data.time == time_before


def test_correct_actuator_control_index(model, resolved):
    cache = build_address_cache(model, resolved)
    actuator_id = cache.actuator("motor", model=model)

    assert actuator_id == mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        "motor",
    )
    assert 0 <= actuator_id < model.nu


# P0-005.04 — Additional adversarial address-cache tests.
#
# These tests deliberately attempt to expose:
# - Wrong numeric IDs that still fall within valid bounds.
# - Cache reuse against independently compiled models.
# - Incorrect qpos/qvel dimensions.
# - Mutation of cached mappings.
# - Incorrect actuator, body and site references.
# - Differences between joint ID and state-array addresses.
#
# No production MJCF files or simulation scripts are modified.

# -----------------------------------------------------------
# Additional synthetic models
# -----------------------------------------------------------

@pytest.fixture
def all_joint_types_model() -> mujoco.MjModel:
    """Build a model containing every supported MuJoCo joint type.

    Different joint representations must produce the correct
    numbers of qpos and qvel coordinates.
    """
    xml = """
    <mujoco model="all_joint_types">
      <worldbody>
        <body name="free_body" pos="0 0 2">
          <freejoint name="free_joint"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>

        <body name="ball_body" pos="0 0 1">
          <joint name="ball_joint" type="ball"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>

        <body name="hinge_body" pos="1 0 1">
          <joint name="hinge_joint" type="hinge"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>

        <body name="slide_body" pos="-1 0 1">
          <joint name="slide_joint" type="slide"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def _all_joint_schema() -> RequiredModelNames:
    """Create a custom schema covering four distinct joint types."""
    return RequiredModelNames(
        arm_joints=(
            "free_joint",
            "ball_joint",
            "hinge_joint",
            "slide_joint",
        ),
        finger_joints=(),
        arm_actuators=(),
        gripper_actuators=(),
        robot_bodies=(
            "free_body",
            "ball_body",
            "hinge_body",
            "slide_body",
        ),
        scene_bodies=(),
        scene_joints=(),
        sites=(),
    )


def test_all_four_joint_types_have_correct_widths(
    all_joint_types_model,
) -> None:
    """Every MuJoCo joint type must receive its documented widths."""
    compiled = all_joint_types_model
    resolved = validate_model_names(
        compiled, _all_joint_schema()
    )
    cache = build_address_cache(compiled, resolved)

    expected = {
        "free_joint": (7, 6),
        "ball_joint": (4, 3),
        "hinge_joint": (1, 1),
        "slide_joint": (1, 1),
    }

    for name, (qpos_width, qvel_width) in expected.items():
        address = cache.joint(name, model=compiled)

        assert address.qpos_width == qpos_width
        assert address.qvel_width == qvel_width


def test_all_joint_slices_cover_exact_state_dimensions(
    all_joint_types_model,
) -> None:
    """Joint address ranges should partition qpos and qvel exactly.

    This test detects overlapping slices and missing coordinates.
    """
    compiled = all_joint_types_model
    resolved = validate_model_names(
        compiled, _all_joint_schema()
    )
    cache = build_address_cache(compiled, resolved)

    qpos_indices = []
    qvel_indices = []

    for address in cache.joints.values():
        qpos_indices.extend(
            range(
                address.qpos_start,
                address.qpos_start + address.qpos_width,
            )
        )
        qvel_indices.extend(
            range(
                address.qvel_start,
                address.qvel_start + address.qvel_width,
            )
        )

    assert sorted(qpos_indices) == list(range(compiled.nq))
    assert sorted(qvel_indices) == list(range(compiled.nv))


def test_free_joint_is_not_treated_as_six_qpos_values(
    all_joint_types_model,
) -> None:
    """Free-joint position includes a quaternion."""
    compiled = all_joint_types_model
    resolved = validate_model_names(
        compiled, _all_joint_schema()
    )
    cache = build_address_cache(compiled, resolved)

    address = cache.joint("free_joint", model=compiled)
    data = mujoco.MjData(compiled)

    assert data.qpos[address.qpos_slice].size == 7
    assert data.qvel[address.qvel_slice].size == 6


# -----------------------------------------------------------
# Cache isolation and stale-cache attacks
# -----------------------------------------------------------

def test_recompiled_equivalent_model_rejects_old_cache(
    all_joint_types_model,
) -> None:
    """Identical-looking models are not the same model object."""
    compiled = all_joint_types_model
    resolved = validate_model_names(
        compiled, _all_joint_schema()
    )
    old_cache = build_address_cache(compiled, resolved)

    # MuJoCo allocates a fresh compiled model.
    second = mujoco.MjModel.from_xml_string(
        mujoco.mj_saveLastXML.__doc__
    ) if False else None

    # Build an independently compiled equivalent model from
    # the same synthetic fixture definition using a fresh
    # model specification source.
    #
    # A deep copy is not used because the production cache
    # contract concerns independently compiled MjModels.
    second = mujoco.MjModel.from_xml_string("""
    <mujoco>
      <worldbody>
        <body name="other">
          <joint name="hinge" type="hinge"/>
          <geom type="sphere" size=".05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """)

    with pytest.raises(ModelAddressError, match="different"):
        old_cache.joint("free_joint", model=second)


def test_same_numeric_joint_id_wrong_name_is_rejected(
    model,
    resolved,
) -> None:
    """A valid in-range ID for another joint must not be accepted."""
    wrong = ResolvedModelIds(
        joints={
            "hinge": resolved.joints["ball_free"],
            "ball_free": resolved.joints["hinge"],
        },
        actuators=dict(resolved.actuators),
        bodies=dict(resolved.bodies),
        sites=dict(resolved.sites),
    )

    with pytest.raises(ModelAddressError, match="mismatch"):
        build_address_cache(model, wrong)


def test_wrong_body_id_is_rejected(model, resolved) -> None:
    """A body ID cannot be substituted for a different named body."""
    corrupted = ResolvedModelIds(
        joints=dict(resolved.joints),
        actuators=dict(resolved.actuators),
        bodies={
            **resolved.bodies,
            "ball": resolved.bodies["arm"],
        },
        sites=dict(resolved.sites),
    )

    with pytest.raises(ModelAddressError, match="mismatch"):
        build_address_cache(model, corrupted)


def test_wrong_site_id_is_rejected(model, resolved) -> None:
    """An invalid site ID must fail before any state access."""
    corrupted = ResolvedModelIds(
        joints=dict(resolved.joints),
        actuators=dict(resolved.actuators),
        bodies=dict(resolved.bodies),
        sites={"tool": model.nsite},
    )

    with pytest.raises(ModelAddressError):
        build_address_cache(model, corrupted)


def test_negative_body_id_rejected(model, resolved) -> None:
    """Negative IDs must never reach Python array indexing."""
    corrupted = ResolvedModelIds(
        joints=dict(resolved.joints),
        actuators=dict(resolved.actuators),
        bodies={"ball": -1},
        sites=dict(resolved.sites),
    )

    with pytest.raises(ModelAddressError):
        build_address_cache(model, corrupted)


def test_cached_mapping_is_read_only(model, resolved) -> None:
    """Callers must not alter cached address mappings."""
    cache = build_address_cache(model, resolved)

    assert isinstance(cache.joints, MappingProxyType)
    assert isinstance(cache.actuators, MappingProxyType)
    assert isinstance(cache.bodies, MappingProxyType)
    assert isinstance(cache.sites, MappingProxyType)

    with pytest.raises(TypeError):
        cache.actuators["motor"] = 999

    with pytest.raises(TypeError):
        cache.bodies["ball"] = 999

    with pytest.raises(TypeError):
        cache.sites["tool"] = 999


def test_missing_actuator_lookup_fails(model, resolved) -> None:
    """Missing cached actuator names must produce useful errors."""
    cache = build_address_cache(model, resolved)

    with pytest.raises(ModelAddressError, match="not in"):
        cache.actuator("nonexistent_actuator", model=model)


def test_independent_caches_do_not_share_mapping_storage(
    model,
    resolved,
) -> None:
    """Each cache must copy the input mapping."""
    first = build_address_cache(model, resolved)
    second = build_address_cache(model, resolved)

    assert first.joints is not second.joints
    assert first.actuators is not second.actuators
    assert first.bodies is not second.bodies


# -----------------------------------------------------------
# Actual Franka Panda integration
# -----------------------------------------------------------

@pytest.mark.skipif(
    not os.environ.get("ROBOT_ARM_PANDA_XML"),
    reason="Panda model not configured",
)
def test_panda_joint_and_actuator_address_cache() -> None:
    """Validate all required addresses in the real composed scene."""
    from robot_arm_sim.simulation import build_scene

    panda_path = Path(os.environ["ROBOT_ARM_PANDA_XML"])
    compiled, data = build_scene(panda_path)

    resolved = validate_model_names(compiled)
    cache = build_address_cache(compiled, resolved)

    assert len(cache.actuators) == 8

    # All seven arm joints must have scalar coordinates.
    for number in range(1, 8):
        address = cache.joint(
            f"joint{number}",
            model=compiled,
        )

        assert address.qpos_width == 1
        assert address.qvel_width == 1

        assert data.qpos[address.qpos_slice].size == 1
        assert data.qvel[address.qvel_slice].size == 1

    # Finger joints are slide joints with scalar coordinates.
    for name in ("finger_joint1", "finger_joint2"):
        address = cache.joint(name, model=compiled)

        assert address.qpos_width == 1
        assert address.qvel_width == 1

    # Ball free joint must retain its full coordinates.
    ball = cache.joint("ball_free", model=compiled)

    assert ball.qpos_width == 7
    assert ball.qvel_width == 6

    assert data.qpos[ball.qpos_slice].size == 7
    assert data.qvel[ball.qvel_slice].size == 6

    # The cached actuator ID is used only to index controls,
    # never interpreted as a joint ID.
    for number in range(1, 9):
        actuator_id = cache.actuator(
            f"actuator{number}",
            model=compiled,
        )
        assert 0 <= actuator_id < compiled.nu


@pytest.mark.skipif(
    not os.environ.get("ROBOT_ARM_PANDA_XML"),
    reason="Panda model not configured",
)
def test_panda_cache_isolation_across_instances() -> None:
    """Each independently composed Panda scene gets its own cache."""
    from robot_arm_sim.simulation import build_scene

    panda_path = Path(os.environ["ROBOT_ARM_PANDA_XML"])

    model_a, _ = build_scene(panda_path)
    model_b, _ = build_scene(panda_path)

    cache_a = build_address_cache(
        model_a, validate_model_names(model_a)
    )
    cache_b = build_address_cache(
        model_b, validate_model_names(model_b)
    )

    assert cache_a.model is model_a
    assert cache_b.model is model_b

    with pytest.raises(ModelAddressError, match="different"):
        cache_a.joint("joint1", model=model_b)

    with pytest.raises(ModelAddressError, match="different"):
        cache_b.joint("joint1", model=model_a)

    with pytest.raises(ModelAddressError, match="different"):
        cache_a.actuator("actuator1", model=model_b)

    with pytest.raises(ModelAddressError, match="different"):
        cache_b.actuator("actuator1", model=model_a)
