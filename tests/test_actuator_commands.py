
"""P0-007.01 — Typed actuator command and registry tests.

Verify:
- Complete immutable Panda commands.
- Correct logical-to-MuJoCo actuator mapping.
- Compiled actuator ordering, not assumed index ordering.
- Rejection of invalid commands and incompatible models.
- Compatibility with the existing Simulator stepping API.
"""

from dataclasses import FrozenInstanceError, replace
import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_arm_sim.actuator_commands import (
    PANDA_ACTUATORS,
    ActuatorDescriptor,
    ActuatorRegistry,
    PandaCommand,
)
from robot_arm_sim.simulator import Simulator


@pytest.fixture
def sim():
    """Create a fresh Panda simulation using reviewed assets."""
    source = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not source:
        pytest.fail(
            "ROBOT_ARM_PANDA_XML is required"
        )

    simulator = Simulator(Path(source))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


@pytest.fixture
def registry(sim):
    """Construct a registry from the compiled MuJoCo model."""
    return ActuatorRegistry.from_model(sim.model)


@pytest.fixture
def valid_command(sim):
    """Use the model's initial known-valid raw control values.

    Resolve values by actuator names rather than assuming
    that the control vector is already in logical order.
    """
    values = {}

    for logical_name, mujoco_name in PANDA_ACTUATORS:
        actuator_id = sim.model.actuator(mujoco_name).id
        values[logical_name] = float(
            sim.data.ctrl[actuator_id]
        )

    return PandaCommand(**values)


def test_command_has_exactly_eight_named_channels(valid_command):
    """The complete command has seven arm plus one gripper input."""
    names = valid_command.as_named_values()

    assert tuple(names) == (
        "arm1",
        "arm2",
        "arm3",
        "arm4",
        "arm5",
        "arm6",
        "arm7",
        "gripper",
    )


def test_command_is_immutable(valid_command):
    """Ordinary reassignment must be rejected."""
    with pytest.raises(FrozenInstanceError):
        valid_command.arm1 = 1.0


def test_command_has_no_instance_dictionary(valid_command):
    """Slots prevent arbitrary attributes being attached."""
    assert not hasattr(valid_command, "__dict__")

    with pytest.raises((AttributeError, TypeError)):
        valid_command.unrelated = 1.0


def test_named_dictionary_is_independent(valid_command):
    """Mutating an exported dictionary must not change the command."""
    exported = valid_command.as_named_values()
    original = valid_command.arm1

    exported["arm1"] = 12345.0

    assert valid_command.arm1 == original


def test_incomplete_command_is_rejected():
    """Missing arm or gripper channels are not permitted."""
    with pytest.raises(TypeError):
        PandaCommand(
            arm1=0.0,
            arm2=0.0,
            arm3=0.0,
            arm4=0.0,
            arm5=0.0,
            arm6=0.0,
            arm7=0.0,
        )


def test_unknown_command_field_is_rejected(valid_command):
    """Commands cannot silently accept unrecognised fields."""
    values = valid_command.as_named_values()
    values["arm8"] = 0.0

    with pytest.raises(TypeError):
        PandaCommand(**values)


@pytest.mark.parametrize(
    "bad_value",
    [
        None,
        True,
        False,
        "0.0",
        b"0.0",
        1 + 2j,
        object(),
        np.bool_(True),
    ],
)
def test_invalid_scalar_types_rejected(
    valid_command,
    bad_value,
):
    """Reject coercion from non-real command representations."""
    values = valid_command.as_named_values()
    values["arm1"] = bad_value

    with pytest.raises(TypeError):
        PandaCommand(**values)


@pytest.mark.parametrize(
    "bad_value",
    [
        float("nan"),
        float("inf"),
        float("-inf"),
        10**400,
    ],
)
def test_nonfinite_and_oversized_values_rejected(
    valid_command,
    bad_value,
):
    """Invalid values cannot enter an immutable command."""
    values = valid_command.as_named_values()
    values["arm1"] = bad_value

    with pytest.raises(ValueError):
        PandaCommand(**values)


def test_numpy_numeric_scalars_are_accepted(valid_command):
    """Valid NumPy scalars remain supported for callers."""
    values = valid_command.as_named_values()

    values["arm1"] = np.float32(0.0)
    values["arm2"] = np.float64(0.0)
    values["gripper"] = np.int64(0)

    command = PandaCommand(**values)

    assert isinstance(command.arm1, float)
    assert isinstance(command.arm2, float)
    assert isinstance(command.gripper, float)


def test_registry_requires_real_mujoco_model():
    """Do not create a registry from unrelated objects."""
    with pytest.raises(TypeError):
        ActuatorRegistry.from_model(object())


def test_registry_has_exactly_eight_descriptors(registry, sim):
    """The reference Panda exposes eight named actuators."""
    descriptors = registry.describe()

    assert len(descriptors) == 8
    assert sim.model.nu == 8
    assert all(
        isinstance(d, ActuatorDescriptor)
        for d in descriptors
    )


def test_registry_names_match_compiled_model(registry, sim):
    """Every descriptor must refer to the actual compiled name."""
    for descriptor in registry.describe():
        actual_name = mujoco.mj_id2name(
            sim.model,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            descriptor.actuator_id,
        )

        assert actual_name == descriptor.mujoco_name


def test_registry_logical_names_are_complete(registry):
    """No expected logical channel is omitted or duplicated."""
    names = [
        descriptor.logical_name
        for descriptor in registry.describe()
    ]

    assert set(names) == {
        logical_name
        for logical_name, _ in PANDA_ACTUATORS
    }
    assert len(set(names)) == 8


def test_registry_actuator_ids_form_complete_permutation(registry):
    """The mapping covers the entire compiled control vector."""
    ids = [
        descriptor.actuator_id
        for descriptor in registry.describe()
    ]

    assert sorted(ids) == list(range(8))


def test_registry_control_range_matches_model(registry, sim):
    """Capture actual compiled range metadata, not guessed limits."""
    for descriptor in registry.describe():
        actuator_id = descriptor.actuator_id
        limited = bool(
            sim.model.actuator_ctrllimited[actuator_id]
        )

        assert descriptor.control_limited is limited

        if limited:
            assert descriptor.control_range == (
                float(
                    sim.model.actuator_ctrlrange[
                        actuator_id, 0
                    ]
                ),
                float(
                    sim.model.actuator_ctrlrange[
                        actuator_id, 1
                    ]
                ),
            )
        else:
            assert descriptor.control_range is None


def test_registry_metadata_is_immutable(registry):
    """Callers cannot reassign descriptor fields."""
    descriptor = registry.describe()[0]

    with pytest.raises(FrozenInstanceError):
        descriptor.actuator_id = 99


def test_registry_mapping_matches_initial_ctrl(
    registry,
    sim,
    valid_command,
):
    """Command conversion reconstructs the actual MuJoCo buffer."""
    vector = registry.to_vector(valid_command)

    assert vector.shape == (sim.model.nu,)
    assert vector.dtype == np.float64

    np.testing.assert_array_equal(
        vector,
        sim.data.ctrl,
    )


def test_command_conversion_uses_ids_not_descriptor_order(
    registry,
    valid_command,
):
    """Registry enumeration order must not control output positions."""
    reordered = ActuatorRegistry(
        descriptors=tuple(
            reversed(registry.describe())
        )
    )

    expected = registry.to_vector(valid_command)
    actual = reordered.to_vector(valid_command)

    np.testing.assert_array_equal(actual, expected)


def test_every_channel_maps_to_correct_actuator(registry, sim):
    """Distinct command values must not be permuted accidentally."""
    # Use each actuator's current legal command as a baseline.
    values = {}

    for logical_name, mujoco_name in PANDA_ACTUATORS:
        actuator_id = sim.model.actuator(mujoco_name).id
        values[logical_name] = float(
            sim.data.ctrl[actuator_id]
        )

    # Use a small distinguishable change only where it is
    # known to remain within an actuator's compiled limits.
    for descriptor in registry.describe():
        candidate = dict(values)

        if descriptor.control_limited:
            low, high = descriptor.control_range
            candidate[descriptor.logical_name] = (
                low + high
            ) / 2.0
        else:
            candidate[descriptor.logical_name] = 0.25

        command = PandaCommand(**candidate)
        vector = registry.to_vector(command)

        assert vector[descriptor.actuator_id] == pytest.approx(
            candidate[descriptor.logical_name]
        )

        # Every other position retains its baseline command.
        for other in registry.describe():
            if other.actuator_id == descriptor.actuator_id:
                continue

            assert vector[other.actuator_id] == pytest.approx(
                values[other.logical_name]
            )


def test_conversion_returns_independent_arrays(
    registry,
    valid_command,
):
    """Two conversions must not share mutable vector storage."""
    first = registry.to_vector(valid_command)
    second = registry.to_vector(valid_command)

    assert not np.shares_memory(first, second)

    first[0] = 999.0

    assert second[0] != 999.0
    assert valid_command.arm1 != 999.0


def test_conversion_rejects_wrong_command_type(registry):
    """No dictionary can bypass the typed command interface."""
    with pytest.raises(TypeError):
        registry.to_vector({"arm1": 0.0})


def test_invalid_registry_rejects_duplicate_actuator_ids(
    registry,
    valid_command,
):
    """A malformed mapping must never overwrite another channel."""
    original = list(registry.describe())

    original[1] = replace(
        original[1],
        actuator_id=original[0].actuator_id,
    )

    malformed = ActuatorRegistry(
        descriptors=tuple(original)
    )

    with pytest.raises(ValueError, match="mapping"):
        malformed.to_vector(valid_command)


def test_invalid_registry_rejects_missing_channel(
    registry,
    valid_command,
):
    """Conversion must not silently leave vector entries absent."""
    malformed = ActuatorRegistry(
        descriptors=registry.describe()[:-1]
    )

    with pytest.raises(ValueError):
        malformed.to_vector(valid_command)


def test_invalid_registry_rejects_unknown_logical_name(
    registry,
    valid_command,
):
    """Unexpected logical channel names must fail conversion."""
    descriptors = list(registry.describe())
    descriptors[0] = replace(
        descriptors[0],
        logical_name="unrecognised",
    )

    malformed = ActuatorRegistry(
        descriptors=tuple(descriptors)
    )

    with pytest.raises(ValueError, match="mapping"):
        malformed.to_vector(valid_command)


def test_range_violation_is_not_clamped(
    registry,
    valid_command,
):
    """P0 commands use reject-not-clamp semantics."""
    limited = next(
        descriptor
        for descriptor in registry.describe()
        if descriptor.control_limited
    )

    values = valid_command.as_named_values()
    values[limited.logical_name] = (
        limited.control_range[1] + 1.0
    )

    command = PandaCommand(**values)

    with pytest.raises(ValueError, match="outside"):
        registry.to_vector(command)


def test_gripper_range_comes_from_model(registry):
    """Gripper command range is not interpreted as metres."""
    descriptor = next(
        d for d in registry.describe()
        if d.logical_name == "gripper"
    )

    assert descriptor.mujoco_name == "actuator8"
    assert descriptor.control_limited
    assert descriptor.control_range == (0.0, 255.0)


def test_conversion_does_not_modify_mujoco_state(
    registry,
    sim,
    valid_command,
):
    """Constructing a vector alone is not simulation execution."""
    original_ctrl = sim.data.ctrl.copy()
    original_time = float(sim.data.time)
    original_physics = sim.physics_step_count
    original_controller = sim.controller_step_count

    registry.to_vector(valid_command)

    np.testing.assert_array_equal(
        sim.data.ctrl,
        original_ctrl,
    )
    assert sim.data.time == original_time
    assert sim.physics_step_count == original_physics
    assert sim.controller_step_count == original_controller


def test_existing_simulator_accepts_registry_vector(
    registry,
    sim,
    valid_command,
):
    """The new interface integrates without changing Simulator."""
    vector = registry.to_vector(valid_command)

    sim.step(
        control=vector,
        n_substeps=1,
    )

    np.testing.assert_array_equal(
        sim.data.ctrl,
        vector,
    )

    assert sim.physics_step_count == 1
    assert sim.controller_step_count == 0


def test_controller_period_accepts_registry_vector(
    registry,
    sim,
    valid_command,
):
    """Controller stepping remains compatible with typed commands."""
    vector = registry.to_vector(valid_command)

    sim.step_controller_period(
        control=vector,
        period=0.04,
    )

    assert sim.physics_step_count == 20
    assert sim.controller_step_count == 1

    np.testing.assert_array_equal(
        sim.data.ctrl,
        vector,
    )
