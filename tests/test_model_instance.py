
"""P0-005.02 — Independent MuJoCo model/state ownership tests.

The tests use temporary MJCF fixtures for standalone model loading.
A real Panda asset integration test is included separately.

Further regression tests to ensure independent ownership of model 
and state are included below.
"""

import gc
import os
import weakref
from pathlib import Path
import copy
import hashlib
import pickle
from unittest.mock import patch
import numpy as np
import mujoco
import pytest

from robot_arm_sim.model_instance import SimulationInstance


MODEL_XML = """
<mujoco model="independent_instance_test">
  <option timestep="0.002"/>

  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge"
             axis="0 1 0"
             limited="true"
             range="-1 1"/>
      <geom name="link_geom"
            type="box"
            size="0.05 0.05 0.05"
            mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="motor"
           joint="hinge"
           ctrllimited="true"
           ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"""

@pytest.mark.skipif(
    not os.environ.get("ROBOT_ARM_PANDA_XML"),
    reason="Actual Panda model path not configured",
)
def test_panda_multi_instance_state_isolation() -> None:
    """Verify independent state in two real composed robot scenes.

    Tests joint coordinates, actuator commands, ball state,
    compiled model parameters and physics clock independence.

    No robot assets are modified.
    """
    path = Path(os.environ["ROBOT_ARM_PANDA_XML"])

    a = SimulationInstance.from_robot_scene(path)
    b = SimulationInstance.from_robot_scene(path)

    assert a.model is not b.model
    assert a.data is not b.data

    # Preserve snapshots of all important independent state.
    b_qpos = b.data.qpos.copy()
    b_qvel = b.data.qvel.copy()
    b_ctrl = b.data.ctrl.copy()
    b_mass = b.model.body_mass.copy()
    b_time = float(b.data.time)

    joint_id = mujoco.mj_name2id(
        a.model,
        mujoco.mjtObj.mjOBJ_JOINT,
        "joint1",
    )
    assert joint_id >= 0

    joint_address = int(a.model.jnt_qposadr[joint_id])

    # A small perturbation probes buffer isolation only.
    # It is not a control, reachability or collision-safety test.
    a.data.qpos[joint_address] += 0.001

    # Verify an independent actuator buffer is available.
    assert a.model.nu > 0
    a.data.ctrl[0] += 0.001

    # Modify A's compiled parameter without changing its XML.
    mass_body = next(
        i for i in range(1, a.model.nbody)
        if a.model.body_mass[i] > 0
    )
    a.model.body_mass[mass_body] *= 1.001

    # Check B has retained its complete initial snapshots.
    np.testing.assert_array_equal(b.data.qpos, b_qpos)
    np.testing.assert_array_equal(b.data.qvel, b_qvel)
    np.testing.assert_array_equal(b.data.ctrl, b_ctrl)
    np.testing.assert_array_equal(b.model.body_mass, b_mass)
    assert b.data.time == b_time

    # Advance only A. B must not advance.
    initial_a_time = float(a.data.time)
    mujoco.mj_forward(a.model, a.data)
    mujoco.mj_step(a.model, a.data)

    assert a.data.time == pytest.approx(
        initial_a_time + a.model.opt.timestep
    )
    assert b.data.time == b_time


@pytest.fixture
def model_path(tmp_path: Path) -> Path:
    """Provide a valid, isolated model file."""
    path = tmp_path / "model.xml"
    path.write_text(MODEL_XML, encoding="utf-8")
    return path


def test_instance_contains_correct_types(
    model_path: Path,
) -> None:
    instance = SimulationInstance.from_model_file(model_path)

    assert isinstance(instance.model, mujoco.MjModel)
    assert isinstance(instance.data, mujoco.MjData)


def test_separate_instances_have_distinct_models(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    assert a is not b
    assert a.model is not b.model
    assert a.data is not b.data


def test_initial_states_are_equivalent(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    assert list(a.data.qpos) == pytest.approx(list(b.data.qpos))
    assert list(a.data.qvel) == pytest.approx(list(b.data.qvel))
    assert list(a.data.ctrl) == pytest.approx(list(b.data.ctrl))
    assert a.data.time == pytest.approx(b.data.time)


def test_joint_positions_are_independent(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    a.data.qpos[0] = 0.5

    assert a.data.qpos[0] == pytest.approx(0.5)
    assert b.data.qpos[0] == pytest.approx(0.0)


def test_joint_velocities_are_independent(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    a.data.qvel[0] = 0.25

    assert b.data.qvel[0] == pytest.approx(0.0)


def test_actuator_commands_are_independent(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    a.data.ctrl[0] = 0.75

    assert a.data.ctrl[0] == pytest.approx(0.75)
    assert b.data.ctrl[0] == pytest.approx(0.0)


def test_mutable_model_properties_are_independent(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    original = b.model.opt.timestep
    a.model.opt.timestep = 0.005

    assert a.model.opt.timestep == pytest.approx(0.005)
    assert b.model.opt.timestep == pytest.approx(original)


def test_stepping_one_instance_does_not_step_another(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    mujoco.mj_step(a.model, a.data)

    assert a.data.time == pytest.approx(0.002)
    assert b.data.time == pytest.approx(0.0)


def test_repeated_stepping_remains_isolated(
    model_path: Path,
) -> None:
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    for _ in range(100):
        mujoco.mj_step(a.model, a.data)

    assert a.data.time == pytest.approx(0.2)
    assert b.data.time == pytest.approx(0.0)
    assert all(abs(float(x)) < float("inf") for x in a.data.qpos)


def test_attribute_reassignment_is_rejected(
    model_path: Path,
) -> None:
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises(AttributeError, match="ownership references are read-only"):
        instance.model = None  # type: ignore[assignment]


def test_instance_keeps_model_and_data_alive(
    model_path: Path,
) -> None:
    instance = SimulationInstance.from_model_file(model_path)

    # Garbage collection must not invalidate the objects kept by
    # the owning instance.
    gc.collect()

    assert instance.model.njnt == 1
    assert instance.data.qpos.shape == (1,)


def test_multiple_instances_remain_usable(
    model_path: Path,
) -> None:
    instances = [
        SimulationInstance.from_model_file(model_path)
        for _ in range(8)
    ]

    assert len({id(i.model) for i in instances}) == 8
    assert len({id(i.data) for i in instances}) == 8

    for index, instance in enumerate(instances):
        instance.data.ctrl[0] = index / 10.0

    for index, instance in enumerate(instances):
        assert instance.data.ctrl[0] == pytest.approx(index / 10.0)


def test_loading_has_no_physics_time_advance(
    model_path: Path,
) -> None:
    instance = SimulationInstance.from_model_file(model_path)

    assert instance.data.time == pytest.approx(0.0)


def test_source_model_remains_unchanged(
    model_path: Path,
) -> None:
    original = model_path.read_bytes()

    SimulationInstance.from_model_file(model_path)

    assert model_path.read_bytes() == original


def test_invalid_file_propagates_loader_error(
    tmp_path: Path,
) -> None:
    missing = tmp_path / "missing.xml"

    with pytest.raises(ValueError, match="missing.xml"):
        SimulationInstance.from_model_file(missing)


@pytest.mark.skipif(
    not os.environ.get("ROBOT_ARM_PANDA_XML"),
    reason="Optional local Panda model not configured",
)
def test_real_panda_instances_are_independent() -> None:
    """Verify ownership through the production scene builder."""
    path = Path(os.environ["ROBOT_ARM_PANDA_XML"])

    a = SimulationInstance.from_robot_scene(path)
    b = SimulationInstance.from_robot_scene(path)

    assert a.model is not b.model
    assert a.data is not b.data

    assert a.model.nu == b.model.nu
    assert a.model.njnt == b.model.njnt

    # Mutate only one robot's state without bypassing joint addressing.
    joint_id = mujoco.mj_name2id(
        a.model, mujoco.mjtObj.mjOBJ_JOINT, "joint1"
    )
    assert joint_id >= 0

    qpos_address = a.model.jnt_qposadr[joint_id]
    original = float(b.data.qpos[qpos_address])
    a.data.qpos[qpos_address] = original + 0.01

    assert b.data.qpos[qpos_address] == pytest.approx(original)

def test_public_constructor_rejected() -> None:
    """A caller must not create an instance without a factory."""
    with pytest.raises(TypeError, match="from_model_file"):
        SimulationInstance()


@pytest.mark.parametrize(
    "attribute",
    ["model", "data", "_model", "_data", "_sealed", "extra"],
)
def test_all_ordinary_attribute_writes_rejected(
    model_path: Path,
    attribute: str,
) -> None:
    """Public and internal ownership slots must reject reassignment."""
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises((AttributeError, TypeError)):
        setattr(instance, attribute, None)


@pytest.mark.parametrize(
    "attribute",
    ["model", "data", "_model", "_data", "_sealed"],
)
def test_ownership_attribute_deletion_rejected(
    model_path: Path,
    attribute: str,
) -> None:
    """Deleting references must not corrupt an instance."""
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises((AttributeError, TypeError)):
        delattr(instance, attribute)

    assert isinstance(instance.model, mujoco.MjModel)
    assert isinstance(instance.data, mujoco.MjData)


def test_shallow_copy_is_rejected(
    model_path: Path,
) -> None:
    """A shallow copy must not create two owners of one state."""
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises(TypeError, match="cannot be copied"):
        copy.copy(instance)


def test_deep_copy_is_rejected(
    model_path: Path,
) -> None:
    """An implicit deep clone must not bypass model creation."""
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises(TypeError, match="deep-copied"):
        copy.deepcopy(instance)


def test_pickle_roundtrip_is_rejected(
    model_path: Path,
) -> None:
    """Deserialisation must not silently create a shared owner."""
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises(TypeError, match="cannot be pickled"):
        pickle.dumps(instance)


def test_independent_arrays_do_not_share_memory(
    model_path: Path,
) -> None:
    """Separate instances must not alias native state buffers."""
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    for field in ("qpos", "qvel", "ctrl"):
        first = getattr(a.data, field)
        second = getattr(b.data, field)

        assert first is not second
        assert not np.shares_memory(first, second)


def test_state_mutation_does_not_cross_instances(
    model_path: Path,
) -> None:
    """Mutate several state arrays in A and verify B is unchanged."""
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    before_qpos = b.data.qpos.copy()
    before_qvel = b.data.qvel.copy()
    before_ctrl = b.data.ctrl.copy()
    before_time = float(b.data.time)

    a.data.qpos[:] = 0.4
    a.data.qvel[:] = -0.2
    a.data.ctrl[:] = 0.6
    a.data.time = 12.5

    np.testing.assert_array_equal(b.data.qpos, before_qpos)
    np.testing.assert_array_equal(b.data.qvel, before_qvel)
    np.testing.assert_array_equal(b.data.ctrl, before_ctrl)
    assert b.data.time == before_time


def test_reverse_direction_isolation(
    model_path: Path,
) -> None:
    """Ownership independence must hold in both directions."""
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    before = a.data.qpos.copy()

    b.data.qpos[0] = -0.7
    b.data.ctrl[0] = -0.8

    np.testing.assert_array_equal(a.data.qpos, before)
    assert a.data.ctrl[0] == pytest.approx(0.0)


def test_mutable_model_arrays_are_independent(
    model_path: Path,
) -> None:
    """Compiled model parameters must not be shared."""
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    before = b.model.body_mass.copy()

    a.model.body_mass[1] *= 1.5

    np.testing.assert_array_equal(b.model.body_mass, before)
    assert a.model.body_mass[1] != b.model.body_mass[1]


def test_repeated_creation_does_not_reuse_models(
    model_path: Path,
) -> None:
    """Create many simultaneously live independent instances."""
    instances = [
        SimulationInstance.from_model_file(model_path)
        for _ in range(32)
    ]

    assert len({id(x.model) for x in instances}) == 32
    assert len({id(x.data) for x in instances}) == 32

    for index, instance in enumerate(instances):
        instance.data.time = index + 0.25

    for index, instance in enumerate(instances):
        assert instance.data.time == pytest.approx(index + 0.25)


def test_gc_does_not_corrupt_surviving_instance(
    model_path: Path,
) -> None:
    """Releasing one owner must not invalidate another owner."""
    survivor = SimulationInstance.from_model_file(model_path)
    temporary = SimulationInstance.from_model_file(model_path)

    temporary.data.ctrl[0] = 0.9

    del temporary
    gc.collect()

    assert survivor.model.njnt == 1
    assert survivor.data.ctrl[0] == pytest.approx(0.0)

    mujoco.mj_step(survivor.model, survivor.data)
    assert survivor.data.time == pytest.approx(0.002)


def test_failed_load_produces_no_instance(
    model_path: Path,
) -> None:
    """A failing dependency must not return a partial owner."""
    from robot_arm_sim.model_loader import ModelLoadError

    with patch(
        "robot_arm_sim.model_instance.load_model",
        side_effect=ModelLoadError("injected load failure"),
    ) as loader:
        with pytest.raises(ModelLoadError, match="injected"):
            SimulationInstance.from_model_file(model_path)

    loader.assert_called_once_with(model_path)


def test_failed_scene_creation_produces_no_instance(
    model_path: Path,
) -> None:
    """The factory must propagate scene validation errors."""
    with patch(
        "robot_arm_sim.model_instance.build_scene",
        side_effect=ValueError("injected scene failure"),
    ) as builder:
        with pytest.raises(ValueError, match="injected"):
            SimulationInstance.from_robot_scene(model_path)

    builder.assert_called_once_with(model_path)


def test_invalid_internal_pair_types_rejected(
    model_path: Path,
) -> None:
    """Internal creation must reject objects of the wrong type."""
    instance = SimulationInstance.from_model_file(model_path)

    with pytest.raises(TypeError, match="MjModel"):
        SimulationInstance._create(None, instance.data)

    with pytest.raises(TypeError, match="MjData"):
        SimulationInstance._create(instance.model, None)


def test_source_file_hash_remains_unchanged(
    model_path: Path,
) -> None:
    """Allocation and stepping must not modify the MJCF source."""
    before = hashlib.sha256(model_path.read_bytes()).digest()

    instance = SimulationInstance.from_model_file(model_path)

    for _ in range(100):
        mujoco.mj_step(instance.model, instance.data)

    after = hashlib.sha256(model_path.read_bytes()).digest()
    assert after == before


def test_parallel_interleaved_simulations_isolated(
    model_path: Path,
) -> None:
    """Interleaved sequential stepping must not couple states.

    This test is deliberately single-threaded. It verifies isolation,
    not concurrent thread safety.
    """
    a = SimulationInstance.from_model_file(model_path)
    b = SimulationInstance.from_model_file(model_path)

    a.data.ctrl[0] = 0.5
    b.data.ctrl[0] = -0.5

    for _ in range(200):
        mujoco.mj_step(a.model, a.data)
        mujoco.mj_step(b.model, b.data)

    assert a.data.time == pytest.approx(0.4)
    assert b.data.time == pytest.approx(0.4)

    assert np.all(np.isfinite(a.data.qpos))
    assert np.all(np.isfinite(b.data.qpos))
    assert np.all(np.isfinite(a.data.qvel))
    assert np.all(np.isfinite(b.data.qvel))

    assert abs(a.data.qpos[0] - b.data.qpos[0]) > 1e-5


def test_subclass_factory_is_rejected(
    model_path: Path,
) -> None:
    """A subclass cannot silently change construction semantics."""
    DerivedInstance = type("DerivedInstance", (SimulationInstance,), {})

    with pytest.raises(TypeError, match="Subclass"):
        DerivedInstance.from_model_file(model_path)
    with pytest.raises(TypeError, match="Subclass"):
        DerivedInstance.from_robot_scene(model_path)

