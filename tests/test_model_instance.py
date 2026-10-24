
"""P0-005.02 — Independent MuJoCo model/state ownership tests.

The tests use temporary MJCF fixtures for standalone model loading.
A real Panda asset integration test is included separately.
"""

import gc
import os
import weakref
from pathlib import Path

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

    from dataclasses import FrozenInstanceError

    with pytest.raises(FrozenInstanceError):
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
