
"""P0-006.05 — Physics stepping semantics.

Verify that Simulator.step() advances MuJoCo by exactly
the requested number of physics steps, using validated
actuator controls and no application-level behaviour policy.
"""

import os
from pathlib import Path
import mujoco.viewer
import mujoco
import numpy as np
import pytest

from robot_arm_sim.simulator import (
    Simulator,
    SimulatorLifecycleError,
    SimulatorState,
)


@pytest.fixture
def sim():
    """Create one isolated real Panda simulation per test."""
    source = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not source:
        pytest.fail(
            "ROBOT_ARM_PANDA_XML is required for stepping tests"
        )

    simulator = Simulator(Path(source))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


def test_one_step_advances_one_physics_timestep(sim):
    """Default stepping means one MuJoCo integrator step."""
    start = float(sim.data.time)
    dt = float(sim.model.opt.timestep)

    sim.step()

    assert sim.data.time - start == pytest.approx(dt)


@pytest.mark.parametrize(
    "n_substeps",
    [1, 2, 5, 20, 40],
)
def test_time_advances_by_exact_substep_count(sim, n_substeps):
    """Elapsed time is proportional to the selected count."""
    start = float(sim.data.time)
    dt = float(sim.model.opt.timestep)

    sim.step(n_substeps=n_substeps)

    assert sim.data.time - start == pytest.approx(
        n_substeps * dt,
        rel=1e-10,
        abs=1e-12,
    )


def test_stepping_twice_accumulates_physics_time(sim):
    """Successive calls advance the same simulation clock."""
    dt = float(sim.model.opt.timestep)
    start = float(sim.data.time)

    sim.step(n_substeps=3)
    sim.step(n_substeps=7)

    assert sim.data.time - start == pytest.approx(
        10 * dt,
        rel=1e-10,
        abs=1e-12,
    )


def test_step_uses_loaded_model_timestep(sim):
    """Execution must read model.opt.timestep, not hardcode 0.002."""
    sim.model.opt.timestep = 0.001

    start = float(sim.data.time)

    sim.step(n_substeps=4)

    assert sim.data.time - start == pytest.approx(0.004)


def test_step_advances_ball_under_gravity(sim):
    """The physics integrator must evolve the free ball."""
    sim.reset()

    ball_id = sim.model.body("ball").id
    initial_z = float(sim.data.xpos[ball_id, 2])

    sim.step(n_substeps=20)

    assert float(sim.data.xpos[ball_id, 2]) < initial_z


def test_no_control_preserves_existing_command(sim):
    """control=None must not overwrite current actuator values."""
    before = sim.data.ctrl.copy()

    sim.step(control=None)

    np.testing.assert_array_equal(sim.data.ctrl, before)


def test_valid_control_vector_is_written(sim):
    """A full numeric command is applied before stepping."""
    command = sim.data.ctrl.copy()

    # The existing command provides a known valid baseline.
    sim.step(control=command, n_substeps=1)

    np.testing.assert_array_equal(sim.data.ctrl, command)


def test_valid_gripper_control_is_not_treated_as_metres(sim):
    """The eighth actuator uses the model's documented units."""
    command = sim.data.ctrl.copy()
    gripper = sim.model.actuator("actuator8").id

    assert sim.model.actuator_ctrllimited[gripper]

    # This model has the verified [0, 255] control range.
    command[gripper] = 255.0

    sim.step(control=command)

    assert sim.data.ctrl[gripper] == pytest.approx(255.0)


def test_numpy_array_and_list_controls_are_accepted(sim):
    """Ordinary full-length actuator vectors are supported."""
    baseline = sim.data.ctrl.copy()

    sim.step(control=baseline.tolist())
    np.testing.assert_array_equal(sim.data.ctrl, baseline)

    sim.step(control=baseline)
    np.testing.assert_array_equal(sim.data.ctrl, baseline)


def test_input_command_is_copied_not_modified(sim):
    """Writing controls to MjData must not mutate caller storage."""
    command = sim.data.ctrl.copy()
    before = command.copy()

    sim.step(control=command)

    np.testing.assert_array_equal(command, before)


@pytest.mark.parametrize("count", [0, -1, -20])
def test_nonpositive_substeps_rejected_without_stepping(sim, count):
    """Invalid iteration counts cannot change simulation time."""
    before = float(sim.data.time)

    with pytest.raises(ValueError, match="n_substeps"):
        sim.step(n_substeps=count)

    assert sim.data.time == before


@pytest.mark.parametrize(
    "count",
    [True, False, 1.0, "2", None, np.int64(2)],
)
def test_invalid_substep_types_rejected(sim, count):
    """Booleans and implicit integer conversions are prohibited."""
    before = float(sim.data.time)

    with pytest.raises(TypeError, match="n_substeps"):
        sim.step(n_substeps=count)

    assert sim.data.time == before


@pytest.mark.parametrize(
    "invalid",
    [
        [],
        [0.0],
        np.zeros(7),
        np.zeros(9),
    ],
)
def test_wrong_control_shape_rejected(sim, invalid):
    """Raw command length must equal model.nu exactly."""
    start = float(sim.data.time)
    original = sim.data.ctrl.copy()

    with pytest.raises(ValueError, match="shape"):
        sim.step(control=invalid)

    assert sim.data.time == start
    np.testing.assert_array_equal(sim.data.ctrl, original)


@pytest.mark.parametrize(
    "invalid",
    [
        np.zeros((1, 8)),
        np.zeros((8, 1)),
    ],
)
def test_multidimensional_control_rejected(sim, invalid):
    """Actuator commands must be one-dimensional."""
    with pytest.raises(ValueError, match="one-dimensional"):
        sim.step(control=invalid)


@pytest.mark.parametrize(
    "invalid",
    [
        np.full(8, np.nan),
        np.full(8, np.inf),
        np.full(8, -np.inf),
    ],
)
def test_nonfinite_controls_rejected(sim, invalid):
    """All actuator values must be finite."""
    start = float(sim.data.time)
    original = sim.data.ctrl.copy()

    with pytest.raises(ValueError, match="NaN or infinity"):
        sim.step(control=invalid)

    assert sim.data.time == start
    np.testing.assert_array_equal(sim.data.ctrl, original)


@pytest.mark.parametrize(
    "invalid",
    [
        np.ones(8, dtype=bool),
        ["0"] * 8,
        ["invalid"] * 8,
    ],
)
def test_non_numeric_control_vectors_rejected(sim, invalid):
    """No silent boolean or string-to-number conversion."""
    with pytest.raises(TypeError, match="numeric"):
        sim.step(control=invalid)


@pytest.mark.parametrize("invalid", [True, 1.5, "123", {}])
def test_invalid_control_representations_rejected(sim, invalid):
    """Reject ambiguous controls and scalar representations."""
    with pytest.raises((TypeError, ValueError)):
        sim.step(control=invalid)


def test_out_of_range_gripper_control_rejected(sim):
    """The gripper actuator cannot exceed its compiled limits."""
    original = sim.data.ctrl.copy()
    command = original.copy()

    gripper = sim.model.actuator("actuator8").id
    command[gripper] = 256.0

    with pytest.raises(ValueError, match="actuator8"):
        sim.step(control=command)

    np.testing.assert_array_equal(sim.data.ctrl, original)


def test_negative_gripper_control_rejected(sim):
    """Lower gripper bounds must also be enforced."""
    command = sim.data.ctrl.copy()
    gripper = sim.model.actuator("actuator8").id

    command[gripper] = -1.0

    with pytest.raises(ValueError, match="actuator8"):
        sim.step(control=command)


def test_invalid_control_does_not_partially_write(sim):
    """Failure for a late actuator must preserve earlier controls."""
    original = sim.data.ctrl.copy()
    command = original.copy()

    command[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step(control=command)

    np.testing.assert_array_equal(sim.data.ctrl, original)


@pytest.mark.parametrize(
    "invalid_dt",
    [0.0, -0.002, float("nan"), float("inf")],
)
def test_invalid_timestep_rejected(sim, invalid_dt):
    """No physics integration with an invalid timestep."""
    sim.model.opt.timestep = invalid_dt
    start = float(sim.data.time)

    with pytest.raises(ValueError, match="timestep"):
        sim.step()

    assert sim.data.time == start


def test_model_data_identity_preserved_during_stepping(sim):
    """Stepping must not rebuild the robot or replace data."""
    model = sim.model
    data = sim.data

    sim.step(n_substeps=5)

    assert sim.model is model
    assert sim.data is data


def test_step_invokes_mujoco_exactly_n_times(sim, monkeypatch):
    """The number of integration calls must equal n_substeps."""
    original_step = mujoco.mj_step
    calls = []

    def recording_step(model, data):
        calls.append((model, data))
        return original_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", recording_step)

    sim.step(n_substeps=7)

    assert len(calls) == 7
    assert all(
        model is sim.model and data is sim.data
        for model, data in calls
    )


def test_invalid_control_never_invokes_mujoco_step(
    sim,
    monkeypatch,
):
    """Validation must happen before the first physics step."""
    def forbidden_step(*args, **kwargs):
        raise AssertionError(
            "Invalid input reached mujoco.mj_step"
        )

    monkeypatch.setattr(mujoco, "mj_step", forbidden_step)

    command = sim.data.ctrl.copy()
    command[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step(control=command)


def test_control_is_held_constant_across_substeps(
    sim,
    monkeypatch,
):
    """One supplied control vector must be used for every step."""
    command = sim.data.ctrl.copy()
    command[-1] = 255.0

    observed = []
    original_step = mujoco.mj_step

    def recording_step(model, data):
        observed.append(data.ctrl.copy())
        return original_step(model, data)

    monkeypatch.setattr(mujoco, "mj_step", recording_step)

    sim.step(control=command, n_substeps=5)

    assert len(observed) == 5

    for actual in observed:
        np.testing.assert_array_equal(actual, command)


def test_reset_after_stepping_restores_initial_time_and_state(sim):
    """The prior reset contract must still work after integration."""
    original_qpos = sim.data.qpos.copy()
    original_qvel = sim.data.qvel.copy()
    original_time = float(sim.data.time)

    sim.step(n_substeps=15)
    assert sim.data.time > original_time

    sim.reset()

    np.testing.assert_array_equal(sim.data.qpos, original_qpos)
    np.testing.assert_array_equal(sim.data.qvel, original_qvel)
    assert sim.data.time == original_time


def test_multiple_instances_advance_independently(sim):
    """One simulation must never advance another's clock."""
    source = os.environ["ROBOT_ARM_PANDA_XML"]
    second = Simulator(Path(source))
    second.load()

    try:
        original_second_time = float(second.data.time)

        sim.step(n_substeps=5)

        assert second.data.time == original_second_time
        assert sim.data.time > original_second_time
    finally:
        second.close()


def test_step_before_load_is_rejected():
    """Existing lifecycle guards remain in force."""
    sim = Simulator("unused.xml")

    with pytest.raises(SimulatorLifecycleError):
        sim.step()


def test_step_after_close_is_rejected(sim):
    """A closed simulator cannot continue physics integration."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.step()


def test_step_does_not_create_viewer(sim, monkeypatch):
    """Stepping has no GUI dependency."""
    def forbidden_viewer(*args, **kwargs):
        raise AssertionError("Viewer creation is prohibited")

    monkeypatch.setattr(
        mujoco.viewer,
        "launch",
        forbidden_viewer,
    )

    sim.step(n_substeps=2)


def test_none_control_does_not_attempt_array_conversion(
    sim,
    monkeypatch,
):
    """Omitted control must bypass NumPy command conversion.

    This catches accidental indentation changes that cause
    None to be processed as a zero-dimensional array.
    """
    original_asarray = np.asarray

    def guarded_asarray(value, *args, **kwargs):
        if value is None:
            raise AssertionError(
                "control=None must not be converted "
                "to a NumPy array"
            )
        return original_asarray(value, *args, **kwargs)

    monkeypatch.setattr(
        np,
        "asarray",
        guarded_asarray,
    )

    original_ctrl = sim.data.ctrl.copy()
    original_time = float(sim.data.time)

    sim.step(control=None, n_substeps=3)

    np.testing.assert_array_equal(
        sim.data.ctrl,
        original_ctrl,
    )
    assert sim.data.time > original_time
