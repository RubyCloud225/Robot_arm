
"""P0-006.08 — Simulator lifecycle and input guards.

Verify invalid requests fail before actuator writes and
physics integration, while preserving episode accounting.

The tests use the real Panda model in headless mode.
"""

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_arm_sim.simulator import (
    Simulator,
    SimulatorLifecycleError,
)


@pytest.fixture
def sim():
    """Create one independently loaded simulator per test."""
    source = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not source:
        pytest.fail(
            "ROBOT_ARM_PANDA_XML is required"
        )

    instance = Simulator(Path(source))
    instance.load()

    try:
        yield instance
    finally:
        instance.close()


def capture_state(sim):
    """Snapshot externally visible state before rejection."""
    return (
        sim.data.qpos.copy(),
        sim.data.qvel.copy(),
        sim.data.ctrl.copy(),
        float(sim.data.time),
        sim.physics_step_count,
        sim.controller_step_count,
    )


def assert_state_unchanged(sim, before):
    """An invalid request must not mutate simulation state."""
    qpos, qvel, ctrl, time, physics, controller = before

    np.testing.assert_array_equal(sim.data.qpos, qpos)
    np.testing.assert_array_equal(sim.data.qvel, qvel)
    np.testing.assert_array_equal(sim.data.ctrl, ctrl)

    assert sim.data.time == time
    assert sim.physics_step_count == physics
    assert sim.controller_step_count == controller


@pytest.mark.parametrize(
    "operation",
    [
        lambda s: s.reset(),
        lambda s: s.step(),
        lambda s: s.step(n_substeps=3),
        lambda s: s.step_controller_period(period=0.04),
    ],
)
def test_operations_before_load_rejected(operation):
    """State-changing operations require successful load()."""
    simulator = Simulator("unused.xml")

    with pytest.raises(SimulatorLifecycleError):
        operation(simulator)


@pytest.mark.parametrize(
    "operation",
    [
        lambda s: s.reset(),
        lambda s: s.step(),
        lambda s: s.step(n_substeps=3),
        lambda s: s.step_controller_period(period=0.04),
    ],
)
def test_operations_after_close_rejected(sim, operation):
    """Closing permanently disables simulation operations."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        operation(sim)


def test_load_after_close_is_rejected(sim):
    """A closed wrapper must not silently rebuild a model."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.load()


def test_duplicate_load_is_rejected_without_mutation(sim):
    """Loading twice must not replace a working simulation."""
    before = capture_state(sim)
    model = sim.model
    data = sim.data

    with pytest.raises(SimulatorLifecycleError):
        sim.load()

    assert sim.model is model
    assert sim.data is data
    assert_state_unchanged(sim, before)


@pytest.mark.parametrize(
    "count",
    [0, -1, -100],
)
def test_nonpositive_substeps_preserve_state(sim, count):
    """Illegal iteration counts cannot initiate integration."""
    before = capture_state(sim)

    with pytest.raises(ValueError, match="n_substeps"):
        sim.step(n_substeps=count)

    assert_state_unchanged(sim, before)


@pytest.mark.parametrize(
    "count",
    [True, False, 1.0, "20", None, np.int64(20)],
)
def test_invalid_substep_types_preserve_state(sim, count):
    """Reject implicit conversions to Python int."""
    before = capture_state(sim)

    with pytest.raises(TypeError, match="n_substeps"):
        sim.step(n_substeps=count)

    assert_state_unchanged(sim, before)


@pytest.mark.parametrize(
    "command",
    [
        np.zeros(0),
        np.zeros(7),
        np.zeros(9),
        np.zeros((1, 8)),
        np.zeros((8, 1)),
    ],
)
def test_invalid_action_shapes_preserve_state(sim, command):
    """Actuator command dimensions must equal (model.nu,)."""
    before = capture_state(sim)

    with pytest.raises(ValueError):
        sim.step(control=command)

    assert_state_unchanged(sim, before)


@pytest.mark.parametrize(
    "command",
    [
        np.ones(8, dtype=bool),
        ["0"] * 8,
        np.zeros(8, dtype=complex),
    ],
)
def test_invalid_action_dtypes_preserve_state(sim, command):
    """Reject boolean, string and complex commands."""
    before = capture_state(sim)

    with pytest.raises(TypeError, match="numeric"):
        sim.step(control=command)

    assert_state_unchanged(sim, before)


@pytest.mark.parametrize(
    "invalid",
    [float("nan"), float("inf"), float("-inf")],
)
def test_nonfinite_supplied_control_preserves_state(sim, invalid):
    """Nonfinite commands cannot initiate physics stepping."""
    command = sim.data.ctrl.copy()
    command[-1] = invalid
    before = capture_state(sim)

    with pytest.raises(ValueError):
        sim.step(control=command)

    assert_state_unchanged(sim, before)



def test_oversized_numeric_control_is_rejected(sim):
    """Reject integer controls beyond float64 representation.

    NumPy may represent an extremely large Python integer
    using an object dtype rather than a numeric dtype.

    The simulator may reject that representation with
    TypeError or reject numeric conversion with ValueError.

    Both are acceptable provided the command is rejected
    before any simulation state changes occur.
    """
    command = sim.data.ctrl.copy().tolist()
    command[-1] = 10**400

    # Capture the complete state before submitting
    # the invalid actuator command.
    before = capture_state(sim)

    # Both rejection paths are valid. The important
    # requirement is that invalid input cannot reach
    # the physics engine.
    with pytest.raises((TypeError, ValueError)):
        sim.step(control=command)

    # Verify no actuator commands, robot positions,
    # velocities, simulation time or execution counters
    # were changed by the rejected request.
    assert_state_unchanged(sim, before)



@pytest.mark.parametrize(
    "invalid",
    [float("nan"), float("inf"), float("-inf"), 256.0, -1.0],
)
def test_invalid_retained_gripper_control_rejected(sim, invalid):
    """control=None must not bypass corrupted data.ctrl."""
    gripper_id = sim.model.actuator("actuator8").id
    sim.data.ctrl[gripper_id] = invalid

    # Snapshot AFTER the intentional low-level corruption.
    before = capture_state(sim)

    with pytest.raises(ValueError):
        sim.step(control=None)

    assert_state_unchanged(sim, before)


def test_invalid_retained_control_rejected_by_controller(sim):
    """The controller-period path inherits retained-control guards."""
    gripper_id = sim.model.actuator("actuator8").id
    sim.data.ctrl[gripper_id] = float("nan")

    before = capture_state(sim)

    with pytest.raises(ValueError):
        sim.step_controller_period(
            control=None,
            period=0.04,
        )

    assert_state_unchanged(sim, before)


def test_valid_retained_control_still_steps(sim):
    """The existing control=None contract remains usable."""
    original_ctrl = sim.data.ctrl.copy()

    sim.step(control=None, n_substeps=3)

    np.testing.assert_array_equal(
        sim.data.ctrl,
        original_ctrl,
    )
    assert sim.physics_step_count == 3
    assert sim.controller_step_count == 0


def test_valid_controller_period_still_counts_once(sim):
    """P0-006.06 and P0-006.07 must remain functional."""
    sim.step_controller_period(
        control=None,
        period=0.04,
    )

    assert sim.physics_step_count == 20
    assert sim.controller_step_count == 1
    assert sim.simulation_time == pytest.approx(0.04)


def test_none_is_not_converted_to_numpy_array(
    sim,
    monkeypatch,
):
    """Protect against the earlier P0-006.05 indentation bug."""
    original_asarray = np.asarray

    def guarded_asarray(value, *args, **kwargs):
        if value is None:
            raise AssertionError(
                "None was passed to np.asarray"
            )

        return original_asarray(value, *args, **kwargs)

    monkeypatch.setattr(
        np,
        "asarray",
        guarded_asarray,
    )

    sim.step(control=None)

    assert sim.physics_step_count == 1


def test_invalid_action_does_not_invoke_physics(
    sim,
    monkeypatch,
):
    """Validation must fail before the first integration."""
    def forbidden_step(*args, **kwargs):
        raise AssertionError(
            "Invalid command reached MuJoCo"
        )

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        forbidden_step,
    )

    command = sim.data.ctrl.copy()
    command[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step(control=command)


def test_invalid_controller_period_preserves_counters(sim):
    """Rejected timing cannot create physics or controller ticks."""
    before = capture_state(sim)

    with pytest.raises(ValueError):
        sim.step_controller_period(period=0.031)

    assert_state_unchanged(sim, before)


def test_failed_controller_action_preserves_counters(sim):
    """A valid period with bad controls must not record a tick."""
    before = capture_state(sim)
    command = sim.data.ctrl.copy()
    command[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step_controller_period(
            control=command,
            period=0.04,
        )

    assert_state_unchanged(sim, before)


def test_close_is_idempotent(sim):
    """Repeated closure is safe and cannot restart simulation."""
    sim.close()
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.step()


def test_rejected_seed_preserves_episode(sim):
    """Unsupported seeded resets must not change episode state."""
    sim.step(n_substeps=3)
    before = capture_state(sim)

    with pytest.raises(NotImplementedError):
        sim.reset(seed=1)

    assert_state_unchanged(sim, before)
