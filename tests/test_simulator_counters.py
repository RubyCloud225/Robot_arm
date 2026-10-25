
"""P0-006.07 — Simulator time and execution counter tests.

Verify that each physics integration and controller period
is accounted for exactly once.

Counters are episode-local and independent per simulator.

Tests also cover rejected operations, partial execution
failures, resetting and lifecycle access restrictions.
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
    """Provide a fresh, independent Panda simulator per test."""
    source = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not source:
        pytest.fail(
            "ROBOT_ARM_PANDA_XML must be configured for "
            "P0-006.07 integration testing"
        )

    simulator = Simulator(Path(source))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


def test_initial_counters_are_zero(sim):
    """Loading the model must not count as physics execution."""
    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(0.0)


def test_single_physics_step_counts_once(sim):
    """One physics integration increases only the physics count."""
    timestep = float(sim.model.opt.timestep)

    sim.step()

    assert sim.physics_step_count == 1
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(timestep)


@pytest.mark.parametrize("count", [1, 2, 5, 20, 40])
def test_multiple_physics_substeps(sim, count):
    """The physics counter matches successful integrations."""
    timestep = float(sim.model.opt.timestep)

    sim.step(n_substeps=count)

    assert sim.physics_step_count == count
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(
        count * timestep
    )


def test_multiple_raw_step_calls_accumulate(sim):
    """Successive raw physics requests accumulate counts."""
    sim.step(n_substeps=3)
    sim.step(n_substeps=7)

    assert sim.physics_step_count == 10
    assert sim.controller_step_count == 0


def test_one_controller_period_counts_once(sim):
    """A controller period counts once, not once per substep."""
    sim.step_controller_period(period=0.04)

    assert sim.physics_step_count == 20
    assert sim.controller_step_count == 1
    assert sim.simulation_time == pytest.approx(0.04)


def test_multiple_controller_periods(sim):
    """Three controller periods have 60 physics integrations."""
    for _ in range(3):
        sim.step_controller_period(period=0.04)

    assert sim.physics_step_count == 60
    assert sim.controller_step_count == 3
    assert sim.simulation_time == pytest.approx(0.12)


def test_mixed_stepping_never_double_counts(sim):
    """Raw physics steps must not count as controller periods."""
    sim.step(n_substeps=5)
    sim.step_controller_period(period=0.04)
    sim.step(n_substeps=2)
    sim.step_controller_period(period=0.04)

    assert sim.physics_step_count == 47
    assert sim.controller_step_count == 2
    assert sim.simulation_time == pytest.approx(0.094)


def test_one_substep_controller_period(sim):
    """Even a single-substep controller update counts once."""
    timestep = float(sim.model.opt.timestep)

    sim.step_controller_period(period=timestep)

    assert sim.physics_step_count == 1
    assert sim.controller_step_count == 1


def test_invalid_raw_substeps_preserve_counters(sim):
    """Rejected physics requests cause no accounting changes."""
    sim.step(n_substeps=3)

    initial = (
        sim.physics_step_count,
        sim.controller_step_count,
        sim.simulation_time,
    )

    with pytest.raises(ValueError):
        sim.step(n_substeps=0)

    assert (
        sim.physics_step_count,
        sim.controller_step_count,
        sim.simulation_time,
    ) == initial


def test_invalid_raw_control_preserves_counters(sim):
    """Invalid actuator controls must not count as execution."""
    command = sim.data.ctrl.copy()
    command[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step(control=command)

    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0


def test_invalid_controller_period_preserves_counters(sim):
    """A fractional timing ratio cannot produce a controller tick."""
    initial_time = sim.simulation_time

    with pytest.raises(ValueError):
        sim.step_controller_period(period=0.031)

    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0
    assert sim.simulation_time == initial_time


def test_invalid_controller_control_preserves_counters(sim):
    """Invalid controller commands must not start integration."""
    command = sim.data.ctrl.copy()
    command[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step_controller_period(
            control=command,
            period=0.04,
        )

    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0


def test_partial_physics_failure_counts_only_completed_steps(
    sim,
    monkeypatch,
):
    """If step three fails, only the preceding two count."""
    original_step = mujoco.mj_step
    call_count = 0

    def failing_step(model, data):
        nonlocal call_count
        call_count += 1

        if call_count == 3:
            raise RuntimeError("Injected physics failure")

        return original_step(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        failing_step,
    )

    with pytest.raises(RuntimeError, match="Injected"):
        sim.step(n_substeps=5)

    assert call_count == 3
    assert sim.physics_step_count == 2
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(
        2 * float(sim.model.opt.timestep)
    )


def test_partial_controller_failure_does_not_count_tick(
    sim,
    monkeypatch,
):
    """A controller tick requires every substep to succeed."""
    original_step = mujoco.mj_step
    call_count = 0

    def failing_step(model, data):
        nonlocal call_count
        call_count += 1

        if call_count == 4:
            raise RuntimeError("Interrupted controller period")

        return original_step(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        failing_step,
    )

    with pytest.raises(RuntimeError, match="Interrupted"):
        sim.step_controller_period(period=0.04)

    assert sim.physics_step_count == 3
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(
        3 * float(sim.model.opt.timestep)
    )


def test_previous_controller_success_survives_later_failure(
    sim,
    monkeypatch,
):
    """An incomplete later period cannot erase prior successes."""
    sim.step_controller_period(period=0.04)

    original_step = mujoco.mj_step
    call_count = 0

    def failing_step(model, data):
        nonlocal call_count
        call_count += 1

        if call_count == 2:
            raise RuntimeError("Injected later failure")

        return original_step(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        failing_step,
    )

    with pytest.raises(RuntimeError):
        sim.step_controller_period(period=0.04)

    assert sim.physics_step_count == 21
    assert sim.controller_step_count == 1


def test_reset_clears_episode_counters(sim):
    """Reset begins a new episode with fresh accounting."""
    sim.step_controller_period(period=0.04)
    sim.step(n_substeps=5)

    assert sim.physics_step_count == 25
    assert sim.controller_step_count == 1

    sim.reset()

    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(0.0)


def test_new_episode_starts_counting_again(sim):
    """Counters restart normally after a successful reset."""
    sim.step(n_substeps=10)
    sim.reset()
    sim.step_controller_period(period=0.04)

    assert sim.physics_step_count == 20
    assert sim.controller_step_count == 1


def test_repeated_resets_are_idempotent_for_counters(sim):
    """Multiple resets leave the new episode counters at zero."""
    sim.step(n_substeps=3)

    sim.reset()
    sim.reset()

    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0


def test_rejected_seed_preserves_existing_counts(sim):
    """Seed validation failure must not clear current counters."""
    sim.step(n_substeps=5)

    with pytest.raises(NotImplementedError):
        sim.reset(seed=123)

    assert sim.physics_step_count == 5
    assert sim.controller_step_count == 0


def test_failed_forward_reset_does_not_clear_counters(
    sim,
    monkeypatch,
):
    """Counter clearing occurs only after reset fully succeeds."""
    sim.step(n_substeps=5)

    def failing_forward(*args, **kwargs):
        raise RuntimeError("Injected forward failure")

    monkeypatch.setattr(
        mujoco,
        "mj_forward",
        failing_forward,
    )

    with pytest.raises(RuntimeError, match="Injected"):
        sim.reset()

    assert sim.physics_step_count == 5
    assert sim.controller_step_count == 0


def test_simulation_time_uses_mujoco_clock(sim):
    """Time is not reconstructed from the step counter."""
    sim.step(n_substeps=2)

    sim.data.time = 7.25

    assert sim.simulation_time == pytest.approx(7.25)
    assert sim.physics_step_count == 2


def test_timestep_change_preserves_execution_counts(sim):
    """Physics step count is independent of timestep duration."""
    sim.step(n_substeps=2)

    sim.model.opt.timestep = 0.001
    sim.step(n_substeps=3)

    assert sim.physics_step_count == 5
    assert sim.controller_step_count == 0
    assert sim.simulation_time == pytest.approx(0.007)


def test_multiple_simulators_own_independent_counters(sim):
    """One instance cannot change another instance's counts."""
    source = os.environ["ROBOT_ARM_PANDA_XML"]

    second = Simulator(Path(source))
    second.load()

    try:
        sim.step_controller_period(period=0.04)
        second.step(n_substeps=3)

        assert sim.physics_step_count == 20
        assert sim.controller_step_count == 1

        assert second.physics_step_count == 3
        assert second.controller_step_count == 0

        sim.reset()

        assert second.physics_step_count == 3
        assert second.controller_step_count == 0
    finally:
        second.close()


@pytest.mark.parametrize(
    "name",
    [
        "simulation_time",
        "physics_step_count",
        "controller_step_count",
    ],
)
def test_properties_reject_access_before_load(name):
    """Counter and time access requires LOADED lifecycle state."""
    simulator = Simulator("unused.xml")

    with pytest.raises(SimulatorLifecycleError):
        getattr(simulator, name)


@pytest.mark.parametrize(
    "name",
    [
        "simulation_time",
        "physics_step_count",
        "controller_step_count",
    ],
)
def test_properties_reject_access_after_close(sim, name):
    """A closed simulator has no publicly accessible episode."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        getattr(sim, name)


@pytest.mark.parametrize(
    "name",
    [
        "simulation_time",
        "physics_step_count",
        "controller_step_count",
    ],
)
def test_public_properties_cannot_be_assigned(sim, name):
    """Public accounting properties must remain read-only."""
    with pytest.raises(AttributeError):
        setattr(sim, name, 123)


def test_properties_exist_on_simulator_class():
    """Catch a property accidentally declared outside the class."""
    for name in (
        "simulation_time",
        "physics_step_count",
        "controller_step_count",
    ):
        assert isinstance(getattr(Simulator, name), property)


def test_mj_forward_does_not_increment_counts(sim):
    """Forward recomputation alone is not an integration."""
    mujoco.mj_forward(sim.model, sim.data)

    assert sim.physics_step_count == 0
    assert sim.controller_step_count == 0
