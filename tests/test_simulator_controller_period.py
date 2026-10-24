
"""P0-006.06 — Controller-period timing validation.

The controller period must map to a positive integer number
of MuJoCo physics integrations.

All ordinary invalid requests must fail before simulation
state is modified.
"""

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_arm_sim.simulator import (
    Simulator,
    SimulatorLifecycleError,
    _controller_substep_count,
)


@pytest.fixture
def sim():
    """Create one independent Panda simulation for each test."""
    path = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not path:
        pytest.fail(
            "ROBOT_ARM_PANDA_XML is required for "
            "controller-period integration tests"
        )

    simulator = Simulator(Path(path))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


@pytest.mark.parametrize(
    ("period", "dt", "expected"),
    [
        (0.002, 0.002, 1),
        (0.010, 0.002, 5),
        (0.040, 0.002, 20),
        (0.080, 0.002, 40),
        (0.030, 0.003, 10),
        (0.001, 0.001, 1),
    ],
)
def test_compatible_periods_have_expected_substep_counts(
    period,
    dt,
    expected,
):
    """All accepted ratios produce the correct integer count."""
    assert _controller_substep_count(period, dt) == expected


@pytest.mark.parametrize(
    ("period", "dt"),
    [
        (0.031, 0.002),
        (0.025, 0.004),
        (0.001, 0.002),
        (0.0001, 0.002),
        (0.0401, 0.002),
    ],
)
def test_noninteger_or_short_periods_are_rejected(
    period,
    dt,
):
    """Never round a materially incompatible controller duration."""
    with pytest.raises(ValueError):
        _controller_substep_count(period, dt)


@pytest.mark.parametrize(
    "period",
    [
        0.0,
        -0.04,
        float("nan"),
        float("inf"),
        float("-inf"),
    ],
)
def test_invalid_period_values_are_rejected(period):
    """Reject nonpositive and nonfinite requested durations."""
    with pytest.raises(ValueError):
        _controller_substep_count(period, 0.002)


@pytest.mark.parametrize(
    "period",
    [True, False, None, "0.04", [], {}],
)
def test_invalid_period_types_are_rejected(period):
    """Do not coerce booleans, strings or containers into time."""
    with pytest.raises(TypeError):
        _controller_substep_count(period, 0.002)


@pytest.mark.parametrize(
    "dt",
    [0.0, -0.002, float("nan"), float("inf")],
)
def test_invalid_physics_timestep_is_rejected(dt):
    """A corrupted timestep cannot be used for scheduling."""
    with pytest.raises(ValueError):
        _controller_substep_count(0.04, dt)


def test_float_representation_noise_is_accepted():
    """Ordinary floating-point error should not invalidate timing."""
    period = 0.1 + 0.2
    timestep = 0.1

    assert _controller_substep_count(period, timestep) == 3


def test_large_finite_but_nonrepresentable_ratio_rejected():
    """Ratio overflow cannot produce a usable substep count."""
    with pytest.raises(ValueError, match="not finite"):
        _controller_substep_count(1e308, 1e-308)


def test_default_controller_period_advances_20_steps(sim):
    """40 ms requires 20 physics steps at the default 2 ms dt."""
    assert sim.model.opt.timestep == pytest.approx(0.002)

    initial_time = float(sim.data.time)

    sim.step_controller_period(period=0.04)

    assert sim.data.time - initial_time == pytest.approx(
        0.04,
        rel=1e-10,
        abs=1e-12,
    )


def test_one_controller_period_calls_mujoco_20_times(
    sim,
    monkeypatch,
):
    """Count real MuJoCo integrations, not only elapsed time."""
    original_step = mujoco.mj_step
    calls = []

    def recording_step(model, data):
        calls.append((model, data))
        return original_step(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        recording_step,
    )

    sim.step_controller_period(period=0.04)

    assert len(calls) == 20
    assert all(
        model is sim.model and data is sim.data
        for model, data in calls
    )


def test_controller_period_holds_command_constant(
    sim,
    monkeypatch,
):
    """Do not alter the selected command between physics steps."""
    command = sim.data.ctrl.copy()
    gripper_id = sim.model.actuator("actuator8").id
    command[gripper_id] = 255.0

    observed = []
    original_step = mujoco.mj_step

    def recording_step(model, data):
        observed.append(data.ctrl.copy())
        return original_step(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        recording_step,
    )

    sim.step_controller_period(
        control=command,
        period=0.04,
    )

    assert len(observed) == 20

    for value in observed:
        np.testing.assert_array_equal(value, command)


def test_none_control_retains_existing_commands(sim):
    """The controller wrapper must preserve P0-006.05 semantics."""
    initial_controls = sim.data.ctrl.copy()

    sim.step_controller_period(
        control=None,
        period=0.04,
    )

    np.testing.assert_array_equal(
        sim.data.ctrl,
        initial_controls,
    )


def test_controller_period_uses_current_model_timestep(sim):
    """Do not hardcode the original 0.002-second physics step."""
    sim.model.opt.timestep = 0.004

    initial_time = float(sim.data.time)

    sim.step_controller_period(period=0.04)

    assert sim.data.time - initial_time == pytest.approx(
        0.04,
        rel=1e-10,
        abs=1e-12,
    )


def test_invalid_period_preserves_state_and_control(sim):
    """Incompatible periods must not cause partial execution."""
    original_time = float(sim.data.time)
    original_qpos = sim.data.qpos.copy()
    original_ctrl = sim.data.ctrl.copy()

    with pytest.raises(ValueError):
        sim.step_controller_period(
            control=original_ctrl.copy(),
            period=0.031,
        )

    assert sim.data.time == original_time
    np.testing.assert_array_equal(
        sim.data.qpos,
        original_qpos,
    )
    np.testing.assert_array_equal(
        sim.data.ctrl,
        original_ctrl,
    )


def test_invalid_period_does_not_call_physics(
    sim,
    monkeypatch,
):
    """Validate controller duration before any MuJoCo stepping."""
    def forbidden_step(*args, **kwargs):
        raise AssertionError(
            "Invalid controller period reached MuJoCo"
        )

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        forbidden_step,
    )

    with pytest.raises(ValueError):
        sim.step_controller_period(period=0.031)


def test_invalid_control_does_not_advance_controller_period(sim):
    """The existing step() method must reject bad commands."""
    initial_time = float(sim.data.time)
    original_ctrl = sim.data.ctrl.copy()

    invalid = original_ctrl.copy()
    invalid[-1] = 9999.0

    with pytest.raises(ValueError):
        sim.step_controller_period(
            control=invalid,
            period=0.04,
        )

    assert sim.data.time == initial_time
    np.testing.assert_array_equal(
        sim.data.ctrl,
        original_ctrl,
    )


def test_missing_period_is_rejected(sim):
    """Require an explicit controller duration."""
    with pytest.raises(TypeError):
        sim.step_controller_period()


def test_repeated_controller_period_calls_accumulate_time(sim):
    """Multiple controller ticks advance consistent physics time."""
    initial_time = float(sim.data.time)

    for _ in range(3):
        sim.step_controller_period(period=0.04)

    assert sim.data.time - initial_time == pytest.approx(
        0.12,
        rel=1e-10,
        abs=1e-12,
    )


def test_physics_step_and_controller_step_can_be_mixed(sim):
    """Controller periods and raw steps use the same clock."""
    initial_time = float(sim.data.time)

    sim.step(n_substeps=5)
    sim.step_controller_period(period=0.04)

    assert sim.data.time - initial_time == pytest.approx(
        0.05,
        rel=1e-10,
        abs=1e-12,
    )


def test_reset_after_controller_period_restores_baseline(sim):
    """Controller execution must remain compatible with reset."""
    initial_time = float(sim.data.time)
    initial_qpos = sim.data.qpos.copy()

    sim.step_controller_period(period=0.04)
    assert sim.data.time > initial_time

    sim.reset()

    assert sim.data.time == initial_time
    np.testing.assert_array_equal(
        sim.data.qpos,
        initial_qpos,
    )


def test_controller_period_preserves_model_data_identity(sim):
    """No compilation or replacement should occur."""
    original_model = sim.model
    original_data = sim.data

    sim.step_controller_period(period=0.04)

    assert sim.model is original_model
    assert sim.data is original_data


def test_two_simulators_have_independent_controller_time(sim):
    """A controller tick in one instance leaves the other untouched."""
    source = os.environ["ROBOT_ARM_PANDA_XML"]
    other = Simulator(Path(source))
    other.load()

    try:
        other_time = float(other.data.time)

        sim.step_controller_period(period=0.04)

        assert other.data.time == other_time
        assert sim.data.time > other_time
    finally:
        other.close()


def test_controller_period_before_load_is_rejected():
    """The wrapper cannot operate before model loading."""
    sim = Simulator("unused.xml")

    with pytest.raises(SimulatorLifecycleError):
        sim.step_controller_period(period=0.04)


def test_controller_period_after_close_is_rejected(sim):
    """Closed simulators cannot resume execution."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.step_controller_period(period=0.04)
