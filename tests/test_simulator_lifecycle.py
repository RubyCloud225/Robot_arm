
"""P0-006.01 — Simulator lifecycle and ownership tests.

These tests validate the wrapper without implementing reset,
control or stepping ahead of their separate P0-006 tickets.
"""

from pathlib import Path
import os

import mujoco
import pytest

from robot_arm_sim.simulator import (
    Simulator,
    SimulatorLifecycleError,
    SimulatorState,
)


@pytest.fixture
def panda_path() -> Path:
    """Require the pinned local Panda model used by the runner."""
    value = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not value:
        pytest.skip("Panda model path not configured")

    path = Path(value)
    assert path.is_file()
    return path


def test_new_simulator_does_not_load_model(panda_path):
    """Construction must not allocate model/data."""
    sim = Simulator(panda_path)

    assert sim.state is SimulatorState.NEW
    assert sim.is_loaded is False
    assert sim._instance is None


def test_explicit_load_creates_model_and_data(panda_path):
    """load() must create both required MuJoCo objects."""
    sim = Simulator(panda_path)
    sim.load()

    assert sim.state is SimulatorState.LOADED
    assert sim.is_loaded is True
    assert isinstance(sim.model, mujoco.MjModel)
    assert isinstance(sim.data, mujoco.MjData)

    sim.close()


def test_independent_simulators_have_independent_models(panda_path):
    """Never share mutable model/state between instances."""
    first = Simulator(panda_path)
    second = Simulator(panda_path)

    first.load()
    second.load()

    try:
        assert first.model is not second.model
        assert first.data is not second.data
    finally:
        first.close()
        second.close()


def test_loading_one_simulator_does_not_load_another(panda_path):
    """Model allocation is explicitly per instance."""
    first = Simulator(panda_path)
    second = Simulator(panda_path)

    first.load()

    try:
        assert first.is_loaded
        assert second.state is SimulatorState.NEW
        assert second._instance is None
    finally:
        first.close()
        second.close()


def test_model_access_before_load_rejected(panda_path):
    sim = Simulator(panda_path)

    with pytest.raises(
        SimulatorLifecycleError, match="NEW"
    ):
        _ = sim.model


def test_data_access_before_load_rejected(panda_path):
    sim = Simulator(panda_path)

    with pytest.raises(
        SimulatorLifecycleError, match="NEW"
    ):
        _ = sim.data


def test_duplicate_load_rejected(panda_path):
    sim = Simulator(panda_path)
    sim.load()

    try:
        original_model = sim.model

        with pytest.raises(SimulatorLifecycleError):
            sim.load()

        # Duplicate loading must not replace the model.
        assert sim.model is original_model
    finally:
        sim.close()


def test_failed_load_preserves_new_state(tmp_path):
    """A missing model must not leave partial ownership."""
    sim = Simulator(tmp_path / "missing_panda.xml")

    with pytest.raises(ValueError):
        sim.load()

    assert sim.state is SimulatorState.NEW
    assert sim._instance is None
    assert sim.is_loaded is False


def test_close_transitions_to_closed(panda_path):
    sim = Simulator(panda_path)
    sim.load()
    sim.close()

    assert sim.state is SimulatorState.CLOSED
    assert sim.is_loaded is False
    assert sim._instance is None


def test_close_before_load_is_safe(panda_path):
    """Closing an unused wrapper is permitted."""
    sim = Simulator(panda_path)
    sim.close()

    assert sim.state is SimulatorState.CLOSED


def test_double_close_is_safe(panda_path):
    sim = Simulator(panda_path)
    sim.load()

    sim.close()
    sim.close()

    assert sim.state is SimulatorState.CLOSED


def test_access_after_close_rejected(panda_path):
    sim = Simulator(panda_path)
    sim.load()
    sim.close()

    with pytest.raises(
        SimulatorLifecycleError, match="CLOSED"
    ):
        _ = sim.model

    with pytest.raises(
        SimulatorLifecycleError, match="CLOSED"
    ):
        _ = sim.data


def test_reload_after_close_rejected(panda_path):
    sim = Simulator(panda_path)
    sim.load()
    sim.close()

    with pytest.raises(
        SimulatorLifecycleError, match="CLOSED"
    ):
        sim.load()


def test_reset_before_load_rejected(panda_path):
    sim = Simulator(panda_path)

    with pytest.raises(SimulatorLifecycleError):
        sim.reset()


def test_step_before_load_rejected(panda_path):
    sim = Simulator(panda_path)

    with pytest.raises(SimulatorLifecycleError):
        sim.step()


def test_reset_not_yet_implemented(panda_path):
    """006.01 must not claim reset functionality prematurely."""
    sim = Simulator(panda_path)
    sim.load()

    try:
        with pytest.raises(NotImplementedError):
            sim.reset()
    finally:
        sim.close()


def test_step_not_yet_implemented(panda_path):
    """006.01 must not claim controlled motion prematurely."""
    sim = Simulator(panda_path)
    sim.load()

    try:
        with pytest.raises(NotImplementedError):
            sim.step()
    finally:
        sim.close()


@pytest.mark.parametrize(
    "invalid",
    [None, 17, 2.5, True, ""],
)
def test_invalid_constructor_path_rejected(invalid):
    with pytest.raises((TypeError, ValueError)):
        Simulator(invalid)


def test_load_does_not_advance_physics(panda_path):
    sim = Simulator(panda_path)
    sim.load()

    try:
        assert sim.data.time == pytest.approx(0.0)
    finally:
        sim.close()


def test_close_does_not_advance_physics(panda_path):
    sim = Simulator(panda_path)
    sim.load()

    # Hold an external reference to check that closure
    # itself does not change the underlying MuJoCo data.
    data = sim.data
    original_time = float(data.time)

    sim.close()

    assert data.time == original_time


def test_close_does_not_destroy_external_reference(panda_path):
    """Document the actual lifetime semantics of Python bindings."""
    sim = Simulator(panda_path)
    sim.load()

    external_model = sim.model
    external_data = sim.data

    sim.close()

    assert isinstance(external_model, mujoco.MjModel)
    assert isinstance(external_data, mujoco.MjData)


def test_instances_remain_independent_after_one_closes(panda_path):
    first = Simulator(panda_path)
    second = Simulator(panda_path)

    first.load()
    second.load()

    first.close()

    try:
        assert second.state is SimulatorState.LOADED
        assert isinstance(second.model, mujoco.MjModel)
        assert isinstance(second.data, mujoco.MjData)
    finally:
        second.close()
