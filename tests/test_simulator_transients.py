
"""P0-006.03 — MuJoCo transient-state reset tests.

Challenge the reset implementation with stale applied forces,
solver warm-start values and actuator activation state.

Tests operate on independent compiled Panda simulations.
Neither model assets nor the test runner are modified.
"""

from pathlib import Path
from dataclasses import replace
import os

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
    """Give every test an independently loaded Panda scene."""
    source = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not source:
        pytest.skip("Panda model path not configured")

    simulator = Simulator(Path(source))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


def test_generalised_applied_forces_are_cleared(sim):
    """A previous joint force must not survive reset."""
    sim.data.qfrc_applied[:] = 12.5

    assert np.any(sim.data.qfrc_applied != 0.0)

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qfrc_applied,
        np.zeros_like(sim.data.qfrc_applied),
    )


def test_external_body_forces_are_cleared(sim):
    """Clear all externally applied body forces and torques."""
    sim.data.xfrc_applied[:] = -7.5

    assert np.any(sim.data.xfrc_applied != 0.0)

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.xfrc_applied,
        np.zeros_like(sim.data.xfrc_applied),
    )


def test_solver_warmstart_is_cleared(sim):
    """Previous acceleration estimates must not leak."""
    sim.data.qacc_warmstart[:] = 3.25

    assert np.any(sim.data.qacc_warmstart != 0.0)

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qacc_warmstart,
        np.zeros_like(sim.data.qacc_warmstart),
    )


def test_transients_are_cleared_together(sim):
    """Reset every supported transient class in one operation."""
    sim.data.qfrc_applied[:] = 100.0
    sim.data.xfrc_applied[:] = -100.0
    sim.data.qacc_warmstart[:] = 1000.0

    sim.reset()

    assert np.count_nonzero(sim.data.qfrc_applied) == 0
    assert np.count_nonzero(sim.data.xfrc_applied) == 0
    assert np.count_nonzero(sim.data.qacc_warmstart) == 0


def test_reset_restores_captured_actuator_activation(sim):
    """Activation must return to its initial value, not be guessed.

    Some models have no stateful actuator activations (na == 0).
    In that case, equality of the empty activation arrays is
    still valid, but this test does not claim to exercise a
    non-empty actuator activation state.
    """
    original_activation = sim.data.act.copy()

    if sim.data.act.size:
        sim.data.act[:] = 8.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.act,
        original_activation,
    )


def test_reset_does_not_zero_valid_control_commands(sim):
    """Initial arm and gripper commands must be preserved."""
    initial_controls = sim.data.ctrl.copy()

    sim.data.ctrl[:] = 0.0
    sim.data.qfrc_applied[:] = 50.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.ctrl,
        initial_controls,
    )
    assert np.count_nonzero(sim.data.qfrc_applied) == 0


def test_reset_restores_joint_state_and_clears_forces(sim):
    """Primary-state restoration and cleanup must work together."""
    original_qpos = sim.data.qpos.copy()
    original_qvel = sim.data.qvel.copy()

    sim.data.qpos[:] += 0.01
    sim.data.qvel[:] = 0.8
    sim.data.qfrc_applied[:] = 20.0
    sim.data.qacc_warmstart[:] = -3.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qpos, original_qpos
    )
    np.testing.assert_array_equal(
        sim.data.qvel, original_qvel
    )
    assert np.count_nonzero(sim.data.qfrc_applied) == 0
    assert np.count_nonzero(sim.data.qacc_warmstart) == 0


def test_reset_preserves_ball_quaternion(sim):
    """Clearing transients must not corrupt the free-ball pose."""
    jid = sim.model.joint("ball_free").id
    address = int(sim.model.jnt_qposadr[jid])

    original = sim.data.qpos[
        address:address + 7
    ].copy()

    # This is a valid unit quaternion, not a velocity vector.
    sim.data.qpos[address + 3:address + 7] = (
        0.0, 0.0, 1.0, 0.0
    )
    sim.data.xfrc_applied[:] = 25.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qpos[address:address + 7],
        original,
    )


def test_reset_restores_target_and_clears_forces(sim):
    """The fixed target is restored alongside the transient state."""
    target_id = sim.model.body("target").id
    original_target = sim.model.body_pos[target_id].copy()

    sim.model.body_pos[target_id] += [0.1, 0.0, 0.0]
    sim.data.xfrc_applied[:] = 10.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.model.body_pos[target_id],
        original_target,
    )
    assert np.count_nonzero(sim.data.xfrc_applied) == 0


def test_reset_restores_time_and_clears_warmstart(sim):
    """No previous physics time or warm-start state may remain."""
    original_time = float(sim.data.time)

    sim.data.time = 14.0
    sim.data.qacc_warmstart[:] = 5.0

    sim.reset()

    assert float(sim.data.time) == original_time
    assert np.count_nonzero(sim.data.qacc_warmstart) == 0


def test_repeated_resets_do_not_reintroduce_forces(sim):
    """A second reset must not recover stale transient buffers."""
    sim.data.qfrc_applied[:] = 9.0
    sim.data.xfrc_applied[:] = 9.0

    sim.reset()
    sim.reset()
    sim.reset()

    assert np.count_nonzero(sim.data.qfrc_applied) == 0
    assert np.count_nonzero(sim.data.xfrc_applied) == 0
    assert np.count_nonzero(sim.data.qacc_warmstart) == 0


def test_two_simulators_have_independent_transients(sim):
    """Resetting one instance cannot clean another instance."""
    source = os.environ["ROBOT_ARM_PANDA_XML"]
    second = Simulator(Path(source))
    second.load()

    try:
        sim.data.qfrc_applied[:] = 3.0
        second.data.qfrc_applied[:] = 7.0

        sim.reset()

        assert np.count_nonzero(sim.data.qfrc_applied) == 0
        assert np.all(second.data.qfrc_applied == 7.0)
    finally:
        second.close()


def test_reset_keeps_model_and_data_identity(sim):
    """Native reset must not rebuild or replace MuJoCo objects."""
    original_model = sim.model
    original_data = sim.data

    sim.data.xfrc_applied[:] = 5.0
    sim.reset()

    assert sim.model is original_model
    assert sim.data is original_data


def test_reset_does_not_call_mj_step(sim, monkeypatch):
    """Reset cannot advance the physical simulation clock."""
    def forbidden_step(*args, **kwargs):
        raise AssertionError(
            "reset() must not call mj_step"
        )

    monkeypatch.setattr(mujoco, "mj_step", forbidden_step)

    sim.data.qfrc_applied[:] = 4.0
    sim.reset()

    assert sim.state is SimulatorState.LOADED


def test_reset_does_not_call_mj_forward_yet(sim, monkeypatch):
    """P0-006.04 retains responsibility for forward refresh."""
    def forbidden_forward(*args, **kwargs):
        raise AssertionError(
            "mj_forward belongs to P0-006.04"
        )

    monkeypatch.setattr(
        mujoco, "mj_forward", forbidden_forward
    )

    sim.reset()


def test_native_reset_is_called_once(sim, monkeypatch):
    """Confirm that the native transient cleanup path is used."""
    original_reset = mujoco.mj_resetData
    calls = []

    def recording_reset(model, data):
        calls.append((model, data))
        return original_reset(model, data)

    monkeypatch.setattr(
        mujoco, "mj_resetData", recording_reset
    )

    sim.reset()

    assert len(calls) == 1
    assert calls[0][0] is sim.model
    assert calls[0][1] is sim.data


def test_unsupported_seed_preserves_all_transients(sim):
    """Rejected requests must not trigger a partial native reset."""
    sim.data.qfrc_applied[:] = 4.0
    sim.data.xfrc_applied[:] = 5.0
    sim.data.qacc_warmstart[:] = 6.0

    before_qfrc = sim.data.qfrc_applied.copy()
    before_xfrc = sim.data.xfrc_applied.copy()
    before_warmstart = sim.data.qacc_warmstart.copy()

    with pytest.raises(NotImplementedError):
        sim.reset(seed=42)

    np.testing.assert_array_equal(
        sim.data.qfrc_applied, before_qfrc
    )
    np.testing.assert_array_equal(
        sim.data.xfrc_applied, before_xfrc
    )
    np.testing.assert_array_equal(
        sim.data.qacc_warmstart, before_warmstart
    )


def test_reset_after_close_is_rejected():
    """Closure continues to prevent all subsequent reset calls."""
    sim = Simulator("unused.xml")
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.reset()


def test_reset_does_not_modify_model_joint_limits(sim):
    """Transient reset must leave structural limits untouched."""
    initial = sim.model.jnt_range.copy()

    sim.data.qfrc_applied[:] = 2.0
    sim.reset()

    np.testing.assert_array_equal(
        sim.model.jnt_range, initial
    )


def test_no_viewer_is_required(sim):
    """A headless simulation must be able to reset normally."""
    sim.data.xfrc_applied[:] = 1.0

    sim.reset()

    assert np.count_nonzero(sim.data.xfrc_applied) == 0


def test_corrupt_snapshot_rejected_before_native_reset(
    sim,
    monkeypatch,
):
    """Shape incompatibility must not partially clear live state."""
    snapshot = sim._reset_snapshot
    assert snapshot is not None

    sim.data.qfrc_applied[:] = 17.0

    # A wrong qvel length is enough to invalidate the
    # snapshot without changing live MuJoCo dimensions.
    sim._reset_snapshot = replace(
        snapshot,
        qvel=snapshot.qvel[:-1].copy(),
    )

    def forbidden_reset(*args, **kwargs):
        raise AssertionError(
            "Native reset called before dimension validation"
        )

    monkeypatch.setattr(
        mujoco,
        "mj_resetData",
        forbidden_reset,
    )

    with pytest.raises(
        SimulatorLifecycleError,
        match="dimensions",
    ):
        sim.reset()

    assert np.all(sim.data.qfrc_applied == 17.0)
