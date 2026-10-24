
"""P0-006.02 — Deterministic primary-state reset tests.

Tests the real composed Panda scene without modifying source
MJCF assets or the existing test runner.
"""

import os
from pathlib import Path

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
    """Load an independent Panda simulator per test."""
    value = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not value:
        pytest.skip("Panda model path not configured")

    simulator = Simulator(Path(value))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


def baseline(sim):
    """Create independent baseline copies for comparison."""
    target_id = sim.model.body("target").id

    return {
        "qpos": sim.data.qpos.copy(),
        "qvel": sim.data.qvel.copy(),
        "ctrl": sim.data.ctrl.copy(),
        "target": sim.model.body_pos[target_id].copy(),
        "time": float(sim.data.time),
    }


def assert_restored(sim, initial):
    """Verify every required primary-state component."""
    target_id = sim.model.body("target").id

    np.testing.assert_array_equal(
        sim.data.qpos, initial["qpos"]
    )
    np.testing.assert_array_equal(
        sim.data.qvel, initial["qvel"]
    )
    np.testing.assert_array_equal(
        sim.data.ctrl, initial["ctrl"]
    )
    np.testing.assert_array_equal(
        sim.model.body_pos[target_id],
        initial["target"],
    )
    assert float(sim.data.time) == initial["time"]


def test_reset_restores_entire_primary_state(sim):
    """All state arrays and the fixed target are restored."""
    initial = baseline(sim)

    sim.data.qpos[:9] += 0.01
    sim.data.qvel[:] = 0.25
    sim.data.ctrl[:] = 1.0
    sim.model.body("target").pos[:] += [0.02, 0.0, 0.0]
    sim.data.time = 1.25

    sim.reset()

    assert_restored(sim, initial)


def test_reset_restores_all_seven_arm_joints(sim):
    """Arm state must use compiled qpos addresses."""
    initial = baseline(sim)

    for index in range(1, 8):
        jid = sim.model.joint(f"joint{index}").id
        qadr = int(sim.model.jnt_qposadr[jid])
        vadr = int(sim.model.jnt_dofadr[jid])

        sim.data.qpos[qadr] += 0.01
        sim.data.qvel[vadr] = 0.1

    sim.reset()

    assert_restored(sim, initial)


def test_reset_restores_both_finger_joints(sim):
    """Neither finger joint may remain at its moved state."""
    initial = baseline(sim)

    for name in ("finger_joint1", "finger_joint2"):
        jid = sim.model.joint(name).id
        qadr = int(sim.model.jnt_qposadr[jid])
        vadr = int(sim.model.jnt_dofadr[jid])

        sim.data.qpos[qadr] += 0.001
        sim.data.qvel[vadr] = 0.05

    sim.reset()

    assert_restored(sim, initial)


def test_reset_restores_ball_translation_and_quaternion(sim):
    """Preserve the free joint's complete 7-value qpos slice."""
    initial = baseline(sim)

    jid = sim.model.joint("ball_free").id
    address = int(sim.model.jnt_qposadr[jid])

    # Move the ball and replace its orientation with a valid
    # non-identity unit quaternion.
    sim.data.qpos[address:address + 3] = (
        0.10, 0.05, 0.80
    )
    sim.data.qpos[address + 3:address + 7] = (
        0.0, 1.0, 0.0, 0.0
    )

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qpos[address:address + 7],
        initial["qpos"][address:address + 7],
    )


def test_reset_restores_all_ball_velocities(sim):
    """Free-joint velocity has 6, not 7, coordinates."""
    initial = baseline(sim)

    jid = sim.model.joint("ball_free").id
    address = int(sim.model.jnt_dofadr[jid])

    sim.data.qvel[address:address + 6] = (
        0.5, -0.2, 0.3, 0.1, 0.4, -0.1
    )

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qvel[address:address + 6],
        initial["qvel"][address:address + 6],
    )


def test_reset_restores_every_actuator_command(sim):
    """Restore arm servo controls and the gripper command."""
    initial = baseline(sim)

    sim.data.ctrl[:] = 0.0
    sim.data.ctrl[
        sim.model.actuator("actuator8").id
    ] = 255.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.ctrl, initial["ctrl"]
    )


def test_reset_restores_target_position(sim):
    """The fixed goal position lives in MjModel.body_pos."""
    initial = baseline(sim)
    target_id = sim.model.body("target").id

    sim.model.body_pos[target_id] = [0.40, 0.1, 0.52]

    sim.reset()

    np.testing.assert_array_equal(
        sim.model.body_pos[target_id],
        initial["target"],
    )


def test_reset_restores_time_without_stepping(sim):
    """Reset must return to the captured initial clock."""
    initial = baseline(sim)
    sim.data.time = 9.0

    sim.reset()

    assert sim.data.time == initial["time"]


def test_reset_preserves_model_and_data_identity(sim):
    """Reset reuses the same objects rather than reloading."""
    model = sim.model
    data = sim.data

    sim.data.qvel[:] = 1.0
    sim.reset()

    assert sim.model is model
    assert sim.data is data


def test_repeated_reset_is_idempotent(sim):
    """Resetting repeatedly must not accumulate changes."""
    initial = baseline(sim)

    sim.data.qvel[:] = 0.2
    sim.reset()
    sim.reset()
    sim.reset()

    assert_restored(sim, initial)


def test_reset_snapshot_is_not_a_live_view(sim):
    """Mutating MjData must not mutate the saved baseline."""
    initial = baseline(sim)

    # Make a substantial change after the baseline is saved.
    sim.data.qvel[:] = 100.0
    sim.data.ctrl[:] = 100.0

    sim.reset()

    assert_restored(sim, initial)


def test_reset_does_not_replace_other_instance_state(sim):
    """Reset must affect only its own model/data instance."""
    path = os.environ["ROBOT_ARM_PANDA_XML"]

    second = Simulator(Path(path))
    second.load()

    try:
        second.data.qvel[:] = 0.75
        before = second.data.qvel.copy()

        sim.data.qvel[:] = 1.5
        sim.reset()

        np.testing.assert_array_equal(
            second.data.qvel, before
        )
    finally:
        second.close()


def test_reset_before_load_is_rejected():
    """The existing NEW state guard must remain intact."""
    sim = Simulator("unused.xml")

    with pytest.raises(SimulatorLifecycleError):
        sim.reset()


def test_reset_after_close_is_rejected(sim):
    """Closing permanently disables reset."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.reset()


def test_seeded_reset_not_silently_accepted(sim):
    """Seeded random resets require their own implementation."""
    with pytest.raises(NotImplementedError):
        sim.reset(seed=42)


def test_unsupported_seed_does_not_change_state(sim):
    """Rejecting a seed must leave the live state untouched."""
    sim.data.qvel[:] = 0.3
    before = sim.data.qvel.copy()

    with pytest.raises(NotImplementedError):
        sim.reset(seed=123)

    np.testing.assert_array_equal(
        sim.data.qvel, before
    )


def test_reset_does_not_advance_physics(sim, monkeypatch):
    """Reset must never invoke the stepping entrypoint."""
    def forbidden_step(*args, **kwargs):
        raise AssertionError("reset() called mj_step")

    monkeypatch.setattr(
        mujoco, "mj_step", forbidden_step
    )

    sim.reset()

    assert sim.state is SimulatorState.LOADED


def test_reset_does_not_recompile_scene(sim, monkeypatch):
    """Restoring state must not create a replacement model."""
    import robot_arm_sim.model_instance as instances

    def forbidden_build(*args, **kwargs):
        raise AssertionError("reset() rebuilt the scene")

    monkeypatch.setattr(
        instances, "build_scene", forbidden_build
    )

    sim.reset()


def test_initial_snapshot_is_cleared_on_close(sim):
    """Closing releases the wrapper's saved baseline."""
    assert sim._reset_snapshot is not None

    sim.close()

    assert sim._reset_snapshot is None


def test_snapshot_arrays_are_read_only(sim):
    """Internal baselines must resist accidental writes."""
    snapshot = sim._reset_snapshot
    assert snapshot is not None

    for array in (
        snapshot.qpos,
        snapshot.qvel,
        snapshot.ctrl,
        snapshot.target_position,
    ):
        assert array.flags.writeable is False


def test_step_remains_unimplemented(sim):
    """Reset implementation must not enable motion prematurely."""
    with pytest.raises(NotImplementedError):
        sim.step()
