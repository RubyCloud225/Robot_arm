
"""P0-006.04 — Forward kinematics and contact refresh tests.

Validate that Simulator.reset() restores primary state,
clears transients and recomputes MuJoCo's derived state.

All real-model tests use an independent Panda simulation.
No production assets or runner scripts are modified.
"""

from pathlib import Path
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
    """Create and close one independent Panda simulation."""
    path = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not path:
        pytest.fail(
            "ROBOT_ARM_PANDA_XML is required for "
            "P0-006.04 integration tests"
        )

    simulator = Simulator(Path(path))
    simulator.load()

    try:
        yield simulator
    finally:
        simulator.close()


def contact_pairs(model, data):
    """Return an unordered set of contacting geometry pairs.

    Contact pairs are normalized because the ordering of
    geom1 and geom2 is not a semantic property of contact.
    """
    return {
        frozenset(
            (
                int(data.contact[i].geom1),
                int(data.contact[i].geom2),
            )
        )
        for i in range(data.ncon)
    }


def baseline_derived(sim):
    """Capture a forward-computed reference for the initial scene.

    Calling mj_forward before taking the reference avoids
    comparing a fresh reset against uninitialized/stale
    derived data from another stage of the model lifecycle.
    """
    mujoco.mj_forward(sim.model, sim.data)

    return {
        "xpos": sim.data.xpos.copy(),
        "xquat": sim.data.xquat.copy(),
        "site_xpos": sim.data.site_xpos.copy(),
        "geom_xpos": sim.data.geom_xpos.copy(),
        "contacts": contact_pairs(sim.model, sim.data),
        "ncon": int(sim.data.ncon),
        "qpos": sim.data.qpos.copy(),
        "qvel": sim.data.qvel.copy(),
        "ctrl": sim.data.ctrl.copy(),
        "time": float(sim.data.time),
    }


def test_reset_refreshes_all_body_positions(sim):
    """All world body positions match the baseline after reset."""
    reference = baseline_derived(sim)

    sim.data.qpos[:] += 0.01
    mujoco.mj_forward(sim.model, sim.data)

    sim.reset()

    np.testing.assert_allclose(
        sim.data.xpos,
        reference["xpos"],
        rtol=0.0,
        atol=1e-10,
    )


def test_reset_refreshes_body_orientations(sim):
    """World-space body quaternions are recalculated."""
    reference = baseline_derived(sim)

    joint = sim.model.joint("joint1")
    address = int(sim.model.jnt_qposadr[joint.id])

    sim.data.qpos[address] += 0.1
    mujoco.mj_forward(sim.model, sim.data)

    sim.reset()

    np.testing.assert_allclose(
        sim.data.xquat,
        reference["xquat"],
        rtol=0.0,
        atol=1e-10,
    )


def test_reset_refreshes_all_site_positions(sim):
    """End-effector, finger and target sites use restored state."""
    reference = baseline_derived(sim)

    joint = sim.model.joint("joint2")
    address = int(sim.model.jnt_qposadr[joint.id])

    sim.data.qpos[address] += 0.1
    mujoco.mj_forward(sim.model, sim.data)

    sim.reset()

    np.testing.assert_allclose(
        sim.data.site_xpos,
        reference["site_xpos"],
        rtol=0.0,
        atol=1e-10,
    )


def test_reset_refreshes_collision_geometry_positions(sim):
    """Collision geometry returns to baseline world transforms."""
    reference = baseline_derived(sim)

    joint = sim.model.joint("joint3")
    address = int(sim.model.jnt_qposadr[joint.id])

    sim.data.qpos[address] -= 0.1
    mujoco.mj_forward(sim.model, sim.data)

    sim.reset()

    np.testing.assert_allclose(
        sim.data.geom_xpos,
        reference["geom_xpos"],
        rtol=0.0,
        atol=1e-10,
    )


def test_ball_world_position_matches_restored_freejoint(sim):
    """The free ball must report its restored world position."""
    sim.reset()

    ball_id = sim.model.body("ball").id
    joint_id = sim.model.joint("ball_free").id
    qpos_address = int(sim.model.jnt_qposadr[joint_id])

    # This scene has a free ball whose world translation is
    # represented by the first three free-joint coordinates.
    np.testing.assert_allclose(
        sim.data.xpos[ball_id],
        sim.data.qpos[qpos_address:qpos_address + 3],
        rtol=0.0,
        atol=1e-10,
    )


def test_target_marker_position_is_refreshed(sim):
    """Target placement changes must not leave stale world data."""
    reference = baseline_derived(sim)

    target_id = sim.model.body("target").id
    site_id = sim.model.site("target_marker").id

    sim.model.body_pos[target_id] += (
        0.10, 0.05, 0.0
    )

    # Compute the moved target's world-space position.
    mujoco.mj_forward(sim.model, sim.data)

    sim.reset()

    np.testing.assert_allclose(
        sim.data.site_xpos[site_id],
        reference["site_xpos"][site_id],
        rtol=0.0,
        atol=1e-10,
    )


def test_reset_recomputes_current_contacts(sim):
    """Old ball/table contacts must not survive a baseline reset."""
    reference = baseline_derived(sim)

    joint_id = sim.model.joint("ball_free").id
    address = int(sim.model.jnt_qposadr[joint_id])

    # Put the ball partly inside the tabletop in a temporary
    # private simulation, creating a definite contact scenario.
    # This is a test of contact recomputation, not a legal
    # production spawn configuration.
    sim.data.qpos[address:address + 3] = (
        0.0, 0.0, 0.52
    )
    mujoco.mj_forward(sim.model, sim.data)

    altered_contacts = contact_pairs(sim.model, sim.data)

    # Identify the table surface and the ball directly, so
    # this test does not depend on generic contact counts.
    table_geom = sim.model.geom("table_top").id

    ball_body = sim.model.body("ball").id
    ball_geoms = {
        geom
        for geom in range(sim.model.ngeom)
        if int(sim.model.geom_bodyid[geom]) == ball_body
    }

    assert any(
        table_geom in pair
        and bool(ball_geoms.intersection(pair))
        for pair in altered_contacts
    ), "Expected ball/table contact was not generated"

    sim.reset()

    assert sim.data.ncon == reference["ncon"]
    assert contact_pairs(sim.model, sim.data) == (
        reference["contacts"]
    )


def test_reset_preserves_primary_state_after_forward(sim):
    """mj_forward must not integrate position or velocity."""
    reference = baseline_derived(sim)

    sim.data.qvel[:] = 0.5
    sim.data.time = 7.0

    sim.reset()

    np.testing.assert_array_equal(
        sim.data.qpos,
        reference["qpos"],
    )
    np.testing.assert_array_equal(
        sim.data.qvel,
        reference["qvel"],
    )
    np.testing.assert_array_equal(
        sim.data.ctrl,
        reference["ctrl"],
    )
    assert sim.data.time == reference["time"]


def test_reset_does_not_advance_simulation_time(sim):
    """Forward dynamics is not a physics integration step."""
    reference = baseline_derived(sim)

    sim.data.time = 5.0

    sim.reset()

    assert float(sim.data.time) == reference["time"]


def test_reset_does_not_call_mj_step(sim, monkeypatch):
    """Guard against accidentally advancing physics."""
    def forbidden_step(*args, **kwargs):
        raise AssertionError(
            "reset() must not advance physics"
        )

    monkeypatch.setattr(
        mujoco,
        "mj_step",
        forbidden_step,
    )

    sim.reset()

    assert sim.state is SimulatorState.LOADED


def test_reset_calls_forward_exactly_once(sim, monkeypatch):
    """Forward refresh occurs once per successful reset."""
    original = mujoco.mj_forward
    calls = []

    def record_forward(model, data):
        calls.append((model, data))
        return original(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_forward",
        record_forward,
    )

    sim.reset()

    assert len(calls) == 1
    assert calls[0][0] is sim.model
    assert calls[0][1] is sim.data


def test_forward_occurs_after_state_restoration(
    sim,
    monkeypatch,
):
    """Inspect primary state at the forward-computation boundary."""
    reference = baseline_derived(sim)
    original_forward = mujoco.mj_forward
    observed = []

    sim.data.qvel[:] = 9.0
    sim.data.time = 9.0

    def inspect_forward(model, data):
        observed.append({
            "qpos": data.qpos.copy(),
            "qvel": data.qvel.copy(),
            "time": float(data.time),
        })
        return original_forward(model, data)

    monkeypatch.setattr(
        mujoco,
        "mj_forward",
        inspect_forward,
    )

    sim.reset()

    assert len(observed) == 1

    np.testing.assert_array_equal(
        observed[0]["qpos"],
        reference["qpos"],
    )
    np.testing.assert_array_equal(
        observed[0]["qvel"],
        reference["qvel"],
    )
    assert observed[0]["time"] == reference["time"]


def test_applied_forces_remain_cleared_after_forward(sim):
    """Derived-state refresh must not retain user-applied forces."""
    sim.data.qfrc_applied[:] = 30.0
    sim.data.xfrc_applied[:] = -20.0
    sim.data.qacc_warmstart[:] = 10.0

    sim.reset()

    assert np.count_nonzero(sim.data.qfrc_applied) == 0
    assert np.count_nonzero(sim.data.xfrc_applied) == 0

    # Do not assert every derived solver field is zero.
    # mj_forward may legitimately calculate nonzero values.


def test_reset_keeps_model_and_data_objects(sim):
    """No new model/data pair may be created during reset."""
    original_model = sim.model
    original_data = sim.data

    sim.reset()

    assert sim.model is original_model
    assert sim.data is original_data


def test_repeated_resets_have_stable_derived_state(sim):
    """Repeated resets reproduce the same forward-computed state."""
    sim.reset()
    first_xpos = sim.data.xpos.copy()
    first_sites = sim.data.site_xpos.copy()
    first_contacts = contact_pairs(sim.model, sim.data)

    sim.reset()
    sim.reset()

    np.testing.assert_allclose(
        sim.data.xpos, first_xpos, rtol=0.0, atol=1e-10
    )
    np.testing.assert_allclose(
        sim.data.site_xpos,
        first_sites,
        rtol=0.0,
        atol=1e-10,
    )
    assert contact_pairs(sim.model, sim.data) == first_contacts


def test_rejected_seed_does_not_call_forward(sim, monkeypatch):
    """Invalid reset arguments must fail before recomputation."""
    def forbidden_forward(*args, **kwargs):
        raise AssertionError(
            "Invalid reset called mj_forward"
        )

    monkeypatch.setattr(
        mujoco,
        "mj_forward",
        forbidden_forward,
    )

    with pytest.raises(NotImplementedError):
        sim.reset(seed=123)


def test_reset_before_loading_is_rejected():
    """Existing lifecycle restrictions still apply."""
    sim = Simulator("unused.xml")

    with pytest.raises(SimulatorLifecycleError):
        sim.reset()


def test_reset_after_closure_is_rejected(sim):
    """A closed simulator must not restart physics processing."""
    sim.close()

    with pytest.raises(SimulatorLifecycleError):
        sim.reset()


def test_independent_simulators_do_not_share_refresh_state(sim):
    """Resetting one simulator cannot change another's cached data."""
    source = os.environ["ROBOT_ARM_PANDA_XML"]
    other = Simulator(Path(source))
    other.load()

    try:
        mujoco.mj_forward(other.model, other.data)
        other_xpos = other.data.xpos.copy()

        joint = sim.model.joint("joint1")
        address = int(sim.model.jnt_qposadr[joint.id])

        sim.data.qpos[address] += 0.1
        mujoco.mj_forward(sim.model, sim.data)

        sim.reset()

        np.testing.assert_array_equal(
            other.data.xpos,
            other_xpos,
        )
    finally:
        other.close()
