
"""P0-005.06 — Panda limit-validation regression tests."""

import os
from pathlib import Path

import mujoco
import numpy as np
import pytest

from robot_arm_sim.model_limits import (
    ModelLimitsError,
    validate_panda_limits,
)
from robot_arm_sim.simulation import build_scene


@pytest.fixture(scope="module")
def panda_model():
    """Load the real Panda model once for read-only tests."""
    path_string = os.environ.get("ROBOT_ARM_PANDA_XML")
    if not path_string:
        pytest.skip("Panda model path not configured")

    model, _ = build_scene(Path(path_string))
    return model



@pytest.fixture
def model() -> mujoco.MjModel:
    """Create an independent Panda model for each destructive test.

    Every invocation recompiles the composed robot scene using the
    existing read-only scene builder.

    This ensures:
    - No mutation of the shared panda_model fixture.
    - No mutation of cached MJCF or mesh assets.
    - No leakage of corrupted limits between tests.
    - No temporary model serialisation or binary files.
    """
    panda_path = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not panda_path:
        pytest.skip("Panda model path not configured")

    compiled_model, _ = build_scene(Path(panda_path))

    return compiled_model



def _save_temporary_model(source):
    """Intentionally not implemented in this proposal.

    Replace this with a pytest tmp_path fixture using
    mujoco.mj_saveModel() before approving the tests.
    """
    raise NotImplementedError


def test_real_panda_limits_valid(panda_model):
    report = validate_panda_limits(panda_model)

    assert len(report.joint_ranges) == 9
    assert len(report.actuator_ranges) == 8

    assert report.actuator_ranges["actuator8"] == (
        0.0, 255.0
    )


def test_all_home_values_inside_bounds(panda_model):
    report = validate_panda_limits(panda_model)

    for name, value in report.home_positions.items():
        low, high = report.joint_ranges[name]
        assert low <= value <= high

    for name, value in report.home_controls.items():
        low, high = report.actuator_ranges[name]
        assert low <= value <= high


@pytest.mark.parametrize("bad", [
    float("nan"),
    float("inf"),
    -float("inf"),
    True,
    "invalid",
])
def test_invalid_joint_defaults_rejected(panda_model, bad):
    with pytest.raises(ModelLimitsError):
        validate_panda_limits(
            panda_model,
            joint_defaults={"joint1": bad},
        )


@pytest.mark.parametrize("bad", [
    float("nan"),
    float("inf"),
    -float("inf"),
    -1.0,
    256.0,
    True,
])
def test_invalid_gripper_commands_rejected(panda_model, bad):
    with pytest.raises(ModelLimitsError):
        validate_panda_limits(
            panda_model,
            actuator_defaults={"actuator8": bad},
        )


def test_joint_limit_disabled_rejected(model):
    jid = model.joint("joint1").id
    model.jnt_limited[jid] = 0

    with pytest.raises(ModelLimitsError, match="limit disabled"):
        validate_panda_limits(model)


def test_actuator_limit_disabled_rejected(model):
    aid = model.actuator("actuator1").id
    model.actuator_ctrllimited[aid] = 0

    with pytest.raises(ModelLimitsError, match="limit disabled"):
        validate_panda_limits(model)


@pytest.mark.parametrize("bad", [
    float("nan"),
    float("inf"),
    -float("inf"),
])
def test_nonfinite_joint_bound_rejected(model, bad):
    jid = model.joint("joint1").id
    model.jnt_range[jid, 1] = bad

    with pytest.raises(ModelLimitsError, match="non-finite"):
        validate_panda_limits(model)


def test_reversed_actuator_range_rejected(model):
    aid = model.actuator("actuator2").id
    model.actuator_ctrlrange[aid] = [2.0, -2.0]

    with pytest.raises(ModelLimitsError, match="invalid range"):
        validate_panda_limits(model)


def test_corrupted_home_position_rejected(model):
    kid = model.key("home").id
    jid = model.joint("joint4").id
    address = int(model.jnt_qposadr[jid])

    model.key_qpos[kid, address] = 999.0

    with pytest.raises(ModelLimitsError, match="home joint"):
        validate_panda_limits(model)


def test_gripper_range_is_not_metres(model):
    aid = model.actuator("actuator8").id
    model.actuator_ctrlrange[aid] = [0.0, 0.08]

    with pytest.raises(ModelLimitsError, match="actuator8"):
        validate_panda_limits(model)


def test_validator_preserves_model(panda_model):
    before = panda_model.jnt_range.copy()
    before_ctrl = panda_model.actuator_ctrlrange.copy()
    before_key = panda_model.key_qpos.copy()

    validate_panda_limits(panda_model)

    np.testing.assert_array_equal(
        panda_model.jnt_range, before
    )
    np.testing.assert_array_equal(
        panda_model.actuator_ctrlrange, before_ctrl
    )
    np.testing.assert_array_equal(
        panda_model.key_qpos, before_key
    )
