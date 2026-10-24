
"""P0-005.07 — Model diagnostics regression tests.

Tests the complete diagnostic pipeline, including real-model
integration, failure reporting, deterministic formatting,
independent validation stages and read-only behaviour.
"""

from dataclasses import FrozenInstanceError
from pathlib import Path
import os

import mujoco
import numpy as np
import pytest

from robot_arm_sim.model_diagnostics import (
    DiagnosticCheck,
    DiagnosticStatus,
    ModelDiagnosticReport,
    ModelDiagnosticsError,
    diagnose_model,
    format_model_diagnostics,
)
from robot_arm_sim.simulation import build_scene


@pytest.fixture(scope="module")
def panda_model() -> mujoco.MjModel:
    """Provide one read-only Panda model for successful tests."""
    path = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not path:
        pytest.skip("Panda model path not configured")

    model, _ = build_scene(Path(path))
    return model


@pytest.fixture
def model() -> mujoco.MjModel:
    """Recompile a private Panda scene for destructive tests.

    No shared fixture, cached source MJCF or production model
    instance is modified by negative tests.
    """
    path = os.environ.get("ROBOT_ARM_PANDA_XML")

    if not path:
        pytest.skip("Panda model path not configured")

    model, _ = build_scene(Path(path))
    return model


def check(report, name):
    """Return the result of one diagnostic stage by name."""
    matches = [
        item for item in report.checks
        if item.name == name
    ]
    assert len(matches) == 1
    return matches[0]


def test_real_panda_diagnostics_pass(panda_model):
    """Run all five checks on the composed Panda scene."""
    report = diagnose_model(panda_model)

    assert report.passed
    assert len(report.checks) == 5
    assert report.failures == ()

    assert all(
        item.status is DiagnosticStatus.PASS
        for item in report.checks
    )


def test_diagnostic_metadata_matches_compiled_model(panda_model):
    """Diagnostics must read counts rather than hardcode them."""
    report = diagnose_model(panda_model)

    assert report.bodies == panda_model.nbody
    assert report.joints == panda_model.njnt
    assert report.nq == panda_model.nq
    assert report.nv == panda_model.nv
    assert report.actuators == panda_model.nu
    assert report.sites == panda_model.nsite
    assert report.timestep == pytest.approx(
        float(panda_model.opt.timestep)
    )
    assert report.mujoco_version == mujoco.__version__


def test_panda_expected_dimensions(panda_model):
    """Assert the structural numbers already established in P0."""
    report = diagnose_model(panda_model)

    assert report.joints == 10
    assert report.nq == 16
    assert report.nv == 15
    assert report.actuators == 8


def test_report_contains_all_required_stages(panda_model):
    """No required validation stage can be omitted."""
    report = diagnose_model(panda_model)

    assert tuple(item.name for item in report.checks) == (
        "Named components",
        "Address cache",
        "Panda structure",
        "Joint/actuator limits",
        "Physics timestep",
    )


def test_source_path_is_identification_only(panda_model):
    """Source metadata must not trigger file loading."""
    source = "nonexistent/source/panda.xml"

    report = diagnose_model(
        panda_model,
        source_path=source,
    )

    assert report.source_path == source
    assert report.passed


@pytest.mark.parametrize(
    "bad",
    ["", "   "],
)
def test_blank_source_path_rejected(panda_model, bad):
    with pytest.raises(ModelDiagnosticsError):
        diagnose_model(panda_model, source_path=bad)


@pytest.mark.parametrize("bad", [None, 5, "model"])
def test_invalid_model_argument_rejected(bad):
    with pytest.raises(TypeError):
        diagnose_model(bad)


def test_invalid_source_type_rejected(panda_model):
    with pytest.raises(TypeError):
        diagnose_model(panda_model, source_path=123)


def test_missing_required_name_blocks_address_cache(model):
    """Distinguish a failed check from a dependent blocked check.

    Compiled name data is not changed here. Instead, use
    a separate valid MuJoCo model lacking Panda components.
    """
    other = mujoco.MjModel.from_xml_string("""
    <mujoco model="incomplete">
      <worldbody>
        <body name="object">
          <joint name="some_joint" type="hinge"/>
          <geom type="sphere" size=".05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """)

    report = diagnose_model(other)

    assert not report.passed
    assert check(
        report, "Named components"
    ).status is DiagnosticStatus.FAIL

    assert check(
        report, "Address cache"
    ).status is DiagnosticStatus.BLOCKED

    assert check(
        report, "Panda structure"
    ).status is DiagnosticStatus.FAIL


def test_disabled_joint_limit_fails_overall(model):
    """A valid model with an unsafe joint must not pass."""
    joint_id = int(model.joint("joint1").id)
    model.jnt_limited[joint_id] = 0

    report = diagnose_model(model)

    assert not report.passed

    limits = check(report, "Joint/actuator limits")
    assert limits.status is DiagnosticStatus.FAIL
    assert "joint1" in limits.message

    # Other independent validation stages remain available.
    assert check(
        report, "Named components"
    ).status is DiagnosticStatus.PASS


def test_nonfinite_actuator_range_is_reported(model):
    """An invalid control range must appear in failure details."""
    actuator_id = int(model.actuator("actuator8").id)
    model.actuator_ctrlrange[actuator_id, 1] = float("nan")

    report = diagnose_model(model)

    assert not report.passed

    limits = check(report, "Joint/actuator limits")
    assert limits.status is DiagnosticStatus.FAIL
    assert "actuator8" in limits.message


def test_invalid_timestep_is_reported(model):
    """Diagnostic validation must reject a non-positive step."""
    model.opt.timestep = 0.0

    report = diagnose_model(model)

    assert not report.passed

    step = check(report, "Physics timestep")
    assert step.status is DiagnosticStatus.FAIL
    assert "timestep" in step.message


def test_diagnostic_report_is_immutable(panda_model):
    """Previously captured metadata must not be reassigned."""
    report = diagnose_model(panda_model)

    with pytest.raises(FrozenInstanceError):
        report.joints = 999

    with pytest.raises(FrozenInstanceError):
        report.checks = ()


def test_report_is_snapshot_not_live_model_view(model):
    """Later model changes must not rewrite past metadata."""
    report = diagnose_model(model)
    original_timestep = report.timestep

    model.opt.timestep = 0.004

    assert report.timestep == original_timestep


def test_repeated_diagnostics_are_deterministic(panda_model):
    """Identical model inputs produce identical reports."""
    first = diagnose_model(panda_model)
    second = diagnose_model(panda_model)

    assert first == second
    assert format_model_diagnostics(first) == (
        format_model_diagnostics(second)
    )


def test_text_report_contains_required_counts(panda_model):
    report = diagnose_model(panda_model)
    output = format_model_diagnostics(report)

    assert "MODEL DIAGNOSTICS" in output
    assert "MuJoCo version:" in output
    assert "Bodies:" in output
    assert "Joints:" in output
    assert "Position (nq):" in output
    assert "Velocity (nv):" in output
    assert "Actuators:" in output
    assert "Sites:" in output
    assert "Timestep:" in output
    assert "Overall: PASS" in output


def test_failure_text_contains_actionable_error(model):
    joint_id = int(model.joint("joint1").id)
    model.jnt_limited[joint_id] = 0

    report = diagnose_model(model)
    output = format_model_diagnostics(report)

    assert "Joint/actuator limits: FAIL" in output
    assert "joint1" in output
    assert "Overall: FAIL" in output


def test_blocked_stage_does_not_appear_as_pass():
    """A skipped dependent check is never reported as successful."""
    report = ModelDiagnosticReport(
        model_name="incomplete",
        source_path=None,
        mujoco_version=mujoco.__version__,
        bodies=1,
        joints=0,
        nq=0,
        nv=0,
        actuators=0,
        sites=0,
        timestep=0.002,
        checks=(
            DiagnosticCheck(
                name="Named components",
                status=DiagnosticStatus.FAIL,
                message="Missing joint",
            ),
            DiagnosticCheck(
                name="Address cache",
                status=DiagnosticStatus.BLOCKED,
                message="Name resolution failed",
            ),
        ),
    )

    assert not report.passed
    assert len(report.failures) == 2

    output = format_model_diagnostics(report)

    assert "Address cache: BLOCKED" in output
    assert "Overall: FAIL" in output


def test_no_checks_is_not_a_success():
    """The absence of test results cannot imply validity."""
    report = ModelDiagnosticReport(
        model_name="empty",
        source_path=None,
        mujoco_version=mujoco.__version__,
        bodies=0,
        joints=0,
        nq=0,
        nv=0,
        actuators=0,
        sites=0,
        timestep=0.002,
        checks=(),
    )

    assert report.passed is False


def test_invalid_report_type_rejected():
    with pytest.raises(TypeError):
        format_model_diagnostics(None)


def test_diagnostics_do_not_mutate_model_or_data(model):
    """Passing diagnostics must preserve model and state arrays."""
    data = mujoco.MjData(model)

    before_qpos = data.qpos.copy()
    before_qvel = data.qvel.copy()
    before_ctrl = data.ctrl.copy()
    before_ranges = model.jnt_range.copy()
    before_actuators = model.actuator_ctrlrange.copy()
    before_time = float(data.time)

    report = diagnose_model(model)

    assert report.passed

    np.testing.assert_array_equal(data.qpos, before_qpos)
    np.testing.assert_array_equal(data.qvel, before_qvel)
    np.testing.assert_array_equal(data.ctrl, before_ctrl)
    np.testing.assert_array_equal(
        model.jnt_range, before_ranges
    )
    np.testing.assert_array_equal(
        model.actuator_ctrlrange, before_actuators
    )
    assert data.time == before_time


def test_failure_diagnostics_do_not_mutate_model(model):
    """Failures must also preserve data and model parameters."""
    data = mujoco.MjData(model)

    joint_id = int(model.joint("joint1").id)
    model.jnt_limited[joint_id] = 0

    before_qpos = data.qpos.copy()
    before_limited = model.jnt_limited.copy()
    before_time = float(data.time)

    report = diagnose_model(model)

    assert not report.passed
    np.testing.assert_array_equal(data.qpos, before_qpos)
    np.testing.assert_array_equal(
        model.jnt_limited, before_limited
    )
    assert data.time == before_time


def test_formatter_does_not_print(panda_model, capsys):
    """Producing reports must remain silent by default."""
    report = diagnose_model(panda_model)
    formatted = format_model_diagnostics(report)

    assert formatted

    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_diagnostics_never_call_mj_step(
    panda_model,
    monkeypatch,
):
    """No validation stage may advance physics.

    Patch the MuJoCo stepping entrypoint so an accidental call
    fails immediately, rather than relying only on data.time.
    """
    def forbidden_step(*args, **kwargs):
        raise AssertionError(
            "Diagnostics must not call mujoco.mj_step"
        )

    monkeypatch.setattr(mujoco, "mj_step", forbidden_step)

    report = diagnose_model(panda_model)

    assert report.passed


@pytest.mark.parametrize(
    "invalid",
    [
        float("nan"),
        float("inf"),
        -float("inf"),
        0.0,
        -0.002,
    ],
)
def test_invalid_physics_timestep_never_passes(
    model,
    invalid,
):
    """Every invalid timestep must force overall failure."""
    model.opt.timestep = invalid

    report = diagnose_model(model)

    assert not report.passed
    assert check(
        report, "Physics timestep"
    ).status is DiagnosticStatus.FAIL


def test_malformed_model_reports_multiple_failures():
    """Independent validation failures should be accumulated."""
    incomplete = mujoco.MjModel.from_xml_string("""
    <mujoco model="incorrect">
      <worldbody>
        <body name="unrelated">
          <joint name="unknown" type="hinge"/>
          <geom type="sphere" size=".05" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """)

    report = diagnose_model(incomplete)

    assert not report.passed
    assert len(report.failures) >= 3

    statuses = {
        item.name: item.status
        for item in report.checks
    }

    assert statuses["Named components"] is DiagnosticStatus.FAIL
    assert statuses["Address cache"] is DiagnosticStatus.BLOCKED
    assert statuses["Panda structure"] is DiagnosticStatus.FAIL
    assert statuses["Joint/actuator limits"] is DiagnosticStatus.FAIL


def test_unexpected_programming_error_is_not_masked(
    panda_model,
    monkeypatch,
):
    """Unexpected runtime errors must not become ordinary FAILs.

    This prevents implementation defects from being mistaken
    for an invalid robot model.
    """
    import robot_arm_sim.model_diagnostics as diagnostics

    def broken_validator(_model):
        raise RuntimeError("unexpected validator defect")

    monkeypatch.setattr(
        diagnostics,
        "validate_panda_structure",
        broken_validator,
    )

    with pytest.raises(
        RuntimeError,
        match="unexpected validator defect",
    ):
        diagnose_model(panda_model)


def test_report_is_not_revalidated_when_formatted(
    panda_model,
    monkeypatch,
):
    """Formatting must use a saved snapshot only."""
    report = diagnose_model(panda_model)

    import robot_arm_sim.model_diagnostics as diagnostics

    def prohibited_validator(*args, **kwargs):
        raise AssertionError(
            "Formatting reran model validation"
        )

    monkeypatch.setattr(
        diagnostics,
        "validate_model_names",
        prohibited_validator,
    )
    monkeypatch.setattr(
        diagnostics,
        "validate_panda_structure",
        prohibited_validator,
    )
    monkeypatch.setattr(
        diagnostics,
        "validate_panda_limits",
        prohibited_validator,
    )

    output = format_model_diagnostics(report)

    assert "Overall: PASS" in output
