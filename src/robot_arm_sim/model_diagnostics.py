
"""P0-005.07 — Read-only MuJoCo model diagnostics.

Collect descriptive model metadata and run the previously
implemented P0-005 validation stages.

The module does not load models, change simulation state,
print to stdout or call MuJoCo's physics stepping functions.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import mujoco

from .model_ids import validate_model_names
from .model_addresses import build_address_cache
from .model_structure import validate_panda_structure
from .model_limits import validate_panda_limits


class DiagnosticStatus(str, Enum):
    """Outcome of an individual diagnostic stage."""

    PASS = "PASS"
    FAIL = "FAIL"
    BLOCKED = "BLOCKED"


@dataclass(frozen=True, slots=True)
class DiagnosticCheck:
    """Result of one independent validation stage."""

    name: str
    status: DiagnosticStatus
    message: str


@dataclass(frozen=True, slots=True)
class ModelDiagnosticReport:
    """Immutable snapshot of model metadata and check outcomes."""

    model_name: str
    source_path: str | None
    mujoco_version: str

    bodies: int
    joints: int
    nq: int
    nv: int
    actuators: int
    sites: int
    timestep: float

    checks: tuple[DiagnosticCheck, ...]

    @property
    def passed(self) -> bool:
        """Overall success requires every stage to pass."""
        return (
            bool(self.checks)
            and all(
                check.status is DiagnosticStatus.PASS
                for check in self.checks
            )
        )

    @property
    def failures(self) -> tuple[DiagnosticCheck, ...]:
        """Return failed or blocked stages, never hide failures."""
        return tuple(
            check
            for check in self.checks
            if check.status is not DiagnosticStatus.PASS
        )


class ModelDiagnosticsError(ValueError):
    """The diagnostic interface received an invalid argument."""


def _model_name(model: mujoco.MjModel) -> str:
    """Read the compiled model name from its name table.

    MuJoCo stores the model name at the beginning of the
    compiled names buffer. This is not a robot-body name.
    """
    names = bytes(model.names)
    raw_name = names.split(b"\x00", 1)[0]

    return raw_name.decode("utf-8", errors="replace")


def _check(
    name: str,
    operation,
) -> DiagnosticCheck:
    """Run a validation stage and preserve its failure message.

    Known model-validation errors are expected diagnostic
    failures. Unexpected exceptions are deliberately not
    converted to PASS or suppressed as ordinary model errors.
    """
    try:
        operation()
    except ValueError as error:
        return DiagnosticCheck(
            name=name,
            status=DiagnosticStatus.FAIL,
            message=str(error),
        )

    return DiagnosticCheck(
        name=name,
        status=DiagnosticStatus.PASS,
        message="Validation successful",
    )


def diagnose_model(
    model: mujoco.MjModel,
    *,
    source_path: str | Path | None = None,
) -> ModelDiagnosticReport:
    """Produce a deterministic snapshot of Panda model diagnostics.

    Args:
        model: An already compiled MuJoCo model.
        source_path: Optional source identification. No file is read.

    Returns:
        ModelDiagnosticReport containing model metadata and
        validation results.

    Raises:
        TypeError: Invalid model or source-path type.
        ModelDiagnosticsError: Invalid source-path content.
        Unexpected exceptions from underlying validators are
        propagated so programming defects are not misreported
        as malformed user models.

    Notes:
        The report targets the complete Panda task scene.
        Generic or standalone models may be valid MuJoCo
        models but fail this Panda-specific contract.
    """
    if not isinstance(model, mujoco.MjModel):
        raise TypeError(
            "diagnose_model requires a mujoco.MjModel"
        )

    if source_path is not None:
        if not isinstance(source_path, (str, Path)):
            raise TypeError(
                "source_path must be str, Path or None"
            )

        if not str(source_path).strip():
            raise ModelDiagnosticsError(
                "source_path must not be blank"
            )

    # Use compiled metadata, not assumptions about the Panda.
    timestep = float(model.opt.timestep)

    # Name resolution must succeed before address caching.
    # If it fails, the address stage cannot safely execute.
    try:
        resolved = validate_model_names(model)
    except ValueError as error:
        names_check = DiagnosticCheck(
            name="Named components",
            status=DiagnosticStatus.FAIL,
            message=str(error),
        )
        address_check = DiagnosticCheck(
            name="Address cache",
            status=DiagnosticStatus.BLOCKED,
            message="Required name validation failed",
        )
    else:
        names_check = DiagnosticCheck(
            name="Named components",
            status=DiagnosticStatus.PASS,
            message="Validation successful",
        )

        address_check = _check(
            "Address cache",
            lambda: build_address_cache(model, resolved),
        )

    structure_check = _check(
        "Panda structure",
        lambda: validate_panda_structure(model),
    )

    limits_check = _check(
        "Joint/actuator limits",
        lambda: validate_panda_limits(model),
    )

    # The timestep is a prerequisite for meaningful simulation.
    # This check is independent of the Panda's structure.
    if not math.isfinite(timestep) or timestep <= 0.0:
        timestep_check = DiagnosticCheck(
            name="Physics timestep",
            status=DiagnosticStatus.FAIL,
            message=(
                "Model timestep must be finite and positive; "
                f"found {timestep!r}"
            ),
        )
    else:
        timestep_check = DiagnosticCheck(
            name="Physics timestep",
            status=DiagnosticStatus.PASS,
            message="Validation successful",
        )

    return ModelDiagnosticReport(
        model_name=_model_name(model),
        source_path=(
            str(source_path) if source_path is not None
            else None
        ),
        mujoco_version=mujoco.__version__,
        bodies=int(model.nbody),
        joints=int(model.njnt),
        nq=int(model.nq),
        nv=int(model.nv),
        actuators=int(model.nu),
        sites=int(model.nsite),
        timestep=timestep,
        checks=(
            names_check,
            address_check,
            structure_check,
            limits_check,
            timestep_check,
        ),
    )


def format_model_diagnostics(
    report: ModelDiagnosticReport,
) -> str:
    """Format a diagnostic report as stable, readable plain text.

    Formatting never reruns validation or reads live model state.
    """
    if not isinstance(report, ModelDiagnosticReport):
        raise TypeError(
            "Expected a ModelDiagnosticReport"
        )

    lines = [
        "MODEL DIAGNOSTICS",
        "=" * 48,
        f"Model:           {report.model_name}",
        f"Source:          {report.source_path or 'Not supplied'}",
        f"MuJoCo version:  {report.mujoco_version}",
        f"Bodies:          {report.bodies}",
        f"Joints:          {report.joints}",
        f"Position (nq):   {report.nq}",
        f"Velocity (nv):   {report.nv}",
        f"Actuators:       {report.actuators}",
        f"Sites:           {report.sites}",
        f"Timestep:        {report.timestep:.6f} s",
        "",
        "VALIDATION",
        "-" * 48,
    ]

    for check in report.checks:
        lines.append(
            f"{check.name}: {check.status.value}"
        )

        if check.status is not DiagnosticStatus.PASS:
            for line in check.message.splitlines():
                lines.append(f"  {line}")

    lines.extend((
        "",
        f"Overall: {'PASS' if report.passed else 'FAIL'}",
    ))

    return "\n".join(lines) + "\n"
