
"""Explicit, read-only loading of MuJoCo MJCF model files.

P0-005.01: Validate model paths and report useful loader failures.

This module does not download assets, modify source files, run physics
steps or validate the robot-specific joint/actuator schema.
"""
import stat
from pathlib import Path

import mujoco


class ModelLoadError(ValueError):
    """A model cannot be located, read, parsed or compiled."""


def _resolve_model_path(path: str | Path) -> Path:
    """Resolve a model path and reject unusable filesystem entries.

    Relative paths are interpreted against the current working directory.
    This function does not create directories or modify files.
    """
    if not isinstance(path, (str, Path)):
        raise TypeError("Model path must be a string or pathlib.Path")

    if isinstance(path, str) and not path.strip():
        raise ModelLoadError("Model path cannot be empty")

    try:
        candidate = Path(path).expanduser().resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise ModelLoadError(
            f"Cannot resolve model path {str(path)!r}: {error}"
        ) from error

    if not candidate.is_file():
        raise ModelLoadError(
            f"Model path is not a regular file: {candidate}"
        )

    # Check permission metadata for consistent diagnostics even when
    # testing under an account that has elevated filesystem privileges.
    # This is not a complete access-control check (e.g. ACLs).
    try:
        mode = candidate.stat().st_mode
        if not mode & (stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH):
            raise ModelLoadError(
                f"Model file has no read permission bits: {candidate}"
            )
    except OSError as error:
        raise ModelLoadError(
            f"Cannot inspect model file {candidate}: {error}"
        ) from error

    # Opening detects actual access failures before invoking MuJoCo.
    # MuJoCo itself subsequently opens and parses the file.
    try:
        with candidate.open("rb"):
            pass
    except OSError as error:
        raise ModelLoadError(
            f"Cannot open model file {candidate}: {error}"
        ) from error

    return candidate


def load_model(path: str | Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Load and compile a fresh MuJoCo model and state.

    Args:
        path: MJCF/XML model path, including any relative asset references.

    Returns:
        A newly compiled MjModel and corresponding fresh MjData.

    Raises:
        TypeError: The path argument has an unsupported type.
        ModelLoadError: The model file or its dependencies cannot be
            loaded, parsed or compiled.

    Notes:
        Relative asset paths are resolved by MuJoCo using its normal
        XML/model loading rules. No physics step is performed.

        Successful loading proves that MuJoCo accepts the model; it does
        not establish robot reachability, control safety or physical
        correctness.
    """
    model_path = _resolve_model_path(path)

    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except (ValueError, OSError, RuntimeError) as error:
        raise ModelLoadError(
            f"Failed to load or compile MuJoCo model "
            f"{model_path} ({type(error).__name__}): {error}"
        ) from error

    # Allocate a fresh state buffer owned by this model. Do not reuse
    # state from another simulation or advance the physics clock.
    try:
        data = mujoco.MjData(model)
    except (ValueError, RuntimeError) as error:
        raise ModelLoadError(
            f"Failed to initialise MuJoCo state for "
            f"{model_path}: {error}"
        ) from error

    return model, data
