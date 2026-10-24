
"""P0-005.06 — Read-only Panda joint and actuator limit validation.

Validates scalar joint limits, actuator control ranges, home
keyframe values and optional named control/position defaults.

No physics stepping, clamping or model/state mutation occurs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

import mujoco


class ModelLimitsError(ValueError):
    """One or more required model limits or defaults are invalid."""


ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
FINGER_JOINTS = ("finger_joint1", "finger_joint2")
ACTUATORS = tuple(f"actuator{i}" for i in range(1, 9))

# This is a separate Panda-specific validation policy.
# Source-model unit and transmission verification remain separate.
EXPECTED_GRIPPER_CONTROL_RANGE = (0.0, 255.0)


@dataclass(frozen=True, slots=True)
class LimitsReport:
    """Validated model bounds and home values.

    The returned maps contain immutable snapshots, not mutable
    views of MuJoCo's compiled arrays.
    """

    joint_ranges: Mapping[str, tuple[float, float]]
    actuator_ranges: Mapping[str, tuple[float, float]]
    home_positions: Mapping[str, float]
    home_controls: Mapping[str, float]


def _id(
    model: mujoco.MjModel,
    kind: mujoco.mjtObj,
    name: str,
    errors: list[str],
) -> int | None:
    """Resolve a named component, recording missing entries."""
    identifier = int(mujoco.mj_name2id(model, kind, name))

    if identifier < 0:
        errors.append(f"Missing {kind.name}: {name}")
        return None

    return identifier


def _finite_bounds(
    lower: float,
    upper: float,
    label: str,
    errors: list[str],
) -> tuple[float, float] | None:
    """Require finite, strictly ordered endpoints."""
    lo, hi = float(lower), float(upper)

    if not math.isfinite(lo) or not math.isfinite(hi):
        errors.append(f"{label}: non-finite range")
        return None

    if lo >= hi:
        errors.append(
            f"{label}: invalid range [{lo}, {hi}]"
        )
        return None

    return lo, hi


def _check_default(
    value: object,
    bounds: tuple[float, float],
    label: str,
    errors: list[str],
) -> float | None:
    """Check a numeric default without clamping or coercing booleans."""
    if isinstance(value, bool) or not isinstance(
        value, (int, float)
    ):
        errors.append(f"{label}: default must be numeric")
        return None

    number = float(value)

    if not math.isfinite(number):
        errors.append(f"{label}: default is non-finite")
        return None

    lo, hi = bounds

    if not lo <= number <= hi:
        errors.append(
            f"{label}: default {number} outside [{lo}, {hi}]"
        )
        return None

    return number


def validate_panda_limits(
    model: mujoco.MjModel,
    *,
    joint_defaults: Mapping[str, float] | None = None,
    actuator_defaults: Mapping[str, float] | None = None,
) -> LimitsReport:
    """Validate compiled Panda limits and named initial values.

    Required joint/actuator names are resolved independently.
    The home keyframe is checked using compiled qpos and ctrl
    address spaces, never assuming joint ID equals qpos index.

    All validation is read-only. Errors are accumulated and
    raised together without returning a partial report.
    """
    if not isinstance(model, mujoco.MjModel):
        raise TypeError("Expected a mujoco.MjModel")

    errors: list[str] = []
    joints: dict[str, tuple[float, float]] = {}
    controls: dict[str, tuple[float, float]] = {}
    home_positions: dict[str, float] = {}
    home_controls: dict[str, float] = {}

    for name in (*ARM_JOINTS, *FINGER_JOINTS):
        jid = _id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name, errors
        )
        if jid is None:
            continue

        # This contract requires limits for all nine scalar
        # Panda joints. The ball's free joint is not included.
        if not bool(model.jnt_limited[jid]):
            errors.append(f"{name}: joint limit disabled")
            continue

        bounds = _finite_bounds(
            *model.jnt_range[jid],
            f"joint {name}",
            errors,
        )
        if bounds is not None:
            joints[name] = bounds

    for name in ACTUATORS:
        aid = _id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR,
            name, errors,
        )
        if aid is None:
            continue

        if not bool(model.actuator_ctrllimited[aid]):
            errors.append(
                f"{name}: actuator control limit disabled"
            )
            continue

        bounds = _finite_bounds(
            *model.actuator_ctrlrange[aid],
            f"actuator {name}",
            errors,
        )
        if bounds is not None:
            controls[name] = bounds

    # The Panda gripper uses a 0–255 command scale.
    # Do not confuse these values with physical jaw separation.
    gripper = controls.get("actuator8")
    if gripper is not None and gripper != (
        EXPECTED_GRIPPER_CONTROL_RANGE
    ):
        errors.append(
            "actuator8: expected control range [0, 255], "
            f"found {gripper}"
        )

    home_id = _id(
        model, mujoco.mjtObj.mjOBJ_KEY,
        "home", errors,
    )

    if home_id is not None:
        for name, bounds in joints.items():
            jid = int(mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            ))

            # Joint IDs and qpos offsets differ in models
            # containing multi-coordinate joints.
            qadr = int(model.jnt_qposadr[jid])

            value = float(model.key_qpos[home_id, qadr])
            checked = _check_default(
                value, bounds, f"home joint {name}", errors
            )

            if checked is not None:
                home_positions[name] = checked

        for name, bounds in controls.items():
            aid = int(mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
            ))

            # Control entries use actuator IDs, not joint IDs.
            value = float(model.key_ctrl[home_id, aid])
            checked = _check_default(
                value, bounds,
                f"home actuator {name}", errors,
            )

            if checked is not None:
                home_controls[name] = checked

    # Explicitly supplied defaults must use known names.
    for provided, known, label in (
        (joint_defaults, joints, "joint"),
        (actuator_defaults, controls, "actuator"),
    ):
        if provided is None:
            continue
        if not isinstance(provided, Mapping):
            errors.append(f"{label} defaults must be a mapping")
            continue

        for name, value in provided.items():
            if name not in known:
                errors.append(
                    f"Unknown or invalid {label} default: {name!r}"
                )
                continue

            _check_default(
                value, known[name],
                f"default {label} {name}",
                errors,
            )

    if errors:
        raise ModelLimitsError(
            "Panda limits validation failed:\n"
            + "\n".join(f"  - {error}" for error in errors)
        )

    return LimitsReport(
        joint_ranges=MappingProxyType(joints),
        actuator_ranges=MappingProxyType(controls),
        home_positions=MappingProxyType(home_positions),
        home_controls=MappingProxyType(home_controls),
    )
