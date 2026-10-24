
"""P0-005.03 — Resolve required MuJoCo components by name.

Names provide a stable logical interface to compiled model objects.
Numeric IDs are model-specific and must not be assumed constant
between differently composed or recompiled models.

This module performs read-only structural validation.
It does not move the robot, alter state, or step physics.
"""

from dataclasses import dataclass
from typing import Iterable

import mujoco


class ModelStructureError(ValueError):
    """A required named model component is missing or invalid."""


@dataclass(frozen=True, slots=True)
class RequiredModelNames:
    """Names required for the composed Panda pick-and-place task."""

    arm_joints: tuple[str, ...] = (
        "joint1", "joint2", "joint3", "joint4",
        "joint5", "joint6", "joint7",
    )
    finger_joints: tuple[str, ...] = (
        "finger_joint1", "finger_joint2",
    )
    arm_actuators: tuple[str, ...] = (
        "actuator1", "actuator2", "actuator3",
        "actuator4", "actuator5", "actuator6",
        "actuator7",
    )
    gripper_actuators: tuple[str, ...] = ("actuator8",)
    robot_bodies: tuple[str, ...] = (
        "hand", "left_finger", "right_finger",
    )
    scene_bodies: tuple[str, ...] = ("ball", "target")
    scene_joints: tuple[str, ...] = ("ball_free",)
    sites: tuple[str, ...] = ("target_marker",)


@dataclass(frozen=True, slots=True)
class ResolvedModelIds:
    """Model-specific identifiers resolved from verified names.

    These numeric IDs only belong to the model used for resolution.
    Do not reuse them against an independently compiled model.
    """

    joints: dict[str, int]
    actuators: dict[str, int]
    bodies: dict[str, int]
    sites: dict[str, int]


def resolve_id(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    name: str,
) -> int:
    """Resolve one required name to a valid model-specific ID.

    Raises:
        TypeError: Unsupported model, object type or name.
        ModelStructureError: Named object does not exist.
    """
    if not isinstance(model, mujoco.MjModel):
        raise TypeError("Expected a mujoco.MjModel")

    supported_types = {
        mujoco.mjtObj.mjOBJ_JOINT,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        mujoco.mjtObj.mjOBJ_BODY,
        mujoco.mjtObj.mjOBJ_SITE,
    }

    if not isinstance(object_type, mujoco.mjtObj):
        raise TypeError("Expected a mujoco.mjtObj value")

    if object_type not in supported_types:
        raise TypeError(
            f"Unsupported MuJoCo object type: {object_type}"
        )

    if not isinstance(name, str) or not name.strip():
        raise ValueError("Component name must be non-empty")
    
    if name != name.strip():
        raise ModelStructureError(
            f"Component name contains surrounding whitespace: {name!r}"
        )


    # MuJoCo returns -1 when the requested name is not found.
    # Never use that negative value as an array index.
    identifier = mujoco.mj_name2id(model, object_type, name)

    if identifier < 0:
        raise ModelStructureError(
            f"Required {object_type.name} component "
            f"{name!r} is missing from the compiled model"
        )

    # Confirm that the returned index resolves back to the same
    # name. This adds an inexpensive integrity check.
    resolved_name = mujoco.mj_id2name(
        model, object_type, identifier
    )

    if resolved_name != name:
        raise ModelStructureError(
            f"Name-resolution mismatch for {name!r}: "
            f"resolved to {resolved_name!r}"
        )

    return int(identifier)


def _resolve_group(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    names: Iterable[str],
    errors: list[str],
) -> dict[str, int]:
    """Resolve a category while collecting all missing names."""
    result: dict[str, int] = {}

    for name in names:
        if name in result:
            errors.append(
                f"Duplicate required name in schema: {name!r}"
            )
            continue

        try:
            result[name] = resolve_id(
                model, object_type, name
            )
        except (ModelStructureError, ValueError, TypeError) as error:
            errors.append(str(error))

    return result


def validate_model_names(
    model: mujoco.MjModel,
    required: RequiredModelNames | None = None,
) -> ResolvedModelIds:
    """Resolve every required model object or fail before stepping.

    No partial result is returned when validation fails.
    All categories are checked to produce useful diagnostics.
    """
    if not isinstance(model, mujoco.MjModel):
        raise TypeError("Expected a mujoco.MjModel")

    if required is None:
        required = RequiredModelNames()

    if not isinstance(required, RequiredModelNames):
        raise TypeError("Expected RequiredModelNames")

    
    groups = (
        required.arm_joints,
        required.finger_joints,
        required.arm_actuators,
        required.gripper_actuators,
        required.robot_bodies,
        required.scene_bodies,
        required.scene_joints,
        required.sites,
    )

    if not any(groups):
        raise ModelStructureError(
            "Required model-name schema cannot be empty"
        )
    
    errors: list[str] = []

    joints = _resolve_group(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        (
            *required.arm_joints,
            *required.finger_joints,
            *required.scene_joints,
        ),
        errors,
    )

    actuators = _resolve_group(
        model,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        (
            *required.arm_actuators,
            *required.gripper_actuators,
        ),
        errors,
    )

    bodies = _resolve_group(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        (
            *required.robot_bodies,
            *required.scene_bodies,
        ),
        errors,
    )

    sites = _resolve_group(
        model,
        mujoco.mjtObj.mjOBJ_SITE,
        required.sites,
        errors,
    )

    if errors:
        details = "\n".join(
            f"  - {error}" for error in errors
        )
        raise ModelStructureError(
            "Model structural name validation failed:\n"
            + details
        )

    return ResolvedModelIds(
        joints=joints,
        actuators=actuators,
        bodies=bodies,
        sites=sites,
    )
