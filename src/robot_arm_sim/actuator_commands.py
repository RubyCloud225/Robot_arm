
"""P0-007.01 — Typed Panda actuator commands.

Represent all eight actuator commands by logical names and
convert them into MuJoCo's actual compiled actuator order.

This module does not perform physics stepping, motion planning,
inverse kinematics or actuator execution.

Important:
    Raw actuator controls are NOT necessarily joint angles.
    The semantics of each actuator must be verified separately.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np


# Logical application names mapped to the selected Panda
# model's expected actuator names.
#
# This is a compatibility contract for this reference robot,
# NOT an assumption about actuator ordering in MjModel.
PANDA_ACTUATORS: tuple[tuple[str, str], ...] = (
    ("arm1", "actuator1"),
    ("arm2", "actuator2"),
    ("arm3", "actuator3"),
    ("arm4", "actuator4"),
    ("arm5", "actuator5"),
    ("arm6", "actuator6"),
    ("arm7", "actuator7"),
    ("gripper", "actuator8"),
)


def _finite_real(value: object, name: str) -> float:
    """Convert a real numeric scalar to finite float64.

    No strings, booleans, complex values or arbitrary
    user-defined conversion objects are accepted.

    This helper is shared by the typed command and registry
    so both have consistent numeric input expectations.
    """
    if isinstance(value, (bool, np.bool_)):
        raise TypeError(f"{name} must be a real number")

    if not isinstance(
        value,
        (int, float, np.integer, np.floating),
    ):
        raise TypeError(f"{name} must be a real number")

    try:
        with np.errstate(over="raise", invalid="raise"):
            converted = float(value)
    except (OverflowError, ValueError, FloatingPointError) as error:
        raise ValueError(
            f"{name} cannot be represented as float64"
        ) from error

    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")

    return converted


@dataclass(frozen=True, slots=True)
class PandaCommand:
    """One complete Panda actuator command.

    All values are raw actuator control inputs, not assumed
    joint angles, joint velocities or gripper distances.

    The seven arm channels must be supplied together,
    alongside the single gripper channel.
    """

    arm1: float
    arm2: float
    arm3: float
    arm4: float
    arm5: float
    arm6: float
    arm7: float
    gripper: float

    def __post_init__(self) -> None:
        """Validate every channel before accepting the command.

        dataclass(frozen=True) prevents ordinary reassignment
        after construction. object.__setattr__ is used here
        only while finalising the initial immutable values.
        """
        for field_name, _ in PANDA_ACTUATORS:
            converted = _finite_real(
                getattr(self, field_name),
                field_name,
            )
            object.__setattr__(
                self,
                field_name,
                converted,
            )

    def as_named_values(self) -> dict[str, float]:
        """Return a new dictionary of all eight logical values.

        Returning a new dictionary prevents callers from
        modifying the immutable command through an alias.
        """
        return {
            field_name: getattr(self, field_name)
            for field_name, _ in PANDA_ACTUATORS
        }


@dataclass(frozen=True, slots=True)
class ActuatorDescriptor:
    """Immutable record of one compiled actuator.

    control_range is None for an actuator without an enabled
    control limit.

    A control range does not specify a physical joint range.
    """

    logical_name: str
    mujoco_name: str
    actuator_id: int
    control_limited: bool
    control_range: tuple[float, float] | None


@dataclass(frozen=True, slots=True)
class ActuatorRegistry:
    """Map Panda logical channels into compiled MuJoCo order.

    This registry accepts only the expected eight-actuator
    Panda command contract.

    It is intentionally lightweight and model-specific.
    """

    descriptors: tuple[ActuatorDescriptor, ...]

    @classmethod
    def from_model(
        cls,
        model: mujoco.MjModel,
    ) -> ActuatorRegistry:
        """Inspect and validate the model's actuator registry.

        Resolve names through MuJoCo, never through assumed
        numeric positions.

        Reject missing, duplicate, unnamed or unexpected
        actuators rather than silently generating an
        incomplete or incorrectly ordered command.
        """
        if not isinstance(model, mujoco.MjModel):
            raise TypeError("model must be mujoco.MjModel")

        expected_count = len(PANDA_ACTUATORS)

        if model.nu != expected_count:
            raise ValueError(
                "Panda actuator count mismatch: expected "
                f"{expected_count}, received {model.nu}"
            )

        compiled_names: list[str] = []

        for actuator_id in range(model.nu):
            name = mujoco.mj_id2name(
                model,
                mujoco.mjtObj.mjOBJ_ACTUATOR,
                actuator_id,
            )

            if not name:
                raise ValueError(
                    "Compiled Panda model contains an "
                    "unnamed actuator"
                )

            compiled_names.append(name)

        if len(set(compiled_names)) != model.nu:
            raise ValueError(
                "Compiled actuator names are not unique"
            )

        expected_names = {
            mujoco_name
            for _, mujoco_name in PANDA_ACTUATORS
        }

        if set(compiled_names) != expected_names:
            missing = sorted(
                expected_names - set(compiled_names)
            )
            unexpected = sorted(
                set(compiled_names) - expected_names
            )
            raise ValueError(
                "Unexpected Panda actuator configuration: "
                f"missing={missing}, unexpected={unexpected}"
            )

        descriptors: list[ActuatorDescriptor] = []

        for logical_name, mujoco_name in PANDA_ACTUATORS:
            actuator_id = int(
                mujoco.mj_name2id(
                    model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    mujoco_name,
                )
            )

            if actuator_id < 0:
                raise ValueError(
                    f"Missing actuator: {mujoco_name}"
                )

            limited = bool(
                model.actuator_ctrllimited[actuator_id]
            )

            control_range = None

            if limited:
                low = float(
                    model.actuator_ctrlrange[actuator_id, 0]
                )
                high = float(
                    model.actuator_ctrlrange[actuator_id, 1]
                )

                if (
                    not math.isfinite(low)
                    or not math.isfinite(high)
                    or low >= high
                ):
                    raise ValueError(
                        "Invalid control range for "
                        f"{mujoco_name}"
                    )

                control_range = (low, high)

            descriptors.append(
                ActuatorDescriptor(
                    logical_name=logical_name,
                    mujoco_name=mujoco_name,
                    actuator_id=actuator_id,
                    control_limited=limited,
                    control_range=control_range,
                )
            )

        return cls(descriptors=tuple(descriptors))

    def to_vector(
        self,
        command: PandaCommand,
    ) -> np.ndarray:
        """Convert a complete named command to MuJoCo order.

        Produces a private float64 array of shape (8,).

        Range violations are errors, not clamped values.

        No actuator writes or MuJoCo physics calls occur.
        """
        if not isinstance(command, PandaCommand):
            raise TypeError(
                "command must be PandaCommand"
            )

        expected_count = len(PANDA_ACTUATORS)

        if len(self.descriptors) != expected_count:
            raise ValueError(
                "Actuator registry is incomplete"
            )

        expected_pairs = dict(PANDA_ACTUATORS)

        if any(
            expected_pairs.get(descriptor.logical_name)
            != descriptor.mujoco_name
            for descriptor in self.descriptors
        ):
            raise ValueError(
                "Actuator registry name mapping is invalid"
            )

        # Guard against manually constructed registries
        # with ambiguous, missing or duplicate channels.
        expected_logical = {
            logical_name
            for logical_name, _ in PANDA_ACTUATORS
        }

        actual_logical = {
            descriptor.logical_name
            for descriptor in self.descriptors
        }

        actuator_ids = [
            descriptor.actuator_id
            for descriptor in self.descriptors
        ]

        if (
            actual_logical != expected_logical
            or len(actual_logical) != expected_count
            or set(actuator_ids) != set(range(expected_count))
            or len(set(actuator_ids)) != expected_count
        ):
            raise ValueError(
                "Actuator registry mapping is invalid"
            )

        values = command.as_named_values()
        vector = np.empty(expected_count, dtype=np.float64)

        for descriptor in self.descriptors:
            value = values[descriptor.logical_name]

            if descriptor.control_limited:
                if descriptor.control_range is None:
                    raise ValueError(
                        "Missing control range for "
                        f"{descriptor.mujoco_name}"
                    )

                low, high = descriptor.control_range

                if (
                    not math.isfinite(low)
                    or not math.isfinite(high)
                    or low >= high
                ):
                    raise ValueError(
                        "Invalid control range for "
                        f"{descriptor.mujoco_name}"
                    )

                if not low <= value <= high:
                    raise ValueError(
                        f"{descriptor.mujoco_name} control "
                        f"{value} is outside [{low}, {high}]"
                    )

            vector[descriptor.actuator_id] = value

        return vector

    def describe(self) -> tuple[ActuatorDescriptor, ...]:
        """Return immutable registry metadata for documentation."""
        return self.descriptors
