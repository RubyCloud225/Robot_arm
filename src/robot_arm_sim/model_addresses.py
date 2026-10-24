
"""P0-005.04 — Model-bound MuJoCo identifier and address cache.

Resolve joint, actuator, body and site identifiers once per model.

Joint IDs are NOT qpos or qvel addresses.
Actuator IDs are NOT joint IDs.

This module never modifies an MjModel or MjData and never steps physics.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping

import mujoco

from .model_ids import ResolvedModelIds


class ModelAddressError(ValueError):
    """Resolved identifiers are invalid for the supplied model."""


# MuJoCo stores different numbers of position and velocity values
# depending on joint type.
_JOINT_WIDTHS = {
    int(mujoco.mjtJoint.mjJNT_HINGE): (1, 1),
    int(mujoco.mjtJoint.mjJNT_SLIDE): (1, 1),
    int(mujoco.mjtJoint.mjJNT_BALL): (4, 3),
    int(mujoco.mjtJoint.mjJNT_FREE): (7, 6),
}


@dataclass(frozen=True, slots=True)
class JointAddress:
    """Validated address information for one named joint."""

    name: str
    joint_id: int
    joint_type: int
    qpos_start: int
    qpos_width: int
    qvel_start: int
    qvel_width: int

    @property
    def qpos_slice(self) -> slice:
        """Return the joint's complete position coordinate range."""
        return slice(
            self.qpos_start,
            self.qpos_start + self.qpos_width,
        )

    @property
    def qvel_slice(self) -> slice:
        """Return the joint's complete velocity coordinate range."""
        return slice(
            self.qvel_start,
            self.qvel_start + self.qvel_width,
        )


@dataclass(frozen=True, slots=True)
class ModelAddressCache:
    """Read-only model-specific component and address cache.

    The original model is retained to prevent accidental cache reuse
    with another compiled model.

    Numerical identifiers remain valid only for this model's
    unchanged compiled structure.
    """

    model: mujoco.MjModel
    joints: Mapping[str, JointAddress]
    actuators: Mapping[str, int]
    bodies: Mapping[str, int]
    sites: Mapping[str, int]

    def assert_model(self, model: mujoco.MjModel) -> None:
        """Reject reuse of the cache against a different model."""
        if model is not self.model:
            raise ModelAddressError(
                "Address cache belongs to a different MjModel"
            )

    def joint(
        self,
        name: str,
        *,
        model: mujoco.MjModel,
    ) -> JointAddress:
        """Return a validated joint address for the supplied model."""
        self.assert_model(model)

        try:
            return self.joints[name]
        except KeyError as error:
            raise ModelAddressError(
                f"Joint {name!r} is not in the validated cache"
            ) from error

    def actuator(
        self,
        name: str,
        *,
        model: mujoco.MjModel,
    ) -> int:
        """Return an actuator's control-array index."""
        self.assert_model(model)

        try:
            return self.actuators[name]
        except KeyError as error:
            raise ModelAddressError(
                f"Actuator {name!r} is not in the validated cache"
            ) from error


def _validate_named_id(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    name: str,
    identifier: int,
    count: int,
) -> int:
    """Verify ID bounds and its round-trip name against this model."""
    if isinstance(identifier, bool) or not isinstance(identifier, int):
        raise ModelAddressError(
            f"Invalid identifier type for {name!r}"
        )

    if not 0 <= identifier < count:
        raise ModelAddressError(
            f"Identifier {identifier} for {name!r} "
            f"is outside [0, {count})"
        )

    actual_name = mujoco.mj_id2name(
        model, object_type, identifier
    )

    if actual_name != name:
        raise ModelAddressError(
            f"Identifier mismatch for {name!r}: "
            f"model contains {actual_name!r} at ID {identifier}"
        )

    return identifier


def build_address_cache(
    model: mujoco.MjModel,
    resolved: ResolvedModelIds,
) -> ModelAddressCache:
    """Build an independently validated cache of model addresses.

    Args:
        model: The compiled model whose indexing is being cached.
        resolved: Name-to-ID mappings obtained during P0-005.03.

    Returns:
        A model-bound cache with immutable public mappings.

    Raises:
        TypeError: Invalid argument types.
        ModelAddressError: Stale IDs, invalid offsets or bad widths.

    Notes:
        The function validates the mappings again because callers
        may have altered the dictionaries returned by P0-005.03.
        """
    if not isinstance(model, mujoco.MjModel):
        raise TypeError("Expected a mujoco.MjModel")

    if not isinstance(resolved, ResolvedModelIds):
        raise TypeError("Expected ResolvedModelIds")

    joints: dict[str, JointAddress] = {}

    for name, identifier in resolved.joints.items():
        joint_id = _validate_named_id(
            model,
            mujoco.mjtObj.mjOBJ_JOINT,
            name,
            identifier,
            model.njnt,
        )

        joint_type = int(model.jnt_type[joint_id])

        if joint_type not in _JOINT_WIDTHS:
            raise ModelAddressError(
                f"Unsupported joint type for {name!r}: {joint_type}"
            )

        qpos_width, qvel_width = _JOINT_WIDTHS[joint_type]
        qpos_start = int(model.jnt_qposadr[joint_id])
        qvel_start = int(model.jnt_dofadr[joint_id])

        if (
            qpos_start < 0
            or qpos_start + qpos_width > model.nq
        ):
            raise ModelAddressError(
                f"Invalid qpos address for joint {name!r}"
            )

        if (
            qvel_start < 0
            or qvel_start + qvel_width > model.nv
        ):
            raise ModelAddressError(
                f"Invalid qvel address for joint {name!r}"
            )

        joints[name] = JointAddress(
            name=name,
            joint_id=joint_id,
            joint_type=joint_type,
            qpos_start=qpos_start,
            qpos_width=qpos_width,
            qvel_start=qvel_start,
            qvel_width=qvel_width,
        )

    categories = (
        (
            resolved.actuators,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            model.nu,
        ),
        (
            resolved.bodies,
            mujoco.mjtObj.mjOBJ_BODY,
            model.nbody,
        ),
        (
            resolved.sites,
            mujoco.mjtObj.mjOBJ_SITE,
            model.nsite,
        ),
    )

    checked_categories: list[dict[str, int]] = []

    for mapping, object_type, count in categories:
        checked: dict[str, int] = {}

        for name, identifier in mapping.items():
            checked[name] = _validate_named_id(
                model, object_type, name, identifier, count
            )

        checked_categories.append(checked)

    actuators, bodies, sites = checked_categories

    # Copy input mappings before wrapping. A later mutation of the
    # P0-005.03 result must not corrupt the validated cache.
    return ModelAddressCache(
        model=model,
        joints=MappingProxyType(dict(joints)),
        actuators=MappingProxyType(dict(actuators)),
        bodies=MappingProxyType(dict(bodies)),
        sites=MappingProxyType(dict(sites)),
    )
