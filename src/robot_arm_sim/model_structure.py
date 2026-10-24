
"""P0-005.05 — Panda task-scene structural validation.

Check the expected joint and actuator counts, joint types, and
free-ball state dimensions before simulation stepping.

This validator is read-only. It does not change the model,
modify simulation state, or execute physics.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco


class ModelStructureValidationError(ValueError):
    """The compiled model violates the robot structural contract."""


@dataclass(frozen=True, slots=True)
class StructureReport:
    """Successful structural validation results."""

    arm_joint_count: int
    finger_joint_count: int
    arm_actuator_count: int
    gripper_actuator_count: int
    total_joint_count: int
    total_actuator_count: int
    ball_qpos_width: int
    ball_qvel_width: int


ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 8))
FINGER_JOINTS = ("finger_joint1", "finger_joint2")

ARM_ACTUATORS = tuple(
    f"actuator{i}" for i in range(1, 8)
)
GRIPPER_ACTUATORS = ("actuator8",)

BALL_JOINT = "ball_free"
BALL_BODY = "ball"

EXPECTED_JOINT_COUNT = 10
EXPECTED_ACTUATOR_COUNT = 8

# MuJoCo encodes position and velocity differently for
# quaternion-bearing joints.
JOINT_WIDTHS = {
    int(mujoco.mjtJoint.mjJNT_HINGE): (1, 1),
    int(mujoco.mjtJoint.mjJNT_SLIDE): (1, 1),
    int(mujoco.mjtJoint.mjJNT_BALL): (4, 3),
    int(mujoco.mjtJoint.mjJNT_FREE): (7, 6),
}


def _find(
    model: mujoco.MjModel,
    kind: mujoco.mjtObj,
    name: str,
) -> int:
    """Return a named component ID or -1 when absent."""
    return int(mujoco.mj_name2id(model, kind, name))


def _check_joint(
    model: mujoco.MjModel,
    name: str,
    expected_type: mujoco.mjtJoint,
    errors: list[str],
) -> int | None:
    """Check joint existence, type and state-array bounds."""
    joint_id = _find(
        model, mujoco.mjtObj.mjOBJ_JOINT, name
    )

    if joint_id < 0:
        errors.append(f"Missing required joint: {name}")
        return None

    actual_type = int(model.jnt_type[joint_id])

    if actual_type != int(expected_type):
        errors.append(
            f"Joint {name!r} has type {actual_type}; "
            f"expected {expected_type.name}"
        )
        return None

    qpos_width, qvel_width = JOINT_WIDTHS[actual_type]
    qpos_start = int(model.jnt_qposadr[joint_id])
    qvel_start = int(model.jnt_dofadr[joint_id])

    if not 0 <= qpos_start <= model.nq - qpos_width:
        errors.append(
            f"Joint {name!r} has invalid qpos range: "
            f"[{qpos_start}, {qpos_start + qpos_width})"
        )

    if not 0 <= qvel_start <= model.nv - qvel_width:
        errors.append(
            f"Joint {name!r} has invalid qvel range: "
            f"[{qvel_start}, {qvel_start + qvel_width})"
        )

    return joint_id


def validate_panda_structure(
    model: mujoco.MjModel,
) -> StructureReport:
    """Validate the complete composed Panda pick-and-place scene.

    The strict contract requires:
        7 arm hinge joints.
        2 gripper slide joints.
        1 ball free joint.
        7 arm actuators and 1 gripper actuator.
        Exactly 10 model joints and 8 actuators.
        Ball state dimensions of 7 qpos and 6 qvel.

    Raises:
        TypeError: The argument is not an MjModel.
        ModelStructureValidationError: Structural mismatch.

    No MjData is required, so validation cannot advance
    the physics clock.
    """
    if not isinstance(model, mujoco.MjModel):
        raise TypeError(
            "Structural validation requires mujoco.MjModel"
        )

    errors: list[str] = []

    # Counts detect unexpected extra components that a simple
    # name-existence check would otherwise overlook.
    if model.njnt != EXPECTED_JOINT_COUNT:
        errors.append(
            f"Expected {EXPECTED_JOINT_COUNT} total joints; "
            f"found {model.njnt}"
        )

    if model.nu != EXPECTED_ACTUATOR_COUNT:
        errors.append(
            f"Expected {EXPECTED_ACTUATOR_COUNT} actuators; "
            f"found {model.nu}"
        )

    # Validate each named joint individually rather than relying
    # on a particular numeric ordering in the compiled model.
    for name in ARM_JOINTS:
        _check_joint(
            model,
            name,
            mujoco.mjtJoint.mjJNT_HINGE,
            errors,
        )

    for name in FINGER_JOINTS:
        _check_joint(
            model,
            name,
            mujoco.mjtJoint.mjJNT_SLIDE,
            errors,
        )

    ball_joint_id = _check_joint(
        model,
        BALL_JOINT,
        mujoco.mjtJoint.mjJNT_FREE,
        errors,
    )

    # Verify that ball_free belongs to the actual ball body.
    # A correctly named free joint on another body is invalid.
    ball_body_id = _find(
        model, mujoco.mjtObj.mjOBJ_BODY, BALL_BODY
    )

    if ball_body_id < 0:
        errors.append("Missing required body: ball")

    if ball_joint_id is not None and ball_body_id >= 0:
        actual_body = int(model.jnt_bodyid[ball_joint_id])

        if actual_body != ball_body_id:
            errors.append(
                "ball_free is attached to the wrong body"
            )

        # A free joint contributes 7 position coordinates
        # and 6 velocity coordinates. Its address bounds
        # were checked in _check_joint().
        if int(model.jnt_type[ball_joint_id]) != int(
            mujoco.mjtJoint.mjJNT_FREE
        ):
            errors.append(
                "ball_free must use free-joint dimensions"
            )

    # Validate actuator existence by name and numeric bounds.
    # Transmission semantics are deliberately left for
    # actuator validation in the later control milestones.
    for name in (*ARM_ACTUATORS, *GRIPPER_ACTUATORS):
        actuator_id = _find(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, name
        )

        if not 0 <= actuator_id < model.nu:
            errors.append(
                f"Missing required actuator: {name}"
            )

    # Reject structural differences in the total state sizes.
    # Seven arm hinges + two finger slides + one free ball.
    expected_nq = 7 + 2 + 7
    expected_nv = 7 + 2 + 6

    if model.nq != expected_nq:
        errors.append(
            f"Expected nq={expected_nq}; found {model.nq}"
        )

    if model.nv != expected_nv:
        errors.append(
            f"Expected nv={expected_nv}; found {model.nv}"
        )

    if errors:
        details = "\n".join(
            f"  - {error}" for error in errors
        )
        raise ModelStructureValidationError(
            "Panda structural validation failed:\n"
            + details
        )

    return StructureReport(
        arm_joint_count=len(ARM_JOINTS),
        finger_joint_count=len(FINGER_JOINTS),
        arm_actuator_count=len(ARM_ACTUATORS),
        gripper_actuator_count=len(GRIPPER_ACTUATORS),
        total_joint_count=model.njnt,
        total_actuator_count=model.nu,
        ball_qpos_width=7,
        ball_qvel_width=6,
    )
