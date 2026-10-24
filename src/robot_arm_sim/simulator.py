
"""P0-006.03 — Simulator lifecycle and transient-state reset.

Own an independently compiled Panda simulation and restore its
initial physical state without recompilation or physics stepping.

The reset procedure uses MuJoCo's native reset operation to clear
transient simulation data, then restores the captured task baseline.

Forward kinematics/contact recomputation belongs to P0-006.04.
"""

from __future__ import annotations

from dbm import error
import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from random import seed

import mujoco
import numpy as np

from .model_instance import SimulationInstance


class SimulatorState(str, Enum):
    """Explicit public lifecycle states."""

    NEW = "NEW"
    LOADED = "LOADED"
    CLOSED = "CLOSED"


class SimulatorLifecycleError(RuntimeError):
    """An operation is invalid in the current lifecycle state."""


@dataclass(frozen=True, slots=True, eq=False)
class _ResetSnapshot:
    """Independent baseline captured after successful scene loading.

    The NumPy arrays contain copies of the initial model/data
    values rather than views into MuJoCo's mutable buffers.
    """

    qpos: np.ndarray
    qvel: np.ndarray
    ctrl: np.ndarray
    act: np.ndarray
    target_position: np.ndarray
    time: float


def _readonly_copy(values: np.ndarray) -> np.ndarray:
    """Make an independent, read-only copy of a MuJoCo array."""
    result = np.array(values, copy=True)
    result.setflags(write=False)
    return result


def _capture_initial_state(
    instance: SimulationInstance,
) -> _ResetSnapshot:
    """Capture the composed Panda's initial primary state.

    Target position is stored in MjModel.body_pos because the
    goal is a fixed body rather than a free-joint object.
    """
    model = instance.model
    data = instance.data

    target_id = int(
        mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            "target",
        )
    )
    if target_id < 0:
        raise ValueError(
            "Cannot capture reset state: target body is missing"
        )

    if not math.isfinite(float(data.time)):
        raise ValueError(
            "Cannot capture reset state: time is not finite"
        )

    return _ResetSnapshot(
        qpos=_readonly_copy(data.qpos),
        qvel=_readonly_copy(data.qvel),
        ctrl=_readonly_copy(data.ctrl),
        act=_readonly_copy(data.act),
        target_position=_readonly_copy(
            model.body_pos[target_id]
        ),
        time=float(data.time),
    )


class Simulator:
    """Own one composed Panda scene and its reset baseline.

    Lifecycle:
        NEW -> load() -> LOADED -> close() -> CLOSED

    The loaded MjModel and MjData objects are retained across
    reset calls. Closing permanently releases wrapper ownership.

    This class does not create a viewer or rendering context.
    """

    __slots__ = (
        "_panda_path",
        "_instance",
        "_state",
        "_reset_snapshot",
    )

    def __init__(self, panda_path: str | Path) -> None:
        """Record the model location without compiling anything."""
        if not isinstance(panda_path, (str, Path)):
            raise TypeError("panda_path must be str or Path")

        if not str(panda_path).strip():
            raise ValueError("panda_path cannot be blank")

        self._panda_path = Path(panda_path)
        self._instance: SimulationInstance | None = None
        self._reset_snapshot: _ResetSnapshot | None = None
        self._state = SimulatorState.NEW

    @property
    def state(self) -> SimulatorState:
        """Return the current lifecycle state."""
        return self._state

    @property
    def is_loaded(self) -> bool:
        """Whether a model/data pair is currently available."""
        return self._state is SimulatorState.LOADED

    def _require_loaded(self) -> SimulationInstance:
        """Enforce the LOADED state before accessing simulation data."""
        if (
            self._state is not SimulatorState.LOADED
            or self._instance is None
        ):
            raise SimulatorLifecycleError(
                "Simulator must be loaded before this operation; "
                f"current state is {self._state.value}"
            )
        return self._instance

    @property
    def model(self) -> mujoco.MjModel:
        """Return the currently owned compiled model."""
        return self._require_loaded().model

    @property
    def data(self) -> mujoco.MjData:
        """Return the currently owned mutable simulation state."""
        return self._require_loaded().data

    def load(self) -> None:
        """Create a fresh scene and capture its reset baseline.

        Publish the loaded instance only after scene construction
        and baseline capture have both succeeded.
        """
        if self._state is not SimulatorState.NEW:
            raise SimulatorLifecycleError(
                "load() is allowed only from NEW; "
                f"current state is {self._state.value}"
            )

        instance = SimulationInstance.from_robot_scene(
            self._panda_path
        )
        snapshot = _capture_initial_state(instance)

        self._instance = instance
        self._reset_snapshot = snapshot
        self._state = SimulatorState.LOADED

    def reset(self, seed: int | None = None) -> None:
        """Restore the Panda scene to a consistent initial state.

        P0-006.02:
            Restore robot, gripper, ball, target and time.

        P0-006.03:
            Clear transient MuJoCo data using mj_resetData()
            and restore the captured actuator activation.

        P0-006.04:
            Recompute forward kinematics, geometry transforms,
            velocities, accelerations and current contacts
            using mj_forward().

        The same MjModel and MjData objects are retained.

        Reset never calls mj_step() or advances simulation time.
        """
        instance = self._require_loaded()

        # Seeded random resets remain outside P0-006.
        # Reject before modifying any simulation state.
        if seed is not None:
            raise NotImplementedError(
                "Seeded resets are reserved for P0-010"
            )

        snapshot = self._reset_snapshot

        if snapshot is None:
            raise SimulatorLifecycleError(
                "Loaded simulator has no reset snapshot"
            )

        model = instance.model
        data = instance.data

        # The fixed target belongs to MjModel rather than qpos.
        # Resolve it by name to avoid relying on a numeric body ID.
        target_id = int(
            mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                "target",
            )
        )

        if target_id < 0:
            raise SimulatorLifecycleError(
                "Cannot reset: target body is missing"
            )

        # Perform all ordinary compatibility checks before
        # invoking the native reset. An invalid snapshot must
        # not result in partially cleared simulation data.
        #
        # Note that actuator activation (na) is separate from
        # actuator control (nu), so their sizes must not be
        # assumed identical.
        if (
            data.qpos.shape != snapshot.qpos.shape
            or data.qvel.shape != snapshot.qvel.shape
            or data.ctrl.shape != snapshot.ctrl.shape
            or data.act.shape != snapshot.act.shape
            or model.body_pos[target_id].shape
            != snapshot.target_position.shape
        ):
            raise SimulatorLifecycleError(
                "Cannot reset: current state dimensions "
                "do not match the initial snapshot"
            )

        # Step 1 — Clear MuJoCo-managed transient state.
        #
        # This clears stale externally applied forces,
        # solver warm-start values and other resettable
        # engine-owned data.
        #
        # The native reset also replaces primary state with
        # MuJoCo defaults, so the task-specific baseline
        # must be restored immediately afterwards.
        mujoco.mj_resetData(model, data)

        # Step 2 — Restore the original composed scene.
        #
        # Use full-array copies rather than reconstructing
        # individual joint indices. This preserves all
        # articulated joints and the ball's four-component
        # orientation quaternion.
        np.copyto(data.qpos, snapshot.qpos)
        np.copyto(data.qvel, snapshot.qvel)

        # Restore initial actuator commands and activation.
        # These must not be blindly set to zero.
        np.copyto(data.ctrl, snapshot.ctrl)
        np.copyto(data.act, snapshot.act)

        # Restore the target's fixed model-space position.
        # This must happen before forward kinematics so its
        # world-space transform is calculated correctly.
        np.copyto(
            model.body_pos[target_id],
            snapshot.target_position,
        )

        # Restore the exact baseline simulation clock.
        data.time = snapshot.time

        # Step 3 — Recompute derived simulation state.
        #
        # This updates body/site/geometry transforms, current
        # contacts and other MuJoCo forward-dynamics outputs.
        #
        # mj_forward does not integrate qpos or qvel and does
        # not advance data.time.
        #
        # No physics step is performed here.
        mujoco.mj_forward(model, data)
        
    def step(self, control: object = None, *, n_substeps: int = 1,) -> None:
        """Advance the simulation by an exact number of physics steps.
        P0-006.05 defines one substep as one mujoco.mj_step()
        using the loaded model's physics timestep.
        
        Args:
            control:
                None to preserve existing data.ctrl, or a full
                vector of MuJoCo actuator control inputs ordered
                exactly as the compiled model's actuators.
                Values are NOT assumed to be joint angles.

            n_substeps:
                Number of consecutive physics steps. Must be
                a positive Python integer, excluding bool.

        Raises:
            SimulatorLifecycleError:
                Simulator has not been loaded or was closed.
            TypeError:
                Invalid substep type or control representation.
            ValueError:
                Invalid substep count, control shape, values,
                actuator ranges or physics timestep.
        Notes:
            Input validation occurs before actuator writes
            and before any physics stepping.

        A command is held constant for the entire call.

        A successful return means all requested physics
        steps completed; this does not certify stability,
        reachability or safety of the resulting trajectory.

        Unexpected engine exceptions propagate. Successful
        earlier substeps are not automatically rolled back.
        """
        instance = self._require_loaded()
        model = instance.model
        data = instance.data

        # Reject booleans explicitly: bool is a subclass of int
        # in Python, but True must never silently mean one step.
        if type(n_substeps) is not int:
            raise TypeError(
                "n_substeps must be a positive Python integer"
            )

        if n_substeps <= 0:
            raise ValueError(
                "n_substeps must be greater than zero"
            )

        # A modified or corrupted model timestep must not
        # reach MuJoCo's physics integrator.
        timestep = float(model.opt.timestep)

        if not np.isfinite(timestep) or timestep <= 0.0:
            raise ValueError(
                "Physics timestep must be finite and positive"
            )

        validated_control = None

        # A missing command means: preserve the currently
        # applied MuJoCo actuator controls.
        #
        # Only explicitly supplied commands should undergo
        # conversion, shape validation and range checking.
        if control is not None:

            # Reject strings, byte sequences and mappings.
            # These are not acceptable actuator command vectors.
            if isinstance(control, (str, bytes, dict)):
                raise TypeError(
                    "control must be a numeric actuator vector"
                )

            try:
                raw = np.asarray(control)
            except (TypeError, ValueError) as error:
                raise TypeError(
                    "control must be a numeric actuator vector"
                ) from error

            # The supplied command must contain one value
            # per actuator in the compiled MuJoCo model.
            if raw.ndim != 1:
                raise ValueError(
                    "control must be a one-dimensional vector"
                )

            if raw.shape != (model.nu,):
                raise ValueError(
                    "control shape mismatch: expected "
                    f"({model.nu},), received {raw.shape}"
                )

            # Accept real numeric arrays only.
            # Prevent implicit conversion of booleans,
            # strings or complex values into controls.
            if raw.dtype.kind not in ("i", "u", "f"):
                raise TypeError(
                    "control must contain real numeric values"
                )

            # Copy into private float64 storage so the
            # caller cannot change the validated command.
            validated_control = np.array(
                raw,
                dtype=np.float64,
                copy=True,
            )

            if not np.all(np.isfinite(validated_control)):
                raise ValueError(
                    "control contains NaN or infinity"
                )

            # Apply the model's verified actuator ranges.
            # These are actuator control units, not
            # necessarily joint positions or angles.
            for actuator_id in range(model.nu):
                name = mujoco.mj_id2name(
                    model,
                    mujoco.mjtObj.mjOBJ_ACTUATOR,
                    actuator_id,
                ) or f"actuator {actuator_id}"

                if not bool(
                    model.actuator_ctrllimited[actuator_id]
                ):
                    continue

                low = float(
                    model.actuator_ctrlrange[actuator_id, 0]
                )
                high = float(
                    model.actuator_ctrlrange[actuator_id, 1]
                )

                if (
                    not np.isfinite(low)
                    or not np.isfinite(high)
                    or low >= high
                ):
                    raise ValueError(
                        f"Invalid control range for {name}"
                    )

                value = validated_control[actuator_id]

                if value < low or value > high:
                    raise ValueError(
                        f"Control for {name} is outside "
                        f"its permitted range [{low}, {high}]: "
                        f"{value}"
                    )

        # The remaining code stays outside the conditional.
        #
        # If control was None, validated_control stays None
        # and the current actuator controls are preserved.
        if validated_control is not None:
            np.copyto(data.ctrl, validated_control)

        for _ in range(n_substeps):
            mujoco.mj_step(model, data)


    def close(self) -> None:
        """Release wrapper references and permanently close."""
        if self._state is SimulatorState.CLOSED:
            return

        self._reset_snapshot = None
        self._instance = None
        self._state = SimulatorState.CLOSED
