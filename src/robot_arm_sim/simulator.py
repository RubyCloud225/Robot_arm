
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


def _controller_substep_count(
    period: float,
    timestep: float,
) -> int:
    """Convert a controller period into exact physics substeps.

    A controller update may span several physics integrations,
    but those integrations must all have the same model timestep.

    Reject incompatible periods instead of silently choosing
    a shorter or longer simulated duration.

    This function performs no simulation writes and does not
    invoke MuJoCo.
    """
    # bool is technically an int in Python, but True and False
    # are not meaningful duration arguments.
    for name, value in (
        ("controller period", period),
        ("physics timestep", timestep),
    ):
        if isinstance(value, (bool, np.bool_)):
            raise TypeError(
                f"{name} must be a real numeric value"
            )

        if not isinstance(
            value,
            (int, float, np.integer, np.floating),
        ):
            raise TypeError(
                f"{name} must be a real numeric value"
            )

        if not np.isfinite(value) or value <= 0:
            raise ValueError(
                f"{name} must be finite and positive"
            )

    # Dividing durations expressed in seconds gives a
    # dimensionless number of physics steps.
    ratio = float(period) / float(timestep)

    # Extremely large ratios can overflow to infinity.
    # They cannot represent a usable finite substep count.
    if not math.isfinite(ratio):
        raise ValueError(
            "controller period / timestep is not finite"
        )

    rounded = round(ratio)

    # Require at least one complete physics integration.
    if rounded < 1:
        raise ValueError(
            "controller period is shorter than one "
            "physics timestep"
        )

    # Match the tolerance already used by config.py.
    # This permits ordinary floating-point representation
    # noise, but rejects materially noninteger ratios.
    if not math.isclose(
        ratio,
        rounded,
        rel_tol=1e-9,
        abs_tol=1e-9,
    ):
        raise ValueError(
            "controller period must be an integer "
            "multiple of the physics timestep"
        )

    return int(rounded)


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
        "_physics_step_count",
        "_controller_step_count",
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
        self._physics_step_count = 0
        self._controller_step_count = 0
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

    
    @property
    def simulation_time(self) -> float:
        """Return the authoritative MuJoCo simulation time.

        The result is expressed in simulated seconds, not
        elapsed wall-clock time.

        Available only while the simulator is LOADED.
        """
        instance = self._require_loaded()
        return float(instance.data.time)

    @property
    def physics_step_count(self) -> int:
        """Return successful physics integrations this episode.

        Each successfully completed mujoco.mj_step() counts
        once, irrespective of whether the integration was
        requested by step() or step_controller_period().

        Loading and reset forward computations do not count.
        """
        self._require_loaded()
        return self._physics_step_count

    @property
    def controller_step_count(self) -> int:
        """Return completed controller periods this episode.

        A controller period counts only when the entire
        underlying physics integration batch succeeds.

        Calls directly to step() do not increment this count.
        """
        self._require_loaded()
        return self._controller_step_count


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

        
        # Reset the physics step count for this simulation instance.
        self._physics_step_count = 0
        # Reset the controller step count for this simulation instance.
        self._controller_step_count = 0

    
    def step_controller_period(
        self, control: object = None, *, period: float,) -> None:
        """Advance exactly one nominal controller period.

        The requested controller duration is converted into
        a positive integer number of MuJoCo physics substeps.

        One actuator command is applied for the whole period.

        Parameters
        ----------
        control:
            Full raw actuator vector in compiled model order,
            or None to preserve the current actuator commands.

        period:
            Controller period in seconds. Must be finite,
            positive and an integer multiple of the current
            model physics timestep, allowing only minor
            floating-point representation differences.

        Notes
        -----
        This is a timing convenience layer, not a new physics
        integrator and not a controller policy.

        Existing Simulator.step() handles control validation,
        actuator writes and the individual mj_step calls.

        Application-level counters belong to P0-006.07.
        """
        instance = self._require_loaded()

        # Read the current compiled model timestep.
        # Do not assume the packaged default is always used.
        timestep = float(instance.model.opt.timestep)

        # Validate timing before delegating to the physics
        # stepping interface. An invalid period must cause
        # no actuator changes or physics integration.
        n_substeps = _controller_substep_count(
            period,
            timestep,
        )

        # Reuse the existing P0-006.05 path. This preserves
        # its actuator validation, control=None behaviour,
            # lifecycle guards and error semantics.
        self.step(
            control=control,
            n_substeps=n_substeps,
        )

        # Record the completion of a controller period.
        self._controller_step_count += 1

        return
    
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

        # Record each physics step.
        for _ in range(n_substeps):
            mujoco.mj_step(model, data)
            self._physics_step_count += 1


    def close(self) -> None:
        """Release wrapper references and permanently close."""
        if self._state is SimulatorState.CLOSED:
            return

        self._reset_snapshot = None
        self._instance = None
        self._state = SimulatorState.CLOSED
