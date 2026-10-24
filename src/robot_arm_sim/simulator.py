
"""P0-006.03 — Simulator lifecycle and transient-state reset.

Own an independently compiled Panda simulation and restore its
initial physical state without recompilation or physics stepping.

The reset procedure uses MuJoCo's native reset operation to clear
transient simulation data, then restores the captured task baseline.

Forward kinematics/contact recomputation belongs to P0-006.04.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

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
        """Restore the complete supported MuJoCo reset state.

        Steps:
        1. Validate the lifecycle and baseline.
        2. Reset MuJoCo-managed data, including transient buffers.
        3. Restore captured positions, velocities, commands,
           actuator activations and clock.
        4. Restore the fixed target body's initial placement.

        Deliberately does not call mj_forward or mj_step.
        """
        instance = self._require_loaded()

        # Seed-driven random initial conditions are deferred to
        # P0-010. Reject rather than silently ignore the request.
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

        # Validate all dimensions before any reset writes.
        # In particular, actuator activation dimensions (na)
        # are not necessarily the same as actuator controls (nu).
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

        # Native MuJoCo reset clears more than qpos/qvel:
        # externally applied forces, solver warm-start values
        # and other engine-managed transient state are reset.
        #
        # This is performed BEFORE restoring the composed task
        # baseline, because the task's valid initial controls
        # and joint pose need not match generic defaults.
        mujoco.mj_resetData(model, data)

        # Restore the precise state captured after build_scene.
        # Full qpos copying preserves the free-ball quaternion.
        np.copyto(data.qpos, snapshot.qpos)
        np.copyto(data.qvel, snapshot.qvel)
        np.copyto(data.ctrl, snapshot.ctrl)
        np.copyto(data.act, snapshot.act)

        # The goal marker is a fixed body in the compiled model.
        np.copyto(
            model.body_pos[target_id],
            snapshot.target_position,
        )

        data.time = snapshot.time

        # P0-006.04 will refresh derived transforms and contacts.
        # There must be no physics advance in this ticket.

    def step(
        self,
        control: object = None,
        *,
        n_substeps: int = 1,
    ) -> None:
        """Reserved for P0-006.05 through P0-006.08."""
        self._require_loaded()
        raise NotImplementedError(
            "step() is reserved for P0-006.05–006.08"
        )

    def close(self) -> None:
        """Release wrapper references and permanently close."""
        if self._state is SimulatorState.CLOSED:
            return

        self._reset_snapshot = None
        self._instance = None
        self._state = SimulatorState.CLOSED
