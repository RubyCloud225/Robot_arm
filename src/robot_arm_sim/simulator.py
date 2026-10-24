
"""P0-006.02 — Simulator lifecycle and deterministic state reset.

Simulator owns one independently compiled Panda scene.

P0-006.02 captures a baseline immediately after loading and
restores primary MuJoCo state without replacing MjModel/MjData.

Transient solver cleanup, forward-kinematic refresh and
physics stepping are reserved for subsequent P0-006 tickets.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import mujoco
import numpy as np

from .model_instance import SimulationInstance


class SimulatorState(str, Enum):
    """Public lifecycle states."""

    NEW = "NEW"
    LOADED = "LOADED"
    CLOSED = "CLOSED"


class SimulatorLifecycleError(RuntimeError):
    """An operation is invalid in the current lifecycle state."""


@dataclass(frozen=True, slots=True, eq=False)
class _ResetSnapshot:
    """Private baseline for restoring primary simulation state.

    Each array is copied from the loaded model or data.
    References to the live MuJoCo arrays are never stored.

    Arrays are made read-only to reduce the risk of accidental
    corruption of the stored reset baseline.
    """

    qpos: np.ndarray
    qvel: np.ndarray
    ctrl: np.ndarray
    target_position: np.ndarray
    time: float


def _readonly_copy(values: np.ndarray) -> np.ndarray:
    """Capture an independent, read-only NumPy array."""
    snapshot = np.array(values, copy=True)
    snapshot.setflags(write=False)
    return snapshot


def _capture_initial_state(
    instance: SimulationInstance,
) -> _ResetSnapshot:
    """Capture the exact initial state produced by build_scene().

    The target is represented by a fixed MuJoCo body. Unlike
    the free ball, its placement lives in model.body_pos,
    rather than in data.qpos.

    All qpos/qvel coordinates are captured as full arrays to
    preserve the Panda's actual home pose and the ball's
    translation and quaternion without manual reconstruction.
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

    return _ResetSnapshot(
        qpos=_readonly_copy(data.qpos),
        qvel=_readonly_copy(data.qvel),
        ctrl=_readonly_copy(data.ctrl),
        target_position=_readonly_copy(
            model.body_pos[target_id]
        ),
        time=float(data.time),
    )


class Simulator:
    """Own one Panda model/data pair and its reset baseline.

    The simulator begins in NEW. load() allocates the scene,
    captures its initial primary state, and enters LOADED.

    reset() restores those primary values without replacing
    MjModel or MjData.

    close() permanently transitions to CLOSED.
    """

    __slots__ = (
        "_panda_path",
        "_instance",
        "_state",
        "_reset_snapshot",
    )

    def __init__(self, panda_path: str | Path) -> None:
        """Record a model path without loading a model."""
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
        """Whether this wrapper owns a loaded simulation."""
        return self._state is SimulatorState.LOADED

    def _require_loaded(self) -> SimulationInstance:
        """Reject model/state access outside LOADED."""
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
        """Return the currently loaded compiled model."""
        return self._require_loaded().model

    @property
    def data(self) -> mujoco.MjData:
        """Return the live, mutable MuJoCo simulation state."""
        return self._require_loaded().data

    def load(self) -> None:
        """Build the scene and capture a per-instance reset state.

        All work must succeed before entering LOADED.

        If building the scene or capturing the baseline fails,
        the simulator stays in NEW without publishing a partial
        model/data pair or an incomplete reset snapshot.
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

        # Publish both objects only once initialization succeeds.
        self._instance = instance
        self._reset_snapshot = snapshot
        self._state = SimulatorState.LOADED

    def reset(self, seed: int | None = None) -> None:
        """Restore the original robot and task primary state.

        Reuses the same model/data objects. Restores:
        - All robot, finger and ball position coordinates.
        - All robot, finger and ball velocity coordinates.
        - All actuator control entries.
        - The target's fixed body position.
        - Initial simulation time.

        A non-None seed is rejected for now because seeded
        random reset behaviour has not been implemented.
        Silently accepting and ignoring a seed would create
        misleading reproducibility guarantees.

        Does not clear applied forces or solver transients
        (006.03), recompute kinematics (006.04), or step physics.
        """
        instance = self._require_loaded()

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

        # Check array dimensions before performing any writes.
        # This catches misuse of a baseline with incompatible
        # state buffers rather than silently truncating data.
        if (
            data.qpos.shape != snapshot.qpos.shape
            or data.qvel.shape != snapshot.qvel.shape
            or data.ctrl.shape != snapshot.ctrl.shape
            or model.body_pos[target_id].shape
            != snapshot.target_position.shape
        ):
            raise SimulatorLifecycleError(
                "Cannot reset: current state dimensions "
                "do not match the initial snapshot"
            )

        # Copy the actual captured values. Do not regenerate
        # joint states from IDs or convert the ball quaternion.
        np.copyto(data.qpos, snapshot.qpos)
        np.copyto(data.qvel, snapshot.qvel)
        np.copyto(data.ctrl, snapshot.ctrl)

        # Restore the fixed goal body. It has no qpos entry.
        np.copyto(
            model.body_pos[target_id],
            snapshot.target_position,
        )

        # Restore the original simulation clock.
        data.time = snapshot.time

        # No mj_forward(), mj_resetData() or mj_step() here.
        # Those semantics are handled explicitly in later
        # lifecycle tickets.

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
        """Release wrapper-owned references permanently.

        External MuJoCo references retained by callers may
        remain alive. Repeated closure is harmless.
        """
        if self._state is SimulatorState.CLOSED:
            return

        self._reset_snapshot = None
        self._instance = None
        self._state = SimulatorState.CLOSED
