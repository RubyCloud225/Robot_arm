
"""P0-006.01 — Explicit simulation lifecycle ownership.

The Simulator wraps one independently allocated Panda scene.

This initial milestone implements loading and closure.
State reset, physics stepping and timing are intentionally
reserved for the remaining P0-006 tickets.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path

import mujoco

from .model_instance import SimulationInstance


class SimulatorState(str, Enum):
    """Public lifecycle states."""

    NEW = "NEW"
    LOADED = "LOADED"
    CLOSED = "CLOSED"


class SimulatorLifecycleError(RuntimeError):
    """An operation is invalid in the current lifecycle state."""


class Simulator:
    """Own the lifecycle of one Panda simulation instance.

    Args:
        panda_path: Path to the locally available Panda MJCF.

    Lifecycle:
        NEW -> load() -> LOADED -> close() -> CLOSED

    An instance cannot be loaded twice or reopened after closure.
    Create a new Simulator for another independent simulation.

    No viewer or graphical context is created by this class.
    """

    __slots__ = ("_panda_path", "_instance", "_state")

    def __init__(self, panda_path: str | Path) -> None:
        """Record the model path without compiling or loading."""
        if not isinstance(panda_path, (str, Path)):
            raise TypeError("panda_path must be str or Path")

        if not str(panda_path).strip():
            raise ValueError("panda_path cannot be blank")

        self._panda_path = Path(panda_path)
        self._instance: SimulationInstance | None = None
        self._state = SimulatorState.NEW

    @property
    def state(self) -> SimulatorState:
        """Return the current lifecycle state."""
        return self._state

    @property
    def is_loaded(self) -> bool:
        """Whether this simulator currently owns a model/data pair."""
        return self._state is SimulatorState.LOADED

    def _require_loaded(self) -> SimulationInstance:
        """Reject access before load or after closure."""
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
        """Access the currently loaded compiled model."""
        return self._require_loaded().model

    @property
    def data(self) -> mujoco.MjData:
        """Access the currently loaded mutable simulation state.

        This access remains available for development/debugging.
        Callers must not share or misuse the mutable state.
        """
        return self._require_loaded().data

    def load(self) -> None:
        """Build and own one independent composed Panda scene.

        A failed initial load leaves the wrapper in NEW.
        No partially allocated instance becomes accessible.
        """
        if self._state is not SimulatorState.NEW:
            raise SimulatorLifecycleError(
                "load() is allowed only from NEW; "
                f"current state is {self._state.value}"
            )

        # Reuse the existing P0-005.02 ownership factory.
        # The factory must successfully return before the
        # wrapper publishes its loaded state.
        instance = SimulationInstance.from_robot_scene(
            self._panda_path
        )

        self._instance = instance
        self._state = SimulatorState.LOADED

    def reset(self, seed: int | None = None) -> None:
        """Reserved for P0-006.02 through P0-006.04."""
        self._require_loaded()
        raise NotImplementedError(
            "reset() is reserved for P0-006.02–006.04"
        )

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
        """Release this wrapper's references to model and data.

        Repeated closure is harmless. Closure is permanent.

        External references to MuJoCo objects, if retained by
        callers, may continue to exist. This method does not
        destroy objects still referenced elsewhere.
        """
        if self._state is SimulatorState.CLOSED:
            return

        self._instance = None
        self._state = SimulatorState.CLOSED
