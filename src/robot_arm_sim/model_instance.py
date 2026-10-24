
"""P0-005.02 — Independent MuJoCo model and state ownership.

An independent simulation owns one compiled MjModel and its matching
MjData. Supported factories allocate both objects afresh.

This module does not provide stepping, reset, rendering or close().
Those responsibilities belong to P0-006.

Ownership is a cooperative application invariant, not a Python
security boundary. Exposed MuJoCo objects remain mutable.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, SupportsIndex, final

import mujoco

from .model_loader import load_model
from .simulation import build_scene


@final
class SimulationInstance:
    """Retain one MuJoCo model and its corresponding simulation state.

    Supported construction:
        from_model_file(path)
        from_robot_scene(panda_path)

    Invariants:
        - Factories create distinct model/state pairs.
        - Model and data references cannot be ordinarily reassigned.
        - No second owning wrapper is created by copying.
        - The pair remains referenced while the wrapper is alive.

    Limitations:
        The underlying MuJoCo objects are mutable.
        Python reflection and deliberate reference sharing are not
        prevented. This class is not a thread-safety mechanism.
    """

    # Avoid an instance dictionary and ordinary arbitrary attributes.
    # Slots do not make the underlying MuJoCo objects immutable.
    __slots__ = ("_model", "_data", "_sealed")

    def __init__(self) -> None:
        """Reject construction without a supported factory."""
        raise TypeError(
            "SimulationInstance must be created using "
            "from_model_file() or from_robot_scene()"
        )

    @property
    def model(self) -> mujoco.MjModel:
        """Return the compiled model owned by this instance."""
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        """Return the mutable state associated with this model."""
        return self._data

    def __setattr__(self, name: str, value: object) -> None:
        """Reject ordinary mutation of the ownership wrapper."""
        raise AttributeError(
            "SimulationInstance ownership references are read-only"
        )

    def __delattr__(self, name: str) -> None:
        """Reject ordinary deletion of ownership references."""
        raise AttributeError(
            "SimulationInstance ownership references cannot be deleted"
        )

    @classmethod
    def _create(
        cls,
        model: mujoco.MjModel,
        data: mujoco.MjData,
    ) -> SimulationInstance:
        """Create a wrapper for a pair returned by a trusted factory.

        This private method is only for internal factory use.
        It must never be used as a public adoption API.

        The supplied pair is expected to have been allocated together.
        Python MuJoCo does not expose a general ownership certificate
        proving that an arbitrary MjData belongs to an MjModel.
        """
        if cls is not SimulationInstance:
            raise TypeError(
                "SimulationInstance subclass construction is unsupported"
            )

        if not isinstance(model, mujoco.MjModel):
            raise TypeError("Expected mujoco.MjModel")

        if not isinstance(data, mujoco.MjData):
            raise TypeError("Expected mujoco.MjData")

        # Bypass the intentionally disabled public constructor.
        # No external user code runs between these assignments.
        instance = object.__new__(cls)
        object.__setattr__(instance, "_model", model)
        object.__setattr__(instance, "_data", data)
        object.__setattr__(instance, "_sealed", True)

        return instance

    @classmethod
    def from_model_file(
        cls,
        path: str | Path,
    ) -> SimulationInstance:
        """Create a standalone simulation from an MJCF file.

        The P0-005.01 loader performs path validation, compilation
        and matching MjData allocation.

        No physics step is executed here.
        Loader exceptions propagate to the caller.
        """
        if cls is not SimulationInstance:
            raise TypeError("Subclass factories are unsupported")

        model, data = load_model(path)
        return cls._create(model, data)

    @classmethod
    def from_robot_scene(
        cls,
        panda_path: str | Path,
    ) -> SimulationInstance:
        """Create an independent composed Franka Panda scene.

        Preserve the scene builder's existing home initialisation,
        collision checks and placement validation.

        The scene builder may call mj_forward() to calculate derived
        state. This factory performs no physics stepping.
        """
        if cls is not SimulationInstance:
            raise TypeError("Subclass factories are unsupported")

        model, data = build_scene(panda_path)
        return cls._create(model, data)

    def __copy__(self) -> SimulationInstance:
        """Prevent shallow-copy aliases to the same MuJoCo state."""
        raise TypeError(
            "SimulationInstance cannot be copied; "
            "create a fresh instance through a factory"
        )

    def __deepcopy__(self, memo: dict) -> SimulationInstance:
        """Prevent implicit cloning with undefined state semantics."""
        raise TypeError(
            "SimulationInstance cannot be deep-copied; "
            "create a fresh instance through a factory"
        )

    def __reduce_ex__(self, protocol: int) -> object:
        """Prevent serialisation that bypasses factory allocation."""
        raise TypeError(
            "SimulationInstance cannot be pickled"
        )
