
"""P0-005.02 — Ownership of independent MuJoCo simulation instances.

Each instance keeps strong references to its own compiled model and
corresponding state. Creation must allocate a fresh model/state pair.

This module does not implement stepping, reset, rendering or cleanup.
Those lifecycle behaviours belong to P0-006.
"""

from dataclasses import dataclass
from pathlib import Path

import mujoco

from .model_loader import load_model
from .simulation import build_scene


@dataclass(frozen=True, slots=True, eq=False)
class SimulationInstance:
    """Own a compiled MuJoCo model and its associated state.

    Attributes:
        model: Compiled model belonging to this simulation instance.
        data: Mutable simulation state allocated for this model.

    Notes:
        The frozen dataclass prevents reassignment of the model/data
        attributes. It does NOT make the underlying MuJoCo objects
        immutable or thread-safe.

        Do not share model or data references across independently
        controlled simulations.
    """

    model: mujoco.MjModel
    data: mujoco.MjData

    @classmethod
    def from_model_file(
        cls, path: str | Path
    ) -> "SimulationInstance":
        """Create an instance from a standalone MJCF file.

        Delegates path/error handling to the P0-005.01 loader.
        Each invocation must produce a newly allocated model and state.
        """
        model, data = load_model(path)
        return cls(model=model, data=data)

    @classmethod
    def from_robot_scene(
        cls, panda_path: str | Path
    ) -> "SimulationInstance":
        """Create an instance of the existing Panda task scene.

        Preserve build_scene's home-keyframe initialization,
        scene composition and collision/placement validation.
        """
        model, data = build_scene(panda_path)
        return cls(model=model, data=data)
