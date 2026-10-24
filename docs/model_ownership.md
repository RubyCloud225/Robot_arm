# P0-005.02 — Model and Simulation State Ownership

**Project:** Robot Arm Pick-and-Place Simulator  
**Milestone:** P0 — Simulation Foundation  
**Status:** Proposed — pending implementation and validation

## 1. Objective

Ensure that each independently controlled MuJoCo simulation has its own compiled model and associated simulation state.

No independent simulation may inadvertently modify another simulation's joint positions, actuator commands, velocities, simulation time or mutable model configuration.

## 2. Ownership Model

A `SimulationInstance` holds:

- One `mujoco.MjModel`.
- One corresponding `mujoco.MjData`.

The instance retains strong references to both objects for its lifetime.

The model describes the compiled physical and structural properties of the simulation.

The state contains the evolving numerical information used by MuJoCo, including joint coordinates, velocities, actuator controls and simulation time.

Both objects are allocated afresh when a new instance is created through a supported factory method.

## 3. Construction

### Standalone model

`SimulationInstance.from_model_file(path)`

This uses the P0-005.01 model loader.

Each invocation loads a new compiled model and creates a corresponding state.

### Composed Panda scene

`SimulationInstance.from_robot_scene(panda_path)`

This delegates to the existing `build_scene()` function.

The existing scene composition, home initialization and placement validations remain in effect.

No production simulation scripts are replaced as part of P0-005.02.

## 4. State Isolation

Changes to one instance's:

- `qpos`
- `qvel`
- `ctrl`
- simulation time
- mutable compiled model configuration

must not affect another independently created instance.

The implementation must demonstrate isolation through automated tests.

## 5. Lifetime and Memory Management

`SimulationInstance` retains references to its model and data.

The objects remain available while the instance is alive and are normally released when no references remain.

Python reference management and MuJoCo's native bindings are responsible for memory ownership.

This milestone does not implement explicit lifecycle or resource-shutdown methods.

Those belong to P0-006.

## 6. Concurrency

Independent model/state pairs provide isolation of mutable simulation data.

They do not, by themselves, guarantee thread safety for all possible parallel execution patterns.

A single instance should not be mutated concurrently without an explicitly designed synchronisation mechanism.

Parallel simulation execution must be validated separately before being treated as supported.

## 7. Tests

The test suite must demonstrate:

1. Correct MuJoCo object types.
2. Distinct model identities.
3. Distinct state identities.
4. Equivalent initial conditions.
5. Independent joint positions.
6. Independent joint velocities.
7. Independent actuator commands.
8. Independent model parameter changes.
9. Independent physics progression.
10. Successful repeated creation of multiple instances.
11. Valid object lifetime during garbage collection.
12. Rejection of instance attribute reassignment.
13. Propagation of model-loading failures.
14. No modification of source model files.
15. Optional integration with the actual Franka Panda scene.

All required tests must operate without a graphical interface.

## 8. Limitations

P0-005.02 does not provide:

- Full simulation lifecycle management.
- Automatic reset.
- Thread-safe simultaneous mutation of a shared instance.
- Guaranteed reproducibility across operating systems.
- Joint or actuator name validation.
- Cartesian control or inverse kinematics.
- Successful grasping or reachability.
- Automatic cleanup of external resources.
- Protection against callers deliberately sharing MuJoCo references.

These responsibilities remain separate milestones.

## 9. Acceptance Criteria

P0-005.02 is complete when:

- Each supported factory produces a fresh model/state pair.
- State isolation is demonstrated by automated tests.
- Model parameter isolation is demonstrated.
- One instance can step without advancing another.
- Existing model-loading and scene-composition behaviour is preserved.
- Tests pass on a documented supported environment.
- Test results and commit references are recorded in `STATUS.md`.

## 10. Change Control

All implementation and test execution require the repository owner's explicit approval under `AGENTS.md`.

No existing source files or scripts may be modified without additional authorisation.