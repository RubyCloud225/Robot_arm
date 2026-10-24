
# P0-006 — Simulation Lifecycle API

## P0-006.01 — Simulator Wrapper

**Status:** Proposed — awaiting implementation approval.

## Objective

Introduce a small, explicit lifecycle API for the
Franka Panda pick-and-place simulation.

The wrapper must reuse the existing P0-005 model
ownership layer rather than creating a competing
model-loading or state-management implementation.

## Lifecycle

NEW -> LOADED -> CLOSED

### NEW

- No model or simulation state has been allocated.
- Model/data access is prohibited.
- load() is permitted.
- close() is permitted.

### LOADED

- A fresh MjModel and MjData are owned by the wrapper.
- Model/data inspection is permitted.
- A second load() call is prohibited.
- close() is permitted.
- reset() and step() are reserved for later tickets.

### CLOSED

- The wrapper holds no simulation instance.
- Further loading is prohibited.
- Model/data access is prohibited.
- Repeated close() calls are harmless.
- A new Simulator must be created for another run.

## Model Ownership

Simulator delegates initial model creation to:

    SimulationInstance.from_robot_scene(path)

This preserves existing scene composition, home
initialisation and placement checks.

Each Simulator owns its own model/data pair.

Closing clears the wrapper's reference to that pair.
It does not invalidate model/data references retained
by other Python objects.

## Public Interface

    Simulator(panda_path)
    Simulator.load()
    Simulator.reset(seed=None)
    Simulator.step(control=None, n_substeps=1)
    Simulator.close()

Properties:

    state
    is_loaded
    model
    data

The reset and step methods are unimplemented in 006.01.
They raise NotImplementedError in the LOADED state.

## Error Handling

Invalid constructor input raises TypeError or ValueError.

Invalid lifecycle operations raise
SimulatorLifecycleError.

Model-loading errors propagate from the existing
scene builder.

Failed loading must not publish a partial instance.

## GUI Independence

Simulator must not:

- Create a viewer.
- Open a window.
- Initialise a rendering context.
- Require a display server.

It must remain usable in headless tests.

## Testing

Tests are located in:

    tests/test_simulator_lifecycle.py

Required coverage:

- Initial NEW state.
- Explicit successful loading.
- Model/data ownership.
- Multiple independent simulators.
- Duplicate-load rejection.
- Failed-load state preservation.
- Closure and idempotent close.
- Access rejection before loading.
- Access rejection after closure.
- No reload after closure.
- No unexpected physics stepping.
- External-reference lifetime behaviour.
- Independent operation after another instance closes.

## Deferred Behaviour

P0-006.02:
    Complete reset-state restoration.

P0-006.03:
    Transient forces, controls and solver-state cleanup.

P0-006.04:
    Derived kinematic/contact recomputation.

P0-006.05:
    Step semantics and physics advancement.

P0-006.06:
    Controller-period timing.

P0-006.07:
    Simulation time and counter tracking.

P0-006.08:
    Full parameter and action validation.

P0-006.09:
    Explicit headless integration tests.

## Change Control

Implementation requires explicit approval under AGENTS.md.
Running or compiling tests requires separate approval.

No existing simulation scripts, model assets or test-runner
scripts may be changed without approval.
