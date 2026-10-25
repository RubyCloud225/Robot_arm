
# P0-006.08 — Simulator Lifecycle and Input Guards

**Project:** Robot Arm Pick-and-Place Simulator
**Status:** Proposed — validation pending

## Objective

Ensure invalid operations are rejected before physical
simulation begins or actuator commands are modified.

This ticket consolidates lifecycle, action-shape,
substep-count and controller-period validation.

## Lifecycle Contract

Supported lifecycle:

    NEW -> LOADED -> CLOSED

The simulator cannot return from CLOSED to LOADED.

Before load(), the following operations are rejected:

- reset()
- step()
- step_controller_period()
- model/data access
- execution counter access

Following close(), those operations remain unavailable.

Repeated close() calls are allowed.

Repeated load() calls are rejected.

## Physics Step Validation

Simulator.step() requires:

- A loaded simulation instance.
- A positive Python integer n_substeps.
- A finite, positive physics timestep.
- A valid actuator command.

Boolean, floating-point, string and NumPy integer
substep counts are rejected by the existing interface.

No implicit substep conversion is performed.

## Actuator Control Validation

Controls must have exactly model.nu entries.

They must be one-dimensional and contain real numeric
values representable as finite float64 numbers.

For actuators with enabled control limits, the value
must lie within the configured control range.

Actuator values are not assumed to be joint angles.

Invalid inputs are rejected rather than silently
clamped to a permitted range.

## Existing Commands

Passing control=None preserves the current values
in data.ctrl.

However, those existing commands must pass the same
finite-value and actuator-range checks as newly
supplied commands.

This guards against invalid direct modifications
to the exposed MuJoCo data buffers.

Validation of existing commands does not itself
write new actuator values.

## Controller-Period Validation

step_controller_period() inherits the actuator
validation in Simulator.step().

Its period must map to a positive integer number
of physics integrations under P0-006.06.

Invalid controller periods are rejected before
physics stepping begins.

## Failure Guarantees

For an ordinary input-validation failure:

- No MuJoCo integration is performed.
- No new actuator command is applied.
- Simulation time remains unchanged.
- Physics step count remains unchanged.
- Controller step count remains unchanged.

If the current actuator buffer was already corrupted
by external code, rejection does not automatically
repair that buffer.

Unexpected exceptions occurring after physics
integration begins do not guarantee a rollback.

## Tests

Primary test file:

    tests/test_simulator_guards.py

Existing regression coverage:

    tests/test_simulator_lifecycle.py
    tests/test_simulator_stepping.py
    tests/test_simulator_controller_period.py
    tests/test_simulator_counters.py

Required checks include lifecycle rejection, invalid
action shapes and dtypes, invalid substeps, nonfinite
controls, controller validation, unchanged-state
guarantees and counter preservation.

## Limitations

Input validation is not equivalent to physical safety.

The current interface does not guarantee:

- Collision-free motion.
- Reachable Cartesian targets.
- Actuator torque safety over a trajectory.
- Safe learned behaviour.
- Protection against arbitrary direct MuJoCo mutation.
- Transactional rollback after an engine exception.

These remain separate concerns in later milestones.

## Acceptance Criteria

P0-006.08 is complete when:

1. Lifecycle guards reject invalid operations.
2. Illegal substep counts are rejected.
3. Invalid action dimensions and types are rejected.
4. Nonfinite or out-of-range controls are rejected.
5. Existing controls are checked before reuse.
6. Validation failures preserve simulation state
   and execution counters.
7. Controller-period execution inherits the guards.
8. Existing tests and new regression tests pass.
9. Validation evidence is recorded in P0_CHECKLIST.md.

## Change Control

All changes require explicit approval under AGENTS.md.

No changes are authorised to scripts/run_tests.py,
robot model assets, timing configuration or scene scripts.

Testing and compilation require separate approval.
