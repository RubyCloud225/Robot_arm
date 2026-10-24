
# P0-006.05 — Physics Stepping Semantics

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Lifecycle API
**Status:** Proposed — pending test validation

## Objective

Define and implement the meaning of Simulator.step().

The simulator advances the actual MuJoCo physical
simulation without introducing movement planning,
inverse kinematics or learned behaviour.

## Public Interface

    Simulator.step(control=None, *, n_substeps=1)

## Physics Step Contract

One substep means exactly one mujoco.mj_step() call.

The simulation's physics timestep comes from:

    model.opt.timestep

A call containing N substeps advances N physics
integration steps, assuming every call succeeds.

Examples with the current default dt = 0.002 seconds:

    n_substeps=1  -> 0.002 seconds
    n_substeps=5  -> 0.010 seconds
    n_substeps=20 -> 0.040 seconds

The nominal controller period is 0.040 seconds,
but controller-period scheduling is not introduced
until P0-006.06.

## Control Semantics

control=None:
    Preserve the current values in data.ctrl.

control=vector:
    Apply a complete MuJoCo actuator command vector
    ordered by compiled actuator index.

The vector length must equal model.nu.

These inputs are raw actuator controls and must not
be interpreted automatically as joint angles,
joint velocities or Cartesian coordinates.

The Panda gripper's actuator command has its own
documented units and range.

## Control Validation

For supplied control vectors:

- Require one-dimensional shape.
- Require model.nu elements.
- Require real numeric values.
- Reject boolean and string vectors.
- Reject NaN and infinity.
- Validate finite enabled actuator control ranges.
- Reject values outside enabled limits.
- Do not silently clamp commands.
- Do not partially update data.ctrl on validation failure.

The detailed named actuator interface belongs to P0-007.

## Substep Validation

n_substeps must be a positive Python integer.

Booleans, strings, floats, zero and negative counts
are rejected.

The compiled physics timestep must remain finite
and strictly positive.

All ordinary validation must occur before integration.

## Execution

After validation:

1. Apply the validated command, if supplied.
2. Call mujoco.mj_step() exactly n_substeps times.
3. Retain the same control values for each substep.
4. Return with the updated MuJoCo state and time.

Simulator.step() does not create a viewer or render.

## Errors and Partial Progress

Invalid inputs are rejected before stepping.

If MuJoCo raises an unexpected exception during the
integration loop, earlier successful substeps are
not automatically rolled back.

The resulting model/data state must not be described
as transactionally restored.

## Reset Compatibility

A subsequent Simulator.reset() restores the initial
robot, ball, gripper, target and simulation clock.

It also clears transient state and recomputes
forward kinematics under P0-006.02–006.04.

## Validation

Test file:

    tests/test_simulator_stepping.py

Required checks:

- Single-step time advancement.
- Multiple substep durations.
- Repeated step calls.
- Configured timestep use.
- Ball gravitational motion.
- Existing-control preservation.
- Valid actuator command application.
- Gripper actuator semantics.
- Invalid control shape and type rejection.
- NaN and infinity rejection.
- Actuator control bounds.
- Invalid substep count/type rejection.
- No partial control writes on validation failure.
- Exactly N MuJoCo stepping calls.
- Constant command throughout a step batch.
- Reset after stepping.
- Multi-instance isolation.
- Lifecycle guards.
- Headless execution.

## Limitations

P0-006.05 does not establish:

- Collision-safe trajectories.
- Correct actuator-to-joint movement response.
- A stable hold controller.
- Gripper grasp success.
- End-effector reachability.
- Long-duration numerical stability.
- Controller-period scheduling.
- Learned robotic behaviour.

These concerns are covered by subsequent tickets.

## Acceptance Criteria

The ticket is complete when:

1. One substep produces one MuJoCo physics step.
2. N substeps produce N physics steps.
3. Elapsed time matches the configured timestep.
4. Invalid commands cannot initiate stepping.
5. Valid commands are held throughout execution.
6. Independent simulations remain isolated.
7. Reset continues to work after stepping.
8. New tests and the full regression suite pass.
9. Results are recorded in STATUS.md.

## Change Control

All code changes require owner approval under AGENTS.md.

Test execution requires separate approval.

No model assets, scene composition code, model
ownership files or scripts/run_tests.py may be
modified without authorisation.
