
# P0-006.06 — Controller-Period Stepping

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Lifecycle API
**Status:** Proposed — awaiting approval and validation

## 1. Objective

Establish an explicit relationship between the controller
update period and MuJoCo's physics integration timestep.

A single controller action may require several physics
integrations before the next control update.

This ticket provides deterministic controller-period
stepping without introducing controller behaviour.

## 2. Existing Configuration

The simulation configuration defines:

    simulation.timestep = 0.002
    simulation.controller_period = 0.04

Both values are expressed in seconds.

The configuration loader already rejects controller
periods that are not integer multiples of the timestep.

P0-006.06 provides equivalent validation at the
Simulator execution boundary.

## 3. Physics and Controller Timing

Physics timestep:

    dt = model.opt.timestep

Requested controller period:

    T = controller period in seconds

Required number of physics steps:

    N = T / dt

N must be a positive integer within the documented
floating-point compatibility tolerance.

For the default configuration:

    dt = 0.002 seconds
    T  = 0.040 seconds
    N  = 20 physics steps

A controller period therefore comprises 20 calls to
mujoco.mj_step() at the default settings.

## 4. Public API

The existing physics interface is unchanged:

    Simulator.step(control=None, *, n_substeps=1)

The additional controller-period interface is:

    Simulator.step_controller_period(
        control=None,
        *,
        period=<seconds>,
    )

The period argument is mandatory.

The caller supplies the intended controller period,
typically from SimulationConfig.controller_period.

## 5. Execution Contract

For each successful controller-period call:

1. Require a loaded simulator.
2. Read the current model physics timestep.
3. Validate period and timestep.
4. Calculate the positive integer substep count.
5. Delegate to the existing Simulator.step().
6. Preserve the actuator command throughout the period.
7. Return with advanced physics state.

This API does not run a control policy.

It only applies a command for a defined interval.

## 6. Invalid Periods

The following are rejected:

- Zero or negative periods.
- NaN or infinite periods.
- Boolean period arguments.
- Strings or container arguments.
- Periods shorter than a physics timestep.
- Noninteger period-to-timestep ratios.
- Nonfinite or nonpositive physics timesteps.

Timing validation occurs before any physics integration.

Invalid actuator commands are rejected through the
existing P0-006.05 control validation.

## 7. Floating-Point Tolerance

Binary floating-point arithmetic may introduce small
representation differences.

Period ratios use:

    relative tolerance = 1e-9
    absolute tolerance = 1e-9

This is consistent with the existing configuration
loader's integer-multiple validation.

The tolerance does not authorize deliberate
fractional physics steps.

## 8. Control Semantics

A supplied control vector contains raw MuJoCo actuator
commands in compiled actuator order.

These values are not automatically joint positions.

The command is held constant through all physics
substeps within the requested controller period.

Passing control=None retains the existing data.ctrl.

The detailed actuator and movement interface remains
part of P0-007.

## 9. Time Advancement

For successful execution:

    elapsed simulation time ~= N * dt

The controller-period wrapper does not maintain
a separate clock.

MuJoCo's data.time remains authoritative.

Application-level step and controller counters are
deferred to P0-006.07.

## 10. Reset Compatibility

Calling Simulator.reset() after one or more controller
periods must restore the original simulation state.

Existing primary-state, transient-cleanup and
forward-kinematics guarantees continue to apply.

## 11. Behaviour Separation

P0-006.06 is not a behaviour-learning implementation.

Future controllers may use the following execution loop:

    Observe current robot state
        |
        v
    Select desired actuator command
        |
        v
    step_controller_period()
        |
        v
    Read updated simulation state
        |
        v
    Repeat

The behaviour or learning component remains independent
of this simulation timing interface.

## 12. Validation

Test file:

    tests/test_simulator_controller_period.py

Coverage includes:

- Compatible integer-multiple periods.
- Rejection of incompatible periods.
- Rejection of invalid numeric values and types.
- Floating-point compatibility tolerance.
- Exactly 20 steps for default controller timing.
- Exactly N underlying physics integrations.
- Constant actuator commands during a controller period.
- Preservation of existing controls when control=None.
- Use of the current model physics timestep.
- Rejection before mutation on invalid inputs.
- Multiple controller-period executions.
- Mixing raw physics steps with controller periods.
- Reset after controller-period execution.
- Model and data ownership preservation.
- Independent simulation instances.
- Lifecycle error handling.

## 13. Limitations

This milestone does not implement:

- Controller callbacks during integration.
- Policy inference or MAHQ behaviour learning.
- Inverse kinematics or trajectory generation.
- Automatic controller-period configuration binding.
- Independent controller and physics step counters.
- Real-time wall-clock scheduling.
- Collision avoidance or safe movement guarantees.
- Partial-period integration.

If future controller periods require noninteger
substep relationships, the timing policy must be
redesigned and independently validated.

## 14. Acceptance Criteria

P0-006.06 is complete when:

1. Compatible controller periods map to correct
   integer physics step counts.
2. Incompatible timing relationships are rejected.
3. Invalid inputs cause no physics time advancement.
4. Raw actuator controls retain P0-006.05 semantics.
5. No duplicated actuator-validation implementation
   is introduced.
6. Reset and lifecycle behaviour remain correct.
7. New integration tests pass.
8. The full regression suite passes.
9. Validation evidence is recorded in STATUS.md.

## 15. Change Control

All code changes require explicit approval under AGENTS.md.

Test execution requires separate approval.

Do not modify:

- Robot or scene assets.
- Model ownership code.
- Existing physics stepping behaviour.
- scripts/run_tests.py.
