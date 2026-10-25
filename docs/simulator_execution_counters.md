
# P0-006.07 — Simulation Time and Execution Counters

**Project:** Robot Arm Pick-and-Place Simulator  
**Milestone:** P0 — Simulation Lifecycle API  
**Ticket:** P0-006.07  
**Status:** Implementation proposed — validation pending

## 1. Objective

Track simulation time, physics integrations and controller
execution independently.

These measurements establish a consistent execution record
for future robotic movement, trajectory analysis, control
algorithms and MAHQ behaviour-learning experiments.

## 2. Public Interface

The Simulator class exposes three read-only properties.

### 2.1 simulation_time

Type: float  
Units: simulated seconds  
Source: MjData.time

Returns the current MuJoCo simulation clock.

This represents simulated physical time, not elapsed
wall-clock time.

### 2.2 physics_step_count

Type: int  
Units: completed physics integrations

Counts successful `mujoco.mj_step()` calls performed
through the Simulator execution interface.

Each completed physics integration increments this
counter exactly once.

### 2.3 controller_step_count

Type: int  
Units: completed controller periods

Counts successful calls to
`Simulator.step_controller_period()`.

A controller period is counted only when all its physics
substeps have completed successfully.

Raw calls to `Simulator.step()` do not increment the
controller counter.

## 3. Timing Relationship

The default simulation configuration defines:

- Physics timestep: 0.002 seconds.
- Controller period: 0.040 seconds.
- Physics steps per controller period: 20.

The relationship is:

    physics_substeps = controller_period / physics_timestep

The controller period must map to a positive integer
number of physics steps, as validated in P0-006.06.

The current model timestep is used rather than a
hardcoded default.

## 4. Execution Accounting

### 4.1 Raw Physics Steps

Calling:

    sim.step(n_substeps=5)

produces, on success:

    physics_step_count += 5
    controller_step_count += 0

### 4.2 Controller-Period Steps

Calling:

    sim.step_controller_period(period=0.04)

at the default physics timestep produces:

    physics_step_count += 20
    controller_step_count += 1

The controller-period method delegates to
`Simulator.step()` to execute physics integrations.

The physics count must not be incremented a second
time by the controller-period wrapper.

### 4.3 Mixed Execution

Example:

    sim.step(n_substeps=5)
    sim.step_controller_period(period=0.04)
    sim.step_controller_period(period=0.04)

Expected results:

    physics_step_count = 45
    controller_step_count = 2
    simulation_time = 0.090 seconds

These values assume the initial simulation time was zero.

## 5. Lifecycle Behaviour

### 5.1 Construction

The internal counters are initialized to zero.

### 5.2 Load

After a successful load, both counters are zero.

Loading the scene must not count as physics integration
or controller execution.

### 5.3 Reset

Following a successful reset:

    physics_step_count = 0
    controller_step_count = 0

Simulation time is restored using the existing reset
snapshot mechanism.

The forward-kinematics recomputation performed during
reset is not counted as a physics step.

The counters must only be cleared after all reset
operations complete successfully.

### 5.4 Close

Closing the simulator releases episode accounting.

Attempting to access public counter properties after
closure raises SimulatorLifecycleError.

Repeated close operations remain safe.

## 6. Failure Behaviour

### 6.1 Invalid Inputs

Invalid substep counts, controller periods and actuator
commands must not increment either counter.

Validation occurs before physical execution.

### 6.2 Partial Physics Failure

Each physics step is counted after its corresponding
`mujoco.mj_step()` returns successfully.

If an exception occurs partway through an integration
batch, earlier completed integrations remain counted.

For example, if a five-step request fails before
the third integration:

    physics_step_count += 2

The failed integration is not counted.

### 6.3 Incomplete Controller Period

A controller period is counted only after its entire
physics integration batch completes.

If three integrations succeed before a fourth raises:

    physics_step_count += 3
    controller_step_count += 0

An earlier successfully completed controller period
must retain its recorded count.

### 6.4 Failed Reset

A failed reset must not clear the counters.

This is an accounting guarantee, not a guarantee
that MuJoCo state is transactionally rolled back
after an internal engine failure.

## 7. Simulation Time

`simulation_time` reads directly from `data.time`.

It must not be reconstructed from:

    physics_step_count * current_timestep

The timestep may change between execution calls.

Simulation time also may be changed directly by callers
with low-level access to MuJoCo data.

## 8. Independent Simulation Instances

Each Simulator instance owns its own counters.

Stepping or resetting one simulation must not alter
the accounting of another simulation.

No global counters are permitted.

## 9. Read-Only Properties

The public properties cannot be assigned through
ordinary property assignment.

They remain accessible only in the LOADED lifecycle state.

## 10. External MuJoCo Operations

The Simulator exposes MjModel and MjData for testing
and diagnostic purposes.

A caller could directly invoke:

    mujoco.mj_step(sim.model, sim.data)

Such operations are not included in
`physics_step_count`.

The counters therefore describe execution through
the Simulator API rather than every possible external
modification to the MuJoCo state.

## 11. Verification

Test file:

    tests/test_simulator_counters.py

Required validation:

- Zero initial counters.
- Correct single-step accounting.
- Multiple physics substep accounting.
- Controller-period accounting.
- Mixed stepping without double-counting.
- Invalid input rejection.
- Partial physics execution failures.
- Incomplete controller-period failures.
- Reset and new episode accounting.
- Failed reset preserving counts.
- Simulation time sourced from MuJoCo.
- Changing physics timesteps.
- Independent simulator isolation.
- Read-only public properties.
- Lifecycle protection.
- Forward computation not counting as integration.

## 12. Limitations

P0-006.07 does not provide:

- Wall-clock execution measurements.
- Real-time controller scheduling guarantees.
- CPU or GPU performance profiling.
- External MuJoCo integration accounting.
- Motion planning or inverse kinematics.
- MAHQ policy inference.
- Training episode evaluation.
- Collision-safe movement guarantees.

## 13. Acceptance Criteria

P0-006.07 may be marked complete when:

1. Physics integrations are counted once each.
2. Completed controller periods are counted once each.
3. Direct physics stepping does not count as a controller tick.
4. Invalid requests preserve the counters.
5. Partial execution is accounted for correctly.
6. Reset returns both counters to zero.
7. Simulation time remains authoritative.
8. Independent instances remain isolated.
9. All dedicated and regression tests pass.
10. Evidence is recorded in P0_CHECKLIST.md.

## 14. Change Control

All source changes require explicit approval under AGENTS.md.

Compilation and test execution require separate approval.

Do not modify model assets, scene composition,
simulation configuration or scripts/run_tests.py.
