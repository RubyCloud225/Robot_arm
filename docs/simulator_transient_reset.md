
# P0-006.03 — Transient-State Cleanup

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Lifecycle API
**Ticket:** P0-006.03
**Status:** Proposed — pending approval and testing

## 1. Objective

Ensure Simulator.reset() removes transient MuJoCo state
that could otherwise affect a subsequent simulation run.

P0-006.02 already restores the primary robot, ball
and target configuration.

P0-006.03 adds native engine-state cleanup.

## 2. Reset Strategy

The simulator captures the initial composed scene state
following a successful load().

Reset executes these operations in order:

1. Validate lifecycle state.
2. Reject unsupported seed requests.
3. Validate reset snapshot availability and dimensions.
4. Invoke mujoco.mj_resetData(model, data).
5. Restore captured qpos, qvel, ctrl and act arrays.
6. Restore the original fixed target position.
7. Restore initial simulation time.

The native MuJoCo reset operation is used because
transient simulation data extends beyond qpos and qvel.

## 3. Transient State

The required explicitly tested fields are:

### qfrc_applied

User-applied generalized forces.

Expected post-reset state: all zeros.

### xfrc_applied

External forces and torques on bodies.

Expected post-reset state: all zeros.

### qacc_warmstart

Solver warm-start acceleration estimates.

Expected post-reset state: all zeros.

### act

Stateful actuator activation.

Expected post-reset state: the captured baseline.

The length of act is model-dependent and is not assumed
to equal the number of actuator control channels.

### ctrl

Actuator command inputs.

Expected post-reset state: the captured baseline.

Controls must not be unconditionally zeroed because
the model may require nonzero initial hold commands.

## 4. Preserved Primary State

The following values are restored after native reset:

- All robot arm joint positions and velocities.
- Both gripper finger joint states.
- Ball translation and orientation quaternion.
- Ball linear and angular velocities.
- Initial actuator controls.
- Initial actuator activation.
- Target fixed-body position.
- Initial simulation clock.

Neither MjModel nor MjData is replaced.

## 5. Additional Native Reset Effects

mujoco.mj_resetData() resets other MuJoCo-managed data
in addition to the explicitly tested fields.

This may include additional integration, user-data,
mocap and solver-related state according to the
selected MuJoCo release and compiled model.

The current reset snapshot does not restore arbitrary
runtime edits to every such field.

Future changes introducing mocap-controlled targets,
custom plugins or user-data-dependent tasks require
their own reset-contract review and tests.

## 6. Safety and Failure Behaviour

Reset is permitted only in the LOADED state.

All known input and shape validation occurs before
the native reset operation.

An unsupported seed must not alter simulation state.

A missing snapshot or incompatible state shape must
not trigger partial reset.

Unexpected native MuJoCo errors propagate; full
transactional rollback after an engine error is not
guaranteed in this ticket.

## 7. Physics Semantics

reset() must not call mj_step().

P0-006.03 also does not call mj_forward().

Derived body transforms and contact information are
therefore not yet guaranteed to match the restored
primary state immediately after reset.

P0-006.04 will introduce the required forward
recomputation.

## 8. Testing

Test file:

    tests/test_simulator_transients.py

Coverage:

- Generalized force cleanup.
- External body force/torque cleanup.
- Solver warm-start cleanup.
- Simultaneous transient contamination.
- Captured actuator activation restoration.
- Preservation of initial actuator controls.
- Primary state restoration alongside cleanup.
- Ball quaternion integrity.
- Target position restoration.
- Simulation clock restoration.
- Multiple consecutive resets.
- Isolation between independent simulators.
- MjModel/MjData identity preservation.
- No mj_step calls.
- No premature mj_forward calls.
- Exactly one native reset per successful reset call.
- Rejected seed leaves state unchanged.
- Invalid reset snapshot is rejected before mutation.
- No changes to compiled joint limits.
- Headless operation.

## 9. Known Limitations

The complete end-to-end reset contract remains in
development until P0-006.04 is complete.

This ticket does not provide:

- Refreshed kinematics/contact observations.
- Actuator command validation.
- Controller timing or automatic stepping.
- Randomised, seed-controlled reset conditions.
- Reset hooks for arbitrary external controllers.
- General restoration of modified MjModel parameters.

## 10. Acceptance Criteria

P0-006.03 can be marked complete when:

- Required transient forces and warm-start state clear.
- Initial actuator controls and activations are restored.
- Primary reset behaviour remains correct.
- Independent instances remain isolated.
- Invalid requests cause no reset-state mutation.
- Existing model/data objects remain unchanged.
- No unexpected physics stepping occurs.
- All new tests pass.
- All existing regression tests pass.
- Evidence is recorded in STATUS.md.

## 11. Change Control

All changes require explicit code approval under AGENTS.md.

Test execution requires separate approval.

Do not alter model assets, scene composition,
model ownership, or scripts/run_tests.py.
