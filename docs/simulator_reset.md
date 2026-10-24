
# P0-006.02 — Deterministic Simulation State Reset

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Lifecycle
**Status:** Proposed — pending approval and validation

## Objective

Implement Simulator.reset() to restore the primary
simulation state captured immediately after load().

Reset must preserve the initial Panda home configuration,
finger positions, actuator controls, ball free-joint
state and target placement.

No model recompilation or physics stepping is permitted.

## Initial State Capture

Following a successful scene load, copy:

- Entire MjData.qpos array.
- Entire MjData.qvel array.
- Entire MjData.ctrl array.
- MjModel.body_pos for the named target body.
- Initial MjData.time value.

These values form the private reset snapshot.

The snapshot must not contain writable views into
the live MuJoCo state arrays.

## Reset Contract

Calling reset() in the LOADED state restores:

1. All position coordinates.
2. All velocity coordinates.
3. All actuator controls.
4. The target body's fixed position.
5. Initial simulation time.

The existing MjModel and MjData objects are retained.

Reset must not allocate a replacement scene.

## Robot State

The Panda has seven arm joints and two finger joints.

The entire qpos and qvel arrays are copied rather than
manually reconstructing the robot's home pose.

This preserves values created by the existing scene
composition and home-keyframe initialization logic.

## Ball State

The ball is connected through ball_free, a free joint.

It contributes:

    qpos: 7 coordinates
    qvel: 6 coordinates

The position coordinates contain a three-dimensional
translation and a four-dimensional quaternion.

Reset copies the stored quaternion directly without
converting it to Euler angles.

Both linear and angular velocities are restored.

## Target State

The target is a fixed body.

Its configured position is represented by
MjModel.body_pos rather than a free-joint qpos entry.

Reset restores the target's original body position.

## Simulation Time

Reset restores the initial recorded MjData.time.

It does not call mujoco.mj_step().

## Seeds

P0-006.02 supports reset(seed=None).

A non-None seed is rejected with NotImplementedError.

Seed-driven random initialisation and deterministic
command-sequence replay are handled in P0-010.

No seed may be silently ignored.

## Error Handling

Reset is permitted only after load() and before close().

Reset must fail if:

- The simulator is NEW or CLOSED.
- The initial snapshot is unavailable.
- The required target body is missing.
- Current primary-state array dimensions differ
  from the captured snapshot.

Invalid seed requests must not modify state.

## Current Limitations

P0-006.02 restores primary state only.

It does not yet guarantee cleanup of:

- Applied generalized forces.
- External body forces.
- Actuator activation state, if applicable.
- Solver warm-start values.
- Cached contact information.
- Derived body/world transforms.

The relevant responsibilities remain assigned to:

P0-006.03 — Transient-state cleanup.
P0-006.04 — Forward recomputation.

An immediate observation of derived positions and
contacts should not be considered refreshed until
P0-006.04 is implemented.

## Tests

Test file:

    tests/test_simulator_reset.py

Coverage includes:

- All arm joint positions and velocities.
- Both finger joints.
- Ball translation and quaternion.
- Ball linear and angular velocities.
- Every actuator command.
- Target fixed-body position.
- Simulation clock.
- Model/data object identity preservation.
- Repeated reset idempotence.
- Snapshot independence.
- Multi-instance isolation.
- Invalid lifecycle operations.
- Unsupported seed behaviour.
- No unexpected physics stepping.
- No model recompilation.
- Snapshot cleanup on close.

Update the existing lifecycle reset-placeholder test
to reflect the new reset implementation.

## Acceptance Criteria

P0-006.02 is complete when:

- Reset restores every captured primary-state field.
- The real Panda model passes reset integration tests.
- Reset preserves MjModel and MjData identity.
- Repeated resets produce identical baseline values.
- Independent simulators cannot alter each other's
  reset snapshots or live state.
- Adversarial tests pass.
- All pre-existing tests remain passing.
- Documentation and checklist evidence are updated.

## Change Control

All proposed changes require approval under AGENTS.md.

Compilation and test execution require separate approval.

Do not modify robot assets, simulation composition,
or scripts/run_tests.py for this ticket.
