
# P0-006.04 — Forward Kinematics and Contact Refresh

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Lifecycle API
**Ticket:** P0-006.04
**Status:** Proposed — awaiting approval and validation

## Objective

Ensure that Simulator.reset() produces a physically
consistent derived simulation state after restoring
the initial robot, gripper, ball and target configuration.

Reset must recompute world transforms and current
contact information before returning to the caller.

## Dependencies

- P0-006.01 — Lifecycle ownership.
- P0-006.02 — Primary-state restoration.
- P0-006.03 — Transient-state cleanup.

## Reset Sequence

The complete reset sequence is:

1. Validate lifecycle state and reset arguments.
2. Validate the saved reset snapshot.
3. Call mujoco.mj_resetData(model, data).
4. Restore saved qpos and qvel.
5. Restore saved ctrl and act.
6. Restore the fixed target position.
7. Restore initial simulation time.
8. Call mujoco.mj_forward(model, data).

No calls to mujoco.mj_step() are permitted.

## Forward Dynamics

mujoco.mj_forward() performs forward calculations for
the current model and state without integrating the
simulation forward in time.

The operation updates derived simulation quantities,
including:

- Body world positions and orientations.
- Site world positions.
- Geometry world positions.
- Contact detection information.
- Other derived dynamics fields.

The operation must occur after the restored state
and target placement have been written.

## Coordinate Consistency

The Panda arm and gripper must report world transforms
consistent with the restored joint configuration.

The free ball's body position must match its restored
free-joint translation.

The target marker's world position must reflect its
restored fixed-body placement.

All positional observations must use the world frame
provided by the composed MuJoCo scene.

## Contact Consistency

Contacts produced by a previous ball position must
not remain after a reset to the initial configuration.

Contact comparisons should use geometry identifiers,
not solely raw contact counts.

Raw contact count is not proof of stable gripping.

## Numerical Validation

Tests compare derived positions against a reference
calculated by forward dynamics at the initial scene.

The proposed positional tolerance is:

    absolute tolerance: 1e-10
    relative tolerance: 0

This is an implementation regression tolerance,
not a physical robot accuracy specification.

## Transient-State Interaction

mj_forward() may calculate nonzero derived dynamics
quantities during reset.

Tests must therefore distinguish:

- Applied forces that must remain cleared.
- Derived forces or accelerations that may legitimately
  be calculated by forward dynamics.
- Solver internal values governed by MuJoCo semantics.

The simulator must not zero derived fields after
mj_forward() merely to satisfy a test.

## Simulation Time

Reset restores the initial simulation clock.

Forward computation does not advance data.time.

Physics integration and explicit step-count tracking
remain responsibilities of P0-006.05–006.07.

## Object Identity

Reset must reuse the same:

- MjModel instance.
- MjData instance.
- Captured baseline for the simulator.

No scene recompilation is permitted during reset.

## Error Handling

Invalid lifecycle states and unsupported seed requests
must fail before transient cleanup or forward dynamics.

Snapshot validation must occur before modifying state.

Unexpected failures inside MuJoCo are not guaranteed
to be transactionally reversible by this ticket.

## Test Coverage

File:

    tests/test_simulator_forward.py

Required checks:

- Body position refresh.
- Body orientation refresh.
- Site position refresh.
- Geometry position refresh.
- Ball free-joint/world-coordinate consistency.
- Target marker refresh.
- Stale contact elimination.
- Full primary-state preservation.
- Simulation-time preservation.
- No mj_step calls.
- Exactly one explicit mj_forward call.
- Correct ordering of state restoration and forward.
- Cleared applied-force preservation.
- Repeated reset determinism.
- Model/data identity preservation.
- Lifecycle error handling.
- Multi-instance independence.

The obsolete P0-006.03 test forbidding mj_forward()
must be replaced.

## Limitations

P0-006.04 does not implement:

- Physics time advancement.
- Joint or gripper control policy.
- Safe actuator command interfaces.
- Controller-period scheduling.
- Automated reaching or inverse kinematics.
- Successful ball grasping.
- Seed-controlled random placements.

These belong to subsequent milestones.

## Acceptance Criteria

P0-006.04 is complete when:

1. A successful reset recomputes derived physical state.
2. Stale contact information is eliminated.
3. The restored world transforms match the initial scene.
4. Primary qpos/qvel/ctrl and time remain unchanged
   by forward computation.
5. Previously validated transient-state behaviour remains.
6. The reset operation never performs a physics step.
7. All dedicated tests pass.
8. The full repository regression suite passes.
9. Completion evidence is recorded in STATUS.md.

## Change Control

All changes require explicit owner approval under AGENTS.md.

Test execution requires separate approval.

Do not modify robot assets, model composition, model
ownership, CLI scripts or scripts/run_tests.py.
