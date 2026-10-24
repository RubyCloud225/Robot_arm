
# P0-005.05 — Robot Structural Validation

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Foundation
**Ticket:** P0-005.05
**Status:** Proposed — awaiting approval and validation

## 1. Objective

Validate that the composed Franka Emika Panda simulation
contains the expected articulated structure before physics
stepping or control execution.

The validator checks joint counts, joint types, actuator
counts, model state dimensions and ball free-joint ownership.

This is separate from successful MJCF compilation.

## 2. Dependencies

- P0-005.01 — Model loading
- P0-005.02 — Model and state ownership
- P0-005.03 — Named component resolution
- P0-005.04 — Model-specific ID and address caching

The structural validator operates on a compiled MjModel.

It must not modify model parameters or simulation state.

## 3. Expected Panda Structure

### Arm

Seven revolute joints:

- joint1
- joint2
- joint3
- joint4
- joint5
- joint6
- joint7

Every arm joint must use mjJNT_HINGE.

Each arm joint contributes:

- 1 qpos coordinate
- 1 qvel coordinate

Total arm dimensions:

- qpos: 7
- qvel: 7

### Gripper

Two finger joints:

- finger_joint1
- finger_joint2

Both must use mjJNT_SLIDE.

Each contributes:

- 1 qpos coordinate
- 1 qvel coordinate

Total finger dimensions:

- qpos: 2
- qvel: 2

The two finger joints are coupled in the selected
Panda model.

Their physical coupling is not validated by this ticket.

### Actuators

Expected arm actuators:

- actuator1 through actuator7

Expected gripper actuator:

- actuator8

Expected total actuator count: 8

The standard Panda gripper uses a tendon-driven actuator.

This ticket validates the expected actuator names and
count, not transmission semantics or control values.

### Ball

Expected body:

    ball

Expected joint:

    ball_free

Required joint type:

    mjJNT_FREE

The joint must belong to the named ball body.

Its position state contains:

- X position
- Y position
- Z position
- Quaternion W
- Quaternion X
- Quaternion Y
- Quaternion Z

Total qpos width: 7

Its velocity state contains:

- 3 translational velocity DoFs
- 3 rotational velocity DoFs

Total qvel width: 6

The quaternion must not be treated as three independent
Euler angles.

## 4. Expected Complete Model Dimensions

| Property | Expected value |
|---|---:|
| Arm joints | 7 |
| Finger joints | 2 |
| Ball free joints | 1 |
| Total joints (njnt) | 10 |
| Arm actuators | 7 |
| Gripper actuators | 1 |
| Total actuators (nu) | 8 |
| Position coordinates (nq) | 16 |
| Velocity coordinates (nv) | 15 |

These requirements apply to the selected complete Panda
pick-and-place task scene.

They must not be applied to the standalone robot XML
or unrelated generic MJCF models.

## 5. Validation Behaviour

The validator must:

1. Confirm the argument is a compiled MjModel.
2. Verify expected total joint count.
3. Verify expected actuator count.
4. Resolve the seven named arm joints.
5. Confirm each arm joint is a hinge.
6. Resolve the two named finger joints.
7. Confirm each finger joint is a slide.
8. Resolve the ball_free joint.
9. Confirm ball_free is a free joint.
10. Confirm ball_free belongs to the ball body.
11. Validate joint state-array bounds.
12. Verify total nq and nv.
13. Verify required actuator names.
14. Aggregate structural errors.
15. Reject invalid models before stepping.

All validation must be deterministic and read-only.

## 6. Error Handling

Structural failures raise:

    ModelStructureValidationError

The error should identify every detected structural
violation rather than reporting only the first.

Missing names, incorrect joint types, unexpected counts
and invalid dimensions must produce actionable messages.

No partial success report may be returned.

## 7. Testing

Test file:

    tests/test_model_structure.py

Required regression coverage:

- Valid synthetic Panda-shaped structure
- Incorrect total joint count
- Incorrect actuator count
- Missing arm joint
- Incorrect arm/finger joint type
- Missing gripper actuator
- Unexpected extra actuator
- Invalid ball joint type
- Free joint attached to incorrect body
- Incorrect state dimensions
- Invalid input model
- No state modification
- No physics stepping
- Real composed Panda integration

Where practical, adversarial tests must use temporary
or in-memory MJCF fixtures.

Production assets must remain unchanged.

## 8. Test Runner

The existing repository runner is used:

    ./scripts/run_tests.py

The new tests must be discovered automatically by pytest.

No modifications to scripts/run_tests.py are required.

The actual Panda model is resolved using the existing
ROBOT_ARM_PANDA_XML environment configuration.

A skipped Panda integration test is not considered a pass
for the full-model validation requirement.

## 9. Limitations

Successful structural validation does not establish:

- Correct actuator-to-joint transmission
- Correct gripper coupling
- Valid actuator control limits
- Valid physical joint ranges
- Stable contact dynamics
- Safe motion under commanded controls
- Cartesian reachability
- Inverse kinematics
- Successful grasping or manipulation

These responsibilities are addressed by later tickets.

## 10. Acceptance Criteria

P0-005.05 is complete when:

- The structural validator is implemented.
- Seven Panda arm hinge joints are confirmed.
- Two finger slide joints are confirmed.
- Eight expected actuators are confirmed.
- Ball free-joint ownership is confirmed.
- Ball qpos/qvel dimensions are validated.
- Full model state dimensions are validated.
- Missing, extra and malformed components are rejected.
- Adversarial tests pass.
- Real Panda integration passes.
- Existing regression tests remain passing.
- Test evidence is recorded in STATUS.md.

## 11. Change Control

All code and documentation changes require explicit
approval under AGENTS.md.

Test execution requires separate approval.

No MJCF assets, production simulation scripts, actuator
settings or physical control behaviour may be changed
without authorisation.
