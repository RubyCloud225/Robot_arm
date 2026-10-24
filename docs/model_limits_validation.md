
# P0-005.06 — Joint and Actuator Limits Validation

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Foundation
**Status:** Proposed — pending validation

## 1. Objective

Validate the finite bounds and initial values of the
selected Franka Emika Panda simulation.

The validator must detect incorrect joint limits,
actuator control ranges and home-keyframe values
before any physics stepping or robot control.

## 2. Scope

The validator checks:

- Seven arm joint limits.
- Two finger joint limits.
- Eight actuator control ranges.
- Required limit-enforcement flags.
- Finite and strictly ordered bounds.
- Home-keyframe joint positions.
- Home-keyframe actuator controls.
- Optional named default joint positions.
- Optional named default actuator commands.

The ball's free joint is excluded from scalar joint
range validation.

## 3. Joint Limit Requirements

All seven arm joints must have enabled, finite limits.

The two finger slide joints must also have enabled,
finite limits.

For every required scalar joint:

    lower < upper

Both endpoints must be finite.

Home positions and explicitly provided default values
must be within the corresponding limits.

No automatic clipping or clamping is performed.

## 4. Actuator Requirements

The seven arm actuators and one gripper actuator must
have enabled control limits.

Every control range must have finite endpoints and
strictly increasing values.

The gripper actuator uses:

    actuator8
    control range: [0, 255]

This is an actuator-specific command scale.

It must not be interpreted as gripper opening width
in metres or as an angular position in radians.

## 5. Home-Keyframe Validation

Read the named "home" keyframe from the compiled model.

Resolve joints by name.

Use:

    model.jnt_qposadr[joint_id]

to access the appropriate scalar joint position.

Use actuator IDs for the corresponding keyframe
control-array entries.

Do not assume joint IDs, qpos addresses and actuator
IDs are equivalent.

All required home positions and controls must be
finite and inside their validated bounds.

## 6. Error Behaviour

Validation failures raise ModelLimitsError.

Errors must identify the affected joint or actuator
and explain the invalid value or missing constraint.

Multiple errors should be aggregated where possible.

No partially successful report is returned.

## 7. Read-Only Contract

The validator must not:

- Modify compiled joint ranges.
- Modify actuator ranges.
- Change model parameters.
- Modify qpos or qvel.
- Modify actuator commands.
- Advance simulation time.
- Execute physics.
- Modify MJCF files.

## 8. Testing

Tests are located in:

    tests/test_model_limits.py

The regression suite must include:

- Valid real Panda limits.
- Valid home-keyframe values.
- Missing or disabled limits.
- NaN and infinite bounds.
- Equal and reversed range endpoints.
- Out-of-range defaults.
- Invalid gripper command values.
- Corrupted home-keyframe values.
- Read-only validation.
- Failure isolation.
- Preservation of prior P0 functionality.

Tests must use private model instances when
mutating compiled parameters.

## 9. Test Runner

The existing project runner is used:

    ./scripts/run_tests.py

No test-runner modifications are required.

## 10. Limitations

Successful limits validation does not prove:

- Correct actuator-to-joint transmission.
- Correct position-servo behaviour.
- Stable physical motion.
- Safe collision-free trajectories.
- Gripper opening width.
- Reachability of the ball.
- Successful grasping.
- Safe operation of physical hardware.

These responsibilities belong to later tickets.

## 11. Acceptance Criteria

P0-005.06 is complete when:

- All required joint limits are validated.
- All actuator control ranges are validated.
- Limit-enforcement flags are checked.
- Home-keyframe values are validated.
- Invalid defaults are rejected.
- Adversarial cases pass.
- Real Panda integration passes.
- The full repository regression suite passes.
- Test results are recorded in STATUS.md.

## 12. Change Control

Implementation and execution remain subject
to AGENTS.md approval requirements.

No production model or simulation scripts may
be modified without explicit permission.
