
# P0-007.01 — Typed Actuator Command Interface

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0-007 — Robot Actuator Control and Movement
**Status:** Proposed — awaiting approval and validation

## 1. Objective

Introduce a complete, typed actuator command representation
for the selected Franka Panda simulation.

Convert named commands into the compiled MuJoCo actuator
ordering without modifying the existing Simulator API.

This ticket concerns command representation and conversion.
It does not establish reliable robot movement or autonomous
behaviour.

## 2. Architecture

The interface contains two components:

1. PandaCommand
2. ActuatorRegistry

PandaCommand stores eight named actuator control values.

ActuatorRegistry resolves logical channel names to the
actual actuator IDs in the compiled MjModel.

The resulting NumPy vector is passed to the existing
Simulator.step() or Simulator.step_controller_period() API.

No new physics stepping implementation is introduced.

## 3. Command Structure

PandaCommand is a frozen, slotted dataclass.

Its required fields are:

| Field | Expected MuJoCo actuator |
|---|---|
| arm1 | actuator1 |
| arm2 | actuator2 |
| arm3 | actuator3 |
| arm4 | actuator4 |
| arm5 | actuator5 |
| arm6 | actuator6 |
| arm7 | actuator7 |
| gripper | actuator8 |

All eight fields are required.

Partial commands are not supported.

Omitting fields, adding unknown fields or supplying
invalid scalar types is rejected.

The command values are converted to finite Python floats.

## 4. Control Semantics

Every command field represents a raw actuator control
value for the corresponding MuJoCo actuator.

These values must not automatically be interpreted as:

- Joint angles in radians.
- Joint velocities in radians per second.
- Joint torques in newton-metres.
- Gripper opening widths in metres.

The physical meaning depends on the selected actuator
transmission, gain, bias, dynamics and model definition.

Detailed verification of those semantics belongs to
P0-007.17.

## 5. Registry Construction

ActuatorRegistry.from_model(model) reads the actual
compiled MuJoCo model.

It verifies:

- The model is an MjModel.
- The expected number of actuators is present.
- Every actuator has a name.
- Actuator names are unique.
- Expected Panda actuator names exist.
- No additional unexpected actuators exist.
- Compiled actuator IDs are resolved by name.
- Enabled control ranges are finite and correctly ordered.

No assumption is made that actuator1 has array index 0.

The registry uses each resolved actuator ID when
constructing the final vector.

## 6. Control Vector

The result of:

    registry.to_vector(command)

is an independent NumPy float64 array.

Expected shape:

    (model.nu,)

For this reference Panda model:

    (8,)

Vector index i corresponds to the actuator with
compiled MuJoCo actuator ID i.

The vector must include every actuator exactly once.

## 7. Range Validation

PandaCommand validates numeric representation and finiteness.

ActuatorRegistry validates model-specific actuator
control limits when converting commands.

Out-of-range values are rejected.

Values are not silently clamped or substituted.

For actuators without enabled control limits, the
registry does not invent artificial numeric ranges.

## 8. Gripper

The selected Panda model has previously documented
gripper control limits of:

    0 to 255

These are raw actuator control values.

They are not an opening width in millimetres, metres
or any other physical distance unless verified against
the selected model's control transmission.

Actual finger movement and opening width will be
measured in subsequent P0-007 tickets.

## 9. Command Example

    registry = ActuatorRegistry.from_model(sim.model)

    command = PandaCommand(
        arm1=...,
        arm2=...,
        arm3=...,
        arm4=...,
        arm5=...,
        arm6=...,
        arm7=...,
        gripper=...,
    )

    vector = registry.to_vector(command)

    sim.step_controller_period(
        control=vector,
        period=0.04,
    )

The example intentionally leaves actuator values
unspecified because their physical meaning must
be verified for the reference model.

For an initial known-valid command, the actuator
values may be read from the loaded model's existing
data.ctrl buffer using resolved actuator IDs.

That does not by itself establish a stable hold policy.

## 10. Immutability

PandaCommand is immutable through ordinary attribute
assignment.

ActuatorDescriptor and ActuatorRegistry are also frozen.

Registry metadata is stored in tuples.

Command vector conversion returns a fresh mutable
NumPy array for use with MuJoCo.

This is application-level immutability, not a security
boundary against deliberate Python reflection.

## 11. Failure Behaviour

Registry creation rejects an incompatible model.

Command construction rejects invalid numeric values.

Vector conversion rejects missing mappings,
invalid registry definitions and out-of-range controls.

The conversion process performs no physics stepping
and does not modify MjData.

The existing Simulator stepping methods retain
responsibility for final command validation and
execution.

## 12. Verification

Test module:

    tests/test_actuator_commands.py

Required evidence includes:

- Complete typed command representation.
- Command immutability.
- Missing and unexpected field rejection.
- Invalid scalar and nonfinite value rejection.
- Real model registry construction.
- Complete actuator ID coverage.
- Actual compiled name-to-ID mapping.
- Model-derived control range metadata.
- Independence from descriptor iteration order.
- Rejection of malformed registry mappings.
- Rejection rather than clamping of invalid controls.
- Independent converted vector storage.
- Conversion without simulation mutation.
- Compatibility with Simulator.step().
- Compatibility with step_controller_period().

Existing simulator regression tests must remain passing.

## 13. Known Limitations

P0-007.01 does not implement:

- Physical movement or hold policies.
- Joint-space trajectory generation.
- Cartesian inverse kinematics.
- Automatic reachability analysis.
- Gripper opening-width control.
- Collision avoidance.
- Grasping.
- Learned movement behaviour.
- MAHQ integration.

Typed actuator commands alone do not guarantee
safe or meaningful robot motion.

## 14. Acceptance Criteria

P0-007.01 is complete when:

1. All eight named fields are required.
2. Commands are immutable.
3. Numeric values are finite and correctly typed.
4. The compiled model provides actuator ordering.
5. Every actuator is mapped exactly once.
6. Unknown or incomplete mappings are rejected.
7. Model control ranges are respected.
8. Invalid controls are rejected without clamping.
9. Conversion does not mutate simulation state.
10. The vector works with the existing stepping API.
11. All new and existing regression tests pass.
12. Evidence is recorded in P0_CHECKLIST.md.

## 15. Change Control

This ticket introduces only:

- src/robot_arm_sim/actuator_commands.py
- tests/test_actuator_commands.py
- docs/actuator_commands.md

No changes are authorised to:

- src/robot_arm_sim/simulator.py
- Robot models or scene assets
- Actuator definitions or geometry
- Simulation configuration
- scripts/run_tests.py
- AGENTS.md

Source implementation requires explicit approval.

Compilation and test execution require separate approval.
