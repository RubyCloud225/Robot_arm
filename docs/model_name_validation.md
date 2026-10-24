
# P0-005.03 — Model Component Name Resolution and Validation

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Foundation
**Ticket:** P0-005.03
**Status:** Proposed — pending implementation and test execution

## 1. Objective

Provide a centralised, deterministic mechanism for resolving
required MuJoCo model components by their names.

The system must identify missing or incorrectly named joints,
actuators, sites and bodies before simulation stepping.

A successfully compiled model is not necessarily a valid model
for the intended robot task.

Structural validation is therefore performed separately from
the model-loading stage.

## 2. Scope

P0-005.03 includes:

- Resolving named joints.
- Resolving named actuators.
- Resolving named bodies.
- Resolving named sites.
- Validating required component existence.
- Rejecting invalid names and duplicate schema entries.
- Reporting all missing required components.
- Supporting both the default Panda schema and custom schemas.
- Performing validation without modifying simulation state.

P0-005.03 excludes:

- Joint and actuator address caching.
- Joint-type and dimension validation.
- Actuator command-range validation.
- Robot reachability analysis.
- Cartesian control and inverse kinematics.
- Collision and contact validation.
- Physics stepping and simulation reset.

These responsibilities belong to subsequent P0 tickets.

## 3. Existing System

The repository uses MuJoCo as its physics and robot-model engine.

P0-005.01 provides explicit MJCF model loading and error
handling.

P0-005.02 provides independent model/state ownership through
SimulationInstance.

P0-005.03 introduces a centralised name-resolution layer
operating on a compiled mujoco.MjModel.

The model resolver must remain independent of rendering,
interactive viewers and simulation lifecycle methods.

## 4. Source Model

The selected robot is the Franka Emika Panda from
Google DeepMind's MuJoCo Menagerie.

Expected entrypoint:

    franka_emika_panda/panda.xml

The project uses the standard Panda model, not its MJX variant.

The canonical source-level mapping is documented in:

    docs/assets_and_licenses.md

Compiled-model name validation is required because upstream
MJCF revisions or composition changes may alter model contents.

## 5. Required Model Components

### 5.1 Arm Joints

| Name | Role |
|---|---|
| joint1 | Arm joint 1 |
| joint2 | Arm joint 2 |
| joint3 | Arm joint 3 |
| joint4 | Arm joint 4 |
| joint5 | Arm joint 5 |
| joint6 | Arm joint 6 |
| joint7 | Arm joint 7 |

All seven joints must exist in the compiled Panda scene.

This ticket checks names, not movement limits or joint
types. Those checks belong to P0-005.05 and P0-005.06.

### 5.2 Finger Joints

| Name | Role |
|---|---|
| finger_joint1 | First gripper finger joint |
| finger_joint2 | Second gripper finger joint |

The underlying model includes coupled parallel-jaw behaviour.

This ticket does not attempt to interpret that coupling
or calculate opening width.

### 5.3 Arm Actuators

Required actuator names:

    actuator1
    actuator2
    actuator3
    actuator4
    actuator5
    actuator6
    actuator7

These are the seven Panda arm-control channels.

They must not be assumed to have numeric indices
corresponding to joint IDs.

### 5.4 Gripper Actuator

Required actuator:

    actuator8

The standard Panda model uses this actuator for its
tendon-driven gripper mechanism.

Its command range is documented as 0–255 control units
in the reviewed model definition.

These values are not finger widths in metres.

P0-005.03 confirms the actuator exists only.

Control semantics, transmission and permitted command
ranges are validated separately.

### 5.5 Robot Bodies

Required bodies:

    hand
    left_finger
    right_finger

These provide named structural references for later
end-effector and finger-state observations.

### 5.6 Task Bodies

Required bodies:

    ball
    target

These bodies belong to the composed pick-and-place scene.

They are not expected in the standalone Panda MJCF file.

### 5.7 Task Joint

Required joint:

    ball_free

This must exist in the composed scene.

Validation that it is a MuJoCo free joint, with seven
qpos coordinates and six qvel coordinates, belongs to
P0-005.05.

### 5.8 Sites

Required site:

    target_marker

The selected standard Panda MJCF does not establish a
project-owned end-effector site.

Do not assume that "grasp_site" or "ee_site" exists.

Until an end-effector site is explicitly introduced and
validated, the hand body remains the named structural
reference for end-effector observations.

A body origin is not automatically equivalent to a
tool-centre point.

## 6. Name-Resolution Rules

The resolver uses:

    mujoco.mj_name2id(model, object_type, name)

The object category is always explicit.

Supported categories:

- mjOBJ_JOINT
- mjOBJ_ACTUATOR
- mjOBJ_BODY
- mjOBJ_SITE

The resolver must reject:

- Missing names.
- Empty strings.
- Whitespace-only strings.
- Leading/trailing whitespace.
- Unsupported object categories.
- Invalid model objects.
- Duplicate names within the same object category.
- Empty complete-validation schemas.

Names are case-sensitive.

No implicit normalisation, prefix matching or fallback
to another object category is permitted.

## 7. Error Handling

Missing components raise:

    ModelStructureError

The error must identify:

1. The failing object category.
2. The requested name.
3. The reason for failure.

Complete-schema validation collects all identified
missing-name errors before raising.

This allows a developer to fix multiple model-definition
problems in one pass.

A partial successful result must not be returned when
a required component is missing.

## 8. Model-Specific Identifier Rules

Numeric MuJoCo IDs are local to a compiled model.

They are not persistent identifiers.

Two independently compiled models may assign different
numeric IDs to equivalent named components.

Therefore:

- Resolve names separately for each model.
- Never reuse IDs from another model without validation.
- Never interpret an actuator ID as a joint ID.
- Never interpret a joint ID as a qpos address.
- Never treat a body ID as a site ID.
- Never use -1 from a failed name lookup as an array index.

P0-005.04 will establish validated model-specific ID
caching and correct address handling.

## 9. Read-Only Behaviour

Name resolution must not modify:

- Joint positions.
- Joint velocities.
- Actuator commands.
- Simulation time.
- Body masses.
- Joint ranges.
- Physics timestep.
- Model assets.

The resolver must not call:

    mujoco.mj_step()

It does not need to create a renderer or graphical viewer.

All structural validation tests must run headlessly.

## 10. Validation Strategy

The automated test suite must cover:

### Valid Resolution

- Existing joint name.
- Existing actuator name.
- Existing body name.
- Existing site name.
- Complete custom schema.
- Complete composed Panda scene.

### Invalid Resolution

- Missing joint.
- Missing actuator.
- Missing body.
- Missing site.
- Wrong object category.
- Invalid model type.
- Invalid object type.
- Blank name.
- Whitespace-padded name.
- Incorrect name capitalisation.
- Duplicate schema entries.
- Completely empty schema.
- Incorrect schema type.

### Robustness

- Multiple failures reported together.
- Model-definition ordering changes.
- No unintended model mutation.
- No unintended state mutation.
- No physics stepping.
- No graphical dependencies.
- No silent fallback to another component type.


## Automated Test Integration

P0-005.03 tests are integrated into the existing repository
test runner:

    scripts/run_tests.py

The test module is:

    tests/test_model_ids.py

The runner discovers the tests automatically through pytest.

Execute the full regression suite using:

    python scripts/run_tests.py

The runner must:

- Use the project virtual environment.
- Resolve the configured Panda model.
- Include all P0-005.03 unit tests.
- Include adversarial name-resolution tests.
- Include real Panda model integration tests.
- Preserve pytest failure exit codes.
- Report skipped tests accurately.
- Leave production scripts and assets unchanged.

No separate test runner is required.

P0-005.03 cannot be marked complete until the required
tests pass and their results are recorded in STATUS.md.


## 11. Real Panda Integration

A dedicated integration test must load the actual
composed Franka Panda task scene.

It must verify every component in the default
RequiredModelNames schema.

The test must use the reviewed local model assets.

An integration test that is skipped because the Panda
model is unavailable must not be reported as passed.

The standalone Panda model must not be incorrectly
validated against task-scene names such as ball,
target or target_marker.

## 12. Limitations

Successful name resolution proves that the expected
names exist in the compiled model.

It does not prove:

- The component has the expected physical meaning.
- A joint has the correct type.
- An actuator controls the expected joint.
- An actuator command is within its control range.
- A site has the correct local transform.
- The gripper can reach the ball.
- The model is physically stable.
- A grasp can be completed.
- A cached numeric ID remains valid for another model.

These checks belong to later milestones.

## 13. Acceptance Criteria

P0-005.03 can be marked complete only when:

- The resolver is implemented and reviewed.
- All required named-object categories are supported.
- Missing and invalid components produce useful errors.
- Duplicate and malformed schemas are rejected.
- No physics stepping occurs during validation.
- Synthetic regression tests pass.
- Adversarial regression tests pass.
- Real composed Panda integration passes.
- Existing model-loading and ownership tests remain passing.
- Test evidence is recorded in STATUS.md.

## 14. Change Control

All code and documentation changes require approval
under AGENTS.md.

No existing simulation scripts or MJCF files may be
modified without separate explicit permission.

No tests, builds or simulations may be executed
without explicit execution approval.
