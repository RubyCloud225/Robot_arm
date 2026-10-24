
# P0-005.04 — Model Identifier and Address Caching

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Foundation
**Status:** Proposed — awaiting implementation and validation

## 1. Objective

Create a validated, model-specific cache of MuJoCo identifiers
and joint-state addresses.

Prevent incorrect assumptions that joint IDs, qpos addresses,
qvel addresses, actuator IDs, body IDs and site IDs are
interchangeable.

## 2. Dependencies

P0-005.01 — Model loading
P0-005.02 — Independent model/state ownership
P0-005.03 — Required component name resolution

P0-005.04 consumes the validated name-to-ID mappings produced
by P0-005.03.

## 3. Ownership Rules

Each cache belongs to exactly one compiled MjModel.

The cache retains a reference to that model and rejects
requests involving a different model object.

Caches must not be shared between independently compiled
simulation instances.

Recompiling a model requires constructing a new cache.

## 4. MuJoCo Indexing Rules

Joint ID:
    Index into the model's joint table.

qpos address:
    model.jnt_qposadr[joint_id]

qvel address:
    model.jnt_dofadr[joint_id]

Actuator ID:
    Index used for data.ctrl and model actuator arrays.

Body ID:
    Index into the model's body arrays.

Site ID:
    Index into the model's site arrays.

None of these indexing spaces may be substituted for another.

## 5. Joint Coordinate Dimensions

| Joint type | qpos width | qvel width |
|---|---:|---:|
| Hinge | 1 | 1 |
| Slide | 1 | 1 |
| Ball | 4 | 3 |
| Free | 7 | 6 |

The differences arise from MuJoCo's joint-state
representations.

A ball joint uses a quaternion for orientation.
A free joint uses three position coordinates and a
four-element orientation quaternion.

Velocity dimensions use the corresponding independent
degrees of freedom.

## 6. Validation

Before caching, verify:

- Correct object type.
- Numeric identifier validity.
- Identifier within model bounds.
- ID-to-name round-trip consistency.
- Supported joint type.
- Valid qpos start address and width.
- Valid qvel start address and width.
- Actuator index within model.nu.
- Body index within model.nbody.
- Site index within model.nsite.

The resolver must reject stale, cross-wired or invalid
identifiers with informative errors.

## 7. Cache Immutability

The cache exposes read-only mappings.

Inputs are copied during construction so later mutations
of P0-005.03 dictionaries do not corrupt cached addresses.

The cache does not make the underlying MuJoCo model immutable.

Deliberate modification of compiled structural metadata
is outside the supported cache contract.

## 8. Panda-Specific Requirements

The composed Panda scene must validate:

- Seven named arm joints.
- Two named gripper finger joints.
- Eight named actuators.
- The ball's free joint.
- Required body IDs.
- Required site IDs.

For ball_free:

- qpos width must be 7.
- qvel width must be 6.

The cache must use actual compiled model addresses,
not hard-coded joint ordering assumptions.

## 9. Safety and Limitations

Successful caching does not prove:

- Correct actuator transmission.
- Correct control ranges.
- Correct joint limits.
- Safe robot motion.
- Valid end-effector placement.
- Ball reachability.
- Successful grasping.

These belong to later P0 milestones.

The cache is not a thread-safety mechanism.

## 10. Automated Test Execution

Tests belong in:

    tests/test_model_addresses.py

The existing runner must discover them automatically:

    python scripts/run_tests.py

The existing run_tests.py script does not need modification.

Tests must run headlessly and include adversarial
validation of stale, swapped and invalid identifiers.

## 11. Acceptance Criteria

P0-005.04 is complete when:

- Named identifiers are cached per model.
- qpos and qvel addresses are independently validated.
- All four MuJoCo joint types are tested.
- Wrong and stale identifiers are rejected.
- Read-only mapping behaviour is verified.
- Cross-model cache reuse is rejected.
- Real Panda integration tests pass.
- Existing regression tests remain passing.
- Test evidence is recorded in STATUS.md.

## 12. Change Control

All implementation requires explicit owner approval
under AGENTS.md.

Tests and simulations require separate execution approval.

Existing model, simulation and control scripts must not
be modified without explicit permission.
