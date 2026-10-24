
# P0-005.07 — Model Diagnostics

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Foundation
**Ticket:** P0-005.07
**Status:** Proposed — pending validation

## 1. Objective

Provide a deterministic, read-only diagnostic report for
the compiled Franka Panda pick-and-place simulation model.

The report identifies the loaded model, records its
structural dimensions and summarises the outcomes of
P0-005.03 through P0-005.06.

The diagnostic system is not responsible for loading,
repairing or stepping models.

## 2. Dependencies

- P0-005.01 — Explicit model loading.
- P0-005.02 — Model/data ownership.
- P0-005.03 — Named component validation.
- P0-005.04 — Model-specific address caching.
- P0-005.05 — Panda structural validation.
- P0-005.06 — Joint and actuator limit validation.

## 3. Public Interface

Module:

    robot_arm_sim.model_diagnostics

Functions:

    diagnose_model(model, source_path=None)
    format_model_diagnostics(report)

Data structures:

    DiagnosticCheck
    DiagnosticStatus
    ModelDiagnosticReport

## 4. Model Metadata

The report records:

- Compiled model name.
- Optional supplied source path.
- MuJoCo version.
- Number of bodies (nbody).
- Number of joints (njnt).
- Position-state size (nq).
- Velocity-state size (nv).
- Number of actuators (nu).
- Number of sites (nsite).
- Physics timestep in seconds.

All counts are read from the compiled MjModel.

The source path is metadata only. Diagnostic generation
does not open or validate the referenced file.

## 5. Validation Pipeline

### Named Components

Invoke validate_model_names(model).

Required joints, actuators, bodies and sites must be
present under their documented names.

### Address Cache

Invoke build_address_cache(model, resolved).

This stage depends on successful name resolution.

A failed name check causes address validation to
be marked BLOCKED rather than incorrectly marked PASS.

### Panda Structure

Invoke validate_panda_structure(model).

Verify the expected joint structure, actuator counts,
model dimensions and ball free-joint association.

### Limits

Invoke validate_panda_limits(model).

Verify finite joint bounds, actuator ranges and
home/default values according to the Panda contract.

### Physics Timestep

Verify that the compiled timestep is finite and
strictly greater than zero.

## 6. Result States

PASS:
    The validation stage executed successfully.

FAIL:
    The stage executed and rejected the model.

BLOCKED:
    A necessary prerequisite failed, preventing safe
    execution of this stage.

Overall PASS requires every required stage to be PASS.

Any FAIL or BLOCKED stage produces overall FAIL.

An empty set of checks must never count as success.

## 7. Failure Handling

Known validation errors are recorded with their messages.

Dependent checks must not execute if their prerequisites
have failed.

Independent checks should continue where safe.

Unexpected programming errors are propagated instead
of being silently converted to validation failures.

The report must not present incomplete work as success.

## 8. Read-Only Guarantees

Diagnostics must never:

- Load or modify source MJCF assets.
- Modify MjModel parameters.
- Modify MjData state.
- Set actuator controls.
- Change joint positions.
- Advance simulation time.
- Call mujoco.mj_step().
- Create an interactive viewer.
- Print automatically to stdout.

The report contains immutable metadata snapshots.

## 9. Output Formatting

The default formatter emits human-readable plain text.

Formatting must not rerun model validation or access
mutable state from the original model.

Callers decide whether to print, store or display
the resulting text.

Diagnostic output is not written automatically
to persistent files.

## 10. Testing

Test module:

    tests/test_model_diagnostics.py

Required coverage:

- Real composed Panda diagnostics.
- Actual compiled model dimensions.
- All required validation stages.
- Correct overall PASS/FAIL semantics.
- Missing component names.
- Blocked address-cache validation.
- Disabled and invalid joint limits.
- Invalid actuator ranges.
- Invalid physics timestep.
- Multiple independent validation failures.
- Unexpected validator exceptions.
- Deterministic repeated output.
- Report immutability and snapshot semantics.
- No model or state mutation.
- No physics stepping.
- Silent report generation.
- No revalidation during formatting.

## 11. Test Execution

The existing repository runner is used:

    ./scripts/run_tests.py

No changes to the runner are required.

The Panda model is loaded using the existing
ROBOT_ARM_PANDA_XML configuration.

A skipped real-model integration test does not satisfy
the ticket's acceptance criteria.

## 12. Limitations

A passing diagnostic report establishes that the selected
compiled scene satisfies the implemented P0-005 checks.

It does not establish:

- Correct physical actuator response.
- Collision-free motion.
- Gripper opening or closing success.
- End-effector reachability.
- Stable long-duration simulation.
- Deterministic lifecycle reset.
- Successful grasping or pick-and-place.
- Suitability for physical robot deployment.

These remain responsibilities of subsequent milestones.

## 13. Acceptance Criteria

P0-005.07 is complete when:

- All required metadata fields are reported.
- Existing validators are reused.
- Failure and blocked states are distinguished.
- Invalid models cannot produce overall PASS.
- Diagnostic output is deterministic.
- Successful and failed validations are read-only.
- Real Panda integration tests pass.
- Adversarial diagnostic tests pass.
- The full repository regression suite passes.
- Validation evidence is recorded in STATUS.md.

## 14. Change Control

All code and documentation changes require explicit
owner approval under AGENTS.md.

Test execution requires separate approval.

Existing robot assets, model-loading scripts,
simulation logic and test-runner scripts must not
be modified without authorisation.
