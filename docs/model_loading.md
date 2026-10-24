# P0-005.01 — MuJoCo Model Loading

**Project:** Robot Arm Pick-and-Place Simulator  
**Milestone:** P0 — Simulation Foundation  
**Component:** Model Loader  
**Status:** Proposed — awaiting approval and validation

## 1. Purpose

The model loader provides a consistent interface for locating, reading and compiling MuJoCo MJCF models.

The loader is responsible for detecting invalid or unavailable model inputs and returning diagnostic information that identifies the source of failure.

It must not alter existing models, download assets, modify simulation scripts or execute physics steps.

## 2. Design

The model-loading interface is defined in:

`src/robot_arm_sim/model_loader.py`

The public function is:

`load_model(path: str | Path) -> tuple[mujoco.MjModel, mujoco.MjData]`

It returns a compiled MuJoCo model and a fresh simulation state.

### Processing sequence

1. Validate the path argument.
2. Expand and resolve the filesystem path.
3. Confirm the path identifies a readable regular file.
4. Delegate MJCF parsing and referenced-asset loading to MuJoCo.
5. Compile the model.
6. Allocate fresh `MjData`.
7. Return the model and state.

Loading does not perform a physics step.

## 3. Error handling

All expected model-loading failures are reported through `ModelLoadError`, which derives from `ValueError`.

| Failure | Expected response |
|---|---|
| Missing model | Raise `ModelLoadError` with the affected path |
| Directory supplied | Reject as a non-file |
| Unreadable file | Report access failure |
| Malformed XML | Preserve MuJoCo parser diagnostic |
| Invalid MJCF element | Preserve MuJoCo compilation diagnostic |
| Invalid model reference | Report the referenced identifier where available |
| Missing mesh or texture | Preserve dependency-loading diagnostic |
| Missing included XML | Preserve dependency-loading diagnostic |
| Invalid argument type | Raise `TypeError` |

The original exception is retained through exception chaining.

## 4. Asset handling

Robot models may reference external meshes, textures and additional MJCF files.

The loader relies on MuJoCo's normal asset-resolution behaviour.

It must not automatically download missing assets, change model paths, substitute alternative assets or modify XML to resolve errors.

For the Franka Panda, the expected asset source and revision are documented separately.

The existing Panda model has previously been validated against MuJoCo Menagerie revision:

`cda5236f98383f6fbaa4820c6b18dba892cd087f`

That revision was used in earlier project validation. The loader does not independently verify its authenticity or enforce an asset checksum.

## 5. State ownership

Each successful call creates a fresh `MjModel` and `MjData`.

State from independent loading operations is not reused.

This provides a foundation for isolated simulation instances and repeatable validation.

Simulation reset behaviour remains a separate requirement under P0-006.

## 6. Integration with the current project

The existing project already uses two loading mechanisms:

- `simulation.py` uses `MjSpec` to compose the Panda with the demo scene.
- `cli.py` loads the standalone demonstration using `MjModel.from_xml_path()`.

P0-005.01 introduces a common error-handling interface for standalone model files.

It does not immediately replace scene composition or existing CLI loading paths.

Future integration must be proposed separately and must preserve the established scene, placement checks and working behaviour.

## 7. Testing strategy

The loader is tested using small, isolated MJCF fixtures.

Required test categories:

- Successful loading.
- Invalid paths and types.
- Filesystem errors.
- Malformed XML.
- Invalid MuJoCo definitions.
- Missing referenced assets.
- Preservation of original exceptions.
- No modification of model files.
- Independent model/state allocation.
- No unintended physics execution.

Tests must run without downloading robot assets.

Optional integration testing may use the pinned Panda model from local storage.

## 8. Known limitations

Successful model compilation does not establish:

- Correct joint-to-actuator mapping.
- Valid gripper behaviour.
- Physical reachability.
- Collision-safe robot movement.
- Correct control units.
- Complete observation mappings.
- Deterministic simulation across platforms.
- Licence compliance or model provenance.

These requirements are addressed in subsequent P0 tasks.

Filesystem validation is also subject to normal operating-system race conditions: a file may change between path validation and MuJoCo loading. The loader is not intended as a security boundary for untrusted model assets.

## 9. Acceptance criteria

P0-005.01 may be marked complete when:

- The loader compiles a valid model.
- Invalid input classes fail with actionable diagnostics.
- Referenced-asset failures are identified.
- Fresh model/state allocation is demonstrated.
- Loading does not alter the source model.
- Unit tests pass in the declared supported environment.
- Existing simulation behaviour remains unaffected.
- Test results, environment details and relevant commit are recorded in `STATUS.md`.

P0-005.01 does not require implementation of the subsequent structural-validation tasks.

## 10. Change control

Any implementation, integration, modification, compilation or test execution is subject to `AGENTS.md`.

No code or existing scripts may be changed without explicit prior approval. Compilation and tests require separate authorisation.