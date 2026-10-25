# P0 CHECKLIST — Simulation Foundation

**Project:** Robot Arm Pick-and-Place Simulator  
**Milestone:** P0  
**Status:** Not started  
**Scope:** Standalone, public, open-source MuJoCo simulation  
**Last updated:** 2026-10-22

## Objective

Create a reproducible, visible, stable MuJoCo scene containing a controllable robotic arm with gripper, a dynamic ball, a support table and a target. Provide a minimal programmatic simulation interface, observable physical state, interactive and headless execution, and automated validation. **Do not implement pick-and-place planning, learning, reinforcement-learning training, or any external operating-system integration in P0.**

**Definition of done:** A clean installation can load and view the scene; gravity acts on a free ball; the robot and gripper react to safe control commands; every required observation is finite and dimensionally consistent; reset restores the same starting state for a fixed configuration/seed; and the automated P0 tests pass.

## Conventions

- `[ ]` means incomplete. Tick only after running the associated check.
- Record completion evidence (test name, result, platform, dependency version and commit) in `STATUS.md`.
- Keep source, models and documentation in version control. Keep generated videos, checkpoints and large logs out of version control.
- Initial design values: physics timestep `0.002 s`, nominal controller period `0.04 s` (20 simulation steps), SI units. Treat as configurable defaults subject to validation.
- Prefer a redistributable 6–7 degree-of-freedom arm with an actuated parallel-jaw gripper. **Selection is an explicit P0 decision, not a presumed asset.**

---

## P0-001 — Repository and project scaffolding

**Goal:** A clean repository that can be cloned and installed.

- [x] P0-001.01 Choose and document the repository name and short public description.
- [x] P0-001.02 Create top-level `README.md`, `DESIGN.md`, `ROADMAP.md`, `STATUS.md` and `P0_CHECKLIST.md`.
- [x] P0-001.03 Choose a recognised open-source code licence (for example, MIT or Apache-2.0) and add its full text; confirm any third-party model asset conditions are compatible.
- [x] P0-001.04 Add `.gitignore` for `.venv/`, `__pycache__/`, `.pytest_cache/`, `.mypy_cache/`, generated logs, render outputs, video recordings and training artefacts.
- [x] P0-001.05 Define the following proposed layout; adjust deliberately if necessary:

```text
robot-arm-simulator/
├── README.md
├── DESIGN.md
├── ROADMAP.md
├── STATUS.md
├── P0_CHECKLIST.md
├── LICENSE
├── .gitignore
├── pyproject.toml
├── assets/
│   ├── robots/
│   └── scenes/
├── src/robot_arm_sim/
│   ├── __init__.py
│   ├── simulation.py
│   ├── observations.py
│   ├── controls.py
│   ├── model_ids.py
│   ├── viewer.py
│   ├── cli.py
│   ├── config.py
│   └── assets/
│       ├── default.toml
│       └── demo.xml
├── tests/
│   ├── test_model.py
│   ├── test_physics.py
│   ├── test_simulation.py
│   ├── test_observations.py
│   ├── test_controls.py
│   ├── test_reset.py
│   └── test_headless.py
└── docs/
    └── assets_and_licenses.md
```

- [x] P0-001.06 Make package imports work from an editable install rather than from ad-hoc path modifications.
- [x] P0-001.07 Confirm Git tracks project sources but ignores generated artefacts.

**Acceptance evidence:** `git status` includes only intended files after a test run; a clean environment can install `requirements.txt` and import the package.

## P0-002 — Runtime, dependencies and configuration

**Goal:** One documented and repeatable Python environment.

- [x] P0-002.01 Select a supported Python minor version and record it in `pyproject.toml` and `README.md`.
- [x] P0-002.02 Declare MuJoCo and pytest as dependencies (place development-only dependencies in an optional `dev` group).
- [x] P0-002.03 Lock exact direct/transitive dependency versions and artifact hashes in `uv.lock`; document the generated exact-version editable pip export in `requirements.txt` and update commands in `README.md`. Use `uv sync --locked` for hash-verified installs. Python/platform markers preserve compatible artifacts across the declared range.
- [x] P0-002.04 Implement a documented virtual-environment setup using `python -m venv .venv`, activation and `python -m pip install -r requirements.txt`.
- [x] P0-002.05 Record the verified environment: macOS 26.6.2 (Apple Silicon), Python 3.14.6 and MuJoCo 3.16.0. Other operating systems and Python releases remain unverified.
- [x] P0-002.06 Create a single packaged configuration source for model path, seed, physics timestep, controller period, named default joint pose, named default gripper commands, ball spawn and target position.
- [x] P0-002.07 Validate configuration values at startup: positive/integral timestep periods, finite numeric values, valid model/body/joint/actuator names and joint/actuator limits.
- [x] P0-002.08 Keep configuration independent of machine-specific absolute paths; resolve relative model paths from the TOML file.

**Acceptance evidence:** New environment installs, imports `mujoco`, loads the packaged configuration/model and rejects malformed configuration with an actionable error. Verified environment details are recorded above; other platforms/versions are not implied by this check.

## P0-003 — Arm and gripper model selection

**Goal:** Pick one real, redistributable MJCF model and understand its control semantics.

- [x] P0-003.01 Compare at least two available arm-with-gripper models for MJCF compatibility, documented actuation, asset licensing and contact geometry.
- [x] P0-003.02 Select one reference model and record its source URL, upstream revision/release, author/credit and redistribution licence.
- [x] P0-003.03 Verify meshes and textures have redistribution rights independently of any licence covering the MJCF definition.
- [x] P0-003.04 Record arm joint names, number of arm DoFs, joint type, range, axis and home position.
- [x] P0-003.05 Record gripper joint/finger names, coupled-joint behaviour (if any), opening width and permitted actuator range.
- [x] P0-003.06 Record actuator names, type (position, velocity or torque), control ranges and units; **do not equate actuator controls with joint angles unless verified**.
- [x] P0-003.07 Identify end-effector and finger geometry/site names for observations and later control.
- [x] P0-003.08 Identify robot collision geometry and contact-exclusion settings that may affect grasping.
- [x] P0-003.09 Decide whether to vendor assets with licence and attribution or fetch a pinned external package on install. Make initial setup reproducible and avoid silent latest-version downloads.
- [x] P0-003.10 Store this mapping in `docs/assets_and_licenses.md` or a dedicated robot model reference.

**Acceptance evidence:** Model loads and the documentation contains a complete, confirmed joint/actuator/site map and asset provenance.

## P0-004 — Construct the simulation scene

**Goal:** Create a coherent MJCF world with a robot, table, movable ball and target.

- [x] P0-004.01 Create a scene entrypoint that includes or attaches the selected robot model using valid MJCF model composition.
- [x] P0-004.02 Set scene units to metres, kilograms and seconds; verify the gravity vector corresponds to Earth-like downward gravity. *(Evidence: `tests/test_scene_units.py` — gravity `(0, 0, -9.81)` in demo and robot-composed scenes; 0.1 s free-fall matches ½gt² within 5 %.)*
- [x] P0-004.03 Define a fixed support table with position, dimensions and collision geometry. *(Evidence: `assets/demo.xml` — fixed 0.90 × 0.70 × 0.05 m tabletop, centre `(0, 0, 0.475)`, top z=0.50 m; four colliding 0.06 × 0.06 × 0.45 m box legs at x=±0.39, y=±0.29 m. `tests/test_table.py` verifies fixed support, ball contact with every leg, and open space underneath.)*
- [x] P0-004.04 Define a floor/ground plane suitable for visual debugging and physical contact. *(Evidence: `assets/demo.xml` — fixed +Z plane at z=0, procedural checker material, 4 × 4 m visible patch and infinite collision surface, explicit friction/contact masks. `tests/test_floor.py` verifies ground orientation, material assignment, contact boundaries and settled drops inside/outside the visible patch. Interactive appearance remains unverified.)*
- [x] P0-004.05 Position the robot base rigidly; ensure arm sweep and table surfaces do not unintentionally interpenetrate at reset. *(Evidence: `docs/robot_placement.md`, `tests/test_robot_placement.py` — fixed pedestal/base at `(-0.65, 0, 0.50)` m; actual Panda home has 0.12844 m minimum table clearance; 84 bounded sweep samples and 1,000 physics steps passed. Build/CLI reject support penetration, including configured reset overrides. Arbitrary full-workspace motion is not certified.)*
- [x] P0-004.06 Create a sphere with explicit radius, mass/density, inertia (computed or physically consistent), material/friction and contact participation. *(Evidence: `docs/ball_physics.md`, `tests/test_ball.py` — 0.04 m radius, 0.1 kg mass, computed uniform-sphere inertia 0.000064 kg m² per axis; named red material, explicit friction/contact masks. Compiled properties, force/torque response on all axes and resolved table friction pass in standalone and composed scenes.)*
- [x] P0-004.07 Give the ball a **free joint** so gravity, momentum and collisions govern motion. *(Evidence: `ball_free` in `assets/demo.xml`; `tests/test_ball.py` checks six unactuated velocity DoFs, zero damping/friction loss/armature, ballistic motion for four launch conditions, conserved spin on all axes and contact-driven momentum changes at both tabletop sides. All cases run standalone and with a synthetic fixed robot.)*
- [x] P0-004.08 Place the ball initially above or on the table without model-geometry overlap that causes explosive contact correction. *(Evidence: `docs/spawn_and_target.md`, `tests/test_spawn_and_target.py` — default spawn has 0.08 m clearance; overlap checks run at composition and after configuration, rejecting table/leg/floor/robot penetration. Exact resting contact is allowed; default drop remains finite and settles with peak speed below 1.5 m/s.)*
- [x] P0-004.09 Add a visually identifiable goal marker (e.g. translucent disc/site) that cannot inadvertently collide with or constrain the ball. *(Evidence: translucent green `target_marker` cylinder site, radius 0.06 m, fixed body at `(0.25, 0, 0.502)` m; configuration movement and three 1,000-step trajectory comparisons against a marker-free scene pass in `tests/test_spawn_and_target.py`. Interactive appearance remains unverified.)*
- [x] P0-004.10 Add fixed camera(s), lights and sufficiently distinct arm/ball/target colours; ensure legibility in the interactive viewer. *(Evidence: fixed cameras/lights/materials and framing tests; overview rendered and visually inspected on 2026-10-23, showing full robot/table and distinct ball/target. Standard Python interactive viewer launched without errors; user interaction remains untested. See `docs/scene_settings.md` and local ignored `renders/scene-overview.png`.)*
- [x] P0-004.11 Select physics options, solver settings and timestep explicitly; preserve rationale in comments or documentation. *(Evidence: `assets/demo.xml`, `docs/scene_settings.md` — 0.002 s, implicitfast, Newton, elliptic cones, 100 iterations, tolerance 1e-8, explicit soft-contact defaults; configuration/composition tests and full physics suite pass without prior integrator warnings.)*
- [x] P0-004.12 Confirm ground, table, sphere, gripper and finger collision groups/filters allow the intended contacts. *(Evidence: `tests/test_contact_filters.py` uses actual Panda meshes/pads for ball and support collision probes, checks scene-mask compatibility, disabled visual meshes and upstream base exclusion. Existing support/ball tests remain passing. This verifies participation, not grasp success.)*

**Acceptance evidence:** Scene compiles without invalid references; all objects are visible, physically located and correctly supported; the sphere is not welded, kinematic or artificially attached.

## P0-005 — Model loading and structural validation

**Goal:** Fail early when an asset or MJCF definition is broken.

- [x] P0-005.01 Implement an explicit model loader with useful errors for missing files and invalid XML. *(Evidence: `src/robot_arm_sim/model_loader.py`, `tests/test_model_loader.py`, `docs/model_loading.md` — validated successful MuJoCo model loading, missing files, invalid paths, malformed XML/MJCF, missing dependencies and model compilation errors. Tests also verify exception chaining, independent MjModel/MjData allocation, preservation of source files and no unintended physics stepping. P0-005.01 tests passed; model-loader integration into existing scene composition remains outside this task.)*

- [x] P0-005.02 Construct and own a fresh `mujoco.MjModel` and `mujoco.MjData` for each independent simulation instance (or document a safe explicit ownership model).

  **Evidence:** `src/robot_arm_sim/model_instance.py`, `tests/test_model_instance.py`, `docs/model_ownership.md`.

  Implemented explicit simulation ownership through `SimulationInstance`, with independent model/state allocation using `from_model_file()` and `from_robot_scene()`. Restricted direct construction, ordinary ownership-reference reassignment and deletion, copying, deep copying and serialisation.

  **Validation:** All reported P0-005.02 tests passed, including:

  - Independent `MjModel` and `MjData` identities and memory buffers.
  - Isolation of joint positions, velocities, actuator commands and simulation time.
  - Isolation of mutable compiled model parameters.
  - Repeated creation of 32 independent simulation instances.
  - Rejection of direct construction, reassignment, copying and pickling.
  - Preservation of surviving instances after garbage collection.
  - Correct propagation of injected loading and scene-construction failures.
  - Independent physics progression under repeated and interleaved stepping.
  - Preservation of source MJCF files.
  - Franka Panda scene integration and state-isolation tests.

  **Result:** PASS — all tests reported successful.

  **Limitations:** Ownership is an application-level contract, not a security boundary. Underlying MuJoCo objects remain mutable. Concurrent mutation of a shared instance, simulation lifecycle management, reset, robot reachability and grasp validation are outside P0-005.02.

  **Status:** COMPLETE.


- [x] P0-005.03 Resolve all required joints, actuators, sites and bodies from **names**, and validate their existence before stepping.

  **Evidence:** `src/robot_arm_sim/model_ids.py`, `tests/test_model_ids.py`, `docs/model_name_validation.md`, `scripts/run_tests.py`.

  Implemented centralised, read-only MuJoCo component resolution using `mujoco.mj_name2id()` with explicit object-type validation. Required Franka Panda arm joints, finger joints, actuators, gripper actuator, robot bodies, ball, target and scene sites are resolved by name against the compiled model.

  **Validation coverage:**
  - Correct resolution of named joints, actuators, bodies and sites.
  - Rejection of missing, malformed and incorrectly categorised components.
  - Case-sensitive matching and rejection of invalid whitespace.
  - Detection of duplicate names and empty validation schemas.
  - Aggregated diagnostic reporting for multiple missing components.
  - Independent resolution across differently ordered compiled models.
  - Rejection of invalid model types and unsupported object categories.
  - Verification that failed validation does not return partially resolved results.
  - Confirmation that name resolution does not mutate model parameters or simulation state.
  - Confirmation that validation does not advance simulation physics.
  - Regression coverage for model loading, ownership and existing simulation functionality.

  **Test execution:**
  - Runner: `python scripts/run_tests.py`
  - Result: **307 passed, 0 failed** (user-reported).
  - Full repository regression suite completed successfully.

  **Limitations:** Successful name resolution confirms component existence, not joint types, state-array addresses, actuator transmission semantics, control ranges or physical reachability. These are addressed in P0-005.04 through P0-005.06 and subsequent control-validation tickets.

  **Real-model validation:** The Franka Emika Panda MJCF
  was loaded from the local `.cache/robot_models/franka_emika_panda/` directory through the existing test runner.
  
  **Regression result:** 307 tests passed, 0 failed.
  
  **Asset handling:** The locally cached Panda model and its supporting assets were used without modification.

  **Status:** COMPLETE — P0-005.03 accepted based on reported test results.


- [x] P0-005.04 Cache validated model IDs without assuming joint IDs, `qpos` indices and actuator indices are interchangeable.

  **Evidence:** `src/robot_arm_sim/model_addresses.py`, `tests/test_model_addresses.py`, `docs/model_address_caching.md`, `scripts/run_tests.py`.

  Implemented model-specific, read-only address caching for MuJoCo joints, actuators, bodies and sites. Cached identifiers are validated against their originating compiled `MjModel`, with joint position and velocity addresses resolved independently through `jnt_qposadr` and `jnt_dofadr`.

  **Validation coverage:**
  - Correct address and dimensional mapping for hinge, slide, ball and free joints.
  - Verification of 7 qpos and 6 qvel coordinates for free joints.
  - Validation of address bounds and complete joint-state slices.
  - Rejection of negative, invalid, swapped and incorrectly named identifiers.
  - Rejection of stale caches and cross-model cache reuse.
  - Validation of actuator control indices independently from joint IDs.
  - Detection of invalid body and site identifiers.
  - Read-only cache mappings and isolation from subsequent input mutations.
  - Confirmation that cache construction does not modify model parameters or simulation state.
  - Real Franka Panda integration validating arm joints, finger joints, all eight actuators and ball free-joint dimensions.
  - Independent cache ownership across multiple Panda simulation instances.
  - Full regression validation of previous P0 milestones.

  **Execution environment:**
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - pytest: 9.1.1
  - pluggy: 1.7.0
  - Runner: `./scripts/run_tests.py`
  - Result: **335 passed, 0 failed, 0 skipped**
  - Execution time: 1.68 seconds

  **Limitations:** Cached identifiers remain valid only for their originating compiled model and unchanged model structure. Caching does not establish actuator transmission semantics, control limits, physical reachability or safe robot motion.

  **Status:** COMPLETE — validated.


- [x] P0-005.05 Validate expected arm joint count, gripper actuators and free-ball qpos/qvel dimensions.

  **Evidence:** `src/robot_arm_sim/model_structure.py`, `tests/test_model_structure.py`, `docs/model_structure_validation.md`.

  Implemented read-only structural validation of the complete Franka Emika Panda pick-and-place scene.

  **Validated structure:**
  - Seven named arm hinge joints.
  - Two named gripper slide joints.
  - Seven arm actuators and one gripper actuator.
  - Exactly 10 joints and 8 actuators.
  - Total model dimensions: nq=16 and nv=15.
  - Ball free joint with 7 qpos and 6 qvel coordinates.
  - Verification that ball_free belongs to the ball body.
  - Model state-array address bounds.

  **Adversarial validation:**
  - Rejection of missing and unexpected joints.
  - Rejection of missing and additional actuators.
  - Detection of incorrect joint types.
  - Detection of a free joint attached to the wrong body.
  - Rejection of additional joints even when all required names remain present.
  - Detection of hinge-to-slide substitution despite unchanged model dimensions.
  - Preservation of model state and simulation time after validation failures.

  **Real-model integration:**
  The composed Franka Panda scene passed structural validation using the locally configured robot model.

  **Regression results:**
  - Runner: `./scripts/run_tests.py`
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - pytest: 9.1.1
  - Tests collected: 359
  - Passed: 359
  - Failed: 0
  - Skipped: 0
  - Execution time: 1.75 seconds

  **Limitations:** Structural validation does not establish actuator transmission correctness, control-range safety, gripper coupling behaviour, robot reachability or successful manipulation.

  **Status:** COMPLETE — validated.


- [x] P0-005.06 Validate finite bounds and default values for joint ranges and actuators.

  **Evidence:** `src/robot_arm_sim/model_limits.py`,
  `tests/test_model_limits.py`,
  `docs/model_limits_validation.md`.

  Implemented read-only validation of Franka Panda joint
  limits, actuator control ranges and home-keyframe values.

  **Validation coverage:**
  - Seven arm joints and two gripper finger joints.
  - Eight named actuators.
  - Enabled joint and actuator limit enforcement.
  - Finite, strictly ordered joint and actuator bounds.
  - Home-keyframe joint positions within permitted ranges.
  - Home actuator controls within permitted ranges.
  - Explicit joint and actuator default-value validation.
  - Rejection of NaN, infinity and invalid numeric types.
  - Rejection of disabled or reversed limits.
  - Rejection of corrupted home joint positions.
  - Verification of gripper actuator8 control range [0, 255].
  - Confirmation that gripper commands are not interpreted
    as physical opening width in metres.
  - Independent compiled models for destructive tests.
  - No modification of source robot assets or shared fixtures.
  - Read-only validator behaviour.

  **Real-model validation:**
  The composed Franka Panda model passed the limits
  validation tests.

  **Regression test results:**
  - Runner: `./scripts/run_tests.py`
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - pytest: 9.1.1
  - Tests collected: 381
  - Passed: 381
  - Failed: 0
  - Skipped: 0
  - Execution time: 1.85 seconds

  **Limitations:**
  Numeric limit validation does not prove actuator
  transmission correctness, motion safety, control
  stability or end-effector reachability.

  **Status:** COMPLETE — validated.


- [x] P0-005.07 Provide an informative diagnostic report listing model name, number of bodies, joints, DoFs, actuators and timestep.

  **Evidence:** `src/robot_arm_sim/model_diagnostics.py`,
  `tests/test_model_diagnostics.py`,
  `docs/model_diagnostics.md`.

  Implemented a deterministic, read-only diagnostic reporting
  system for the compiled Franka Panda simulation model.

  **Diagnostic coverage:**
  - Compiled model name and optional source-path metadata.
  - MuJoCo version.
  - Body, joint, actuator and site counts.
  - Position dimensions (nq) and velocity DoFs (nv).
  - Compiled physics timestep.
  - Named component validation.
  - Model-specific address-cache validation.
  - Panda structural validation.
  - Joint and actuator limits validation.
  - Physics timestep validation.

  **Failure handling:**
  - Distinguishes PASS, FAIL and BLOCKED states.
  - Rejects overall success when any required check fails.
  - Detects invalid joint limits and actuator ranges.
  - Detects invalid or non-finite physics timesteps.
  - Aggregates independent validation failures.
  - Preserves actionable error messages.
  - Propagates unexpected programming errors.
  - Never interprets missing validation results as success.

  **Report integrity:**
  - Immutable report snapshots.
  - Deterministic repeated diagnostic results.
  - Human-readable text formatting.
  - No automatic printing or file modification.
  - No revalidation during formatting.
  - No simulation physics stepping.
  - No mutation of model parameters or simulation state.

  **Real-model validation:**
  The complete composed Franka Emika Panda scene passed
  all required diagnostic stages.

  **Regression results:**
  - Command: `./scripts/run_tests.py`
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - pytest: 9.1.1
  - Tests collected: 416
  - Passed: 416
  - Failed: 0
  - Skipped: 0
  - Execution time: 2.01 seconds

  **Limitations:**
  Model diagnostics establish structural and numerical
  validity under the implemented checks. They do not
  demonstrate successful actuator motion, safe trajectories,
  gripper operation or end-effector reachability.

  **Status:** COMPLETE — validated.

**Acceptance evidence:** Good model loads; bad or incomplete models fail before a physics step with identifiable error messages.

## P0-006 — Simulation lifecycle API

**Goal:** Separate simulation mechanics from application scripts and rendering.


- [x] P0-006.01 Implement a Simulator wrapper with explicit
  `load()`, `reset()`, `step()` and `close()` lifecycle methods.

  **Evidence:**
  - `src/robot_arm_sim/simulator.py`
  - `tests/test_simulator_lifecycle.py`
  - `docs/simulator_lifecycle.md`

  Implemented the initial Simulator lifecycle wrapper using
  the existing SimulationInstance ownership infrastructure.

  **Validated functionality:**
  - Explicit NEW, LOADED and CLOSED lifecycle states.
  - Lazy model allocation through load().
  - Independent MjModel and MjData ownership per simulator.
  - Rejection of duplicate loading.
  - Rejection of loading after closure.
  - Rejection of model/data access before initialisation.
  - Rejection of model/data access after closure.
  - Safe closure before loading.
  - Idempotent close() behaviour.
  - Failed loading preserves the NEW state.
  - Independent simulations remain valid when another closes.
  - No unexpected physics stepping or time advancement.
  - No viewer or rendering dependency.
  - Documented external-reference lifetime semantics.
  - Rejection of invalid constructor arguments.

  **Deferred functionality:**
  reset() and step() currently raise NotImplementedError
  when the simulator is loaded. Their implementations are
  intentionally reserved for subsequent P0-006 tickets.

  **Regression evidence:**
  - Command: `./scripts/run_tests.py`
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - pytest: 9.1.1
  - Tests collected: 442
  - Tests passed: 442
  - Failed: 0
  - Skipped: 0
  - Execution time: 2.23 seconds

  **Status:** COMPLETE — lifecycle wrapper foundation validated.


- [x] P0-006.02 Make reset() restore robot qpos/qvel,
  gripper command, ball free-joint pose/velocity
  and target placement.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_reset.py
  - tests/test_simulator_lifecycle.py
  - docs/simulator_reset.md

  **Validated functionality:**
  - Captures an independent reset baseline on load().
  - Restores all robot joint positions and velocities.
  - Restores both gripper finger joint states.
  - Restores all eight actuator controls.
  - Restores ball translation and orientation quaternion.
  - Restores all six ball velocity coordinates.
  - Restores the target's fixed-body position.
  - Restores initial simulation time.
  - Preserves MjModel and MjData object identity.
  - Supports repeated, idempotent resets.
  - Prevents cross-instance state contamination.
  - Rejects reset before loading or after closure.
  - Rejects unsupported seeded reset requests.
  - Does not recompile the robot scene or step physics.
  - Releases the reset snapshot on close().

  **Regression results:**
  - Command: ./scripts/run_tests.py
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - pytest: 9.1.1
  - Tests collected: 462
  - Passed: 462
  - Failed: 0
  - Skipped: 0
  - Runtime: 2.54 seconds

  **Known limitations:**
  Reset currently restores primary model/data state only.
  Clearing applied forces, actuator activation and solver
  transients is deferred to P0-006.03.
  Refreshing derived kinematics and contact information
  is deferred to P0-006.04.

  **Status:** COMPLETE — validated.


- [x] P0-006.03 Clear stale applied forces, controls,
  warm-start artifacts and other appropriate transient state
  before producing reset observations.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_transients.py
  - docs/simulator_transient_reset.md

  **Implemented functionality:**
  - Uses mujoco.mj_resetData() to clear transient state.
  - Clears applied generalized forces (qfrc_applied).
  - Clears external body forces and torques (xfrc_applied).
  - Clears solver acceleration warm-start (qacc_warmstart).
  - Restores captured actuator activation state (act).
  - Restores initial actuator commands (ctrl).
  - Preserves robot, gripper and ball reset configuration.
  - Restores target placement and simulation time.
  - Retains the existing MjModel and MjData objects.
  - Rejects invalid reset requests before native reset.
  - Does not recompile the model or advance physics.
  - Supports repeated resets and independent instances.

  **Validation:**
  - User reports that the full test suite passed.
  - No failure details reported for P0-006.03.
  - Exact test count, versions and execution time
    should be copied from the latest test log.

  **Limitations:**
  Derived transforms and contact information are not
  yet refreshed after reset.

  **Status:** COMPLETE — user-reported tests passed.


- [x] P0-006.04 Recompute derived kinematics/contact data
  after state writes with the correct MuJoCo forward operation.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_forward.py
  - tests/test_simulator_transients.py
  - docs/simulator_forward_reset.md

  **Implementation:**
  Extended Simulator.reset() to invoke mujoco.mj_forward()
  after restoring the initial simulation state.

  The reset sequence now:
  1. Validates the simulator lifecycle and reset arguments.
  2. Validates the captured reset snapshot.
  3. Clears transient state using mujoco.mj_resetData().
  4. Restores qpos, qvel, ctrl and actuator activation.
  5. Restores the target body's initial position.
  6. Restores initial simulation time.
  7. Recomputes derived state using mujoco.mj_forward().

  **Validation coverage:**
  - Recalculation of body world positions (xpos).
  - Recalculation of body orientations (xquat).
  - Recalculation of site world positions (site_xpos).
  - Recalculation of collision geometry positions (geom_xpos).
  - Verification of ball free-joint/world-position consistency.
  - Verification of restored target marker position.
  - Removal of stale contact data after reset.
  - Verification of current contact geometry pairs.
  - Preservation of primary qpos, qvel and ctrl values.
  - Preservation of the initial simulation clock.
  - Preservation of cleared applied forces.
  - Exactly one explicit mj_forward() call per reset.
  - Verification that forward computation occurs after
    primary-state restoration.
  - No mj_step() calls or physics time advancement.
  - Preservation of MjModel and MjData object identity.
  - Deterministic derived state across repeated resets.
  - Rejection of invalid lifecycle operations.
  - Rejection of unsupported seeded resets.
  - Isolation between independent simulator instances.
  - Headless execution without viewer dependencies.

  **Regression test results:**
  - Command: ./scripts/run_tests.py
  - Platform: macOS (Darwin)
  - Python: 3.14.6
  - Tests collected: 502
  - Passed: 502
  - Failed: 0
  - Skipped: 0
  - Execution time: 3.03 seconds

  **Known limitations:**
  - Physics stepping is not yet implemented in Simulator.step().
  - Actuator control and movement validation remain separate.
  - Controller-period scheduling is not yet implemented.
  - Reset determinism across operating systems is not guaranteed.
  - Automated reaching, grasping and pick-and-place are outside
    the scope of this ticket.

  **Status:** COMPLETE — validated.


- [x] P0-006.05 Define whether a step is one physics step
  or one controller period; document and test time advance.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_stepping.py
  - tests/test_simulator_lifecycle.py
  - tests/test_simulator_reset.py
  - docs/simulator_stepping.md

  **Implementation:**
  - Defined one substep as exactly one mujoco.mj_step().
  - Implemented Simulator.step(control=None, n_substeps=1).
  - Supports multiple consecutive physics substeps.
  - Uses the compiled model's physics timestep.
  - Holds the supplied actuator command across substeps.
  - Preserves existing actuator controls when control=None.
  - Accepts full actuator vectors in compiled actuator order.
  - Does not interpret raw actuator controls as joint angles.
  - Validates actuator command shape, type and finite values.
  - Validates enabled actuator control ranges.
  - Rejects invalid substep counts and physics timesteps.
  - Rejects invalid commands before modifying controls
    or advancing simulation time.
  - Preserves MjModel and MjData object identity.
  - Supports headless physics stepping.
  - Preserves reset functionality following physics stepping.

  **Validation coverage:**
  - Single-step physics time advancement.
  - Multiple substep counts and cumulative time.
  - Configured timestep usage.
  - Ball movement under gravity.
  - Valid actuator command application.
  - Preservation of existing controls with control=None.
  - Regression test preventing NumPy conversion of None.
  - Rejection of malformed, nonfinite and out-of-range controls.
  - Verification of exactly N mujoco.mj_step() calls.
  - Constant control values throughout a substep batch.
  - Independent simulator instance isolation.
  - Reset after physical stepping.
  - Lifecycle guards and headless operation.

  **Regression results:**
  - Command: ./scripts/run_tests.py
  - Tests passed: 558
  - Failures: 0 (reported)
  - Result: PASS

  **Known limitations:**
  - Controller-period stepping is deferred to P0-006.06.
  - Explicit physics/controller counters are deferred to P0-006.07.
  - Full actuator control behaviour is deferred to P0-007.
  - No autonomous behaviour or movement planning is implemented.
  - Valid actuator commands do not guarantee safe trajectories.

  **Status:** COMPLETE — user-reported tests passed.

- [x] P0-006.06 Ensure controller-period stepping has an
  integer relationship to the physics timestep, or handle
  noninteger periods deliberately.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_controller_period.py
  - docs/simulator_controller_period.md
  - src/robot_arm_sim/config.py

  **Implementation:**
  - Added _controller_substep_count() to calculate the
    required number of physics steps per controller period.
  - Added Simulator.step_controller_period().
  - Uses the current compiled model physics timestep.
  - Requires a positive integer number of physics substeps.
  - Rejects materially noninteger timing relationships.
  - Permits small floating-point representation differences.
  - Reuses the existing Simulator.step() implementation.
  - Preserves actuator validation and control=None behaviour.
  - Holds the selected actuator command across all substeps.
  - Does not introduce controller or AI behaviour logic.
  - Does not replace MjModel or MjData.

  **Default timing configuration:**
  - Physics timestep: 0.002 seconds.
  - Controller period: 0.040 seconds.
  - Physics steps per controller period: 20.

  **Validation coverage:**
  - Compatible controller-period/timestep ratios.
  - Rejection of incompatible fractional ratios.
  - Rejection of periods shorter than one physics step.
  - Rejection of zero, negative and nonfinite periods.
  - Rejection of invalid argument types.
  - Floating-point compatibility tolerance.
  - Exactly 20 physics steps for the default period.
  - Correct simulation time advancement.
  - Constant actuator controls throughout each period.
  - Preservation of existing controls when control=None.
  - Validation before actuator writes or physics stepping.
  - Rejection of invalid actuator commands.
  - Accumulated time across multiple controller periods.
  - Mixed physics-step and controller-period execution.
  - Reset after controller-period stepping.
  - Independent simulator instance isolation.
  - Lifecycle guards.
  - Public Simulator method availability.

  **Regression results:**
  - Command: ./scripts/run_tests.py
  - Tests passed: 602
  - Failures: 0 (user reported)
  - Result: PASS

  **Known limitations:**
  - Physics and controller step counters remain P0-006.07.
  - No learned controller or behavioural policy is implemented.
  - No wall-clock real-time execution guarantee.
  - No inverse kinematics or autonomous manipulation.
  - Noninteger controller periods are deliberately rejected.

  **Status:** COMPLETE — user-reported tests passed.


- [x] P0-006.07 Track simulation time and physics/controller
  step counts.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_counters.py
  - docs/simulator_execution_counters.md

  **Implementation:**
  - Added read-only simulation_time property using MjData.time.
  - Added episode-local physics_step_count.
  - Added episode-local controller_step_count.
  - Physics counter increments after each successful
    mujoco.mj_step() integration.
  - Controller counter increments after each fully completed
    step_controller_period() call.
  - Direct physics stepping does not increment controller count.
  - Controller-period execution does not double-count physics.
  - Successful reset restores both counters to zero.
  - Failed resets preserve existing counter values.
  - Counter properties enforce simulator lifecycle state.
  - Independent simulator instances maintain separate counts.

  **Validation coverage:**
  - Zero counters following initial load.
  - Single and multiple physics integrations.
  - Single and repeated controller periods.
  - Mixed physics/controller stepping without double-counting.
  - Invalid actuator commands and step counts.
  - Invalid controller-period rejection.
  - Partial physics execution failure accounting.
  - Incomplete controller periods not counted.
  - Preservation of earlier successful controller periods.
  - Reset clearing both counters.
  - New episode accounting after reset.
  - Idempotent repeated resets.
  - Failed reset preserving counter values.
  - Authoritative MuJoCo simulation clock.
  - Timestep changes without counter corruption.
  - Independent simulator instance isolation.
  - Read-only property enforcement.
  - Lifecycle access restrictions.
  - Forward computation not counted as a physics step.

  **Default execution timing:**
  - Physics timestep: 0.002 seconds.
  - Controller period: 0.040 seconds.
  - Physics steps per controller period: 20.

  **Regression results:**
  - Command: ./scripts/run_tests.py
  - Tests passed: 640
  - Failures: 0
  - Result: PASS (user reported)

  **Known limitations:**
  - Counters track integrations through the Simulator API,
    not direct external calls to mujoco.mj_step().
  - Simulation time is not wall-clock execution time.
  - No real-time execution guarantees.
  - No autonomous movement or behaviour learning.
  - No controller policy or trajectory planning.

  **Status:** COMPLETE — validated.


- [x] P0-006.08 Guard against use before initialisation,
  after closure, invalid action shapes and illegal
  substep counts.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_guards.py
  - tests/test_simulator_lifecycle.py
  - tests/test_simulator_stepping.py
  - tests/test_simulator_controller_period.py
  - tests/test_simulator_counters.py
  - docs/simulator_guards.md

  **Implementation:**
  - Preserved explicit NEW, LOADED and CLOSED lifecycle guards.
  - Rejected stepping and reset operations before loading.
  - Rejected operations after simulator closure.
  - Prevented duplicate loading and reopening after closure.
  - Validated positive integer physics substep counts.
  - Rejected invalid actuator control dimensions and types.
  - Rejected NaN, infinity and out-of-range actuator values.
  - Validated existing data.ctrl values when control=None.
  - Rejected unrepresentable numeric control values.
  - Ensured validation occurs before actuator writes
    and physics integration.
  - Preserved controller-period validation from P0-006.06.
  - Preserved physics/controller accounting from P0-006.07.
  - Prevented invalid commands from initiating MuJoCo stepping.

  **Validation coverage:**
  - Lifecycle state enforcement.
  - Invalid operations before load and after close.
  - Duplicate load rejection.
  - Illegal substep counts and argument types.
  - Incorrect actuator vector dimensions.
  - Boolean, string and complex actuator commands.
  - Nonfinite and oversized control values.
  - Corrupted retained actuator commands.
  - Invalid controller-period commands.
  - Validation without simulation state mutation.
  - Preservation of qpos, qvel and actuator controls.
  - Preservation of simulation time on rejected requests.
  - Preservation of physics and controller counters.
  - Regression protection for control=None.
  - Valid raw and controller-period stepping.
  - Unsupported seeded reset rejection.

  **Regression results:**
  - Command: ./scripts/run_tests.py
  - Tests passed: 685
  - Failed: 0
  - Result: PASS (user reported)

  **Known limitations:**
  - Validation does not guarantee collision-free motion.
  - Direct external mutation of MuJoCo buffers remains possible.
  - Unexpected physics engine failures do not guarantee rollback.
  - Trajectory safety and autonomous control remain separate tasks.
  - Full headless-operation acceptance is covered by P0-006.09.

  **Status:** COMPLETE — validated.


- [x] P0-006.09 Verify headless simulation operation.

  **Evidence:**
  - src/robot_arm_sim/simulator.py
  - tests/test_simulator_headless.py
  - docs/simulator_headless.md

  **Implementation:**
  - Verified the Simulator lifecycle operates without
    creating an interactive MuJoCo viewer.
  - Verified model loading and reset without rendering.
  - Verified physics stepping without graphics output.
  - Verified controller-period stepping without rendering.
  - Preserved simulation time and execution counters.
  - Preserved model and data ownership across operations.
  - Confirmed access to derived physics state without
    camera rendering.
  - Confirmed headless contact-data availability.
  - Verified repeated reset and execution cycles.
  - Verified independent simulator instances.
  - Added fresh-subprocess headless verification.
  - No production simulator modifications required.

  **Validation coverage:**
  - Complete NEW -> LOADED -> CLOSED lifecycle.
  - Renderer construction prohibited during execution.
  - Ball movement under gravity.
  - Body, site and geometry positions.
  - Contact generation and inspection.
  - Physics step counting.
  - Controller-period counting.
  - Simulation clock advancement.
  - Reset state restoration.
  - Independent model/data instances.
  - Lifecycle error handling.
  - Subprocess execution without DISPLAY and
    WAYLAND_DISPLAY environment variables.
  - Regression testing of existing simulation features.

  **Regression results:**
  - Command: ./scripts/run_tests.py
  - Tests passed: 693
  - Failures: 0
  - Result: PASS (user reported)

  **Known limitations:**
  - Headless operation was validated on the tested host,
    not every operating system or deployment environment.
  - Offscreen image rendering was not tested.
  - Real-time execution guarantees were not established.
  - No autonomous robot behaviour was introduced.
  - Hardware deployment remains outside the scope.

  **Status:** COMPLETE — validated.


**Acceptance evidence:** Automated tests run `load → reset → step → reset → close`; simulation clock advances as specified; reset returns known state.


## P0-007 — Robot actuator control and movement

**Goal:** Establish a verified actuator-control interface,
demonstrate measured arm and gripper movement, and determine
whether the Panda can physically reach the ball.

**Prerequisites:**
- P0-003: Robot model and actuator identification.
- P0-005: Independent model and data ownership.
- P0-006: Simulator lifecycle and stepping API.

**Baseline:**
- 693 tests passed (user reported).
- No P0-007 implementation or validation is assumed complete.

### Existing requirements

- [ ] P0-007.01 Define a named actuator command vector and document shape, units and correspondence to MuJoCo `data.ctrl` entries.
- [ ] P0-007.02 Validate every command is finite; reject NaNs, infinities and shape mismatches.
- [ ] P0-007.03 Bound actuator input to each actuator's verified `ctrlrange`; define clamp-versus-error behaviour and test it.
- [ ] P0-007.04 Implement and test the default neutral/hold command appropriate to the selected model (zero effort is not necessarily a safe hold).
- [ ] P0-007.05 Implement a controlled single-joint movement test while other commanded joints hold position.
- [ ] P0-007.06 Observe the moved joint's measured `qpos` and `qvel`, not merely its requested command.
- [ ] P0-007.07 Implement a safe gripper-open command and measure the actual achieved finger separation/joint positions.
- [ ] P0-007.08 Implement a safe gripper-close command and measure achieved closure in an unobstructed initial configuration.
- [ ] P0-007.09 Ensure commands do not drive joints beyond model limits in the smoke test; log any unexpected constraint violation.
- [ ] P0-007.10 Document limits of P0 control: direct verified actuator commands only; Cartesian inverse kinematics and grasp sequences are not required yet.
- [ ] P0-007.11 Record the initial end-effector position and ball position in world coordinates.
- [ ] P0-007.12 Determine whether the ball is reachable within the Panda's joint limits.
- [ ] P0-007.13 Identify which joint movements bring the gripper toward the ball.
- [ ] P0-007.14 Verify that commanded actuator movements produce the expected joint motion.
- [ ] P0-007.15 Test a collision-safe configuration that positions the gripper near the ball.
- [ ] P0-007.16 Document the reachable workspace and any restrictions caused by the table or pedestal.

### Additional actuator requirements

- [ ] P0-007.17 Verify the compiled actuator-to-joint transmission mapping, control mode, gain and bias semantics for every arm and gripper actuator. Record whether each control represents position, velocity, effort or another model-specific quantity. Do not assume `data.ctrl` contains joint angles.

- [ ] P0-007.18 Define the named controller interface independently of the raw MuJoCo control-vector ordering. Verify that named commands are converted to exactly `model.nu` correctly ordered values, with no omitted or duplicated actuator entries.

- [ ] P0-007.19 Verify command preservation and rejection semantics. Invalid named commands must not partially update `data.ctrl`, advance simulation time or increment execution counters. Keep the P0-006 validation guarantees intact.

- [ ] P0-007.20 Measure single-joint command response. Record starting position, requested command, measured final `qpos`, peak `qvel`, direction of motion and elapsed simulated time. Define an appropriate model-specific tolerance instead of assuming exact tracking.

- [ ] P0-007.21 Verify the neutral/hold controller under gravity. Measure joint drift during a defined stationary interval and after completing a commanded movement. Distinguish position holding from applying zero actuator effort.

- [ ] P0-007.22 Verify gripper actuator behaviour against measured finger motion. Record each finger joint, movement direction, opening width, achieved closure and relevant actuator limits. Do not infer gripper width directly from an unverified actuator-control value.

### Additional reachability requirements

- [ ] P0-007.23 Define the end-effector reference used for reachability calculations. Record the actual site or body identifier, reference frame, and whether the reference represents the wrist, tool centre point or a point between the fingers.

- [ ] P0-007.24 Define the ball approach target geometrically. Account for the ball radius, gripper finger geometry, desired approach direction and clearance. Do not treat the ball centre as an automatically valid end-effector target.

- [ ] P0-007.25 Establish a repeatable reachability investigation from the reset configuration. Record initial joint positions, end-effector position, ball position and initial end-effector-to-target displacement in world coordinates.

- [ ] P0-007.26 Investigate the arm-extension limitation. Identify which joint configurations increase useful reach toward the ball, whether a joint limit is encountered, and whether control mapping or inadequate settling time explains any observed lack of extension.

- [ ] P0-007.27 Separate geometric and collision-free reachability. First assess whether a pose satisfies joint limits and target-position tolerance; then assess whether the arm can achieve an appropriate approach configuration without prohibited penetration of the robot, table or pedestal.

- [ ] P0-007.28 Verify any selected approach motion over intermediate simulation states. Record minimum clearance, joint-limit margins and unexpected contacts. A collision-free final pose alone is insufficient evidence of a safe movement path.

### Additional repeatability and evidence requirements

- [ ] P0-007.29 Repeat representative arm and gripper movement trials after reset. Confirm consistent movement direction, measured joint response and end-effector displacement within documented tolerances. Do not claim full deterministic behaviour without appropriate evidence.

- [ ] P0-007.30 Produce an actuator and reachability validation report containing model revision, actuator mapping, movement commands, measured joint state, world-frame end-effector and ball positions, reachability result, observed limitations and test evidence.

### Acceptance evidence

P0-007 is complete when:

1. A named actuator-control interface is implemented and validated.
2. Raw actuator control semantics are correctly documented.
3. Invalid commands are rejected without partial state mutation.
4. At least one arm joint moves under an explicit valid command.
5. Joint movement is measured from actual MuJoCo state.
6. Other commanded joints demonstrate documented hold behaviour.
7. Gripper opening and closing are physically measured.
8. Arm and gripper joint limits are respected in representative tests.
9. End-effector and ball world positions are correctly identified.
10. The ball's geometric reachability is established.
11. Collision-free approach feasibility is investigated.
12. The original arm-extension problem is explained by evidence.
13. Representative movements are repeatable after reset.
14. Relevant test and regression suites pass.
15. Known limitations are documented without overstating capability.

### Scope limitations

P0-007 does not require:

- Autonomous pick-and-place.
- A trained movement policy.
- MAHQ model integration.
- Full Cartesian inverse-kinematics control.
- Trajectory optimisation.
- General-purpose motion planning.
- A verified physical grasp of the ball.
- Real hardware deployment.

A minimal offline kinematic feasibility investigation is
permitted as a proposed diagnostic, subject to approval.

Any changes to robot geometry, joint definitions,
actuator mappings, collision properties or scene placement
require separate, explicit approval under AGENTS.md.

Do not modify scripts/run_tests.py.

Source changes, tests and compilation require their
respective approvals before execution.


## P0-008 — Observation schema and measurements

**Goal:** Produce typed, dimensionally stable and numerically valid physical observations.

- [ ] P0-008.01 Specify a named observation schema with array shapes, units, coordinate frames and source MuJoCo fields.
- [ ] P0-008.02 Read arm joint positions and velocities using the correct address arrays for the selected joint types.
- [ ] P0-008.03 Read gripper actuator/joint state and compute meaningful opening width if the model permits it.
- [ ] P0-008.04 Read ball world position, orientation and linear/angular velocity using the correct MuJoCo state representation.
- [ ] P0-008.05 Read target position in the same world frame as the ball position.
- [ ] P0-008.06 Read end-effector world position from a validated site or body reference.
- [ ] P0-008.07 Expose simulation time and optionally the latest applied action for debugging.
- [ ] P0-008.08 Include contact count and/or selected contact observations as diagnostics; document that raw contact count is **not** proof of a stable grasp.
- [ ] P0-008.09 Verify all observation arrays have expected dimensions, types and finite values after reset and after stepping.
- [ ] P0-008.10 Return observations as snapshots/copies or document view lifetime to prevent mutable `MjData` buffers corrupting stored histories.
- [ ] P0-008.11 Test coordinates: moving the ball to a known world position must change its reported position consistently.

**Acceptance evidence:** Tests validate each observation field, including finite values, correct shape and frame consistency.

## P0-009 — Physics, stability and contact tests

**Goal:** Show that the simulation obeys basic physical expectations instead of merely animating objects.

- [ ] P0-009.01 Test model compilation and successful initial `mj_forward`/equivalent before stepping.
- [ ] P0-009.02 Record ball centre, table height and radius; confirm the initial placement is non-interpenetrating within a defined tolerance.
- [ ] P0-009.03 Starting the ball above support, step physics and confirm its vertical position decreases under gravity.
- [ ] P0-009.04 Continue stepping and confirm the ball settles close to expected support height without passing through the table.
- [ ] P0-009.05 Confirm genuine sphere/table contact is generated in the appropriate resting scenario.
- [ ] P0-009.06 Confirm ball movement is caused by simulation physics, not by writing its pose on every frame.
- [ ] P0-009.07 Run an idle simulation for a documented simulated duration (suggest 10 seconds) and check all states remain finite.
- [ ] P0-009.08 Check robot joints remain within model joint limits (allow a documented tiny numerical tolerance).
- [ ] P0-009.09 Check that object speeds, penetrations and contacts do not exhibit unexplained blowups under normal reset/default control.
- [ ] P0-009.10 Verify target marker does not exert contact forces or alter the ball trajectory.
- [ ] P0-009.11 Document any contact parameters or solver tuning needed, distinguishing physical plausibility from convenience values.

**Acceptance evidence:** Gravity, settling, resting contact, idle stability and joint-limit tests pass; no artificial ball attachment is used.

## P0-010 — Deterministic reset and seeded runs

**Goal:** Reproduce initial conditions and repeatable runs under the same supported stack.

- [ ] P0-010.01 Define a seed-driven random generator owned by the simulation/task setup, not an uncontrolled global RNG.
- [ ] P0-010.02 With default P0 fixed placements, reset twice with the same seed and verify identical initial state within defined numeric tolerance.
- [ ] P0-010.03 After moving joints or ball, reset and verify **all** qpos/qvel/controls and task markers return to starting state.
- [ ] P0-010.04 Ensure any physics settling step after reset is either deterministic and documented or separately controlled; report time-zero semantics.
- [ ] P0-010.05 Run the same prescribed command sequence twice from identical reset conditions; compare final state with documented tolerances.
- [ ] P0-010.06 Document platform/version constraints on determinism; do not require cross-CPU or cross-OS bit-for-bit equality.
- [ ] P0-010.07 Reserve seeded random ball/target placements for P2 unless a minimal seed implementation is required by the API.

**Acceptance evidence:** Same platform and versions, same config/seed and controls → equivalent initial and final observations within stated tolerance.

## P0-011 — Viewer and manual debugging

**Goal:** Inspect the scene and actuator behaviour without affecting headless correctness.

- [ ] P0-011.01 Add a CLI command to launch an interactive MuJoCo viewer using the supported viewer API for the pinned version.
- [ ] P0-011.02 Document any OS-specific constraints for interactive rendering (including macOS launch requirements, if applicable).
- [ ] P0-011.03 Verify floor, table, arm, fingers, ball and target are visible and visually distinct.
- [ ] P0-011.04 Frame the scene with a camera angle that makes the gripper and ball observable.
- [ ] P0-011.05 Demonstrate an idle run, one safe joint movement and gripper open/close from the CLI or a clearly documented demo script.
- [ ] P0-011.06 Provide a way to pause/inspect the scene, reset it and exit cleanly using supported viewer controls or a simple application UI.
- [ ] P0-011.07 Keep viewer creation and rendering outside core simulation tests so CI can run without a desktop session.
- [ ] P0-011.08 Record a short screenshot or clip if useful to project documentation, but do not substitute visual evidence for automated checks.

**Acceptance evidence:** A developer can open the scene, verify intended motion visually, reset and exit without hanging or crashing.

## P0-012 — Headless smoke test and automated test suite

**Goal:** Prove the core simulator is testable without a graphical desktop.

- [ ] P0-012.01 Configure pytest discovery and clear commands in `README.md`.
- [ ] P0-012.02 Add `test_model.py` covering scene load, named references, joint counts, required actuators and shape checks.
- [ ] P0-012.03 Add `test_physics.py` covering gravity, support contact, stable rest, marker noninteraction and finite states.
- [ ] P0-012.04 Add `test_simulation.py` covering load/reset/step/close, clock advancement and bad lifecycle calls.
- [ ] P0-012.05 Add `test_observations.py` covering shape, units documented in the schema, finiteness and independence from live mutable data.
- [ ] P0-012.06 Add `test_controls.py` covering command ranges, invalid inputs, joint response and gripper open/close.
- [ ] P0-012.07 Add `test_reset.py` covering full-state reset and repeatability under a declared test configuration.
- [ ] P0-012.08 Add `test_headless.py` demonstrating several simulation steps and valid state with **no** viewer opened.
- [ ] P0-012.09 Define practical tolerances and simulation-duration bounds in tests instead of relying on exact equality for evolving floating-point states.
- [ ] P0-012.10 Add an automated test command (e.g. `python -m pytest -q`) and ensure a non-zero exit on failures.
- [ ] P0-012.11 Keep heavy visual/rendering tests optional and separate from required P0 CI tests.
- [ ] P0-012.12 Confirm tests can run independently without depending on execution order or stale global simulator state.

**Acceptance evidence:** All required tests pass locally and in the chosen headless test environment, with recorded commands and results.

## P0-013 — Release hygiene and documentation

**Goal:** Make P0 understandable and reproducible by an outside contributor.

- [ ] P0-013.01 Update `README.md` with tested installation, CLI launch, viewer, headless test and known limitations.
- [ ] P0-013.02 Update `DESIGN.md` to match the **actual** model, control semantics, coordinate frames and observation schema.
- [ ] P0-013.03 Update `ROADMAP.md` only for deliberate milestone changes; leave pick-and-place and training for later milestones.
- [ ] P0-013.04 Update `STATUS.md` with actual pass/fail results, test count, versions, unresolved issues and next step.
- [ ] P0-013.05 Verify the licence and third-party model/mesh/texture attributions are present and accurate.
- [ ] P0-013.06 Confirm no private file paths, secrets or unlicensed datasets/assets are committed.
- [ ] P0-013.07 Run clean-install and smoke-test instructions from a fresh environment or clean checkout.
- [ ] P0-013.08 Tag P0 complete only when all exit-gate checks below are supported by evidence.

**Acceptance evidence:** A second developer can follow the README and reproduce headless tests and the working viewer on a documented supported platform.

---

## P0 exit-gate verification matrix

| Gate | Required demonstration | Suggested evidence |
|---|---|---|
| G01 — Clean setup | Package installs and imports on documented Python/platform | Install log and version list |
| G02 — Legal assets | Code and robot assets can be distributed under their stated licences | `LICENSE`, asset manifest |
| G03 — Model validity | Scene compiles and all required IDs exist | `test_model.py` |
| G04 — Visible scene | Robot, table, ball and target visible | Viewer manual check |
| G05 — Physical ball | Ball falls, contacts table and settles without pose scripting | `test_physics.py` |
| G06 — Safe motion | Arm joint responds and constraints are respected | `test_controls.py` |
| G07 — Gripper motion | Gripper opens and closes using actual actuators | `test_controls.py`, viewer |
| G08 — Observability | Required observation arrays have correct shape and finite values | `test_observations.py` |
| G09 — Reset | Full state resets reproducibly for fixed configuration/seed | `test_reset.py` |
| G10 — Headless | Simulation runs without display/viewer | `test_headless.py` |
| G11 — Stability | Idle simulation has no nonfinite or runaway state | `test_physics.py` |
| G12 — Documented | README, design, status and licences match tested code | Documentation review |

## Recommended execution order

1. **Bootstrap:** P0-001 → P0-002.
2. **Select real robot assets:** P0-003 (before implementing model-dependent interfaces).
3. **Build and compile a scene:** P0-004 → P0-005.
4. **Core simulator:** P0-006, followed by P0-008 observations.
5. **Actuators:** P0-007.
6. **Physical validity:** P0-009 → P0-010.
7. **Usability and automated tests:** P0-011 → P0-012.
8. **Documentation and release gate:** P0-013 plus G01–G12.

## Explicit exclusions (planned for later milestones)

- Automated reaching, inverse kinematics, waypoint planning and grasp sequencing.
- A successful pick, lift, transport and release.
- Training data generation, Gymnasium API, rewards or reinforcement learning.
- Generalisation over random object placements or multiple robot models.
- Integration into any separate execution platform or operating system.

## P0 close-out record (fill in when complete)

- Commit or release tag: **TBD**
- Operating system / architecture: **TBD**
- Python version: **TBD**
- MuJoCo version: **TBD**
- Robot model and licence: **TBD**
- Test command: `python -m pytest -q`
- Tests passed / failed / skipped: **TBD**
- Physics configuration and numerical tolerances: **TBD**
- Viewer launch command: **TBD**
- Open issues: **TBD**
- Decision: **NOT READY — implementation has not started**
