# STATUS — Robot Arm Pick-and-Place Simulator

**Updated:** 2026-10-22  
**Current phase:** P0 — Simulation foundation  
**Overall status:** Editable package installation and a minimal MuJoCo physics demo are implemented; robot control and pick-and-place are not started.

## Verified today

- [x] Defined a single-arm, single-ball pick-and-place project scope.
- [x] Drafted `README.md`, `DESIGN.md`, `ROADMAP.md` and `STATUS.md`.
- [x] Added the MIT project licence.
- [x] Selected MuJoCo as the intended physics engine and Python as the initial implementation language.
- [x] Added editable-install packaging metadata and a requirements file.
- [x] Added an exact, hash-checked dependency lock and documented its regeneration strategy; retained an exact-version pip export for editable installs.
- [x] Added a packaged table-and-ball scene with GUI and headless run options.
- [x] Installed the project in a clean Python 3.14 virtual environment and verified the import resolves to `src/robot_arm_sim`.
- [x] Ran the headless demo for 1,000 steps; the ball settled at 0.54 m on the table.
- [x] Added a packaged TOML configuration with relative model paths and runtime checks for timing, scene entities, joint limits and actuator ranges.
- [x] Recorded the successful validation environment: macOS 26.6.2 arm64, Python 3.14.6, MuJoCo 3.16.0.
- [x] Added and passed 12 focused configuration tests.
- [x] Verified that at least one automated test passes (12 configuration tests).
- [x] Defined a staged scripted-baseline → robustness → learning plan.

## Not yet verified

- [ ] Investigate the movement control reported as not working during the first viewer run; deferred at the user's request. Confirm which control and expected motion before fixing.
- [ ] A specific arm asset is selected and licence checked.
- [ ] Python 3.11–3.13 compatibility and operating systems other than macOS tested.
- [ ] Robot and gripper model implemented.
- [ ] Interactive viewer launch tested.
- [ ] Control commands and observations work.
- [ ] Ball grasp/lift/place works.
- [ ] Any training run has been performed.

**Configuration tests:** 12 passed. **Successful pick-and-place episodes:** 0. **Training runs:** 0.

## Next development session — P0 priority order

1. Choose a permissively redistributable arm/gripper model, then inspect its actuation semantics and meshes.
2. Extend the scene to include the robot while preserving the headless physics smoke check.
3. Add controlled arm movement and validate joint/gripper limits.
4. Add automated physics tests that step the model and inspect object state.

## Known risks and blockers

| Risk | Impact | Next action |
|---|---|---|
| Arm asset licensing | Can prevent open-source redistribution | Verify licence and attribution before bundling |
| Sphere grasp contact/friction | May make first pick-and-place difficult | Tune and test fingers/contact independently |
| Control model mismatch | Cartesian commands may not work directly | Select joint, IK or mocap control explicitly |
| Training too early | Can hide scene defects behind policy failures | Require scripted P1 baseline before P3/P4 |
| Platform-specific rendering | Viewer may differ from headless execution | Test both paths and pin versions |

## Progress notes

- **2026-10-23:** User reported that the movement control is not working during visual review. Recorded for later investigation; no controller fix attempted. Scripted arm movement remains unimplemented.
- **2026-10-23:** Launched the interactive Panda scene with desktop access using `PYTHONPATH=src .venv/bin/python -m robot_arm_sim --robot --robot-model /tmp/robot-arm-panda-validation/panda.xml`; process remained running without errors. `mjpython` failed internally, so viewer instructions now use standard Python. Rendered and inspected the `overview` camera after 1,000 steps: all scene elements visible and distinct; ball z=0.53963282 m. Preview saved to ignored `renders/scene-overview.png`. Closed P0-004.10 visual setup verification; user interaction remains untested.

- **2026-10-23:** Implemented P0-004.10 views/lighting (final visual acceptance pending) and completed P0-004.11–12. Added overview/top-down cameras, key/fill lights, explicit implicitfast/Newton/elliptic solver settings and documented rationale. Added 19 framing/settings/real-Panda collision-filter tests, including all hand/finger mesh/pad ball contacts and table/floor probes. `ROBOT_ARM_PANDA_XML=/tmp/robot-arm-panda-validation/panda.xml PYTHONPATH=src .venv/bin/python -m pytest -q`: **199 passed, no warnings**. `PYTHONPATH=src .venv/bin/python -m robot_arm_sim --robot --robot-model /tmp/robot-arm-panda-validation/panda.xml --headless --steps 1000`: completed; ball `(0, 0, 0.540)` m. See `docs/scene_settings.md` for macOS viewer commands and pending visual check. Changes are uncommitted.
- **2026-10-23:** Completed P0-004.08–09: verified 0.08 m default ball clearance, added signed-distance overlap rejection during composition/configuration, and replaced target geometry with a translucent green visual-only site. Added 31 tests covering resting/clear/embedded resets, support/robot overlaps, bounded settling, target structure/configuration and trajectory invariance with the marker removed. Updated a synthetic robot fixture whose home geometry overlapped the default ball. `ROBOT_ARM_PANDA_XML=/tmp/robot-arm-panda-validation/panda.xml PYTHONPATH=src .venv/bin/python -m pytest -q`: **180 passed**, two existing integrator warnings. Offscreen rendering could not run; interactive appearance remains unverified. See `docs/spawn_and_target.md`. Changes are uncommitted.
- **2026-10-23:** Completed P0-004.07: verified and documented existing `ball_free` joint with six unactuated velocity DoFs. Added 20 regression cases for unconstrained structure, ballistic launches, conserved spin and contact-driven momentum changes. `ROBOT_ARM_PANDA_XML=/tmp/robot-arm-panda-validation/panda.xml PYTHONPATH=src .venv/bin/python -m pytest -q`: **149 passed**, two existing integrator warnings. Without local Panda assets: **147 passed, 2 skipped**. Changes are uncommitted.
- **2026-10-23:** Completed P0-004.06: explicit ball material, friction and contact participation; documented MuJoCo-computed uniform-sphere inertia (0.000064 kg m² per axis for 0.04 m radius and 0.1 kg mass). Added 16 tests covering compiled properties, analytical linear/angular acceleration and resolved contact friction in standalone/composed scenes. `ROBOT_ARM_PANDA_XML=/tmp/robot-arm-panda-validation/panda.xml PYTHONPATH=src .venv/bin/python -m pytest -q`: **129 passed**, two existing integrator warnings. Without local Panda assets: **127 passed, 2 skipped**. The 8 cm diameter is retained; grasp clearance/tuning remains unverified. See `docs/ball_physics.md`. Changes are uncommitted.
- **2026-10-23:** Completed P0-004.05: fixed Panda base at `(-0.65, 0, 0.50)` m on a colliding pedestal, with support-penetration validation at construction and after CLI reset overrides. Real Panda revision `cda5236f98383f6fbaa4820c6b18dba892cd087f`, held only in temporary storage, passed 84 bounded sweep samples and 1,000 stepping checks; home table clearance 0.12844 m. `ROBOT_ARM_PANDA_XML=/tmp/robot-arm-panda-validation/panda.xml PYTHONPATH=src .venv/bin/python -m pytest -q`: **113 passed**, two integrator-conflict warnings (scene Euler retained; P0-004.11). See `docs/robot_placement.md` for reproduction, tolerances and limits. Changes are uncommitted.
- **2026-10-23:** Completed P0-004.04: world-fixed ground plane at z=0 with procedural checker material, explicit contact masks and friction; documented finite visual patch versus infinite collision surface. Added 11 floor checks covering setup, contact/clearance boundaries, and ball settling in all directions including outside the visible patch. `PYTHONPATH=src .venv/bin/python -m pytest -q`: **102 passed**. Interactive visual appearance has not been checked. Changes are uncommitted.
- **2026-10-23:** Added 62 table edge cases: 1 mm overlap versus clearance at tabletop sides, underside and all four upper corners; all four side faces of every leg; ball settling near each edge/corner; and drops outside each side reaching the floor without table contact. `PYTHONPATH=src .venv/bin/python -m pytest -q`: **91 passed**. Changes are uncommitted.
- **2026-10-23:** Completed P0-004.03: fixed tabletop (0.90 × 0.70 × 0.05 m, surface z=0.50 m) and four colliding box legs (0.06 × 0.06 × 0.45 m). Visual and collision geometry match; friction uses the scene default `(1, 0.005, 0.0001)`. `PYTHONPATH=src .venv/bin/python -m pytest -q`: **29 passed**, including six table tests covering fixed support, contact with each leg and clearance underneath. Changes are uncommitted; robot sweep/reset clearance remains P0-004.05.
- **2026-10-22:** Initial architecture and milestone documentation created.
- **2026-10-22:** Added editable packaging, `requirements.txt`, a minimal table-and-ball MuJoCo scene, and launch commands. `python -m robot_arm_sim --headless --steps 1000` completed successfully in a clean Python 3.14 virtual environment; the ball settled at z=0.54 m. The demo does not include a robot or grasping behavior.
- **2026-10-22:** Added packaged TOML configuration; relative model paths resolve from the configuration file, and configured joint/actuator names and limits are checked against the loaded MuJoCo model.
- **2026-10-22:** In a clean environment on macOS 26.6.2 arm64 / Python 3.14.6, installed MuJoCo 3.16.0, passed 12 configuration tests and completed the configured 1,000-step headless demo. Other operating systems and Python 3.11–3.13 have not been verified.
- **2026-10-22:** Resolved `uv.lock` across the declared Python range and generated the hash-checked pip export in `requirements.txt`; exact setuptools build backend version is pinned.

## Update policy

When completing a task, record the commit reference, command used, actual result and any relevant metrics. Update this file after validated milestones; do not mark a checkbox complete based on code presence alone.
