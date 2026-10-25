
# P0-006.09 — Headless Simulation Acceptance

**Project:** Robot Arm Pick-and-Place Simulator
**Milestone:** P0 — Simulation Lifecycle API
**Status:** Proposed — validation pending

## 1. Objective

Verify that the Simulator API can execute without
an interactive viewer, rendering window or explicitly
created graphics context.

The headless simulation must retain physical behaviour,
state access, contacts, reset and execution accounting.

## 2. Definition of Headless Operation

Headless execution means that the core simulator
does not require:

- An interactive MuJoCo viewer.
- A visible simulation window.
- An OpenGL rendering context.
- Camera-image rendering.
- A graphical display server.

Headless operation does not disable physics,
contacts or actuator controls.

## 3. Supported Lifecycle

The following operations must function headlessly:

    Simulator(panda_path)
    Simulator.load()
    Simulator.reset()
    Simulator.step()
    Simulator.step_controller_period()
    Simulator.close()

The normal NEW, LOADED and CLOSED lifecycle
restrictions remain in effect.

## 4. Model and Data Ownership

The simulator retains an independently compiled
MjModel and its corresponding MjData.

No renderer is required to construct either object.

The same model/data pair must survive stepping
and resetting.

Closing releases the Simulator's ownership references.

## 5. Physics Execution

Physics stepping uses mujoco.mj_step().

The default physics timestep is 0.002 seconds.

The default controller period is 0.040 seconds,
requiring 20 physics integrations.

These values can differ when another validated
configuration or model timestep is used.

Neither stepping method launches a viewer.

## 6. Derived Physics State

The following quantities must remain accessible
without camera rendering:

- Body positions and orientations.
- Site positions.
- Collision geometry positions.
- Joint positions and velocities.
- Actuator commands.
- Contact information.
- Simulation time.
- Physics step count.
- Controller step count.

These values are calculated by the MuJoCo simulation
and forward-dynamics routines.

They do not depend on image generation.

## 7. Headless Subprocess Verification

The test suite must include a fresh Python
subprocess that:

1. Starts without DISPLAY and WAYLAND_DISPLAY.
2. Imports MuJoCo and the Simulator API.
3. Loads the reviewed Panda model.
4. Resets the simulation.
5. Executes physics steps.
6. Executes a controller period.
7. Verifies time and execution counters.
8. Resets again.
9. Closes cleanly.
10. Exits successfully.

The child process is isolated from the test
runner's previously imported Python modules.

This is not a complete substitute for running
on a genuinely display-less CI environment.

## 8. Rendering Separation

The core Simulator API must not construct:

- mujoco.Renderer
- An interactive MuJoCo viewer.
- A rendering window.
- A graphics context for camera frames.

Future rendering or visualisation components
should remain separate from the physics lifecycle.

Importing MuJoCo itself is permitted.

## 9. Validation

Primary test file:

    tests/test_simulator_headless.py

Validation covers:

- Complete lifecycle without renderer construction.
- Physical state evolution without rendering.
- Derived state access.
- Contact availability.
- Repeated reset.
- Independent simulator instances.
- Existing lifecycle guards.
- Fresh interpreter execution without display variables.

## 10. Known Limitations

The following are outside P0-006.09:

- Offscreen camera rendering.
- EGL or OSMesa backend configuration.
- Viewer performance or interactive controls.
- GPU image capture.
- Real-time operation.
- Robotics learning or behaviour inference.
- Physical hardware deployment.

A passing subprocess test demonstrates the tested
headless API path on the current host.

It does not prove compatibility across every
operating system, graphics driver or deployment
environment.

## 11. Acceptance Criteria

P0-006.09 is complete when:

1. Simulator construction requires no viewer.
2. Loading and resetting work without rendering.
3. Physics stepping works without rendering.
4. Controller-period stepping works without rendering.
5. Time and counters remain correct.
6. Derived body and contact state remains accessible.
7. Independent instances work without a shared viewer.
8. A fresh subprocess succeeds without display variables.
9. No production rendering dependency is introduced.
10. The full regression suite passes.

## 12. Change Control

All source modifications require owner approval
under AGENTS.md.

Test execution requires separate authorisation.

Do not alter:

- scripts/run_tests.py
- Scene or Panda model assets
- The existing simulator stepping interfaces
- Model ownership and lifecycle semantics
- Actuator control logic
