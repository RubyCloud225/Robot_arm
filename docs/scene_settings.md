# P0-004.10–12 — Views, physics and contact filters

## Views and lighting (P0-004.10)

The packaged scene defines two world-fixed cameras:

- `overview`: an oblique view from `(1.5, -2, 1.6)` m toward the workspace,
  vertical field of view 45 degrees.
- `top_down`: from `(-0.2, 0, 2.8)` m, looking down -Z, vertical field of view
  50 degrees.

A shadow-casting key light, softer fill light and viewer headlight illuminate
the scene. The free camera starts centred on `(-0.2, 0, 0.65)` m, with azimuth
125 degrees and elevation -30 degrees. Panda's upstream white/black materials,
red ball, translucent green target, brown table and blue-grey checker floor
separate the main scene elements by colour.

`tests/test_scene_settings.py` projects table corners, ball, goal and a robot
reference envelope into both fixed camera frustums at 4:3 aspect ratio. Wider
viewports retain horizontal coverage. This checks geometric framing, not
occlusion or interactive visual legibility. The overview was rendered and inspected on 2026-10-23: the full arm,
table, red ball and green target are visible and distinct. The standard
Python interactive launcher also remained running without errors. Select `overview` or
`top_down` in the viewer's camera control, and enable contact-point/force
visualizations when debugging contacts.

On the validated macOS setup, run the blocking viewer with standard Python
from the repository root (`mjpython` returned an internal viewer exception):

```sh
PYTHONPATH=src .venv/bin/python -m robot_arm_sim --robot --robot-model /tmp/robot-arm-panda-validation/panda.xml
```

That temporary Panda path was used for validation and may disappear. Substitute
a locally available reviewed Panda path if necessary. The robot-free scene is:

```sh
PYTHONPATH=src .venv/bin/python -m robot_arm_sim
```

## Explicit physics (P0-004.11)

| Setting | Value | Rationale |
|---|---|---|
| Timestep | 0.002 s | 500 Hz physics; 20 steps per default 0.04 s control period |
| Integrator | `implicitfast` | Matches the Panda model and handles stiff servo dynamics |
| Solver | `Newton` | Iterative constraint solution for the small articulated scene |
| Friction cone | `elliptic` | Smooth friction-cone model for sliding contacts |
| Iteration cap | 100 | Allows contact/robot constraint convergence within a bounded solve |
| Solver tolerance | 1e-8 | Explicit convergence criterion |
| No-slip iterations | 0 | Keep ordinary soft friction contacts; no extra no-slip correction |
| Scene geom `solref` | `(0.02, 1)` | Compliant, critically damped default; contact time scale spans 10 physics steps |
| Scene geom `solimp` | `(0.9, 0.95, 0.001)` | Explicit baseline impedance parameters |

Settings are in `assets/demo.xml`; configuration can override timestep, with
existing checks enforcing an integer number of physics steps per control
period. Scene geom defaults do not overwrite Panda's explicitly defined
contact classes. Changing the timestep or contact parameters requires rerunning
stability/contact tests; these initial values do not establish grasp success.

The previous Euler/implicitfast composition warning is resolved. The real Panda
suite passes without warnings; a 1,000-step robot CLI run finishes with ball
centre `(0, 0, 0.540)` m. Existing drops, momentum, contact-boundary and base
stability checks all run under these settings.

## Collision participation (P0-004.12)

Scene physical geometry uses `contype=1`, `conaffinity=1`: floor, tabletop,
legs, pedestal and ball can contact the imported robot colliders. Panda hand,
finger meshes and all ten pad boxes retain upstream masks and settings.
Visual meshes have both masks zero; target is a site with no collision geom.
MuJoCo geom groups control display, independently of contact masks. No blanket
arm/table or gripper/ball exclusion is introduced. The upstream link0/link1
exclusion and normal parent-child/fixed-body filtering remain intact.

`tests/test_contact_filters.py` requires real Panda assets through
`ROBOT_ARM_PANDA_XML` (or the existing default model location). It checks:

- Mask compatibility between hand/finger colliders and all scene supports/ball.
- Disabled visual-mesh masks and five active pad boxes per finger.
- Actual ball contact with every hand/finger collider, including every pad.
- Actual table/floor contact with every hand/finger collider, using deliberately
  shifted robot poses solely for collision probes; these poses are never stepped.
- Preservation of the upstream base/first-link exclusion.

Table/floor/ball drop and boundary tests cover other scene contacts. The real
contact probes verify participation, not a stable two-finger grasp. When Panda
assets are unavailable these integration tests skip explicitly.
