# P0-004.05 — Fixed robot placement and reset clearance

The Panda is mounted at world **(-0.65, 0, 0.50) m**, with identity
rotation, facing +X across the table. A world-fixed **0.24 × 0.24 × 0.50 m**
box pedestal supports it. Its top is level with the tabletop; its right face
is 0.08 m from the table's left edge. Both the pedestal and robot base are
rigidly fixed, without joints. Imported models with joints on a root body
are rejected. The robot-free demo does not include the pedestal.

`build_scene()` restores the imported `home` joint positions and actuator
commands, then checks robot collision geometry against the tabletop, all
four legs, floor and pedestal. The CLI repeats this check after applying
configured reset joint positions. Invalid poses fail with the offending
geometry names and penetration depth. Direct callers that modify joint
positions should call `mj_forward()` followed by `validate_robot_placement()`.

Distance queries include fixed-body geometry pairs that MuJoCo may omit
from its contact list. Visual-only geometry is ignored. Penetration tolerance
is 10 micrometres, except for the fixed base against its mounting surface:
0.1 mm accommodates the upstream base mesh extending approximately 0.032 mm
below its nominal mounting plane. Moving links receive no such allowance.

## Validation on 2026-10-23

Tested the actual standard Panda model from MuJoCo Menagerie at revision
[`cda5236f98383f6fbaa4820c6b18dba892cd087f`](https://github.com/google-deepmind/mujoco_menagerie/tree/cda5236f98383f6fbaa4820c6b18dba892cd087f/franka_emika_panda).
The XML, its 67 mesh dependencies and LICENSE were fetched into temporary
storage for local validation; none are redistributed in this repository.
This validation revision does not close the asset provenance or installation
pinning work in P0-003.

Panda XML SHA-256:
`96ad67da03710f17f798c9478fd9e9efdf24a3bf8359f05e456dd9fb158ea273`

- Closest home-pose robot collider to any table collider: **0.12844 m**.
- **84 sampled poses** cleared the supports: 21 samples per joint, varying
  one joint at a time from home: joint1 [-0.5, 0.5], joint2 [-0.4, 0.4],
  joint4 [-1.9, -1.2], joint6 [1.3, 1.9] radians.
- **1,000 physics steps** at home: base position/orientation remained fixed,
  state remained finite, and support penetration checks passed each step.
- Regression cases reject movable roots, fixed geometry overlapping supports,
  and an articulated link moved into the table after construction.

Run all checks, including the two real-model integration tests:

```sh
ROBOT_ARM_PANDA_XML=/path/to/panda.xml PYTHONPATH=src .venv/bin/python -m pytest -q
```

Without a local model, the real-model tests skip explicitly; synthetic
regressions still run. This is a bounded check around home, not a guarantee
for arbitrary arm poses, simultaneous joint movements, self-collision,
reachability or a pick-and-place trajectory. Interactive viewing remains
unverified. P0-004.11 now selects implicitfast explicitly, resolving the
previous Euler/implicitfast composition warning; see `scene_settings.md`.
