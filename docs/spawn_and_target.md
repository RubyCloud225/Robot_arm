# P0-004.08–09 — Ball spawn and visual goal

## Ball spawn

The default ball centre is `(0, 0, 0.62)` m. Its 0.04 m radius puts the bottom
at z=0.58 m, **0.08 m above** the z=0.50 m tabletop. Initial velocity is zero
in a newly constructed scene. The ball falls naturally and settles at a centre
height of approximately 0.54 m.

`validate_ball_spawn()` checks signed collision distances after `mj_forward()`.
It runs during robot composition and after `apply_config()` updates the ball,
robot joint positions and target. Collision-enabled geometry must not overlap
the sphere by more than 1 micrometre (numerical tolerance). Exact resting
contact is allowed. Invalid resets raise `ValueError` with the collider name
and penetration depth before stepping. Direct callers changing reset positions
must forward the model and run the validator again.

The check covers floor, tabletop, legs, pedestal and robot collision geometry;
visual-only geometry and sites are not obstacles. It validates overlap, not
reachability or whether a custom spawn will eventually land on the table.
The packaged default spawn lies over the centre of the tabletop.

Regression tests cover clear and touching placements, a 1 mm tabletop overlap,
embedded placements, all legs, the floor and robot geometry. The default drop
stays finite, remains below 1.5 m/s (the expected impact speed is about
1.25 m/s), and settles without a launch from penetration correction.

## Goal marker

`target_marker` is a translucent green **cylinder site**, radius 0.06 m and
half-height 0.002 m, on a world-fixed `target` body. Its default centre is
`(0.25, 0, 0.502)` m, so the bottom meets the tabletop and the top remains
visible above it. `task.target_position` moves this body and its site.

A site supplies a visual reference without collision geometry, mass, joints,
actuation or physical constraints. It replaces the previous non-colliding
`target_marker` geom; consumers should now use `model.site("target_marker")`
and `data.site("target_marker")`. The target body name and position interface
are unchanged. Its displayed 6 cm radius is a visual guide; it does not change
the design's proposed 5 cm success threshold or implement scoring.

The suite checks site type, shape, colour, translucency, fixed placement and
configuration updates. Three 1,000-step comparisons place the marker through
the ball at spawn, resting height and the tabletop; ball position/velocity
and contact counts match a scene with the entire marker body removed.

Tests run against standalone and synthetic robot-composed scenes; the full
suite also includes real Panda placement tests when assets are supplied.
An attempted offscreen render could not run in the current environment;
interactive appearance remains unverified.
