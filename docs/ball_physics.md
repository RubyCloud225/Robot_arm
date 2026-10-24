# P0-004.06 — Task sphere physics

The task ball is a uniform solid sphere. Its explicit MJCF properties live
on `ball_geom` in `src/robot_arm_sim/assets/demo.xml`:

| Property | Value |
|---|---|
| Radius | 0.04 m (8 cm diameter) |
| Mass | 0.1 kg |
| Principal moments of inertia | 0.000064 kg m² on each axis |
| Centre of mass | Body origin, coincident with sphere centre |
| Visual material | `ball_material`, opaque red |
| Friction (sliding, torsional, rolling) | `(1, 0.005, 0.0001)` |
| Contact dimension | 3: normal and two sliding friction directions |
| Collision masks | `contype=1`, `conaffinity=1` |

MuJoCo infers inertia from the sphere geometry and explicit mass using
`I = (2/5)mr²`; no separate inertial element duplicates the mass definition.
The implied density is approximately 373 kg/m³. These are initial simulation
parameters, not calibrated properties of a particular real ball.

The material controls appearance. Friction and contact masks are specified
separately on the geom, rather than inherited implicitly from scene defaults.
The torsional and rolling coefficients are recorded but are inactive for a
three-dimensional contact unless another participating geom raises the
resolved contact dimension. Grasp-specific tuning remains future work.

The existing radius, mass and spawn height are preserved. An 8 cm diameter
matches the Panda's nominal maximum jaw travel, so comfortable grasp clearance
is not established; later grasp tests may require reducing the radius. This
ticket establishes sphere physics, not successful grasping.

`tests/test_ball.py` verifies the compiled mass, radius, centre of mass and
inertia; checks acceleration against `F/m` and angular acceleration against
`torque/I` along all three axes; and inspects the resolved sliding friction
of actual table contacts. Each case runs both in the standalone scene and
with a synthetic fixed robot attached. Floor and table boundary/drop tests
continue to cover contact with other scene supports.

## P0-004.07 — Free motion

`ball_free` is a free joint on the world-child `ball` body: three translation
and three rotation velocity coordinates, with seven position coordinates
(XYZ plus a unit quaternion). It has no joint limits, damping, friction loss
or armature. The ball is not controlled by mocap or attached to the robot.
Initial pose assignment happens at setup; simulation stepping determines
subsequent motion through gravity, momentum and contact forces.

Twenty additional regression cases run in standalone and composed scenes:
they check the joint's unconstrained structure, ballistic trajectories from
rest and horizontal/upward/downward launches, conserved free spin around
each axis, and left/right tabletop-side impacts that generate contact force
and oppose incoming momentum. Ballistic comparisons allow numerical
integration error. These tests do not require artificial ball attachment or
position updates during stepping.
